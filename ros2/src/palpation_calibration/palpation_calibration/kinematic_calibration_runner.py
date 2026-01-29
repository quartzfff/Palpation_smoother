import rclpy
from rclpy.node import Node
import numpy as np
import csv

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


# --------------------------------------------------
# Utilities
# --------------------------------------------------

def smoothstep(n):
    t = np.linspace(0.0, 1.0, n)
    return t**2 * (3.0 - 2.0 * t)

def interpolate_jp(start, end, dt=2.0, rate=50):
    n = max(2, int(dt * rate))
    s = smoothstep(n)
    q0 = np.array(start, dtype=float)
    q1 = np.array(end, dtype=float)
    return [((1.0 - si) * q0 + si * q1).tolist() for si in s]

def quat_to_vec(q):
    return np.array([q.x, q.y, q.z, q.w], dtype=float)


# --------------------------------------------------
# Calibration Runner
# --------------------------------------------------

class JPCalibrationRunner(Node):
    """
    CP-style publisher:
      - Build a finite joint-space trajectory
      - Publish one waypoint each tick (50 Hz)
      - After trajectory ends: hold & log for pause_time while still publishing hold command
      - Then advance

    IMPORTANT:
      - Translations commanded in meters (robot interface)
      - CSV logged in mm (for your C++ calibration code)
    """

    MM_TO_M = 0.001

    def __init__(self):
        super().__init__('jp_calibration_runner')

        # -----------------------------
        # Timing (match CP logic)
        # -----------------------------
        self.publish_rate = 50.0
        self.transition_time = 2.0   # seconds moving between poses
        self.pause_time = 1.0        # seconds holding & logging at each pose

        # -----------------------------
        # Joint names
        # -----------------------------
        self.joint_names = [
            'inner_rotation',
            'outer_rotation',
            'inner_translation',
            'outer_translation',
            'tool'
        ]

        # -----------------------------
        # Publishers / Subscribers
        # -----------------------------
        self.jp_pub = self.create_publisher(JointState, '/ves/right/joint/servo_jp', 10)

        self.jp_sub = self.create_subscription(
            JointState, '/ves/right/joint/setpoint_jp', self.jp_callback, 10
        )

        self.ndi_sub = self.create_subscription(
            PoseStamped, '/sensor_pose_raw', self.ndi_callback, 10
        )

        # -----------------------------
        # Latest measurements
        # -----------------------------
        self.current_joints = None          # as reported by setpoint_jp (robot units)
        self.current_ndi_pos = None
        self.current_ndi_quat = None

        # -----------------------------
        # Stage definition (commands stored in mm for translation)
        # -----------------------------
        self.stage = 1
        self.command_index = -1

        self.stage1_cmds_mm = self.build_stage1_commands_mm()  # [rad, rad, mm, mm, ???]
        self.stage2_cmds_mm = self.build_stage2_commands_mm()
        self.home_cmd_mm = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.cmds_mm = self.stage1_cmds_mm

        self.get_logger().info(
            f"Stage 1 poses: {len(self.stage1_cmds_mm)}, "
            f"Stage 2 poses: {len(self.stage2_cmds_mm)}"
        )

        # -----------------------------
        # Trajectory / hold state
        # -----------------------------
        self.current_trajectory = []  # list of joint vectors in ROBOT UNITS
        self.hold_until = None
        self.hold_cmd = None          # joint vector in ROBOT UNITS (what we keep publishing during pause)

        self.ready = False            # wait until we have joints + ndi before starting

        # -----------------------------
        # CSV logging (exact schema your C++ code expects)
        # -----------------------------
        self.csv1 = open('calib_stage1_inner.csv', 'w', newline='')
        self.csv2 = open('calib_stage2_outer.csv', 'w', newline='')

        self.writer1 = csv.writer(self.csv1)
        self.writer2 = csv.writer(self.csv2)

        header = [
            'theta_in', 'theta_out', 'd_in_mm', 'd_out_mm',
            'px_mm', 'py_mm', 'pz_mm',
            'qx', 'qy', 'qz', 'qw'
        ]
        self.writer1.writerow(header)
        self.writer2.writerow(header)

        # -----------------------------
        # Single publish loop timer (like CP code)
        # -----------------------------
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.tick)

    # --------------------------------------------------
    # Build commands (translations in mm here)
    # --------------------------------------------------

    def build_stage1_commands_mm(self):
        # inner_translation sweep 5..40 mm, everything else fixed
        return [[0.0, 0.0, float(d), 0.0, 0.0] for d in np.linspace(5.0, 40.0, 10)]

    def build_stage2_commands_mm(self):
        cmds = []
        for theta_out in np.linspace(-0.6, 0.6, 9):
            for d_out in np.linspace(5.0, 20.0, 6):
                cmds.append([0.0, float(theta_out), 20.0, float(d_out), 0.0])
        return cmds

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def jp_callback(self, msg):
        if len(msg.position) == 5:
            self.current_joints = list(msg.position)

    def ndi_callback(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        self.current_ndi_pos = np.array([p.x, p.y, p.z], dtype=float)
        self.current_ndi_quat = quat_to_vec(q)

    # --------------------------------------------------
    # Command conversion helpers
    # --------------------------------------------------

    def cmd_mm_to_robot_units(self, cmd_mm):
        """
        Convert [theta_in(rad), theta_out(rad), d_in(mm), d_out(mm), tool(?)] -> robot units.
        Assumption: translations are meters.
        """
        theta_in, theta_out, d_in_mm, d_out_mm, tool = cmd_mm
        return [
            float(theta_in),
            float(theta_out),
            float(d_in_mm) * self.MM_TO_M,
            float(d_out_mm) * self.MM_TO_M,
            float(tool)
        ]

    # --------------------------------------------------
    # Publishing / Logging
    # --------------------------------------------------

    def publish_command(self, q_robot_units):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(q_robot_units)
        self.jp_pub.publish(msg)

    def log_sample(self, cmd_mm):
        """
        Log ONE sample row (called repeatedly during hold).
        CSV must remain in mm for C++ calibration code.
        """
        if self.current_ndi_pos is None or self.current_ndi_quat is None:
            return

        row = [
            cmd_mm[0], cmd_mm[1], cmd_mm[2], cmd_mm[3],
            self.current_ndi_pos[0], self.current_ndi_pos[1], self.current_ndi_pos[2],
            self.current_ndi_quat[0], self.current_ndi_quat[1], self.current_ndi_quat[2], self.current_ndi_quat[3],
        ]

        if self.stage == 1:
            self.writer1.writerow(row)
        elif self.stage == 2:
            self.writer2.writerow(row)

    # --------------------------------------------------
    # Stage / command sequencing (CP style)
    # --------------------------------------------------

    def start_if_ready(self):
        if self.ready:
            return
        if self.current_joints is None or self.current_ndi_pos is None:
            return

        # Initialize "last commanded" from the robot's current setpoint (robot units)
        # So we don't jump on first interpolation.
        self.hold_cmd = list(self.current_joints)

        self.ready = True
        self.get_logger().info("✅ Got initial joints + NDI. Starting Stage 1.")
        self.send_next_command()

    def send_next_command(self):
        self.command_index += 1

        if self.command_index >= len(self.cmds_mm):
            self.advance_stage()
            return

        cmd_mm = self.cmds_mm[self.command_index]
        next_cmd_robot = self.cmd_mm_to_robot_units(cmd_mm)

        self.get_logger().info(
            f"➡️ Stage {self.stage} command {self.command_index + 1}/{len(self.cmds_mm)}: {cmd_mm}"
        )

        # Build trajectory from current hold_cmd (robot units) to next_cmd_robot
        self.current_trajectory = interpolate_jp(
            self.hold_cmd,
            next_cmd_robot,
            dt=self.transition_time,
            rate=int(self.publish_rate)
        )

        # After trajectory, we will hold at next_cmd_robot
        self.hold_cmd = list(next_cmd_robot)

        # Start holding AFTER trajectory finishes
        self.hold_until = None  # will be set when trajectory empties

    def advance_stage(self):
        self.command_index = -1
        self.current_trajectory = []
        self.hold_until = None

        if self.stage == 1:
            self.get_logger().info("🏠 Stage 1 complete → going home")
            self.stage = 1.5
            self.cmds_mm = [self.home_cmd_mm]

        elif self.stage == 1.5:
            self.get_logger().info("▶️ Home reached → starting Stage 2")
            self.stage = 2
            self.cmds_mm = self.stage2_cmds_mm

        else:
            self.get_logger().info("✅ Calibration complete. Shutting down.")
            try:
                self.csv1.flush(); self.csv2.flush()
                self.csv1.close(); self.csv2.close()
            except Exception:
                pass
            rclpy.shutdown()
            return

        self.send_next_command()

    # --------------------------------------------------
    # Main 50 Hz loop (single timer like CP code)
    # --------------------------------------------------

    def tick(self):
        # Wait for initial data before doing anything
        self.start_if_ready()
        if not self.ready:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # 1) If trajectory still active: publish next waypoint
        if self.current_trajectory:
            q = self.current_trajectory.pop(0)
            self.publish_command(q)
            return

        # 2) Trajectory finished: begin hold window if not started
        if self.hold_until is None:
            self.get_logger().info(f"⏸️ Holding {self.pause_time}s and logging")
            self.hold_until = now + self.pause_time

        # 3) During hold: keep publishing hold command AND log
        self.publish_command(self.hold_cmd)

        # Log using the current command in mm (for C++ calibration)
        if self.stage in (1, 2):
            cmd_mm = self.cmds_mm[self.command_index]
            self.log_sample(cmd_mm)

        # 4) End of hold -> next command
        if now >= self.hold_until:
            self.hold_until = None
            self.send_next_command()


# --------------------------------------------------
# Main
# --------------------------------------------------

def main():
    rclpy.init()
    node = JPCalibrationRunner()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
