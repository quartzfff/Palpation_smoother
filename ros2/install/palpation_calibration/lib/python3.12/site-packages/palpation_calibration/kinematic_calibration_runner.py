import rclpy
from rclpy.node import Node
import numpy as np
import csv
import time

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


# --------------------------------------------------
# Utilities
# --------------------------------------------------

def quat_to_vec(q):
    return np.array([q.x, q.y, q.z, q.w])


def quat_angle(q1, q2):
    dot = abs(np.dot(q1, q2))
    dot = np.clip(dot, -1.0, 1.0)
    return 2.0 * np.arccos(dot) * 180.0 / np.pi


# --------------------------------------------------
# Calibration Runner
# --------------------------------------------------

class CalibrationRunner(Node):

    DWELL_TIME = 1.0        # seconds
    MIN_SAMPLES = 5         # minimum samples per pose

    def __init__(self):
        super().__init__('kinematic_calibration_runner')

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
        self.jp_pub = self.create_publisher(
            JointState,
            '/ves/right/joint/servo_jp',
            10
        )

        self.jp_sub = self.create_subscription(
            JointState,
            '/ves/right/joint/setpoint_jp',
            self.jp_callback,
            10
        )

        self.ndi_sub = self.create_subscription(
            PoseStamped,
            '/sensor_pose_raw',
            self.ndi_callback,
            10
        )

        # -----------------------------
        # State
        # -----------------------------
        self.current_joints = None
        self.current_ndi_pos = None
        self.current_ndi_quat = None

        # Per-command tracking
        self.cmd_sent = False
        self.cmd_pose_ref = None

        # Dwell tracking
        self.in_dwell = False
        self.dwell_start_time = None
        self.samples_this_pose = 0

        # -----------------------------
        # Calibration stages
        # -----------------------------
        self.stage = 1
        self.cmd_idx = 0

        self.stage1_cmds = self.build_stage1_commands()
        self.stage2_cmds = self.build_stage2_commands()
        self.home_cmd = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.get_logger().info(
            f"Stage 1 poses: {len(self.stage1_cmds)}, "
            f"Stage 2 poses: {len(self.stage2_cmds)}"
        )

        # -----------------------------
        # CSV files
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
        # Timer
        # -----------------------------
        self.timer = self.create_timer(0.02, self.update)

    # --------------------------------------------------

    def build_stage1_commands(self):
        return [[0.0, 0.0, d, 0.0, 0.0] for d in np.linspace(5.0, 40.0, 10)]

    def build_stage2_commands(self):
        cmds = []
        for theta_out in np.linspace(-0.6, 0.6, 9):
            for d_out in np.linspace(5.0, 20.0, 6):
                cmds.append([0.0, theta_out, 20.0, d_out, 0.0])
        return cmds

    # --------------------------------------------------

    def jp_callback(self, msg):
        if len(msg.position) == 5:
            self.current_joints = list(msg.position)

    def ndi_callback(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        self.current_ndi_pos = np.array([p.x, p.y, p.z])
        self.current_ndi_quat = quat_to_vec(q)

    # --------------------------------------------------

    def publish_command(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(q)
        self.jp_pub.publish(msg)

    # --------------------------------------------------

    def motion_stopped(self):
        dp = np.linalg.norm(self.current_ndi_pos - self.cmd_pose_ref[0])
        dtheta = quat_angle(self.current_ndi_quat, self.cmd_pose_ref[1])
        return dp < 0.5 and dtheta < 1.0

    # --------------------------------------------------

    def log_sample(self):
        q = self.current_joints
        p = self.current_ndi_pos
        quat = self.current_ndi_quat

        row = [
            q[0], q[1], q[2], q[3],
            p[0], p[1], p[2],
            quat[0], quat[1], quat[2], quat[3]
        ]

        if self.stage == 1:
            self.writer1.writerow(row)
        elif self.stage == 2:
            self.writer2.writerow(row)

        self.samples_this_pose += 1

    # --------------------------------------------------

    def advance_stage(self):
        self.cmd_idx = 0
        self.cmd_sent = False
        self.in_dwell = False
        self.samples_this_pose = 0

        if self.stage == 1:
            self.get_logger().info("Stage 1 complete → returning home")
            self.stage = 1.5
        elif self.stage == 1.5:
            self.get_logger().info("Home reached → starting Stage 2")
            self.stage = 2
        else:
            self.get_logger().info("Calibration complete. Shutting down.")
            self.csv1.close()
            self.csv2.close()
            rclpy.shutdown()

    # --------------------------------------------------

    def update(self):
        if self.current_ndi_pos is None or self.current_joints is None:
            return

        # ---------- Stage selection ----------
        if self.stage == 1:
            cmds = self.stage1_cmds
        elif self.stage == 1.5:
            cmds = [self.home_cmd]
        elif self.stage == 2:
            cmds = self.stage2_cmds
        else:
            return

        if self.cmd_idx >= len(cmds):
            self.advance_stage()
            return

        # ---------- Send command ----------
        if not self.cmd_sent:
            self.publish_command(cmds[self.cmd_idx])
            self.cmd_pose_ref = (
                self.current_ndi_pos.copy(),
                self.current_ndi_quat.copy()
            )
            self.cmd_sent = True
            self.in_dwell = False
            self.samples_this_pose = 0
            return

        # ---------- Wait for stop ----------
        if not self.in_dwell:
            if self.motion_stopped():
                self.in_dwell = True
                self.dwell_start_time = time.time()
            return

        # ---------- Dwell & log ----------
        self.log_sample()

        if (time.time() - self.dwell_start_time >= self.DWELL_TIME and
                self.samples_this_pose >= self.MIN_SAMPLES):

            self.get_logger().info(
                f"Stage {self.stage}, pose {self.cmd_idx + 1}: "
                f"{self.samples_this_pose} samples logged"
            )

            self.cmd_idx += 1
            self.cmd_sent = False
            self.in_dwell = False
            self.samples_this_pose = 0


# --------------------------------------------------
# Main
# --------------------------------------------------

def main():
    rclpy.init()
    node = CalibrationRunner()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
