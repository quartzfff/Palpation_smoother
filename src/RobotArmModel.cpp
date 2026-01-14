#include "RobotArmModel.hpp"
#include "CosseratSolver.hpp"
#include <cmath>


RobotArmModel::RobotArmModel(const Tube& outer,
                             const Tube& inner,
                             const Tube* tool)
    : outer_(outer),
      inner_(inner),
      tool_(tool)
{
    // stiffness scaling 
    alpha_b_ = 1.0;

    // motor torque → rotation coupling
    Kg_ = 0.0;

    // clearance model 
    r_mm_ = 1000.0 / 65.0;
    c_mm_ = 0.3;
}



double RobotArmModel::clearanceAngle(double tubed_length) const
{
    return 2.0 * std::atan(
        (tubed_length - std::sqrt(tubed_length * tubed_length +
                   2.0 * c_mm_ * r_mm_ - c_mm_ * c_mm_))
        / (c_mm_ - 2.0 * r_mm_)
    );
}


FKResult RobotArmModel::forwardKinematics(
    const JointState& q,
    const Wrench& tip,
    const FKResult* prev) const
{
    FKResult result;

    // --------------------------------------------------
    // 1. Base internal force & moment
    // --------------------------------------------------
    Eigen::Vector3d n0 = tip.force;
    Eigen::Vector3d m0 = tip.moment;

    if (prev && !prev->samples.empty()) {
        const StateSample& last = prev->samples.back();
        m0 += last.p.cross(tip.force);
    }

    // --------------------------------------------------
    // 2. Clearance tilt
    // --------------------------------------------------
    double L_inside_mm = std::max(0.0, 5.0 - std::max(q.d1, 0.0));
    double theta_c = clearanceAngle(L_inside_mm);

    Eigen::Matrix3d R0 =
        Rz(q.theta1 + Kg_ * m0.z()) * Rx(theta_c);

    // --------------------------------------------------
    // 3. Initial Cosserat state
    // --------------------------------------------------
    StateSample state;
    state.p = Eigen::Vector3d::Zero();
    state.R = R0;
    state.n = n0;
    state.m = m0;
    state.theta = q.theta1;

    // --------------------------------------------------
    // 4. Build segments (pure geometry / mechanics)
    // --------------------------------------------------
    std::vector<Segment> segments = buildSegments(q);

    // --------------------------------------------------
    // 5. Integrate each segment
    // --------------------------------------------------

    CosseratSolver solver(alpha_b_);
    const int N = 10;

    bool inner_started = false;

    for (const Segment& seg : segments) {

        // Integrate current segment
        state = solver.integrateSegment(
            seg,
            state,
            N,
            result.samples
        );

        // --------------------------------------------------
        // Theta switch: outer tube → inner tube (MATLAB)
        // --------------------------------------------------
        if (!inner_started && seg.s1 >= q.d1 && q.d2 > q.d1) {

            // MATLAB:
            // Rnext = R_end * Rz(theta2 - theta1end)
            state.R = state.R * Rz(q.theta2 - state.theta);

            // Reset theta to inner tube rotation
            state.theta = q.theta2;

            inner_started = true;
        }
    }



    return result;
}


Eigen::Matrix3d RobotArmModel::Rz(double theta) const
{
    double c = std::cos(theta);
    double s = std::sin(theta);
    Eigen::Matrix3d R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

Eigen::Matrix3d RobotArmModel::Rx(double theta) const
{
    double c = std::cos(theta);
    double s = std::sin(theta);
    Eigen::Matrix3d R;
    R << 1,  0,  0,
         0,  c, -s,
         0,  s,  c;
    return R;
}


std::vector<Segment>
RobotArmModel::buildSegments(const JointState& q) const
{
    std::vector<Segment> segments;
    const double straight_len = 5.0;

    // --- Tube properties ---
    const double EI_outer = outer_.EI();
    const double EI_inner = inner_.EI();
    const double EI_tool  = tool_ ? tool_->EI() : 0.0;

    const double GJ_outer = outer_.GJ();
    const double GJ_inner = inner_.GJ();
    const double GJ_tool  = tool_ ? tool_->GJ() : 0.0;

    const bool has_tool = (tool_ != nullptr);

    // =====================================================
    // Outer tube exposed (base → d1)
    // MATLAB: EI1+EI2(+EI3), torsion = GJ1
    // =====================================================
    const double EI_exposed =
        EI_outer + EI_inner + (has_tool ? EI_tool : 0.0);
    const double GJ_exposed = GJ_outer;

    if (q.d1 > straight_len) {
        // Curved portion
        segments.push_back({
            0.0,
            q.d1 - straight_len,
            outer_.kappa(),
            EI_exposed,
            GJ_exposed
        });

        // Straight tip
        segments.push_back({
            q.d1 - straight_len,
            q.d1,
            Eigen::Vector3d::Zero(),
            EI_exposed,
            GJ_exposed
        });
    } else {
        // Entire exposed part straight
        segments.push_back({
            0.0,
            q.d1,
            Eigen::Vector3d::Zero(),
            EI_exposed,
            GJ_exposed
        });
    }

    // =====================================================
    // Overlap region (d1 → d2)
    // MATLAB: EI2(+EI3), torsion = GJ2(+GJ3)
    // =====================================================
    if (q.d2 > q.d1) {
        const double EI_overlap =
            EI_inner + (has_tool ? EI_tool : 0.0);
        const double GJ_overlap =
            GJ_inner + (has_tool ? GJ_tool : 0.0);

        segments.push_back({
            q.d1,
            q.d2,
            Eigen::Vector3d::Zero(),
            EI_overlap,
            GJ_overlap
        });
    }

    // =====================================================
    // Tool segment (d2 → d3)
    // =====================================================
    if (has_tool) {
        const double d3 = q.d2 + tool_->length();
        segments.push_back({
            q.d2,
            d3,
            Eigen::Vector3d::Zero(),
            EI_tool,
            GJ_tool
        });
    }

    return segments;
}
