#include "Compliance.hpp"

// ------------------------------------------------------
// Numerical compliance via finite differences
// MATLAB-equivalent implementation
// ------------------------------------------------------
ComplianceResult computeCompliance(
    const RobotArmModel& model,
    const JointState& q,
    const Wrench& /*tip*/,   // <-- ignored on purpose
    double delta)
{
    ComplianceResult out;
    out.Cp.setZero();
    out.Ct.setZero();

    // ==================================================
    // 1. Baseline solve: NO FORCE (Y0 = Statics_Model(q))
    // ==================================================
    Wrench tip0;
    tip0.force.setZero();
    tip0.moment.setZero();

    FKResult fk0 = model.forwardKinematics(q, tip0);
    const StateSample& s0 = fk0.samples.back();

    const Eigen::Vector3d p0 = s0.p;
    const Eigen::Vector3d t0 = s0.R.col(2);  // tangent

    // ==================================================
    // 2. Finite differences in force
    //    Y = Statics_Model(q, Ftip_test, Mtip, Yprev)
    // ==================================================
    for (int j = 0; j < 3; ++j) {

        // Perturbed force (Ftip_test)
        Wrench tip_pert;
        tip_pert.force.setZero();
        tip_pert.moment.setZero();
        tip_pert.force(j) = delta;

        // IMPORTANT: pass fk0 as prev (Yprev)
        FKResult fk =
            model.forwardKinematics(q, tip_pert, &fk0);

        const StateSample& s = fk.samples.back();

        // Finite difference
        out.Cp.col(j) = (s.p - p0) / delta;
        out.Ct.col(j) = (s.R.col(2) - t0) / delta;
    }

    return out;
}
