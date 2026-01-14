#include "StaticsEvaluation.hpp"

StaticsResult evaluateStaticsAndCompliance(
    const RobotArmModel& model,
    const JointState& q,
    const Wrench& tip,
    const FKResult* Yprev_in,
    double delta)
{
    StaticsResult out;

    // --------------------------------------------------
    // 1. Determine Yprev (MATLAB-equivalent)
    // --------------------------------------------------
    FKResult Yprev_local;
    const FKResult* Yprev = Yprev_in;

    if (!Yprev) {
        Wrench zero;
        zero.force.setZero();
        zero.moment.setZero();
        Yprev_local = model.forwardKinematics(q, zero);
        Yprev = &Yprev_local;
    }

    // --------------------------------------------------
    // 2. Baseline at actual force (Y0)
    // --------------------------------------------------
    out.Y0 = model.forwardKinematics(q, tip, Yprev);

    const auto& s0 = out.Y0.samples.back();
    Eigen::Vector3d p0 = s0.p;
    Eigen::Vector3d t0 = s0.R.col(2);

    // --------------------------------------------------
    // 3. Compliance (linearized about Yprev, not Y0)
    // --------------------------------------------------
    out.C.Cp.setZero();
    out.C.Ct.setZero();

    for (int j = 0; j < 3; ++j) {
        Wrench tip_pert = tip;
        tip_pert.force(j) += delta;

        FKResult Yj = model.forwardKinematics(q, tip_pert, Yprev);
        const auto& sj = Yj.samples.back();

        out.C.Cp.col(j) = (sj.p - p0) / delta;
        out.C.Ct.col(j) = (sj.R.col(2) - t0) / delta;
    }

    // --------------------------------------------------
    // 4. Final backbone (MATLAB last line)
    // --------------------------------------------------
    out.Y = model.forwardKinematics(q, tip, Yprev);

    return out;
}
