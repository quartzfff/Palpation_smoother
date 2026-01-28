#include "factors/TipPoseTangentCalibFactor.hpp"

gtsam::Vector
TipPoseTangentCalibFactor::evaluateError(
    const double& kappa_in,
    gtsam::OptionalMatrixType H) const
{
    // ----------------------------------------
    // Build robot with calibrated inner kappa
    // ----------------------------------------
    RobotArmModel robot = robot_template_;
    robot.setInnerKappa(kappa_in);

    // ----------------------------------------
    // FK (no force)
    // ----------------------------------------
    Wrench zero;
    zero.force.setZero();
    zero.moment.setZero();

    FKResult Y = robot.forwardKinematics(q_, zero);

    const StateSample& s = Y.samples.back();
    Eigen::Vector3d p_fk = s.p;
    Eigen::Vector3d t_fk = s.R.col(2).normalized();

    // ----------------------------------------
    // Residual
    // ----------------------------------------
    Eigen::Vector3d r_p = p_meas_ - p_fk;
    Eigen::Vector3d r_t = t_meas_.cross(t_fk);

    gtsam::Vector6 r;
    r << r_p, r_t;

    // ----------------------------------------
    // Jacobian (finite-difference, scalar)
    // ----------------------------------------
    if (H) {
        const double eps = 1e-6;

        RobotArmModel robot_eps = robot_template_;
        robot_eps.setInnerKappa(kappa_in + eps);

        FKResult Y_eps = robot_eps.forwardKinematics(q_, zero);
        const StateSample& s_eps = Y_eps.samples.back();

        Eigen::Vector3d dp =
            (s_eps.p - p_fk) / eps;

        Eigen::Vector3d dt =
            (t_meas_.cross(s_eps.R.col(2).normalized()) - r_t) / eps;

        gtsam::Matrix J(6,1);
        J << dp, dt;
        *H = J;
    }

    return r;
}
