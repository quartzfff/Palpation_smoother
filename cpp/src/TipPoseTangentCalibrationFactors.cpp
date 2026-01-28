#include "factors/TipPoseTangentCalibrationFactors.hpp"

// ============================================================
// Stage 1 — INNER curvature κ_in_eq
// ============================================================
TipPoseTangentCalibFactorInner::TipPoseTangentCalibFactorInner(
    gtsam::Key kappa_key,
    const RobotArmModel& robot_template,
    const JointState& q,
    const Eigen::Vector3d& p_meas,
    const Eigen::Vector3d& t_meas,
    const gtsam::SharedNoiseModel& noise)
  : Base(noise, kappa_key),
    robot_template_(robot_template),
    q_(q),
    p_meas_(p_meas),
    t_meas_(t_meas.normalized())
{}

gtsam::Vector
TipPoseTangentCalibFactorInner::evaluateError(
    const double& kappa_in_eq,
    gtsam::OptionalMatrixType H) const
{
  RobotArmModel robot = robot_template_;
  robot.setInnerKappaX(kappa_in_eq);

  Wrench zero;
  zero.force.setZero();
  zero.moment.setZero();

  FKResult Y = robot.forwardKinematics(q_, zero);
  const StateSample& s = Y.samples.back();

  Eigen::Vector3d p_fk = s.p;
  Eigen::Vector3d t_fk = s.R.col(2).normalized();

  Eigen::Vector3d r_p = p_meas_ - p_fk;
  Eigen::Vector3d r_t = t_meas_.cross(t_fk);

  gtsam::Vector6 r;
  r << r_p, r_t;

  if (H) {
    const double eps = 1e-6;

    RobotArmModel r_eps = robot_template_;
    r_eps.setInnerKappaX(kappa_in_eq + eps);

    FKResult Ye = r_eps.forwardKinematics(q_, zero);
    const StateSample& se = Ye.samples.back();

    Eigen::Vector3d dp = (se.p - p_fk) / eps;
    Eigen::Vector3d dt =
        (t_meas_.cross(se.R.col(2).normalized()) - r_t) / eps;

    gtsam::Matrix J(6,1);
    J << dp, dt;
    *H = J;
  }

  return r;
}

// ============================================================
// Stage 2 — OUTER curvature κ_out_eq + θ_offset
// ============================================================
TipPoseTangentCalibFactorOuter::TipPoseTangentCalibFactorOuter(
    gtsam::Key kappa_key,
    gtsam::Key theta_key,
    const RobotArmModel& robot_template,
    const JointState& q,
    const Eigen::Vector3d& p_meas,
    const Eigen::Vector3d& t_meas,
    const gtsam::SharedNoiseModel& noise)
  : Base(noise, kappa_key, theta_key),
    robot_template_(robot_template),
    q_(q),
    p_meas_(p_meas),
    t_meas_(t_meas.normalized())
{}

gtsam::Vector
TipPoseTangentCalibFactorOuter::evaluateError(
    const double& kappa_out_eq,
    const double& theta_offset,
    gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2) const
{
  RobotArmModel robot = robot_template_;
  robot.setOuterKappaEq(kappa_out_eq);
  robot.setOuterThetaOffset(theta_offset);

  Wrench zero;
  zero.force.setZero();
  zero.moment.setZero();

  FKResult Y = robot.forwardKinematics(q_, zero);
  const StateSample& s = Y.samples.back();

  Eigen::Vector3d p_fk = s.p;
  Eigen::Vector3d t_fk = s.R.col(2).normalized();

  Eigen::Vector3d r_p = p_meas_ - p_fk;
  Eigen::Vector3d r_t = t_meas_.cross(t_fk);

  gtsam::Vector6 r;
  r << r_p, r_t;

  const double eps = 1e-6;

  if (H1) {
    RobotArmModel r1 = robot_template_;
    r1.setOuterKappaEq(kappa_out_eq + eps);
    r1.setOuterThetaOffset(theta_offset);

    FKResult Y1 = r1.forwardKinematics(q_, zero);
    Eigen::Vector3d dp =
        (Y1.samples.back().p - p_fk) / eps;
    Eigen::Vector3d dt =
        (t_meas_.cross(Y1.samples.back().R.col(2).normalized()) - r_t) / eps;

    gtsam::Matrix J(6,1);
    J << dp, dt;
    *H1 = J;
  }

  if (H2) {
    RobotArmModel r2 = robot_template_;
    r2.setOuterKappaEq(kappa_out_eq);
    r2.setOuterThetaOffset(theta_offset + eps);

    FKResult Y2 = r2.forwardKinematics(q_, zero);
    Eigen::Vector3d dp =
        (Y2.samples.back().p - p_fk) / eps;
    Eigen::Vector3d dt =
        (t_meas_.cross(Y2.samples.back().R.col(2).normalized()) - r_t) / eps;

    gtsam::Matrix J(6,1);
    J << dp, dt;
    *H2 = J;
  }

  return r;
}
