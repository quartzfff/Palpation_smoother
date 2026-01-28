#pragma once

#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include "RobotArmModel.hpp"

// ============================================================
// Stage 1 — INNER curvature
// ============================================================
class TipPoseTangentCalibFactorInner
  : public gtsam::NoiseModelFactor1<double>
{
public:
  using Base = gtsam::NoiseModelFactor1<double>;

  TipPoseTangentCalibFactorInner(
      gtsam::Key kappa_key,
      const RobotArmModel& robot_template,
      const JointState& q,
      const Eigen::Vector3d& p_meas,
      const Eigen::Vector3d& t_meas,
      const gtsam::SharedNoiseModel& noise);

  gtsam::Vector evaluateError(
      const double& kappa,
      gtsam::OptionalMatrixType H) const override;

private:
  RobotArmModel robot_template_;
  JointState q_;
  Eigen::Vector3d p_meas_;
  Eigen::Vector3d t_meas_;
};

// ============================================================
// Stage 2 — OUTER curvature + theta offset
// ============================================================
class TipPoseTangentCalibFactorOuter
  : public gtsam::NoiseModelFactor2<double, double>
{
public:
  using Base = gtsam::NoiseModelFactor2<double, double>;

  TipPoseTangentCalibFactorOuter(
      gtsam::Key kappa_key,
      gtsam::Key theta_key,
      const RobotArmModel& robot_template,
      const JointState& q,
      const Eigen::Vector3d& p_meas,
      const Eigen::Vector3d& t_meas,
      const gtsam::SharedNoiseModel& noise);

  gtsam::Vector evaluateError(
      const double& kappa,
      const double& theta,
      gtsam::OptionalMatrixType H1,
      gtsam::OptionalMatrixType H2) const override;

private:
  RobotArmModel robot_template_;
  JointState q_;
  Eigen::Vector3d p_meas_;
  Eigen::Vector3d t_meas_;
};
