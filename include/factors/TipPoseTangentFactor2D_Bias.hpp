#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>

#include <Eigen/Dense>

#include "RobotArmModel.hpp"
#include "StaticsEvaluation.hpp"

// Factor on (f2_k, bias) where
//   f2_k  ∈ R^2   : force coordinates in top-2 SVD subspace
//   bias ∈ R^6   : [bp; bt] additive measurement bias in residual space
//
// Residual:
//   r = [p_meas - p_pred(F)] ; [t_meas - t_pred(F)]  - bias
//
// with F = V2 * f2_k, where V2 is top-2 right singular vectors of J = [Cp; Ct].
class TipPoseTangentFactor2D_Bias
  : public gtsam::NoiseModelFactor2<gtsam::Vector2, gtsam::Vector6>
{
public:
  TipPoseTangentFactor2D_Bias(
      gtsam::Key f2_key,
      gtsam::Key bias_key,
      const RobotArmModel& robot,
      const JointState& q,
      const Eigen::Vector3d& p_meas,
      const Eigen::Vector3d& t_meas,
      double delta_fd,
      const gtsam::SharedNoiseModel& noise)
    : gtsam::NoiseModelFactor2<gtsam::Vector2, gtsam::Vector6>(noise, f2_key, bias_key),
      robot_(robot),
      q_(q),
      p_meas_(p_meas),
      t_meas_(t_meas),
      delta_fd_(delta_fd)
  {}

  gtsam::Vector evaluateError(
      const gtsam::Vector2& f2,
      const gtsam::Vector6& bias,
      gtsam::OptionalMatrixType H_f2,
      gtsam::OptionalMatrixType H_bias) const override
  {
    // ------------------------------------------------------
    // 1) Warm-start statics at zero wrench (MATLAB style)
    // ------------------------------------------------------
    Wrench zero;
    zero.force.setZero();
    zero.moment.setZero();

    FKResult Yprev = robot_.forwardKinematics(q_, zero);

    // Evaluate statics+compliance at zero to build J and V2 basis
    StaticsResult ref =
        evaluateStaticsAndCompliance(robot_, q_, zero, &Yprev, delta_fd_);

    // J = [Cp; Ct] ∈ R^{6x3}
    Eigen::Matrix<double,6,3> J;
    J.topRows<3>()    = ref.C.Cp;
    J.bottomRows<3>() = ref.C.Ct;

    // SVD to get V2 (top-2 force directions)
    Eigen::JacobiSVD<Eigen::Matrix<double,6,3>> svd(
        J, Eigen::ComputeFullV);

    Eigen::Matrix<double,3,2> V2 = svd.matrixV().leftCols<2>();

    // ------------------------------------------------------
    // 2) Build physical force F = V2 * f2 (3D local frame)
    // ------------------------------------------------------
    Eigen::Vector3d F = V2 * Eigen::Vector2d(f2(0), f2(1));

    Wrench tip;
    tip.force  = F;
    tip.moment.setZero();

    // ------------------------------------------------------
    // 3) Evaluate statics+compliance at this tip wrench
    // ------------------------------------------------------
    // Reuse Yprev warm-start (good for realtime too)
    StaticsResult res =
        evaluateStaticsAndCompliance(robot_, q_, tip, &Yprev, delta_fd_);

    const StateSample& s = res.Y.samples.back();
    Eigen::Vector3d p_pred = s.p;
    Eigen::Vector3d t_pred = s.R.col(2);

    // ------------------------------------------------------
    // 4) Residual in 6D, subtract bias
    // ------------------------------------------------------
    Eigen::Matrix<double,6,1> r0;
    r0.head<3>() = p_meas_ - p_pred;
    r0.tail<3>() = t_meas_ - t_pred;

    Eigen::Matrix<double,6,1> r = r0 - Eigen::Matrix<double,6,1>(bias);

    // ------------------------------------------------------
    // 5) Jacobians
    //    dr/df2 = - [Cp; Ct] * dF/df2,   where dF/df2 = V2
    //    dr/dbias = -I6
    // ------------------------------------------------------
    if (H_f2) {
      Eigen::Matrix<double,6,3> J_F;
      J_F.topRows<3>()    = res.C.Cp;
      J_F.bottomRows<3>() = res.C.Ct;

      Eigen::Matrix<double,6,2> H = -J_F * V2;
      *H_f2 = H;
    }

    if (H_bias) {
      *H_bias = -Eigen::Matrix<double,6,6>::Identity();
    }

    return r;
  }

private:
  const RobotArmModel& robot_;
  JointState q_;
  Eigen::Vector3d p_meas_;
  Eigen::Vector3d t_meas_;
  double delta_fd_;
};
