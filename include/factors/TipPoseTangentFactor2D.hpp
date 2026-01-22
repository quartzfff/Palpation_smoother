#pragma once

#include <Eigen/Dense>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/Symbol.h>

#include "RobotArmModel.hpp"
#include "StaticsEvaluation.hpp"

/**
 * TipPoseTangentFactor2D
 *
 * MATLAB-equivalent factor:
 *
 *   [U,S,V] = svd([Cp; Ct])
 *   E       = [1 0 0 0 0 0;
 *              0 1 0 0 0 0]
 *   Apt     = E * S * V'
 *   F       = pinv(Apt) * E * U' * r
 *
 * Implemented as a factor graph by optimizing:
 *
 *   f ∈ R^2
 *   F = V(:,1:2) * f
 *
 * Residual:
 *   r = [p_meas - p_pred;
 *        t_meas - t_pred]
 *
 * Jacobian:
 *   dr/df = -[Cp; Ct] * V(:,1:2)
 */
class TipPoseTangentFactor2D
  : public gtsam::NoiseModelFactor1<gtsam::Vector2>
{
public:
  TipPoseTangentFactor2D(
      gtsam::Key f2_key,
      const RobotArmModel& robot,
      const JointState& q,
      const Eigen::Vector3d& p_meas,
      const Eigen::Vector3d& t_meas,
      double delta_fd,
      const gtsam::SharedNoiseModel& noise)
    : gtsam::NoiseModelFactor1<gtsam::Vector2>(noise, f2_key),
      robot_(robot),
      q_(q),
      p_meas_(p_meas),
      t_meas_(t_meas),
      delta_fd_(delta_fd)
  {
    // --------------------------------------------------
    // MATLAB: [U,S,V] = svd([Cp; Ct]) at ZERO force
    // --------------------------------------------------
    Wrench zero;
    zero.force.setZero();
    zero.moment.setZero();

    // Warm-start FK(q, 0)
    FKResult Yprev = robot_.forwardKinematics(q_, zero);

    // Statics + compliance
    StaticsResult ref =
        evaluateStaticsAndCompliance(robot_, q_, zero, &Yprev, delta_fd_);

    // Build stacked compliance Jacobian J = [Cp; Ct]
    Eigen::Matrix<double,6,3> J;
    J.topRows<3>()    = ref.C.Cp;
    J.bottomRows<3>() = ref.C.Ct;

    // SVD(J) = U S V'
    Eigen::JacobiSVD<Eigen::Matrix<double,6,3>> svd(
        J, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Keep dominant 2 force directions: V(:,1:2)
    V2_ = svd.matrixV().leftCols<2>();
  }

  gtsam::Vector evaluateError(
      const gtsam::Vector2& f,
      gtsam::OptionalMatrixType H) const override
  {
    // --------------------------------------------------
    // Map 2D force parameters -> 3D force
    // MATLAB: F = V(:,1:2) * f
    // --------------------------------------------------
    Eigen::Vector3d F = V2_ * f;

    // Tip wrench
    Wrench tip;
    tip.force  = F;
    tip.moment.setZero();

    // Warm-start statics
    Wrench zero;
    zero.force.setZero();
    zero.moment.setZero();

    FKResult Yprev = robot_.forwardKinematics(q_, zero);

    StaticsResult res =
        evaluateStaticsAndCompliance(robot_, q_, tip, &Yprev, delta_fd_);

    const StateSample& s = res.Y.samples.back();

    // --------------------------------------------------
    // Residual r = meas - pred
    // --------------------------------------------------
    Eigen::Matrix<double,6,1> r;
    r.head<3>() = p_meas_ - s.p;
    r.tail<3>() = t_meas_ - s.R.col(2);

    // --------------------------------------------------
    // Jacobian: dr/df = -[Cp; Ct] * V2
    // --------------------------------------------------
    if (H) {
      Eigen::Matrix<double,6,3> JF;
      JF.topRows<3>()    = -res.C.Cp;
      JF.bottomRows<3>() = -res.C.Ct;

      *H = JF * V2_;
    }

    return r;
  }

private:
  const RobotArmModel& robot_;
  JointState q_;

  Eigen::Vector3d p_meas_;
  Eigen::Vector3d t_meas_;

  double delta_fd_;

  // Force subspace basis: V(:,1:2)
  Eigen::Matrix<double,3,2> V2_;
};
