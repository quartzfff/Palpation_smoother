#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/Symbol.h>

#include "RobotArmModel.hpp"
#include "StaticsEvaluation.hpp"

class TipPoseTangentFactor
  : public gtsam::NoiseModelFactor1<gtsam::Vector3>
{
public:
  TipPoseTangentFactor(
      gtsam::Key force_key,
      const RobotArmModel& robot,
      const JointState& q,
      const Eigen::Vector3d& p_meas,
      const Eigen::Vector3d& t_meas,
      double delta_fd,
      const gtsam::SharedNoiseModel& noise)
    : gtsam::NoiseModelFactor1<gtsam::Vector3>(noise, force_key),
      robot_(robot),
      q_(q),
      p_meas_(p_meas),
      t_meas_(t_meas),
      delta_fd_(delta_fd)
  {}

  gtsam::Vector evaluateError(
      const gtsam::Vector3& F,
      gtsam::OptionalMatrixType H) const override
  {
    // ----------------------------------------
    // 1. Build tip wrench
    // ----------------------------------------
    Wrench tip;
    tip.force  = Eigen::Vector3d(F(0), F(1), F(2));
    tip.moment.setZero();

    // ----------------------------------------
    // 2. Warm-start statics (MATLAB equivalent)
    //    Yprev = FK(q, 0)
    // ----------------------------------------
    Wrench zero;
    zero.force.setZero();
    zero.moment.setZero();

    FKResult Yprev = robot_.forwardKinematics(q_, zero);

    // ----------------------------------------
    // 3. Evaluate statics + compliance at F
    // ----------------------------------------
    StaticsResult res =
        evaluateStaticsAndCompliance(robot_, q_, tip, &Yprev, delta_fd_);

    const StateSample& s = res.Y.samples.back();

    Eigen::Vector3d p_pred = s.p;
    Eigen::Vector3d t_pred = s.R.col(2);

    // ----------------------------------------
    // 4. Residual (measured - predicted)
    // ----------------------------------------
    Eigen::Matrix<double,6,1> r;
    r.head<3>() = p_meas_ - p_pred;
    r.tail<3>() = t_meas_ - t_pred;

    // ----------------------------------------
    // 5. Jacobian: dr/dF = -[Cp; Ct]
    // ----------------------------------------
    if (H) {
      Eigen::Matrix<double,6,3> J;
      J.topRows<3>()    = -res.C.Cp;
      J.bottomRows<3>() = -res.C.Ct;
      *H = J;
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
