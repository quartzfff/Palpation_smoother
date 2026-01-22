#include <iostream>
#include <vector>
#include <memory>
#include <iomanip>
#include <algorithm>

#include <Eigen/Dense>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include "CsvLoader.hpp"
#include "RobotArmModel.hpp"
#include "StaticsEvaluation.hpp"
#include "factors/TipPoseTangentFactor2D_Bias.hpp"

// ======================================================
// Build robot model
// ======================================================
static RobotArmModel buildRobot()
{
  const double OD1 = 1.56, ID1 = 1.14;
  const double OD2 = 1.04, ID2 = 0.82;
  const double OD3 = 0.70, ID3 = 0.50;

  const double E  = 60e9 / 1e6;   // N/mm^2
  const double nu = 0.3;
  const double G  = E / (2.0 * (1.0 + nu));

  const double I1 = M_PI / 4.0 * (std::pow(OD1/2.0,4) - std::pow(ID1/2.0,4));
  const double I2 = M_PI / 4.0 * (std::pow(OD2/2.0,4) - std::pow(ID2/2.0,4));
  const double I3 = M_PI / 4.0 * (std::pow(OD3/2.0,4) - std::pow(ID3/2.0,4));

  const double J1 = 2.0 * I1;
  const double J2 = 2.0 * I2;
  const double J3 = 2.0 * I3;

  const double EI1 = E * I1, EI2 = E * I2, EI3 = E * I3;
  const double GJ1 = G * J1, GJ2 = G * J2, GJ3 = G * J3;

  Eigen::Vector3d k1(65.0/1000.0, 0.0, 0.0);
  Eigen::Vector3d k1eq = k1 * (EI1 / (EI1 + EI2 + EI3));

  Tube outer(50.0, EI1, GJ1, k1eq);
  Tube inner(50.0, EI2, GJ2, Eigen::Vector3d::Zero());
  static Tube tool(13.75, EI3, GJ3, Eigen::Vector3d::Zero());

  return RobotArmModel(outer, inner, &tool);
}

// ======================================================
// MAIN
// ======================================================
int main()
{
  RobotArmModel robot = buildRobot();

  // -----------------------------
  // Load measurements
  // -----------------------------
  Eigen::Matrix3d R_calib;
  R_calib << 1, 0,  0,
             0, 0, -1,
             0, 1,  0;

  Eigen::Vector3d t_calib(0,0,0);
  Eigen::Vector3d p_offset_base(0,0,13.75);

  const int stride = 10;

  auto data = load_meas_csv(
      "tube2_hole0_down.csv",
      R_calib, t_calib, p_offset_base, stride);

  if (data.empty()) {
    std::cerr << "ERROR: No data loaded\n";
    return 1;
  }

  std::cout << "Loaded " << data.size() << " samples\n";
  std::cout << std::fixed << std::setprecision(4);

  // =====================================================
  // STEP 1: Estimate bias from first N frames (zero force)
  // =====================================================
  const size_t Nbias = std::min<size_t>(20, data.size());

  Eigen::Matrix<double,6,1> b_est = Eigen::Matrix<double,6,1>::Zero();

  Wrench zero;
  zero.force.setZero();
  zero.moment.setZero();

  for (size_t k = 0; k < Nbias; ++k) {

    StaticsResult stat0 =
        evaluateStaticsAndCompliance(robot, data[k].q, zero);

    const StateSample& s = stat0.Y.samples.back();

    Eigen::Matrix<double,6,1> r;
    r.head<3>() = data[k].p_meas - s.p;
    r.tail<3>() = data[k].t_meas - s.R.col(2);

    b_est += r;
  }

  b_est /= static_cast<double>(Nbias);

  std::cout << "\nEstimated bias from first " << Nbias << " samples:\n";
  std::cout << "  bp = " << b_est.head<3>().transpose() << " mm\n";
  std::cout << "  bt = " << b_est.tail<3>().transpose() << "\n\n";

  // =====================================================
  // STEP 2: Build factor graph (bias is FIXED)
  // =====================================================
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  gtsam::Key B = gtsam::Symbol('B', 0);

  initial.insert(
      B,
      (gtsam::Vector6() << b_est(0), b_est(1), b_est(2),
                           b_est(3), b_est(4), b_est(5)).finished());

  for (size_t k = 0; k < data.size(); ++k) {
    initial.insert(gtsam::Symbol('F', k), gtsam::Vector2(0.0, 0.0));
  }

  // -----------------------------
  // Noise models
  // -----------------------------
  Eigen::Matrix<double,6,1> sig_meas;
  sig_meas << 0.5, 0.5, 0.5,
              0.03, 0.03, 0.03;
  auto measNoise =
      gtsam::noiseModel::Diagonal::Sigmas(sig_meas);

  auto smoothNoise =
      gtsam::noiseModel::Isotropic::Sigma(2, 0.5);

  // VERY tight prior → bias is frozen
  Eigen::Matrix<double,6,1> sig_lock;
  sig_lock << 0.05, 0.05, 0.05,
              0.002, 0.002, 0.002;
  auto biasLock =
      gtsam::noiseModel::Diagonal::Sigmas(sig_lock);

  graph.add(gtsam::PriorFactor<gtsam::Vector6>(
      B,
      (gtsam::Vector6() << b_est(0), b_est(1), b_est(2),
                           b_est(3), b_est(4), b_est(5)).finished(),
      biasLock));

  // -----------------------------
  // Add measurement + smoothness
  // -----------------------------
  const double delta_fd = 1e-7;

  for (size_t k = 0; k < data.size(); ++k) {

    gtsam::Key Fk = gtsam::Symbol('F', k);

    graph.add(std::make_shared<TipPoseTangentFactor2D_Bias>(
        Fk, B,
        robot,
        data[k].q,
        data[k].p_meas,
        data[k].t_meas,
        delta_fd,
        measNoise));

    if (k > 0) {
      graph.add(gtsam::BetweenFactor<gtsam::Vector2>(
          gtsam::Symbol('F', k-1),
          Fk,
          gtsam::Vector2(0.0, 0.0),
          smoothNoise));
    }
  }

  // =====================================================
  // Optimize
  // =====================================================
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  params.setMaxIterations(50);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  gtsam::Values result = optimizer.optimize();

  // =====================================================
  // Print reconstructed 3D forces (MATLAB equivalent)
  // =====================================================
  std::cout << "\nEstimated forces:\n";

  for (size_t k = 0; k < data.size(); ++k) {

    gtsam::Vector2 f2 =
        result.at<gtsam::Vector2>(gtsam::Symbol('F', k));

    // Recompute V2
    StaticsResult ref =
        evaluateStaticsAndCompliance(robot, data[k].q, zero);

    Eigen::Matrix<double,6,3> J;
    J.topRows<3>()    = ref.C.Cp;
    J.bottomRows<3>() = ref.C.Ct;

    Eigen::JacobiSVD<Eigen::Matrix<double,6,3>> svd(
        J, Eigen::ComputeFullV);

    Eigen::Matrix<double,3,2> V2 = svd.matrixV().leftCols<2>();
    Eigen::Vector3d F = V2 * f2;

    std::cout << "F[" << k << "] = "
              << F.transpose() << " N\n";
  }

  return 0;
}
