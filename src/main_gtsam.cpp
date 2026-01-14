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
#include "factors/TipPoseTangentFactor.hpp"

// ======================================================
// Build robot model (same as test_fk)
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
  // CSV → base-frame measurements
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

  std::cout << "Loaded " << data.size() << " samples from CSV\n";

  if (data.empty()) {
    std::cerr << "ERROR: No data loaded.\n";
    return 1;
  }

  std::cout << std::fixed << std::setprecision(4);

  // -----------------------------
  // Debug: compare prediction vs measurement
  // IMPORTANT: use evaluateStaticsAndCompliance() (warm-started)
  // -----------------------------
  Wrench tip0;
  tip0.force.setZero();
  tip0.moment.setZero();

  const size_t Ncheck = std::min<size_t>(10, data.size());

  for (size_t k = 0; k < Ncheck; ++k) {

    const auto& m = data[k];

    // ✅ use statics wrapper
    StaticsResult stat0 = evaluateStaticsAndCompliance(robot, m.q, tip0);

    const StateSample& s_fk = stat0.Y.samples.back();
    Eigen::Vector3d p_fk = s_fk.p;
    Eigen::Vector3d t_fk = s_fk.R.col(2);

    std::cout << "\n==============================\n";
    std::cout << "Sample " << k << "\n";

    // Joint state
    std::cout << "Joint state USED for FK:\n";
    std::cout << "  d1 (outer, mm)   = " << m.q.d1 << "\n";
    std::cout << "  d2 (inner, mm)   = " << m.q.d2 << "\n";
    std::cout << "  theta1 (outer)   = " << m.q.theta1 << " rad\n";
    std::cout << "  theta2 (inner)   = " << m.q.theta2 << " rad\n";

    // FK prediction
    std::cout << "Pred (statics, zero force):\n";
    std::cout << "  p_fk = [" << p_fk.transpose() << "]\n";
    std::cout << "  t_fk = [" << t_fk.transpose() << "]\n";

    // Measurement
    std::cout << "Measurement:\n";
    std::cout << "  p_meas = [" << m.p_meas.transpose() << "]\n";
    std::cout << "  t_meas = [" << m.t_meas.transpose() << "]\n";

    // Difference
    Eigen::Vector3d dp = m.p_meas - p_fk;
    Eigen::Vector3d dt = m.t_meas - t_fk;

    std::cout << "Difference (meas - pred):\n";
    std::cout << "  dp = [" << dp.transpose()
              << "], |dp| = " << dp.norm() << " mm\n";
    std::cout << "  dt = [" << dt.transpose()
              << "], |dt| = " << dt.norm() << "\n";
  }

  // -----------------------------
  // Build factor graph
  // -----------------------------
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  // Measurement noise
  Eigen::Matrix<double,6,1> sig;
  sig << 0.5, 0.5, 0.5,     // mm
         0.03, 0.03, 0.03;  // unitless tangent
  auto measNoise = gtsam::noiseModel::Diagonal::Sigmas(sig);

  // Smoothness on force
  auto smoothNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.5);  // N

  // Prior on F0
  auto priorNoise  = gtsam::noiseModel::Isotropic::Sigma(3, 5.0);  // N

  const double delta_fd = 1e-7;

  for (size_t k = 0; k < data.size(); ++k) {
    gtsam::Key Fk = gtsam::Symbol('F', static_cast<uint64_t>(k));

    graph.add(std::make_shared<TipPoseTangentFactor>(
        Fk, robot, data[k].q, data[k].p_meas, data[k].t_meas, delta_fd, measNoise));

    initial.insert(Fk, gtsam::Vector3(0.0, 0.0, 0.0));

    if (k > 0) {
      gtsam::Key Fkm1 = gtsam::Symbol('F', static_cast<uint64_t>(k-1));
      graph.add(gtsam::BetweenFactor<gtsam::Vector3>(
          Fkm1, Fk, gtsam::Vector3(0.0, 0.0, 0.0), smoothNoise));
    }
  }

  graph.add(gtsam::PriorFactor<gtsam::Vector3>(
      gtsam::Symbol('F', 0), gtsam::Vector3(0.0, 0.0, 0.0), priorNoise));

  // -----------------------------
  // Optimize
  // -----------------------------
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SUMMARY");
  params.setMaxIterations(50);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  gtsam::Values result = optimizer.optimize();

  // -----------------------------
  // Print results
  // -----------------------------
  for (size_t k = 0; k < data.size(); ++k) {
    gtsam::Key Fk = gtsam::Symbol('F', static_cast<uint64_t>(k));
    gtsam::Vector3 Fest = result.at<gtsam::Vector3>(Fk);
    std::cout << "F[" << k << "] = " << Fest.transpose() << " N\n";
  }

  return 0;
}
