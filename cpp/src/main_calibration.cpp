#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>

#include <Eigen/Dense>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "RobotArmModel.hpp"
#include "factors/TipPoseTangentCalibrationFactors.hpp"

// ============================================================
// Data container
// ============================================================
struct CalibSample {
  JointState q;              // FK joint state
  Eigen::Vector3d p_meas;    // mm, base frame
  Eigen::Vector3d t_meas;    // unit, base frame
};

// ============================================================
// Quaternion → tangent
// ============================================================
static Eigen::Vector3d quatToTangentZ(const Eigen::Vector4d& qxyzw)
{
  Eigen::Quaterniond q(qxyzw[3], qxyzw[0], qxyzw[1], qxyzw[2]);
  q.normalize();
  return (q.toRotationMatrix() * Eigen::Vector3d::UnitZ()).normalized();
}

// ============================================================
// Load CSV
// ============================================================
static std::vector<CalibSample> load_calib_csv(
    const std::string& path,
    const Eigen::Matrix3d& R_calib,
    const Eigen::Vector3d& t_calib,
    const Eigen::Vector3d& p_offset)
{
  std::ifstream fin(path);
  if (!fin.is_open())
    throw std::runtime_error("Cannot open CSV: " + path);

  std::vector<CalibSample> data;
  std::string line;

  std::getline(fin, line); // header

  while (std::getline(fin, line)) {
    if (line.empty()) continue;

    std::stringstream ss(line);
    std::string tok;
    std::vector<double> v;

    while (std::getline(ss, tok, ',')) {
      v.push_back(std::stod(tok));
    }
    if (v.size() < 11) continue;

    // CSV order:
    // theta_in, theta_out, d_in, d_out, px,py,pz, qx,qy,qz,qw
    JointState q;
    q.theta2 = v[0];
    q.theta1 = v[1];
    q.d2     = v[2];
    q.d1     = v[3];

    Eigen::Vector3d p_ndi(v[4], v[5], v[6]);
    Eigen::Vector4d qxyzw(v[7], v[8], v[9], v[10]);

    Eigen::Vector3d p_base = R_calib * p_ndi + t_calib + p_offset;
    Eigen::Vector3d t_base = (R_calib * quatToTangentZ(qxyzw)).normalized();

    data.push_back({q, p_base, t_base});
  }

  std::cout << "Loaded " << data.size() << " samples from " << path << "\n";
  return data;
}

// ============================================================
// Build robot template
// ============================================================
struct RobotTemplateOut {
  RobotArmModel robot;
  double kout_eq_x;
};

static RobotTemplateOut buildRobotTemplate()
{
  const double OD1=1.56, ID1=1.14;
  const double OD2=1.04, ID2=0.82;
  const double OD3=0.70, ID3=0.50;

  const double E  = 60e9 / 1e6;   // N/mm^2
  const double nu = 0.3;
  const double G  = E / (2*(1+nu));

  auto I = [](double OD, double ID) {
    return M_PI/4.0 * (pow(OD/2,4) - pow(ID/2,4));
  };

  double I1=I(OD1,ID1), I2=I(OD2,ID2), I3=I(OD3,ID3);
  double EI1=E*I1, EI2=E*I2, EI3=E*I3;
  double GJ1=2*EI1, GJ2=2*EI2, GJ3=2*EI3;

  Eigen::Vector3d k1(65.0/1000.0,0,0);
  Eigen::Vector3d k1eq = k1 * (EI1 / (EI1 + EI2 + EI3));

  Tube outer(50.0, EI1, GJ1, k1eq);
  Tube inner(50.0, EI2, GJ2, Eigen::Vector3d::Zero());
  static Tube tool(13.75, EI3, GJ3, Eigen::Vector3d::Zero());

  RobotArmModel robot(outer, inner, &tool);
  return {robot, k1eq.x()};
}

// ============================================================
// Stage 1 — inner curvature
// ============================================================
static double run_stage1_inner(
    const RobotArmModel& robot,
    const std::vector<CalibSample>& data)
{
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values init;

  gtsam::Key K = gtsam::Symbol('I',0);

  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
    (Eigen::Matrix<double,6,1>() << 0.5,0.5,0.5, 0.02,0.02,0.02).finished());

  graph.add(gtsam::PriorFactor<double>(
    K, 0.0, gtsam::noiseModel::Isotropic::Sigma(1,0.02)));

  for (const auto& s : data)
    graph.add(std::make_shared<TipPoseTangentCalibFactorInner>(
      K, robot, s.q, s.p_meas, s.t_meas, noise));

  init.insert(K, 0.0);

  gtsam::LevenbergMarquardtOptimizer opt(graph, init);
  return opt.optimize().at<double>(K);
}

// ============================================================
// Stage 2 — outer curvature + theta offset
// ============================================================
static std::pair<double,double> run_stage2_outer(
    const RobotArmModel& robot,
    const std::vector<CalibSample>& data,
    double kappa_in_eq,
    double kout_prior)
{
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values init;

  gtsam::Key KO = gtsam::Symbol('O',0);
  gtsam::Key KT = gtsam::Symbol('T',0);

  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
    (Eigen::Matrix<double,6,1>() << 0.5,0.5,0.5, 0.02,0.02,0.02).finished());

  graph.add(gtsam::PriorFactor<double>(
    KO, kout_prior, gtsam::noiseModel::Isotropic::Sigma(1,0.01)));
  graph.add(gtsam::PriorFactor<double>(
    KT, 0.0, gtsam::noiseModel::Isotropic::Sigma(1,0.15)));

  RobotArmModel locked = robot;
  locked.setInnerKappaX(kappa_in_eq);

  for (const auto& s : data)
    graph.add(std::make_shared<TipPoseTangentCalibFactorOuter>(
      KO, KT, locked, s.q, s.p_meas, s.t_meas, noise));

  init.insert(KO, kout_prior);
  init.insert(KT, 0.0);

  auto res = gtsam::LevenbergMarquardtOptimizer(graph, init).optimize();
  return {res.at<double>(KO), res.at<double>(KT)};
}

// ============================================================
// Main
// ============================================================
int main(int argc, char** argv)
{
  std::string csv1 = "calib_stage1_inner.csv";
  std::string csv2 = "calib_stage2_outer.csv";
  if (argc > 1) csv1 = argv[1];
  if (argc > 2) csv2 = argv[2];

  Eigen::Matrix3d R;
  R << 1,0,0, 0,0,-1, 0,1,0;

  Eigen::Vector3d t(0,0,0);
  Eigen::Vector3d tool_offset(0,0,13.75);

  auto data1 = load_calib_csv(csv1, R, t, tool_offset);
  auto data2 = load_calib_csv(csv2, R, t, tool_offset);

  auto tmpl = buildRobotTemplate();

  double kin = run_stage1_inner(tmpl.robot, data1);
  auto [kout, dth] = run_stage2_outer(tmpl.robot, data2, kin, tmpl.kout_eq_x);

  std::cout << "\nRESULT:\n";
  std::cout << "  kappa_in_eq   = " << kin  << " [1/mm]\n";
  std::cout << "  kappa_out_eq  = " << kout << " [1/mm]\n";
  std::cout << "  theta_offset  = " << dth  << " [rad]\n";

  return 0;
}
