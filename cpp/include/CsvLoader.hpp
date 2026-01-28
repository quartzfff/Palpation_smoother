#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "RobotArmModel.hpp"   // for JointState

struct MeasRow {
  JointState q;
  Eigen::Vector3d p_meas;  // base frame, mm
  Eigen::Vector3d t_meas;  // base frame, unit
};

std::vector<MeasRow> load_meas_csv(
    const std::string& csv_path,
    const Eigen::Matrix3d& R_calib,
    const Eigen::Vector3d& t_calib,
    const Eigen::Vector3d& p_offset_base = Eigen::Vector3d(0,0,13.75),
    int stride = 1  // use 10 to downsample, etc.
);
