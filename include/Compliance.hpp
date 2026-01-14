#pragma once
#include <Eigen/Dense>
#include "RobotArmModel.hpp"

struct ComplianceResult{
    Eigen::Matrix3d Cp;
    Eigen::Matrix3d Ct;
};

ComplianceResult computeCompliance(
    const RobotArmModel& model,
    const JointState& q,
    const Wrench& tip,
    double delta);
