#pragma once

#include "RobotArmModel.hpp"
#include "Compliance.hpp"

struct StaticsResult {
    FKResult Y;          // final backbone
    FKResult Y0;         // baseline at Ftip
    ComplianceResult C;  // Cp, Ct
};

StaticsResult evaluateStaticsAndCompliance(
    const RobotArmModel& model,
    const JointState& q,
    const Wrench& tip,
    const FKResult* Yprev = nullptr,
    double delta = 1e-7
);
