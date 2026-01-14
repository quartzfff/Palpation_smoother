#pragma once
#include <Eigen/Dense>
#include <vector>
#include "RobotArmModel.hpp"

class CosseratSolver {
public:
    explicit CosseratSolver(double alpha_b = 1.0);

    // Integrate ONE segment with Heun3
    // N = number of integration points 
    // Appends samples to out_samples
    // Returns final state at seg.s1
    StateSample integrateSegment(
        const Segment& seg,
        const StateSample& y0,
        int N,
        std::vector<StateSample>& out_samples) const;

private:
    double alpha_b_;

    // Cosserat ODE: ydot = f(y)
    StateSample ode(
        const Segment& seg,
        const StateSample& y) const;

    // hat operator
    static Eigen::Matrix3d hat(const Eigen::Vector3d& u);

    // y + h * k
    static StateSample addScaled(
        const StateSample& y,
        const StateSample& k,
        double h);
};
