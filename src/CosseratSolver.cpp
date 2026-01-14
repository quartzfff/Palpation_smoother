#include "CosseratSolver.hpp"
#include <cmath>
#include <stdexcept>

CosseratSolver::CosseratSolver(double alpha_b)
    : alpha_b_(alpha_b)
{}


Eigen::Matrix3d CosseratSolver::hat(const Eigen::Vector3d& u)
{
    Eigen::Matrix3d U;
    U <<     0.0, -u.z(),  u.y(),
          u.z(),    0.0, -u.x(),
         -u.y(),  u.x(),   0.0;
    return U;
}



StateSample CosseratSolver::ode(
    const Segment& seg,
    const StateSample& y) const
{
    if (seg.EI_eff <= 0.0 || seg.GJ_eff <= 0.0) {
        throw std::runtime_error("CosseratSolver::ode: EI_eff and GJ_eff must be > 0.");
    }

    // MATLAB:
    // Kinv = diag([1/(EI*alpha_b), 1/(EI*alpha_b), 1/GJ])
    Eigen::Matrix3d Kinv = Eigen::Matrix3d::Zero();
    Kinv(0,0) = 1.0 / (seg.EI_eff * alpha_b_);
    Kinv(1,1) = 1.0 / (seg.EI_eff * alpha_b_);
    Kinv(2,2) = 1.0 / (seg.GJ_eff * alpha_b_);

    // pdot = R(:,3)
    Eigen::Vector3d pdot = y.R.col(2);

    // u = kappa + Kinv * R.' * m
    Eigen::Vector3d u =
        seg.kappa_eff + Kinv * (y.R.transpose() * y.m);


        

    StateSample ydot;
    ydot.p     = pdot;
    ydot.R     = y.R * hat(u);
    ydot.n     = Eigen::Vector3d::Zero();
    ydot.m     = -pdot.cross(y.n);
    ydot.theta = u.z();

    return ydot;
}


StateSample CosseratSolver::addScaled(
    const StateSample& y,
    const StateSample& k,
    double h)
{
    StateSample out = y;
    out.p     += h * k.p;
    out.R     += h * k.R;
    out.n     += h * k.n;
    out.m     += h * k.m;
    out.theta += h * k.theta;
    return out;
}



StateSample CosseratSolver::integrateSegment(
    const Segment& seg,
    const StateSample& y0,
    int N,
    std::vector<StateSample>& out_samples) const
{
    if (N < 2 || seg.s1 <= seg.s0) {
        out_samples.push_back(y0);
        return y0;
    }

    // Heun3 coefficients (MATLAB)
    const double c2  = 1.0 / 3.0;
    const double a21 = 1.0 / 3.0;
    const double c3  = 2.0 / 3.0;
    const double a31 = 0.0;
    const double a32 = 2.0 / 3.0;
    const double b1  = 1.0 / 4.0;
    const double b2  = 0.0;
    const double b3  = 3.0 / 4.0;

    const double h = (seg.s1 - seg.s0) / static_cast<double>(N - 1);

    StateSample y = y0;
    out_samples.push_back(y);

    for (int i = 0; i < N - 1; ++i) {
        // K1
        StateSample K1 = ode(seg, y);

        // K2
        StateSample y2 = addScaled(y, K1, h * a21);
        StateSample K2 = ode(seg, y2);

        // K3
        StateSample y3 = y;
        y3.p     += h * (a32 * K2.p);
        y3.R     += h * (a32 * K2.R);
        y3.n     += h * (a32 * K2.n);
        y3.m     += h * (a32 * K2.m);
        y3.theta += h * (a32 * K2.theta);

        StateSample K3 = ode(seg, y3);

        // Update
        y.p     += h * (b1*K1.p + b3*K3.p);
        y.R     += h * (b1*K1.R + b3*K3.R);
        y.n     += h * (b1*K1.n + b3*K3.n);
        y.m     += h * (b1*K1.m + b3*K3.m);
        y.theta += h * (b1*K1.theta + b3*K3.theta);

        out_samples.push_back(y);
    }

    return y;
}
