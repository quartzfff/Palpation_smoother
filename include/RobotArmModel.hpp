#pragma once
#include <Eigen/Dense>
#include <vector>

struct JointState {
    double d1;        // outer translation
    double d2;        // inner translation
    double theta1;    // outer rotation
    double theta2;    // inner rotation
};

struct Wrench {
    Eigen::Vector3d force;
    Eigen::Vector3d moment;
};

struct StateSample {
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    Eigen::Vector3d n;
    Eigen::Vector3d m;
    double theta;
};

struct FKResult {
    std::vector<StateSample> samples;
};

struct Segment {
    double s0;
    double s1;

    Eigen::Vector3d kappa_eff;   // piecewise curvature
    double EI_eff;
    double GJ_eff;
};

class Tube {
public:
    Tube(double length,
         double EI,
         double GJ,
         const Eigen::Vector3d& kappa)
        : length_(length), EI_(EI), GJ_(GJ), kappa_(kappa) {}

    double length() const { return length_; }
    double EI() const { return EI_; }
    double GJ() const { return GJ_; }
    const Eigen::Vector3d& kappa() const { return kappa_; }

private:
    double length_;
    double EI_;
    double GJ_;
    Eigen::Vector3d kappa_;   // nominal curvature
};


class RobotArmModel {
public:
    RobotArmModel(const Tube& outer,
                  const Tube& inner,
                  const Tube* tool = nullptr);

    FKResult forwardKinematics(
        const JointState& q,
        const Wrench& tip,
        const FKResult* prev = nullptr) const;

private:
    // physical tubes
    Tube outer_;
    Tube inner_;
    const Tube* tool_;   // nullptr if no tool

    // model constants 
    double alpha_b_;
    double alpha_t_;
    double Kg_;

    // clearance model
    double r_mm_;
    double c_mm_;

    // helpers
    Eigen::Matrix3d Rz(double theta) const;
    Eigen::Matrix3d Rx(double theta) const;
    double clearanceAngle(double L_mm) const;

    // segment construction
    std::vector<Segment> buildSegments(const JointState& q) const;
};
