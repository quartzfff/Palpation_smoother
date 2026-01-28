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
    void setKappa(const Eigen::Vector3d& kappa) { kappa_ = kappa; }

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
        // ---- Calibration setters ----
    // Set intrinsic curvature magnitude along +x in tube material frame
    void setInnerKappaX(double kappa_x) {
        inner_.setKappa(Eigen::Vector3d(kappa_x, 0.0, 0.0));
    }


    // void setOuterKappa(double kappa_x) {
    //     outer_.setKappa(Eigen::Vector3d(kappa_x, 0.0, 0.0));
    // }
    void setOuterKappaEq(double kappa_eq_x) {
        outer_.setKappa(Eigen::Vector3d(kappa_eq_x, 0.0, 0.0));
    }


    void setOuterThetaOffset(double dtheta) {
        theta1_offset_ = dtheta;
    }

    void setClearanceParams(double r_mm, double c_mm) {
        r_mm_ = r_mm;
        c_mm_ = c_mm;
    }

    double outerThetaOffset() const { return theta1_offset_; }
    double clearance_r_mm()  const { return r_mm_; }
    double clearance_c_mm()  const { return c_mm_; }

private:
    // physical tubes
    Tube outer_;
    Tube inner_;
    const Tube* tool_;   // nullptr if no tool

    // model constants 
    double alpha_b_;
    double alpha_t_;
    double Kg_;

    double theta1_offset_ = 0.0;

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
