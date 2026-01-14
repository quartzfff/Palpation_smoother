#pragma once
#include <eigen3/Dense>

class Tube{

    public:

        Tube() = default;

        Tube(double length, double EI, double GJ, const Eigen::Vector3d& kappa):
            length_(length), EI_(EI), GJ_(GJ),kappa_(kappa) {}

          double length() const { return length_; }
        double EI() const { return EI_; }
        double GJ() const { return GJ_; }
        const Eigen::Vector3d& kappa() const { return kappa_; }

        bool isStraight() const {
            return kappa_.norm() < 1e-8;
        }

    private:
          double length_;              // mm
          double EI_;                  // bending stiffness
          double GJ_;                  // torsional stiffness
          Eigen::Vector3d kappa_;      // intrinsic curvature [kx, ky, kz]
}