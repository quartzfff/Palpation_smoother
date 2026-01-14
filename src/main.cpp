#include <iostream>
#include <Eigen/Dense>
#include "Compliance.hpp"
#include "RobotArmModel.hpp"
#include "StaticsEvaluation.hpp"

#include <cmath>





int main()
{

    // ---------------- Robot Parameters ----------------

// Geometry (mm)
const double OD1 = 1.56;
const double ID1 = 1.14;
const double OD2 = 1.04;
const double ID2 = 0.82;
const double OD3 = 0.70;
const double ID3 = 0.50;

// Material
const double E  = 60e9 / 1e6;          // N/mm^2
const double nu = 0.3;
const double G  = E / (2.0 * (1.0 + nu));

// ---------------- Section properties ----------------

// Second moment of area (mm^4)
const double I1 = M_PI / 4.0 *
    (std::pow(OD1 / 2.0, 4) - std::pow(ID1 / 2.0, 4));

const double I2 = M_PI / 4.0 *
    (std::pow(OD2 / 2.0, 4) - std::pow(ID2 / 2.0, 4));

const double I3 = M_PI / 4.0 *
    (std::pow(OD3 / 2.0, 4) - std::pow(ID3 / 2.0, 4));

// Torsional constants
const double J1 = 2.0 * I1;
const double J2 = 2.0 * I2;
const double J3 = 2.0 * I3;

// ---------------- Stiffness ----------------

const double EI1 = E * I1;
const double EI2 = E * I2;
const double EI3 = E * I3;

const double GJ1 = G * J1;
const double GJ2 = G * J2;
const double GJ3 = G * J3;

// ---------------- Curvature ----------------

// Nominal outer-tube curvature (1/mm)
Eigen::Vector3d k1(65.0 / 1000.0, 0.0, 0.0);

// Equivalent curvature (MATLAB k1eq)
Eigen::Vector3d k1eq =
    k1 * (EI1 / (EI1 + EI2 + EI3));
    // -------------------------------
    // 1. Define tubes
    // -------------------------------
Tube outer(
    /*length*/ 50.0,
    /*EI*/     EI1,
    /*GJ*/     GJ1,
    /*kappa*/  k1eq
);

Tube inner(
    /*length*/ 50.0,
    /*EI*/     EI2,
    /*GJ*/     GJ2,
    /*kappa*/  Eigen::Vector3d::Zero()
);

Tube tool(
    /*length*/ 13.75,
    /*EI*/     EI3,
    /*GJ*/     GJ3,
    /*kappa*/  Eigen::Vector3d::Zero()
);
    // No tool for first test
    //RobotArmModel robot(outer, inner, nullptr);
    RobotArmModel robot(outer, inner, &tool);


    // -------------------------------
    // 2. Joint state
    // -------------------------------
    JointState q;
    q.d1 = 0.0;           // outer insertion
    q.d2 = 0.0;           // inner insertion
    q.theta1 = 3.1416;
    q.theta2 = 0.0; // 90 deg inner rotation

    // -------------------------------
    // 3. Tip wrench (zero for FK test)
    // -------------------------------
    Wrench tip;
    tip.force  = Eigen::Vector3d::Zero();
   // tip.force = Eigen::Vector3d(1.0, 0.0, 0.0);

    tip.moment = Eigen::Vector3d::Zero();



StaticsResult res =
    evaluateStaticsAndCompliance(robot, q, tip);

std::cout << "Tip position:\n"
          << res.Y.samples.back().p.transpose() << "\n\n";

std::cout << "Cp:\n" << res.C.Cp << "\n\n";
std::cout << "Ct:\n" << res.C.Ct << "\n\n";


    return 0;
}
