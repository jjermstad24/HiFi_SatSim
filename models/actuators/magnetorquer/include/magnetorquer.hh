/*******************************************************************************

Purpose:
  High-fidelity magnetorquer model with inductive dynamics and saturation

Library dependencies:
  ((../src/magnetorquer.cpp))

*******************************************************************************/

#pragma once
#include "utils/math/include/vector3.hh"
#include "utils/math/include/math_utils.hh"
#include <cmath>

namespace gnc {

class Magnetorquer {
public:
    // Parameters
    double R;
    double L;
    double max_current;
    double max_dipole;
    double N;
    double area;

    // State
    double current;
    double dipole_body[3];

    // Inputs
    double cmd_dipole_body[3];
    double B_body[3];

    // Outputs
    double torque_body[3];

    Magnetorquer();

    void initialize();
    void update(double dt);

private:
    void compute_current(double dt);
    void compute_dipole();
    void compute_torque();
};

}