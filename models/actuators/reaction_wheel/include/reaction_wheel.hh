/*******************************************************************************

Purpose:
  High-fidelity reaction wheel model with saturation, friction, and dynamics

Library dependencies:
  ((../src/reaction_wheel.cpp))

*******************************************************************************/

#pragma once

#include "utils/math/include/math_utils.hh"
#include <cmath>
#include <algorithm>

namespace gnc {

class ReactionWheel {
public:
    // Parameters
    double J;               // wheel inertia (kg*m^2)
    double max_torque;      // Nm
    double max_speed;       // rad/s

    double viscous_friction;
    double coulomb_friction;

    double torque_rate_limit; // Nm/s

    double axis[3];         // unit vector in body frame

    // State
    double omega;           // wheel speed (rad/s)
    double torque_actual;

    // Input
    double torque_cmd;

    // Outputs
    double torque_body[3];
    double momentum;

    ReactionWheel();

    void initialize();
    void update(double dt);

private:
    void apply_limits(double dt);
    void update_dynamics(double dt);
    void compute_outputs();
};

}