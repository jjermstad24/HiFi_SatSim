#include "../include/reaction_wheel.hh"

namespace gnc {

ReactionWheel::ReactionWheel()
{
    omega = 0.0;
    torque_actual = 0.0;
}

void ReactionWheel::initialize()
{
    omega = 0.0;
    torque_actual = 0.0;
}

void ReactionWheel::update(double dt)
{
    apply_limits(dt);
    update_dynamics(dt);
    compute_outputs();
}

void ReactionWheel::apply_limits(double dt)
{
    // Torque saturation
    double tau_cmd = math::clamp(torque_cmd, -max_torque, max_torque);

    // Rate limit
    double delta = tau_cmd - torque_actual;
    double max_delta = torque_rate_limit * dt;

    delta = math::clamp(delta, -max_delta, max_delta);

    torque_actual += delta;
}

void ReactionWheel::update_dynamics(double dt)
{
    // Friction
    double friction = viscous_friction * omega;

    if (std::abs(omega) > 1e-6) {
        friction += coulomb_friction * (omega > 0 ? 1.0 : -1.0);
    }

    double domega = (torque_actual - friction) / J;
    omega += domega * dt;

    // Speed saturation
    omega = math::clamp(omega, -max_speed, max_speed);
}

void ReactionWheel::compute_outputs()
{
    // Spacecraft torque = reaction torque
    torque_body[0] = -torque_actual * axis[0];
    torque_body[1] = -torque_actual * axis[1];
    torque_body[2] = -torque_actual * axis[2];

    momentum = J * omega;
}

}