#include "../include/magnetorquer.hh"

namespace gnc {

Magnetorquer::Magnetorquer()
{
    current = 0.0;
}

void Magnetorquer::initialize()
{
    current = 0.0;
}

void Magnetorquer::update(double dt)
{
    compute_current(dt);
    compute_dipole();
    compute_torque();
}

void Magnetorquer::compute_current(double dt)
{
    // Magnitude of commanded dipole
    double m_cmd_mag =
        std::sqrt(cmd_dipole_body[0]*cmd_dipole_body[0] +
                  cmd_dipole_body[1]*cmd_dipole_body[1] +
                  cmd_dipole_body[2]*cmd_dipole_body[2]);

    // Convert to current
    double i_cmd = m_cmd_mag / (N * area);

    // Clamp
    i_cmd = math::clamp(i_cmd, -max_current, max_current);

    // RL time constant
    double tau = L / R;

    // First-order lag
    double di = (i_cmd - current) / tau;
    current += di * dt;
}

void Magnetorquer::compute_dipole()
{
    double m_scale = N * area;

    // Saturation using tanh
    double i_eff = max_current * std::tanh(current / max_current);

    double m_mag = m_scale * i_eff;

    // Normalize command direction
    double norm =
        std::sqrt(cmd_dipole_body[0]*cmd_dipole_body[0] +
                  cmd_dipole_body[1]*cmd_dipole_body[1] +
                  cmd_dipole_body[2]*cmd_dipole_body[2]);

    if (norm > 1e-8) {
        dipole_body[0] = m_mag * cmd_dipole_body[0] / norm;
        dipole_body[1] = m_mag * cmd_dipole_body[1] / norm;
        dipole_body[2] = m_mag * cmd_dipole_body[2] / norm;
    } else {
        dipole_body[0] = dipole_body[1] = dipole_body[2] = 0.0;
    }
}

void Magnetorquer::compute_torque()
{
    torque_body[0] = dipole_body[1]*B_body[2] - dipole_body[2]*B_body[1];
    torque_body[1] = dipole_body[2]*B_body[0] - dipole_body[0]*B_body[2];
    torque_body[2] = dipole_body[0]*B_body[1] - dipole_body[1]*B_body[0];
}

}