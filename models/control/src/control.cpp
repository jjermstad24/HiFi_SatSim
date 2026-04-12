
#include "../include/control.hh"

namespace gnc {

void Control::update(double /*dt*/)
{
    jeod::Quaternion q_error;

    // q_error = q_current * q_desired^(-1)
    current_state.q.multiply_conjugate(desired_state.q, q_error);

    // Extract quaternion vector part (JEOD convention: vector = [x,y,z])
    double e[3];
    e[0] = q_error.vector[0];
    e[1] = q_error.vector[1];
    e[2] = q_error.vector[2];

    // Angular velocity error: we = w_desired - w_current
    double we[3];
    jeod::Vector3::decr(desired_state.w, current_state.w, we);

    // Torque = -Kp * e - Kd * we
    out.torque[0] = (-10.0 * e[0]) + (2.0 * we[0]);
    out.torque[1] = (-10.0 * e[1]) + (2.0 * we[1]);
    out.torque[2] = (-10.0 * e[2]) + (2.0 * we[2]);

    // Position error: pe = r_desired - r_current
    double pe[3];
    jeod::Vector3::decr(desired_state.r, current_state.r, pe);

    // Velocity error: ve = v_desired - v_current
    double ve[3];
    jeod::Vector3::decr(desired_state.v, current_state.v, ve);

    // Force = Kp * pe + Kd * ve
    out.force[0] = (1e-3 * pe[0]) + (1e-2 * ve[0]);
    out.force[1] = (1e-3 * pe[1]) + (1e-2 * ve[1]);
    out.force[2] = (1e-3 * pe[2]) + (1e-2 * ve[2]);
}

}
