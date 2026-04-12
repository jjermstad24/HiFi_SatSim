#include "../include/control.hh"

#include <cmath>

namespace gnc {

Control::Control()
    : Kp(10.0),
      Kd(2.0)
{
}

void Control::update(double /*dt*/)
{
    jeod::Quaternion q_error;

    // q_error = q_current * q_desired^(-1)
    current_state.q.multiply_conjugate(desired_state.q, q_error);

    err.q_vector[0] = q_error.vector[0];
    err.q_vector[1] = q_error.vector[1];
    err.q_vector[2] = q_error.vector[2];
    err.q_scalar = q_error.scalar;

    const double vnorm = std::sqrt(err.q_vector[0] * err.q_vector[0] +
                                   err.q_vector[1] * err.q_vector[1] +
                                   err.q_vector[2] * err.q_vector[2]);
    err.attitude_principal_angle_rad =
        2.0 * std::atan2(vnorm, std::fabs(q_error.scalar));

    jeod::Vector3::decr(desired_state.w, current_state.w, err.omega_err);

    // τ = -Kp q_vec - Kd ω_body : P on quaternion error, D on measured rate (damping).
    out.torque[0] = (-Kp * err.q_vector[0]) - (Kd * current_state.w[0]);
    out.torque[1] = (-Kp * err.q_vector[1]) - (Kd * current_state.w[1]);
    out.torque[2] = (-Kp * err.q_vector[2]) - (Kd * current_state.w[2]);

    jeod::Vector3::decr(desired_state.r, current_state.r, err.r_err);
    jeod::Vector3::decr(desired_state.v, current_state.v, err.v_err);

    // Force = Kp_trans * r_err + Kd_trans * v_err
    out.force[0] = (1e-3 * err.r_err[0]) + (1e-2 * err.v_err[0]);
    out.force[1] = (1e-3 * err.r_err[1]) + (1e-2 * err.v_err[1]);
    out.force[2] = (1e-3 * err.r_err[2]) + (1e-2 * err.v_err[2]);
}

}
