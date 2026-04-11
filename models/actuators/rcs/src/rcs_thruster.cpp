#include "../include/rcs_thruster.hh"

namespace gnc {
static constexpr double G0 = 9.80665;
RCSThruster::RCSThruster() {
    valve_position = 0.0; chamber_pressure = 0.0;
    is_on = false; on_timer = 0.0; off_timer = 0.0;
}
void RCSThruster::initialize() {
    valve_position = 0.0; chamber_pressure = 0.0;
}
void RCSThruster::update(double dt) {
    update_logic(dt); update_valve(dt); update_pressure(dt); compute_force();
}
void RCSThruster::update_logic(double dt) {
    if (cmd > 0.5) {
        on_timer += dt; off_timer = 0.0;
        if (on_timer > ignition_delay) is_on = true;
    } else {
        off_timer += dt; on_timer = 0.0;
        if (off_timer > shutdown_delay) is_on = false;
    }
    if (!is_on && on_timer > 0.0 && on_timer < min_on_time) is_on = true;
}
void RCSThruster::update_valve(double dt) {
    double u = is_on ? 1.0 : 0.0;
    double dx = (u - valve_position) / valve_tau;
    valve_position += dx * dt;
    valve_position = math::clamp(valve_position, 0.0, 1.0);
}
void RCSThruster::update_pressure(double dt) {
    double P_cmd = valve_position * chamber_pressure_nom;
    double dP = (P_cmd - chamber_pressure) / pressure_tau;
    chamber_pressure += dP * dt;
}
void RCSThruster::compute_force() {
    double thrust = (chamber_pressure / chamber_pressure_nom) * thrust_vac;
    force_body[0] = thrust * direction[0];
    force_body[1] = thrust * direction[1];
    force_body[2] = thrust * direction[2];
    torque_body[0] = position[1]*force_body[2] - position[2]*force_body[1];
    torque_body[1] = position[2]*force_body[0] - position[0]*force_body[2];
    torque_body[2] = position[0]*force_body[1] - position[1]*force_body[0];
    mass_flow = thrust / (Isp * G0);
}
}
