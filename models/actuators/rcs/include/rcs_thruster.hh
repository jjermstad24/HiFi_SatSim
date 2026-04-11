/*******************************************************************************
Purpose:
  High-fidelity RCS thruster model with valve and pressure dynamics
Library dependencies:
  ((../src/rcs_thruster.cpp))
*******************************************************************************/
#pragma once
#include "utils/math/include/math_utils.hh"

namespace gnc {
class RCSThruster {
public:
    double thrust_vac, Isp, chamber_pressure_nom;
    double valve_tau, pressure_tau;
    double ignition_delay, shutdown_delay, min_on_time;
    double position[3], direction[3];
    double valve_position, chamber_pressure, on_timer, off_timer;
    bool is_on;
    double cmd;
    double force_body[3], torque_body[3], mass_flow;
    RCSThruster();
    void initialize();
    void update(double dt);
private:
    void update_logic(double dt);
    void update_valve(double dt);
    void update_pressure(double dt);
    void compute_force();
};
}
