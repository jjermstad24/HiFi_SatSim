#pragma once

#include "utils/quaternion/include/quat.hh"

namespace gnc {

struct Sim2FswBus {
    // State
    double r_eci[3];        // Position in ECI frame
    double v_eci[3];        // Velocity in ECI frame
    jeod::Quaternion q_eci_to_body; // Quaternion from ECI to Body frame
    double w_body[3];       // Angular velocity in Body frame

    // Environment
    double B_body[3];       // Magnetic field in Body frame (T)

    // Time (if needed by FSW internally)
    double sim_time;
    double fsw_dt;          // FSW timestep
};

} // namespace gnc
