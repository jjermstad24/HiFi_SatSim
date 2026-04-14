#pragma once

#include "utils/quaternion/include/quat.hh"

namespace gnc {

struct Sim2FswBus {
    // State
    double r_eci[3];        //!< trick_units(m) Position in ECI frame
    double v_eci[3];        //!< trick_units(m/s) Velocity in ECI frame
    jeod::Quaternion q_eci_to_body; //!< trick_units(--) Quaternion from ECI to Body frame
    double w_body[3];       //!< trick_units(rad/s) Angular velocity in Body frame

    // Environment
    double B_body[3];       //!< trick_units(T) Magnetic field in Body frame (T)

    // Time (if needed by FSW internally)
    double sim_time;        //!< trick_units(s)
    double fsw_dt;          //!< trick_units(s) FSW timestep
};

} // namespace gnc
