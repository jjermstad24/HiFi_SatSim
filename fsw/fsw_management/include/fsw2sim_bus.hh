#pragma once

namespace gnc {

struct Fsw2SimBus {
    static constexpr int num_rcs_thrusters = 12;

    double rw_torque_cmd[3];        //!< trick_units(N*m) Reaction Wheel torque command
    double mtq_dipole_cmd[3];       //!< trick_units(A*m^2) Magnetorquer dipole command
    double rcs_thruster_cmd[num_rcs_thrusters]; //!< trick_units(--) RCS per-thruster normalized command [0,1]
};

} // namespace gnc
