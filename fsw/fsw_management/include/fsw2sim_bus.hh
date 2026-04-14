#pragma once

namespace gnc {

struct Fsw2SimBus {
    double rw_torque_cmd[3];        // Reaction Wheel torque command
    double mtq_dipole_cmd[3];       // Magnetorquer dipole command
    // Note: RCS commands will likely be internal to the FSW or passed through a more complex struct
    // For now, let's assume direct thruster commands will be managed by the FSW internal RCSCluster
};

} // namespace gnc
