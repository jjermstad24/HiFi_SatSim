#include "../include/fsw.hh"

#include <cstring> // For std::memcpy, std::memset

namespace gnc {

Fsw::Fsw()
    : control_mode(ACTUATOR_MODE_RW_ONLY) // Default to RW only control
{
    // Constructor of member objects will be called automatically
}

void Fsw::initialize() {
    // Magnetorquer initialization
    magnetorquer.R = 2.0;
    magnetorquer.L = 0.02;
    magnetorquer.max_current = 0.5;
    magnetorquer.max_dipole = 0.05;
    magnetorquer.N = 150.0;
    magnetorquer.area = 1.0e-4;
    magnetorquer.initialize();

    // RCS CLUSTER (12 thrusters, 6-DOF layout) initialization
    // X-pointing thrusters (Force in +/- X, Torque in +/- Z)
    rcs_cluster.thrusters[0].position[0] =  1.0; rcs_cluster.thrusters[0].position[1] =  1.0; rcs_cluster.thrusters[0].position[2] =  0.0;
    rcs_cluster.thrusters[0].direction[0] = -1.0; // Force -X, Torque +Z

    rcs_cluster.thrusters[1].position[0] =  1.0; rcs_cluster.thrusters[1].position[1] = -1.0; rcs_cluster.thrusters[1].position[2] =  0.0;
    rcs_cluster.thrusters[1].direction[0] = -1.0; // Force -X, Torque -Z

    rcs_cluster.thrusters[2].position[0] = -1.0; rcs_cluster.thrusters[2].position[1] =  1.0; rcs_cluster.thrusters[2].position[2] =  0.0;
    rcs_cluster.thrusters[2].direction[0] =  1.0; // Force +X, Torque -Z

    rcs_cluster.thrusters[3].position[0] = -1.0; rcs_cluster.thrusters[3].position[1] = -1.0; rcs_cluster.thrusters[3].position[2] =  0.0;
    rcs_cluster.thrusters[3].direction[0] =  1.0; // Force +X, Torque +Z

    // Y-pointing thrusters (Force in +/- Y, Torque in +/- X)
    rcs_cluster.thrusters[4].position[0] =  0.0; rcs_cluster.thrusters[4].position[1] =  1.0; rcs_cluster.thrusters[4].position[2] =  1.0;
    rcs_cluster.thrusters[4].direction[1] = -1.0; // Force -Y, Torque +X

    rcs_cluster.thrusters[5].position[0] =  0.0; rcs_cluster.thrusters[5].position[1] =  1.0; rcs_cluster.thrusters[5].position[2] = -1.0;
    rcs_cluster.thrusters[5].direction[1] = -1.0; // Force -Y, Torque -X

    rcs_cluster.thrusters[6].position[0] =  0.0; rcs_cluster.thrusters[6].position[1] = -1.0; rcs_cluster.thrusters[6].position[2] =  1.0;
    rcs_cluster.thrusters[6].direction[1] =  1.0; // Force +Y, Torque -X

    rcs_cluster.thrusters[7].position[0] =  0.0; rcs_cluster.thrusters[7].position[1] = -1.0; rcs_cluster.thrusters[7].position[2] = -1.0;
    rcs_cluster.thrusters[7].direction[1] =  1.0; // Force +Y, Torque +X

    // Z-pointing thrusters (Force in +/- Z, Torque in +/- Y)
    rcs_cluster.thrusters[8].position[0] =  1.0; rcs_cluster.thrusters[8].position[1] =  0.0; rcs_cluster.thrusters[8].position[2] =  1.0;
    rcs_cluster.thrusters[8].direction[2] = -1.0; // Force -Z, Torque +Y

    rcs_cluster.thrusters[9].position[0] = -1.0; rcs_cluster.thrusters[9].position[1] =  0.0; rcs_cluster.thrusters[9].position[2] =  1.0;
    rcs_cluster.thrusters[9].direction[2] = -1.0; // Force -Z, Torque -Y

    rcs_cluster.thrusters[10].position[0] =  1.0; rcs_cluster.thrusters[10].position[1] =  0.0; rcs_cluster.thrusters[10].position[2] = -1.0;
    rcs_cluster.thrusters[10].direction[2] =  1.0; // Force +Z, Torque -Y

    rcs_cluster.thrusters[11].position[0] = -1.0; rcs_cluster.thrusters[11].position[1] =  0.0; rcs_cluster.thrusters[11].position[2] = -1.0;
    rcs_cluster.thrusters[11].direction[2] =  1.0; // Force +Z, Torque +Y

    // COMMON THRUSTER PARAMETERS
    for (int i = 0; i < 12; i++) {
        rcs_cluster.thrusters[i].thrust_vac = 6.0;
        rcs_cluster.thrusters[i].Isp = 220.0;
        rcs_cluster.thrusters[i].chamber_pressure_nom = 2e5;
        rcs_cluster.thrusters[i].valve_tau = 0.02;
        rcs_cluster.thrusters[i].pressure_tau = 0.05;
        rcs_cluster.thrusters[i].min_on_time = 0.01;
        rcs_cluster.thrusters[i].initialize();
    }

    // REACTION WHEEL CLUSTER initialization
    rw_cluster.num_wheels = 3;

    // X wheel
    rw_cluster.wheels[0].axis[0] = 1.0; rw_cluster.wheels[0].axis[1] = 0.0; rw_cluster.wheels[0].axis[2] = 0.0;
    // Y wheel
    rw_cluster.wheels[1].axis[0] = 0.0; rw_cluster.wheels[1].axis[1] = 1.0; rw_cluster.wheels[1].axis[2] = 0.0;
    // Z wheel
    rw_cluster.wheels[2].axis[0] = 0.0; rw_cluster.wheels[2].axis[1] = 0.0; rw_cluster.wheels[2].axis[2] = 1.0;

    for (int i = 0; i < rw_cluster.num_wheels; i++) {
        rw_cluster.wheels[i].J = 0.001;
        rw_cluster.wheels[i].max_torque = 0.02;
        rw_cluster.wheels[i].max_speed = 6000.0 * 2.0 * 3.14159 / 60.0;
        rw_cluster.wheels[i].viscous_friction = 1e-5;
        rw_cluster.wheels[i].coulomb_friction = 1e-4;
        rw_cluster.wheels[i].torque_rate_limit = 0.1;
        rw_cluster.wheels[i].initialize();
    }
}

void Fsw::update(const Sim2FswBus& sim2fsw_bus, Fsw2SimBus& fsw2sim_bus) {

    // --- GNC: STATE EXTRACTION (SIM → GNC) ---
    // Position / velocity (inertial)
    for (int i = 0; i < 3; i++) {
        guidance.sc_pos_eci[i] = sim2fsw_bus.r_eci[i];
        guidance.sc_vel_eci[i] = sim2fsw_bus.v_eci[i];
        guidance.sc_omega_body[i] = sim2fsw_bus.w_body[i];
    }

    // GuidanceMode: IDLE, TARGET, SLEW, STATIONKEEP, RCS_MANEUVER
    // guidance.mode can be set via simulation input (vehicle.fsw.guidance.mode)
    guidance.update(sim2fsw_bus.fsw_dt);

    // --- CONTROL (feeds allocator when *_cmd_source = FROM_CONTROL) ---
    // Attitude
    control.current_state.q = sim2fsw_bus.q_eci_to_body; // q_eci_to_body is equivalent to Q_parent_this.conjugate()
    control.desired_state.q = guidance.out.q_desired;

    // Rates
    for (int i = 0; i < 3; i++) {
        control.current_state.w[i]  = sim2fsw_bus.w_body[i];
        control.desired_state.w[i] = guidance.out.w_desired[i];
    }

    // Translation
    for (int i = 0; i < 3; i++) {
        control.current_state.r[i]  = sim2fsw_bus.r_eci[i];
        control.current_state.v[i]  = sim2fsw_bus.v_eci[i];
        control.desired_state.r[i] = guidance.out.r_desired[i];
        control.desired_state.v[i] = guidance.out.v_desired[i];
    }
    control.update(sim2fsw_bus.fsw_dt);

    // --- ALLOCATION (routing: effector_enable_mask + *_cmd_source vs guidance.out) ---
    for (int i = 0; i < 3; i++) {
        allocator.torque_cmd_body[i] = control.out.torque[i];
        allocator.force_cmd_eci[i] = 0.0; // Force disabled to prevent saturation
        allocator.B_body[i] = sim2fsw_bus.B_body[i];
    }

    // GNC routing: configure allocator based on control_mode
    switch (control_mode) {
        case gnc::ACTUATOR_MODE_RW_ONLY:
            allocator.apply_control_mode(gnc::GNC_EFFECTOR_RW,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL);
            allocator.rw_torque_fraction = 1.0;
            allocator.rcs_torque_fraction = 0.0;
            allocator.mtq_torque_fraction = 0.0;
            break;
        case gnc::ACTUATOR_MODE_RCS_ONLY:
            allocator.apply_control_mode(gnc::GNC_EFFECTOR_RCS,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL);
            allocator.rw_torque_fraction = 0.0;
            allocator.rcs_torque_fraction = 1.0;
            allocator.mtq_torque_fraction = 0.0;
            break;
        case gnc::ACTUATOR_MODE_RW_AND_RCS:
            allocator.apply_control_mode(gnc::GNC_EFFECTOR_RW | gnc::GNC_EFFECTOR_RCS,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL);
            allocator.rw_torque_fraction = 0.5; // Example split
            allocator.rcs_torque_fraction = 0.5;
            allocator.mtq_torque_fraction = 0.0;
            break;
        case gnc::ACTUATOR_MODE_MTQ_ONLY:
            allocator.apply_control_mode(gnc::GNC_EFFECTOR_MTQ,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL);
            allocator.rw_torque_fraction = 0.0;
            allocator.rcs_torque_fraction = 0.0;
            allocator.mtq_torque_fraction = 1.0;
            break;
        case gnc::ACTUATOR_MODE_ALL:
            allocator.apply_control_mode(gnc::GNC_EFFECTOR_ALL,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL);
            allocator.rw_torque_fraction = 0.33; // Example split
            allocator.rcs_torque_fraction = 0.33;
            allocator.mtq_torque_fraction = 0.34;
            break;
        default:
            // Default to RW only if an unknown mode is set
            allocator.apply_control_mode(gnc::GNC_EFFECTOR_RW,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL,
                                         gnc::GNC_CMD_FROM_CONTROL);
            allocator.rw_torque_fraction = 1.0;
            allocator.rcs_torque_fraction = 0.0;
            allocator.mtq_torque_fraction = 0.0;
            break;
    }

    allocator.update(
        sim2fsw_bus.q_eci_to_body, // Assumes q_eci_to_body is conjugate of q_parent_this
        rcs_cluster,
        guidance.out);

    // --- APPLY FSW OUTPUTS TO BUS ---
    for (int i = 0; i < 3; i++) {
        fsw2sim_bus.rw_torque_cmd[i] = allocator.rw_torque_cmd[i];
        fsw2sim_bus.mtq_dipole_cmd[i] = allocator.mtq_dipole_cmd[i];
        // For RCS, the thruster commands are directly applied to rcs_cluster internally by allocator.update()
    }
}

} // namespace gnc
