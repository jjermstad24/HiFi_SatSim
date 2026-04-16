#include "../include/fsw.hh"

#include <cstring> // For std::memcpy, std::memset
#include <cstdio>
#include <cmath>

namespace gnc {

Fsw::Fsw()
    : control_mode(ACTUATOR_MODE_RW_ONLY),
      current_activity(FSW_ACTIVITY_DETUMBLE),
      sequencer_enabled(false),
      sequence_auto_advance(false),
      activity_elapsed_s(0.0)
{
    for (int i = 0; i < FSW_ACTIVITY_COUNT; i++) {
        activities[i].num_exit_groups = 0;
        for (int g = 0; g < ActivityConfig::max_exit_groups; g++) {
            activities[i].exit_groups[g].num_conditions = 0;
        }
    }

    // Activity definitions and transition logic are owned here in C++.
    activities[FSW_ACTIVITY_DETUMBLE].guidance_mode = IDLE;
    activities[FSW_ACTIVITY_DETUMBLE].control_mode = ACTUATOR_MODE_RCS_ONLY;
    activities[FSW_ACTIVITY_DETUMBLE].kp = 0.0;
    activities[FSW_ACTIVITY_DETUMBLE].kd = 5.0;
    activities[FSW_ACTIVITY_DETUMBLE].num_exit_groups = 1;
    activities[FSW_ACTIVITY_DETUMBLE].exit_groups[0].num_conditions = 1;
    activities[FSW_ACTIVITY_DETUMBLE].exit_groups[0].conditions[0].signal = ACTIVITY_SIGNAL_BODY_RATE_MAG;
    activities[FSW_ACTIVITY_DETUMBLE].exit_groups[0].conditions[0].cmp = ACTIVITY_CMP_LT;
    activities[FSW_ACTIVITY_DETUMBLE].exit_groups[0].conditions[0].threshold = 0.005; // rad/s

    activities[FSW_ACTIVITY_POINTING].guidance_mode = TARGET;
    activities[FSW_ACTIVITY_POINTING].control_mode = ACTUATOR_MODE_RW_ONLY;
    activities[FSW_ACTIVITY_POINTING].kp = 10.0;
    activities[FSW_ACTIVITY_POINTING].kd = 2.0;
    activities[FSW_ACTIVITY_POINTING].num_exit_groups = 1;
    activities[FSW_ACTIVITY_POINTING].exit_groups[0].num_conditions = 1;
    activities[FSW_ACTIVITY_POINTING].exit_groups[0].conditions[0].signal = ACTIVITY_SIGNAL_ELAPSED_TIME;
    activities[FSW_ACTIVITY_POINTING].exit_groups[0].conditions[0].cmp = ACTIVITY_CMP_GE;
    activities[FSW_ACTIVITY_POINTING].exit_groups[0].conditions[0].threshold = 30.0; // s

    activities[FSW_ACTIVITY_SLEW].guidance_mode = SLEW;
    activities[FSW_ACTIVITY_SLEW].control_mode = ACTUATOR_MODE_RW_AND_RCS;
    activities[FSW_ACTIVITY_SLEW].kp = 8.0;
    activities[FSW_ACTIVITY_SLEW].kd = 2.0;
    activities[FSW_ACTIVITY_SLEW].num_exit_groups = 1;
    activities[FSW_ACTIVITY_SLEW].exit_groups[0].num_conditions = 2;
    // Example AND: elapsed_time >= 20s AND body_rate_mag < 0.02 rad/s
    activities[FSW_ACTIVITY_SLEW].exit_groups[0].conditions[0].signal = ACTIVITY_SIGNAL_ELAPSED_TIME;
    activities[FSW_ACTIVITY_SLEW].exit_groups[0].conditions[0].cmp = ACTIVITY_CMP_GE;
    activities[FSW_ACTIVITY_SLEW].exit_groups[0].conditions[0].threshold = 20.0;
    activities[FSW_ACTIVITY_SLEW].exit_groups[0].conditions[1].signal = ACTIVITY_SIGNAL_BODY_RATE_MAG;
    activities[FSW_ACTIVITY_SLEW].exit_groups[0].conditions[1].cmp = ACTIVITY_CMP_LT;
    activities[FSW_ACTIVITY_SLEW].exit_groups[0].conditions[1].threshold = 0.02;

    activities[FSW_ACTIVITY_STATIONKEEP].guidance_mode = STATIONKEEP;
    activities[FSW_ACTIVITY_STATIONKEEP].control_mode = ACTUATOR_MODE_RCS_ONLY;
    activities[FSW_ACTIVITY_STATIONKEEP].kp = 1.0;
    activities[FSW_ACTIVITY_STATIONKEEP].kd = 0.5;
    activities[FSW_ACTIVITY_STATIONKEEP].num_exit_groups = 1;
    activities[FSW_ACTIVITY_STATIONKEEP].exit_groups[0].num_conditions = 1;
    activities[FSW_ACTIVITY_STATIONKEEP].exit_groups[0].conditions[0].signal = ACTIVITY_SIGNAL_ELAPSED_TIME;
    activities[FSW_ACTIVITY_STATIONKEEP].exit_groups[0].conditions[0].cmp = ACTIVITY_CMP_GE;
    activities[FSW_ACTIVITY_STATIONKEEP].exit_groups[0].conditions[0].threshold = 120.0; // s

    set_activity(current_activity);
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

int Fsw::activity_count()
{
    return FSW_ACTIVITY_COUNT;
}

bool Fsw::evaluate_activity_condition(const ActivityCondition& c,
                                     const Sim2FswBus& sim2fsw_bus) const
{
    double value = 0.0;
    switch (c.signal) {
        case ACTIVITY_SIGNAL_BODY_RATE_MAG:
            value = std::sqrt(sim2fsw_bus.w_body[0] * sim2fsw_bus.w_body[0] +
                              sim2fsw_bus.w_body[1] * sim2fsw_bus.w_body[1] +
                              sim2fsw_bus.w_body[2] * sim2fsw_bus.w_body[2]);
            break;
        case ACTIVITY_SIGNAL_BODY_RATE_X_ABS:
            value = std::fabs(sim2fsw_bus.w_body[0]);
            break;
        case ACTIVITY_SIGNAL_BODY_RATE_Y_ABS:
            value = std::fabs(sim2fsw_bus.w_body[1]);
            break;
        case ACTIVITY_SIGNAL_BODY_RATE_Z_ABS:
            value = std::fabs(sim2fsw_bus.w_body[2]);
            break;
        case ACTIVITY_SIGNAL_ELAPSED_TIME:
            value = activity_elapsed_s;
            break;
        default:
            return false;
    }

    switch (c.cmp) {
        case ACTIVITY_CMP_LT: return value < c.threshold;
        case ACTIVITY_CMP_LE: return value <= c.threshold;
        case ACTIVITY_CMP_GT: return value > c.threshold;
        case ACTIVITY_CMP_GE: return value >= c.threshold;
        default: return false;
    }
}

bool Fsw::should_exit_current_activity(const Sim2FswBus& sim2fsw_bus) const
{
    if (current_activity < 0 || current_activity >= FSW_ACTIVITY_COUNT) {
        return false;
    }
    const ActivityConfig& cfg = activities[current_activity];
    if (cfg.num_exit_groups <= 0) {
        return false;
    }

    // OR across groups, AND within each group.
    for (int g = 0; g < cfg.num_exit_groups && g < ActivityConfig::max_exit_groups; g++) {
        const ActivityCriteriaGroup& group = cfg.exit_groups[g];
        if (group.num_conditions <= 0) {
            continue;
        }
        bool group_ok = true;
        for (int i = 0; i < group.num_conditions && i < ActivityCriteriaGroup::max_conditions; i++) {
            if (!evaluate_activity_condition(group.conditions[i], sim2fsw_bus)) {
                group_ok = false;
                break;
            }
        }
        if (group_ok) {
            return true;
        }
    }
    return false;
}

void Fsw::set_activity(int activity_idx)
{
    if (activity_idx < 0 || activity_idx >= FSW_ACTIVITY_COUNT) {
        return;
    }

    current_activity = activity_idx;
    guidance.mode = activities[current_activity].guidance_mode;
    control_mode = activities[current_activity].control_mode;
    control.Kp = activities[current_activity].kp;
    control.Kd = activities[current_activity].kd;
    activity_elapsed_s = 0.0;
}

void Fsw::transition_to_next_activity()
{
    int next = current_activity + 1;
    if (next >= FSW_ACTIVITY_COUNT) {
        next = 0;
    }
    set_activity(next);
}

// ---------------------------------------------------------------------------
// run_targeting
//
// Called every FSW tick.  Responsibilities:
//  1. Compute LVLH az/el for every target on the bus.
//  2. Select the best un-imaged, visible target.
//  3. Populate guidance.target_pos with the ECI position of that target.
//  4. Accumulate imaging dwell; mark bin complete when dwell is satisfied.
//  5. If the current target becomes unselected (set changed), reset dwell.
// ---------------------------------------------------------------------------
void Fsw::run_targeting(const Sim2FswBus& sim2fsw_bus)
{
    // -- 1. Compute az/el --------------------------------------------------
    targeting.compute_azel(sim2fsw_bus.r_eci,
                           sim2fsw_bus.v_eci,
                           sim2fsw_bus.target_eci,
                           sim2fsw_bus.num_targets);

    // -- 2. Select best target ---------------------------------------------
    int new_sel = targeting.select_target(sim2fsw_bus.num_targets);

    // Detect a switch in target selection and reset dwell accordingly.
    if (new_sel != targeting.selected_target_idx) {
        if (targeting.selected_target_idx >= 0) {
            std::printf("[Targeting] Switch target %d -> %d  (dwell reset)\n",
                        targeting.selected_target_idx, new_sel);
        }
        targeting.selected_target_idx = new_sel;
        targeting.current_dwell_s     = 0.0;
        targeting.dwelling            = (new_sel >= 0);
    }

    // -- 3. Populate guidance.target_pos -----------------------------------
    if (new_sel >= 0 && new_sel < sim2fsw_bus.num_targets) {
        guidance.target_pos[0] = sim2fsw_bus.target_eci[new_sel][0];
        guidance.target_pos[1] = sim2fsw_bus.target_eci[new_sel][1];
        guidance.target_pos[2] = sim2fsw_bus.target_eci[new_sel][2];
        guidance.target_frame  = INERTIAL;

        // Remember which az/el bin this corresponds to.
        targeting.selected_az_bin =
            static_cast<int>(targeting.target_azel[new_sel].az_deg / AZ_STEP);
        targeting.selected_el_bin =
            static_cast<int>(targeting.target_azel[new_sel].el_deg / EL_STEP);

        // -- 4. Accumulate dwell and mark bin when complete ----------------
        bool just_imaged = targeting.update_dwell(sim2fsw_bus.fsw_dt);
        if (just_imaged) {
            std::printf("[Targeting] Target %d imaged  az=%.1f deg  el=%.1f deg\n",
                        new_sel,
                        targeting.target_azel[new_sel].az_deg,
                        targeting.target_azel[new_sel].el_deg);
        }
    }
}

void Fsw::update(const Sim2FswBus& sim2fsw_bus, Fsw2SimBus& fsw2sim_bus) {
    if (sequencer_enabled &&
        current_activity >= 0 &&
        current_activity < FSW_ACTIVITY_COUNT) {
        guidance.mode = activities[current_activity].guidance_mode;
        control_mode = activities[current_activity].control_mode;
        control.Kp = activities[current_activity].kp;
        control.Kd = activities[current_activity].kd;
    }

    // --- GNC: STATE EXTRACTION (SIM → GNC) ---
    // Position / velocity (inertial)
    for (int i = 0; i < 3; i++) {
        guidance.sc_pos_eci[i] = sim2fsw_bus.r_eci[i];
        guidance.sc_vel_eci[i] = sim2fsw_bus.v_eci[i];
        guidance.sc_omega_body[i] = sim2fsw_bus.w_body[i];
    }

    // --- TARGETING ALGORITHM (runs when in TARGET mode) ---
    // Compute az/el of all targets, select best un-imaged target, and
    // populate guidance.target_pos automatically every tick.
    if (guidance.mode == TARGET && sim2fsw_bus.num_targets > 0) {
        run_targeting(sim2fsw_bus);
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
    }

    for (int i = 0; i < Fsw2SimBus::num_rcs_thrusters; i++) {
        fsw2sim_bus.rcs_thruster_cmd[i] = rcs_cluster.thrusters[i].cmd;
    }

    if (sequencer_enabled) {
        activity_elapsed_s += sim2fsw_bus.fsw_dt;
    }
    if (sequencer_enabled &&
        sequence_auto_advance &&
        should_exit_current_activity(sim2fsw_bus)) {
        transition_to_next_activity();
    }
}

} // namespace gnc
