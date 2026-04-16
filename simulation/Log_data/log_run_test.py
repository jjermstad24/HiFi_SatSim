# Data recording for RUN_test: target, vehicle state, actuators, planets,
# guidance, control/allocator, orb elem, euler.


def setup_run_test_logging(log_cycle=1):
    """Register all ASCII log groups used by RUN_test."""

    dr_group = trick.sim_services.DRAscii("target")
    dr_group.thisown = 0
    dr_group.set_cycle(log_cycle)
    dr_group.freq = trick.sim_services.DR_Always
    for t in range(1,6):
        for i in range(3):
            dr_group.add_variable(
                "target%d.pos_inertial[%d]" % (t, i), "target%d.r_inertial[%d]" % (t, i)
            )
    trick.add_data_record_group(dr_group)

    dr_group = trick.sim_services.DRAscii("fsw")
    dr_group.thisown = 0
    dr_group.set_cycle(log_cycle)
    dr_group.freq = trick.sim_services.DR_Always
    dr_group.add_variable("vehicle.fsw.current_activity")
    dr_group.add_variable("vehicle.fsw.targeting.selected_target_idx")
    for i in range(3):
        dr_group.add_variable(f'vehicle.sim2fsw_bus.w_body[{i}]')
    trick.add_data_record_group(dr_group)

    dr_group = trick.sim_services.DRAscii("vehicle")
    dr_group.thisown = 0
    dr_group.set_cycle(log_cycle)
    dr_group.freq = trick.sim_services.DR_Always
    for i in range(3):
        dr_group.add_variable(
            "vehicle.dyn_body.core_body.state.trans.position[%d]" % i,
            "vehicle.r_inertial[%d]" % i,
        )
    for i in range(3):
        dr_group.add_variable(
            "vehicle.dyn_body.core_body.state.trans.velocity[%d]" % i,
            "vehicle.v_inertial[%d]" % i,
        )
    for i in range(3):
        for j in range(3):
            dr_group.add_variable(
                "vehicle.dyn_body.core_body.state.rot.T_parent_this[%d][%d]" % (i, j),
                "vehicle.T_i2b[%d][%d]" % (i, j),
            )
    trick.add_data_record_group(dr_group)

    dr_group = trick.sim_services.DRAscii("actuators")
    dr_group.thisown = 0
    dr_group.set_cycle(log_cycle)
    dr_group.freq = trick.sim_services.DR_Always
    for i in range(3):
        dr_group.add_variable(
            "vehicle.rcs_cluster.total_force[%d]" % i, "rcs_total_force_%d" % i
        )
    for i in range(3):
        dr_group.add_variable(
            "vehicle.rcs_cluster.total_torque[%d]" % i, "rcs_total_torque_%d" % i
        )
    for i in range(3):
        dr_group.add_variable(
            "vehicle.rw_cluster.torque_body[%d]" % i, "rw_total_torque_%d" % i
        )
    for i in range(3):
        dr_group.add_variable(
            "vehicle.rw_cluster.total_momentum[%d]" % i, "rw_total_momentum_%d" % i
        )
    for i in range(3):
        dr_group.add_variable(
            "vehicle.rw_cluster.wheels[%d].omega" % i, "rw_wheel_speed_%d" % i
        )
    for i in range(3):
        dr_group.add_variable(
            "vehicle.magnetorquer.torque_body[%d]" % i, "mtq_total_torque_%d" % i
        )
    for i in range(3):
        dr_group.add_variable(
            "vehicle.magnetorquer.dipole_body[%d]" % i, "mtq_dipole_%d" % i
        )
    trick.add_data_record_group(dr_group)

    dr_group = trick.sim_services.DRAscii("planets")
    dr_group.thisown = 0
    dr_group.set_cycle(log_cycle)
    dr_group.freq = trick.sim_services.DR_Always
    for planet in ("earth", "moon"):
        for i in range(3):
            dr_group.add_variable(
                "%s.planet.inertial.state.trans.position[%d]" % (planet, i),
                "%s.r_inertial[%d]" % (planet, i),
            )
        for i in range(3):
            dr_group.add_variable(
                "%s.planet.inertial.state.trans.velocity[%d]" % (planet, i),
                "%s.v_inertial[%d]" % (planet, i),
            )
        for i in range(3):
            for j in range(3):
                dr_group.add_variable(
                    "%s.planet.pfix.state.rot.T_parent_this[%d][%d]" % (planet, i, j),
                    "%s.T_i2b[%d][%d]" % (planet, i, j),
                )
    trick.add_data_record_group(dr_group)

    exec(open("Log_data/log_guidance.py").read(), globals())
    log_guidance(log_cycle, "vehicle", "vehicle")

    exec(open("Log_data/log_control.py").read(), globals())
    log_control(log_cycle, "vehicle", "vehicle")

    exec(open("Log_data/log_orb_elem.py").read(), globals())
    log_orb_elem(log_cycle, "vehicle", "vehicle")

    exec(open("Log_data/log_euler.py").read(), globals())
    log_euler(log_cycle, "vehicle", "vehicle")
