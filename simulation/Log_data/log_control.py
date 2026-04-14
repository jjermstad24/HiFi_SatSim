def log_control(log_cycle, VEH_NAME, VEH_OBJ):
    """Log control plant I/O, allocator inputs, and routed actuator commands."""
    recording_group_name = VEH_NAME + "_control"
    dr_group = trick.sim_services.DRAscii(recording_group_name)
    dr_group.thisown = 0
    dr_group.set_cycle(log_cycle)
    dr_group.freq = trick.sim_services.DR_Always

    for ii in range(3):
        dr_group.add_variable(VEH_OBJ + ".fsw.control.out.torque[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.control.out.force[%d]" % ii)

    dr_group.add_variable(VEH_OBJ + ".fsw.control.err.q_scalar")
    dr_group.add_variable(VEH_OBJ + ".fsw.control.err.attitude_principal_angle_rad")
    for ii in range(3):
        dr_group.add_variable(VEH_OBJ + ".fsw.control.err.q_vector[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.control.err.omega_err[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.control.err.r_err[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.control.err.v_err[%d]" % ii)

    for ii in range(3):
        dr_group.add_variable(VEH_OBJ + ".fsw.control.current_state.r[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.control.current_state.v[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.control.current_state.w[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.control.desired_state.r[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.control.desired_state.v[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.control.desired_state.w[%d]" % ii)

    dr_group.add_variable(VEH_OBJ + ".fsw.control.current_state.q.scalar")
    dr_group.add_variable(VEH_OBJ + ".fsw.control.desired_state.q.scalar")
    for ii in range(3):
        dr_group.add_variable(VEH_OBJ + ".fsw.control.current_state.q.vector[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.control.desired_state.q.vector[%d]" % ii)

    dr_group.add_variable(VEH_OBJ + ".fsw.allocator.rw_torque_fraction")
    dr_group.add_variable(VEH_OBJ + ".fsw.allocator.mtq_torque_fraction")
    dr_group.add_variable(VEH_OBJ + ".fsw.allocator.rcs_torque_fraction")
    dr_group.add_variable(VEH_OBJ + ".fsw.allocator.effector_enable_mask")
    dr_group.add_variable(VEH_OBJ + ".fsw.allocator.rcs_cmd_source")
    dr_group.add_variable(VEH_OBJ + ".fsw.allocator.rw_cmd_source")
    dr_group.add_variable(VEH_OBJ + ".fsw.allocator.mtq_cmd_source")

    for ii in range(3):
        dr_group.add_variable(VEH_OBJ + ".fsw.allocator.torque_cmd_body[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.allocator.force_cmd_eci[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.allocator.B_body[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.allocator.rcs_force_body_injection[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.allocator.rw_torque_cmd[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.allocator.mtq_dipole_cmd[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.allocator.rcs_force_body[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.allocator.rcs_torque_body[%d]" % ii)

    trick.add_data_record_group(dr_group)
