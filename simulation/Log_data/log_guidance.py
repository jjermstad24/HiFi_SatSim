def log_guidance(log_cycle, VEH_NAME, VEH_OBJ):
    """Log guidance mode, inputs, and GuidanceOutput (refs + actuator cmds)."""
    recording_group_name = VEH_NAME + "_guidance"
    dr_group = trick.sim_services.DRAscii(recording_group_name)
    dr_group.thisown = 0
    dr_group.set_cycle(log_cycle)
    dr_group.freq = trick.sim_services.DR_Always

    dr_group.add_variable(VEH_OBJ + ".fsw.guidance.mode")
    dr_group.add_variable(VEH_OBJ + ".fsw.guidance.target_frame")
    dr_group.add_variable(VEH_OBJ + ".fsw.guidance.slew_time")
    dr_group.add_variable(VEH_OBJ + ".fsw.guidance.slew_duration")
    dr_group.add_variable(VEH_OBJ + ".fsw.guidance.stationkeep_emit_rcs_pd")
    dr_group.add_variable(VEH_OBJ + ".fsw.guidance.stationkeep_kp")
    dr_group.add_variable(VEH_OBJ + ".fsw.guidance.stationkeep_kd")

    for ii in range(3):
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.sc_pos_eci[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.sc_vel_eci[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.sc_omega_body[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.target_pos[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.maneuver_force_eci[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.hold_r_eci[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.hold_v_eci[%d]" % ii)

    for ii in range(3):
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.out.r_desired[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.out.v_desired[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.out.w_desired[%d]" % ii)

    dr_group.add_variable(VEH_OBJ + ".fsw.guidance.out.q_desired.scalar")
    for ii in range(3):
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.out.q_desired.vector[%d]" % ii)

    for ii in range(3):
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.out.rcs_force_cmd_eci[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.out.rcs_torque_cmd_body[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.out.rw_torque_cmd_body[%d]" % ii)
        dr_group.add_variable(VEH_OBJ + ".fsw.guidance.out.mtq_dipole_cmd_body[%d]" % ii)

    trick.add_data_record_group(dr_group)
