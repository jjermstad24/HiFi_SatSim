# Configure the dynamics manager to operate in empty space mode
dynamics.dyn_manager_init.mode = trick.DynManagerInit.EphemerisMode_Ephemerides
dynamics.dyn_manager_init.central_point_name = "Earth"

# Use Runge-Kutta 4 integrator
rk_integrator = trick.RK4IntegratorConstructor()
dynamics.dyn_manager_init.integ_constructor = rk_integrator

# Set up vehicle 1 in Earth orbit
vehicle.dyn_body.name = trick.NamedItem("vehicle")
vehicle.dyn_body.integ_frame_name = "Earth.inertial"
vehicle.dyn_body.translational_dynamics = True
vehicle.dyn_body.rotational_dynamics = True

exec(open("Modified_data/time.py", "r").read())
exec(open("Modified_data/vehicle_mass_props.py", "r").read())
exec(open("Modified_data/vehicle_grav_controls.py", "r").read())
exec(open("Modified_data/vehicle_state.py", "r").read())

exec(open("Modified_data/targets.py", "r").read())

vehicle.load_config('/home/jjermsta/SimulationFramework/simulation/Modified_data/vehicle_config.json')

# ---------------------------------------------------------------------------
# FSW configuration
# ---------------------------------------------------------------------------
# Start in TARGET mode so the targeting algorithm runs from the first tick.
vehicle.fsw.sequencer_enabled = True
vehicle.fsw.sequence_auto_advance = False       # targeting algo manages selection
vehicle.fsw.set_activity(1)                     # FSW_ACTIVITY_POINTING (TARGET mode)

# Override targeting algorithm defaults set in targets.py if desired:
# vehicle.fsw.targeting.image_dwell_time_s = 60.0
# vehicle.fsw.targeting.min_elevation_deg  = 10.0

# ---------------------------------------------------------------------------
# Simulation stop time: ~60 minutes to observe several orbital passes
# ---------------------------------------------------------------------------
trick.sim_services.exec_set_terminate_time(100.0)

exec(open("Log_data/log_run_test.py").read())
# setup_run_test_logging(0.025)

# ---------------------------------------------------------------------------
# DataLogger configuration (Native Parquet/CSV)
# ---------------------------------------------------------------------------
# Target 1-5
for t in range(1, 6):
    for i in range(3):
        data_logger.add_variable(f"target{t}.pos_inertial[{i}]")

# FSW
data_logger.add_variable("vehicle.fsw.current_activity")
data_logger.add_variable("vehicle.fsw.targeting.selected_target_idx")
for i in range(3):
    data_logger.add_variable(f"vehicle.sim2fsw_bus.w_body[{i}]")
    data_logger.add_variable(f"vehicle.sim2fsw_bus.B_body[{i}]")

# Vehicle States
for i in range(3):
    data_logger.add_variable(f"vehicle.dyn_body.core_body.state.trans.position[{i}]")
    data_logger.add_variable(f"vehicle.dyn_body.core_body.state.trans.velocity[{i}]")
for i in range(3):
    for j in range(3):
        data_logger.add_variable(f"vehicle.dyn_body.core_body.state.rot.T_parent_this[{i}][{j}]")

# Actuators
for i in range(3):
    data_logger.add_variable(f"vehicle.rcs_cluster.total_force[{i}]")
    data_logger.add_variable(f"vehicle.rcs_cluster.total_torque[{i}]")
    data_logger.add_variable(f"vehicle.rw_cluster.torque_body[{i}]")
    data_logger.add_variable(f"vehicle.rw_cluster.total_momentum[{i}]")
    data_logger.add_variable(f"vehicle.rw_cluster.wheels[{i}].omega")
    data_logger.add_variable(f"vehicle.magnetorquer.torque_body[{i}]")
    data_logger.add_variable(f"vehicle.magnetorquer.dipole_body[{i}]")

# Planets (Earth, Moon)
for planet in ["earth", "moon"]:
    for i in range(3):
        data_logger.add_variable(f"{planet}.planet.inertial.state.trans.position[{i}]")
        data_logger.add_variable(f"{planet}.planet.inertial.state.trans.velocity[{i}]")
    for i in range(3):
        for j in range(3):
            data_logger.add_variable(f"{planet}.planet.pfix.state.rot.T_parent_this[{i}][{j}]")

# Guidance
data_logger.add_variable("vehicle.fsw.guidance.mode")
data_logger.add_variable("vehicle.fsw.guidance.target_frame")
data_logger.add_variable("vehicle.fsw.guidance.slew_time")
data_logger.add_variable("vehicle.fsw.guidance.slew_duration")
for i in range(3):
    data_logger.add_variable(f"vehicle.fsw.guidance.sc_pos_eci[{i}]")
    data_logger.add_variable(f"vehicle.fsw.guidance.sc_vel_eci[{i}]")
    data_logger.add_variable(f"vehicle.fsw.guidance.sc_omega_body[{i}]")
    data_logger.add_variable(f"vehicle.fsw.guidance.target_pos[{i}]")
    data_logger.add_variable(f"vehicle.fsw.guidance.out.r_desired[{i}]")
    data_logger.add_variable(f"vehicle.fsw.guidance.out.v_desired[{i}]")
    data_logger.add_variable(f"vehicle.fsw.guidance.out.w_desired[{i}]")
    data_logger.add_variable(f"vehicle.fsw.guidance.out.rcs_force_cmd_eci[{i}]")
    data_logger.add_variable(f"vehicle.fsw.guidance.out.rcs_torque_cmd_body[{i}]")
    data_logger.add_variable(f"vehicle.fsw.guidance.out.rw_torque_cmd_body[{i}]")
    data_logger.add_variable(f"vehicle.fsw.guidance.out.mtq_dipole_cmd_body[{i}]")

# Control
data_logger.add_variable("vehicle.fsw.control.mode")
for i in range(3):
     data_logger.add_variable(f"vehicle.fsw.control.q_err.vector[{i}]")
data_logger.add_variable("vehicle.fsw.control.q_err.scalar")

# Orb Elem
vars_orb = ["semi_major_axis", "semiparam", "e_mag", "inclination", "arg_periapsis", 
            "long_asc_node", "r_mag", "vel_mag", "true_anom", "mean_motion", 
            "orbital_anom", "orb_energy", "orb_ang_momentum"]
for v in vars_orb:
    data_logger.add_variable(f"vehicle.orb_elem.elements.{v}")

# At the end of the simulation, write to parquet
# Trick 10+ uses trick.add_event() or we can just use a trick job.
# For now, let's just use a trick job defined in the SimObject for shutdown.
