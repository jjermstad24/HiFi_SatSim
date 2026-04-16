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

# --- Detumbling Test: High initial angular velocity ---
# Override the default low-rate initialization from vehicle_state.py so
# RCS commands exceed the thruster on-threshold and jets visibly fire.
vehicle.rot_init.ang_velocity = [0.2, -0.1, 0.05]

exec(open("Modified_data/targets.py", "r").read())

vehicle.load_config('/home/jjermsta/SimulationFramework/simulation/Modified_data/vehicle_config.json')

# --- FSW Activity Sequencer ---
# Start from DETUMBLE and continue through remaining activities.
# vehicle.fsw.sequencer_enabled = True
# vehicle.fsw.sequence_auto_advance = True
vehicle.fsw.set_activity(0) # FSW_ACTIVITY_DETUMBLE

trick.sim_services.exec_set_terminate_time(300.0)

exec(open("Log_data/log_run_test.py").read())
setup_run_test_logging(0.025)

