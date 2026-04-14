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

import numpy as np

target.lon = np.deg2rad(-70)
target.lat = np.deg2rad(30)
target.alt = 0

vehicle.load_config('/home/jjermsta/SimulationFramework/simulation/Modified_data/vehicle_config.json')

# --- Orbit Insertion via sequencer activity configuration ---
vehicle.fsw.guidance.maneuver_force_eci = [20.0, 10.0, 0.0] # N (example)
vehicle.fsw.sequencer_enabled = True
vehicle.fsw.sequence_auto_advance = False
vehicle.fsw.set_activity(3) # FSW_ACTIVITY_STATIONKEEP
vehicle.fsw.activities[3].guidance_mode = 4 # RCS_MANEUVER
vehicle.fsw.activities[3].control_mode = 1 # ACTUATOR_MODE_RCS_ONLY

# --- For constant force, we need allocator to take command from guidance ---
# GncActuatorCmdSource: GNC_CMD_FROM_CONTROL = 0, GNC_CMD_FROM_GUIDANCE = 1
vehicle.fsw.allocator.rcs_cmd_source = 1 

trick.sim_services.exec_set_terminate_time(500.0)

exec(open("Log_data/log_run_test.py").read())
setup_run_test_logging(0.025)

for i in range(3):
    vehicle.fsw.guidance.target_pos[i] = trick.attach_units("m", target.pos_inertial[i])
