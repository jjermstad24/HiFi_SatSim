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

# --- Launch Vehicle Deployment: High initial tip-off rates ---
vehicle.rot_init.ang_velocity = [0.1, 0.1, 0.1] # Rad/s

import numpy as np

target.lon = np.deg2rad(-70)
target.lat = np.deg2rad(30)
target.alt = 0

vehicle.load_config('/home/jjermsta/SimulationFramework/simulation/Modified_data/vehicle_config.json')

# --- GNC: IDLE Mode (to start with) ---
vehicle.fsw.guidance.mode = 0 # IDLE
vehicle.fsw.guidance.target_frame = 0

# --- FSW Control Mode: RCS ONLY to handle deployment rates ---
vehicle.fsw.control_mode = 1 # RCS_ONLY

trick.sim_services.exec_set_terminate_time(500.0)

exec(open("Log_data/log_run_test.py").read())
setup_run_test_logging(0.025)

for i in range(3):
    vehicle.fsw.guidance.target_pos[i] = target.pos_inertial[i]
