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

# --- GNC: target tracking (nadir / Earth-centered LOS; vehicle.sm feeds target_pos each step) ---
# gnc::GuidanceMode: IDLE=0, TARGET=1, SLEW=2, STATIONKEEP=3, RCS_MANEUVER=4
# gnc::GuidanceFrame: INERTIAL=0, LVLH=1, NED=2
vehicle.guidance.mode = 1
vehicle.guidance.target_frame = 0

trick.sim_services.exec_set_terminate_time(1000.0)

exec(open("Log_data/log_run_test.py").read())
setup_run_test_logging(0.025)
