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
exec(open("Modified_data/time.py","r").read())
exec(open("Modified_data/vehicle_mass_props.py","r").read())
exec(open("Modified_data/vehicle_grav_controls.py","r").read())
exec(open("Modified_data/vehicle_state.py","r").read())

import numpy as np
target.lon = np.deg2rad(-70)
target.lat = np.deg2rad(30)
target.alt = 0

trick.sim_services.exec_set_terminate_time(6000.0)

dr_group = trick.sim_services.DRAscii("target")
dr_group.thisown = 0
dr_group.set_cycle(1)
dr_group.freq = trick.sim_services.DR_Always
for i in range(3):
  dr_group.add_variable(f"target.pos_inertial[{i}]",f'target.r_inertial[{i}]')
trick.add_data_record_group(dr_group)

dr_group = trick.sim_services.DRAscii("vehicle")
dr_group.thisown = 0
dr_group.set_cycle(1)
dr_group.freq = trick.sim_services.DR_Always
for i in range(3):
  dr_group.add_variable(f"vehicle.dyn_body.core_body.state.trans.position[{i}]",f'vehicle.r_inertial[{i}]')
for i in range(3):
  dr_group.add_variable(f"vehicle.dyn_body.core_body.state.trans.velocity[{i}]",f'vehicle.v_inertial[{i}]')
for i in range(3):
  for j in range(3):
    dr_group.add_variable(f"vehicle.dyn_body.core_body.state.rot.T_parent_this[{i}][{j}]",f'vehicle.T_i2b[{i}]')
trick.add_data_record_group(dr_group)

exec(open("Log_data/log_orb_elem.py").read())
log_orb_elem(1,"vehicle","vehicle")

exec(open("Log_data/log_euler.py").read())
log_euler(1,"vehicle","vehicle")

dr_group = trick.sim_services.DRAscii("planets")
dr_group.thisown = 0
dr_group.set_cycle(1)
dr_group.freq = trick.sim_services.DR_Always
for planet in [
  'earth',
  'moon',
  ]:
  for i in range(3):
    dr_group.add_variable(f"{planet}.planet.inertial.state.trans.position[{i}]",f'{planet}.r_inertial[{i}]')
  for i in range(3):
    dr_group.add_variable(f"{planet}.planet.inertial.state.trans.velocity[{i}]",f'{planet}.v_inertial[{i}]')
  for i in range(3):
    for j in range(3):
      dr_group.add_variable(f"{planet}.planet.pfix.state.rot.T_parent_this[{i}][{j}]",f'{planet}.T_i2b[{i}]')
trick.add_data_record_group(dr_group)