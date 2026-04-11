exec(open("Log_data/utilities.py").read())
def log_euler ( log_cycle , VEH_NAME , VEH_OBJ ) :
  recording_group_name =  ""+VEH_NAME+"_euler"
  dr_group = trick.sim_services.DRAscii(recording_group_name)
  dr_group.thisown = 0
  dr_group.set_cycle(log_cycle)
  dr_group.freq = trick.sim_services.DR_Always
  log_3_vec(dr_group,""+VEH_OBJ+".euler_rpy.ref_body_angles")
  log_3_vec(dr_group,""+VEH_OBJ+".euler_rpy.body_ref_angles")

  trick.add_data_record_group(dr_group)
  return