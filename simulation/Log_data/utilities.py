def log_3_vec( drg, var):
  for ii in range(3):
    drg.add_variable(var+"["+str(ii)+"]")