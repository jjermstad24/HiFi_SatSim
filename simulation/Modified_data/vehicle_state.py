vehicle.trans_init.set_subject_body(vehicle.dyn_body)
vehicle.trans_init.reference_ref_frame_name = "Earth.inertial"
vehicle.trans_init.body_frame_id            = "composite_body"
vehicle.trans_init.position                 = [2191880.0, -6022135.0, 3700000.0]
vehicle.trans_init.velocity                 = [6843.0, 2510.0, 758.0]

vehicle.rot_init.set_subject_body(vehicle.dyn_body)
vehicle.rot_init.reference_ref_frame_name = "Earth.inertial"
vehicle.rot_init.body_frame_id            = "composite_body"
vehicle.rot_init.orientation.data_source  = trick.Orientation.InputEigenRotation
vehicle.rot_init.orientation.eigen_angle  = 0.0
vehicle.rot_init.orientation.eigen_axis   = [0.0, 0.0, 1.0]
vehicle.rot_init.ang_velocity             = [0.002, 0.0002, 0.0003]

dynamics.dyn_manager.add_body_action( vehicle.mass_init )
dynamics.dyn_manager.add_body_action( vehicle.trans_init )
dynamics.dyn_manager.add_body_action( vehicle.rot_init )

vehicle.orb_elem.reference_name = "Earth"
vehicle.pfix.reference_name = "Earth"
vehicle.lvlh.reference_name = "Earth"
vehicle.euler_rpy.sequence = trick.Orientation.Roll_Pitch_Yaw