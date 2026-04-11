/*******************************************************************************
Purpose:
  ()

Library dependencies:
  ()
*******************************************************************************/

#pragma once
#include "utils/math/include/vector3.hh"
#include "utils/quaternion/include/quat.hh"

namespace gnc {

struct State {
    jeod::Quaternion q; // (--)
    double omega[3]; // (--)
    double gyro_bias[3]; // (--)
};

struct SensorData {
    double gyro[3]; // (--)
    double mag[3]; // (--)
    jeod::Quaternion star_tracker_q; // (--)
    bool star_tracker_valid; // (--)
};

struct AttitudeCommand {
    jeod::Quaternion q_cmd; // (--)
    double omega_cmd[3]; // (--)
};

struct ActuatorCommand {
    double torque_cmd[3]; // (--)
    double mtq_dipole[3]; // (--)
};

}
