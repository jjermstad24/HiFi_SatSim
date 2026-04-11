/*******************************************************************************
Purpose:
  ()

Library dependencies:
  ((../src/gnc_system.cpp))
*******************************************************************************/

#pragma once
#include "navigation/include/mekf.hh"
#include "guidance/include/nadir_pointing.hh"
#include "control/include/attitude_controller.hh"
#include "control_bdot/include/bdot_controller.hh"
#include "core_mode/include/mode_manager.hh"

namespace gnc {

class GncSystem {
public:
    GncSystem();
    void step(const SensorData&, const double r[3], const double v[3], double dt, ActuatorCommand&);
private:
    MEKF nav_;
    NadirPointing guidance_;
    AttitudeController ctrl_;
    BDotController bdot_;
    ModeManager mode_;
    double B_prev_[3];
};

}
