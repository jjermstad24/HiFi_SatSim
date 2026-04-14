/*******************************************************************************
Purpose:
  ()

Library dependencies:
  ((../src/attitude_controller.cpp))
*******************************************************************************/

#pragma once
#include "common/include/types.hh"

namespace gnc {

class AttitudeController {
public:
    AttitudeController(double kp, double kd);
    void compute(const State&, const AttitudeCommand&, double torque[3]);

private:
    double kp_; //!< trick_units(--)
    double kd_; //!< trick_units(--)
};

}
