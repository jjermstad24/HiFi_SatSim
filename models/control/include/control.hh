
/*******************************************************************************
Purpose:
  Full control system

Library dependencies:
  ((../src/control.cpp))
*******************************************************************************/
#pragma once
#include "utils/math/include/vector3.hh"
#include "utils/quaternion/include/quat.hh"

namespace gnc {

struct ControlOutput {
    double torque[3];
    double force[3];
};

struct State {
  double r[3];
  double v[3];
  jeod::Quaternion q;
  double w[3];
};

class Control {
public:
    State current_state;
    State desired_state;

    ControlOutput out;

    void update(double dt);
};

}
