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

/** Attitude / rate / translation errors used for the PD law (logged each step). */
struct ControlErrorState {
    /** Vector part of q_err = q_current ⊗ q_desired* (JEOD quaternion convention). */
    double q_vector[3];
    /**
     * Scalar part of q_err. For a unit attitude error quaternion, zero rotation means
     * q_err = identity → scalar → 1 and vector → 0 (not scalar → 0).
     */
    double q_scalar;
    /** Principal rotation angle (rad) corresponding to q_err; → 0 when aligned. */
    double attitude_principal_angle_rad;
    /** ω_desired − ω_current (body, rad/s). */
    double omega_err[3];
    /** r_desired − r (ECI, m). */
    double r_err[3];
    /** v_desired − v (ECI, m/s). */
    double v_err[3];
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
    ControlErrorState err;

    // Attitude gains
    double Kp;
    double Kd;

    Control();

    void update(double dt);
};

}
