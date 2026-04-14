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
    double torque[3]; //!< trick_units(N*m)
    double force[3]; //!< trick_units(N)
};

/** Attitude / rate / translation errors used for the PD law (logged each step). */
struct ControlErrorState {
    /** Vector part of q_err = q_current ⊗ q_desired* (JEOD quaternion convention). */
    double q_vector[3]; //!< trick_units(--)
    /**
     * Scalar part of q_err. For a unit attitude error quaternion, zero rotation means
     * q_err = identity → scalar → 1 and vector → 0 (not scalar → 0).
     */
    double q_scalar; //!< trick_units(--)
    /** Principal rotation angle (rad) corresponding to q_err; → 0 when aligned. */
    double attitude_principal_angle_rad; //!< trick_units(rad)
    /** ω_desired − ω_current (body, rad/s). */
    double omega_err[3]; //!< trick_units(rad/s)
    /** r_desired − r (ECI, m). */
    double r_err[3]; //!< trick_units(m)
    /** v_desired − v (ECI, m/s). */
    double v_err[3]; //!< trick_units(m/s)
};

struct State {
  double r[3]; //!< trick_units(m)
  double v[3]; //!< trick_units(m/s)
  jeod::Quaternion q; //!< trick_units(--)
  double w[3]; //!< trick_units(rad/s)
};

class Control {
public:
    State current_state; //!< trick_units(--)
    State desired_state; //!< trick_units(--)

    ControlOutput out; //!< trick_units(--)
    ControlErrorState err; //!< trick_units(--)

    // Attitude gains
    double Kp; //!< trick_units(--)
    double Kd; //!< trick_units(--)

    Control();

    void update(double dt);
};

}
