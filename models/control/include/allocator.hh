
/*******************************************************************************
Purpose:
  Maps control wrench (body torque, ECI force) into actuator-native commands:
  reaction wheel torque, magnetorquer dipole, and RCS force/torque (body).

Library dependencies:
  ((../src/allocator.cpp))
*******************************************************************************/
#pragma once

#include "gnc_mode.hh"
#include "guidance/include/guidance.hh"
#include "utils/quaternion/include/quat.hh"

namespace gnc {

class RCSCluster;

class Allocator {
public:
    // Inputs — set each frame before update()
    /** Attitude torque demand, body frame (matches Control::out.torque). */
    double torque_cmd_body[3];
    /** Translational force demand, ECI frame (matches Control::out.force). */
    double force_cmd_eci[3];
    /** Local geomagnetic field, body frame (T). */
    double B_body[3];

    /**
     * Added to RCS body force after ECI→body rotation of force_cmd_eci.
     * Use for open-loop tests or disturbances (default zero).
     */
    double rcs_force_body_injection[3];

    /**
     * Portion of torque_cmd_body sent to each path (each in [0,1]).
     * Defaults: wheels = 1, MTQ = 0, RCS torque = 0 (RCS does translation only).
     */
    double rw_torque_fraction;
    double mtq_torque_fraction;
    double rcs_torque_fraction;

    /** Bitwise OR of GncEffectorMask; disabled actuators are zeroed. */
    unsigned effector_enable_mask;

    GncActuatorCmdSource rcs_cmd_source;
    GncActuatorCmdSource rw_cmd_source;
    GncActuatorCmdSource mtq_cmd_source;

    // Outputs — connect to actuators (RCS wrench is also mapped to thruster cmds)
    double rw_torque_cmd[3];
    double mtq_dipole_cmd[3];
    /** Commanded RCS wrench in body frame (before per-thruster allocation). */
    double rcs_force_body[3];
    double rcs_torque_body[3];

    Allocator();

    /** One-call routing: which actuators are on and where their commands come from. */
    void apply_control_mode(unsigned effector_mask,
                            GncActuatorCmdSource rcs_src,
                            GncActuatorCmdSource rw_src,
                            GncActuatorCmdSource mtq_src);

    /**
     * @param q_parent_this JEOD body attitude: transforms vectors body → ECI parent.
     * @param rcs Thruster geometry must be initialized; receives per-thruster cmd.
     * @param g_out Guidance actuator commands (used when *_cmd_source is FROM_GUIDANCE).
     */
    void update(const jeod::Quaternion & q_parent_this,
                RCSCluster & rcs,
                const GuidanceOutput & g_out);
};

} // namespace gnc
