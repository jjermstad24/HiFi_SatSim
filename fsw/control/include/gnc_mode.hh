/*******************************************************************************
Purpose:
  Effector enables and command routing for the GNC → allocator → actuator chain.
Library dependencies:
  (())
*******************************************************************************/
#pragma once

namespace gnc {

/** Bitmask: which hardware paths are active (allocator zeros disabled paths). */
enum GncEffectorMask : unsigned {
    GNC_EFFECTOR_NONE = 0,
    GNC_EFFECTOR_RCS = 1u << 0,
    GNC_EFFECTOR_RW = 1u << 1,
    GNC_EFFECTOR_MTQ = 1u << 2,
    GNC_EFFECTOR_ALL = GNC_EFFECTOR_RCS | GNC_EFFECTOR_RW | GNC_EFFECTOR_MTQ
};

/** Per actuator: use closed-loop control output or guidance-published command. */
enum GncActuatorCmdSource : int {
    GNC_CMD_FROM_CONTROL = 0,
    GNC_CMD_FROM_GUIDANCE = 1
};

/** Selects an actuator control mode from the various options below. */
enum ActuatorControlMode : int {
    ACTUATOR_MODE_RW_ONLY = 0,
    ACTUATOR_MODE_RCS_ONLY = 1,
    ACTUATOR_MODE_RW_AND_RCS = 2,
    ACTUATOR_MODE_MTQ_ONLY = 3,
    ACTUATOR_MODE_ALL = 4
};

} // namespace gnc
