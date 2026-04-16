/*******************************************************************************
Purpose:
  Data-only header describing a ground target's position outputs so that
  VehicleSimObject can hold pointers to GroundTargetSimObject instances
  without including the Trick .sm file (which is not C++ includable).

  GroundTargetSimObject extends this struct with the Trick SimObject
  machinery defined in target.sm.
*******************************************************************************/
#pragma once
#include "utils/planet_fixed/planet_fixed_posn/include/planet_fixed_posn.hh"
#include "environment/planet/include/planet.hh"

/**
 * Plain C++ base providing the observable state of a ground target.
 * Trick's `GroundTargetSimObject` (target.sm) inherits from this.
 */
struct GroundTargetData {
    double lat;           //!< trick_units(rad)
    double lon;           //!< trick_units(rad)
    double alt;           //!< trick_units(m)
    double pos_inertial[3]; //!< trick_units(m)
};
