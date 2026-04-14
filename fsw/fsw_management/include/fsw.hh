/*******************************************************************************
Purpose:
  Full guidance system with frames, slew, targeting

Library dependencies:
  ((../src/fsw.cpp))
*******************************************************************************/
#pragma once

#include "control/include/gnc_mode.hh" // For ActuatorControlMode
#include "control/include/control.hh"
#include "control/include/allocator.hh"
#include "guidance/include/guidance.hh"
#include "actuators/magnetorquer/include/magnetorquer.hh"
#include "actuators/rcs/include/rcs_cluster.hh"
#include "actuators/reaction_wheel/include/rw_cluster.hh"

#include "sim2fsw_bus.hh"
#include "fsw2sim_bus.hh"

namespace gnc {

class Fsw {
public:
    Guidance guidance;
    Control control;
    Allocator allocator;

    Magnetorquer magnetorquer; // Physical model, but initialized and potentially updated internally by FSW
    RCSCluster rcs_cluster;    // Physical model, but initialized and potentially updated internally by FSW
    RWCluster rw_cluster;      // Physical model, but initialized and potentially updated internally by FSW

    ActuatorControlMode control_mode;

    Fsw(); // Constructor

    /**
     * Main FSW update function.
     * Takes inputs from sim, computes commands, and populates outputs to sim.
     */
    void update(const Sim2FswBus& sim2fsw_bus, Fsw2SimBus& fsw2sim_bus);

    /** Initialize FSW components (magnetorquer, rcs, rw clusters) */
    void initialize();
};

} // namespace gnc
