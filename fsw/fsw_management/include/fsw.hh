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
#include "guidance/include/targeting.hh"
#include "actuators/magnetorquer/include/magnetorquer.hh"
#include "actuators/rcs/include/rcs_cluster.hh"
#include "actuators/reaction_wheel/include/rw_cluster.hh"

#include "sim2fsw_bus.hh"
#include "fsw2sim_bus.hh"

namespace gnc {

enum FswActivity : int {
    FSW_ACTIVITY_DETUMBLE = 0,
    FSW_ACTIVITY_POINTING = 1,
    FSW_ACTIVITY_SLEW = 2,
    FSW_ACTIVITY_STATIONKEEP = 3,
    FSW_ACTIVITY_COUNT = 4
};

enum ActivitySignal : int {
    ACTIVITY_SIGNAL_BODY_RATE_MAG = 0,
    ACTIVITY_SIGNAL_BODY_RATE_X_ABS = 1,
    ACTIVITY_SIGNAL_BODY_RATE_Y_ABS = 2,
    ACTIVITY_SIGNAL_BODY_RATE_Z_ABS = 3,
    ACTIVITY_SIGNAL_ELAPSED_TIME = 4
};

enum ActivityComparator : int {
    ACTIVITY_CMP_LT = 0,
    ACTIVITY_CMP_LE = 1,
    ACTIVITY_CMP_GT = 2,
    ACTIVITY_CMP_GE = 3
};

struct ActivityCondition {
    ActivitySignal signal; //!< trick_units(--)
    ActivityComparator cmp; //!< trick_units(--)
    double threshold; //!< trick_units(--)
};

struct ActivityCriteriaGroup {
    static const int max_conditions = 8;
    ActivityCondition conditions[max_conditions]; //!< trick_units(--)
    int num_conditions; //!< trick_units(--)
};

struct ActivityConfig {
    static const int max_exit_groups = 4;
    GuidanceMode guidance_mode; //!< trick_units(--)
    ActuatorControlMode control_mode; //!< trick_units(--)
    double kp; //!< trick_units(--)
    double kd; //!< trick_units(--)
    ActivityCriteriaGroup exit_groups[max_exit_groups]; //!< trick_units(--)
    int num_exit_groups; //!< trick_units(--)
};

class Fsw {
public:
    Guidance guidance; //!< trick_units(--)
    TargetingAlgorithm targeting; //!< trick_units(--)
    Control control; //!< trick_units(--)
    Allocator allocator; //!< trick_units(--)

    Magnetorquer magnetorquer; //!< trick_units(--) // Physical model, but initialized and potentially updated internally by FSW
    RCSCluster rcs_cluster;    //!< trick_units(--) // Physical model, but initialized and potentially updated internally by FSW
    RWCluster rw_cluster;      //!< trick_units(--) // Physical model, but initialized and potentially updated internally by FSW

    ActuatorControlMode control_mode; //!< trick_units(--)

    // Activity sequencer
    ActivityConfig activities[FSW_ACTIVITY_COUNT]; //!< trick_units(--)
    int current_activity; //!< trick_units(--)
    bool sequencer_enabled; //!< trick_units(--)
    bool sequence_auto_advance; //!< trick_units(--)
    double activity_elapsed_s; //!< trick_units(s)

    Fsw(); // Constructor

    /**
     * Main FSW update function.
     * Takes inputs from sim, computes commands, and populates outputs to sim.
     */
    void update(const Sim2FswBus& sim2fsw_bus, Fsw2SimBus& fsw2sim_bus);

    /**
     * Run targeting algorithm: compute az/el for all known targets, select
     * the best un-imaged target, set guidance.target_pos, and accumulate
     * imaging dwell.  Called once per FSW tick from update().
     */
    void run_targeting(const Sim2FswBus& sim2fsw_bus);

    /** Initialize FSW components (magnetorquer, rcs, rw clusters) */
    void initialize();

    /** Apply a specific activity index [0, FSW_ACTIVITY_COUNT). */
    void set_activity(int activity_idx);

    /** Move to next activity; wraps at final activity. */
    void transition_to_next_activity();

    /** Number of available sequencer activities. */
    static int activity_count();

private:
    bool evaluate_activity_condition(const ActivityCondition& c,
                                     const Sim2FswBus& sim2fsw_bus) const;
    bool should_exit_current_activity(const Sim2FswBus& sim2fsw_bus) const;
};

} // namespace gnc
