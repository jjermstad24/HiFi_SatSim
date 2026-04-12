
/*******************************************************************************
Purpose:
  Full guidance system with frames, slew, targeting

Library dependencies:
  ((../src/guidance.cpp))
*******************************************************************************/
#pragma once
#include "utils/math/include/vector3.hh"
#include "utils/quaternion/include/quat.hh"

namespace gnc {

enum GuidanceMode {
    IDLE = 0,
    TARGET,
    SLEW,
    STATIONKEEP,
    /** Constant ECI force command for RCS (see maneuver_force_eci). */
    RCS_MANEUVER
};
enum GuidanceFrame { INERTIAL=0, LVLH, NED };

struct GuidanceOutput {
    double r_desired[3];
    double v_desired[3];
    jeod::Quaternion q_desired;
    double w_desired[3];

    /** Actuator commands in guidance frame (allocator reads per routing). */
    double rcs_force_cmd_eci[3];
    double rcs_torque_cmd_body[3];
    double rw_torque_cmd_body[3];
    double mtq_dipole_cmd_body[3];
};

class Guidance {
public:
    double sc_pos_eci[3];
    double sc_vel_eci[3];
    /** Body angular rate (rad/s), same frame as MEKF/control — feed from vehicle each step. */
    double sc_omega_body[3];
    jeod::Quaternion q_eci_to_lvlh;
    jeod::Quaternion q_eci_to_ned;

    double target_pos[3];
    GuidanceFrame target_frame;

    GuidanceMode mode;

    double slew_time, slew_duration;
    jeod::Quaternion q_start, q_target;

    /** Used when mode == RCS_MANEUVER (ECI, N). */
    double maneuver_force_eci[3];

    /**
     * When true and mode == STATIONKEEP, publish orbital PD into out.rcs_force_cmd_eci
     * using hold_r_eci / hold_v_eci as setpoints (still fills r_desired / v_desired).
     */
    bool stationkeep_emit_rcs_pd;
    double hold_r_eci[3];
    double hold_v_eci[3];
    double stationkeep_kp;
    double stationkeep_kd;

    GuidanceOutput out;

    Guidance();

    void update(double dt);

private:
    void to_eci(const double in[3], double out[3], GuidanceFrame f);
    jeod::Quaternion compute_pointing(const double vec1[3], const double vec2[3]);
    jeod::Quaternion slerp(const jeod::Quaternion&, const jeod::Quaternion&, double);

    void idle();
    void target();
    void slew(double dt);
    void stationkeep();
    void rcs_maneuver();
};

}
