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
    double r_desired[3]; //!< trick_units(m)
    double v_desired[3]; //!< trick_units(m/s)
    jeod::Quaternion q_desired; //!< trick_units(--)
    double w_desired[3]; //!< trick_units(rad/s)

    /** Actuator commands in guidance frame (allocator reads per routing). */
    double rcs_force_cmd_eci[3]; //!< trick_units(N)
    double rcs_torque_cmd_body[3]; //!< trick_units(N*m)
    double rw_torque_cmd_body[3]; //!< trick_units(N*m)
    double mtq_dipole_cmd_body[3]; //!< trick_units(A*m^2)
};

class Guidance {
public:
    double sc_pos_eci[3]; //!< trick_units(m)
    double sc_vel_eci[3]; //!< trick_units(m/s)
    /** Body angular rate (rad/s), same frame as MEKF/control — feed from vehicle each step. */
    double sc_omega_body[3]; //!< trick_units(rad/s)
    jeod::Quaternion q_eci_to_lvlh; //!< trick_units(--)
    jeod::Quaternion q_eci_to_ned; //!< trick_units(--)

    double target_pos[3]; //!< trick_units(m)
    GuidanceFrame target_frame; //!< trick_units(--)

    GuidanceMode mode; //!< trick_units(--)

    double slew_time; //!< trick_units(s)
    double slew_duration; //!< trick_units(s)
    jeod::Quaternion q_start; //!< trick_units(--)
    jeod::Quaternion q_target; //!< trick_units(--)

    /** Used when mode == RCS_MANEUVER (ECI, N). */
    double maneuver_force_eci[3]; //!< trick_units(N)

    /**
     * When true and mode == STATIONKEEP, publish orbital PD into out.rcs_force_cmd_eci
     * using hold_r_eci / hold_v_eci as setpoints (still fills r_desired / v_desired).
     */
    bool stationkeep_emit_rcs_pd; //!< trick_units(--)
    double hold_r_eci[3]; //!< trick_units(m)
    double hold_v_eci[3]; //!< trick_units(m/s)
    double stationkeep_kp; //!< trick_units(--)
    double stationkeep_kd; //!< trick_units(--)

    GuidanceOutput out; //!< trick_units(--)

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
