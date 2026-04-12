
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

enum GuidanceMode { IDLE=0, TARGET, SLEW, STATIONKEEP };
enum GuidanceFrame { INERTIAL=0, LVLH, NED };

struct GuidanceOutput {
    double r_desired[3];
    double v_desired[3];
    jeod::Quaternion q_desired;
    double w_desired[3];
};

class Guidance {
public:
    double sc_pos_eci[3];
    double sc_vel_eci[3];
    jeod::Quaternion q_eci_to_lvlh;
    jeod::Quaternion q_eci_to_ned;

    double target_pos[3];
    GuidanceFrame target_frame;

    GuidanceMode mode;

    double slew_time, slew_duration;
    jeod::Quaternion q_start, q_target;

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
};

}
