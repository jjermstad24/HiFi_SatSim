
#include "core_system/include/gnc_system.hh"
namespace gnc {

GncSystem::GncSystem():ctrl_(0.1,0.01),bdot_(1e5){
    jeod::Vector3::initialize(B_prev_);
}

void GncSystem::step(const SensorData& s,const double r[3],const double v[3],double dt,ActuatorCommand& out){

    nav_.predict(s.gyro, dt);
    if(s.star_tracker_valid) nav_.update_star_tracker(s.star_tracker_q);

    State st = nav_.get_state();
    Mode m = mode_.update(jeod::Vector3::vmag(st.omega));

    if(m == Mode::DETUMBLE){
        bdot_.compute(s.mag, B_prev_, dt, out.mtq_dipole);
        jeod::Vector3::initialize(out.torque_cmd);
    } else {
        auto cmd = guidance_.compute(r,v);
        ctrl_.compute(st, cmd, out.torque_cmd);
        jeod::Vector3::initialize(out.mtq_dipole);
    }

    jeod::Vector3::copy(s.mag, B_prev_);
}

}
