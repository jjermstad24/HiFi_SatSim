
#include "../include/guidance.hh"
#include <cmath>

namespace gnc {

Guidance::Guidance() {
    mode = IDLE;
    slew_time = 0.0;
    slew_duration = 10.0;
}

void Guidance::to_eci(const double in[3], double out[3], GuidanceFrame f) {
    if (f == INERTIAL) {
        out[0] = in[0];
        out[1] = in[1];
        out[2] = in[2];
        return;
    }

    if (f == LVLH) {
        jeod::Quaternion q_lvlh_to_eci;
        q_eci_to_lvlh.conjugate(q_lvlh_to_eci);
        q_lvlh_to_eci.left_quat_transform(in, out);
        return;
    }

    if (f == NED) {
        jeod::Quaternion q_ned_to_eci;
        q_eci_to_ned.conjugate(q_ned_to_eci);
        q_ned_to_eci.left_quat_transform(in, out);
        return;
    }

    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
}

jeod::Quaternion Guidance::compute_pointing(const double sc[3], const double tgt[3]){
    double los[3];
    jeod::Vector3::decr(tgt,sc,los);
    jeod::Vector3::normalize(los);
    double z[3] = {0,0,1};
    double v[3];
    jeod::Vector3::cross(z,los,v);
    double s = sqrt((1+jeod::Vector3::dot(z,los))*2.0);
    jeod::Quaternion q;
    if(s<1e-6){ q.make_identity(); return q; }
    q.scalar = 0.5*s;
    q.vector[0]=v[0]/s; q.vector[1]=v[1]/s; q.vector[2]=v[2]/s;
    q.normalize();
    return q;
}

jeod::Quaternion Guidance::slerp(const jeod::Quaternion& q1,const jeod::Quaternion& q2,double t){
    jeod::Quaternion r;
    double dot = q1.scalar*q2.scalar +
        q1.vector[0]*q2.vector[0] +
        q1.vector[1]*q2.vector[1] +
        q1.vector[2]*q2.vector[2];
    double th = acos(dot);
    double s = sin(th);
    if(s<1e-6) return q1;
    double w1 = sin((1-t)*th)/s;
    double w2 = sin(t*th)/s;
    r.scalar = w1*q1.scalar + w2*q2.scalar;
    for(int i=0;i<3;i++)
        r.vector[i]=w1*q1.vector[i]+w2*q2.vector[i];
    r.normalize();
    return r;
}

void Guidance::idle(){
    out.q_desired.make_identity();
}

void Guidance::target(){
    double tgt[3];
    to_eci(target_pos,tgt,target_frame);
    out.q_desired = compute_pointing(sc_pos_eci,tgt);
}

void Guidance::slew(double dt){
    slew_time += dt;
    double t = slew_time/slew_duration;
    if(t>1) t=1;
    out.q_desired = slerp(q_start,q_target,t);
}

void Guidance::stationkeep(){
    jeod::Vector3::copy(sc_pos_eci, out.r_desired);
    jeod::Vector3::copy(sc_vel_eci, out.v_desired);
}

void Guidance::update(double dt){
    if(mode==IDLE) idle();
    else if(mode==TARGET) target();
    else if(mode==SLEW) slew(dt);
    else if(mode==STATIONKEEP) stationkeep();
}

}
