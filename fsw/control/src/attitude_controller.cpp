
#include "control/include/attitude_controller.hh"
namespace gnc {

AttitudeController::AttitudeController(double kp, double kd):kp_(kp),kd_(kd){}

void AttitudeController::compute(const State& s,const AttitudeCommand& c,double t[3]) {
    jeod::Quaternion qc = s.q;
    qc.conjugate();

    jeod::Quaternion qe;
    c.q_cmd.multiply(qc, qe);

    double e[3];
    jeod::Vector3::copy(qe.vector, e);

    double r[3];
    jeod::Vector3::diff(s.omega, c.omega_cmd, r);

    jeod::Vector3::scale(e, -kp_, t);
    jeod::Vector3::scale_incr(r, -kd_, t);
}

}
