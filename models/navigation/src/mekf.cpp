
#include "navigation/include/mekf.hh"
namespace gnc {

MEKF::MEKF() {
    q_.make_identity();
    jeod::Vector3::initialize(omega_);
    jeod::Vector3::initialize(bias_);
}

void MEKF::predict(const double gyro[3], double dt) {
    double w[3];
    jeod::Vector3::diff(gyro, bias_, w);

    jeod::Quaternionernion dq;
    dq.scalar = 1.0;
    double v[3];
    jeod::Vector3::scale(w, 0.5*dt, v);
    jeod::Vector3::copy(v, dq.vector);

    q_.multiply(dq);
    q_.normalize();

    jeod::Vector3::copy(w, omega_);
}

void MEKF::update_star_tracker(const jeod::Quaternionernion& q_meas) {
    jeod::Quaternionernion qc = q_;
    qc.conjugate();

    jeod::Quaternionernion qe;
    q_meas.multiply(qc, qe);

    double corr[3];
    jeod::Vector3::scale(qe.vector, 0.1, corr);

    jeod::Quaternionernion dq;
    dq.scalar = 1.0;
    jeod::Vector3::copy(corr, dq.vector);

    dq.multiply(q_);
    q_ = dq;
    q_.normalize();
}

State MEKF::get_state() const {
    State s;
    s.q = q_;
    jeod::Vector3::copy(omega_, s.omega);
    jeod::Vector3::copy(bias_, s.gyro_bias);
    return s;
}

}
