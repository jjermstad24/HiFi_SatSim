#include "../include/earth_magnetic_field.hh"
#include <cmath>

namespace gnc {

EarthMagneticField::EarthMagneticField() {}

void EarthMagneticField::compute(const double r_eci[3], double sim_time, double B_eci[3]) {
    double r_mag = std::sqrt(r_eci[0]*r_eci[0] + r_eci[1]*r_eci[1] + r_eci[2]*r_eci[2]);
    if (r_mag < 1e-3) return;
    
    double r_mag3 = r_mag * r_mag * r_mag;
    double r_mag5 = r_mag3 * r_mag * r_mag;

    // Approximate dipole direction in ECI (tilted and rotating)
    double tilt = 11.5 * M_PI / 180.0;
    double lon0 = -70.0 * M_PI / 180.0; // Reference longitude
    double earth_rate = 7.2921159e-5;
    double GST = lon0 + earth_rate * sim_time;

    double m_hat[3];
    m_hat[0] = std::sin(tilt) * std::cos(GST);
    m_hat[1] = std::sin(tilt) * std::sin(GST);
    m_hat[2] = -std::cos(tilt);

    double m[3] = {M * m_hat[0], M * m_hat[1], M * m_hat[2]};
    
    double m_dot_r = m[0]*r_eci[0] + m[1]*r_eci[1] + m[2]*r_eci[2];
    
    for(int i=0; i<3; i++) {
        B_eci[i] = mu0_4pi * (3.0 * m_dot_r * r_eci[i] / r_mag5 - m[i] / r_mag3);
    }
}

}
