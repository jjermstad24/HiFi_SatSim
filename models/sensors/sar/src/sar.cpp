#include "sar.hh"
#include <cmath>

namespace gnc {

SAR::SAR() {
    az_beamwidth_deg = 5.0;
    el_beamwidth_deg = 5.0;
    footprint_valid = false;
    for (int i = 0; i < 4; i++) {
        footprint_ecef[i][0] = 0.0;
        footprint_ecef[i][1] = 0.0;
        footprint_ecef[i][2] = 0.0;
    }
}

void SAR::initialize() {
    // Initialization if necessary
}

void SAR::update(const double r_ecef[3], const double T_lvlh_to_ecef[3][3], jeod::Planet* planet) {
    if (!planet) {
        footprint_valid = false;
        return;
    }

    double req = planet->r_eq;
    double rpol = planet->r_pol;

    // Define 4 corner rays in the LVLH frame
    // Assuming SAR boresight is +Z (Nadir).
    double half_az = (az_beamwidth_deg / 2.0) * M_PI / 180.0;
    double half_el = (el_beamwidth_deg / 2.0) * M_PI / 180.0;
    
    double angles[4][2] = {
        {half_az, half_el},
        {half_az, -half_el},
        {-half_az, -half_el},
        {-half_az, half_el}
    };

    footprint_valid = true;

    for (int i = 0; i < 4; i++) {
        double az = angles[i][0];
        double el = angles[i][1];

        double v_lvlh[3];
        v_lvlh[0] = std::sin(az);
        v_lvlh[1] = std::sin(el);
        v_lvlh[2] = std::cos(az) * std::cos(el);

        math::normalize3(v_lvlh, v_lvlh);

        // Transform ray to ECEF (matrix * col vector)
        double v_ecef[3] = {0, 0, 0};
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                v_ecef[row] += T_lvlh_to_ecef[row][col] * v_lvlh[col];
            }
        }

        double intersect[3];
        bool hits = intersect_ray_ellipsoid(r_ecef, v_ecef, req, rpol, intersect);
        if (hits) {
            footprint_ecef[i][0] = intersect[0];
            footprint_ecef[i][1] = intersect[1];
            footprint_ecef[i][2] = intersect[2];
        } else {
            footprint_valid = false; // Footprint missed the Earth
            break;
        }
    }
}

bool SAR::intersect_ray_ellipsoid(const double p[3], const double d[3], double req, double rpol, double intersection[3]) {
    // Ellipsoid: x^2/req^2 + y^2/req^2 + z^2/rpol^2 = 1
    // Ray: P = p + lambda * d
    
    double a2 = req * req;
    double c2 = rpol * rpol;

    double A = (d[0] * d[0]) / a2 + (d[1] * d[1]) / a2 + (d[2] * d[2]) / c2;
    double B = 2.0 * ((p[0] * d[0]) / a2 + (p[1] * d[1]) / a2 + (p[2] * d[2]) / c2);
    double C = (p[0] * p[0]) / a2 + (p[1] * p[1]) / a2 + (p[2] * p[2]) / c2 - 1.0;

    double discriminant = B * B - 4.0 * A * C;

    if (discriminant < 0.0) {
        return false; // Missed the ellipsoid
    }

    double sqrt_disc = std::sqrt(discriminant);
    double lambda1 = (-B - sqrt_disc) / (2.0 * A);
    double lambda2 = (-B + sqrt_disc) / (2.0 * A);

    double lambda_hit = -1.0;
    if (lambda1 > 0) {
        lambda_hit = lambda1;
    } else if (lambda2 > 0) {
        lambda_hit = lambda2;
    }

    if (lambda_hit < 0) {
        return false;
    }

    intersection[0] = p[0] + lambda_hit * d[0];
    intersection[1] = p[1] + lambda_hit * d[1];
    intersection[2] = p[2] + lambda_hit * d[2];

    return true;
}

} // namespace gnc
