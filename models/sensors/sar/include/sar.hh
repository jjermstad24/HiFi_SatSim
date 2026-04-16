/*******************************************************************************
Purpose:
  SAR sensor model

Library dependencies:
  ((../src/sar.cpp))
*******************************************************************************/
#pragma once

#include "utils/math/include/math_utils.hh"

#include "environment/planet/include/planet.hh"

namespace gnc {

class SAR {
public:
    // Sensor field of view (degrees)
    double az_beamwidth_deg;
    double el_beamwidth_deg;

    // Output footprint in ECEF frame
    // Array of 4 points (each is 3D) representing the corners of the main beam footprint
    double footprint_ecef[4][3];
    bool footprint_valid;

    SAR();

    void initialize();

    /**
     * Compute the ground footprint of the SAR sensor.
     * @param r_ecef Position of the satellite in ECEF frame (m)
     * @param T_lvlh_to_ecef Rotation matrix from LVLH to ECEF 
     * @param planet Pointer to the jeod::Planet (to get req, rpol)
     * 
     * We assume the SAR boresight is nominally aligned with the LVLH +Z axis (Nadir).
     */
    void update(const double r_ecef[3], const double T_lvlh_to_ecef[3][3], jeod::Planet* planet);

private:
    /**
     * Finds the intersection of a ray with an ellipsoid.
     * @param p Ray origin (m)
     * @param d Ray direction (unit vector)
     * @param req Equatorial radius (m)
     * @param rpol Polar radius (m)
     * @param intersection Output intersection point
     * @return true if intersection exists, false otherwise
     */
    bool intersect_ray_ellipsoid(const double p[3], const double d[3], double req, double rpol, double intersection[3]);
};

} // namespace gnc
