/*******************************************************************************
Purpose:
  Simple tilted dipole model for Earth's magnetic field.

Library dependencies:
  ((../src/earth_magnetic_field.cpp))
*******************************************************************************/
#pragma once

namespace gnc {

class EarthMagneticField {
public:
    EarthMagneticField();

    /**
     * Compute geomagnetic field in ECI frame.
     * @param r_eci Spacecraft position in ECI (meters)
     * @param sim_time Simulation time (seconds) - used for dipole rotation
     * @param B_eci Output magnetic field in ECI (Tesla)
     */
    void compute(const double r_eci[3], double sim_time, double B_eci[3]);

private:
    static constexpr double M = 7.746e22; // Earth's magnetic dipole moment (A*m^2)
    static constexpr double mu0_4pi = 1e-7; // mu0 / 4pi
};

}
