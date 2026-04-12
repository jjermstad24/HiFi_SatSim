/*******************************************************************************
Purpose:
  Container for individual RCS thrusters; applies their commands and sums
  wrench at the vehicle interface. Per-thruster commands come from FSW (allocator).
Library dependencies:
  ((../src/rcs_cluster.cpp))
*******************************************************************************/

#pragma once

#include "rcs_thruster.hh"

#include <cstring>

namespace gnc {

class RCSCluster {
public:
    static const int num_thrusters = 12;

    RCSThruster thrusters[num_thrusters];

    /** Body-frame total from all thrusters (sim output / JEOD collection). */
    double total_force[3];
    double total_torque[3];

    RCSCluster();

    void initialize();
    void update(double dt);

private:
    void accumulate_outputs();
};

} // namespace gnc
