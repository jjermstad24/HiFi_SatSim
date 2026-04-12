/*******************************************************************************
Purpose:
  Multi-thruster RCS cluster with control allocation
Library dependencies:
  ((../src/rcs_cluster.cpp))
*******************************************************************************/

#pragma once

#include "rcs_thruster.hh"
#include "utils/math/include/math_utils.hh"

#include <cmath>
#include <cstring>

namespace gnc {

class RCSCluster {
public:

    // HARD DESIGN CONSTANT (this IS the system size)
    static const int num_thrusters = 12;

    RCSThruster thrusters[num_thrusters];

    // Inputs
    double desired_force[3];
    double desired_torque[3];

    // Outputs
    double total_force[3];
    double total_torque[3];

    // Allocation
    double allocation_matrix[6][num_thrusters];
    double commands[num_thrusters];

    RCSCluster();

    void initialize();
    void update(double dt);

private:

    void build_allocation_matrix();
    void solve_allocation();
    void apply_commands(double dt);
    void accumulate_outputs();
};

}