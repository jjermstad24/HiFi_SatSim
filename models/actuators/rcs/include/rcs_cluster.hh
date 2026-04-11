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
    static const int MAX_THRUSTERS = 32;
    RCSThruster thrusters[MAX_THRUSTERS];
    int num_thrusters;
    double desired_force[3], desired_torque[3];
    double total_force[3], total_torque[3];
    double allocation_matrix[6][MAX_THRUSTERS];
    double commands[MAX_THRUSTERS];
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
