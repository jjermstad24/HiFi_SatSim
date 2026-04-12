/*******************************************************************************
Purpose:
  Multi-thruster Reaction Wheel cluster with control allocation
Library dependencies:
  ((../src/rw_cluster.cpp))
*******************************************************************************/

#pragma once

#include "reaction_wheel.hh"

namespace gnc {

class RWCluster {
public:
    static const int MAX_WHEELS = 8;

    ReactionWheel wheels[MAX_WHEELS];
    int num_wheels;

    // Inputs
    double torque_cmd[3];

    // Outputs
    double torque_body[3];
    double total_momentum[3];

    // Internal
    double A[3][MAX_WHEELS];   // axis matrix
    double u[MAX_WHEELS];      // wheel torques

    RWCluster();

    void initialize();
    void update(double dt);

private:
    void build_matrix();
    void solve_allocation();
    void apply_commands(double dt);
    void accumulate();
};

}