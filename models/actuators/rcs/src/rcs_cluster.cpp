#include "../include/rcs_cluster.hh"
#include <cstring>

namespace gnc {

RCSCluster::RCSCluster() {}

void RCSCluster::initialize()
{
    for (int i = 0; i < num_thrusters; i++) {
        thrusters[i].initialize();
    }
}

void RCSCluster::update(double dt)
{
    build_allocation_matrix();
    solve_allocation();
    apply_commands(dt);
    accumulate_outputs();
}

void RCSCluster::build_allocation_matrix()
{
    for (int i = 0; i < num_thrusters; i++) {

        double* d = thrusters[i].direction;
        double* r = thrusters[i].position;

        // =========================
        // FORCE (unit direction)
        // =========================
        allocation_matrix[0][i] = d[0];
        allocation_matrix[1][i] = d[1];
        allocation_matrix[2][i] = d[2];

        // =========================
        // TORQUE (r x F)
        // =========================
        allocation_matrix[3][i] = r[1]*d[2] - r[2]*d[1];
        allocation_matrix[4][i] = r[2]*d[0] - r[0]*d[2];
        allocation_matrix[5][i] = r[0]*d[1] - r[1]*d[0];
    }
}

void RCSCluster::solve_allocation()
{
    double b[6] = {
        desired_force[0], desired_force[1], desired_force[2],
        desired_torque[0], desired_torque[1], desired_torque[2]
    };

    for (int i = 0; i < num_thrusters; i++) {

        double num = 0.0;
        double den = 1e-6;

        for (int j = 0; j < 6; j++) {
            num += allocation_matrix[j][i] * b[j];
            den += allocation_matrix[j][i] * allocation_matrix[j][i];
        }

        double u = num / den;

        commands[i] = math::clamp(u, 0.0, 1.0);
    }
}

void RCSCluster::apply_commands(double dt)
{
    for (int i = 0; i < num_thrusters; i++) {
        thrusters[i].cmd = commands[i];
        thrusters[i].update(dt);
    }
}

void RCSCluster::accumulate_outputs()
{
    std::memset(total_force, 0, sizeof(total_force));
    std::memset(total_torque, 0, sizeof(total_torque));

    for (int i = 0; i < num_thrusters; i++) {
        for (int j = 0; j < 3; j++) {
            total_force[j]  += thrusters[i].force_body[j];
            total_torque[j] += thrusters[i].torque_body[j];
        }
    }
}

}