#include "../include/rw_cluster.hh"
#include "utils/math/include/math_utils.hh"

#include <cmath>
#include <cstring>

namespace gnc {

using namespace gnc::math;

RWCluster::RWCluster()
{
    num_wheels = 0;
}

void RWCluster::initialize()
{
    for (int i = 0; i < num_wheels; i++) {
        wheels[i].initialize();
    }
}

void RWCluster::update(double dt)
{
    build_matrix();
    solve_allocation();
    apply_commands(dt);
    accumulate();
}
void RWCluster::build_matrix()
{
    for (int i = 0; i < num_wheels; i++) {
        A[0][i] = wheels[i].axis[0];
        A[1][i] = wheels[i].axis[1];
        A[2][i] = wheels[i].axis[2];
    }
}
void RWCluster::solve_allocation()
{
    // Compute A * A^T (3x3)
    double ATA[3][3] = {0};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < num_wheels; k++) {
                ATA[i][j] += A[i][k] * A[j][k];
            }
        }
    }

    // NOTE: small system → hardcoded inverse would be better in flight code
    // For now: simple gradient approximation

    double tau[3] = {
        torque_cmd[0],
        torque_cmd[1],
        torque_cmd[2]
    };

    for (int i = 0; i < num_wheels; i++) {

        double projection =
            A[0][i]*tau[0] +
            A[1][i]*tau[1] +
            A[2][i]*tau[2];

        u[i] = clamp(-projection, -wheels[i].max_torque, wheels[i].max_torque);
    }
}
void RWCluster::apply_commands(double dt)
{
    for (int i = 0; i < num_wheels; i++) {
        wheels[i].torque_cmd = u[i];
        wheels[i].update(dt);
    }
}
void RWCluster::accumulate()
{
    std::memset(torque_body, 0, sizeof(torque_body));
    std::memset(total_momentum, 0, sizeof(total_momentum));

    for (int i = 0; i < num_wheels; i++) {

        torque_body[0] += wheels[i].torque_body[0];
        torque_body[1] += wheels[i].torque_body[1];
        torque_body[2] += wheels[i].torque_body[2];

        double h = wheels[i].momentum;

        total_momentum[0] += h * wheels[i].axis[0];
        total_momentum[1] += h * wheels[i].axis[1];
        total_momentum[2] += h * wheels[i].axis[2];
    }
}

}