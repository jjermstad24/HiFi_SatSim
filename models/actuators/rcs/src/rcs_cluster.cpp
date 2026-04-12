#include "../include/rcs_cluster.hh"

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
    for (int i = 0; i < num_thrusters; i++) {
        thrusters[i].update(dt);
    }
    accumulate_outputs();
}

void RCSCluster::accumulate_outputs()
{
    std::memset(total_force, 0, sizeof(total_force));
    std::memset(total_torque, 0, sizeof(total_torque));

    for (int i = 0; i < num_thrusters; i++) {
        for (int j = 0; j < 3; j++) {
            total_force[j] += thrusters[i].force_body[j];
            total_torque[j] += thrusters[i].torque_body[j];
        }
    }
}

} // namespace gnc
