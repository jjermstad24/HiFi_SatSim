#include "../include/rcs_cluster.hh"
namespace gnc {
RCSCluster::RCSCluster() { num_thrusters = 0; }
void RCSCluster::initialize() {
    for (int i=0;i<num_thrusters;i++) thrusters[i].initialize();
}
void RCSCluster::update(double dt) {
    build_allocation_matrix(); solve_allocation();
    apply_commands(dt); accumulate_outputs();
}
void RCSCluster::build_allocation_matrix() {
    for (int i=0;i<num_thrusters;i++) {
        double* d = thrusters[i].direction;
        double* r = thrusters[i].position;
        double F = thrusters[i].thrust_vac;
        allocation_matrix[0][i] = d[0]*F;
        allocation_matrix[1][i] = d[1]*F;
        allocation_matrix[2][i] = d[2]*F;
        allocation_matrix[3][i] = r[1]*(d[2]*F) - r[2]*(d[1]*F);
        allocation_matrix[4][i] = r[2]*(d[0]*F) - r[0]*(d[2]*F);
        allocation_matrix[5][i] = r[0]*(d[1]*F) - r[1]*(d[0]*F);
    }
}
void RCSCluster::solve_allocation() {
    double b[6] = {
        desired_force[0], desired_force[1], desired_force[2],
        desired_torque[0], desired_torque[1], desired_torque[2]
    };
    for (int i=0;i<num_thrusters;i++) commands[i]=0.0;
    for (int iter=0;iter<10;iter++) {
        for (int i=0;i<num_thrusters;i++) {
            double contribution=0.0;
            for (int j=0;j<6;j++) contribution += allocation_matrix[j][i]*b[j];
            commands[i] += 0.0001 * contribution;
            commands[i] = math::clamp(commands[i], 0.0, 1.0);
        }
    }
}
void RCSCluster::apply_commands(double dt) {
    for (int i=0;i<num_thrusters;i++) {
        thrusters[i].cmd = commands[i];
        thrusters[i].update(dt);
    }
}
void RCSCluster::accumulate_outputs() {
    std::memset(total_force,0,sizeof(total_force));
    std::memset(total_torque,0,sizeof(total_torque));
    for (int i=0;i<num_thrusters;i++) {
        for (int j=0;j<3;j++) {
            total_force[j]+=thrusters[i].force_body[j];
            total_torque[j]+=thrusters[i].torque_body[j];
        }
    }
}
}
