
#include "../include/allocator.hh"

#include "actuators/rcs/include/rcs_cluster.hh"
#include "utils/math/include/math_utils.hh"

#include <cmath>
#include <cstring>

namespace gnc {

namespace {

void cross3(const double a[3], const double b[3], double c[3])
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

double dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void rcs_build_allocation_matrix(const RCSCluster & rcs,
                                 double A[6][RCSCluster::num_thrusters])
{
    for (int i = 0; i < RCSCluster::num_thrusters; i++) {
        const double * d = rcs.thrusters[i].direction;
        const double * r = rcs.thrusters[i].position;
        A[0][i] = d[0];
        A[1][i] = d[1];
        A[2][i] = d[2];
        A[3][i] = r[1] * d[2] - r[2] * d[1];
        A[4][i] = r[2] * d[0] - r[0] * d[2];
        A[5][i] = r[0] * d[1] - r[1] * d[0];
    }
}

void rcs_allocate_thrusters(RCSCluster & rcs, const double f[3], const double tau[3])
{
    double A[6][RCSCluster::num_thrusters];
    rcs_build_allocation_matrix(rcs, A);
    const double b[6] = {f[0], f[1], f[2], tau[0], tau[1], tau[2]};
    for (int i = 0; i < RCSCluster::num_thrusters; i++) {
        double num = 0.0;
        double den = 1e-6;
        for (int j = 0; j < 6; j++) {
            num += A[j][i] * b[j];
            den += A[j][i] * A[j][i];
        }
        rcs.thrusters[i].cmd = math::clamp(num / den, 0.0, 1.0);
    }
}

void rcs_zero_thrusters(RCSCluster & rcs)
{
    for (int i = 0; i < RCSCluster::num_thrusters; i++) {
        rcs.thrusters[i].cmd = 0.0;
    }
}

} // namespace

Allocator::Allocator()
    : rw_torque_fraction(1.0),
      mtq_torque_fraction(0.0),
      rcs_torque_fraction(0.0),
      effector_enable_mask(GNC_EFFECTOR_ALL),
      rcs_cmd_source(GNC_CMD_FROM_CONTROL),
      rw_cmd_source(GNC_CMD_FROM_CONTROL),
      mtq_cmd_source(GNC_CMD_FROM_CONTROL)
{
    std::memset(torque_cmd_body, 0, sizeof(torque_cmd_body));
    std::memset(force_cmd_eci, 0, sizeof(force_cmd_eci));
    std::memset(B_body, 0, sizeof(B_body));
    std::memset(rcs_force_body_injection, 0, sizeof(rcs_force_body_injection));
    std::memset(rw_torque_cmd, 0, sizeof(rw_torque_cmd));
    std::memset(mtq_dipole_cmd, 0, sizeof(mtq_dipole_cmd));
    std::memset(rcs_force_body, 0, sizeof(rcs_force_body));
    std::memset(rcs_torque_body, 0, sizeof(rcs_torque_body));
}

void Allocator::apply_control_mode(unsigned effector_mask,
                                   GncActuatorCmdSource rcs_src,
                                   GncActuatorCmdSource rw_src,
                                   GncActuatorCmdSource mtq_src)
{
    effector_enable_mask = effector_mask;
    rcs_cmd_source = rcs_src;
    rw_cmd_source = rw_src;
    mtq_cmd_source = mtq_src;
}

void Allocator::update(const jeod::Quaternion & q_parent_this,
                       RCSCluster & rcs,
                       const GuidanceOutput & g_out)
{
    jeod::Quaternion q_eci_to_body;
    q_parent_this.conjugate(q_eci_to_body);

    if (rcs_cmd_source == GNC_CMD_FROM_GUIDANCE) {
        q_eci_to_body.left_quat_transform(g_out.rcs_force_cmd_eci, rcs_force_body);
    } else {
        q_eci_to_body.left_quat_transform(force_cmd_eci, rcs_force_body);
    }
    for (int i = 0; i < 3; i++) {
        rcs_force_body[i] += rcs_force_body_injection[i];
    }

    if (rcs_cmd_source == GNC_CMD_FROM_GUIDANCE) {
        std::memcpy(rcs_torque_body,
                    g_out.rcs_torque_cmd_body,
                    sizeof(rcs_torque_body));
    } else {
        for (int i = 0; i < 3; i++) {
            rcs_torque_body[i] = rcs_torque_fraction * torque_cmd_body[i];
        }
    }

    if (rw_cmd_source == GNC_CMD_FROM_GUIDANCE) {
        std::memcpy(rw_torque_cmd, g_out.rw_torque_cmd_body, sizeof(rw_torque_cmd));
    } else {
        for (int i = 0; i < 3; i++) {
            rw_torque_cmd[i] = rw_torque_fraction * torque_cmd_body[i];
        }
    }

    if (mtq_cmd_source == GNC_CMD_FROM_GUIDANCE) {
        std::memcpy(mtq_dipole_cmd, g_out.mtq_dipole_cmd_body, sizeof(mtq_dipole_cmd));
    } else {
        double tau_mtq[3] = {
            mtq_torque_fraction * torque_cmd_body[0],
            mtq_torque_fraction * torque_cmd_body[1],
            mtq_torque_fraction * torque_cmd_body[2]};

        const double B2 = dot3(B_body, B_body);
        if (B2 > 1e-18 && mtq_torque_fraction > 0.0) {
            double bxt[3];
            cross3(B_body, tau_mtq, bxt);
            const double inv = -1.0 / B2;
            mtq_dipole_cmd[0] = inv * bxt[0];
            mtq_dipole_cmd[1] = inv * bxt[1];
            mtq_dipole_cmd[2] = inv * bxt[2];
        } else {
            mtq_dipole_cmd[0] = 0.0;
            mtq_dipole_cmd[1] = 0.0;
            mtq_dipole_cmd[2] = 0.0;
        }
    }

    if ((effector_enable_mask & GNC_EFFECTOR_RW) == 0) {
        std::memset(rw_torque_cmd, 0, sizeof(rw_torque_cmd));
    }
    if ((effector_enable_mask & GNC_EFFECTOR_MTQ) == 0) {
        std::memset(mtq_dipole_cmd, 0, sizeof(mtq_dipole_cmd));
    }
    if ((effector_enable_mask & GNC_EFFECTOR_RCS) == 0) {
        rcs_zero_thrusters(rcs);
    } else {
        rcs_allocate_thrusters(rcs, rcs_force_body, rcs_torque_body);
    }
}

} // namespace gnc
