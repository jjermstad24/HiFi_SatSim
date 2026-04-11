
#include "guidance/include/nadir_pointing.hh"
namespace gnc {

AttitudeCommand NadirPointing::compute(const double r[3], const double v[3]) {

    double z[3];
    jeod::Vector3::normalize(r, z);
    jeod::Vector3::negate(z);

    double y[3];
    jeod::Vector3::cross(z, v, y);
    jeod::Vector3::normalize(y);

    double x[3];
    jeod::Vector3::cross(y, z, x);

    double R[3][3];
    for(int i=0;i<3;i++){
        R[i][0]=x[i]; R[i][1]=y[i]; R[i][2]=z[i];
    }

    AttitudeCommand cmd;
    cmd.q_cmd.left_quat_from_transformation(R);
    jeod::Vector3::initialize(cmd.omega_cmd);
    return cmd;
}

}
