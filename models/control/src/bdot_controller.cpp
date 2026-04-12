
#include "control_bdot/include/bdot_controller.hh"
namespace gnc {

BDotController::BDotController(double gain):k_(gain){}

void BDotController::compute(const double B[3], const double B_prev[3], double dt, double dipole[3]) {
    double diff[3];
    jeod::Vector3::diff(B, B_prev, diff);
    jeod::Vector3::scale(diff, 1.0/dt, dipole);
    jeod::Vector3::scale(dipole, -k_, dipole);
}

}
