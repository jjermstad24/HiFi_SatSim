
#pragma once
#include "utils/math/include/vector3.hh"

namespace gnc {

class Allocator {
public:
    double torque[3];
    double force[3];
    double wheel_cmd[3];
    double mtq_cmd[3];
    double thruster_cmd[3];

    double B[3];

    void update();
};

}
