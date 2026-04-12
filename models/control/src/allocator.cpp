
#include "../include/allocator.hh"

namespace gnc {

void Allocator::update(){
    wheel_cmd = torque;

    double B2 = B.dot(B);
    if(B2>1e-8)
        mtq_cmd = B.cross(torque)/B2;
    else
        mtq_cmd = jeod::Vector3(0,0,0);

    thruster_cmd = force;
}

}
