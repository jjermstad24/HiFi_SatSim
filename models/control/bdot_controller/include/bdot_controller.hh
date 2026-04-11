/*******************************************************************************
Purpose:
  ()

Library dependencies:
  ((../src/bdot_controller.cpp))
*******************************************************************************/

#pragma once
#include "common/include/types.hh"

namespace gnc {

class BDotController {
public:
    BDotController(double gain);
    void compute(const double B[3], const double B_prev[3], double dt, double dipole[3]);
private:
    double k_;
};

}
