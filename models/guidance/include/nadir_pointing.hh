/*******************************************************************************
Purpose:
  ()

Library dependencies:
  ((../src/nadir_pointing.cpp))
*******************************************************************************/

#pragma once
#include "common/include/types.hh"

namespace gnc {

class NadirPointing {
public:
    AttitudeCommand compute(const double r[3], const double v[3]);
};

}
