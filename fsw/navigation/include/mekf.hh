/*******************************************************************************
Purpose:
  ()

Library dependencies:
  ((../src/mekf.cpp))
*******************************************************************************/

#pragma once
#include "common/include/types.hh"

namespace gnc {

class MEKF {
public:
    MEKF();
    void predict(const double gyro[3], double dt);
    void update_star_tracker(const jeod::Quaternionernion& q);
    State get_state() const;

private:
    jeod::Quaternionernion q_;
    double omega_[3];
    double bias_[3];
};

}
