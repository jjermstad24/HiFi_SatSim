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
    void update_star_tracker(const jeod::Quaternion& q);
    State get_state() const;

private:
    jeod::Quaternion q_; //!< trick_units(--)
    double omega_[3]; //!< trick_units(rad/s)
    double bias_[3]; //!< trick_units(rad/s)
};

}
