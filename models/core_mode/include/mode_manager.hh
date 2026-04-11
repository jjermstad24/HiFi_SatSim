/*******************************************************************************
Purpose:
  ()

Library dependencies:
  ((../src/mode_manager.cpp))
*******************************************************************************/

#pragma once

namespace gnc {

enum class Mode {
    DETUMBLE,
    NOMINAL
};

class ModeManager {
public:
    Mode update(double omega_norm);
private:
    Mode mode_ = Mode::DETUMBLE;
};

}
