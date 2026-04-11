
#include "core_mode/include/mode_manager.hh"
namespace gnc {

Mode ModeManager::update(double omega_norm) {
    if(omega_norm < 0.01) mode_ = Mode::NOMINAL;
    return mode_;
}

}
