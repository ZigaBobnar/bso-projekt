#include "gyromouse.hpp"

void on_button_state_change(int button, bool state) {
    WRITE_COMMAND("button", "%d,%d", button, state);
}

// Global instance
GyroMouse gyromouse;
