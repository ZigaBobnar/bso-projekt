#include "gyromouse.hpp"
#include "device/wireless.hpp"
#include "device/io.hpp"

void GyroMouse::init_all() {
    io.init();
    
	// wireless.init(WIRELESS_DEFAULT_RADIO_CHANNEL);
}

void GyroMouse::on_button_state_change(int button, bool new_button_state) {
    WRITE_COMMAND("button", "%d,%d", button, new_button_state);

    // Indicate button press with LED
    io.set_row_led(button, new_button_state);
    
    // uint8_t buttons_state = 0;
    // for (int i = 0; i < BUTTONS_COUNT; i++) {
    //     buttons_state |= (io.get_button(i) << i);
    // }
    // wireless.write_packet(WirelessCommand::Mouse_Buttons_State, &buttons_state, 1);
}

void GyroMouse::put_into_dongle_mode() {
    timings.last_drive_mode_keepalive_message = xTaskGetTickCount();

    if (mode == Mode::DONGLE) {
        return;
    }
    
    wireless.switch_to_dongle_mode();
    mode = GyroMouse::Mode::DONGLE;
}

void GyroMouse::put_into_mouse_mode() {
    timings.last_drive_mode_keepalive_message = xTaskGetTickCount();
    
    if (mode == Mode::MOUSE) {
        return;
    }

    wireless.switch_to_mouse_mode();
    mode = GyroMouse::Mode::MOUSE;
}

// Global instance
GyroMouse gyromouse;
