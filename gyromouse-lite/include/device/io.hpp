#pragma once

#include "common.hpp"

// This class handles everything related to the general IO (leds and buttons).
// It is updated periodically.
// The PCF8574 8-bit I/O expander is used
// https://www.ti.com/lit/ds/symlink/pcf8574.pdf
// https://ucilnica.fri.uni-lj.si/pluginfile.php/74518/mod_label/intro/PCF8574_PCF8574A.pdf
class IO {
public:
    void init();
    void update_full();
    void update_leds();

    void _update_module_leds();
    void _update_pcf();

    inline const bool get_module_led(const uint8_t index) const { return module_led_pending_states[index]; }
    void set_module_led(uint8_t index, bool state);
    inline void toggle_module_led(uint8_t index) { set_module_led(index, !get_module_led(index)); }

    inline const bool get_row_led(const uint8_t index) const { return row_led_pending_states[index]; }
    void set_row_led(uint8_t index, bool state);
    inline void toggle_row_led(uint8_t index) { set_row_led(index, !get_row_led(index)); }

    inline const bool get_button(const uint8_t index) const { return button_states[index]; }
    inline const bool button_changed(const uint8_t index) const { return button_states[index] != button_previous_states[index]; }

private:
    bool module_led_states[MODULE_LEDS_COUNT] = { 0 };
    bool module_led_pending_states[MODULE_LEDS_COUNT] = { 0 };
    bool module_led_state_changed = true;
    
    bool row_led_states[ROW_LEDS_COUNT] = { 0 };
    bool row_led_pending_states[ROW_LEDS_COUNT] = { 0 };
    bool row_led_state_changed = true;

    bool button_states[BUTTONS_COUNT] = { 0 };
    bool button_previous_states[BUTTONS_COUNT] = { 0 };

    void process_buttons_update(uint8_t buttons_pcf);
};

extern IO io;
