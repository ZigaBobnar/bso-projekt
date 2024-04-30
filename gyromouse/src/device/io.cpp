#include "device/io.hpp"

#include <esp/gpio.h>
#include "device/drivers/i2c.hpp"

/* Buttons have weird order and inverted logic */
#define BUTTON_0 0x20
#define BUTTON_1 0x10
#define BUTTON_2 0x80
#define BUTTON_3 0x40

#define MODULE_LED_0_PIN 2

void IO::init() {
    DEBUG_COMMAND("debug", "IO::init");
    
    I2C_pcf8574.write_byte(0xff);
    gpio_enable(MODULE_LED_0_PIN, GPIO_OUTPUT);
}

void IO::update_full() {
    _update_module_leds();
    _update_pcf();
}

void IO::update_leds() {
    if (module_led_state_changed) {
        _update_module_leds();
    }

    if (row_led_state_changed) {
        _update_pcf();
    }
}

void IO::_update_module_leds() {
    DEBUG_COMMAND("debug", "IO::_update_module_leds");
    
    module_led_states[0] = module_led_pending_states[0];
    gpio_write(MODULE_LED_0_PIN, !module_led_states[0]);
}

void IO::_update_pcf() {
    DEBUG_COMMAND("debug", "IO::_update_pcf");
    
    uint8_t leds_pcf = 0x00;
    for (int i = 0; i < ROW_LEDS_COUNT; i++) {
        leds_pcf |= (row_led_pending_states[i] << (i));
        row_led_states[i] = row_led_pending_states[i];
    }
   
    // The logic is flipped (0 is on, 1 is off)
    leds_pcf = ~leds_pcf;

    uint8_t buttons_pcf = I2C_pcf8574.read_byte(leds_pcf);
    // I2C_pcf8574.write_byte(leds_pcf);

    // uint8_t buttons_pcf = I2C_pcf8574.read_byte(NULL);
    // I2C_pcf8574.write_byte(leds_pcf);

    for (int i = 0; i < BUTTONS_COUNT; i++) {
        button_previous_states[i] = button_states[i];
    }

    button_states[0] = !(buttons_pcf & BUTTON_0);
    button_states[1] = !(buttons_pcf & BUTTON_1);
    button_states[2] = !(buttons_pcf & BUTTON_2);
    button_states[3] = !(buttons_pcf & BUTTON_3);
}

void IO::set_module_led(uint8_t index, bool state) {
    if (index > MODULE_LEDS_COUNT) {
        return;
    }

    if (state != module_led_pending_states[index]) {
        module_led_pending_states[index] = state;
        module_led_state_changed = true;
    }
}

void IO::set_row_led(uint8_t index, bool state) {
    if (index > ROW_LEDS_COUNT) {
        return;
    }

    if (state != row_led_pending_states[index]) {
        row_led_pending_states[index] = state;
        row_led_state_changed = true;
    }
}

IO io;
