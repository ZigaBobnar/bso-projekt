#pragma once

#include <FreeRTOS.h>
#include <stdint.h>
#include <stdio.h>

#include "config.hpp"

// Hardware properties
#define BUTTONS_COUNT 4 /* All 4 buttons are on PCF8574 */

/* Row leds 0-3 are on PCF8574, module led 0 is the module led on pin 2 */
#define ROW_LEDS_COUNT 4
#define MODULE_LEDS_COUNT 1

#define LEDS_UPDATE_INTERVAL 15
#define BUTTONS_UPDATE_INTERVAL 30

#define IMU_UPDATE_INTERVAL 10


extern bool imu_paused;
extern bool debugging_enabled;


// Tasks
void task_device_io_leds_update(void *pvParameters);
void task_device_io_full_update(void *pvParameters);
void task_device_imu_update(void *pvParameters);

void task_led_blink_invert(void *pvParameters);
void task_led_blink_heartbeat(void *pvParameters);
void task_led_row_counter(void *pvParameters);
void task_buttons_toggle_leds(void *pvParameters);
void task_buttons_write_console(void *pvParameters);
void task_serial_commands(void *pvParameters);

void task_nrf_transmitter(void *pvParameters);
// void task_nrf_receiver(void *pvParameters);

// Common macros

#define COMMAND_FORMAT(command, value) "$" command "=" value
#define WRITE_COMMAND(command, value, ...) printf(COMMAND_FORMAT(command, value) "\n", ##__VA_ARGS__)
#define WRITE_COMMAND_PARTIAL(command, value, ...) printf(COMMAND_FORMAT(command, value) ";", ##__VA_ARGS__)

#ifdef DEBUG_PRINTING_ENABLED
#define DEBUG_PRINT(format, ...) if (debugging_enabled) printf(format, ##__VA_ARGS__)
#define DEBUG_PRINTLN(format, ...) if (debugging_enabled) printf(format "\n", ##__VA_ARGS__)
#define DEBUG_COMMAND(command, value, ...) if (debugging_enabled) WRITE_COMMAND(command, value, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(format, ...)
#define DEBUG_PRINTLN(format, ...)
#define DEBUG_COMMAND(command, value, ...)
#endif
