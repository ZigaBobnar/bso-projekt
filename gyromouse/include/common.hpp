#pragma once

#include <stdint.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "config.hpp"

// Board pin layout
#define BOARD_PIN_SPI_SCK       14
#define BOARD_PIN_SPI_MISO      12
#define BOARD_PIN_SPI_MOSI      13
#define BOARD_PIN_I2C_SCL       14
#define BOARD_PIN_I2C_SDA       12
#define BOARD_PIN_UART_TX       1
#define BOARD_PIN_UART_RX       3
#define BOARD_PIN_SYN115_TXD    4
#define BOARD_PIN_SYN470R_RXD   5
// #define BOARD_PIN_SYN470R_RXD   15
#define BOARD_PIN_NRF24_CS      0
#define BOARD_PIN_NRF24_CE      2 /* Over jumper J7, pulled high by default */
#define BOARD_PIN_NRF24_IRQ     4 /* Shared with TXD over unpopulated jumper J6 */

#define BOARD_I2C_BUS 0
#define BOARD_SPI_BUS 0

// Addresses
#define BOARD_I2C_ADDRESS_PCF8574   0x38
#define BOARD_I2C_ADDRESS_BMP280    0x76
#define BOARD_I2C_ADDRESS_MPU9255   0x68
#define BOARD_I2C_ADDRESS_AK8963    0x0C




// Hardware properties
#define BUTTONS_COUNT 4 /* All 4 buttons are on PCF8574 */

/* Row leds 0-3 are on PCF8574, module led 0 is the module led on pin 2 */
#define ROW_LEDS_COUNT 4
#define MODULE_LEDS_COUNT 1

#define LEDS_UPDATE_INTERVAL_DEFAULT 15
#define BUTTONS_UPDATE_INTERVAL_DEFAULT 30

// #define IMU_UPDATE_INTERVAL 10
// 50Hz ==> 20ms
#define IMU_UPDATE_INTERVAL_DEFAULT 20


#include "gyromouse.hpp"


// Tasks
void task_watchdog(void *pvParameters);

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


inline TickType_t get_time_delta(TickType_t start, TickType_t end) {
    return end - start;
}

inline TickType_t get_time_ms() {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}
