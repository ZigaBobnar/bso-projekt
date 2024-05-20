#pragma once

#include "common.hpp"
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <../components/RF24/RF24.h>

/*
 * Application cenrtric driver for nRF24 radio module.
 * The driver is packet based, so it can understand the lengths of different values
 * 
 * The shockburst of nRF24 has FIFO buffer for 32 bytes, so this should be our maximum length of full packet.
 * In reality we add redundancy with checksum to the end of data bytes and command header byte.
 * For checksum we use simple XOR-based solution of whole packet.
 * All this means we have 30 bytes left for the payload.
 * This is enough for 15x int16 = (5x (3x int16)), which means we could actually transmit all the required data in single packet.
 * 
 * Discard above, shockburst already calculates CRC values and adds it into its packet.
 * We only need to provid 1-32 bytes of payload and send that.
 */




/**
 * Signal lines for the module:
 * IRQ
 * CE
 * CSN
 * SCK
 * MOSI
 * MISO
 */

/**
 * SPI:
 * Maximum data rate: 8Mbps
 * Command structure: MSB first, 8 bits command
 * Data structure: LS byte first, MSB in each byte first
*/


// Default mode is to set address as mouse and go into waiting for dongle to send its announcment
// The addresses are 6 bytes long
const uint8_t WIRELESS_ADDRESS_WIDTH = 5;
// "GyroD" = 0x47 0x79 0x72 0x6F 0x44
// "GyroM" = 0x47 0x79 0x72 0x6F 0x4D
const uint8_t WIRELESS_DONGLE_ADDRESS[WIRELESS_ADDRESS_WIDTH+1] = "GyroD"; // This is the "dongle" connected to the computer
const uint8_t WIRELESS_MOUSE_ADDRESS[WIRELESS_ADDRESS_WIDTH+1]  = "GyroM"; // This is the "mouse" that sends the measured data
const uint8_t WIRELESS_DEFAULT_RADIO_CHANNEL = 32; // 0-125
const uint8_t PING_REQUEST_BYTE     = 0b10101010;
const uint8_t PING_RESPONSE_BYTE    = 0b01010101;




/*
 * Definitions for packet structure:
 *   Command: 1 byte
 *   Data: <variable>
 * 
 * The command can mean either request for data or data response.
 * Request must have zero length data.
 * 
 * All scale values data are in float format (32 bits, 4-byte)
 * Int16 values must be multiplied with scale to obtain real value.
 */
enum class WirelessCommand : uint8_t {
    Unknown = 0,                    // Not used, should indicate an error
    Ping = 1,                       // Response data: 1 byte = 0b10101010
    StartOfNewData = 3,             // Response data: 2 bytes (int16 time in us since the last transmissions)
    AnnounceAsDongle = 4,           // Data: 0 bytes (discard) - this is used to announce that the device is ready to receive new data (receiver is connected to the driver script on computer)
    AnnounceAsMouse = 5,            // Data: 0 bytes (discard) - this is used to announce that the device is ready to send new data

    Mouse_Move_Scale = 10,          // Response data: 4 bytes (float)
    Mouse_Move_Int16_X = 11,        // Response data: 2 bytes (int16)
    Mouse_Move_Int16_Y = 12,        // Response data: 2 bytes (int16)
    Mouse_Move_Int16_XY = 13,       // Response data: 4 bytes (2x int16)
    Mouse_Buttons_State = 14,       // Response data: 1 byte (contains bit masked button states), transmit on change

    Accelerometer_Scale = 20,       // Response data: 4 bytes (float)
    Accelerometer_Int16_X = 21,     // Response data: 2 bytes (int16)
    Accelerometer_Int16_Y = 22,     // Response data: 2 bytes (int16)
    Accelerometer_Int16_Z = 23,     // Response data: 2 bytes (int16)
    Accelerometer_Int16_XYZ = 24,   // Response data: 6 bytes (3x int16)

    Gyroscope_Scale = 30,           // Response data: 4 bytes (float)
    Gyroscope_Int16_X = 31,         // Response data: 2 bytes (int16)
    Gyroscope_Int16_Y = 32,         // Response data: 2 bytes (int16)
    Gyroscope_Int16_Z = 33,         // Response data: 2 bytes (int16)
    Gyroscope_Int16_XYZ = 34,       // Response data: 6 bytes (3x int16)

    Magnetometer_Scale = 40,        // Response data: 4 bytes (float)
    Magnetometer_Int16_X = 41,      // Response data: 2 bytes (int16)
    Magnetometer_Int16_Y = 42,      // Response data: 2 bytes (int16)
    Magnetometer_Int16_Z = 43,      // Response data: 2 bytes (int16)
    Magnetometer_Int16_XYZ = 44,    // Response data: 6 bytes (3x int16)

    Temperature_Scale = 50,         // Response data: 4 bytes (float)
    Temperature_Int16 = 51,         // Response data: 2 bytes (int16)
};
uint8_t get_wireless_command_data_length(WirelessCommand command);
WirelessCommand parse_wireless_command(uint8_t value);

class Wireless {
public:
    Wireless(const int spi_clock_pin, const int chip_select_pin, const int chip_enable_pin);

    /* Set the channel 0-125*/
    void init(const uint8_t channel);

    void write_packet(WirelessCommand command, const uint8_t* data, uint8_t length);

    void _process_data_task();

    void switch_to_dongle_mode();
    void switch_to_mouse_mode();

private:
    int spi_clock_pin;
    int chip_select_pin;
    int chip_enable_pin;
    uint8_t channel;

    RF24 radio;

    bool radio_available;
};

void task_wireless_process_data(void *pvParameters);
extern Wireless wireless;
