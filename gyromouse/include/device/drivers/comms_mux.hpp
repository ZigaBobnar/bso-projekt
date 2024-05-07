#pragma once

/*
 * Allows I2C and SPI peripherals to share the pins by reserving them for one protocol at a time.
 */

#include "common.hpp"
#include <FreeRTOS.h>
#include <semphr.h>
#include <esp/uart.h>
#include <i2c/i2c.h>


class I2CReservation {
public:
    ~I2CReservation();
};

class SPIReservation {
public:
    ~SPIReservation();
};

class SerialReservation {
public:
    ~SerialReservation();
};

class CommsMux {
public:
    enum class EnabledProtocol {
        Unknown,
        Serial,
        I2C,
        SPI,
    };

    struct UartParams {
        uint8_t interface;
        uint32_t baud_rate;
    };

    struct I2CParams {
        uint8_t bus_id;
        uint8_t scl_pin_number;
        uint8_t sda_pin_number;
        i2c_freq_t frequency;
    };

    struct SPIParams {
        uint8_t bus_id;
        uint8_t sck_pin_number;
        uint8_t mosi_pin_number;
        uint8_t miso_pin_number;
    };

    CommsMux(UartParams uart_params, I2CParams i2c_params, SPIParams spi_params);

    SerialReservation reserve_serial();
    I2CReservation reserve_i2c();
    SPIReservation reserve_spi();
private:
    SemaphoreHandle_t protocol_mutex;

    void lock();
    void unlock();

    EnabledProtocol current_protocol = EnabledProtocol::Unknown;

    bool serial_ready = false;
    bool i2c_ready = false;
    bool spi_ready = false;

    UartParams uart_params;
    I2CParams i2c_params;
    SPIParams spi_params;

    friend class SerialReservation;
    friend class I2CReservation;
    friend class SPIReservation;
};

extern CommsMux comms_mux;
