#pragma once

/*
 * Customization of I2C protocol driver to allow for mutex-based access to the bus and devices.
 * 
 * This is so multiple tasks cannot interfere with each other when they try to send data to the devices on same bus.
 */

#include "common.hpp"
#include <FreeRTOS.h>
#include <semphr.h>
#include <i2c/i2c.h>

#define I2C_BUS 0
#define SCL_PIN 14
#define SDA_PIN 12

class I2CDevice;

class I2CBus {
public:
    I2CBus(uint8_t bus_id, uint8_t scl_pin_number, uint8_t sda_pin_number);

    void init();
    bool is_enabled();
    void disable();

private:
    bool enabled = false;
    
    uint8_t bus_id;
    uint8_t scl_pin;
    uint8_t sda_pin;
    SemaphoreHandle_t bus_mutex;

    void read(uint8_t address, const uint8_t* register_addresses, uint8_t* data_out_buffer, uint8_t length);
    int write(uint8_t address, const uint8_t* register_addresses, const uint8_t* data_in_buffer, uint8_t length);

    void lock();
    void unlock();

    void init_nolock();
    void read_nolock(uint8_t address, const uint8_t* register_addresses, uint8_t* data_out_buffer, uint8_t length);
    int write_nolock(uint8_t address, const uint8_t* register_addresses, const uint8_t* data_in_buffer, uint8_t length);

    friend class I2CDevice;
};

class I2CDevice {
public:
    I2CDevice(I2CBus& bus, uint8_t address);

    inline I2CBus& get_bus() { return bus; }

    uint8_t read_byte();
    uint8_t read_byte(const uint8_t register_address);
    uint16_t read_word(const uint8_t high_register_address, const uint8_t low_register_address);
    void read_bytes(const uint8_t register_start_address, uint8_t count, uint8_t* buffer_out);

    bool write_byte(const uint8_t register_address, const uint8_t data);
    bool write_byte(const uint8_t data);

private:
    I2CBus& bus;
    uint8_t address;
    SemaphoreHandle_t device_mutex;
    
    void lock();
    void unlock();
};

extern I2CBus I2C_bus0;

extern I2CDevice I2C_pcf8574;
extern I2CDevice I2C_bmp280;
extern I2CDevice I2C_mpu9255;
extern I2CDevice I2C_ak8963;
