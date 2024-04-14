#include "device/i2c.hpp"

#include <esp/gpio.h>
#include <stdio.h>

I2CBus::I2CBus(uint8_t bus_id, uint8_t scl_pin_number, uint8_t sda_pin_number) : bus_id(bus_id), scl_pin(scl_pin_number), sda_pin(sda_pin_number) {
    bus_mutex = xSemaphoreCreateMutex();
    init();
}

void I2CBus::init() {
    lock();
    init_nolock();
    unlock();
}

void I2CBus::read(uint8_t address, const uint8_t* register_addresses, uint8_t* data_out_buffer, uint8_t length) {
    lock();
    read_nolock(address, register_addresses, data_out_buffer, length);
    unlock();
}

int I2CBus::write(uint8_t address, const uint8_t* register_addresses, const uint8_t* data_in_buffer, uint8_t length) {
    lock();
    int result = write_nolock(address, register_addresses, data_in_buffer, length);
    unlock();

    return result;
}

inline void I2CBus::lock() {
    xSemaphoreTake(bus_mutex, portMAX_DELAY);
}

inline void I2CBus::unlock() {
    xSemaphoreGive(bus_mutex);
}

inline void I2CBus::init_nolock() {
    i2c_init(bus_id, scl_pin, sda_pin, I2C_FREQ_100K);
    gpio_enable(SCL_PIN, GPIO_OUTPUT);
}

inline void I2CBus::read_nolock(uint8_t address, const uint8_t* register_addresses, uint8_t* data_out_buffer, uint8_t length) {
    i2c_slave_read(bus_id, address, register_addresses, data_out_buffer, length);
}

inline int I2CBus::write_nolock(uint8_t address, const uint8_t* register_addresses, const uint8_t* data_in_buffer, uint8_t length) {
    return i2c_slave_write(bus_id, address, register_addresses, data_in_buffer, length);
}


I2CDevice::I2CDevice(I2CBus& bus, uint8_t address) : bus(bus), address(address) {
    device_mutex = xSemaphoreCreateMutex();
}

uint8_t I2CDevice::read_byte(const uint8_t register_address) {
    uint8_t data;
    lock();
    bus.read(address, &register_address, &data, 1);
    unlock();

    return data;
}

uint16_t I2CDevice::read_word(const uint8_t high_register_address, const uint8_t low_register_address) {
    uint8_t data[2];
    lock();
    bus.lock();
    bus.read_nolock(address, &high_register_address, data, 1);
    bus.read_nolock(address, &low_register_address, data + 1, 1);
    bus.unlock();
    unlock();

    return (uint16_t)data[0] << 8 | data[1];
}

bool I2CDevice::write_byte(const uint8_t register_address, const uint8_t data) {
    lock();
    int result = bus.write(address, &register_address, &data, 1);
    unlock();

    return result == 0;
}

bool I2CDevice::write_byte(const uint8_t data) {
    lock();
    bool result = bus.write(address, NULL, &data, 1);
    unlock();

    return result == 0;
}

inline void I2CDevice::lock() {
    xSemaphoreTake(device_mutex, portMAX_DELAY);
}

inline void I2CDevice::unlock() {
    xSemaphoreGive(device_mutex);
}

I2CBus I2C_bus0(I2C_BUS, SCL_PIN, SDA_PIN);

I2CDevice I2C_pcf8574(I2C_bus0, 0x38);
I2CDevice I2C_bmp280(I2C_bus0, 0x76);
I2CDevice I2C_mpu9250(I2C_bus0, 0x68);
