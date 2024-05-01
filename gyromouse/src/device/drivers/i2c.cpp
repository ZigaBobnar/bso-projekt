#include "device/drivers/i2c.hpp"

#include <esp/gpio.h>
#include <stdio.h>

#include "device/io.hpp"

I2CBus::I2CBus(uint8_t bus_id, uint8_t scl_pin_number, uint8_t sda_pin_number) : bus_id(bus_id), scl_pin(scl_pin_number), sda_pin(sda_pin_number) {
    bus_mutex = xSemaphoreCreateMutex();
    init();
}

void I2CBus::init() {
    lock();

    // io_manager.lock(scl_pin);
    
    init_nolock();

    enabled = true;
    unlock();
}

bool I2CBus::is_enabled() {
    return enabled;
}

void I2CBus::disable() {
    lock();
    DEBUG_COMMAND("debug", "I2CBus::disable");
    enabled = false;
    // io_manager.unlock(scl_pin);
    unlock();
}

void I2CBus::read(uint8_t address, const uint8_t* register_addresses, uint8_t* data_out_buffer, uint8_t length) {
    if (!is_enabled()) {
        return;
    }

    lock();
    read_nolock(address, register_addresses, data_out_buffer, length);
    unlock();
}

int I2CBus::write(uint8_t address, const uint8_t* register_addresses, const uint8_t* data_in_buffer, uint8_t length) {
    if (!is_enabled()) {
        return -1;
    }

    lock();
    int result = write_nolock(address, register_addresses, data_in_buffer, length);
    unlock();

    return result;
}

inline void I2CBus::lock() {
    // i2c_spi_switch.reserve_i2c();
    xSemaphoreTake(bus_mutex, portMAX_DELAY);
}

inline void I2CBus::unlock() {
    xSemaphoreGive(bus_mutex);
    // i2c_spi_switch.release_i2c();
}

inline void I2CBus::init_nolock() {
    DEBUG_COMMAND("debug", "I2CBus::init[bus_id=%d, scl=%d, sda=%d]", bus_id, scl_pin, sda_pin);

    i2c_init(bus_id, scl_pin, sda_pin, I2C_FREQ_100K);
    gpio_enable(scl_pin, GPIO_OUTPUT);
}

inline void I2CBus::read_nolock(uint8_t address, const uint8_t* register_addresses, uint8_t* data_out_buffer, uint8_t length) {
    i2c_slave_read(bus_id, address, register_addresses, data_out_buffer, length);
}

inline int I2CBus::write_nolock(uint8_t address, const uint8_t* register_addresses, const uint8_t* data_in_buffer, uint8_t length) {
    return i2c_slave_write(bus_id, address, register_addresses, data_in_buffer, length);
}



I2CDeviceTransaction::I2CDeviceTransaction(I2CDevice& device) : device(device) {
    begin();
}

I2CDeviceTransaction::~I2CDeviceTransaction() {
    finish();
}

void I2CDeviceTransaction::begin() {
    if (!locked) {
        locked = true;
        device.lock();
        device.bus.lock();
    }
}

void I2CDeviceTransaction::finish() {
    if (locked) {
        locked = false;
        device.unlock();
        device.bus.unlock();
    }
}

uint8_t I2CDeviceTransaction::read_byte(const uint8_t register_address) {
    uint8_t data;
    begin();
    device.bus.read_nolock(device.address, &register_address, &data, 1);

    return data;
}

uint8_t I2CDeviceTransaction::read_byte() {
    return read_byte(0);
}

uint16_t I2CDeviceTransaction::read_word(const uint8_t high_register_address, const uint8_t low_register_address) {
    uint8_t data[2];
    begin();
    device.bus.read_nolock(device.address, &high_register_address, data, 1);
    device.bus.read_nolock(device.address, &low_register_address, data + 1, 1);

    return (uint16_t)data[0] << 8 | data[1];
}

void I2CDeviceTransaction::read_bytes(const uint8_t register_start_address, uint8_t count, uint8_t* buffer_out) {
    begin();
    device.bus.read(device.address, &register_start_address, buffer_out, count);
}

bool I2CDeviceTransaction::write_byte(const uint8_t register_address, const uint8_t data) {
    begin();
    int result = device.bus.write(device.address, &register_address, &data, 1);

    return result == 0;
}

bool I2CDeviceTransaction::write_byte(const uint8_t data) {
    begin();
    bool result = device.bus.write(device.address, NULL, &data, 1);
    
    return result == 0;
}

bool I2CDeviceTransaction::write_bytes(const uint8_t register_start_address, uint8_t count, const uint8_t* buffer_in) {
    begin();
    bool result = device.bus.write(device.address, &register_start_address, buffer_in, count);

    return result == 0;
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

uint8_t I2CDevice::read_byte() {
    return read_byte(0);
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

void I2CDevice::read_bytes(const uint8_t register_start_address, uint8_t count, uint8_t* buffer_out) {
    lock();
    bus.read(address, &register_start_address, buffer_out, count);
    unlock();
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

bool I2CDevice::write_bytes(const uint8_t register_start_address, uint8_t count, const uint8_t* buffer_in) {
    lock();
    bool result = bus.write(address, &register_start_address, buffer_in, count);
    unlock();

    return result == 0;
}

I2CDeviceTransaction I2CDevice::begin_transaction() {
    return I2CDeviceTransaction(*this);
}

inline void I2CDevice::lock() {
    // i2c_spi_switch.reserve_i2c();
    xSemaphoreTake(device_mutex, portMAX_DELAY);
}

inline void I2CDevice::unlock() {
    xSemaphoreGive(device_mutex);
    // i2c_spi_switch.release_i2c();
}

I2CBus I2C_bus0(I2C_BUS, SCL_PIN, SDA_PIN);

I2CDevice I2C_pcf8574(I2C_bus0, 0x38);
I2CDevice I2C_bmp280(I2C_bus0, 0x76);
I2CDevice I2C_mpu9255(I2C_bus0, 0x68);
I2CDevice I2C_ak8963(I2C_bus0, 0x0C);
