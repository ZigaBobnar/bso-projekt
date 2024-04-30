#include "device/drivers/mpu925x.hpp"

#define INT16_FLOAT_SCALE 32768.0

#define MPU925x_FIFO_SIZE 512

uint8_t MPU925x_FIFO[MPU925x_FIFO_SIZE];
uint8_t *MPU925x_FIFO_ptr = MPU925x_FIFO;

MPU925x::MPU925x(I2CDevice& mpu_device, I2CDevice& ak_device) : mpu_device(mpu_device), ak_device(ak_device) {}

void MPU925x::init() {
    initialize_start();
    DEBUG_COMMAND("debug", "MPU925x::init[start]");

    mpu_device.get_bus().init();

    // Full reset
    write_register(MPU925x_Register::PWR_MGMT_1, 1 << 7);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    write_register(MPU925x_Register::PWR_MGMT_1, 0x00);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    write_register(MPU925x_Register::PWR_MGMT_1, 0x01);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    write_register(MPU925x_Register::CONFIG, 0x03); // DLPF_CFG = 3
    set_accelerometer_range(accel_range);
    set_gyro_range(gyro_range);

    write_register(MPU925x_Register::SMPLRT_DIV, 0x04);
    // Sample rate = Internal sample rate / (1 + SMPLRT_DIV)

    // Enable I2C bypass
    write_register(MPU925x_Register::INT_PIN_CFG, 0x02);

    // Initialize AK8963
    uint8_t ak_cntl2 = read_ak_register(AK8963_Register::CNTL2);
    write_ak_register(AK8963_Register::CNTL2, ak_cntl2 | (1 << 0)); // Reset module
    vTaskDelay(100 / portTICK_PERIOD_MS);
    write_ak_register(AK8963_Register::CNTL1, 0x16);

    DEBUG_COMMAND("debug", "MPU925x::init[end]");
    initialize_end();
}

uint8_t MPU925x::read_register(MPU925x_Register register_address) {
    return mpu_device.read_byte((uint8_t)register_address);
}

uint16_t MPU925x::read_word_registers(MPU925x_Register h_register_address) {
    uint8_t data[2];
    mpu_device.read_bytes((uint8_t)h_register_address, 2, data);
    
    return (uint16_t)data[0] << 8 | data[1];
}

void MPU925x::read_registers(MPU925x_Register start_register_address, uint8_t count, uint8_t* buffer_out) {
    mpu_device.read_bytes((uint8_t)start_register_address, count, buffer_out);
}

void MPU925x::write_register(MPU925x_Register register_address, uint8_t value) {
    mpu_device.write_byte((uint8_t)register_address, value);
}

uint8_t MPU925x::read_ak_register(AK8963_Register register_address) {
    return ak_device.read_byte((uint8_t)register_address);
}

void MPU925x::read_ak_registers(AK8963_Register start_register_address, uint8_t count, uint8_t* buffer_out) {
    ak_device.read_bytes((uint8_t)start_register_address, count, buffer_out);
}

void MPU925x::write_ak_register(AK8963_Register register_address, uint8_t value) {
    ak_device.write_byte((uint8_t)register_address, value);
}

uint8_t MPU925x::mpu_who_am_i() {
    return read_register(MPU925x_Register::WHO_AM_I);
}

uint8_t MPU925x::ak_who_am_i() {
    return read_ak_register(AK8963_Register::WIA);
}

void MPU925x::set_accelerometer_range(AccelerometerRange range) {
    accel_range = range;

    uint8_t accel_config = 0x00;
    accel_scale_factor = 1.0;
    switch (range) {
        case AccelerometerRange::Accel_2_G:
            accel_config = 0b00000000;
            accel_scale_factor = 2.0;
            break;
        case AccelerometerRange::Accel_4_G:
            accel_config = 0b00001000;
            accel_scale_factor = 4.0;
            break;
        case AccelerometerRange::Accel_8_G:
            accel_config = 0b00010000;
            accel_scale_factor = 8.0;
            break;
        case AccelerometerRange::Accel_16_G:
            accel_config = 0b00011000;
            accel_scale_factor = 16.0;
            break;
    }
    accel_scale_factor /= INT16_FLOAT_SCALE;

    write_register(MPU925x_Register::ACCEL_CONFIG, accel_config);
}

void MPU925x::set_gyro_range(GyroRange range) {
    gyro_range = range;

    uint8_t gyro_config = 0x00;
    gyro_scale_factor = 1.0;
    switch (range) {
        case GyroRange::Gyro_250_DegPerSec:
            gyro_config = 0b00000000;
            gyro_scale_factor = 250.0;
            break;
        case GyroRange::Gyro_500_DegPerSec:
            gyro_config = 0b00001000;
            gyro_scale_factor = 500.0;
            break;
        case GyroRange::Gyro_1000_DegPerSec:
            gyro_config =  0b00010000;
            gyro_scale_factor = 1000.0;
            break;
        case GyroRange::Gyro_2000_DegPerSec:
            gyro_config = 0b00011000;
            gyro_scale_factor = 2000.0;
            break;
    }
    gyro_scale_factor /= INT16_FLOAT_SCALE;

    write_register(MPU925x_Register::GYRO_CONFIG, gyro_config);
}

XYZFloat MPU925x::get_acceleration_values() {
    int16_t accel_x = (int16_t)read_word_registers(MPU925x_Register::ACCEL_XOUT_H);
    int16_t accel_y = (int16_t)read_word_registers(MPU925x_Register::ACCEL_YOUT_H);
    int16_t accel_z = (int16_t)read_word_registers(MPU925x_Register::ACCEL_ZOUT_H);

    return {
        (float)accel_x * accel_scale_factor,
        (float)accel_y * accel_scale_factor,
        (float)accel_z * accel_scale_factor
    };
}

XYZFloat MPU925x::get_gyroscope_values() {
    int16_t gyro_x = (int16_t)read_word_registers(MPU925x_Register::GYRO_XOUT_H);
    int16_t gyro_y = (int16_t)read_word_registers(MPU925x_Register::GYRO_YOUT_H);
    int16_t gyro_z = (int16_t)read_word_registers(MPU925x_Register::GYRO_ZOUT_H);

    return {
        (float)gyro_x * gyro_scale_factor,
        (float)gyro_y * gyro_scale_factor,
        (float)gyro_z * gyro_scale_factor
    };
}

XYZFloat MPU925x::get_magnetometer_values() {
    // Apparently we need to read 8 bytes from the module
    uint8_t mag_bytes[8];
    read_ak_registers(AK8963_Register::H_X_L, 8, mag_bytes);
    
    return {
        ((int16_t)(mag_bytes[0] << 8) | mag_bytes[1]),
        ((int16_t)(mag_bytes[2] << 8) | mag_bytes[3]),
        ((int16_t)(mag_bytes[4] << 8) | mag_bytes[5]),
    };
}

float MPU925x::get_temperature_value() {
    int16_t temperature = (int16_t)read_word_registers(MPU925x_Register::TEMP_OUT_H);

    // Magic constants
    static const float room_temperature_offset = 0;
    static const float temperature_sensitivity = 333.87;
    static const float room_temperature_celsius = 21;

    return ((float)temperature - room_temperature_offset) / temperature_sensitivity + room_temperature_celsius;
}
