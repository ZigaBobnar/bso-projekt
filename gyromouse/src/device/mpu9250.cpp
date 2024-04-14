#include "device/mpu9250.hpp"
#include "MPU9250/MPU9250_registers.h"

#define INT16_FLOAT_SCALE 32768.0

MPU9250::MPU9250(I2CDevice& i2c_device) : device(i2c_device) {}

void MPU9250::init() {
    device.get_bus().init();

    // Reset
    device.write_byte(MPU9250_REG_PWR_MGMT_1, 1 << 7);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    device.write_byte(MPU9250_REG_PWR_MGMT_1, 0x00);
    device.write_byte(MPU9250_REG_PWR_MGMT_1, 0x01);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    device.write_byte(MPU9250_REG_CONFIG, 0x03); // DLPF_CFG = 3
    set_accelerometer_range(accel_range);
    set_gyro_range(gyro_range);
    device.write_byte(MPU9250_REG_SMPLRT_DIV, 0x04);
}

uint8_t MPU9250::read_register(uint8_t register_address) {
    return device.read_byte(register_address);
}

uint16_t MPU9250::read_word_registers(uint8_t h_register_address, uint8_t l_register_address) {
    return device.read_word(h_register_address, l_register_address);
}

uint8_t MPU9250::who_am_i() {
    return device.read_byte(MPU9250_REG_WHO_AM_I);
}

void MPU9250::set_gyro_range(GyroRange range) {
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

    device.write_byte(MPU9250_REG_GYRO_CONFIG, gyro_config);
}

void MPU9250::set_accelerometer_range(AccelerometerRange range) {
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

    device.write_byte(MPU9250_REG_ACCEL_CONFIG, accel_config);
}

MPU9250::XYZValue MPU9250::get_acceleration() {
    int16_t accel_x = (int16_t)read_word_registers(MPU9250_REG_ACCEL_XOUT_H, MPU9250_REG_ACCEL_XOUT_L);
    int16_t accel_y = (int16_t)read_word_registers(MPU9250_REG_ACCEL_YOUT_H, MPU9250_REG_ACCEL_YOUT_L);
    int16_t accel_z = (int16_t)read_word_registers(MPU9250_REG_ACCEL_ZOUT_H, MPU9250_REG_ACCEL_ZOUT_L);

    return {
        (float)accel_x * accel_scale_factor,
        (float)accel_y * accel_scale_factor,
        (float)accel_z * accel_scale_factor
    };
}

MPU9250::XYZValue MPU9250::get_gyro() {
    int16_t gyro_x = (int16_t)read_word_registers(MPU9250_REG_GYRO_XOUT_H, MPU9250_REG_GYRO_XOUT_L);
    int16_t gyro_y = (int16_t)read_word_registers(MPU9250_REG_GYRO_YOUT_H, MPU9250_REG_GYRO_YOUT_L);
    int16_t gyro_z = (int16_t)read_word_registers(MPU9250_REG_GYRO_ZOUT_H, MPU9250_REG_GYRO_ZOUT_L);

    return {
        (float)gyro_x * gyro_scale_factor,
        (float)gyro_y * gyro_scale_factor,
        (float)gyro_z * gyro_scale_factor
    };
}

float MPU9250::get_temperature() {
    int16_t temperature = (int16_t)read_word_registers(MPU9250_REG_TEMP_OUT_H, MPU9250_REG_TEMP_OUT_L);

    room_temperature_offset = 0;
    temperature_sensitivity = 333.87;
    // temperature_sensitivity = 128;
    room_temperature_celsius = 23;
    // room_temperature_celsius = 25;

    return ((float)temperature - room_temperature_offset) / temperature_sensitivity + room_temperature_celsius;
}
