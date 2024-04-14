#pragma once

#include "device/i2c.hpp"

class MPU9250 {
public:
    MPU9250(I2CDevice& i2c_device);

    enum class AccelerometerRange {
        Accel_2_G = 0x00,
        Accel_4_G = 0x01,
        Accel_8_G = 0x02,
        Accel_16_G = 0x03,
    };

    enum class GyroRange {
        Gyro_250_DegPerSec = 0x00,
        Gyro_500_DegPerSec = 0x01,
        Gyro_1000_DegPerSec = 0x02,
        Gyro_2000_DegPerSec = 0x03,
    };

    struct XYZValue {
        float x;
        float y;
        float z;
    };

    void init();

    uint8_t read_register(uint8_t register_address);
    uint16_t read_word_registers(uint8_t h_register_address, uint8_t l_register_address);

    uint8_t who_am_i();

    void set_accelerometer_range(AccelerometerRange range);
    void set_gyro_range(GyroRange range);

    XYZValue get_acceleration();
    XYZValue get_gyro();
    float get_temperature();

private:
    I2CDevice& device;

    AccelerometerRange accel_range = AccelerometerRange::Accel_2_G;
    float accel_scale_factor = 1.0f;
    GyroRange gyro_range = GyroRange::Gyro_250_DegPerSec;
    float gyro_scale_factor = 1.0f;

    float room_temperature_offset = 0;
    float temperature_sensitivity = 0;
    float room_temperature_celsius = 23;
};
