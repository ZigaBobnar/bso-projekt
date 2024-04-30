#include "device/imu.hpp"

#include <stdio.h>
#include "device/drivers/mpu925x.hpp"
#include "device/drivers/i2c.hpp"

MPU925x mpu(I2C_mpu9255, I2C_ak8963);

void IMU::init() {
    if (is_initialized()) {
        // Only initialize once.
        return;
    }

    initialize_start();
    
    DEBUG_COMMAND("debug", "IMU::init[start]");

    mpu.init();

    // Read who-am-i register
    mpu_who_am_i = mpu.mpu_who_am_i();
    const char* detected_mpu_chip = (mpu_who_am_i == mpu.MPU9250_WHOAMI_EXPECTED_VALUE) ? "MPU9250" : (mpu_who_am_i == mpu.MPU9255_WHOAMI_EXPECTED_VALUE) ? "MPU9255" : "unknown";
    DEBUG_COMMAND("debug", "IMU::init MPU925x: Received who am I: 0x%x, detected chip: %s", mpu_who_am_i, detected_mpu_chip);

    ak_who_am_i = mpu.ak_who_am_i();
    const char* detected_ak_chip = (ak_who_am_i == mpu.AK8963_WIA_EXPECTED_VALUE) ? "AK8963" : "unknown";
    DEBUG_COMMAND("debug", "IMU::init AK8963: Received who am I: 0x%x, detected chip: %s, expected default: 0x%x", ak_who_am_i, detected_ak_chip, mpu.AK8963_WIA_EXPECTED_VALUE);

    // Get the self test values
    // float accel_scale = mpu.get_accel_scale_factor();
    // float gyro_scale = mpu.get_gyro_scale_factor();
    // accel_test.x = mpu.read_register(MPU9250_REG_SELF_TEST_X_ACCEL) * accel_scale;
    // accel_test.y = mpu.read_register(MPU9250_REG_SELF_TEST_Y_ACCEL) * accel_scale;
    // accel_test.z = mpu.read_register(MPU9250_REG_SELF_TEST_Z_ACCEL) * accel_scale;
    // gyro_test.x = mpu.read_register(MPU9250_REG_SELF_TEST_X_GYRO) * gyro_scale;
    // gyro_test.y = mpu.read_register(MPU9250_REG_SELF_TEST_Y_GYRO) * gyro_scale;
    // gyro_test.z = mpu.read_register(MPU9250_REG_SELF_TEST_Z_GYRO) * gyro_scale;

    initialize_end();
    
    DEBUG_COMMAND("debug", "IMU::init[end]");
}

void IMU::update() {
    if (!is_initialized()) {
        return;
    }

    accelerometer = mpu.get_acceleration_values();
    gyroscope = mpu.get_gyroscope_values();
    magnetometer = mpu.get_magnetometer_values();
    temperature = mpu.get_temperature_value();
}

IMU imu;
