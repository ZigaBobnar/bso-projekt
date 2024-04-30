#pragma once

#include "common.hpp"
#include "device/utils/xyzfloat.hpp"
#include "device/utils/initializable.hpp"

/**
 * IMU - Intertial measurment unit high level driver
 * Combines the accelerometer, gyroscope (and temperature) sensor
 */
// Use GY-91 module
// The module consists of:
// - BMP280 [address: 0x76] (barometer)
// - MPU-9250 [address: 0x68] (accelerometer, gyroscope, magnetometer)
//   https://ucilnica.fri.uni-lj.si/pluginfile.php/74518/mod_label/intro/RM-MPU-9250A-00-v1.6.pdf
class IMU : Initializable {
public:
    void init();
    void update();

    uint8_t mpu_who_am_i;
    uint8_t ak_who_am_i;
    
    XYZFloat accelerometer;
    XYZFloat gyroscope;
    XYZFloat magnetometer;
    float temperature;
};

extern IMU imu;
