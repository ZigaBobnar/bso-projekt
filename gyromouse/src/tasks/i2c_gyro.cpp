#include "common.hpp"

#include <FreeRTOS.h>
#include <task.h>
// #include <i2c/i2c.h>
// #include <esp/gpio.h>
#include <stdio.h>

#include "MPU9250/MPU9250_registers.h"
#include "device/mpu9250.hpp"

#define GYRO_READ_PERIOD_MS 100


// Use GY-91 module
// The module consists of:
// - BMP280 [address: 0x76] (barometer)
// - MPU-9250 [address: 0x68] (accelerometer, gyroscope, magnetometer)
//   https://ucilnica.fri.uni-lj.si/pluginfile.php/74518/mod_label/intro/RM-MPU-9250A-00-v1.6.pdf
//
// MPU9250 driver inspired by: https://github.com/hideakitai/MPU9250
void task_i2c_gyro(void *pvParameters) {
    MPU9250 mpu(I2C_mpu9250);
    mpu.init();


    // Read who-am-i register
    uint8_t who_am_i = mpu.who_am_i();
    printf("MPU9250: Received who am I: %d, expected default: %d\n", who_am_i, MPU9250_WHOAMI_EXPECTED_VALUE);
    if (who_am_i != MPU9250_WHOAMI_EXPECTED_VALUE) {
        printf("MPU9250: Who am I register does not match expected value\n");
        // vTaskDelete(NULL);
        // return;
    }


    while (true) {
        MPU9250::XYZValue accel = mpu.get_acceleration();
        MPU9250::XYZValue gyro = mpu.get_gyro();
        float temperature = mpu.get_temperature();
        
        printf("%f %f %f %f %f %f %f\n", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, temperature);
        // printf("MPU9250: Accel: %d %d %d, Gyro: %d %d %d, Temp: %d\n", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temperature);

        // Wait for next read
        vTaskDelay(GYRO_READ_PERIOD_MS / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
