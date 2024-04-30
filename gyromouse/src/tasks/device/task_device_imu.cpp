#include "common.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>

#include "device/imu.hpp"

bool imu_paused = false;

inline TickType_t get_time_delta(TickType_t start, TickType_t end) {
    return end - start;
}

inline TickType_t get_time_ms() {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void task_device_imu_update(void *pvParameters) {
    DEBUG_COMMAND("debug", "task_device_imu_update[init]");
    imu.init();
    WRITE_COMMAND("mpu_who_am_i", "0x%x", imu.mpu_who_am_i);
    WRITE_COMMAND("ak_who_am_i", "0x%x", imu.ak_who_am_i);
    // printf("$accel_test=%f,%f,%f\n", imu.accel_test.x, imu.accel_test.y, imu.accel_test.z);
    // printf("$gyro_test=%f,%f,%f\n", imu.gyro_test.x, imu.gyro_test.y, imu.gyro_test.z);

    TickType_t last_update_time = get_time_ms();

    DEBUG_COMMAND("debug", "task_device_imu_update[loop]");
    while (true) {
        if (imu_paused) {
            vTaskDelay(IMU_UPDATE_INTERVAL / portTICK_PERIOD_MS);
            continue;
        }
        
        TickType_t update_time = get_time_ms();
        imu.update();

        WRITE_COMMAND("update", "start");
        WRITE_COMMAND("dt", "%d", get_time_delta(last_update_time, update_time));
        WRITE_COMMAND("accel", "%f,%f,%f", imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z);
        WRITE_COMMAND("gyro", "%f,%f,%f", imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z);
        WRITE_COMMAND("mag", "%f,%f,%f", imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);
        WRITE_COMMAND("temp", "%f", imu.temperature);
        WRITE_COMMAND("update", "done");

        last_update_time = update_time;

        // Wait for next read
        vTaskDelay(IMU_UPDATE_INTERVAL / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
