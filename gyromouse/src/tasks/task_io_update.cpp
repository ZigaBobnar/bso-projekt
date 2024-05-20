#include "common.hpp"

#include "device/io.hpp"

void task_device_io_leds_update(void *pvParameters) {
    while(true) {
        if (gyromouse.io_config.leds_update_interval_ms == 0) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        io.update_leds();
        vTaskDelay(gyromouse.io_config.leds_update_interval_ms / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}


void task_device_io_full_update(void *pvParameters) {
    TickType_t current_time = xTaskGetTickCount();
    
    DEBUG_COMMAND("debug", "task_device_io_full_update[init]");
    io.init();

    DEBUG_COMMAND("debug", "task_device_io_full_update[loop]");
    while(true) {
        if (gyromouse.io_config.buttons_update_interval_ms == 0) {
            vTaskDelayUntil(&current_time, 1000 / portTICK_PERIOD_MS);
            continue;
        }

        io.update_full();

        vTaskDelayUntil(&current_time, gyromouse.io_config.buttons_update_interval_ms / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
