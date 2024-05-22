#include "common.hpp"

#include "device/io.hpp"


// Only update the led values
void task_device_io_leds_update(void *pvParameters) {
    WRITE_COMMAND("debug", "task_device_io_leds_update[init]");

    while(true) {
        if (gyromouse.io_config.leds_update_interval_ms == 0) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        io.update_leds();
        vTaskDelay(pdMS_TO_TICKS(gyromouse.io_config.leds_update_interval_ms));
    }

    vTaskDelete(NULL);
}

void task_device_io_full_update(void *pvParameters) {
    TickType_t current_time = xTaskGetTickCount();
    
    WRITE_COMMAND("debug", "task_device_io_full_update[init]");
    io.init();

    WRITE_COMMAND("debug", "task_device_io_full_update[loop]");
    while(true) {
        if (gyromouse.io_config.buttons_update_interval_ms == 0) {
            vTaskDelayUntil(&current_time, pdMS_TO_TICKS(1000));
            continue;
        }

        io.update_full();

        vTaskDelayUntil(&current_time, pdMS_TO_TICKS(gyromouse.io_config.buttons_update_interval_ms));
    }

    vTaskDelete(NULL);
}
