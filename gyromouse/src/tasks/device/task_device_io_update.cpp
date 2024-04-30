#include "common.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include "device/io.hpp"


void task_device_io_leds_update(void *pvParameters) {
    while(1) {
        io.update_leds();
        vTaskDelay(LEDS_UPDATE_INTERVAL / portTICK_PERIOD_MS);
    }
}

void task_device_io_full_update(void *pvParameters) {
    DEBUG_COMMAND("debug", "task_device_io_full_update[init]");
    io.init();

    DEBUG_COMMAND("debug", "task_device_io_full_update[loop]");
    while(1) {
        io.update_full();
        vTaskDelay(BUTTONS_UPDATE_INTERVAL / portTICK_PERIOD_MS);
    }
}
