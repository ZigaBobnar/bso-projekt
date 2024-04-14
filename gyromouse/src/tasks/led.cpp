#include "common.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include <esp/gpio.h>

const int gpio = 2;

void task_led_blink(void *pvParameters) {
    gpio_enable(gpio, GPIO_OUTPUT);

    while(1) {
        gpio_write(gpio, 1);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        gpio_write(gpio, 0);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}
