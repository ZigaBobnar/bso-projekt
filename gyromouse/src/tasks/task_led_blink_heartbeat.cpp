#include "common.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include "device/io.hpp"

void task_led_blink_invert(void *pvParameters) {
    while(1) {
        io.toggle_module_led(0);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

// #define RATE 75
#define RATE 45
#define RR_INTERVAL 1000 * 60 / RATE
#define PQ 40
#define QR 30
#define R 120
#define RS 60
#define ST 50
#define T 15
#define FINAL_DELAY (RR_INTERVAL - PQ - QR - R - RS - ST - T)

void task_led_blink_heartbeat(void *pvParameters) {
    while(1) {
        io.set_module_led(0, 1);
        io.toggle_module_led(3);
        vTaskDelay(PQ / portTICK_PERIOD_MS);

        io.set_module_led(0, 0);
        io.toggle_module_led(3);
        vTaskDelay(QR / portTICK_PERIOD_MS);

        io.set_module_led(0, 1);
        io.toggle_module_led(3);
        vTaskDelay(R / portTICK_PERIOD_MS);

        io.set_module_led(0, 0);
        io.toggle_module_led(3);
        vTaskDelay(RS / portTICK_PERIOD_MS);
        vTaskDelay(ST / portTICK_PERIOD_MS);

        io.set_module_led(0, 1);
        io.toggle_module_led(3);
        vTaskDelay(T / portTICK_PERIOD_MS);

        io.set_module_led(0, 0);
        io.toggle_module_led(3);
        vTaskDelay(FINAL_DELAY / portTICK_PERIOD_MS);
    }
    
    vTaskDelete(NULL);
}

void task_led_row_counter(void *pvParameters) {
    while (true) {
        if (io.get_row_led(0)) {            
            if (io.get_row_led(1)) {
                if (io.get_row_led(2)) {
                    io.toggle_row_led(3);
                }
                io.toggle_row_led(2);
            }
            io.toggle_row_led(1);
        }
        io.toggle_row_led(0);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
