#include "common.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "device/io.hpp"

void task_buttons_write_console(void *pvParameters) {
    bool button_states[BUTTONS_COUNT] = { 0 };

    while(1) {
        for (int i = 0; i < BUTTONS_COUNT; i++) {
            bool new_state = io.get_button(i);
            if (button_states[i] != new_state) {
                button_states[i] = new_state;

                WRITE_COMMAND("button", "%d,%d", i, new_state);
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
