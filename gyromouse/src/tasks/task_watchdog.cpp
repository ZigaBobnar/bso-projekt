#include "common.hpp"

void task_watchdog(void *pvParameters) {
    TickType_t current_time = gyromouse.timings.last_watchdog_check = xTaskGetTickCount();

	while (true) {
		WRITE_COMMAND("watchdog", "alive(diff=%f)", (current_time - gyromouse.timings.last_watchdog_check) * portTICK_PERIOD_MS);
		gyromouse.timings.last_watchdog_check = current_time;

		vTaskDelayUntil(&current_time, 5000 / portTICK_PERIOD_MS);
	}
}
