#pragma once

#include <FreeRTOS.h>

// Tasks
void task_blink_i2c(void *ignore);
void blinkenTask(void *pvParameters);
void blinkenRegisterTask(void *pvParameters);
void task_ota_setup(void *pvParameters);


