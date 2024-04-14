#pragma once

#include <FreeRTOS.h>

// Tasks
void task_led_blink(void *pvParameters);
void task_i2c_blink(void *pvParameters);
void task_i2c_gyro(void *pvParameters);
