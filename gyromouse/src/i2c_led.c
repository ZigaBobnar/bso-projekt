#include "common.h"

#include <FreeRTOS.h>
#include <task.h>
#include <i2c/i2c.h>

#define I2C_BUS 0
#define SCL_PIN 14
#define SDA_PIN 12
#define led1 0xfe // 0b ???? ???0
#define led2 0xfd // 0b ???? ??0?
#define led3 0xfb // 0b ???? ?0??
#define led4 0xf7 // 0b ???? 0???

#define I2C_ADDRESS 0x38


void task_i2c_blink(void *pvParameters) {
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);

    uint8_t counter = 0;
    uint8_t led_values[] = {
        led1,
        led3,
        led2,
        led4,
    };
    uint8_t led_count = 4;

    while (true)
    {
        uint8_t led_state = 0xff;
        for (uint8_t i = 0; i < led_count; i++)
        {
            if ((counter >> i) & 1)
            {
                led_state &= led_values[i];
            }
        }

        i2c_slave_write(I2C_BUS, I2C_ADDRESS, NULL, &led_state, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        counter++;
    }

    vTaskDelete(NULL);
}
