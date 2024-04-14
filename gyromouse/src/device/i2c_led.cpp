#include "common.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include "device/i2c.hpp"

#define led1 0xfe // 0b ???? ???0
#define led2 0xfd // 0b ???? ??0?
#define led3 0xfb // 0b ???? ?0??
#define led4 0xf7 // 0b ???? 0???

#define I2C_ADDRESS 0x38


// Counter mode (seems broken)
// void task_i2c_blink(void *pvParameters) {
//     uint8_t counter = 0;
//     // uint8_t led_values[] = {
//     //     led1,
//     //     led3,
//     //     led2,
//     //     led4,
//     // };
//     uint8_t led_count = 4;

//     while (true)
//     {
//         uint8_t led_state = 0x00;
//         for (uint8_t i = 0; i < led_count; i++)
//         {
//             if ((counter >> i) & 1)
//             {
//                 led_state |= 1 << i;
//             }
//         }

//         I2C_pcf8574.write_byte(led_state);

//         vTaskDelay(1000 / portTICK_PERIOD_MS);

//         counter++;
//         counter %= 2^led_count;
//     }

//     vTaskDelete(NULL);
// }

void task_i2c_blink(void *pvParameters) {
    uint8_t led_count = 4;
    bool led_states[led_count];
    
    while (true)
    {
        uint8_t write_command = 0xff;

        if (led_states[0]) {            
            if (led_states[1]) {
                if (led_states[2]) {
                    led_states[3] = !led_states[3];
                }
                led_states[2] = !led_states[2];

            }
            led_states[1] = !led_states[1];

        }
        led_states[0] = !led_states[0];

        for (uint8_t i = 0; i < led_count; i++)
        {
            if (led_states[i]) {
                write_command &= ~(1 << i);
            }
        }

        I2C_pcf8574.write_byte(write_command);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}