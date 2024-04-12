/* The classic "blink" example
 *
 * This sample code is in the public domain.
 */
#include <stdlib.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "esp8266.h"
#include "espressif/esp_common.h"
#include "esp/gpio.h"
#include <i2c/i2c.h>

#include "common.h"
#include "config.h"

#define I2C_BUS 0
#define SCL_PIN 14
#define SDA_PIN 12
#define led1 0xfe // 0b ???? ???0
#define led2 0xfd // 0b ???? ??0?
#define led3 0xfb // 0b ???? ?0??
#define led4 0xf7 // 0b ???? 0???

#define I2C_ADDRESS 0x38

const int gpio = 2;


void task_blink_i2c(void *ignore)
{
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

/* This task uses the high level GPIO API (esp_gpio.h) to blink an LED.
 *
 */
void blinkenTask(void *pvParameters)
{
    gpio_enable(gpio, GPIO_OUTPUT);
    while(1) {
        gpio_write(gpio, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_write(gpio, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


/* This task demonstrates an alternative way to use raw register
   operations to blink an LED.

   The step that sets the iomux register can't be automatically
   updated from the 'gpio' constant variable, so you need to change
   the line that sets IOMUX_GPIO2 if you change 'gpio'.

   There is no significant performance benefit to this way over the
   blinkenTask version, so it's probably better to use the blinkenTask
   version.

   NOTE: This task isn't enabled by default, see the commented out line in user_init.
*/
void blinkenRegisterTask(void *pvParameters)
{
    GPIO.ENABLE_OUT_SET = BIT(gpio);
    IOMUX_GPIO2 = IOMUX_GPIO2_FUNC_GPIO | IOMUX_PIN_OUTPUT_ENABLE; /* change this line if you change 'gpio' */
    while(1) {
        GPIO.OUT_SET = BIT(gpio);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        GPIO.OUT_CLEAR = BIT(gpio);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}



void task_ota_setup(void *pvParameters) {
}


#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "esp8266.h"
#include "config.h"
#include "ota-tftp.h"

void user_init(void) {
    uart_set_baud(0, 115200);
    
	// WiFi configuration
	struct sdk_station_config config = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD, };
	sdk_wifi_station_set_auto_connect(1);
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&config);
	sdk_wifi_station_connect();

	// OTA configuration
	ota_tftp_init_server(TFTP_PORT);

    xTaskCreate(blinkenTask, "blinkenTask", 256, NULL, 2, NULL);
    //xTaskCreate(blinkenRegisterTask, "blinkenRegisterTask", 256, NULL, 2, NULL);

    xTaskCreate(task_blink_i2c, "task_blink_i2c", 2048, NULL, 2, NULL);
    
    // xTaskCreate(task_ota_setup, "task_ota_setup", 2048, NULL, 2, NULL);
}
