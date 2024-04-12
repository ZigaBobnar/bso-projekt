#include <FreeRTOS.h>
#include <task.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <ota-tftp.h>

#include "common.h"
#include "config.h"

void init_wifi() {
	struct sdk_station_config config = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD, };
	sdk_wifi_station_set_auto_connect(1);
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&config);
	sdk_wifi_station_connect();
}

/*
 * FreeRTOS Entrypoint
 */
void user_init(void) {
	// Set up UART
    uart_set_baud(0, 115200);

	// Set up WiFi from configuration
	// init_wifi();

	// Set up OTA ftp
	// ota_tftp_init_server(TFTP_PORT);

	// Start tasks
    xTaskCreate(task_led_blink, "led_blink", 256, NULL, 2, NULL);
    // xTaskCreate(task_i2c_blink, "i2c_blink", 256, NULL, 2, NULL);
    xTaskCreate(task_i2c_gyro, "i2c_gyro", 256, NULL, 2, NULL);
}
