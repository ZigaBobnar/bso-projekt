#include <FreeRTOS.h>
#include <task.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <ota-tftp.h>
#include <string.h>

#include "common.hpp"
#include "config.hpp"

#ifdef	__cplusplus
extern "C" {
#endif

void init_wifi() {
	// struct sdk_station_config config = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD };
	struct sdk_station_config config;
	strncpy((char*)config.ssid, WIFI_SSID, 32);
	strncpy((char*)config.password, WIFI_PASSWORD, 64);
	
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
	printf("Initialized URAT at 115200 baud\n");

	// Set up WiFi from configuration
	// init_wifi();
	// printf("Initialized WiFi\n");

	// Set up OTA ftp
	// ota_tftp_init_server(TFTP_PORT);
	// printf("Initialized OTA FTP\n");

	// Start tasks
    size_t blink_task = xTaskCreate(task_led_blink, "led_blink", 256, NULL, 2, NULL);
	printf("Created led_blink task: 0x%x\n", blink_task);
    size_t i2c_blink_task = xTaskCreate(task_i2c_blink, "i2c_blink", 2048, NULL, 2, NULL);
	printf("Created i2c_blink task: 0x%x\n", i2c_blink_task);
    size_t i2c_gyro_task = xTaskCreate(task_i2c_gyro, "i2c_gyro", 256, NULL, 2, NULL);
	printf("Created i2c_gyro task: 0x%x\n", i2c_gyro_task);
}

#ifdef	__cplusplus
}
#endif
