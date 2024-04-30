#include <FreeRTOS.h>
#include <task.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <esp/uart.h>
#include <ota-tftp.h>
#include <string.h>

#include "common.hpp"
#include "config.hpp"


// Default state, can be overriden with console commands
bool debugging_enabled = false;


#ifdef	__cplusplus
extern "C" {
#endif



void init_wifi() {
	struct sdk_station_config config;
	strncpy((char*)config.ssid, WIFI_SSID, 32);
	strncpy((char*)config.password, WIFI_PASSWORD, 64);
	
	sdk_wifi_station_set_auto_connect(1);
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&config);
	sdk_wifi_station_connect();
}

void device_hardware_init(void) {
	// Set up UART
    uart_set_baud(0, 115200);
	WRITE_COMMAND("state", "hardware_init_start");
	WRITE_COMMAND("uart_baud", "115200");
	
	// Set up WiFi from configuration
	// init_wifi();
	// printf("Initialized WiFi\n");
	sdk_wifi_set_opmode(NULL_MODE);

	// Set up OTA ftp
	// ota_tftp_init_server(TFTP_PORT);
	// printf("Initialized OTA FTP\n");

	// Start device drivers tasks
	xTaskCreate(task_device_io_leds_update, "io_leds_update", 256, NULL, 16, NULL);
	xTaskCreate(task_device_io_full_update, "io_full_update", 256, NULL, 16, NULL);
	xTaskCreate(task_device_imu_update, "imu_update", 512, NULL, 16, NULL);

	WRITE_COMMAND("state", "hardware_init_end");
}

/*
 * FreeRTOS Entrypoint
 */
void user_init(void) {
	device_hardware_init();

	WRITE_COMMAND("state", "init_user_start");
    
	// xTaskCreate(task_led_blink_invert, "led_blink_invert", 256, NULL, 2, NULL);
    // xTaskCreate(task_led_blink_heartbeat, "led_blink_heartbeat", 256, NULL, 2, NULL);
    // xTaskCreate(task_led_row_counter, "led_row_counter", 256, NULL, 2, NULL);
	xTaskCreate(task_buttons_toggle_leds, "buttons_toggle_leds", 256, NULL, 2, NULL);
	xTaskCreate(task_buttons_write_console, "buttons_write_console", 256, NULL, 2, NULL);
	xTaskCreate(task_serial_commands, "serial_commands", 512, NULL, 2, NULL);

    xTaskCreate(task_nrf_transmitter, "nrf_transmitter", 256, NULL, 2, NULL);
    // xTaskCreate(task_nrf_receiver, "nrf_receiver", 256, NULL, 2, NULL);

	WRITE_COMMAND("state", "init_user_end");
}

#ifdef	__cplusplus
}
#endif
