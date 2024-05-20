#include <FreeRTOS.h>
#include <task.h>

#include <espressif/esp_wifi.h>
#ifdef ENABLE_WIFI
	#include <espressif/esp_sta.h>
	#ifdef ENABLE_OTA
		#include <ota-tftp.h>
	#endif
#endif

#include <esp/uart.h>
#include <string.h>

#include "common.hpp"
#include "config.hpp"

#include "device/drivers/ask_syn.hpp"
#include "device/wireless_mouse.hpp"


#ifdef	__cplusplus
extern "C" {
#endif


#ifdef ENABLE_WIFI
void init_wifi() {
	struct sdk_station_config config;
	strncpy((char*)config.ssid, WIFI_SSID, 32);
	strncpy((char*)config.password, WIFI_PASSWORD, 64);
	
	sdk_wifi_station_set_auto_connect(1);
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&config);
	sdk_wifi_station_connect();
}
#endif


void task_ping(void *pvParameters) {
	static TickType_t current_time = 0;
	current_time = gyromouse.timings.last_ping_request = xTaskGetTickCount();

	while (true) {
		// TickType_t current_time = xTaskGetTickCount();

		float diff = ((float)current_time - gyromouse.timings.last_ping_request) * portTICK_PERIOD_MS/1000;

		WRITE_COMMAND("ping", "request(diff=%f)", diff);
		gyromouse.mode = GyroMouse::Mode::DONGLE;
		wireless_mouse.write_packet(WirelessMouseCommand::Ping, nullptr, 0);

		gyromouse.timings.last_ping_request = current_time;

		vTaskDelayUntil(&current_time, 1500 / portTICK_PERIOD_MS);
	}
}


/*
 * FreeRTOS Entrypoint
 */
void user_init(void) {
	// Set up UART
    uart_set_baud(0, 115200);
	WRITE_COMMAND("state", "hardware_init_start");
	WRITE_COMMAND("uart_baud", "115200");
	
	WRITE_COMMAND("state", "user_init");

	#ifdef ENABLE_WIFI
		// Set up WiFi from configuration
		init_wifi();
		WRITE_COMMAND("wifi", "initialized");

		#ifdef ENABLE_OTA
			// Set up OTA ftp
			ota_tftp_init_server(TFTP_PORT);
			WRITE_COMMAND("OTA", "initialized");
		#endif
	#else
		sdk_wifi_set_opmode(NULL_MODE);
	#endif




	// Initialize hardware into default state before starting any further tasks

	wireless_mouse.init(WIRELESS_DEFAULT_RADIO_CHANNEL);

	// ask_syn_transmitter.init();

	// ask_syn_receiver.init();
	// ask_syn_receiver.resume_interrupt();




	// Start the periodic tasks

	xTaskCreate(
		task_watchdog,
		"watchdog",
		256,
		NULL,
		tskIDLE_PRIORITY + 6, // Lower priority than the main tasks to see if everything is running
		&gyromouse.tasks.watchdog);

	xTaskCreate(
		task_device_imu_update,
		"device_imu_update",
		512,
		NULL,
		tskIDLE_PRIORITY + 2,
		&gyromouse.tasks.device_imu_update);

	xTaskCreate(task_device_io_leds_update, "io_leds_update", 256, NULL, 6, NULL);
	xTaskCreate(task_device_io_full_update, "io_full_update", 256, NULL, 6, NULL);
	// xTaskCreate(task_device_imu_update, "imu_update", 1024, NULL, 16, NULL);

    
	// xTaskCreate(task_led_blink_invert, "led_blink_invert", 256, NULL, 2, NULL);
    // xTaskCreate(task_led_blink_heartbeat, "led_blink_heartbeat", 256, NULL, 2, NULL);
    // xTaskCreate(task_led_row_counter, "led_row_counter", 256, NULL, 2, NULL);
	// xTaskCreate(task_buttons_toggle_leds, "buttons_toggle_leds", 256, NULL, 2, NULL);
	// xTaskCreate(task_buttons_write_console, "buttons_write_console", 256, NULL, 2, NULL);
	// xTaskCreate(task_serial_commands, "serial_commands", 512, NULL, 2, NULL);

    // xTaskCreate(task_nrf_transmitter, "nrf_transmitter", 256, NULL, 2, NULL);
    // xTaskCreate(task_nrf_receiver, "nrf_receiver", 256, NULL, 2, NULL);

	// xTaskCreate(task_ask_syn_receive_process, "ask_syn_receive_process", 256, NULL, 2, NULL);
	// xTaskCreate(task_ask_syn_transmit_process, "ask_syn_receive_process", 256, NULL, 2, NULL);

	xTaskCreate(
		task_wireless_mouse_process_data,
		"wireless_mouse_process_data",
		256,
		NULL,
		tskIDLE_PRIORITY + 2,
		&gyromouse.tasks.wireless_mouse_process_data);

	// Only in dongle mode:
	// xTaskCreate(task_ping, "ping", 256, NULL, 3, NULL);

	WRITE_COMMAND("state", "init_user_end");
}

#ifdef	__cplusplus
}
#endif
