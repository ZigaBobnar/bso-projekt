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
#include "device/wireless.hpp"


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

void task_dongle_send_ping(void *pvParameters) {
	static TickType_t current_time = 0;
	current_time = gyromouse.timings.last_ping_request = xTaskGetTickCount();

	while (true) {
		float diff = ((float)current_time - gyromouse.timings.last_ping_request) * portTICK_PERIOD_MS/1000;

		if (gyromouse.mode == GyroMouse::Mode::DONGLE) {
			// Check if the last driver mode keepalive update was too long ago and disable dongle mode
			float keepalive_diff = ((float)current_time - gyromouse.timings.last_drive_mode_keepalive_message) * portTICK_PERIOD_MS/1000;
			static float last_check = 0;
			float last_check_diff = ((float)current_time - last_check) * portTICK_PERIOD_MS/1000;
			
			if (keepalive_diff > 1.0f && (last_check_diff > 1.0f || last_check_diff < 0.0f)) {
				// Send the request for mode keepalive every second.
				WRITE_COMMAND("dongle", "check");
				last_check = current_time;
			} else if (keepalive_diff > 5.0f && last_check_diff > 1.0f) {
				// Disable dongle mode after losing communication for more than 5 seconds.

				WRITE_COMMAND("error", "Driver mode keepalive message not received for too long");
				gyromouse.put_into_mouse_mode();
			}
		}

		if (gyromouse.mode == GyroMouse::Mode::DONGLE) {
			// Only in dongle mode:
			WRITE_COMMAND("ping", "request(diff=%f)", diff);
			gyromouse.mode = GyroMouse::Mode::DONGLE;
			wireless.write_packet(WirelessCommand::Ping, nullptr, 0);
			WRITE_COMMAND("ping", "was sent");

			gyromouse.timings.last_ping_request = current_time;
		}

		vTaskDelayUntil(&current_time, 1500 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
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

	gyromouse.init_all();

	// Start the periodic tasks

	if (xTaskCreate(
		task_watchdog,
		"watchdog",
		256,
		NULL,
		tskIDLE_PRIORITY + 1, // Lower priority than the main tasks to see if everything is running
		&gyromouse.tasks.watchdog) != pdPASS)
		WRITE_COMMAND("error", "Failed to create watchdog task");

	// xTaskCreate(
	// 	task_device_imu_update,
	// 	"device_imu_update",
	// 	512,
	// 	NULL,
	// 	tskIDLE_PRIORITY + 4,
	// 	&gyromouse.tasks.device_imu_update);

	if (xTaskCreate(
		task_device_io_leds_update,
		"io_leds_update",
		256,
		NULL,
		tskIDLE_PRIORITY + 5,
		NULL) != pdPASS)
		WRITE_COMMAND("error", "Failed to create leds update task");

	if (xTaskCreate(
		task_device_io_full_update,
		"io_full_update",
		256,
		NULL,
		tskIDLE_PRIORITY + 5,
		NULL) != pdPASS)
		WRITE_COMMAND("error", "Failed to create io update task");

	if (xTaskCreate(
		task_process_serial_commands,
		"process_serial_commands",
		512,
		NULL,
		tskIDLE_PRIORITY + 4,
		&gyromouse.tasks.process_serial_commands) != pdPASS)
		WRITE_COMMAND("error", "Failed to create process serial commands task");

	if (xTaskCreate(
		task_wireless_receive_data,
		"wireless_receive_data",
		512,
		NULL,
		tskIDLE_PRIORITY + 5,
		NULL) != pdPASS)
		WRITE_COMMAND("error", "Failed to create wireless receive data task");

	if (xTaskCreate(
		task_wireless_process_data,
		"wireless_process_data",
		1024,
		NULL,
		tskIDLE_PRIORITY + 2,
		&gyromouse.tasks.wireless_mouse_process_data) != pdPASS)
		WRITE_COMMAND("error", "Failed to create wireless process data task");

	if (xTaskCreate(
		task_dongle_send_ping,
		"dongle_send_ping",
		1024,
		NULL,
		tskIDLE_PRIORITY + 3,
		&gyromouse.tasks.dongle_send_ping) != pdPASS)
		WRITE_COMMAND("error", "Failed to create ping task");

	WRITE_COMMAND("state", "init_user_end");
}

#ifdef	__cplusplus
}
#endif
