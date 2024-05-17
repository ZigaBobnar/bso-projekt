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

// Make task_handles available globally
struct TaskHandles task_handles;


// Default state, can be overriden with console commands
bool debugging_enabled = false;


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

void device_hardware_init(void) {
	// Set up UART
    uart_set_baud(0, 115200);
	WRITE_COMMAND("state", "hardware_init_start");
	WRITE_COMMAND("uart_baud", "115200");
	
	#ifdef ENABLE_WIFI
		// Set up WiFi from configuration
		init_wifi();
		printf("Initialized WiFi\n");

		#ifdef ENABLE_OTA
			// Set up OTA ftp
			ota_tftp_init_server(TFTP_PORT);
			printf("Initialized OTA FTP\n");
		#endif
	#else
		sdk_wifi_set_opmode(NULL_MODE);
	#endif

	wireless_mouse.init(WIRELESS_DEFAULT_RADIO_CHANNEL);

	// ask_syn_transmitter.init();

	// ask_syn_receiver.init();
	// ask_syn_receiver.resume_interrupt();

	// Start device drivers tasks
	// xTaskCreate(task_device_io_leds_update, "io_leds_update", 256, NULL, 16, NULL);
	// xTaskCreate(task_device_io_full_update, "io_full_update", 256, NULL, 16, NULL);
	// xTaskCreate(task_device_imu_update, "imu_update", 1024, NULL, 16, NULL);



	WRITE_COMMAND("state", "hardware_init_end");
}

void task_ask_syn_receive_process(void *pvParameters) {
	while (true) {
		while (ask_syn_receiver.has_data()) {
			uint8_t *data = ask_syn_receiver.get_from_queue();
			for (int i = 0; i < ASK_SYN_PACKET_LENGTH_BYTES; i++) {
				WRITE_COMMAND_PARTIAL("ask_rx_data", "[%d]=0x%x", i, (data[i]));
			}
			printf("\n");
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

// void task_ask_syn_transmit_process(void *pvParameters) {
// 	char data[17] = " Hello World!";

// 	data[0] = 0b01011010;
// 	data[1] = 0b10101010;
// 	data[2] = 0b10101010;
// 	data[3] = 0b10101010;
// 	data[4] = 0b10101010;
// 	data[5] = 0b10101010;
// 	data[6] = 0b10101010;
// 	data[7] = 0b10101010;
// 	data[8] = 0b10101010;
// 	data[9] = 0b10101010;
// 	data[10] = 0b10101010;
// 	data[11] = 0b10101010;
// 	data[12] = 0b10101010;
// 	data[13] = 0b10101010;
// 	data[14] = 0b10101010;
// 	data[15] = 0b10101010;
// 	data[16] = 0b10101010;

// 	while (true) {
// 		vTaskDelay(100 / portTICK_PERIOD_MS);
// 		if (!ask_syn_transmitter.tx_prefill_buffer_full) {
// 			WRITE_COMMAND("ask_tx_data", "Hello World!");
// 			ask_syn_transmitter.send_packet((uint8_t*)data, 4);
// 		}

// 		vTaskDelay(100 / portTICK_PERIOD_MS);
// 	}

// 	vTaskDelete(NULL);
// }



void task_wireless_mouse_process(void *pvParameters) {
	while (true) {
		// WRITE_COMMAND("state", "wireless_mouse_process");
		wireless_mouse.process_data();

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}


void task_watchdog(void *pvParameters) {
	static TickType_t run_timer = 0;
	static TickType_t prev_time = 0;
	run_timer = prev_time = xTaskGetTickCount();

	while (true) {
		TickType_t current_time = xTaskGetTickCount();

		float diff = ((float)current_time - prev_time) * portTICK_PERIOD_MS/1000;
		WRITE_COMMAND("watchdog", "alive(diff=%f)", diff);
		prev_time = current_time;

		vTaskDelayUntil(&run_timer, 5000 / portTICK_PERIOD_MS);
	}
}


void task_ping(void *pvParameters) {
	static TickType_t run_timer = 0;
	// static TickType_t prev_time = 0;
	run_timer = last_ping_request = xTaskGetTickCount();

	while (true) {
		TickType_t current_time = xTaskGetTickCount();

		float diff = ((float)current_time - last_ping_request) * portTICK_PERIOD_MS/1000;

		WRITE_COMMAND("ping", "request(diff=%f)", diff);
		wireless_mouse.switch_to_dongle_mode();
		wireless_mouse.write_packet(WirelessMouseCommand::Ping, nullptr, 0);

		last_ping_request = current_time;

		vTaskDelayUntil(&run_timer, 1500 / portTICK_PERIOD_MS);
	}
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
	// xTaskCreate(task_buttons_toggle_leds, "buttons_toggle_leds", 256, NULL, 2, NULL);
	// xTaskCreate(task_buttons_write_console, "buttons_write_console", 256, NULL, 2, NULL);
	// xTaskCreate(task_serial_commands, "serial_commands", 512, NULL, 2, NULL);

    // xTaskCreate(task_nrf_transmitter, "nrf_transmitter", 256, NULL, 2, NULL);
    // xTaskCreate(task_nrf_receiver, "nrf_receiver", 256, NULL, 2, NULL);

	// xTaskCreate(task_ask_syn_receive_process, "ask_syn_receive_process", 256, NULL, 2, NULL);
	// xTaskCreate(task_ask_syn_transmit_process, "ask_syn_receive_process", 256, NULL, 2, NULL);

	xTaskCreate(task_watchdog, "watchdog", 256, NULL, 3, NULL);
	// xTaskCreate(task_ping, "ping", 256, NULL, 3, NULL);
	xTaskCreate(task_wireless_mouse_process, "wireless_mouse_process", 256, NULL, 2, NULL);


	WRITE_COMMAND("state", "init_user_end");

}

#ifdef	__cplusplus
}
#endif
