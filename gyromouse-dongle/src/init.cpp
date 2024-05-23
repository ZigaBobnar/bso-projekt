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

#include "device/wireless.hpp"
#include "device/drivers/i2c.hpp"

#include "RF24/RF24.h"
#include <i2c/i2c.h>


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








RF24 radio(BOARD_PIN_NRF24_CE, BOARD_PIN_NRF24_CS);
#define rx_size		100
static char rx_data[rx_size];


const uint8_t address[] = { 0x01, 0x23, 0x45, 0x67, 0x89 };

void task_main_receive_data(void *pvArgs) {
    static TickType_t last_receive_time;

	printf("task_main_receive_data\n");

	i2c_init(BOARD_I2C_BUS, BOARD_PIN_I2C_SCL, BOARD_PIN_I2C_SDA, I2C_FREQ_100K);
    // Send data to turn off leds on pcf
    I2C_pcf8574.write_byte(0xff);

	gpio_enable(BOARD_PIN_SPI_SCK, GPIO_OUTPUT);
	gpio_enable(BOARD_PIN_NRF24_CS, GPIO_OUTPUT);

	radio.begin();
	radio.setChannel(36);


	radio.openReadingPipe(1, address);
	radio.startListening();



	WRITE_COMMAND("status", "starting receiving data");
	while (true) {

		if (radio.available()) {
			TickType_t receive_time = xTaskGetTickCount();

			uint8_t payload_size = radio.getDynamicPayloadSize();
			radio.read(&rx_data, payload_size);
			
			WRITE_COMMAND("data", "received %d bytes, dt=", payload_size, (receive_time - last_receive_time) * portTICK_PERIOD_MS);
			last_receive_time = receive_time;


			float mouse_x = 0;
			float mouse_y = 0;
			uint8_t buttons;

			// Parse the data
			mouse_x = (float)(((int16_t *)rx_data)[0]) / 1000.0f;
			mouse_y = (float)(((int16_t *)rx_data)[1]) / 1000.0f;
			buttons = rx_data[4];

			// This is sent for the computer driver
			WRITE_COMMAND("data", "mouse_x=%f, mouse_y=%f, buttons=%d", mouse_x, mouse_y, buttons);

			// Also set the LED values
        	gpio_write(BOARD_PIN_NRF24_CS, 1); // Disable nrf
        	i2c_init(BOARD_I2C_BUS, BOARD_PIN_I2C_SCL, BOARD_PIN_I2C_SDA, I2C_FREQ_100K);
        	I2C_pcf8574.write_byte(buttons >> 4);


			// Go back into listening mode
			radio.startListening();
		}

		// radio.powerDown();
		// vTaskDelay(50 / portTICK_PERIOD_MS);
		// radio.powerUp();
	}




	vTaskDelete(NULL);
}





/*
 * FreeRTOS Entrypoint
 */
void user_init(void) {
	// Set up UART
    uart_set_baud(0, 115200);

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

	// Data task
	if (xTaskCreate(
		task_main_receive_data,
		"main",
		2048,
		NULL,
		tskIDLE_PRIORITY + 8,
		NULL) != pdPASS)
		WRITE_COMMAND("error", "Failed to create main task");

	WRITE_COMMAND("state", "init_user_end");
}

#ifdef	__cplusplus
}
#endif
