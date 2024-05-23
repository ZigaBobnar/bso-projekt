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
#include "device/drivers/mpu925x.hpp"

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








MPU925x mpu(I2C_mpu9255, I2C_ak8963);
RF24 radio(BOARD_PIN_NRF24_CE, BOARD_PIN_NRF24_CS);


const uint8_t address[] = { 0x01, 0x23, 0x45, 0x67, 0x89 };

void task_main_refresh_data_and_send(void *pvArgs) {
	printf("task_main_refresh_data_and_send\n");

	mpu.init();

    // Read who-am-i register
    uint8_t mpu_who_am_i = mpu.mpu_who_am_i();
    const char* detected_mpu_chip = (mpu_who_am_i == mpu.MPU9250_WHOAMI_EXPECTED_VALUE) ? "MPU9250" : (mpu_who_am_i == mpu.MPU9255_WHOAMI_EXPECTED_VALUE) ? "MPU9255" : "unknown";
    WRITE_COMMAND("debug", "IMU::init MPU925x: Received who am I: 0x%x, detected chip: %s", mpu_who_am_i, detected_mpu_chip);

    uint8_t ak_who_am_i = mpu.ak_who_am_i();
    const char* detected_ak_chip = (ak_who_am_i == mpu.AK8963_WIA_EXPECTED_VALUE) ? "AK8963" : "unknown";
    WRITE_COMMAND("debug", "IMU::init AK8963: Received who am I: 0x%x, detected chip: %s, expected default: 0x%x", ak_who_am_i, detected_ak_chip, mpu.AK8963_WIA_EXPECTED_VALUE);

	gpio_enable(BOARD_PIN_I2C_SCL, GPIO_OUTPUT);

	gpio_enable(BOARD_PIN_SPI_SCK, GPIO_OUTPUT);
	gpio_enable(BOARD_PIN_NRF24_CS, GPIO_OUTPUT);

	radio.begin();
	radio.setChannel(36);


	radio.openWritingPipe(address);	
	radio.powerDown();






	while (true) {
		WRITE_COMMAND("status", "refreshing data");
	
		// Reinitialize bus
		gpio_write(BOARD_PIN_NRF24_CS, 1); // Disable nrf
		i2c_init(BOARD_I2C_BUS, BOARD_PIN_I2C_SCL, BOARD_PIN_I2C_SDA, I2C_FREQ_100K);

		
		// XYZFloat accelerometer = mpu.get_acceleration_values();
		XYZFloat gyroscope = mpu.get_gyroscope_values();
		// XYZMagnetometerReading magnetometer = mpu.get_magnetometer_values();
		// temperature = mpu.get_temperature_value();

        // Calculate kinematics of mouse
        int16_t mosue_deltas[2] = {0, 0};
        mosue_deltas[0] = - gyroscope.z * 1000;
        mosue_deltas[1] = - gyroscope.y * 1000;

		WRITE_COMMAND("mouse_delta", "%d,%d", mosue_deltas[0], mosue_deltas[1]);



		WRITE_COMMAND("status", "sending data");

		radio.powerUp();
		radio.stopListening();
		radio.write(mosue_deltas, sizeof(mosue_deltas));
		radio.powerDown();


		vTaskDelay(10 / portTICK_PERIOD_MS);
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

	gyromouse.init_all();

	// Data task
	if (xTaskCreate(
		task_main_refresh_data_and_send,
		"main",
		2048,
		NULL,
		tskIDLE_PRIORITY + 8,
		NULL) != pdPASS)
		WRITE_COMMAND("error", "Failed to create ping task");

	WRITE_COMMAND("state", "init_user_end");
}

#ifdef	__cplusplus
}
#endif
