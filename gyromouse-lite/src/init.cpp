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

#include "device/drivers/mpu925x.hpp"

#include "RF24/RF24.h"
#include <i2c/i2c.h>


#ifdef  __cplusplus
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

// Calibration variables
float baseline_gyro_z = 0;
float baseline_gyro_y = 0;

void calibrate_gyroscope() {
    printf("Calibrating gyroscope...\n");
    const int calibration_duration_ms = 1000;
    const int calibration_samples = calibration_duration_ms / 10; // assuming a 10ms delay between samples

    float sum_gyro_z = 0;
    float sum_gyro_y = 0;

    for (int i = 0; i < calibration_samples; ++i) {
        printf("Calibrating... %d/%d\n", i, calibration_samples);
        XYZFloat gyroscope = mpu.get_gyroscope_values();
        sum_gyro_z += gyroscope.z;
        sum_gyro_y += gyroscope.y;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    baseline_gyro_z = sum_gyro_z / calibration_samples;
    baseline_gyro_y = sum_gyro_y / calibration_samples;

    printf("Calibration complete: baseline_gyro_z = %f, baseline_gyro_y = %f\n", baseline_gyro_z, baseline_gyro_y);
}

void task_main_refresh_data_and_send(void *pvArgs) {
    static TickType_t last_wake_time;

    printf("task_main_refresh_data_and_send\n");

    mpu.init();

    // Send data to turn off leds on pcf
    I2C_pcf8574.write_byte(0xff);

    // Read who-am-i register
    uint8_t mpu_who_am_i = mpu.mpu_who_am_i();
    const char* detected_mpu_chip = (mpu_who_am_i == mpu.MPU9250_WHOAMI_EXPECTED_VALUE) ? "MPU9250" : (mpu_who_am_i == mpu.MPU9255_WHOAMI_EXPECTED_VALUE) ? "MPU9255" : "unknown";
    WRITE_COMMAND("debug", "IMU::init MPU925x: Received who am I: 0x%x, detected chip: %s", mpu_who_am_i, detected_mpu_chip);

    uint8_t ak_who_am_i = mpu.ak_who_am_i();
    const char* detected_ak_chip = (ak_who_am_i == mpu.AK8963_WIA_EXPECTED_VALUE) ? "AK8963" : "unknown";
    WRITE_COMMAND("debug", "IMU::init AK8963: Received who am I: 0x%x, detected chip: %s, expected default: 0x%x", ak_who_am_i, detected_ak_chip, mpu.AK8963_WIA_EXPECTED_VALUE);

    // Calibrate the gyroscope
    calibrate_gyroscope();

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

        XYZFloat gyroscope = mpu.get_gyroscope_values();

        uint8_t buttons_pcf = I2C_pcf8574.read_byte(0xff);
        WRITE_COMMAND("buttons", "%d", buttons_pcf);


        // Adjust readings by baseline (calibration)
        float adjusted_gyro_z = gyroscope.z - baseline_gyro_z;
        float adjusted_gyro_y = gyroscope.y - baseline_gyro_y;

        // Calculate kinematics of mouse
        int16_t mouse_deltas[2] = {1000, 1000};

        // Convert adjusted gyroscope readings to angles (assuming a simplified model for demonstration)
        float angle_z = adjusted_gyro_z; // Replace with appropriate conversion if necessary
        float angle_y = adjusted_gyro_y; // Replace with appropriate conversion if necessary

        // Map angles to 0-2000 range
        if (angle_z >= 0 && angle_z <= 90) {
            mouse_deltas[0] = (int16_t)(1000 + angle_z * (1000.0 / 90.0));
        } else if (angle_z < 0 && angle_z >= -90) {
            mouse_deltas[0] = (int16_t)(1000 + angle_z * (1000.0 / 90.0));
        }

        if (angle_y >= 0 && angle_y <= 90) {
            mouse_deltas[1] = (int16_t)(1000 + angle_y * (1000.0 / 90.0));
        } else if (angle_y < 0 && angle_y >= -90) {
            mouse_deltas[1] = (int16_t)(1000 + angle_y * (1000.0 / 90.0));
        }

        // Constrain values to the 0-2000 range
        mouse_deltas[0] = mouse_deltas[0] < 0 ? 0 : (mouse_deltas[0] > 2000 ? 2000 : mouse_deltas[0]);
        mouse_deltas[1] = mouse_deltas[1] < 0 ? 0 : (mouse_deltas[1] > 2000 ? 2000 : mouse_deltas[1]);

        WRITE_COMMAND("mouse_delta", "%d,%d", mouse_deltas[0], mouse_deltas[1]);


        uint8_t write_buffer[5];
        ((uint16_t*)write_buffer)[0] = mouse_deltas[0];
        ((uint16_t*)write_buffer)[1] = mouse_deltas[1];
        write_buffer[4] = buttons_pcf;

        WRITE_COMMAND("status", "sending data");

        radio.powerUp();
        radio.stopListening();
        radio.write(write_buffer, 5);
        radio.powerDown();


        vTaskDelayUntil(&last_wake_time, 10 / portTICK_PERIOD_MS);
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
        task_main_refresh_data_and_send,
        "main",
        2048,
        NULL,
        tskIDLE_PRIORITY + 8,
        NULL) != pdPASS)
        WRITE_COMMAND("error", "Failed to create main task");

    WRITE_COMMAND("state", "init_user_end");
}

#ifdef  __cplusplus
}
#endif