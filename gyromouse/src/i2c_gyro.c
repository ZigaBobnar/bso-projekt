#include "common.h"

#include <FreeRTOS.h>
#include <task.h>
#include <i2c/i2c.h>

// #include "src/lib/MPU9250/MPU9250RegisterMap.h"
// /home/ziga/FRI/MAG/BSO/projekt/gyromouse/src/lib/MPU9250/MPU9250/MPU9250RegisterMap.h
// #include "/home/ziga/FRI/MAG/BSO/projekt/gyromouse/src/lib/MPU9250/MPU9250/MPU9250RegisterMap.h"
// #include "MPU9250/MPU9250RegisterMap.h"
// #include "src/lib/MPU9250/MPU9250/MPU9250RegisterMap.h"
#include "MPU9250/MPU9250_registers.h"

#define I2C_BUS 0
#define SCL_PIN 14
#define SDA_PIN 12
// #define led1 0xfe // 0b ???? ???0
// #define led2 0xfd // 0b ???? ??0?
// #define led3 0xfb // 0b ???? ?0??
// #define led4 0xf7 // 0b ???? 0???

#define MPU_GYRO_I2C_ADDRESS 0x68
#define GYRO_READ_PERIOD_MS 500

uint8_t read_i2c_byte_register(uint8_t address, uint8_t register_address) {
    uint8_t data;
    i2c_slave_read(I2C_BUS, address, &register_address, &data, 1);

    return data;
}

uint8_t write_i2c_byte_register(uint8_t address, uint8_t register_address, uint8_t data) {
    i2c_slave_write(I2C_BUS, address, &register_address, &data, 1);

    return data;
}

// Use GY-91 module
// The module consists of:
// - BMP280 [address: 0x76] (barometer)
// - MPU-9250 [address: 0x68] (accelerometer, gyroscope, magnetometer)
//   https://ucilnica.fri.uni-lj.si/pluginfile.php/74518/mod_label/intro/RM-MPU-9250A-00-v1.6.pdf
//
// MPU9250 driver inspired by: https://github.com/hideakitai/MPU9250
void task_i2c_gyro(void *pvParameters) {
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);

    //
    // Check that we are able to talk with the module
    //

    // Read who-am-i register
    uint8_t who_am_i;
    read_i2c_byte_register(MPU_GYRO_I2C_ADDRESS, MPU9250_REG_WHO_AM_I);
    printf("MPU9250: Who am I: %d, expected default: 0x71\n", who_am_i);

    //
    // Initialize the module as gyroscope sensor
    //

    // Reset module
    write_i2c_byte_register(MPU_GYRO_I2C_ADDRESS, MPU9250_REG_PWR_MGMT0, 0x80);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Wake up
    write_i2c_byte_register(MPU_GYRO_I2C_ADDRESS, PWR_MGMT_1, 0x00);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Get stable time source
    write_i2c_byte_register(MPU_GYRO_I2C_ADDRESS, PWR_MGMT_1, 0x01);
    vTaskDelay(200 / portTICK_PERIOD_MS);


    // Set configuration (32kHz, 8kHz, 1kHz)
    write_i2c_byte_register(MPU_GYRO_I2C_ADDRESS, MPU_CONFIG, 0x03);

    // Set sample rate
    write_i2c_byte_register(MPU_GYRO_I2C_ADDRESS, SMPLRT_DIV, 0x04);

    // Set gyro range
    write_i2c_byte_register(MPU_GYRO_I2C_ADDRESS, GYRO_CONFIG, 0x18);






    while (true)
    {

        // uint8_t led_state = 0xff;
        // for (uint8_t i = 0; i < led_count; i++)
        // {
        //     if ((counter >> i) & 1)
        //     {
        //         led_state &= led_values[i];
        //     }
        // // }
        // uint8_t gyro[3];
        // uint8_t accel[3];
        // // uint8_t register_address = 0x3b;

        // i2c_slave_read(I2C_BUS, MPU_GYRO_I2C_ADDRESS, &register_address, &gyro[0], 1);
        // printf("Gyro X: %d\n", data);

        
        // // Wait for next read
        // vTaskDelay(GYRO_READ_PERIOD_MS / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
