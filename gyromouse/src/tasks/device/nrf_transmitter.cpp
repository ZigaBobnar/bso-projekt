#include "common.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include "RF24/nRF24L01.h"
#include "RF24/RF24.h"
#include <esp/gpio.h>

#define BUS_I2C		0
#define SCL 		14
#define SDA 		12
#define channel		33

#define CE_NRF		3
#define CS_NRF		0
RF24 radio(CE_NRF, CS_NRF);

uint8_t address[] = { 0x01, 0x23, 0x45, 0x67, 0x89 };

void disable_nrf() {
    gpio_write(CE_NRF, 1);
}

// void task_nrf_transmitter(void *pvParameters) {
//     radio.begin();
//     radio.setChannel(0x60);
//     radio.setPALevel(RF24_PA_HIGH);
//     radio.openWritingPipe(address[1]);
//     radio.openReadingPipe(1, address[0]);
//     radio.startListening();

//     while (true) {
//         if (radio.available()) {
//             uint8_t data[32];
//             radio.read(data, sizeof(data));
//             printf("Received: %s\n", data);
//         }

//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }

//     vTaskDelete(NULL);
// }


void task_nrf_transmitter(void *pvParameters) {
    // gpio_enable(SCL, GPIO_OUTPUT);
    // gpio_enable(CS_NRF, GPIO_OUTPUT);

    // radio.begin();
    // radio.setChannel(channel);

    // radio.openWritingPipe(address);
    // radio.powerDown();


    // while (true) {
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);

    //     radio.powerUp();
    //     radio.stopListening();
    //     radio.write("hello", 6);
    //     radio.powerDown();

    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    vTaskDelete(NULL);
}
