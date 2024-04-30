#pragma once

#include "common.hpp"
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

/*
 * Application cenrtric driver for nRF24 radio module.
 */


/**
 * Signal lines for the module:
 * IRQ
 * CE
 * CSN
 * SCK
 * MOSI
 * MISO
 */

/**
 * SPI:
 * Maximum data rate: 8Mbps
 * Command structure: MSB first, 8 bits command
 * Data structure: LS byte first, MSB in each byte first
*/

class RF24;

class NRF24 {
// public:
//     NRF24(uint8_t chip_enable_pin, uint8_t chip_select_pin, uint8_t spi_bus_id);
public:
    NRF24(const int spi_clock_pin, const int chip_select_pin, const int chip_enable_pin);

    void initialize_module(const int channel);

    // void write_byte(uint8_t data);

    // uint8_t read_byte();

    void pause_radio();

    void resume_radio();

private:
    int spi_clock_pin;
    int chip_select_pin;
    int chip_enable_pin;
    int channel;
    // uint8_t chip_enable_pin;
    // uint8_t chip_select_pin;
    // uint8_t spi_bus_id;

    RF24 radio;


    // void init();

    // enum class DataRate {
    //     // RATE_250KBPS,
    //     RATE_1MBPS,
    //     RATE_2MBPS,
    // };
    

    // enum RFCommand {
    //     R_REGISTER = 0b00000000, /* 0b000a aaaa, a=address, followed by 1-5 bytes */
    //     W_REGISTER = 0b00100000, /* 0b001a aaaa, followed by 1-5 bytes */
    //     R_RX_PAYLOAD = 0b01100001, /* followed by 1-32 bytes */
    //     W_TX_PAYLOAD = 0b10100000, /* followed by 1-32 bytes */
    //     FLUSH_TX = 0b11100001,
    //     FLUSH_RX = 0b11100010,
    //     REUSE_TX_PL = 0b11100011,
    //     ACTIVATE = 0b01010000,
    //     R_RX_PL_WID = 0b01100000,
    //     W_ACK_PAYLOAD = 0b10101000,
    //     W_TX_PAYLOAD_NO_ACK = 0b10110000,
    //     NOP = 0b11111111,
    // };

    // enum RFRegister {
    //     CONFIG = 0x00 /* Configuration register */,
    //     EN_AA = 0x01 /* Enable auto acknowledgment */,
    //     EN_RXADDR = 0x02 /* Enabled RX addresses */,
    //     SETUP_AW = 0x03 /* Setup of address widths */,
    //     SETUP_RETR = 0x04 /* Setup of automatic retransmission */,
    //     RF_CH = 0x05 /* RF channel */,
    //     RF_SETUP = 0x06 /* RF setup register */,
    //     STATUS = 0x07 /* Status register */,
    //     OBSERVE_TX = 0x08 /* Transmit observe register */,
    //     CD = 0x09,
    //     RX_ADDR_P0 = 0x0A /* Receive address*/,
    //     RX_ADDR_P1 = 0x0B,
    //     RX_ADDR_P2 = 0x0C,
    //     RX_ADDR_P3 = 0x0D,
    //     RX_ADDR_P4 = 0x0E,
    //     RX_ADDR_P5 = 0x0F,
    //     TX_ADDR = 0x10 /* Transmit address */,
    //     RX_PW_P0 = 0x11,
    //     RX_PW_P1 = 0x12,
    //     RX_PW_P2 = 0x13,
    //     RX_PW_P3 = 0x14,
    //     RX_PW_P4 = 0x15,
    //     RX_PW_P5 = 0x16,
    //     FIFO_STATUS = 0x17 /* FIFO status register */,
    //     DYNPD = 0x1C /* Dynamic payload length */,
    //     FEATURE = 0x1D /* Feature register */,
    // };

    // void setConfiguration(bool enableCRC) {
    //     uint8_t config = readRegister(RFRegister::CONFIG);
    //     config &= ~(1 << 3); // Clear bit 3
    //     config |= (enableCRC << 3); // Set bit 3
    //     writeRegister(RFRegister::CONFIG, config);
    // }

    // uint8_t readRegister(RFRegister reg) {
    //     uint8_t data[2] = {R_REGISTER | reg, 0};
    //     spi.write(data, 2);
    //     return data[1];
    // }

    // void writeRegister(RFRegister reg, uint8_t value) {
    //     uint8_t data[2] = {W_REGISTER | reg, value};
    //     spi.write(data, 2);
    // }

};
