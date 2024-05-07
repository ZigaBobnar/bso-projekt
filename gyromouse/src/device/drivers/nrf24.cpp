#include "device/drivers/nrf24.hpp"

#include "device/drivers/comms_mux.hpp"

NRF24::NRF24(const int spi_clock_pin, const int chip_select_pin, const int chip_enable_pin) :
    spi_clock_pin(spi_clock_pin), chip_select_pin(chip_select_pin), chip_enable_pin(chip_enable_pin), radio(chip_enable_pin, chip_select_pin) {
}

void NRF24::init(const int channel) {
    SPIReservation reservation = comms_mux.reserve_spi();
    
    this->channel = channel;

	gpio_enable(spi_clock_pin, GPIO_OUTPUT);
	gpio_enable(chip_select_pin, GPIO_OUTPUT);

    radio.begin();
    radio.setChannel(channel);
}


void NRF24::write_byte(uint8_t data) {
    SPIReservation reservation = comms_mux.reserve_spi();

    radio.write(&data, 1);
}


uint8_t NRF24::read_byte() {
    SPIReservation reservation = comms_mux.reserve_spi();

    uint8_t data;
    radio.read(&data, 1);
    return data;
}

NRF24::~NRF24() {
    SPIReservation reservation = comms_mux.reserve_spi();

    radio.powerDown();
}


void NRF24::pause_radio() {
    SPIReservation reservation = comms_mux.reserve_spi();

    radio.powerDown();
}


void NRF24::resume_radio() {
    SPIReservation reservation = comms_mux.reserve_spi();
    
    radio.powerUp();
}


NRF24 nrf24(BOARD_PIN_SPI_SCK, BOARD_PIN_NRF24_CS, BOARD_PIN_NRF24_CE);
