#include "device/nrf24.hpp"


NRF24::NRF24(const int spi_clock_pin, const int chip_select_pin, const int chip_enable_pin) :
    spi_clock_pin(spi_clock_pin), chip_select_pin(chip_select_pin), chip_enable_pin(chip_enable_pin), radio(chip_enable_pin, chip_select_pin) {
}

void NRF24::initialize_module(const int channel) {
    this->channel = channel;

	gpio_enable(spi_clock_pin, GPIO_OUTPUT);
	gpio_enable(chip_select_pin, GPIO_OUTPUT);

    radio.begin();
    radio.setChannel(channel);
}
