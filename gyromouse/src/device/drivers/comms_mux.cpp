#include "device/drivers/comms_mux.hpp"

#include <esp/gpio.h>
#include <esp/spi.h>

I2CReservation::~I2CReservation() {
    comms_mux.unlock();
}

SPIReservation::~SPIReservation() {
    comms_mux.unlock();
}

CommsMux::CommsMux(UartParams uart_params, I2CParams i2c_params, SPIParams spi_params) : uart_params(uart_params), i2c_params(i2c_params), spi_params(spi_params) {
    protocol_mutex = xSemaphoreCreateMutex();

    initialize_serial();
    gpio_enable(i2c_params.scl_pin_number, GPIO_OUTPUT);
}

void CommsMux::initialize_serial() {
	uart_set_baud(uart_params.interface, uart_params.baud_rate);
    serial_ready = true;
}

void CommsMux::initialize_i2c() {
    current_protocol = CommsMux::EnabledProtocol::I2C;
    i2c_ready = true;
    spi_ready = false;

    i2c_init(i2c_params.bus_id, i2c_params.scl_pin_number, i2c_params.sda_pin_number, i2c_params.frequency);
}

void CommsMux::initialize_spi() {
    current_protocol = CommsMux::EnabledProtocol::SPI;
    spi_ready = true;
    i2c_ready = false;
    
    spi_init(comms_mux.spi_params.bus_id, SPI_MODE0, SPI_FREQ_DIV_1M, 1, SPI_LITTLE_ENDIAN, false);
}

SerialReservation CommsMux::reserve_serial() {
    // Serial is a bit different than I2C and SPI as it does not share pins, this is only if we wanted to ensure data is sent over serial without interference from other tasks
    xSemaphoreTake(serial_mutex, portMAX_DELAY);

    return SerialReservation();
}

I2CReservation CommsMux::reserve_i2c() {
    lock();

    if (current_protocol == CommsMux::EnabledProtocol::I2C) {
        // Skip the reconfiguration phase
        return I2CReservation();
    }

    if (i2c_ready) {
        current_protocol = CommsMux::EnabledProtocol::I2C;
        return I2CReservation();
    }

    DEBUG_COMMAND("debug", "I2CSPIMux::reserve_i2c - reconfiguring");

    initialize_i2c();

    return I2CReservation();
}

SPIReservation CommsMux::reserve_spi() {
    lock();

    if (current_protocol == CommsMux::EnabledProtocol::SPI) {
        // Skip the reconfiguration phase
        return SPIReservation();
    }

    if (spi_ready) {
        current_protocol = CommsMux::EnabledProtocol::SPI;
        return SPIReservation();
    }

    DEBUG_COMMAND("debug", "I2CSPIMux::reserve_spi - reconfiguring");

    initialize_spi();

    return SPIReservation();
}

inline void CommsMux::lock() {
    xSemaphoreTake(protocol_mutex, portMAX_DELAY);
    is_in_use = true;
}


CommsMux comms_mux({
    .interface = 0,
    .baud_rate = 115200,
}, {
    .bus_id = BOARD_I2C_BUS,
    .scl_pin_number = BOARD_PIN_I2C_SCL,
    .sda_pin_number = BOARD_PIN_I2C_SDA,
    .frequency = I2C_FREQ_100K,
}, {
    .bus_id = BOARD_SPI_BUS,
    .sck_pin_number = BOARD_PIN_SPI_SCK,
    .mosi_pin_number = BOARD_PIN_SPI_MOSI,
    .miso_pin_number = BOARD_PIN_SPI_MISO,
});
