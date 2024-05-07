#include "device/drivers/comms_mux.hpp"

#include <esp/gpio.h>

I2CReservation::~I2CReservation() {
    comms_mux.unlock();
}

SPIReservation::~SPIReservation() {
    comms_mux.unlock();
}

CommsMux::CommsMux(UartParams uart_params, I2CParams i2c_params, SPIParams spi_params) : uart_params(uart_params), i2c_params(i2c_params), spi_params(spi_params) {
    protocol_mutex = xSemaphoreCreateMutex();

	uart_set_baud(uart_params.interface, uart_params.baud_rate);
    gpio_enable(i2c_params.scl_pin_number, GPIO_OUTPUT);
}

SerialReservation CommsMux::reserve_serial() {
    lock();

    if (current_protocol == CommsMux::EnabledProtocol::Serial) {
        // Skip the reconfiguration phase
        return SerialReservation();
    }

    if (serial_ready) {
        current_protocol = CommsMux::EnabledProtocol::Serial;
        return SerialReservation();
    }

    DEBUG_COMMAND("debug", "I2CSPIMux::reserve_serial - reconfiguring");

    current_protocol = CommsMux::EnabledProtocol::Serial;
    serial_ready = true;
    
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

    current_protocol = CommsMux::EnabledProtocol::I2C;
    i2c_ready = true;
    spi_ready = false;

    i2c_init(i2c_params.bus_id, i2c_params.scl_pin_number, i2c_params.sda_pin_number, i2c_params.frequency);

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

    current_protocol = CommsMux::EnabledProtocol::SPI;
    spi_ready = true;
    i2c_ready = false;

    return SPIReservation();
}

inline void CommsMux::lock() {
    xSemaphoreTake(protocol_mutex, portMAX_DELAY);
}

inline void CommsMux::unlock() {
    xSemaphoreGive(protocol_mutex);
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
