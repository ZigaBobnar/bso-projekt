#pragma once

#include "common.hpp"

// Adaptation of driver from: https://github.com/nRF24/RF24
// Reduced the code to bare minimum working sample
// Implemented waiting for interrupts to prevent unneccessary switching between I2C and SPI on same pins
// Uses reservations to get access to the bus

class NRF24 {
public:

    uint8_t ce_pin;
    uint8_t csn_pin;
    uint8_t irq_pin;

    uint8_t spi_rx_buffer[32+1] __attribute__ ((aligned (16)));
    uint8_t spi_tx_buffer[32+1] __attribute__ ((aligned (16)));

    uint8_t address_width;


    NRF24(uint8_t ce_pin, uint8_t csn_pin, uint8_t irq_pin, uint8_t address_width);

    bool init();
    void chip_select(bool select);
    void chip_enable(bool enable);
    void _handle_interrupt();
    void _check_receive_queue();
    void _handle_received_data();

    uint8_t read_register(uint8_t reg, uint8_t* buffer_out, uint8_t length);
    uint8_t read_register(uint8_t reg);
    uint8_t read_register_nolock(uint8_t reg, uint8_t* buffer_out, uint8_t length);
    uint8_t read_register_nolock(uint8_t reg);
    uint8_t write_register(uint8_t reg, const uint8_t* buffer_in, uint8_t length);
    uint8_t write_register(uint8_t reg, uint8_t value);
    uint8_t write_register_nolock(uint8_t reg, const uint8_t* buffer_in, uint8_t length);
    uint8_t write_register_nolock(uint8_t reg, uint8_t value);

    uint8_t write_payload(uint8_t* buffer_in, uint8_t length, const uint8_t write_type);
    uint8_t write_payload_nolock(uint8_t* buffer_in, uint8_t length, const uint8_t write_type);
    uint8_t read_payload(uint8_t* buffer_out, uint8_t length);
    uint8_t read_payload_nolock(uint8_t* buffer_out, uint8_t length);
    uint8_t get_received_payload_dynamic_size();

    void write(uint8_t* buffer_in, uint8_t length);
    void write(uint8_t* buffer_in, uint8_t length, bool is_multicast);

    void flush_rx();
    void flush_tx();
    void _send_command(uint8_t command);
    
    void start_listening();
    void stop_listening();

    void open_writing_pipe(const uint8_t* address);
    void open_reading_pipe(uint8_t pipe_number, const uint8_t* address);
    void close_reading_pipe(uint8_t pipe_number);

    void set_address_width(uint8_t width);
    void set_retries_config(uint8_t delay_times_250_us, uint8_t count);
    void set_channel(uint8_t channel);
    void set_auto_acknowledge(uint8_t pipe_number, bool enabled);
    void set_pa_level(uint8_t level);
    void set_data_rate(uint8_t rate);
    void set_crc_length(uint8_t length);
    void set_irq_mask_disable(bool tx_ok, bool tx_fail, bool rx_ready);

    bool need_to_check_receive_queue = false;
};

extern NRF24 nrf24;
