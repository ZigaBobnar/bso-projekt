#include "device/drivers/nrf24.hpp"
#include "device/drivers/comms_mux.hpp"
#include "device/wireless.hpp"

#include <nRF24L01.h>

NRF24::NRF24(uint8_t ce_pin, uint8_t csn_pin, uint8_t irq_pin, uint16_t spi_speed, uint8_t address_width) :
    ce_pin(ce_pin),
    csn_pin(csn_pin),
    irq_pin(irq_pin),
    spi_speed(spi_speed),
    address_width(address_width),
    tx_delay(0),
    retries(0) {}

void nrf24_interrupt_handler(uint8_t gpio_num) {
    if (gpio_num == nrf24.irq_pin) {
        nrf24.handle_interrupt();
    }
}

bool NRF24::init() {
    gpio_enable(ce_pin, GPIO_OUTPUT);
    gpio_enable(csn_pin, GPIO_OUTPUT);
    chip_enable(false); // Disable at the beginning
    chip_select(false); // Deselect

    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for the chip to power up

    // SPIReservation spi = comms_mux.reserve_spi();

    // Reset the configuration
    // -> Set the CRCO to 2-byte
    // -> Set the EN_CRC to 1
    // -> Bit PWR_UP to 0
    write_register(NRF_CONFIG, BIT(2) | BIT(3));

    set_retries_config(5 /* 1500 us */, 15 /* 15 retries */);

    uint8_t rf_setup = read_register(RF_SETUP);

    set_data_rate(RF24_1MBPS); // For compatibility with any hardware

    // Enable dynamic payloads on all pipes
    write_register(FEATURE, read_register(FEATURE) | BIT(EN_DPL));
    write_register(DYNPD, BIT(DPL_P5) | BIT(DPL_P4) | BIT(DPL_P3) | BIT(DPL_P2) | BIT(DPL_P1) | BIT(DPL_P0));

    // Reset status register
    write_register(NRF_STATUS, BIT(RX_DR) | BIT(TX_DS) | BIT(MAX_RT));

    // Set default channel
    set_channel(76);

    flush_rx();
    flush_tx();

    // Power up
    uint8_t config = read_register(NRF_CONFIG);
    if (!(config & BIT(PWR_UP))) {
        // Not powered up, power up
        write_register(NRF_CONFIG, config | BIT(PWR_UP));
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait to initialize
    }

    // Enable PTX mode
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) & ~BIT(PRIM_RX));

    // Set up the interrupt to trigger on negative edge as the pin is active low
    gpio_enable(irq_pin, GPIO_INPUT);
    gpio_set_interrupt(irq_pin, GPIO_INTTYPE_EDGE_NEG, nrf24_interrupt_handler);
    gpio_set_pullup(irq_pin, true, false);

    return (rf_setup != 0x00) && (rf_setup != 0xFF);
}

void NRF24::chip_select(bool select) {
    // Inverted logic as it is actually CS negated on the nRF part
    gpio_write(csn_pin, !select);
}

void NRF24::chip_enable(bool enable) {
    gpio_write(ce_pin, enable);
}

void NRF24::handle_interrupt() {
    if (comms_mux.is_in_use) {
        // There might be an ongoing transaction, so we cannot read the interrupt status
        // We can only schedule to handle the interrupt at the regular interval (TODO: find out the way to reschedule the task to raise its priority)

        return;
    }

    if (comms_mux.current_protocol != CommsMux::EnabledProtocol::SPI) {
        // We can reinitialize into SPI mode as comms are not in use
        comms_mux.initialize_spi();
    }

    uint8_t status_value = get_status();
    if (status_value & BIT(RX_DR)) {
        // There is a packet available

        bool fifo_has_data = read_register(FIFO_STATUS) & BIT(RX_EMPTY) == 0;
        
        while (fifo_has_data) {
            // We have data available
            // uint8_t pipe_number = (status_value & (BIT(3) | BIT(2) | BIT(1))) >> 1;
            uint8_t payload_size = read_payload(spi_rx_buffer, sizeof(spi_rx_buffer));

            wireless.add_incoming_packet_to_processing_queue(spi_rx_buffer, payload_size);

            fifo_has_data = read_register(FIFO_STATUS) & BIT(RX_EMPTY) == 0;
            if (fifo_has_data) {
                // We must reload the status
                status_value = get_status();
            }
        }

        // Clear the RX Data Ready bit
        write_register(NRF_STATUS, BIT(RX_DR));
    }
    
    if (status_value & BIT(TX_DS)) {
        // The packet has been sent and ack was received
        write_register(NRF_STATUS, BIT(TX_DS));
    }

    if (status_value & BIT(MAX_RT)) {
        // The packet has not been sent, maximum retransmissions reached
        // We must reset the bit to allow further transmissions
        write_register(NRF_STATUS, BIT(MAX_RT));
    }
}

// bool NRF24::is_connected() {
//     uint8_t setup_value = read_register(SETUP_AW); // Address width
//     return setup_value > 0 && setup_value <= 3;
// }

// void NRF24::start_listening() {
//     // Set the PRIM_RX bit in the CONFIG register
//     write_register(NRF_CONFIG, read_register(NRF_CONFIG) | BIT(PRIM_RX));
//     // Set the status register to clear the RX Data Ready, TX Data Sent and Max Retransmits bits
//     write_register(NRF_STATUS, BIT(RX_DR) | BIT(TX_DS) | BIT(MAX_RT)); 

//     chip_enable(true);

//     // Flush the RX FIFO if custom payload on acknowledge is enabled
//     if (read_register(FEATURE) & BIT(EN_ACK_PAY)) {
//         flush_rx();
//     }
// }

// void NRF24::stop_listening() {
//     chip_enable(false);

//     // TODO
// }




uint8_t NRF24::read_register(uint8_t reg, uint8_t* buffer_out, uint8_t length) {
    SPIReservation spi = comms_mux.reserve_spi();
    uint8_t result = read_register_nolock(reg, buffer_out, length);
    return result;
}

uint8_t NRF24::read_register_nolock(uint8_t reg, uint8_t* buffer_out, uint8_t length) {
    // Start transaction
    chip_select(true);

    uint8_t *rx = spi_rx_buffer;
    uint8_t *tx = spi_tx_buffer;
    uint8_t size = length + 1; // Register value goes to transmit buffer

    *tx++ = R_REGISTER | (REGISTER_MASK & reg);

    while (length--) {
        *tx++ = 0x00; // Zero out the rest of buffer
    }
    
    spi_transfer(comms_mux.spi_params.bus_id, spi_tx_buffer, spi_rx_buffer, size, SPI_8BIT);

    uint8_t status = *rx++;
    while (--size) {
        *buffer_out++ = *rx++;
    }

    // Stop transaction
    chip_select(false);
    
    return status;
}

NRF24 nrf24(BOARD_PIN_NRF24_CE, BOARD_PIN_NRF24_CS, BOARD_PIN_NRF24_IRQ, 1000000, 5);
