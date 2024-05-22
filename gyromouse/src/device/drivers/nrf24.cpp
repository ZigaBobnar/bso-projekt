#include "device/drivers/nrf24.hpp"
#include "device/drivers/comms_mux.hpp"
#include "device/wireless.hpp"

#include <RF24/nRF24L01.h>
#include <RF24/RF24.h>
#include <espressif/esp_common.h>


void nrf24_interrupt_handler(uint8_t gpio_num) {
    if (gpio_num == nrf24.irq_pin) {
        nrf24._handle_interrupt();
    }
}

NRF24::NRF24(uint8_t ce_pin, uint8_t csn_pin, uint8_t irq_pin, uint8_t address_width) :
    ce_pin(ce_pin),
    csn_pin(csn_pin),
    irq_pin(irq_pin),
    address_width(address_width) {}

bool NRF24::init() {
    gpio_enable(BOARD_PIN_SPI_SCK, GPIO_OUTPUT);
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

void NRF24::_handle_interrupt() {
    if (comms_mux.is_in_use) {
        // There might be an ongoing transaction, so we cannot read the interrupt status from within the ISR
        // We can only schedule to handle the interrupt at the regular interval
        need_to_check_receive_queue = true;

        return;
    }

    if (comms_mux.current_protocol != CommsMux::EnabledProtocol::SPI) {
        // We can reinitialize into SPI mode as comms are not in use
        comms_mux.initialize_spi();
    }

    _handle_received_data();
}

void NRF24::_check_receive_queue() {
    _handle_received_data();
}

void NRF24::_handle_received_data() {
    uint8_t status_value = read_register(NRF_STATUS);
    if (status_value & BIT(RX_DR)) {
        // There is a packet available

        bool fifo_has_data = (read_register(FIFO_STATUS) & BIT(RX_EMPTY)) == 0;
        
        while (fifo_has_data) {
            // We have data available
            // uint8_t pipe_number = (status_value & (BIT(3) | BIT(2) | BIT(1))) >> 1;
            uint8_t payload_size = read_payload(spi_rx_buffer, sizeof(spi_rx_buffer));

            wireless.add_incoming_packet_to_processing_queue(spi_rx_buffer, payload_size);

            fifo_has_data = (read_register(FIFO_STATUS) & BIT(RX_EMPTY)) == 0;
            if (fifo_has_data) {
                // We must reload the status
                status_value = read_register(NRF_STATUS);
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

void NRF24::start_listening() {
    // Set the PRIM_RX bit in the CONFIG register
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) | BIT(PRIM_RX));
    // Set the status register to clear the RX Data Ready, TX Data Sent and Max Retransmits bits
    write_register(NRF_STATUS, BIT(RX_DR) | BIT(TX_DS) | BIT(MAX_RT)); 

    chip_enable(true);

    // close_reading_pipe(0);

    // Flush the RX FIFO if custom payload on acknowledge is enabled
    if (read_register(FEATURE) & BIT(EN_ACK_PAY)) {
        flush_rx();
    }
}

void NRF24::stop_listening() {
    chip_enable(false);

    sdk_os_delay_us(85);

    // Flush the RX FIFO if custom payload on acknowledge is enabled
    if (read_register(FEATURE) & BIT(EN_ACK_PAY)) {
        sdk_os_delay_us(85);
        flush_rx();
    }

    // Clear the PRIM_RX bit in the CONFIG register
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) & ~BIT(PRIM_RX));

    // write_register(EN_RXADDR)
}


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

uint8_t NRF24::read_register(uint8_t reg) {
    SPIReservation spi = comms_mux.reserve_spi();
    uint8_t result = read_register_nolock(reg);
    return result;
}

uint8_t NRF24::read_register_nolock(uint8_t reg) {
    // Start transaction
    chip_select(true);

    uint8_t *rx = spi_rx_buffer;
    IGNORE_UNUSED(rx);
    uint8_t *tx = spi_tx_buffer;
    // uint8_t size = length + 1; // Register value goes to transmit buffer

    *tx++ = R_REGISTER | (REGISTER_MASK & reg);
    *tx++ = 0x01; // Apparently we need to send a dummy byte to get the result

    spi_transfer(comms_mux.spi_params.bus_id, spi_tx_buffer, spi_rx_buffer, 2, SPI_8BIT);

    uint8_t status = spi_rx_buffer[0];
    IGNORE_UNUSED(status);
    uint8_t result = spi_rx_buffer[1];

    // Stop transaction
    chip_select(false);

    return result;
}

uint8_t NRF24::write_register(uint8_t reg, const uint8_t* buffer_in, uint8_t length) {
    SPIReservation spi = comms_mux.reserve_spi();
    uint8_t result = write_register_nolock(reg, buffer_in, length);
    return result;
}

uint8_t NRF24::write_register_nolock(uint8_t reg, const uint8_t* buffer_in, uint8_t length) {
    // Start transaction
    chip_select(true);

    uint8_t *rx = spi_rx_buffer;
    uint8_t *tx = spi_tx_buffer;
    uint8_t size = length + 1; // Register value goes to transmit buffer

    *tx++ = W_REGISTER | (REGISTER_MASK & reg);

    while (length--) {
        *tx++ = *buffer_in++;
    }
    
    spi_transfer(comms_mux.spi_params.bus_id, spi_tx_buffer, spi_rx_buffer, size, SPI_8BIT);

    uint8_t status = *rx++;

    // Stop transaction
    chip_select(false);
    
    return status;
}

uint8_t NRF24::write_register(uint8_t reg, uint8_t value) {
    SPIReservation spi = comms_mux.reserve_spi();
    uint8_t result = write_register_nolock(reg, value);
    return result;
}

uint8_t NRF24::write_register_nolock(uint8_t reg, uint8_t value) {
    // Start transaction
    chip_select(true);

    uint8_t *rx = spi_rx_buffer;
    uint8_t *tx = spi_tx_buffer;
    // uint8_t size = length + 1; // Register value goes to transmit buffer

    *tx++ = W_REGISTER | (REGISTER_MASK & reg);
    *tx++ = value;

    spi_transfer(comms_mux.spi_params.bus_id, spi_tx_buffer, spi_rx_buffer, 2, SPI_8BIT);

    uint8_t status = *rx++;

    // Stop transaction
    chip_select(false);

    return status;
}

uint8_t NRF24::write_payload(uint8_t* buffer_in, uint8_t length, const uint8_t write_type) {
    SPIReservation spi = comms_mux.reserve_spi();
    uint8_t result = write_payload_nolock(buffer_in, length, write_type);
    return result;
}

uint8_t NRF24::write_payload_nolock(uint8_t* buffer_in, uint8_t length, const uint8_t write_type) {
    // Start transaction
    chip_select(true);

    uint8_t *rx = spi_rx_buffer;
    uint8_t *tx = spi_tx_buffer;
    uint8_t size = length + 1; // Register value goes to transmit buffer

    *tx++ = write_type;

    while (length--) {
        *tx++ = *buffer_in++;
    }
    
    spi_transfer(comms_mux.spi_params.bus_id, spi_tx_buffer, spi_rx_buffer, size, SPI_8BIT);

    uint8_t status = *rx++;

    // Stop transaction
    chip_select(false);
    
    return status;
}

uint8_t NRF24::read_payload(uint8_t* buffer_out, uint8_t length) {
    SPIReservation spi = comms_mux.reserve_spi();
    uint8_t result = read_payload_nolock(buffer_out, length);
    return result;
}

uint8_t NRF24::read_payload_nolock(uint8_t* buffer_out, uint8_t length) {
    // Start transaction
    chip_select(true);

    uint8_t *rx = spi_rx_buffer;
    uint8_t *tx = spi_tx_buffer;
    uint8_t size = length + 1; // Register value goes to transmit buffer

    *tx++ = R_RX_PAYLOAD; // Read instruction

    while (length--) {
        *tx++ = 0x00; // Zero out the rest of buffer
    }
    
    spi_transfer(comms_mux.spi_params.bus_id, spi_tx_buffer, spi_rx_buffer, size, SPI_8BIT);

    uint8_t status = *rx++;

    // Stop transaction
    chip_select(false);

    if (size > 1) {
        while (--size) {
            *buffer_out++ = *rx++;
        }
    }

    return status;
}

uint8_t NRF24::get_received_payload_dynamic_size() {
    spi_tx_buffer[0] = R_RX_PL_WID;
    spi_tx_buffer[1] = 0xff;
    
    SPIReservation spi = comms_mux.reserve_spi();
    // Start transaction
    chip_select(true);

    spi_transfer(comms_mux.spi_params.bus_id, spi_tx_buffer, spi_rx_buffer, 2, SPI_8BIT);

    // Stop transaction
    chip_select(false);

    uint8_t result = spi_rx_buffer[1];
    if (result > 32) {
        // Invalid value
        flush_rx();
        return 0;
    }

    return result;
}

void NRF24::write(uint8_t* buffer_in, uint8_t length) {
    write(buffer_in, length, false);
}

void NRF24::write(uint8_t* buffer_in, uint8_t length, bool is_multicast) {
    // First check if fifo buffer has space and block if it is full
    while (read_register(FIFO_STATUS) & BIT(TX_FULL)) {
        // The FIFO is full, we must wait for it to be empty
        sdk_os_delay_us(85);
        
        // Check if we reached maximum retransmissions and clear the bit
        if (read_register(NRF_STATUS) & BIT(MAX_RT)) {
            write_register(NRF_STATUS, BIT(MAX_RT));
        }
    }

    WRITE_COMMAND("nrf24", "write", "length=%d", length);
    write_payload(buffer_in, length, is_multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);

    // Start transmission
    chip_enable(true);
}

void NRF24::flush_rx() {
    _send_command(FLUSH_RX);
}

void NRF24::flush_tx() {
    _send_command(FLUSH_TX);
}

void NRF24::_send_command(uint8_t command) {
    SPIReservation spi = comms_mux.reserve_spi();
    
    // Start transaction
    chip_select(true);

    spi_transfer_8(comms_mux.spi_params.bus_id, command);

    // Stop transaction
    chip_select(false);
}

void NRF24::open_writing_pipe(const uint8_t* address) {
    write_register(RX_ADDR_P0, address, address_width);
    write_register(TX_ADDR, address, address_width);

    write_register(RX_PW_P0, 32);
}

void NRF24::open_reading_pipe(uint8_t pipe_number, const uint8_t* address) {
    write_register(RX_ADDR_P0 + pipe_number, address, address_width);
    write_register(RX_PW_P0 + pipe_number, 32);

    write_register(EN_RXADDR, read_register(EN_RXADDR) | BIT(pipe_number));
}

void NRF24::set_address_width(uint8_t width) {
    write_register(SETUP_AW, (width - 2) % 4);
}

void NRF24::set_retries_config(uint8_t delay_times_250_us, uint8_t count) {
    write_register(SETUP_RETR, (delay_times_250_us & 0x0F) << 4 | (count & 0x0F));
}

void NRF24::set_channel(uint8_t channel) {
    write_register(RF_CH, channel);
}

void NRF24::set_auto_acknowledge(uint8_t pipe_number, bool enabled) {
    uint8_t en_aa = read_register(EN_AA);
    if (enabled) {
        en_aa |= BIT(pipe_number);
    } else {
        en_aa &= ~BIT(pipe_number);
    }
    write_register(EN_AA, en_aa);
}

void NRF24::set_pa_level(uint8_t level) {
    uint8_t rf_setup = read_register(RF_SETUP);
    rf_setup &= ~(0x06); // Clear bits 1 and 2
    rf_setup |= (level & 0x03) << 1;
    write_register(RF_SETUP, rf_setup);
}

void NRF24::set_data_rate(uint8_t rate) {
    uint8_t rf_setup = read_register(RF_SETUP);
    rf_setup &= ~(BIT(3) | BIT(5)); // Clear bits 3 and 5
    switch (rate) {
        case RF24_250KBPS:
            rf_setup |= BIT(5);
            break;
        case RF24_2MBPS:
            rf_setup |= BIT(3);
            break;
        case RF24_1MBPS:
        default:
            break;
    }
    write_register(RF_SETUP, rf_setup);
}

void NRF24::set_crc_length(uint8_t length) {
    uint8_t config = read_register(NRF_CONFIG);
    config &= ~(BIT(EN_CRC) | BIT(CRCO));
    if (length > 0) {
        config |= BIT(EN_CRC);
        if (length > 1) {
            config |= BIT(CRCO);
        }
    }
    write_register(NRF_CONFIG, config);
}

void NRF24::set_irq_mask_disable(bool tx_ok, bool tx_fail, bool rx_ready) {
    uint8_t config = read_register(NRF_CONFIG);
    // Clear flags
    config &= ~(BIT(MASK_RX_DR) | BIT(MASK_TX_DS) | BIT(MASK_MAX_RT));
    // Set new flags
    config |= (tx_ok ? 0 : BIT(MASK_TX_DS)) | (tx_fail ? 0 : BIT(MASK_MAX_RT)) | (rx_ready ? 0 : BIT(MASK_RX_DR));
    write_register(NRF_CONFIG, config);
}



NRF24 nrf24(BOARD_PIN_NRF24_CE, BOARD_PIN_NRF24_CS, BOARD_PIN_NRF24_IRQ, 5);
