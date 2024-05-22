#include "device/wireless.hpp"

#include "device/drivers/comms_mux.hpp"


uint8_t get_wireless_command_data_length(WirelessCommand command) {
    switch (command) {
        case WirelessCommand::Ping:
        case WirelessCommand::AnnounceAsDongle:
        case WirelessCommand::AnnounceAsMouse:
        case WirelessCommand::Mouse_Buttons_State:
            return 1;
        case WirelessCommand::StartOfNewData:
        case WirelessCommand::Accelerometer_Int16_X:
        case WirelessCommand::Accelerometer_Int16_Y:
        case WirelessCommand::Accelerometer_Int16_Z:
        case WirelessCommand::Gyroscope_Int16_X:
        case WirelessCommand::Gyroscope_Int16_Y:
        case WirelessCommand::Gyroscope_Int16_Z:
        case WirelessCommand::Magnetometer_Int16_X:
        case WirelessCommand::Magnetometer_Int16_Y:
        case WirelessCommand::Magnetometer_Int16_Z:
        case WirelessCommand::Temperature_Int16:
            return 2;
        case WirelessCommand::Accelerometer_Scale:
        case WirelessCommand::Gyroscope_Scale:
        case WirelessCommand::Magnetometer_Scale:
        case WirelessCommand::Temperature_Scale:
            return 4;
        case WirelessCommand::Accelerometer_Int16_XYZ:
        case WirelessCommand::Gyroscope_Int16_XYZ:
        case WirelessCommand::Magnetometer_Int16_XYZ:
            return 6;
        default:
            return 0; // Unknown command
    }
}

WirelessCommand parse_wireless_command(uint8_t value) {
    WirelessCommand command = static_cast<WirelessCommand>(value);
    switch (command) {
        case WirelessCommand::Ping:
        case WirelessCommand::StartOfNewData:
        case WirelessCommand::AnnounceAsDongle:
        case WirelessCommand::AnnounceAsMouse:
        case WirelessCommand::Mouse_Buttons_State:
        case WirelessCommand::Accelerometer_Scale:
        case WirelessCommand::Accelerometer_Int16_X:
        case WirelessCommand::Accelerometer_Int16_Y:
        case WirelessCommand::Accelerometer_Int16_Z:
        case WirelessCommand::Accelerometer_Int16_XYZ:
        case WirelessCommand::Gyroscope_Scale:
        case WirelessCommand::Gyroscope_Int16_X:
        case WirelessCommand::Gyroscope_Int16_Y:
        case WirelessCommand::Gyroscope_Int16_Z:
        case WirelessCommand::Gyroscope_Int16_XYZ:
        case WirelessCommand::Magnetometer_Scale:
        case WirelessCommand::Magnetometer_Int16_X:
        case WirelessCommand::Magnetometer_Int16_Y:
        case WirelessCommand::Magnetometer_Int16_Z:
        case WirelessCommand::Magnetometer_Int16_XYZ:
        case WirelessCommand::Temperature_Scale:
        case WirelessCommand::Temperature_Int16:
            return command;
        default:
            return WirelessCommand::Unknown;
    }
}

// https://nrf24.github.io/RF24/md_COMMON__ISSUES.html

Wireless::Wireless(const int spi_clock_pin, const int chip_select_pin, const int chip_enable_pin) :
    spi_clock_pin(spi_clock_pin), chip_select_pin(chip_select_pin), chip_enable_pin(chip_enable_pin), radio(chip_enable_pin, chip_select_pin) {
}

void Wireless::init(const uint8_t channel) {
    SPIReservation reservation = comms_mux.reserve_spi();
    
    this->channel = channel;

	gpio_enable(spi_clock_pin, GPIO_OUTPUT);
	gpio_enable(chip_select_pin, GPIO_OUTPUT);

    this->radio_available = radio.begin();
    if (!this->radio_available) {
        WRITE_COMMAND("error", "Failed to initialize nrf24 radio module");
        return;
    }

    radio.setAddressWidth(WIRELESS_ADDRESS_WIDTH);
    radio.setChannel(channel);
    radio.stopListening();
    radio.setPALevel(RF24_PA_LOW); // As we are not far we can decrease the amplifier
    radio.setDataRate(RF24_2MBPS); // Reduce data transfer rate
    radio.setCRCLength(RF24_CRC_8); // Reduce CRC to 8 bits
    radio.setPayloadSize(4); // Reduce static size from 32 to 4 bytes
    radio.setRetries(0 /* Delay (value+1)*250us */, 2 /* Try 2 times */);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.flush_rx();
    radio.flush_tx();

    WRITE_COMMAND("nrf24", "Initialized on channel %d", channel);

    // // Open the reading and writing pipes to correct addresses
    // // We want to be able to receive announcment from dongle:
    // radio.openWritingPipe(WIRELESS_MOUSE_ADDRESS); // Pipe 0 is used to send data from mouse to dongle
    // radio.openReadingPipe(1, WIRELESS_DONGLE_ADDRESS); // Pipe 1 is used to receive data from dongle to mouse
    // // TODO: add function to initialize pipes and addresses correctly for dongle mode
    // radio.startListening();

    vTaskDelay(10 / portTICK_PERIOD_MS);

    radio.printDetails();
 
    // Enable the interrupt -> it is active low
    gpio_enable(BOARD_PIN_NRF24_IRQ, GPIO_INPUT);
    gpio_set_pullup(BOARD_PIN_NRF24_IRQ, true, false);
    // gpio_set_interrupt(BOARD_PIN_NRF24_IRQ, GPIO_INTTYPE_EDGE_NEG, wireless_interrupt_handler);

    switch_to_mouse_mode();

    vTaskDelay(10 / portTICK_PERIOD_MS);

}

// This must be ran as a FreeRTOS task that does not expect to be used for anything else
// TODO: Interrupts are not implemented by base NRF library -> we should have a possibility to prevent constant switching between SPI and I2C mode, it should only switch on interrupt
void Wireless::_process_data_task() {
    while (true) {
        // Check if we have any payloads available.
        // The pipe that contains payload is stored.
        uint8_t pipe;
        while (radio.available(&pipe)) {
            const uint8_t payload_bytes = radio.getDynamicPayloadSize();
            // WRITE_COMMAND("nrf24", "Received packet on pipe %d, length: %d", pipe, payload_bytes);

            uint8_t buffer[payload_bytes];
            radio.read(buffer, payload_bytes);

            WirelessCommand command = parse_wireless_command(buffer[0]);
            // WRITE_COMMAND("nrf24", "Received command 0x%x => parsed 0x%x", buffer[0], static_cast<uint8_t>(command));
            if (command == WirelessCommand::Unknown) {
                WRITE_COMMAND("error", "Unknown command received, 0x%x", buffer[0]);
                continue;
            }

            const bool is_request = payload_bytes == 1;
            const uint8_t data_length = get_wireless_command_data_length(command);
            
            if (!is_request && (payload_bytes - data_length != 1)) {
                WRITE_COMMAND("error", "Data length missmatch for nrf24 packet, command: 0x%x", static_cast<uint8_t>(command));
                continue;
            } else if (command == WirelessCommand::Unknown) {
                WRITE_COMMAND("error", "Unknown command received");
                continue;
            }

            switch (command) {
                case WirelessCommand::Ping:
                    if (is_request) {
                        TickType_t current_time = xTaskGetTickCount();
                        WRITE_COMMAND("ping", "requested(diff=%f)", ((float)current_time - gyromouse.timings.last_ping_requested) * portTICK_PERIOD_MS/1000);
                        gyromouse.timings.last_ping_requested = current_time;
                        // Respond with ping response
                        write_packet(WirelessCommand::Ping, &PING_RESPONSE_BYTE, 1);
                        // WRITE_COMMAND("ping", "sent response");
                    } else {
                        TickType_t current_time = xTaskGetTickCount();
                        
                        if (buffer[1] != PING_RESPONSE_BYTE) {
                            WRITE_COMMAND("error", "Invalid data for ping response");
                            continue;
                        }
                        
                        WRITE_COMMAND("ping", "responded(diff=%f, reqresdiff=%f)", ((float)current_time - gyromouse.timings.last_ping_response) * portTICK_PERIOD_MS/1000, ((float)current_time - gyromouse.timings.last_ping_request) * portTICK_PERIOD_MS/1000);
                        gyromouse.timings.last_ping_response = current_time;
                    }
                    break;
                default:
                    // TODO: Process the rest of commands
                    WRITE_COMMAND("error", "Received but not handled command 0x%x", static_cast<uint8_t>(command));
                    break;
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void Wireless::switch_to_dongle_mode() {
    // radio.stopListening();
    // TODO;
    
    radio.closeReadingPipe(1);

    radio.openWritingPipe(WIRELESS_DONGLE_ADDRESS);
    radio.openReadingPipe(1, WIRELESS_MOUSE_ADDRESS);

    radio.startListening();
    gyromouse.mode = GyroMouse::Mode::DONGLE;
    // write_packet(WirelessCommand::AnnounceAsDongle, nullptr, 0);
}

void Wireless::switch_to_mouse_mode() {
    // radio.stopListening();
    // TODO;
    
    radio.closeReadingPipe(1);

    radio.openWritingPipe(WIRELESS_MOUSE_ADDRESS);
    radio.openReadingPipe(1, WIRELESS_DONGLE_ADDRESS);

    radio.startListening();
    gyromouse.mode = GyroMouse::Mode::MOUSE;
    // write_packet(WirelessCommand::AnnounceAsMouse, nullptr, 0);
}

void Wireless::write_packet(WirelessCommand command, const uint8_t* data, uint8_t length) {
    if (!this->radio_available) {
        return;
    }

    int supposed_length = get_wireless_command_data_length(command);
    if ((length != 0 && supposed_length != length) || command == WirelessCommand::Unknown) {
        WRITE_COMMAND("error", "Invalid data length for nrf24 packet, command: 0x%x, length: %d", static_cast<uint8_t>(command), length);
        return;
    }

    const uint8_t total_length = length + 1;
    uint8_t packet_data[total_length];
    packet_data[0] = static_cast<uint8_t>(command);
    if (data != nullptr) {
        for (int i = 0; i < length; i++) {
            packet_data[i+1] = data[i];
        }
    }
    
    SPIReservation reservation = comms_mux.reserve_spi();

    radio.stopListening();
    WRITE_COMMAND("nrf24", "Sending packet (command: 0x%x, length: %d)", static_cast<uint8_t>(command), length);
    radio.write(packet_data, total_length);
    radio.startListening();
}

void task_wireless_process_data(void *pvParameters) {
    wireless._process_data_task();

    vTaskDelete(NULL);
}

void wireless_interrupt_handler() {
    // This comes from the interrupt pin going low on NRF -> meaning we have to check the status register
}

Wireless wireless(BOARD_PIN_SPI_SCK, BOARD_PIN_NRF24_CS, BOARD_PIN_NRF24_CE);
