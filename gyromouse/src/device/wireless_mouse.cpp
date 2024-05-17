#include "device/wireless_mouse.hpp"

#include "device/drivers/comms_mux.hpp"


uint8_t get_wireless_mouse_command_data_length(WirelessMouseCommand command) {
    switch (command) {
        case WirelessMouseCommand::Ping:
        case WirelessMouseCommand::AnnounceAsDongle:
        case WirelessMouseCommand::AnnounceAsMouse:
        case WirelessMouseCommand::Mouse_Buttons_State:
            return 1;
        case WirelessMouseCommand::StartOfNewData:
        case WirelessMouseCommand::Accelerometer_Int16_X:
        case WirelessMouseCommand::Accelerometer_Int16_Y:
        case WirelessMouseCommand::Accelerometer_Int16_Z:
        case WirelessMouseCommand::Gyroscope_Int16_X:
        case WirelessMouseCommand::Gyroscope_Int16_Y:
        case WirelessMouseCommand::Gyroscope_Int16_Z:
        case WirelessMouseCommand::Magnetometer_Int16_X:
        case WirelessMouseCommand::Magnetometer_Int16_Y:
        case WirelessMouseCommand::Magnetometer_Int16_Z:
        case WirelessMouseCommand::Temperature_Int16:
            return 2;
        case WirelessMouseCommand::Accelerometer_Scale:
        case WirelessMouseCommand::Gyroscope_Scale:
        case WirelessMouseCommand::Magnetometer_Scale:
        case WirelessMouseCommand::Temperature_Scale:
            return 4;
        case WirelessMouseCommand::Accelerometer_Int16_XYZ:
        case WirelessMouseCommand::Gyroscope_Int16_XYZ:
        case WirelessMouseCommand::Magnetometer_Int16_XYZ:
            return 6;
        default:
            return 0; // Unknown command
    }
}

WirelessMouseCommand parse_wireless_mouse_command(uint8_t value) {
    WirelessMouseCommand command = static_cast<WirelessMouseCommand>(value);
    switch (command) {
        case WirelessMouseCommand::Ping:
        case WirelessMouseCommand::StartOfNewData:
        case WirelessMouseCommand::AnnounceAsDongle:
        case WirelessMouseCommand::AnnounceAsMouse:
        case WirelessMouseCommand::Mouse_Buttons_State:
        case WirelessMouseCommand::Accelerometer_Scale:
        case WirelessMouseCommand::Accelerometer_Int16_X:
        case WirelessMouseCommand::Accelerometer_Int16_Y:
        case WirelessMouseCommand::Accelerometer_Int16_Z:
        case WirelessMouseCommand::Accelerometer_Int16_XYZ:
        case WirelessMouseCommand::Gyroscope_Scale:
        case WirelessMouseCommand::Gyroscope_Int16_X:
        case WirelessMouseCommand::Gyroscope_Int16_Y:
        case WirelessMouseCommand::Gyroscope_Int16_Z:
        case WirelessMouseCommand::Gyroscope_Int16_XYZ:
        case WirelessMouseCommand::Magnetometer_Scale:
        case WirelessMouseCommand::Magnetometer_Int16_X:
        case WirelessMouseCommand::Magnetometer_Int16_Y:
        case WirelessMouseCommand::Magnetometer_Int16_Z:
        case WirelessMouseCommand::Magnetometer_Int16_XYZ:
        case WirelessMouseCommand::Temperature_Scale:
        case WirelessMouseCommand::Temperature_Int16:
            return command;
        default:
            return WirelessMouseCommand::Unknown;
    }
}

// https://nrf24.github.io/RF24/md_COMMON__ISSUES.html

WirelessMouse::WirelessMouse(const int spi_clock_pin, const int chip_select_pin, const int chip_enable_pin) :
    spi_clock_pin(spi_clock_pin), chip_select_pin(chip_select_pin), chip_enable_pin(chip_enable_pin), radio(chip_enable_pin, chip_select_pin) {
}

void WirelessMouse::init(const uint8_t channel) {
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
 
    switch_to_mouse_mode();

    vTaskDelay(10 / portTICK_PERIOD_MS);

}

void WirelessMouse::process_data() {
    // Check if we have any payloads available.
    // The pipe that contains payload is stored.
    uint8_t pipe;
    while (radio.available(&pipe)) {
        const uint8_t payload_bytes = radio.getDynamicPayloadSize();
        // WRITE_COMMAND("nrf24", "Received packet on pipe %d, length: %d", pipe, payload_bytes);

        uint8_t buffer[payload_bytes];
        radio.read(buffer, payload_bytes);

        WirelessMouseCommand command = parse_wireless_mouse_command(buffer[0]);
        // WRITE_COMMAND("nrf24", "Received command 0x%x => parsed 0x%x", buffer[0], static_cast<uint8_t>(command));
        if (command == WirelessMouseCommand::Unknown) {
            WRITE_COMMAND("error", "Unknown command received, 0x%x", buffer[0]);
            continue;
        }

        const bool is_request = payload_bytes == 1;
        const uint8_t data_length = get_wireless_mouse_command_data_length(command);
        
        if (!is_request && (payload_bytes - data_length != 1)) {
            WRITE_COMMAND("error", "Data length missmatch for nrf24 packet, command: 0x%x", static_cast<uint8_t>(command));
            continue;
        } else if (command == WirelessMouseCommand::Unknown) {
            WRITE_COMMAND("error", "Unknown command received");
            continue;
        }

        switch (command) {
            case WirelessMouseCommand::Ping:
                static TickType_t last_ping_requested = 0;
                static TickType_t last_ping_response = 0;
                if (is_request) {
                    TickType_t current_time = xTaskGetTickCount();
                    WRITE_COMMAND("ping", "requested(diff=%f)", ((float)current_time - last_ping_requested) * portTICK_PERIOD_MS/1000);
                    last_ping_requested = current_time;
                    // Respond with ping response
                    write_packet(WirelessMouseCommand::Ping, &PING_RESPONSE_BYTE, 1);
                    // WRITE_COMMAND("ping", "sent response");
                } else {
                    TickType_t current_time = xTaskGetTickCount();
                    
                    if (buffer[1] != PING_RESPONSE_BYTE) {
                        WRITE_COMMAND("error", "Invalid data for ping response");
                        continue;
                    }
                    
                    WRITE_COMMAND("ping", "responded(diff=%f, reqresdiff=%f)", ((float)current_time - last_ping_response) * portTICK_PERIOD_MS/1000, ((float)current_time - last_ping_request) * portTICK_PERIOD_MS/1000);
                    last_ping_response = current_time;
                }
                break;
            default:
                // TODO: Process the rest of commands
                WRITE_COMMAND("error", "Received but not handled command 0x%x", static_cast<uint8_t>(command));
                break;
        }
    }
}

void WirelessMouse::switch_to_dongle_mode() {
    // radio.stopListening();
    // TODO;
    
    radio.closeReadingPipe(1);

    radio.openWritingPipe(WIRELESS_DONGLE_ADDRESS);
    radio.openReadingPipe(1, WIRELESS_MOUSE_ADDRESS);

    radio.startListening();
    mode = WirelessMouseMode::Dongle;
    // write_packet(WirelessMouseCommand::AnnounceAsDongle, nullptr, 0);
}

void WirelessMouse::switch_to_mouse_mode() {
    // radio.stopListening();
    // TODO;
    
    radio.closeReadingPipe(1);

    radio.openWritingPipe(WIRELESS_MOUSE_ADDRESS);
    radio.openReadingPipe(1, WIRELESS_DONGLE_ADDRESS);

    radio.startListening();
    mode = WirelessMouseMode::Mouse;
    // write_packet(WirelessMouseCommand::AnnounceAsMouse, nullptr, 0);
}

void WirelessMouse::write_packet(WirelessMouseCommand command, const uint8_t* data, uint8_t length) {
    if (!this->radio_available) {
        return;
    }

    int supposed_length = get_wireless_mouse_command_data_length(command);
    if ((length != 0 && supposed_length != length) || command == WirelessMouseCommand::Unknown) {
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
    // WRITE_COMMAND("nrf24", "Sending packet (command: 0x%x, length: %d)", static_cast<uint8_t>(command), length);
    radio.write(packet_data, total_length);
    radio.startListening();
}

WirelessMouse wireless_mouse(BOARD_PIN_SPI_SCK, BOARD_PIN_NRF24_CS, BOARD_PIN_NRF24_CE);
TickType_t last_ping_request = 0;
