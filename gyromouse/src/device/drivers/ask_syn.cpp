#include "device/drivers/ask_syn.hpp"

#include <esp/gpio.h>


AskSynTransmitter::AskSynTransmitter(uint8_t tx_pin) : tx_pin(tx_pin) {
    tx_mutex = xSemaphoreCreateMutex();
    tx_active_buffer = tx_raw_buffer;
    tx_prefill_buffer = tx_raw_buffer + ASK_SYN_PACKET_LENGTH_BYTES;
}

void AskSynTransmitter::init() {
    gpio_enable(tx_pin, GPIO_OUTPUT);
    gpio_write(tx_pin, 0);
    xTaskCreate(ask_syn_transmitter_task, "ask_syn_transmitter_task", 256, NULL, 2, NULL);
}

void AskSynTransmitter::send_packet(uint8_t *data, size_t length) {
    // Currently cannot send packets larger than the expected size
    if (length > ASK_SYN_PACKET_LENGTH_BYTES) {
        WRITE_COMMAND("error", "AskSynTransmitter::send_packet: Packet length exceeds the maximum");
    }

    if (tx_prefill_buffer_full) {
        WRITE_COMMAND("error", "AskSynTransmitter::send_packet: Prefill buffer is full, cannot send packet");
    }
    
    xSemaphoreTake(tx_mutex, portMAX_DELAY);
    
    // while (tx_prefill_buffer_full) {
    //     // Need to wait for the transmitter to finish sending the previous packets
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }

    // Copy the packet into tx buffer
    size_t i;
    for (i = 0; i < length; i++) {
        tx_prefill_buffer[i] = data[i];
    }
    for (; i < ASK_SYN_PACKET_LENGTH_BYTES; i++) {
        // Fill the rest of the packet with 0x00
        tx_prefill_buffer[i] = 0x00;
    }

    if (!tx_active_buffer_full) {
        // Swap the buffers if the active buffer is not full
        uint8_t *temp = tx_active_buffer;
        tx_active_buffer = tx_prefill_buffer;
        tx_prefill_buffer = temp;

        tx_active_buffer_full = true;
        tx_prefill_buffer_full = false;
    } else {
        tx_prefill_buffer_full = true;
    }

    xSemaphoreGive(tx_mutex);
}

// bool AskSynTransmitter::_has_data() {
//     return (tx_buffer_tail != tx_buffer_head);
// }

// void AskSynTransmitter::_add_to_queue(uint8_t data) {
//     if (((tx_buffer_head + 1) % ASK_SYN_TRANSMIT_BUFFER_SIZE) == tx_buffer_tail) return;

//     tx_buffer[tx_buffer_head] = data;
//     tx_buffer_head = (tx_buffer_head + 1) % ASK_SYN_TRANSMIT_BUFFER_SIZE;
// }

// uint8_t AskSynTransmitter::_get_from_queue() {
//     if (tx_buffer_tail == tx_buffer_head) return 0x00;

//     uint8_t data = tx_buffer[tx_buffer_tail];
//     tx_buffer_tail = (tx_buffer_tail + 1) % ASK_SYN_TRANSMIT_BUFFER_SIZE;
//     return data;
// }

AskSynTransmitter ask_syn_transmitter(BOARD_PIN_SYN115_TXD);


/***
 * NOTE: Highly inefficient approach as this requires us to preemptively switch to the loop and interrupt other tasks, then accurately switch back to here.
 * It would probably be better to use timer based solution.
 * Hardware timer should get enabled whenever there is data packet that needs to be sent, then in hardware interrupt we should ony set the gpio value and increase the position index.
 * If finished, the timer gets disabled. This means we still need to switch tasks but the interrupt runtime should be much more accurate.
 * 
*/

void ask_syn_transmitter_task(void *pvParameters) {
    if (task_handles.ask_syn_transmitter == nullptr) {
        task_handles.ask_syn_transmitter = xTaskGetCurrentTaskHandle();
    }

    // TickType_t last_wake = 0;

    while (true) {
        if (!ask_syn_transmitter.tx_active_buffer_full) {
            if (ask_syn_transmitter.tx_prefill_buffer_full) {
                // Swap the buffers
                uint8_t *temp = ask_syn_transmitter.tx_active_buffer;
                ask_syn_transmitter.tx_active_buffer = ask_syn_transmitter.tx_prefill_buffer;
                ask_syn_transmitter.tx_prefill_buffer = temp;
                ask_syn_transmitter.tx_prefill_buffer_full = false;
                ask_syn_transmitter.tx_active_buffer_full = true;
            } else {
                // vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ASK_SYN_TX_WAIT_FOR_PACKET_SLEEP_DURATION_MS));
                vTaskDelay(pdMS_TO_TICKS(ASK_SYN_TX_WAIT_FOR_PACKET_SLEEP_DURATION_MS));
                continue;
            }
        }
    
        // TickType_t current_tx_time = xTaskGetTickCount();

        WRITE_COMMAND("ask_syn", "Transmitting packet");

        // Wait with start of packet for min idle duration
        gpio_write(ask_syn_transmitter.tx_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(ASK_SYN_MIN_IDLE_DURATION_BITS * ASK_SYN_BIT_DURATION_MS));

        // Transmit start sequence
        for (uint8_t i = 0; i < 8; i++) {
            gpio_write(ask_syn_transmitter.tx_pin, START_SEQUENCE_BYTE & (1 << (7 - i)));
            vTaskDelay(pdMS_TO_TICKS(ASK_SYN_BIT_DURATION_MS));
        }

        // Transmit the packet byte by byte, bit by bit
        for (uint8_t i = 0; i < ASK_SYN_PACKET_LENGTH_BYTES; i++) {

            uint8_t data = ask_syn_transmitter.tx_active_buffer[i];
            for (uint8_t b = 0; b < 8; b++) {
                uint8_t bit_value = (data >> (7 - b)) & (1 << 0);
                
                gpio_write(ask_syn_transmitter.tx_pin, bit_value);
                vTaskDelay(pdMS_TO_TICKS(ASK_SYN_BIT_DURATION_MS));
            }
        }

        // Transmit stop sequence
        for (uint8_t i = 0; i < STOP_SEQUENCE_LENGTH; i++) {
            gpio_write(ask_syn_transmitter.tx_pin, STOP_SEQUENCE[i]);
            vTaskDelay(pdMS_TO_TICKS(ASK_SYN_BIT_DURATION_MS));
        }

        gpio_write(ask_syn_transmitter.tx_pin, 0);

        // Packet fully transmitted, swap buffers if new packet is available
        // xSemaphoreTake(ask_syn_transmitter.tx_mutex, portMAX_DELAY);
        if (ask_syn_transmitter.tx_prefill_buffer_full) {
            uint8_t *temp = ask_syn_transmitter.tx_active_buffer;
            ask_syn_transmitter.tx_active_buffer = ask_syn_transmitter.tx_prefill_buffer;
            ask_syn_transmitter.tx_prefill_buffer = temp;

            ask_syn_transmitter.tx_active_buffer_full = true;
            ask_syn_transmitter.tx_prefill_buffer_full = false;
        } else {
            ask_syn_transmitter.tx_active_buffer_full = false;
        }
        // xSemaphoreGive(ask_syn_transmitter.tx_mutex);

        ask_syn_transmitter.tx_last_transmit_time = xTaskGetTickCount();
    }

    task_handles.ask_syn_transmitter = nullptr;
    vTaskDelete(NULL);
}


AskSynReceiver::AskSynReceiver(uint8_t rx_pin) : rx_pin(rx_pin) {
    rx_buffer_head = 0;
    rx_buffer_tail = 0;
    rx_buffer_size = ASK_SYN_RECEIVE_BUFFER_SIZE;
}

void AskSynReceiver::init() {
    gpio_enable(rx_pin, GPIO_INPUT);
    gpio_set_pullup(rx_pin, true, false);

    rx_last_value = gpio_read(rx_pin) != 0x00;
    rx_last_change_time = xTaskGetTickCount();
    rx_state = ReceiverState::Idle;
    rx_magic_sequence_position = 0;
    rx_active_byte = 0x00;
    rx_active_bit_index = 0;

    xTaskCreate(ask_syn_receiver_raw_processing_task, "ask_syn_receiver_raw_processing_task", 256, NULL, 2, NULL);
}

void AskSynReceiver::pause_interrupt() {
    gpio_set_interrupt(rx_pin, GPIO_INTTYPE_NONE, receive_interrupt_handler);
}

void AskSynReceiver::resume_interrupt() {
    rx_last_change_time = xTaskGetTickCount();
    rx_last_value = gpio_read(rx_pin) != 0x00;

    gpio_set_interrupt(rx_pin, GPIO_INTTYPE_EDGE_ANY, receive_interrupt_handler);
}

bool AskSynReceiver::has_data() {
    return (rx_buffer_tail != rx_buffer_head);
}

uint8_t AskSynReceiver::available_data_count() {
    return (rx_buffer_head - rx_buffer_tail) % ASK_SYN_RECEIVE_BUFFER_SIZE;
}

void AskSynReceiver::_add_to_queue(uint8_t *data, size_t length) {
    if (((rx_buffer_head + 1) % ASK_SYN_RECEIVE_BUFFER_SIZE) == rx_buffer_tail) return;

    if (length > ASK_SYN_PACKET_LENGTH_BYTES) {
        WRITE_COMMAND("error", "AskSynReceiver::__add_to_queue: Packet length exceeds the maximum");
    }

    size_t i;
    for (i = 0; i < length; i++) {
        rx_buffer[rx_buffer_head][i] = data[i];
    }
    for (; i < ASK_SYN_PACKET_LENGTH_BYTES; i++) {
        // Fill the rest of the packet with 0x00
        rx_buffer[rx_buffer_head][i] = 0x00;
    }
    rx_buffer_head = (rx_buffer_head + 1) % ASK_SYN_RECEIVE_BUFFER_SIZE;
}

uint8_t *AskSynReceiver::get_from_queue() {
    if (rx_buffer_tail == rx_buffer_head) return 0x00;

    uint8_t *data = rx_buffer[rx_buffer_tail];
    rx_buffer_tail = (rx_buffer_tail + 1) % ASK_SYN_RECEIVE_BUFFER_SIZE;
    return data;
}

bool AskSynReceiver::_has_bits_to_process() {
    return (rx_received_bits_tail != rx_received_bits_head);
}

void AskSynReceiver::_add_to_bits_queue(uint8_t data, TickType_t time_delta) {
    if (((rx_received_bits_head + 1) % ASK_BIT_QUEUE) == rx_received_bits_tail) return;

    rx_received_bits_queue[rx_received_bits_head] = {
        .data = data,
        .time_delta = time_delta,
    };
    rx_received_bits_head = (rx_received_bits_head + 1) % ASK_BIT_QUEUE;
}

AskSynReceiver::received_bits_t AskSynReceiver::_get_from_bits_queue() {
    if (rx_received_bits_tail == rx_received_bits_head) return {};

    received_bits_t *data = &rx_received_bits_queue[rx_received_bits_tail];
    rx_received_bits_tail = (rx_received_bits_tail + 1) % ASK_BIT_QUEUE;
    return *data;
}

AskSynReceiver ask_syn_receiver(BOARD_PIN_SYN470R_RXD);


void ask_syn_process_bit(bool new_value, bool old_value);

void ask_syn_receiver_raw_processing_task(void *pvParameters) {
    while (true) {

        if (ask_syn_receiver._has_bits_to_process()) {
            do {
                AskSynReceiver::received_bits_t data = ask_syn_receiver._get_from_bits_queue();

                bool old_value = (data.data & (1 << 1)) != 0x00;
                bool new_value = (data.data & (1 << 0)) != 0x00;
                old_value = old_value;
                new_value = new_value;
                TickType_t time_delta = data.time_delta;

                // Determine the number of possible bits in current processing unit.
                // If the value stays the same, multiple bits can hide within the current change.
                // Calculate the number of bits that might have happened since the last change.
                #define DIV_ROUND_TO_CLOSEST(n, d) ( ((n) + (d)/2) / (d) )
                // #define DIV_ROUND_TO_CLOSEST(n, d) ((((n) < 0) == ((d) < 0)) ? (((n) + (d)/2)/(d)) : (((n) - (d)/2)/(d)))
                // uint8_t bits_since_last_change = (uint8_t)DIV_ROUND_TO_CLOSEST(time_delta, pdMS_TO_TICKS(ASK_SYN_BIT_DURATION_MS));
                uint8_t bits_since_last_change = (uint8_t)(time_delta) / pdMS_TO_TICKS(ASK_SYN_BIT_DURATION_MS);

                // WRITE_COMMAND("ask_syn", "Detected %d bits to process (delta = %f)", bits_since_last_change, (float)time_delta / pdMS_TO_TICKS(ASK_SYN_BIT_DURATION_MS));

                if (bits_since_last_change > 8 && (ask_syn_receiver.rx_state == AskSynReceiver::ReceiverState::Idle || ask_syn_receiver.rx_state == AskSynReceiver::ReceiverState::ReceivingStartSequence || ask_syn_receiver.rx_state == AskSynReceiver::ReceiverState::ReceivingStopSequence)) {
                    // The maximum bits that are accepted in this mode is 8
                    bits_since_last_change = 8;
                    // WRITE_COMMAND("ask_syn", "Clamping to 8 bits");
                } else if (bits_since_last_change > (ASK_SYN_PACKET_LENGTH_BYTES * 8 + 1) && ask_syn_receiver.rx_state == AskSynReceiver::ReceiverState::ReceivingData) {
                    // Received much more bits with same value than is expected from within the single packet, stop receiving
                    bits_since_last_change = 8;
                    WRITE_COMMAND("ask_syn", "Too many same value bits, reseting to idle");
                    ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::Idle;
                    ask_syn_receiver.rx_magic_sequence_position = 0;
                }

                for (uint8_t i = 0; i < bits_since_last_change; i++) {
                    ask_syn_process_bit(new_value, old_value);
                    old_value = new_value;
                }
            } while (ask_syn_receiver._has_bits_to_process());
        } else {
            // WRITE_COMMAND("ask_syn", "No bits to process (value=%d)", gpio_read(ask_syn_receiver.rx_pin));
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void ask_syn_process_bit(bool new_value, bool old_value) {
    // Signal structure:
    // | Idle   | |  Start | |     Data (n bytes)   | |  Stop  | | Idle
    // 0 ...... 0 1  0  1  0 dddddddddddddddddddddddd 0  1  0  1 0..........
    // |        | |        | |                      | |        | |

    // WRITE_COMMAND("ask_syn", "Processing bit: %d -> %d", old_value, new_value);


    if (ask_syn_receiver.rx_state == AskSynReceiver::ReceiverState::Idle) {
        if (old_value == false) {
            // Expect idle to be low before starting to receive

            // if (new_value == true) {
            //     WRITE_COMMAND("ask_syn", "Should start receiving!!!!!");
            // }

            if (new_value == ((START_SEQUENCE_BYTE & (1 << 7)) != 0)) {
                // Received the valid first start bit
                // WRITE_COMMAND("ask_syn", "ReceivingStartSequence");
                ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::ReceivingStartSequence;
                ask_syn_receiver.rx_active_byte = 0x00 | (new_value << 0);
                // ask_syn_receiver.rx_magic_sequence_position = 1;
            }
        } else {
            ask_syn_receiver.rx_magic_sequence_position = 0;
        }
    } else if (ask_syn_receiver.rx_state == AskSynReceiver::ReceiverState::ReceivingStartSequence) {
        // Receiving start sequence is also the synchronization step

        ask_syn_receiver.rx_active_byte = (ask_syn_receiver.rx_active_byte << 1) | (new_value << 0);

        // WRITE_COMMAND("ask_syn", "ReceivingStartSequence byte (%x ==== %x)", ask_syn_receiver.rx_active_byte, START_SEQUENCE_BYTE);

        if (ask_syn_receiver.rx_active_byte == START_SEQUENCE_BYTE) {
            WRITE_COMMAND("ask_syn", "ReceivingData");
            ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::ReceivingData;
            ask_syn_receiver.rx_magic_sequence_position = 0;
            ask_syn_receiver.rx_packet_received_bytes = 0;
            ask_syn_receiver.rx_active_bit_index = 0;
            ask_syn_receiver.rx_active_byte = 0x00;
        }
        // ask_syn_receiver.rx_active_byte |= new_value << (7 - ask_syn_receiver.rx_active_bit_index);

        // if (ask_syn_receiver.rx_magic_sequence_position < START_SEQUENCE_LENGTH) {
        //     if (ask_syn_receiver.rx_active_byte == START_SEQUENCE[ask_syn_receiver.rx_magic_sequence_position]) {
        //         ask_syn_receiver.rx_magic_sequence_position++;
        //     } else {
        //         // Invalid start sequence
        //         WRITE_COMMAND("ask_syn", "Idle (invalid start at position %d)", ask_syn_receiver.rx_magic_sequence_position);
        //         ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::Idle;
        //         ask_syn_receiver.rx_magic_sequence_position = 0;
        //     }
        // } else {
        //     // Received the full start sequence
        //     WRITE_COMMAND("ask_syn", "ReceivingData");
        //     ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::ReceivingData;
        //     ask_syn_receiver.rx_magic_sequence_position = 0;
        //     ask_syn_receiver.rx_packet_received_bytes = 0;
        //     ask_syn_receiver.rx_active_bit_index = 0;
        //     ask_syn_receiver.rx_active_byte = 0x00;
        // }

        // if (new_value == START_SEQUENCE[ask_syn_receiver.rx_magic_sequence_position]) {
        //     ask_syn_receiver.rx_magic_sequence_position++;

        //     if (ask_syn_receiver.rx_magic_sequence_position >= START_SEQUENCE_LENGTH) {
        //         WRITE_COMMAND("ask_syn", "ReceivingData");
        //         ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::ReceivingData;
        //         ask_syn_receiver.rx_magic_sequence_position = 0;
        //         ask_syn_receiver.rx_packet_received_bytes = 0;
        //         ask_syn_receiver.rx_active_bit_index = 0;
        //         ask_syn_receiver.rx_active_byte = 0x00;
        //     }
        // } else {
        //     WRITE_COMMAND("ask_syn", "Idle (invalid start at position %d)", ask_syn_receiver.rx_magic_sequence_position);
        //     ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::Idle;
        //     ask_syn_receiver.rx_magic_sequence_position = 0;
        // }
    } else if (ask_syn_receiver.rx_state == AskSynReceiver::ReceiverState::ReceivingData) {
        if (ask_syn_receiver.rx_packet_received_bytes < ASK_SYN_PACKET_LENGTH_BYTES) {
            // Receiving packed data

            ask_syn_receiver.rx_active_byte |= new_value << (7 - ask_syn_receiver.rx_active_bit_index);
            ask_syn_receiver.rx_active_bit_index++;

            if (ask_syn_receiver.rx_active_bit_index >= 8) {
                // WRITE_COMMAND("ask_syn", "ReceivingData byte [%d]=(%x)", ask_syn_receiver.rx_packet_received_bytes, ask_syn_receiver.rx_active_byte);
                ask_syn_receiver.rx_packet_buffer[ask_syn_receiver.rx_packet_received_bytes] = ask_syn_receiver.rx_active_byte;
                ask_syn_receiver.rx_active_byte = 0x00;
                ask_syn_receiver.rx_active_bit_index = 0;
                ask_syn_receiver.rx_packet_received_bytes++;
            }
        } else {
            // Received maximum number of bytes

            if (new_value == STOP_SEQUENCE[0]) {
                // WRITE_COMMAND("ask_syn", "ReceivingStopSequence");
                ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::ReceivingStopSequence;
                ask_syn_receiver.rx_magic_sequence_position = 1;
            } else {
                // Invalid stop sequence
                WRITE_COMMAND("ask_syn", "Idle (invalid stop)");
                ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::Idle;
            }
        }
    } else if (ask_syn_receiver.rx_state == AskSynReceiver::ReceiverState::ReceivingStopSequence) {
        if (new_value == STOP_SEQUENCE[ask_syn_receiver.rx_magic_sequence_position]) {
            ask_syn_receiver.rx_magic_sequence_position++;

            if (ask_syn_receiver.rx_magic_sequence_position >= STOP_SEQUENCE_LENGTH) {
                // Store the packet
                ask_syn_receiver._add_to_queue(ask_syn_receiver.rx_packet_buffer, ASK_SYN_PACKET_LENGTH_BYTES);

                ask_syn_receiver.rx_magic_sequence_position = 0;
                ask_syn_receiver.rx_packet_received_bytes = 0;
                ask_syn_receiver.rx_active_bit_index = 0;
                ask_syn_receiver.rx_active_byte = 0x00;

                WRITE_COMMAND("ask_syn", "Idle (success)");
                ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::Idle;
            }
        } else {
            WRITE_COMMAND("ask_syn", "Idle (invalid stop at position %d)", ask_syn_receiver.rx_magic_sequence_position);
            ask_syn_receiver.rx_state = AskSynReceiver::ReceiverState::Idle;
            ask_syn_receiver.rx_magic_sequence_position = 0;
        }
    }
}

void receive_interrupt_handler(uint8_t gpio_number) {
    static TickType_t current_interrupt_time = 0;

    if (gpio_number == BOARD_PIN_SYN470R_RXD) {
        current_interrupt_time = xTaskGetTickCountFromISR();

        // Debounce the signal
        if (current_interrupt_time - ask_syn_receiver.rx_last_change_time > pdMS_TO_TICKS(ASK_SYN_BIT_DURATION_MS)*3/4) {
            bool new_value = gpio_read(gpio_number) == 0x00;
            uint8_t value = 0 | (ask_syn_receiver.rx_last_value << 1) | (new_value);

            ask_syn_receiver._add_to_bits_queue(value, current_interrupt_time - ask_syn_receiver.rx_last_change_time);

            ask_syn_receiver.rx_last_value = new_value;
            ask_syn_receiver.rx_last_change_time = current_interrupt_time;
        }
    }
}
