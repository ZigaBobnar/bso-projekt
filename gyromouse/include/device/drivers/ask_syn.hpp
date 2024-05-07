#pragma once

#include "common.hpp"

// Packet physical structure:
// TODO: Variable length using different start bytes
// TODO: Checksum instead of stop byte (should it be of whole packet, include start, error correction capabilities? ...?)
// | Idle           | |  Start | |     Data (n bytes)   | |  Stop  | | Idle
// 0 .............. 0 1  0  1  0 dddddddddddddddddddddddd 0  1  0  1 0..........
// |                | |        | |                      | |        | |
#define ASK_SYN_BIT_DURATION_MS 10
#define ASK_SYN_PACKET_LENGTH_BYTES 6
// #define ASK_SYN_PACKET_LENGTH_BYTES 18 /* Space for 18 uint8_t or 9 uint16_t or 4 uint32_t or 4 float */
/* Beter usage is by using:
 *  1 byte -> address
 *  4 bytes -> payload
 *  1 byte -> checksum
 */
#define ASK_SYN_MIN_IDLE_DURATION_BITS 24
// #define ASK_SYN_TRANSMIT_BUFFER_SIZE (ASK_SYN_PACKET_LENGTH_BYTES * 2)
#define ASK_SYN_RECEIVE_BUFFER_SIZE 3
//(ASK_SYN_PACKET_LENGTH_BYTES * 3)

#define ASK_SYN_TX_WAIT_FOR_PACKET_SLEEP_DURATION_MS 30

#define ASK_BIT_QUEUE 512


// Start with 1111 so the driver auto corrects its gain so there are less false positives
const uint8_t START_SEQUENCE_BYTE = 0b11110101;
const uint8_t STOP_SEQUENCE[] = {1, 0, 1, 0, 1, 1, 1, 1};
const uint8_t STOP_SEQUENCE_LENGTH = 8;


class AskSynTransmitter {
public:
    AskSynTransmitter(uint8_t tx_pin);

    void init();

    void send_packet(uint8_t *data, size_t length);

public:
    uint8_t tx_pin;

    SemaphoreHandle_t tx_mutex;

    uint8_t tx_raw_buffer[ASK_SYN_PACKET_LENGTH_BYTES * 2]; // Contains one prefill and one active sending buffer
    uint8_t *tx_prefill_buffer;
    uint8_t *tx_active_buffer;

    bool tx_active_buffer_full = false;
    bool tx_prefill_buffer_full = false;
    
    TickType_t tx_last_transmit_time = 0;
};

extern AskSynTransmitter ask_syn_transmitter;

void ask_syn_transmitter_task(void *pvParameters);


class AskSynReceiver {
public:
    AskSynReceiver(uint8_t rx_pin);

    void init();
    void pause_interrupt();
    void resume_interrupt();

    bool has_data();
    uint8_t available_data_count();
    void _add_to_queue(uint8_t *data, size_t length);
    uint8_t *get_from_queue();

    uint8_t rx_pin;

    uint8_t rx_buffer[ASK_SYN_RECEIVE_BUFFER_SIZE][ASK_SYN_PACKET_LENGTH_BYTES/*ASK_SYN_RECEIVE_BUFFER_SIZE*/];
    size_t rx_buffer_head;
    size_t rx_buffer_tail;
    size_t rx_buffer_size;

    // Data for interrupt handling
    enum class ReceiverState {
        Idle, // Means we wait for at least X bits of time with state always being 0 before we start receiving
        ReceivingStartSequence,
        ReceivingData,
        ReceivingStopSequence,
    } rx_state = ReceiverState::Idle;
    bool rx_last_value = false;
    TickType_t rx_last_change_time = 0;
    uint8_t rx_packet_buffer[ASK_SYN_PACKET_LENGTH_BYTES];
    uint8_t rx_packet_received_bytes = 0;
    uint8_t rx_active_byte = 0x00;
    uint8_t rx_active_bit_index = 0;
    uint8_t rx_magic_sequence_position = 0;
    
    struct received_bits_t {
        uint8_t data;
        TickType_t time_delta;
    } rx_received_bits_queue[ASK_BIT_QUEUE];
    size_t rx_received_bits_head;
    size_t rx_received_bits_tail;
    
    bool _has_bits_to_process();
    void _add_to_bits_queue(uint8_t data, TickType_t time);    
    received_bits_t _get_from_bits_queue();
};

extern AskSynReceiver ask_syn_receiver;


void ask_syn_receiver_raw_processing_task(void *pvParameters);
void receive_interrupt_handler(uint8_t gpio_number);
