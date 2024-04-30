#include "common.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include <esp/uart.h>
#include <string.h>

#define BUFFER_SIZE 512
char input_command_line_buffer[BUFFER_SIZE];
char *buffer_position = &input_command_line_buffer[0];

inline int uart_rxfifo_available_count(int uart_num) {
    return FIELD2VAL(UART_STATUS_RXFIFO_COUNT, UART(uart_num).STATUS);
}

inline char uart_rxfifo_get_char(int uart_num) {
    return UART(uart_num).FIFO;
}

inline bool starts_with(const char *str, const char *prefix) {
    return strncmp(str, prefix, strlen(prefix)) == 0;
}

inline bool string_equals(const char *a, const char *b) {
    return strcmp(a, b) == 0;
}

void task_serial_commands(void *pvParameters) {
    while (1) {
        int count = uart_rxfifo_available_count(0);
        if (count == 0) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }
        
        while (uart_rxfifo_available_count(0) > 0) {
            char c = uart_rxfifo_get_char(0);

            if (c == '\n') {
                *buffer_position = '\0';
                buffer_position = &input_command_line_buffer[0];

                if (string_equals(input_command_line_buffer, "$get_info=firmware")) {
                    WRITE_COMMAND("info", "gyromouse v0.1");
                } else if (starts_with(input_command_line_buffer, "$debug_enabled=")) {
                    char *state = &input_command_line_buffer[15];
                    if (string_equals(state, "true")) {
                        debugging_enabled = true;
                    } else if (string_equals(state, "false")) {
                        debugging_enabled = false;
                    } else {
                        WRITE_COMMAND("error", "invalid_debug_state %s", state);
                    }
                } else if (starts_with(input_command_line_buffer, "$imu_state=")) {
                    char *state = &input_command_line_buffer[11];
                    if (string_equals(state, "start")) {
                        DEBUG_COMMAND("debug", "imu starting");
                        imu_paused = false;
                    } else if (string_equals(state, "stop")) {
                        DEBUG_COMMAND("debug", "imu stopping");
                        imu_paused = true;
                    } else {
                        WRITE_COMMAND("error", "invalid_imu_state %s", state);
                    }
                } else {
                    WRITE_COMMAND("error", "unknown_command %s", input_command_line_buffer);
                }

                continue;
            } else {
                if (buffer_position - &input_command_line_buffer[0] >= 512) {
                    WRITE_COMMAND("error", "command_buffer_overflow");
                    buffer_position = &input_command_line_buffer[0];
                }
                *buffer_position = c;
                buffer_position++;
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
        continue;
    }

    vTaskDelete(NULL);
}
