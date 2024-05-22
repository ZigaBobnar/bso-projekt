#include "common.hpp"

#include "device/imu.hpp"
// #include "device/drivers/ask_syn.hpp"
#include "device/wireless.hpp"

void task_device_imu_update(void *pvParameters) {
    TickType_t current_time = gyromouse.timings.last_imu_update = xTaskGetTickCount();

    DEBUG_COMMAND("debug", "task_device_imu_update[init]");
    imu.init();

    WRITE_COMMAND("mpu_who_am_i", "0x%x", imu.mpu_who_am_i);
    WRITE_COMMAND("ak_who_am_i", "0x%x", imu.ak_who_am_i);
    // WRITE_COMMAND("accel_test", "%f,%f,%f", imu.accel_test.x, imu.accel_test.y, imu.accel_test.z);
    // WRITE_COMMAND("gyro_test", "%f,%f,%f", imu.gyro_test.x, imu.gyro_test.y, imu.gyro_test.z);

    DEBUG_COMMAND("debug", "task_device_imu_update[loop]");
    while (true) {
        if (gyromouse.imu_config.paused) {
            vTaskDelayUntil(&current_time, gyromouse.imu_config.update_interval_ms / portTICK_PERIOD_MS);
            continue;
        }
        
        imu.update();

        // Calculate kinematics of mouse
        int16_t mosue_deltas[2] = {0, 0};
        mosue_deltas[0] = - imu.gyroscope.z * 1000;
        mosue_deltas[1] = - imu.gyroscope.y * 1000;

        WRITE_COMMAND("update", "start");
        WRITE_COMMAND("dt", "%d", get_time_delta(gyromouse.timings.last_imu_update, current_time));
        WRITE_COMMAND("accel", "%f,%f,%f", imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z);
        WRITE_COMMAND("gyro", "%f,%f,%f", imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z);
        WRITE_COMMAND("mag", "%f,%f,%f", imu.magnetometer.values.x, imu.magnetometer.values.y, imu.magnetometer.values.z);
        if (!imu.magnetometer.is_16_bit) {
            WRITE_COMMAND("error", "Magnetometer not in 16-bit mode");
        }
        if (imu.magnetometer.had_overflow) {
            WRITE_COMMAND("error", "Magnetometer overflowed");
        }
        WRITE_COMMAND("temp", "%f", imu.temperature);
        WRITE_COMMAND("mouse", "%d,%d", mosue_deltas[0], mosue_deltas[1]);
        WRITE_COMMAND("update", "done");


        int16_t mouse_move[2] = { mosue_deltas[0], mosue_deltas[1] };
        wireless.write_packet(WirelessCommand::Mouse_Move_Int16_XY, (uint8_t*)mouse_move, 4);


        // uint8_t data_to_send[12];
        // uint32_t packed_float;
        // // Pack accelerometer and gyroscope into data_to_send
        // data_to_send[0] = 0xa0; // Accelerometer data
        // packed_float = *(uint32_t*)(void*)&imu.accelerometer.x; // 4b
        // data_to_send[1] = packed_float & 0xff;
        // data_to_send[2] = (packed_float >> 8) & 0xff;
        // data_to_send[3] = (packed_float >> 16) & 0xff;
        // data_to_send[4] = (packed_float >> 24) & 0xff;
        // packed_float = *(uint32_t*)(void*)&imu.accelerometer.x; // 4b
        // data_to_send[5] = packed_float & 0xff;
        // data_to_send[6] = (packed_float >> 8) & 0xff;
        // data_to_send[7] = (packed_float >> 16) & 0xff;
        // data_to_send[8] = (packed_float >> 24) & 0xff;
        // packed_float = *(uint32_t*)(void*)&imu.accelerometer.x; // 4b -> 12b
        // data_to_send[9] = packed_float & 0xff;
        // data_to_send[10] = (packed_float >> 8) & 0xff;
        // data_to_send[11] = (packed_float >> 16) & 0xff;
        // data_to_send[12] = (packed_float >> 24) & 0xff;
        // // ask_syn_transmitter.send_packet(data_to_send, 12);

        // if (!ask_syn_transmitter.tx_prefill_buffer_full) {
        //     WRITE_COMMAND("ask_syn", "Sending gyro data");
        //     data_to_send[0] = 0xa1; // Gyroscope data
        //     packed_float = *(uint32_t*)(void*)&imu.gyroscope.x; // 4b
        //     data_to_send[1] = packed_float & 0xff;
        //     data_to_send[2] = (packed_float >> 8) & 0xff;
        //     data_to_send[3] = (packed_float >> 16) & 0xff;
        //     data_to_send[4] = (packed_float >> 24) & 0xff;
        //     packed_float = *(uint32_t*)(void*)&imu.gyroscope.y; // 4b
        //     data_to_send[5] = packed_float & 0xff;
        //     data_to_send[6] = (packed_float >> 8) & 0xff;
        //     data_to_send[7] = (packed_float >> 16) & 0xff;
        //     data_to_send[8] = (packed_float >> 24) & 0xff;
        //     packed_float = *(uint32_t*)(void*)&imu.gyroscope.z; // 4b -> 12b
        //     data_to_send[9] = packed_float & 0xff;
        //     data_to_send[10] = (packed_float >> 8) & 0xff;
        //     data_to_send[11] = (packed_float >> 16) & 0xff;
        //     data_to_send[12] = (packed_float >> 24) & 0xff;
        //     ask_syn_transmitter.send_packet(data_to_send, 12);
        // }

        // if (!ask_syn_transmitter.tx_prefill_buffer_full) {
        //     // WRITE_COMMAND("ask_syn", "Sending mouse data");
        //     uint8_t data_to_send[6];
        //     data_to_send[0] = 0xa2; // Mouse data
        //     data_to_send[1] = mosue_deltas[0] & 0xff;
        //     data_to_send[2] = (mosue_deltas[0] >> 8) & 0xff;
        //     data_to_send[3] = mosue_deltas[1] & 0xff;
        //     data_to_send[4] = (mosue_deltas[1] >> 8) & 0xff;
        //     data_to_send[5] = 0x00;
        //     ask_syn_transmitter.send_packet(data_to_send, 6);
        // } else {
        //     WRITE_COMMAND("error", "ask_syn_transmitter buffer full");
        // }

        gyromouse.timings.last_imu_update = current_time;

        // Wait for next read
        vTaskDelayUntil(&current_time, gyromouse.imu_config.update_interval_ms / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
