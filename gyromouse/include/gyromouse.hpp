#pragma once

#include "common.hpp"
#include "device/io.hpp"

// Shared runtime utilities for the gyro mouse project

class GyroMouse {
public:
    enum class Mode {
        DONGLE,
        MOUSE,
        // HARDWIRED_MOUSE,
    } mode = Mode::MOUSE;


    // Default state, can be overriden with console commands
    bool debugging_enabled = false;


    struct IMUConfig {
        bool paused = false;
        int update_interval_ms = IMU_UPDATE_INTERVAL_DEFAULT;
    } imu_config;


    struct IOConfig {
        int leds_update_interval_ms = LEDS_UPDATE_INTERVAL_DEFAULT;
        int buttons_update_interval_ms = BUTTONS_UPDATE_INTERVAL_DEFAULT;
    } io_config;


    struct TaskHandles {
        // TaskHandle_t ask_syn_transmitter = nullptr;
        TaskHandle_t watchdog = nullptr;
        TaskHandle_t device_imu_update = nullptr;
        TaskHandle_t wireless_mouse_process_data = nullptr;
        TaskHandle_t process_serial_commands = nullptr;
        TaskHandle_t dongle_send_ping = nullptr;

        TaskHandle_t ask_syn_transmitter = nullptr;
    } tasks;


    struct Timings {
        TickType_t last_watchdog_check; // Periodic checks

        TickType_t last_drive_mode_keepalive_message; // Connection to the computer
        TickType_t last_ping_request; // Connection to the mouse (sent from dongle to mouse)
        TickType_t last_ping_requested; // Connection to the dongle (received from dongle to mouse)
        TickType_t last_ping_response; // Connection to the dongle (sent from mouse to dongle)

        TickType_t last_imu_update; // IMU update

    } timings;

    void init_all();

    void on_button_state_change(int button, bool new_button_state);

    void put_into_dongle_mode();
    void put_into_mouse_mode();
};

extern GyroMouse gyromouse;
