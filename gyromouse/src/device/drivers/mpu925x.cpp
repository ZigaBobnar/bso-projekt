#include "device/drivers/mpu925x.hpp"

#define INT16_FLOAT_SCALE 32768.0

#define MPU925x_FIFO_SIZE 512

uint8_t MPU925x_FIFO[MPU925x_FIFO_SIZE];
uint8_t *MPU925x_FIFO_ptr = MPU925x_FIFO;

MPU925x::MPU925x(I2CDevice& mpu_device, I2CDevice& ak_device) : mpu_device(mpu_device), ak_device(ak_device) {}

inline uint16_t unpack_word_be(const uint8_t* buffer) {
    return (uint16_t)buffer[0] << 8 | buffer[1];
}

inline void pack_word_be(uint16_t value, uint8_t* buffer) {
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;
}

inline uint16_t unpack_word_le(const uint8_t* buffer) {
    return (uint16_t)buffer[1] << 8 | buffer[0];
}

inline void pack_word_le(uint16_t value, uint8_t* buffer) {
    buffer[0] = value & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
}

void MPU925x::init() {
    initialize_start();
    DEBUG_COMMAND("debug", "MPU925x::init[start]");

    mpu_device.get_bus().init();

    // Full reset
    hardware_reset();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    select_clock_source(MPUClockSource::Internal_20MHz);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    select_clock_source(MPUClockSource::AutoSelect);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    write_register(MPU925x_Register::CONFIG, 0x03); // DLPF_CFG = 3
    set_accelerometer_range(accelerometer_range);
    set_gyro_range(gyro_range);

    // Sample rate = Internal sample rate / (1 + SMPLRT_DIV)
    // set_samplerate_divider(0x04); // fs = fs_int=1kHz / (1 + div) = 200Hz sample rate
    set_samplerate_divider(0x10); // fs = fs_int=1kHz / (1 + div) = 50Hz sample rate

    // Enable FIFO
    // enable_fifo();
    // enable_fifo_module(MPUFifoEnable::Temperature | MPUFifoEnable::GyroX | MPUFifoEnable::GyroY | MPUFifoEnable::GyroZ | MPUFifoEnable::Accelerometer);
    // enable_fifo_module(MPUFifoEnable::Temperature);
    // enable_fifo_module(MPUFifoEnable::Temperature | MPUFifoEnable::Accelerometer);
    
    // set_accelerometer_motion_detection(true, true);
    // set_enabled_interrupts(MPUInterrupt::WakeOnMotion | MPUInterrupt::FifoOverflow | MPUInterrupt::RawDataReady);

    // Initialize AK8963
    enable_i2c_bypass();
    magnetometer_soft_reset();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // magnetometer_set_mode_continuous_measurement_8Hz();
    magnetometer_set_mode_continuous_measurement_100Hz();
    magnetometer_set_16_bit_mode();
    XYZUint8 magnetometer_sensitivities = magnetometer_get_sensitivity_adjustment();
    this->magnetometer_scale_factors = {
        (float)(magnetometer_sensitivities.x - 128) / 256.0f + 1.0f,
        (float)(magnetometer_sensitivities.y - 128) / 256.0f + 1.0f,
        (float)(magnetometer_sensitivities.z - 128) / 256.0f + 1.0f,
    };

    DEBUG_COMMAND("debug", "MPU925x::init[end]");
    initialize_end();
}

uint8_t MPU925x::read_register(MPU925x_Register register_address) {
    return mpu_device.read_byte((uint8_t)register_address);
}

uint16_t MPU925x::read_word_registers(MPU925x_Register h_register_address) {
    uint8_t data[2];
    mpu_device.read_bytes((uint8_t)h_register_address, 2, data);
    
    return unpack_word_be(data);
}

void MPU925x::read_registers(MPU925x_Register start_register_address, uint8_t count, uint8_t* buffer_out) {
    mpu_device.read_bytes((uint8_t)start_register_address, count, buffer_out);
}

void MPU925x::write_register(MPU925x_Register register_address, uint8_t value) {
    mpu_device.write_byte((uint8_t)register_address, value);
}

void MPU925x::write_registers(MPU925x_Register start_register_address, uint8_t count, const uint8_t* buffer_in) {
    mpu_device.write_bytes((uint8_t)start_register_address, count, buffer_in);
}

uint8_t MPU925x::read_ak_register(AK8963_Register register_address) {
    return ak_device.read_byte((uint8_t)register_address);
}

void MPU925x::read_ak_registers(AK8963_Register start_register_address, uint8_t count, uint8_t* buffer_out) {
    ak_device.read_bytes((uint8_t)start_register_address, count, buffer_out);
}

void MPU925x::write_ak_register(AK8963_Register register_address, uint8_t value) {
    ak_device.write_byte((uint8_t)register_address, value);
}

void MPU925x::write_ak_registers(MPU925x_Register start_register_address, uint8_t count, const uint8_t* buffer_in) {
    ak_device.write_bytes((uint8_t)start_register_address, count, buffer_in);
}

uint8_t MPU925x::mpu_who_am_i() {
    return read_register(MPU925x_Register::WHO_AM_I);
}

uint8_t MPU925x::ak_who_am_i() {
    return read_ak_register(AK8963_Register::WIA);
}

XYZFloat MPU925x::get_acceleration_values() {
    uint8_t packed[6];
    read_registers(MPU925x_Register::ACCEL_XOUT_H, 6, packed);

    return {
        (float)(int16_t)unpack_word_be(packed) * accelerometer_scale_factor,
        (float)(int16_t)unpack_word_be(packed + 2) * accelerometer_scale_factor,
        (float)(int16_t)unpack_word_be(packed + 4) * accelerometer_scale_factor,
    };
}

XYZFloat MPU925x::get_gyroscope_values() {
    uint8_t packed[6];
    read_registers(MPU925x_Register::GYRO_XOUT_H, 6, packed);

    return {
        (float)(int16_t)unpack_word_be(packed) * gyro_scale_factor,
        (float)(int16_t)unpack_word_be(packed + 2) * gyro_scale_factor,
        (float)(int16_t)unpack_word_be(packed + 4) * gyro_scale_factor,
    };
}

MPUMagnetometerReading MPU925x::get_magnetometer_values() {
    // Apparently we need to read 8 bytes from the module
    uint8_t packed[8];
    // read_ak_registers(AK8963_Register::H_X_L, 8, packed);
    read_registers(MPU925x_Register::EXT_SENS_DATA_00, 8, packed);
    
    return {
        .values = {
            (int16_t)unpack_word_le(packed) * magnetometer_scale_factors.x,
            (int16_t)unpack_word_le(packed + 2) * magnetometer_scale_factors.y,
            (int16_t)unpack_word_le(packed + 4) * magnetometer_scale_factors.z,
        },
        .is_16_bit = packed[6] & (1 << 4),
        .had_overflow = packed[6] & (1 << 3),
    };
}

float MPU925x::get_temperature_value() {
    int16_t temperature = (int16_t)read_word_registers(MPU925x_Register::TEMP_OUT_H);

    // Magic constants
    static const float room_temperature_offset = 0;
    static const float temperature_sensitivity = 333.87;
    static const float room_temperature_celsius = 21;

    return ((float)temperature - room_temperature_offset) / temperature_sensitivity + room_temperature_celsius;
}

XYZInt16 MPU925x::get_gyroscope_offsets() {
    uint8_t packed[6];

    read_registers(MPU925x_Register::XG_OFFSET_H, 6, packed);

    return {
        unpack_word_be(packed) << 1,
        unpack_word_be(packed + 2) << 1,
        unpack_word_be(packed + 4) << 1,
    };
}

XYZInt16 MPU925x::get_acccelerometer_offsets() {
    uint8_t packed[6];

    read_registers(MPU925x_Register::XA_OFFSET_H, 6, packed);

    return {
        unpack_word_be(packed) >> 1,
        unpack_word_be(packed + 2) >> 1,
        unpack_word_be(packed + 4) >> 1,
    };
}

void MPU925x::set_gyroscope_offsets(XYZInt16 offsets) {
    uint8_t packed[6];
    pack_word_be(offsets.x, packed);
    pack_word_be(offsets.y, packed + 2);
    pack_word_be(offsets.z, packed + 4);

    write_registers(MPU925x_Register::XG_OFFSET_H, 6, packed);
}

void MPU925x::set_accelerometer_offsets(XYZInt16 offsets) {
    uint8_t packed[6];
    pack_word_be(offsets.x, packed);
    pack_word_be(offsets.y, packed + 2);
    pack_word_be(offsets.z, packed + 4);

    write_registers(MPU925x_Register::XA_OFFSET_H, 6, packed);
}

uint8_t MPU925x::get_sample_rate_divider() {
    return read_register(MPU925x_Register::SMPLRT_DIV);
}

void MPU925x::set_samplerate_divider(uint8_t divider) {
    write_register(MPU925x_Register::SMPLRT_DIV, divider);
}

void MPU925x::set_accelerometer_range(MPUAccelerometerRange range) {
    this->accelerometer_range = range;
    this->accelerometer_scale_factor = (
        range == MPUAccelerometerRange::Accel_2_G ? 2.0 :
        range == MPUAccelerometerRange::Accel_4_G ? 4.0 :
        range == MPUAccelerometerRange::Accel_8_G ? 8.0 :
        range == MPUAccelerometerRange::Accel_16_G ? 8.0 :
        1.0) / INT16_FLOAT_SCALE;

    uint8_t accel_config = read_register(MPU925x_Register::ACCEL_CONFIG);
    accel_config &= 0b11100111;
    accel_config |= (((uint8_t)range) << 3) & 0b00011000;
    write_register(MPU925x_Register::ACCEL_CONFIG, accel_config);
}

void MPU925x::set_gyro_range(MPUGyroRange range) {
    this->gyro_range = range;
    this->gyro_scale_factor = (
        range == MPUGyroRange::Gyro_250_DegPerSec ? 250.0 :
        range == MPUGyroRange::Gyro_500_DegPerSec ? 500.0 :
        range == MPUGyroRange::Gyro_1000_DegPerSec ? 1000.0 :
        range == MPUGyroRange::Gyro_2000_DegPerSec ? 2000.0 :
        1.0) / INT16_FLOAT_SCALE;

    uint8_t gyro_config = read_register(MPU925x_Register::GYRO_CONFIG);
    gyro_config &= 0b11100111;
    gyro_config |= (((uint8_t)range) << 3) & 0b00000011;
    write_register(MPU925x_Register::GYRO_CONFIG, gyro_config);
}

// void MPU925x::set_gyro_bandwidth(MPUGyroFilterBandwidth bandwidth) {

// }

// MPUGyroFilterBandwidth MPU925x::get_gyro_bandwidth() {
//     uint8_t dlpf = read_register(MPU925x_Register::CONFIG) & 0b00000111;
//     uint8_t fchoice_b = read_register(MPU925x_Register::GYRO_CONFIG) & 0b00000011;
//     uint8_t fchoice = ~fchoice_b & 0b00000011;

//     if (fchoice & 1 << 0) {
//         return MPUGyroFilterBandwidth::Bandwidth_8800_Hz_Fs_32kHz;
//     } else if (fchoice & 1 << 1) {
//         return MPUGyroFilterBandwidth::Bandwidth_3600_Hz_Fs_32kHz;
//     }



    
// }

// void MPU925x::set_accelerometer_bandwidth(MPUAccelerometerFilterBandwidth bandwidth) {

// }

// MPUAccelerometerFilterBandwidth MPU925x::get_accelerometer_bandwidth() {

// }


MPUFifoEnable MPU925x::get_fifo_enabled_modules() {
    return static_cast<MPUFifoEnable>(read_register(MPU925x_Register::FIFO_EN));
}

bool MPU925x::is_fifo_module_enabled(MPUFifoEnable module) {
    return (get_fifo_enabled_modules() & module) == module;
}

void MPU925x::enable_fifo_module(MPUFifoEnable module) {
    write_register(MPU925x_Register::FIFO_EN, (uint8_t)(get_fifo_enabled_modules() | module));
}

void MPU925x::disable_fifo_module(MPUFifoEnable module) {
    write_register(MPU925x_Register::FIFO_EN, (uint8_t)(get_fifo_enabled_modules() & ~module));
}

bool MPU925x::is_i2c_bypass_enabled() {
    return read_register(MPU925x_Register::INT_PIN_CFG) & (1 << 1);
}

void MPU925x::enable_i2c_bypass() {
    write_register(MPU925x_Register::INT_PIN_CFG, read_register(MPU925x_Register::INT_PIN_CFG) | (1 << 1));
}

void MPU925x::disable_i2c_bypass() {
    write_register(MPU925x_Register::INT_PIN_CFG, read_register(MPU925x_Register::INT_PIN_CFG) & ~(1 << 1));
}

MPUInterrupt MPU925x::get_enabled_interrupts() {
    return static_cast<MPUInterrupt>(read_register(MPU925x_Register::INT_ENABLE));
}

void MPU925x::set_enabled_interrupts(MPUInterrupt interrupts) {
    write_register(MPU925x_Register::INT_ENABLE, (uint8_t)interrupts);
}

MPUInterrupt MPU925x::get_interrupt_statuses() {
    return static_cast<MPUInterrupt>(read_register(MPU925x_Register::INT_STATUS));
}

void MPU925x::set_accelerometer_motion_detection(bool wake_on_motion_detection, bool compare_with_previous_sample) {
    uint8_t config = 0;
    config |= wake_on_motion_detection ? (1 << 6) : 0;
    config |= compare_with_previous_sample ? (1 << 7) : 0;

    write_register(MPU925x_Register::MOT_DETECT_CTRL, config);
}

bool MPU925x::is_fifo_enabled() {
    return read_register(MPU925x_Register::USER_CTRL) & (1 << 6);
}

void MPU925x::enable_fifo() {
    write_register(MPU925x_Register::USER_CTRL, read_register(MPU925x_Register::USER_CTRL) | (1 << 6));
}

void MPU925x::disable_fifo() {
    write_register(MPU925x_Register::USER_CTRL, read_register(MPU925x_Register::USER_CTRL) & ~(1 << 6));
}

void MPU925x::hardware_reset() {
    write_register(MPU925x_Register::PWR_MGMT_1, 1 << 7);
}

void MPU925x::put_into_sleep() {
    write_register(MPU925x_Register::PWR_MGMT_1, read_register(MPU925x_Register::PWR_MGMT_1) | (1 << 6));
}

void MPU925x::wake_up() {
    write_register(MPU925x_Register::PWR_MGMT_1, read_register(MPU925x_Register::PWR_MGMT_1) & ~(1 << 6));
}

void MPU925x::gyros_standby_mode() {
    write_register(MPU925x_Register::PWR_MGMT_1, read_register(MPU925x_Register::PWR_MGMT_1) | (1 << 4));
}

void MPU925x::wake_gyros() {
    write_register(MPU925x_Register::PWR_MGMT_1, read_register(MPU925x_Register::PWR_MGMT_1) & ~(1 << 4));
}

void MPU925x::select_clock_source(MPUClockSource source) {
    write_register(MPU925x_Register::PWR_MGMT_1, (read_register(MPU925x_Register::PWR_MGMT_1) & 0b11111000) | (uint8_t)source);
}

XYZBool MPU925x::accelerometer_enabled_axis() {
    uint8_t enabled = read_register(MPU925x_Register::PWR_MGMT_2);

    return {
        enabled & (1 << 5),
        enabled & (1 << 4),
        enabled & (1 << 3),
    };
}

XYZBool MPU925x::gyroscope_enabled_axis() {
    uint8_t enabled = read_register(MPU925x_Register::PWR_MGMT_2);
    
    return {
        enabled & (1 << 2),
        enabled & (1 << 1),
        enabled & (1 << 0),
    };
}

void MPU925x::set_enabled_module_axis(XYZBool accelerometer_enabled, XYZBool gyroscope_enabled) {
    uint8_t enabled = 0;
    enabled |= accelerometer_enabled.x ? (1 << 5) : 0;
    enabled |= accelerometer_enabled.y ? (1 << 4) : 0;
    enabled |= accelerometer_enabled.z ? (1 << 3) : 0;
    enabled |= gyroscope_enabled.x ? (1 << 2) : 0;
    enabled |= gyroscope_enabled.y ? (1 << 1) : 0;
    enabled |= gyroscope_enabled.z ? (1 << 0) : 0;

    write_register(MPU925x_Register::PWR_MGMT_2, enabled);
}

uint16_t MPU925x::get_fifo_count() {
    return read_word_registers(MPU925x_Register::FIFO_COUNTH) & 0x7FFF;
}

uint8_t MPU925x::read_fifo_byte() {
    return read_register(MPU925x_Register::FIFO_R_W);
}

void MPU925x::read_fifo_bytes(uint8_t count, uint8_t* buffer_out) {
    I2CDeviceTransaction transaction = mpu_device.begin_transaction();
    for (uint8_t i = 0; i < count; i++) {
        transaction.read_byte((uint8_t)MPU925x_Register::FIFO_R_W);
    }
}

bool MPU925x::magnetometer_has_data_ready() {
    return read_ak_register(AK8963_Register::ST1) & 1 << 0;
}

bool MPU925x::magnetometer_was_data_overrun() {
    return read_ak_register(AK8963_Register::ST1) & 1 << 1;
}

void MPU925x::magnetometer_set_mode_power_down() {
    write_ak_register(AK8963_Register::CNTL1, (read_ak_register(AK8963_Register::CNTL1) & 0b11110000) | 0b0000);
}

void MPU925x::magnetometer_set_mode_single_measurement() {
    write_ak_register(AK8963_Register::CNTL1, (read_ak_register(AK8963_Register::CNTL1) & 0b11110000) | 0b0001);
}

void MPU925x::magnetometer_set_mode_continuous_measurement_8Hz() {
    write_ak_register(AK8963_Register::CNTL1, (read_ak_register(AK8963_Register::CNTL1) & 0b11110000) | 0b0010);
}

void MPU925x::magnetometer_set_mode_continuous_measurement_100Hz() {
    write_ak_register(AK8963_Register::CNTL1, (read_ak_register(AK8963_Register::CNTL1) & 0b11110000) | 0b0110);
}

void MPU925x::magnetometer_set_mode_external_trigger_measurement() {
    write_ak_register(AK8963_Register::CNTL1, (read_ak_register(AK8963_Register::CNTL1) & 0b11110000) | 0b0100);
}

void MPU925x::magnetometer_set_16_bit_mode() {
    write_ak_register(AK8963_Register::CNTL1, read_ak_register(AK8963_Register::CNTL1) | (1 << 4));
}

void MPU925x::magnetometer_set_14_bit_mode() {
    write_ak_register(AK8963_Register::CNTL1, read_ak_register(AK8963_Register::CNTL1) & ~(1 << 4));
}

void MPU925x::magnetometer_soft_reset() {
    write_ak_register(AK8963_Register::CNTL2, 1 << 0);
}

XYZUint8 MPU925x::magnetometer_get_sensitivity_adjustment() {
    uint8_t packed[3];

    read_ak_registers(AK8963_Register::ASAX, 3, packed);

    return {
        packed[0],
        packed[1],
        packed[2],
    };
}

// uint8_t MPU925x::read_fifo_bytes() {
//     if (MPU925x_FIFO_ptr == MPU925x_FIFO) {
//         uint16_t count = get_fifo_count();
//         if (count == 0) {
//             return 0;
//         }

//         uint8_t to_read = count > MPU925x_FIFO_SIZE ? MPU925x_FIFO_SIZE : count;
//         read_registers(MPU925x_Register::FIFO_R_W, to_read, MPU925x_FIFO);
//         MPU925x_FIFO_ptr = MPU925x_FIFO;
//     }

//     return *MPU925x_FIFO_ptr++;

// }
