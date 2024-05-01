#pragma once

#include "common.hpp"
#include "device/drivers/i2c.hpp"
#include "device/utils/xyzfloat.hpp"
#include "device/utils/initializable.hpp"

// Documentation at:
// https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf

// Inspired by
// https://github.com/Bill2462/MPU9255-Arduino-Library/

// https://github.com/asukiaaa/MPU9250_asukiaaa/issues/22#issuecomment-1746065219
// https://web.stanford.edu/class/ee267/misc/MPU-9255-Datasheet.pdf
// https://web.stanford.edu/class/ee267/misc/MPU-9255-Register-Map.pdf
// It seems based on received 0x73 whoami that this module is actually MPU-9255

// Packing: uint8_t = [ Bit 7, 6, 5, 4, 3, 2, 1, Bit 0 ]

enum class MPU925x_Register : uint8_t {
    SELF_TEST_X_GYRO    = 0x00, /* R/W xg_st_data[7:0] */
    SELF_TEST_Y_GYRO    = 0x01, /* R/W yg_st_data[7:0] */
    SELF_TEST_Z_GYRO    = 0x02, /* R/W zg_st_data[7:0] */

    SELF_TEST_X_ACCEL   = 0x0D, /* R/W xa_st_data[7:0] */
    SELF_TEST_Y_ACCEL   = 0x0E, /* R/W ya_st_data[7:0] */
    SELF_TEST_Z_ACCEL   = 0x0F, /* R/W za_st_data[7:0] */

    XG_OFFSET_H         = 0x13, /* R/W X_OFFS_USR[15:8] */
    XG_OFFSET_L         = 0x14, /* R/W X_OFFS_USR[7:0] */
    YG_OFFSET_H         = 0x15, /* R/W Y_OFFS_USR[15:8] */
    YG_OFFSET_L         = 0x16, /* R/W Y_OFFS_USR[7:0] */
    ZG_OFFSET_H         = 0x17, /* R/W Z_OFFS_USR[15:8] */
    ZG_OFFSET_L         = 0x18, /* R/W Z_OFFS_USR[7:0] */
    SMPLRT_DIV          = 0x19, /* R/W SMPLRT_DIV[7:0] */

    CONFIG              = 0x1A, /* R/W [ 7=_ 6=FIFO_MODE 5:3=EXT_SYNC_SET[2:0] 2:0=DLPF_CFG[2:0] */
    GYRO_CONFIG         = 0x1B, /* R/W [ 7=XGYRO_Ct_en 6=YGYRO_Ct_en 5=ZGYRO_Ct_en 4:3=GYRO_FS_SEL[1:0] 2=_ 1:0=FCHOICE_B[1:0] ] */
    ACCEL_CONFIG        = 0x1C, /* R/W [ 7=ax_st_en 6=ay_st_en 5=az_st_en 4:3=ACCEL_FS_SEL[1:0] 2:0=_ ] */
    ACCEL_CONFIG_2      = 0x1D, /* R/W [ 7:4=_ 3:2=ACCEL_FCHOICE_B[1:0] 1:0=A_DLPF_CFG[1:0] ] */
    LP_ACCEL_ODR        = 0x1E, /* R/W [ 7:4=_ 3:0=Lposc_clksel[3:0] ] */

    WOM_THR             = 0x1F, /* R/W WOM_THR[7:0] */

    FIFO_EN             = 0x23, /* R/W [ 7=TEMP_FIFO_EN 6=GYRO_XO_UT 5=GYRO_YO_UT 4=GYRO_ZO_UT 3=ACCEL 2=SLV2 1=SLV1 0=SLV0 ] */

    INT_PIN_CFG         = 0x37, /* R/W [ 7=ACTL 6=OPEN 5=LATCH_INT_EN 4=INT_ANYRD_2CLEAR 3=ACTL_FSY_NC 2=FSYNC_INT_MODE_EN 1=BYPASS_EN 0=_ ] */
    INT_ENABLE          = 0x38, /* R/W [ 7=_ 6=WOM_EN 5=_ 4=FIFO_OFLOW_EN 3=FSYNC_INT_EN 2=_ 1=_ 0=RAW_RDY_EN ] */
    INT_STATUS          = 0x3A, /* R/O [ 7=_ 6=WOM_INT 5=_ 4=FIFO_OFLOW_INT 3=FSYNC_INT 2=_ 1=_ 0=RAW_DATA_RDY_INT ] */

    ACCEL_XOUT_H        = 0x3B, /* R/O ACCEL_XOUT_H[15:8] */
    ACCEL_XOUT_L        = 0x3C, /* R/O ACCEL_XOUT_L[7:0] */
    ACCEL_YOUT_H        = 0x3D, /* R/O ACCEL_YOUT_H[15:8] */
    ACCEL_YOUT_L        = 0x3E, /* R/O ACCEL_YOUT_L[7:0] */
    ACCEL_ZOUT_H        = 0x3F, /* R/O ACCEL_ZOUT_H[15:8] */
    ACCEL_ZOUT_L        = 0x40, /* R/O ACCEL_ZOUT_L[7:0] */
    TEMP_OUT_H          = 0x41, /* R/O TEMP_OUT_H[15:8] */
    TEMP_OUT_L          = 0x42, /* R/O TEMP_OUT_L[7:0] */
    GYRO_XOUT_H         = 0x43, /* R/O GYRO_XOUT_H[15:8] */
    GYRO_XOUT_L         = 0x44, /* R/O GYRO_XOUT_L[7:0] */
    GYRO_YOUT_H         = 0x45, /* R/O GYRO_YOUT_H[15:8] */
    GYRO_YOUT_L         = 0x46, /* R/O GYRO_YOUT_L[7:0] */
    GYRO_ZOUT_H         = 0x47, /* R/O GYRO_ZOUT_H[15:8] */
    GYRO_ZOUT_L         = 0x48, /* R/O GYRO_ZOUT_L[7:0] */

    EXT_SENS_DATA_00    = 0x49, /* R/O EXT_SENS_DATA_00[7:0] */

    MOT_DETECT_CTRL     = 0x69, /* R/W [ 7=ACCEL_INTEL_EN 6=ACCEL_INTEL_MODE 5:0=_ ] */
    USER_CTRL           = 0x6A, /* R/W [ 7=_ 6=FIFO_EN 5=I2C_MST_EN 4=I2C_IF_DIS 3=_ 2=FIFO_RST 1=I2C_MST_RST 0=SIG_COND_RST ] */

    PWR_MGMT_1          = 0x6B, /* R/W [ 7=H_RESET 6=SLEEP 5=CYCLE 4=GYRO_STANDBY 3=PD_PTAT 2:0=CLKSEL[2:0] ] */
    PWR_MGMT_2          = 0x6C, /* R/W [ 7:6=_ 5=DIS_XA 4=DIS_YA 3=DIS_ZA 2=DIS_XG 1=DIS_YG 0=DIS_ZG ] */
    FIFO_COUNTH         = 0x72, /* R/W [ 7:5=_ 4:0=FIFO_CNT[12:8] ] */
    FIFO_COUNTL         = 0x73, /* R/W FIFO_CNT[7:0] */
    FIFO_R_W            = 0x74, /* R/W D[7:0] */
    WHO_AM_I            = 0x75, /* R/O WHOAMI[7:0] */
    XA_OFFSET_H         = 0x77, /* R/W XA_OFFS [14:7] */
    XA_OFFSET_L         = 0x78, /* R/W XA_OFFS [6:0] - */
    YA_OFFSET_H         = 0x7A, /* R/W YA_OFFS [14:7] */
    YA_OFFSET_L         = 0x7B, /* R/W YA_OFFS [6:0] - */
    ZA_OFFSET_H         = 0x7D, /* R/W ZA_OFFS [14:7] */
    ZA_OFFSET_L         = 0x7E, /* R/W ZA_OFFS [6:0] - */
};

enum class AK8963_Register : uint8_t {
    WIA                 = 0x00, /* R/O WIA[7:0] */
    INFO                = 0x01, /* R/O INFO[7:0] */
    ST1                 = 0x02, /* R/O [ 7:1=0 0=DRDY ] */
    H_X_L               = 0x03, /* R/O HX[7:0] */
    H_X_H               = 0x04, /* R/O HX[15:8] */
    H_Y_L               = 0x05, /* R/O HY[7:0] */
    H_Y_H               = 0x06, /* R/O HY[15:8] */
    H_Z_L               = 0x07, /* R/O HZ[7:0] */
    H_Z_H               = 0x08, /* R/O HZ[15:8] */
    ST2                 = 0x09, /* R/O [ 7:5=0 4=BITM 3=HOFL 2:0=0 ] */
    CNTL1               = 0x0A, /* R/W [ 7:5=0 4=BIT 3:0=MODE[3:0] ] */
    CNTL2               = 0x0B, /* R/W [ 7:1=0 0=SRST ] */
    ASTC                = 0x0C, /* R/W [ 7=0 6=SELF 5:0=0 ] */
    TS1                 = 0x0D, /* R/W XTS[7:0] */
    TS2                 = 0x0E, /* R/W YTS[7:0] */
    I2CDIS              = 0x0F, /* R/W I2CDIS[ 7:0 ] */
    ASAX                = 0x10, /* R/O ASAX[7:0] */
    ASAY                = 0x11, /* R/O ASAY[7:0] */
    ASAZ                = 0x12, /* R/O ASAZ[7:0] */
};

enum class MPUAccelerometerRange : uint8_t {
    Accel_2_G = 0x00,
    Accel_4_G = 0x01,
    Accel_8_G = 0x02,
    Accel_16_G = 0x03,
};

enum class MPUGyroRange : uint8_t {
    Gyro_250_DegPerSec = 0x00,
    Gyro_500_DegPerSec = 0x01,
    Gyro_1000_DegPerSec = 0x02,
    Gyro_2000_DegPerSec = 0x03,
};

enum class MPUFifoEnable : uint8_t {
    Temperature = 1 << 7,
    GyroX = 1 << 6,
    GyroY = 1 << 5,
    GyroZ = 1 << 4,
    Accelerometer = 1 << 3,
    Slave2 = 1 << 2,
    Slave1 = 1 << 1,
    Slave0 = 1 << 0,
};
constexpr enum MPUFifoEnable operator|(enum MPUFifoEnable a, enum MPUFifoEnable b) {
    return static_cast<enum MPUFifoEnable>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}
constexpr enum MPUFifoEnable operator&(enum MPUFifoEnable a, enum MPUFifoEnable b) {
    return static_cast<enum MPUFifoEnable>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}
constexpr enum MPUFifoEnable operator~(enum MPUFifoEnable a) {
    return static_cast<enum MPUFifoEnable>(~static_cast<uint8_t>(a));
}

enum class MPUInterrupt : uint8_t {
    WakeOnMotion = 1 << 6,
    FifoOverflow = 1 << 4,
    Fsync = 1 << 3,
    RawDataReady = 1 << 0,
};
constexpr enum MPUInterrupt operator|(enum MPUInterrupt a, enum MPUInterrupt b) {
    return static_cast<enum MPUInterrupt>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}
constexpr enum MPUInterrupt operator&(enum MPUInterrupt a, enum MPUInterrupt b) {
    return static_cast<enum MPUInterrupt>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}
constexpr enum MPUInterrupt operator~(enum MPUInterrupt a) {
    return static_cast<enum MPUInterrupt>(~static_cast<uint8_t>(a));
}

struct MPUMagnetometerReading {
    XYZFloat values;
    bool is_16_bit;
    bool had_overflow;
};


class MPU925x : Initializable {
public:
    const uint8_t MPU9250_WHOAMI_EXPECTED_VALUE = 0x71;
    const uint8_t MPU9255_WHOAMI_EXPECTED_VALUE = 0x73;
    const uint8_t AK8963_WIA_EXPECTED_VALUE = 0x48;

    const uint8_t MPU9250_DEFAULT_ADDRESS = 0x68;
    const uint8_t AK8963_DEFAULT_ADDRESS = 0x0C;

    // enum class AccelerometerBandwidth : uint8_t {
    //     Accel_BW_218_1_Hz = 0x01,
    //     Accel_BW_99_Hz = 0x02,
    //     Accel_BW_44_8_Hz = 0x03,
    //     Accel_BW_21_2_Hz = 0x04,
    //     Accel_BW_10_2_Hz = 0x05,
    //     Accel_BW_5_05_Hz = 0x06,
    //     Accel_BW_420_Hz = 0x07,
    // };
    
    // enum class GyroBandwidth : uint8_t {
    //     Gyro_BW_250_Hz = 0x00,
    //     Gyro_BW_184_Hz = 0x01,
    //     Gyro_BW_92_Hz = 0x02,
    //     Gyro_BW_41_Hz = 0x03,
    //     Gyro_BW_20_Hz = 0x04,
    //     Gyro_BW_10_Hz = 0x05,
    //     Gyro_BW_5_Hz = 0x06,
    // };

    const uint8_t Config_FIFOMode_OverwriteWhenFull = 1 << 6;
    const uint8_t Config_FIFOMode_StopWhenFull = 0 << 6;

public:
    MPU925x(I2CDevice& mpu_device, I2CDevice& ak_device);

    void init();

    uint8_t read_register(MPU925x_Register register_address);
    uint16_t read_word_registers(MPU925x_Register l_register_address);
    void read_registers(MPU925x_Register start_register_address, uint8_t count, uint8_t* buffer_out);
    void write_register(MPU925x_Register register_address, uint8_t value);
    void write_registers(MPU925x_Register start_register_address, uint8_t count, const uint8_t* buffer_in);

    uint8_t read_ak_register(AK8963_Register register_address);
    void read_ak_registers(AK8963_Register start_register_address, uint8_t count, uint8_t* buffer_out);
    void write_ak_register(AK8963_Register register_address, uint8_t value);
    void write_ak_registers(MPU925x_Register start_register_address, uint8_t count, const uint8_t* buffer_in);

    uint8_t mpu_who_am_i();
    uint8_t ak_who_am_i();

    XYZFloat get_acceleration_values();
    XYZFloat get_gyroscope_values();
    MPUMagnetometerReading get_magnetometer_values();
    float get_temperature_value();

    XYZInt16 get_gyroscope_offsets();
    XYZInt16 get_acccelerometer_offsets();
    void set_gyroscope_offsets(XYZInt16 offsets);
    void set_accelerometer_offsets(XYZInt16 offsets);

    uint8_t get_sample_rate_divider();
    void set_samplerate_divider(uint8_t divider);

    uint8_t get_configuration();
    void set_configuration(uint8_t config);
    
    const uint8_t Config_FIFO_MODE_STOP_ON_FULL = 1 << 6;

    void set_accelerometer_range(MPUAccelerometerRange range);
    void set_gyro_range(MPUGyroRange range);

    MPUFifoEnable get_fifo_enabled_modules();
    bool is_fifo_module_enabled(MPUFifoEnable module);
    void enable_fifo_module(MPUFifoEnable module);
    void disable_fifo_module(MPUFifoEnable module);

    bool is_i2c_bypass_enabled();
    void enable_i2c_bypass();
    void disable_i2c_bypass();

    MPUInterrupt get_enabled_interrupts();
    void set_enabled_interrupts(MPUInterrupt interrupts);

    MPUInterrupt get_interrupt_statuses();

    void set_accelerometer_motion_detection(bool wake_on_motion_detection, bool compare_with_previous_sample);

    bool is_fifo_enabled();
    void enable_fifo();
    void disable_fifo();

    void hardware_reset();
    void put_into_sleep();
    void wake_up();
    void gyros_standby_mode();
    void wake_gyros();
    
    XYZBool accelerometer_enabled_axis();
    XYZBool gyroscope_enabled_axis();
    void set_enabled_module_axis(XYZBool accelerometer_enabled, XYZBool gyroscope_enabled);

    uint16_t get_fifo_count();
    uint8_t read_fifo_byte();
    void read_fifo_bytes(uint8_t count, uint8_t* buffer_out);

    bool magnetometer_has_data_ready();
    bool magnetometer_was_data_overrun();
    void magnetometer_set_mode_power_down();
    void magnetometer_set_mode_single_measurement();
    void magnetometer_set_mode_continuous_measurement_8Hz();
    void magnetometer_set_mode_continuous_measurement_100Hz();
    void magnetometer_set_mode_external_trigger_measurement();
    void magnetometer_set_16_bit_mode();
    void magnetometer_set_14_bit_mode();
    void magnetometer_soft_reset();
    XYZUint8 magnetometer_get_sensitivity_adjustment();

private:
    I2CDevice& mpu_device;
    I2CDevice& ak_device;

    MPUAccelerometerRange accelerometer_range = MPUAccelerometerRange::Accel_2_G;
    MPUGyroRange gyro_range = MPUGyroRange::Gyro_250_DegPerSec;

    float accelerometer_scale_factor = 1.0f;
    float gyro_scale_factor = 1.0f;
    XYZFloat magnetometer_scale_factors = { 1.0f, 1.0f, 1.0f };
};
