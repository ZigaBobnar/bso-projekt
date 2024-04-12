#pragma once

// Documentation at:
// https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf

// Packing: uint8_t = [ Bit 7, 6, 5, 4, 3, 2, 1, Bit 0 ]

#define MPU9250_REG_SELF_TEST_X_GYRO    0x00 /* R/W xg_st_data[7:0] */
#define MPU9250_REG_SELF_TEST_Y_GYRO    0x01 /* R/W yg_st_data[7:0] */
#define MPU9250_REG_SELF_TEST_Z_GYRO    0x02 /* R/W zg_st_data[7:0] */

#define MPU9250_REG_SELF_TEST_X_ACCEL   0x0D /* R/W xa_st_data[7:0] */
#define MPU9250_REG_SELF_TEST_Y_ACCEL   0x0E /* R/W ya_st_data[7:0] */
#define MPU9250_REG_SELF_TEST_Z_ACCEL   0x0F /* R/W za_st_data[7:0] */

#define MPU9250_REG_XG_OFFSET_H         0x13 /* R/W X_OFFS_USR[15:8] */
#define MPU9250_REG_XG_OFFSET_L         0x14 /* R/W X_OFFS_USR[7:0] */
#define MPU9250_REG_YG_OFFSET_H         0x15 /* R/W Y_OFFS_USR[15:8] */
#define MPU9250_REG_YG_OFFSET_L         0x16 /* R/W Y_OFFS_USR[7:0] */
#define MPU9250_REG_ZG_OFFSET_H         0x17 /* R/W Z_OFFS_USR[15:8] */
#define MPU9250_REG_ZG_OFFSET_L         0x18 /* R/W Z_OFFS_USR[7:0] */
#define MPU9250_REG_SMPLRT_DIV          0x19 /* R/W SMPLRT_DIV[7:0] */

#define MPU9250_REG_CONFIG              0x1A /* R/W [ 7=_ 6=FIFO_MODE 5:3=EXT_SYNC_SET[2:0] 2:0=DLPF_CFG[2:0] */
#define MPU9250_REG_GYRO_CONFIG         0x1B /* R/W [ 7=XGYRO_Ct_en 6=YGYRO_Ct_en 5=ZGYRO_Ct_en 4:3=GYRO_FS_SEL[1:0] 2=_ 1:0=FCHOICE_B[1:0] ] */
#define MPU9250_REG_ACCEL_CONFIG        0x1C /* R/W [ 7=ax_st_en 6=ay_st_en 5=az_st_en 4:3=ACCEL_FS_SEL[1:0] 2:0=_ ] */
#define MPU9250_REG_ACCEL_CONFIG        0x1D /* R/W [ 7:4=_ 3:2=ACCEL_FCHOICE_B[1:0] 1:0=A_DLPF_CFG[1:0] ] */
#define MPU9250_REG_LP_ACCEL_ODR        0x1E /* R/W [ 7:4=_ 3:0=Lposc_clksel[3:0] ] */

#define MPU9250_REG_FIFO_EN             0x23 /* R/W [ 7=TEMP_FIFO_EN 6=GYRO_XO_UT 5=GYRO_YO_UT 4=GYRO_ZO_UT 3=ACCEL 2=SLV2 1=SLV1 0=SLV0 ] */

#define MPU9250_REG_ACCEL_XOUT_H        0x3B /* R/O ACCEL_XOUT_H[15:8] */
#define MPU9250_REG_ACCEL_XOUT_L        0x3C /* R/O ACCEL_XOUT_L[7:0] */
#define MPU9250_REG_ACCEL_YOUT_H        0x3D /* R/O ACCEL_YOUT_H[15:8] */
#define MPU9250_REG_ACCEL_YOUT_L        0x3E /* R/O ACCEL_YOUT_L[7:0] */
#define MPU9250_REG_ACCEL_ZOUT_H        0x3F /* R/O ACCEL_ZOUT_H[15:8] */
#define MPU9250_REG_ACCEL_ZOUT_L        0x40 /* R/O ACCEL_ZOUT_L[7:0] */
#define MPU9250_REG_TEMP_OUT_H          0x41 /* R/O TEMP_OUT_H[15:8] */
#define MPU9250_REG_TEMP_OUT_L          0x42 /* R/O TEMP_OUT_L[7:0] */
#define MPU9250_REG_GYRO_XOUT_H         0x43 /* R/O GYRO_XOUT_H[15:8] */
#define MPU9250_REG_GYRO_XOUT_L         0x44 /* R/O GYRO_XOUT_L[7:0] */
#define MPU9250_REG_GYRO_YOUT_H         0x45 /* R/O GYRO_YOUT_H[15:8] */
#define MPU9250_REG_GYRO_YOUT_L         0x46 /* R/O GYRO_YOUT_L[7:0] */
#define MPU9250_REG_GYRO_ZOUT_H         0x47 /* R/O GYRO_ZOUT_H[15:8] */
#define MPU9250_REG_GYRO_ZOUT_L         0x48 /* R/O GYRO_ZOUT_L[7:0] */

#define MPU9250_REG_PWR_MGMT_1          0x6B /* R/W [ 7=H_RESET 6=SLEEP 5=CYCLE 4=GYRO_STANDBY 3=PD_PTAT 2:0=CLKSEL[2:0] ] */
#define MPU9250_REG_PWR_MGMT_2          0x6C /* R/W [ 7:6=_ 5=DIS_XA 4=DIS_YA 3=DIS_ZA 2=DIS_XG 1=DIS_YG 0=DIS_ZG ] */
#define MPU9250_REG_FIFO_COUNTH         0x72 /* R/W [ 7:5=_ 4:0=FIFO_CNT[12:8] ] */
#define MPU9250_REG_FIFO_COUNTL         0x73 /* R/W FIFO_CNT[7:0] */
#define MPU9250_REG_FIFO_R_W            0x74 /* R/W D[7:0] */
#define MPU9250_REG_WHO_AM_I            0x75 /* R/O WHOAMI[7:0] */
#define MPU9250_REG_XA_OFFSET_H         0x77 /* R/W XA_OFFS [14:7] */
#define MPU9250_REG_XA_OFFSET_L         0x78 /* R/W XA_OFFS [6:0] - */
#define MPU9250_REG_YA_OFFSET_H         0x7A /* R/W YA_OFFS [14:7] */
#define MPU9250_REG_YA_OFFSET_L         0x7B /* R/W YA_OFFS [6:0] - */
#define MPU9250_REG_ZA_OFFSET_H         0x7D /* R/W ZA_OFFS [14:7] */
#define MPU9250_REG_ZA_OFFSET_L         0x7E /* R/W ZA_OFFS [6:0] - */


