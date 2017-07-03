

#ifndef __MP6050_H__
#define __MP6050_H__

#include "mp6050_enum.h"

#define DEBUG           //in code debug time 

#define GYRO_DEVID_MP6050  0x68

#ifdef DEBUG
#define GYRO_REG_DEBUG    0x00
#endif

/**
 *  struct inv_mpu6050_reg_map - Notable registers.
 *  @sample_rate_div:	Divider applied to gyro output rate.
 *  @lpf:		Configures internal low pass filter.
 *  @user_ctrl:		Enables/resets the FIFO.
 *  @fifo_en:		Determines which data will appear in FIFO.
 *  @gyro_config:	gyro config register.
 *  @accl_config:	accel config register
 *  @fifo_count_h:	Upper byte of FIFO count.
 *  @fifo_r_w:		FIFO register.
 *  @raw_gyro:		Address of first gyro register.
 *  @raw_accl:		Address of first accel register.
 *  @temperature:	temperature register
 *  @int_enable:	Interrupt enable register.
 *  @pwr_mgmt_1:	Controls chip's power state and clock source.
 *  @pwr_mgmt_2:	Controls power state of individual sensors.
 */
struct inv_mpu6050_reg_map {
	u8 sample_rate_div;
	u8 lpf;
	u8 user_ctrl;
	u8 fifo_en;
	u8 gyro_config;
	u8 accl_config;
	u8 fifo_count_h;
	u8 fifo_r_w;
	u8 raw_gyro;
	u8 raw_accl;
	u8 temperature;
	u8 int_enable;
	u8 pwr_mgmt_1;
	u8 pwr_mgmt_2;
};

/**
 *  struct inv_mpu6050_chip_config - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @accl_fs:		accel full scale range.
 *  @enable:		master enable state.
 *  @accl_fifo_enable:	enable accel data output
 *  @gyro_fifo_enable:	enable gyro data output
 *  @fifo_rate:		FIFO update rate.
 */
struct inv_mpu6050_chip_config {
	unsigned int fsr:2;
	unsigned int lpf:3;
	unsigned int accl_fs:2;
	unsigned int enable:1;
	unsigned int accl_fifo_enable:1;
	unsigned int gyro_fifo_enable:1;
	u16 fifo_rate;
};

/**
 *  struct inv_mpu6050_hw - Other important hardware information.
 *  @num_reg:	Number of registers on device.
 *  @name:      name of the chip.
 *  @reg:   register map of the chip.
 *  @config:    configuration of the chip.
 */
struct inv_mpu6050_hw {
	u8 num_reg;
	u8 *name;
	const struct inv_mpu6050_reg_map *reg;
	const struct inv_mpu6050_chip_config *config;
};

struct inv_mpu6050_gyro_data {
    s16 x;
    s16 y;
    s16 z;
};

struct inv_mpu6050_accel_data {
    char xh;
    char xl;
    char yh;
    char yl;
    char zh;
    char zl;
};


/*
 *  struct inv_mpu6050_state - Driver state variables.
 *  @TIMESTAMP_FIFO_SIZE: fifo size for timestamp.
 *  @trig:              IIO trigger.
 *  @chip_config:	Cached attribute information.
 *  @reg:		Map of important registers.
 *  @hw:		Other hardware-specific information.
 *  @chip_type:		chip type.
 *  @time_stamp_lock:	spin lock to time stamp.
 *  @client:		i2c client handle.
 *  @plat_data:		platform data.
 *  @timestamps:        kfifo queue to store time stamp.
 */
struct inv_mpu6050_state {
#define TIMESTAMP_FIFO_SIZE 16
	struct inv_mpu6050_chip_config chip_config;
	const struct inv_mpu6050_reg_map *reg;
	const struct inv_mpu6050_hw *hw;
	enum   inv_devices chip_type;
	//spinlock_t time_stamp_lock;                   //debug
	struct i2c_client *client;
	struct i2c_client *accel_client;
	struct i2c_client *gyro_client;
	//struct inv_mpu6050_platform_data plat_data;
	struct mutex mlock;
    int initSt;
    struct inv_mpu6050_gyro_data gyro_data;
    struct inv_mpu6050_accel_data accel_data;
};



#if 0


enum    reg_mpu6050
{
        //register of self_test
        reg_self_test_x = 0x0D,
        reg_self_test_y,
        reg_self_test_z,
        reg_self_test_a,
        
        //register of component config
        reg_simplrt_div = 0x19,
        reg_config,
        reg_gyro_config,
        reg_accel_config,
        reg_fifo_en,
        
        //register of int config
        reg_int_pin_cfg = 0x37,
        reg_int_enable,
        reg_int_status = 0x3A,
        
        //register of sample data output
        reg_accel_xout_h,
        reg_accel_xout_l,
        reg_accel_yout_h,
        reg_accel_yout_l,
        reg_accel_zout_h,
        reg_accel_zout_l,
        reg_temp_out_h,
        reg_temp_out_l,
        reg_gyro_xout_h,
        reg_gyro_xout_l,
        reg_gyro_yout_h,
        reg_gyro_yout_l,
        reg_gyro_zout_h,
        reg_gyro_zout_l,
        
        //register of power manage
        reg_signal_path_reset = 0x68,
        reg_user_ctrl = 0x6A,
        reg_pwr_mgmt1,
        reg_pwr_mgmt2,
        
        //register of fifo ctrl
        reg_fifo_count_h = 0x72,
        reg_fifo_count_l,
        reg_fifo_r_w,
        
        //who am i
        reg_who_am_i = 0x75,
};

enum    axis
{
    XG = 0x01,
    YG = 0x02,
    ZG = 0x04,
    XA = 0x08,
    YA = 0x10,
    ZA = 0x20
};

//register map
#define     SELF_TEST_X             reg_self_test_x
#define     SELF_TEST_Y             reg_self_test_y
#define     SELF_TEST_Z             reg_self_test_z
#define     SELF_TEST_A             reg_self_test_a
#define     SMPLRT_DIV              reg_simplrt_div
#define     CONFIG                  reg_config
#define     GYRO_CONFIG             reg_gyro_config
#define     ACCEL_CONFIG            reg_accel_config
#define     FIFO_EN                 reg_fifo_en
#define     INT_PIN_CFG             reg_int_pin_cfg 
#define     INT_ENABLE              reg_int_enable
#define     INT_STATUS              reg_int_status
#define     ACCEL_XOUT_H            reg_accel_xout_h
#define     ACCEL_XOUT_L            reg_accel_xout_l
#define     ACCEL_YOUT_H            reg_accel_yout_h
#define     ACCEL_YOUT_L            reg_accel_yout_l
#define     ACCEL_ZOUT_H            reg_accel_zout_h
#define     ACCEL_ZOUT_L            reg_accel_zout_l
#define     TEMP_OUT_H              reg_temp_out_h
#define     TEMP_OUT_L              reg_temp_out_L
#define     GYRO_XOUT_H             reg_gyro_xout_h
#define     GYRO_XOUT_L             reg_gyro_xout_l
#define     GYRO_YOUT_H             reg_gyro_yout_h
#define     GYRO_YOUT_L             reg_gyro_yout_l
#define     GYRO_ZOUT_H             reg_gyro_zout_h
#define     GYRO_ZOUT_L             reg_gyro_zout_l
#define     SIGNAL_PATH_RESET       reg_signal_path_reset
#define     USER_CTRL               reg_user_ctrl
#define     PWR_MGMT1               reg_pwr_mgmt1
#define     PWR_MGMT2               reg_pwr_mgmt2
#define     FIFO_COUNT_H            reg_fifo_count_h
#define     FIFO_COUNT_L            reg_fifo_count_l
#define     FIFO_R_W                reg_fifo_r_w
#define     WHO_AM_I                reg_who_am_i

#define     DEVICE_RESET_EN         0x80
#define     DEVICE_RESET_DIS        0x00
#define     TEMP_DIS                0x04

#define     CLK_SEL_8MHZ            0x00
#define     CLK_SEL_AXIS_X          0x01
#define     CLK_SEL_AXIS_Y          0x02
#define     CLK_SEL_AXIS_Z          0x03
#define     CLK_SEL_32_768          0x04
#define     CLK_SEL_19_2_MHz        0x05
#define     CLK_STOP                0x07

#endif

#endif
