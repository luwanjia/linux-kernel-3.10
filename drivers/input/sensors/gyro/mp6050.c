/* drivers/input/sensors/access/kxtik.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/mp6050.h>
#include <linux/sensor-dev.h>


#define MPU6050_RANGE			2000000

#define MPU6050_PRECISION       16
#define MPU6050_BOUNDARY        (0x1 << (MPU6050_PRECISION - 1))
#define MPU6050_GRAVITY_STEP    MPU6050_RANGE / MPU6050_BOUNDARY

#define MP6050_ENABLE			0x00
#define GSENSOR_MIN  		10

/*
 * this is the gyro scale translated from dynamic range plus/minus
 * {250, 500, 1000, 2000} to rad/s
 */
//static const int gyro_scale_6050[] = {133090, 266181, 532362, 1064724};

/*
 * this is the accel scale translated from dynamic range plus/minus
 * {2, 4, 8, 16} to m/s^2
 */
//static const int accel_scale[] = {598, 1196, 2392, 4785};

static const struct inv_mpu6050_reg_map reg_set_6050 = {
	.sample_rate_div	= INV_MPU6050_REG_SAMPLE_RATE_DIV,
	.lpf                    = INV_MPU6050_REG_CONFIG,
	.user_ctrl              = INV_MPU6050_REG_USER_CTRL,
	.fifo_en                = INV_MPU6050_REG_FIFO_EN,
	.gyro_config            = INV_MPU6050_REG_GYRO_CONFIG,
	.accl_config            = INV_MPU6050_REG_ACCEL_CONFIG,
	.fifo_count_h           = INV_MPU6050_REG_FIFO_COUNT_H,
	.fifo_r_w               = INV_MPU6050_REG_FIFO_R_W,
	.raw_gyro               = INV_MPU6050_REG_RAW_GYRO,
	.raw_accl               = INV_MPU6050_REG_RAW_ACCEL,
	.temperature            = INV_MPU6050_REG_TEMPERATURE,
	.int_enable             = INV_MPU6050_REG_INT_ENABLE,
	.pwr_mgmt_1             = INV_MPU6050_REG_PWR_MGMT_1,
	.pwr_mgmt_2             = INV_MPU6050_REG_PWR_MGMT_2,
};

static const struct inv_mpu6050_chip_config chip_config_6050 = {
	.fsr = INV_MPU6050_FSR_2000DPS,
	.lpf = INV_MPU6050_FILTER_20HZ,
	.fifo_rate = INV_MPU6050_INIT_FIFO_RATE,
	.gyro_fifo_enable = false,
	.accl_fifo_enable = false,
	.accl_fs = INV_MPU6050_FS_02G,
};

static const struct inv_mpu6050_hw hw_info[INV_NUM_PARTS] = {
	{
		.num_reg = 117,
		.name = "MPU6050",
		.reg = &reg_set_6050,
		.config = &chip_config_6050,
	},
};



struct inv_mpu6050_state gMpu6050Stat;

int inv_mpu6050_write_reg(struct inv_mpu6050_state *st, int reg, u8 d)
{
	return i2c_smbus_write_i2c_block_data(st->client, reg, 1, &d);
}

/****************operate according to sensor chip:start************/

int inv_mpu6050_set_power_itg(struct inv_mpu6050_state *st, bool power_on)
{
	int result;

	if (power_on)
		result = inv_mpu6050_write_reg(st, st->reg->pwr_mgmt_1, 0);
	else
		result = inv_mpu6050_write_reg(st, st->reg->pwr_mgmt_1,
						INV_MPU6050_BIT_SLEEP);
	if (result)
		return result;

	if (power_on)
		msleep(INV_MPU6050_REG_UP_TIME);

	return 0;
}

int inv_mpu6050_switch_engine(struct inv_mpu6050_state *st, bool en, u32 mask)
{
	u8 d, mgmt_1;
	int result;

	/* switch clock needs to be careful. Only when gyro is on, can
	   clock source be switched to gyro. Otherwise, it must be set to
	   internal clock */
	if (INV_MPU6050_BIT_PWR_GYRO_STBY == mask) {
		result = i2c_smbus_read_i2c_block_data(st->client,
				       st->reg->pwr_mgmt_1, 1, &mgmt_1);
		if (result != 1)
			return result;

		mgmt_1 &= ~INV_MPU6050_BIT_CLK_MASK;
	}

	if ((INV_MPU6050_BIT_PWR_GYRO_STBY == mask) && (!en)) {
		/* turning off gyro requires switch to internal clock first.
		   Then turn off gyro engine */
		mgmt_1 |= INV_CLK_INTERNAL;
		result = inv_mpu6050_write_reg(st, st->reg->pwr_mgmt_1, mgmt_1);
		if (result)
			return result;
	}

	result = i2c_smbus_read_i2c_block_data(st->client,
				       st->reg->pwr_mgmt_2, 1, &d);
	if (result != 1)
		return result;
	if (en)
		d &= ~mask;
	else
		d |= mask;
	result = inv_mpu6050_write_reg(st, st->reg->pwr_mgmt_2, d);
	if (result)
		return result;

	if (en) {
		/* Wait for output stablize */
		msleep(INV_MPU6050_TEMP_UP_TIME);
		if (INV_MPU6050_BIT_PWR_GYRO_STBY == mask) {
			/* switch internal clock to PLL */
			mgmt_1 |= INV_CLK_PLL;
			result = inv_mpu6050_write_reg(st,
					st->reg->pwr_mgmt_1, mgmt_1);
			if (result)
				return result;
		}
	}

	return 0;
}

int inv_reset_fifo(struct inv_mpu6050_state  *st)
{
	int result;
	u8 d;

	/* disable interrupt */
	result = inv_mpu6050_write_reg(st, st->reg->int_enable, 0);
	if (result) {
		dev_err(&st->client->dev, "int_enable failed %d\n", result);
		return result;
	}
	/* disable the sensor output to FIFO */
	result = inv_mpu6050_write_reg(st, st->reg->fifo_en, 0);
	if (result)
		goto reset_fifo_fail;
	/* disable fifo reading */
	result = inv_mpu6050_write_reg(st, st->reg->user_ctrl, 0);
	if (result)
		goto reset_fifo_fail;

	/* reset FIFO*/
	result = inv_mpu6050_write_reg(st, st->reg->user_ctrl,
					INV_MPU6050_BIT_FIFO_RST);
	if (result)
		goto reset_fifo_fail;
	/* enable interrupt */
	if (st->chip_config.accl_fifo_enable ||
	    st->chip_config.gyro_fifo_enable) {
		result = inv_mpu6050_write_reg(st, st->reg->int_enable,
					INV_MPU6050_BIT_DATA_RDY_EN);
		if (result)
			return result;
	}
	/* enable FIFO reading and I2C master interface*/
	result = inv_mpu6050_write_reg(st, st->reg->user_ctrl,
					INV_MPU6050_BIT_FIFO_EN);
	if (result)
		goto reset_fifo_fail;
	/* enable sensor output to FIFO */
	d = 0;
	if (st->chip_config.gyro_fifo_enable)
		d |= INV_MPU6050_BITS_GYRO_OUT;
	if (st->chip_config.accl_fifo_enable)
		d |= INV_MPU6050_BIT_ACCEL_OUT;
	result = inv_mpu6050_write_reg(st, st->reg->fifo_en, d);
	if (result)
		goto reset_fifo_fail;

	return 0;

reset_fifo_fail:
	dev_err(&st->client->dev, "reset fifo failed %d\n", result);
	result = inv_mpu6050_write_reg(st, st->reg->int_enable,
					INV_MPU6050_BIT_DATA_RDY_EN);

	return result;
}

static int inv_mpu6050_init_config(struct inv_mpu6050_state *st)
{
	int result;
	u8 d;

	result = inv_mpu6050_set_power_itg(st, true);
	if (result)
		return result;
	d = (INV_MPU6050_FSR_2000DPS << INV_MPU6050_GYRO_CONFIG_FSR_SHIFT);
	result = inv_mpu6050_write_reg(st, st->reg->gyro_config, d);
	if (result)
		return result;

	d = INV_MPU6050_FILTER_20HZ;
	result = inv_mpu6050_write_reg(st, st->reg->lpf, d);
	if (result)
		return result;

	d = INV_MPU6050_ONE_K_HZ / INV_MPU6050_INIT_FIFO_RATE - 1;
	result = inv_mpu6050_write_reg(st, st->reg->sample_rate_div, d);
	if (result)
		return result;

	d = (INV_MPU6050_FS_02G << INV_MPU6050_ACCL_CONFIG_FSR_SHIFT);
	result = inv_mpu6050_write_reg(st, st->reg->accl_config, d);
	if (result)
		return result;

	memcpy(&st->chip_config, hw_info[st->chip_type].config,
		sizeof(struct inv_mpu6050_chip_config));
	result = inv_mpu6050_set_power_itg(st, false);

	return result;
}

static int inv_mpu6050_set_enable(struct inv_mpu6050_state *st, bool enable)
{
	int result;

	if (enable) {
		result = inv_mpu6050_set_power_itg(st, true);
		if (result)
			return result;

		if (st->chip_config.gyro_fifo_enable) {
			result = inv_mpu6050_switch_engine(st, true,
					INV_MPU6050_BIT_PWR_GYRO_STBY);
			if (result)
				return result;
		}
		if (st->chip_config.accl_fifo_enable) {
			result = inv_mpu6050_switch_engine(st, true,
					INV_MPU6050_BIT_PWR_ACCL_STBY);
			if (result)
				return result;
		}

        result = inv_reset_fifo(st);
		if (result)
			return result;
	} else {
		result = inv_mpu6050_write_reg(st, st->reg->fifo_en, 0);
		if (result)
			return result;

		result = inv_mpu6050_write_reg(st, st->reg->int_enable, 0);
		if (result)
			return result;

		result = inv_mpu6050_write_reg(st, st->reg->user_ctrl, 0);
		if (result)
			return result;

		result = inv_mpu6050_switch_engine(st, false,
					INV_MPU6050_BIT_PWR_GYRO_STBY);
		if (result)
			return result;

		result = inv_mpu6050_switch_engine(st, false,
					INV_MPU6050_BIT_PWR_ACCL_STBY);
		if (result)
			return result;
		result = inv_mpu6050_set_power_itg(st, false);
		if (result)
			return result;
	}
	st->chip_config.enable = enable;

	return 0;
}

static int inv_check_and_setup_chip(struct inv_mpu6050_state *st)
{
	int result;

	st->chip_type = INV_MPU6050;
	st->hw  = &hw_info[st->chip_type];
	st->reg = hw_info[st->chip_type].reg;

	/* reset to make sure previous state are not there */
	result = inv_mpu6050_write_reg(st, st->reg->pwr_mgmt_1,
					INV_MPU6050_BIT_H_RESET);
	if (result)
		return result;
	msleep(INV_MPU6050_POWER_UP_TIME);
	/* toggle power state. After reset, the sleep bit could be on
		or off depending on the OTP settings. Toggling power would
		make it in a definite state as well as making the hardware
		state align with the software state */
	result = inv_mpu6050_set_power_itg(st, false);
	if (result)
		return result;
	result = inv_mpu6050_set_power_itg(st, true);
	if (result)
		return result;

	result = inv_mpu6050_switch_engine(st, false,
					INV_MPU6050_BIT_PWR_ACCL_STBY);
	if (result)
		return result;
	result = inv_mpu6050_switch_engine(st, false,
					INV_MPU6050_BIT_PWR_GYRO_STBY);
	if (result)
		return result;

	return 0;
}

int inv_mpu6050_read_fifo(struct inv_mpu6050_state *st)
{
	size_t bytes_per_datum;
	int result;
	u8 data[INV_MPU6050_OUTPUT_DATA_SIZE];
	u16 fifo_count;
    int index;

	mutex_lock(&st->mlock);     //debug
	bytes_per_datum = 0;

	if (st->chip_config.accl_fifo_enable)
		bytes_per_datum += INV_MPU6050_BYTES_PER_3AXIS_SENSOR;

	if (st->chip_config.gyro_fifo_enable)
		bytes_per_datum += INV_MPU6050_BYTES_PER_3AXIS_SENSOR;

	/*
	 * read fifo_count register to know how many bytes inside FIFO
	 * right now
	 */
	result = i2c_smbus_read_i2c_block_data(st->client,
				       st->reg->fifo_count_h,
				       INV_MPU6050_FIFO_COUNT_BYTE, data);
	if (result != INV_MPU6050_FIFO_COUNT_BYTE) {
        printk(" get byte err\n");   
		goto end_session;
    }
	fifo_count = be16_to_cpup((__be16 *)(&data[0]));
	if (fifo_count < bytes_per_datum) {
        printk(" get data count err! read_c %d , need_c %d \n",fifo_count,bytes_per_datum);   
		goto end_session;
    }
	/* fifo count can't be odd number, if it is odd, reset fifo*/
	if (fifo_count & 1) {
        printk("get odd number \n");
		goto flush_fifo;
    }
	if (fifo_count >  INV_MPU6050_FIFO_THRESHOLD) {
        printk("get too much number \n");
		goto flush_fifo;
    }

	while (fifo_count >= bytes_per_datum) {
		result = i2c_smbus_read_i2c_block_data(st->client,
						       st->reg->fifo_r_w,
						       bytes_per_datum, data);
		if (result != bytes_per_datum) {
            printk(" read data err! read_c %d , need_c %d \n",result,bytes_per_datum);   
			goto flush_fifo;
        }

        index = 0;
        if (st->chip_config.accl_fifo_enable) {
            st->accel_data.xh = data[index++];
            st->accel_data.xl = data[index++];
            st->accel_data.yh = data[index++];
            st->accel_data.yl = data[index++];
            st->accel_data.zh = data[index++];
            st->accel_data.zl = data[index++];
        }

        if (st->chip_config.gyro_fifo_enable) {
            st->gyro_data.x = be16_to_cpup((__be16 *)(&data[index]));
            index += 2;
            st->gyro_data.y = be16_to_cpup((__be16 *)(&data[index]));
            index += 2;
            st->gyro_data.z = be16_to_cpup((__be16 *)(&data[index]));
        }

		fifo_count -= bytes_per_datum;
	}

	mutex_unlock(&st->mlock);
    return 1;    //debug

end_session:
	mutex_unlock(&st->mlock);
    return 0;    //debug

flush_fifo:
	/* Flush HW and SW FIFOs. */
	inv_reset_fifo(st);
	mutex_unlock(&st->mlock);
	return -1;    //debug
}

static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	

    int chipEnable;

    if (enable) {
        if (sensor->type == SENSOR_TYPE_ACCEL) 
            gMpu6050Stat.chip_config.accl_fifo_enable = true;

        if (sensor->type == SENSOR_TYPE_GYROSCOPE) 
            gMpu6050Stat.chip_config.gyro_fifo_enable = true;
    } else {
        if (sensor->type == SENSOR_TYPE_ACCEL) 
            gMpu6050Stat.chip_config.accl_fifo_enable = false;

        if (sensor->type == SENSOR_TYPE_GYROSCOPE) 
            gMpu6050Stat.chip_config.gyro_fifo_enable = false;
    }

    chipEnable = gMpu6050Stat.chip_config.gyro_fifo_enable | gMpu6050Stat.chip_config.accl_fifo_enable;

    return inv_mpu6050_set_enable(&gMpu6050Stat,chipEnable);
}


static int sensor_init(struct i2c_client *client)
{

	int result;
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	

    if (sensor->type == SENSOR_TYPE_ACCEL) {
        gMpu6050Stat.accel_client = client;
        gMpu6050Stat.client = client;
    } else if (sensor->type == SENSOR_TYPE_GYROSCOPE) {
        gMpu6050Stat.gyro_client = client;
        gMpu6050Stat.client = client;
    }

    if (gMpu6050Stat.initSt == 1)
        return 0;

    mutex_init(&gMpu6050Stat.mlock);
    result = inv_check_and_setup_chip(&gMpu6050Stat);
    if (result) {
        return -ENODEV;
    }

    result = inv_mpu6050_init_config(&gMpu6050Stat);
    if (result) {
        dev_err(&client->dev,
                "Could not initialize device.\n");
        return -ENODEV;
    }

    gMpu6050Stat.initSt = 1;

    return 0;
}

static int sensor_convert_data(struct i2c_client *client, char high_byte, char low_byte)
{
    s64 result;

    result = ((int)high_byte << (MPU6050_PRECISION-8)) 
            | ((int)low_byte >> (16-MPU6050_PRECISION));
    if (result < MPU6050_BOUNDARY)
        result = result* MPU6050_GRAVITY_STEP;
    else
        result = ~( ((~result & (0x7fff>>(16-MPU6050_PRECISION)) ) + 1) 
                * MPU6050_GRAVITY_STEP) + 1;

    return (int)result;
}

static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);	

	/* Report acceleration sensor information */
	input_report_abs(sensor->input_dev, ABS_X, axis->x);
	input_report_abs(sensor->input_dev, ABS_Y, axis->y);
	input_report_abs(sensor->input_dev, ABS_Z, axis->z);
	input_sync(sensor->input_dev);
	//DBG("Gsensor x==%d  y==%d z==%d\n",axis->x,axis->y,axis->z);

	return 0;
}

static int gyro_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
	    	(struct sensor_private_data *) i2c_get_clientdata(client);	

	/* Report GYRO  information */
	input_report_rel(sensor->input_dev, ABS_RX, axis->x);
	input_report_rel(sensor->input_dev, ABS_RY, axis->y);
	input_report_rel(sensor->input_dev, ABS_RZ, axis->z);
	input_sync(sensor->input_dev);
	//DBG("gyro x==%d  y==%d z==%d\n",axis->x,axis->y,axis->z);

	return 0;
}

static int sensor_report_value(struct i2c_client *client)
{
    struct inv_mpu6050_gyro_data *gyro_data = &(gMpu6050Stat.gyro_data);
    struct inv_mpu6050_accel_data *accel_data = &(gMpu6050Stat.accel_data);
	struct sensor_private_data *sensor;
    struct sensor_platform_data *pdata;
	struct sensor_axis axis;
	int x,y,z;
	int result;

    //if gyro & accel all enable, only need one time read fifo, default use ACCEL 
    if (gMpu6050Stat.chip_config.accl_fifo_enable &&  gMpu6050Stat.chip_config.gyro_fifo_enable) {
        sensor = (struct sensor_private_data *) i2c_get_clientdata(client);
        if (sensor->type == SENSOR_TYPE_GYROSCOPE)
            return 0;
    }

    if( !(gMpu6050Stat.chip_config.accl_fifo_enable | gMpu6050Stat.chip_config.gyro_fifo_enable) )
        return 0;

    result = inv_mpu6050_read_fifo(&gMpu6050Stat);
    if (result <= 0)
        return result;

    if (gMpu6050Stat.chip_config.accl_fifo_enable) {

        sensor = (struct sensor_private_data *) i2c_get_clientdata(gMpu6050Stat.accel_client);
        pdata = sensor->pdata;

        //this gsensor need 6 bytes buffer
        x = sensor_convert_data(sensor->client, accel_data->xh,accel_data->xl);	//buffer[1]:high bit 
        y = sensor_convert_data(sensor->client, accel_data->yh,accel_data->yl);
        z = sensor_convert_data(sensor->client, accel_data->zh,accel_data->zl);		

        axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
        axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z; 
        axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;

        //Report event only while value is changed to save some power
        if((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) 
            || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) 
            || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN))
        {
            gsensor_report_value(gMpu6050Stat.accel_client, &axis);

            mutex_lock(&(sensor->data_mutex) );
            sensor->axis = axis;
            mutex_unlock(&(sensor->data_mutex) );
        }
    }

    if (gMpu6050Stat.chip_config.gyro_fifo_enable) {

        sensor = (struct sensor_private_data *) i2c_get_clientdata(gMpu6050Stat.gyro_client);
        pdata = sensor->pdata;

        x = gyro_data->x;
        y = gyro_data->y;
        z = gyro_data->z;
        axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
        axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z;	
        axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;

        //filter gyro data
        if((abs(axis.x) > pdata->x_min)||(abs(axis.y) > pdata->y_min)||(abs(axis.z) > pdata->z_min))
        {
            gyro_report_value(gMpu6050Stat.gyro_client, &axis);
            mutex_lock(&(sensor->data_mutex) );
            sensor->axis = axis;
            mutex_unlock(&(sensor->data_mutex) );
        }
    }

    return 0;
}


struct sensor_operate accel_mp6050_ops = {
	.name				= "mp6050_acc",
	.type				= SENSOR_TYPE_ACCEL,//sensor type and it should be correct
	.id_i2c				= ACCEL_ID_MP6050,		//i2c id number
	.id_reg				= INV_MPU6050_REG_WHO_AM_I,		//read device id from this register
	.id_data 			= GYRO_DEVID_MP6050,	//device id
	.precision			= 16,				
	.range				= {-MPU6050_RANGE,MPU6050_RANGE},		//range
	.active				= sensor_active,	
	.init				= sensor_init,
	.report				= sensor_report_value,
};

struct sensor_operate gyro_mp6050_ops = {
	.name				= "mp6050_gyro",
	.type				= SENSOR_TYPE_GYROSCOPE,//sensor type and it should be correct
	.id_i2c				= GYRO_ID_MP6050,		//i2c id number
	.id_reg				= INV_MPU6050_REG_WHO_AM_I,
	.id_data 			= GYRO_DEVID_MP6050,	//device id
	.precision			= 16,
	.range				= {-32768,32768},		//range
	.active				= sensor_active,	
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *gyro_get_ops(void)
{
	return &gyro_mp6050_ops;
}

static struct sensor_operate *accel_get_ops(void)
{
	return &accel_mp6050_ops;
}

static int __init gyro_mp6050_init(void)
{
	struct sensor_operate *ops = accel_get_ops();
	int result = 0;
	int type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, accel_get_ops);
    if (result)
        return result;
    ops = gyro_get_ops();
    type = ops->type;
	result = sensor_register_slave(type, NULL, NULL, gyro_get_ops);
        return result;
}

static void __exit gyro_mp6050_exit(void)
{
	struct sensor_operate *ops = accel_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, accel_get_ops);
    ops = gyro_get_ops();
    type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, gyro_get_ops);
}


module_init(gyro_mp6050_init);
module_exit(gyro_mp6050_exit);


