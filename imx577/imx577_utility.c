/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)		"[imx577]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "hb_cam_utility.h"
#include "hb_camera_data_config.h"
#include "inc/imx577_setting.h"
#define REG_WIDTH	2	//reg16 data8

static uint32_t imx577_exposure_limit[CAM_MAX_NUM];
static int imx577_linear_data_init(sensor_info_t *sensor_info);

static int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if(sensor_info->gpio_num > 0) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
					sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWEROFF_FAIL;
				}
			}
		}
	}

        if (sensor_info->sen_devfd != 0) {
                close(sensor_info->sen_devfd);
                sensor_info->sen_devfd = -1;
        }
	return ret;
}

static int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	vin_dbg("%s gpio_num = %d \n", sensor_info->sensor_name, sensor_info->gpio_num);
	if(sensor_info->gpio_num > 0) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			vin_dbg("%s gpio_pin[%d] = %d \n", sensor_info->sensor_name, gpio, sensor_info->gpio_pin[gpio]);
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
					sensor_info->gpio_level[gpio]);
				usleep(100 * 1000);  //100ms
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
					1 - sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100 * 1000);  //100ms
			}
		}
	}

	return ret;
}

/* Read both bytes even if the first read fails. reg_width is in bits here. */
static int imx577_read_u16(sensor_info_t *sensor_info, uint16_t reg)
{
	int high = vin_i2c_read8(sensor_info->bus_num, 16,
		sensor_info->sensor_addr, reg);
	int low = vin_i2c_read8(sensor_info->bus_num, 16,
		sensor_info->sensor_addr, reg + 1);

	if (high < 0 || low < 0)
		return -RET_ERROR;

	return (high << 8) | low;
}

static int imx577_check_readback(sensor_info_t *sensor_info,
	const imx577_mode_setting_t *setting)
{
	int vts = imx577_read_u16(sensor_info, 0x0340);
	int hts = imx577_read_u16(sensor_info, 0x0342);
	int width;
	int height;

	if (vts >= 0 && hts >= 0)
		vin_info("IMX577 readback VTS=%d HTS=%d\n", vts, hts);

	if (setting->expected_hts == 0 && setting->expected_vts == 0)
		return RET_OK;

	width = imx577_read_u16(sensor_info, 0x034c);
	height = imx577_read_u16(sensor_info, 0x034e);
	if (vts < 0 || hts < 0 || width < 0 || height < 0) {
		vin_err("IMX577 timing/output readback failed\n");
		return -RET_ERROR;
	}

	vin_info("IMX577 readback: %dx%d HTS=%d VTS=%d\n", width, height, hts, vts);
	if ((uint32_t)vts != setting->expected_vts ||
	    (uint32_t)hts != setting->expected_hts ||
	    (uint32_t)width != setting->width || (uint32_t)height != setting->height) {
		vin_err("IMX577 readback mismatch: expected %ux%u HTS=%u VTS=%u\n",
			setting->width, setting->height, setting->expected_hts, setting->expected_vts);
		return -RET_ERROR;
	}

	return RET_OK;
}

static int sensor_init(sensor_info_t *sensor_info)
{
	const imx577_mode_setting_t *setting = NULL;
	size_t i;
	int ret;
	int readback_ret;

	if (sensor_info == NULL) {
		vin_err("IMX577 sensor_info is NULL\n");
		return -RET_ERROR;
	}

	for (i = 0; i < sizeof(imx577_mode_settings) / sizeof(imx577_mode_settings[0]); i++) {
		const imx577_mode_setting_t *candidate = &imx577_mode_settings[i];

		if (sensor_info->width == candidate->width &&
		    sensor_info->height == candidate->height &&
		    sensor_info->fps == candidate->fps && sensor_info->format == candidate->format) {
			setting = candidate;
			break;
		}
	}
	if (setting == NULL || sensor_info->sensor_mode != NORMAL_M ||
	    sensor_info->port < 0 || sensor_info->port >= CAM_MAX_NUM) {
		vin_err("unsupported IMX577 configuration: %dx%d@%d format=0x%x mode=%d port=%d\n",
			(int)sensor_info->width, (int)sensor_info->height,
			(int)sensor_info->fps, (unsigned int)sensor_info->format,
			(int)sensor_info->sensor_mode, (int)sensor_info->port);
		return -RET_ERROR;
	}
	imx577_exposure_limit[sensor_info->port] = setting->exposure_limit;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor reset %s fail\n",
			__LINE__, sensor_info->sensor_name);
		return ret;
	}

	for (i = 0; i < setting->init_setting_count; i++) {
		const imx577_init_setting_t *init = &setting->init_settings[i];

		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
			init->reg_count, init->regs);
		if (ret < 0) {
			vin_err("IMX577 init setting %zu/%u failed: %d\n",
				i + 1, setting->init_setting_count, ret);
			return ret;
		}
	}

	ret = imx577_linear_data_init(sensor_info);
	if (ret < 0)
		return ret;

	readback_ret = imx577_check_readback(sensor_info, setting);
	if (readback_ret < 0)
		return readback_ret;

	vin_info("imx577 config success under %d mode\n\n", sensor_info->sensor_mode);
	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

        switch(sensor_info->sensor_mode) {
                case NORMAL_M:
                case SLAVE_M:
	setting_size = sizeof(imx577_stream_on_setting)/sizeof(uint32_t)/2;
	vin_info("%s start normal/linear mode\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
			setting_size, imx577_stream_on_setting);
	if(ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}
                        break;
                case DOL2_M:
                default:
                        vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
                        ret = -HB_CAM_START_FAIL;
                        break;
        }
	return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	// linear and hdr mode use one stream_off setting
	setting_size =
		sizeof(imx577_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("%s sensor stop\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num,
			sensor_info->sensor_addr, REG_WIDTH,
			setting_size, imx577_stream_off_setting);
	if (ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}

	return ret;
}

static int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sensor_poweroff(sensor_info);
	if (ret < 0) {
		vin_err("%d : deinit %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret; //-HB_CAM_SENSOR_POWEROFF_FAIL
	}
	return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
        printf("test %s, mode = %d gain_num = %d again[0] = %d, dgain[0] = %d\n", __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif
    const uint16_t AGAIN_H = 0x0204;
	const uint16_t AGAIN_L = 0x0205;
	char again_reg_value_h = 0;
	char again_reg_value_l = 0;
	int gain_index = 0;

        if (mode == NORMAL_M) {
	        if (again[0] >= sizeof(imx577_gain_lut)/sizeof(uint32_t))
			gain_index = sizeof(imx577_gain_lut)/sizeof(uint32_t) - 1;
		else
			gain_index = again[0];

		again_reg_value_h = (imx577_gain_lut[gain_index] >> 8) & 0x03;
		again_reg_value_l = (imx577_gain_lut[gain_index] >> 0) & 0xFF;
#ifdef AE_DBG
                printf("%s, gain_index: %d, again:0x0204 = 0x%x,again:0x0205 = 0x%x\n",
				__FUNCTION__, gain_index, again_reg_value_h,again_reg_value_l);
#endif
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_H, again_reg_value_h);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_L, again_reg_value_l);

	} else	{
		vin_err(" unsupport mode %d\n", mode);
	}

    return 0;
}
static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
#ifdef AE_DBG
        printf("line mode %d, --line %d , line_num:%d \n", mode, line[0], line_num);
#endif
    const uint16_t EXP_LINE0 = 0x0202;
	const uint16_t EXP_LINE1 = 0x0203;
	char temp0 = 0, temp1 = 0;

        if (mode == NORMAL_M) {
		uint32_t sline =  line[0];
		if (info->port >= CAM_MAX_NUM || imx577_exposure_limit[info->port] == 0)
			return -RET_ERROR;
		if (sline > imx577_exposure_limit[info->port])
			sline = imx577_exposure_limit[info->port];
		if (sline < 8) sline = 8;

		temp0 = (sline >> 8) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
		temp1 = (sline) & 0xFF;
        vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
#ifdef AE_DBG
        printf("write sline = %d, 0x0202 = 0x%x, 0x0203 = 0x%x\n",
                        sline, temp0, temp1);
#endif
        } else {
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}
static int imx577_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
		sizeof(turning_data.sensor_name));

	turning_data.sensor_data.active_width = sensor_info->width;
	turning_data.sensor_data.active_height = sensor_info->height;

	turning_data.sensor_data.lines_per_second = sensor_info->format == 0x2b ? 93500 : 70560;
	turning_data.sensor_data.exposure_time_max = imx577_exposure_limit[sensor_info->port];
	turning_data.sensor_data.exposure_time_min = 8;
	turning_data.sensor_data.analog_gain_max = sizeof(imx577_gain_lut) / sizeof(imx577_gain_lut[0]) - 1;
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.analog_gain_init = 32;
	turning_data.sensor_data.digital_gain_init = 0;
	turning_data.sensor_data.exposure_time_init = 64;
	uint32_t bit_depth = sensor_info->format == 0x2b ? 10 : 12;
	//sensor bit && bayer
	sensor_data_bayer_fill(&turning_data.sensor_data, bit_depth, (uint32_t)BAYER_START_R, (uint32_t)BAYER_PATTERN_RGGB);
	// sensor exposure_max_bit, maybe not used ?  //FIXME
	sensor_data_bits_fill(&turning_data.sensor_data, bit_depth);

	//some stress test case, we need kernel stream_ctrl.
	turning_data.stream_ctrl.data_length = 1;

	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(imx577_stream_on_setting)) {
		memcpy(stream_on, imx577_stream_on_setting, sizeof(imx577_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(imx577_stream_off_setting)) {
		memcpy(stream_off, imx577_stream_off_setting, sizeof(imx577_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	// sync gain lut to kernel driver.
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx577_gain_lut,
			sizeof(imx577_gain_lut));
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);

	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}

	if (ret < 0) {
		vin_err("%s sync gain lut ioctl fail %d\n", sensor_info->sensor_name, ret);
		return -RET_ERROR;
	}

	return ret;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	vin_info("enable userspace gain control and line control\n");
	// *enable = 0;	//imx577 use kernel space gain contrl and line control
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	return 0;
}


#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(imx577, CAM_MODULE_FLAG_A16D8);
sensor_module_t imx577 = {
	.module = SENSOR_MNAME(imx577),
#else
sensor_module_t imx577 = {
	.module = "imx577",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.aexp_gain_control = sensor_aexp_gain_control,
	.aexp_line_control = sensor_aexp_line_control,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.userspace_control = sensor_userspace_control,
};
