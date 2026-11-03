/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2026.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)             "[sc533hgs]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include "hb_i2c.h"
#include "hb_cam_utility.h"
#include "inc/sc533hgs_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define REG_WIDTH               2
#define SC533HGS_VTS_HIGH       0x320e
#define SC533HGS_VTS_LOW        0x320f
#define SC533HGS_EXP_LINE0      0x3e00
#define SC533HGS_EXP_LINE1      0x3e01
#define SC533HGS_EXP_LINE2      0x3e02
#define SC533HGS_DGAIN_COARSE   0x3e06
#define SC533HGS_DGAIN_FINE     0x3e07
#define SC533HGS_AGAIN_COARSE   0x3e08
#define SC533HGS_AGAIN_FINE     0x3e09

static int sc533hgs_linear_data_init(sensor_info_t *sensor_info);

int sensor_poweroff(sensor_info_t *sensor_info)
{
        int gpio, ret = RET_OK;
        if (sensor_info->gpio_num > 0) {
                for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
                        if (sensor_info->gpio_pin[gpio] != -1) {
                                ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
                                        sensor_info->gpio_level[gpio]);
                                if (ret < 0) {
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

int sensor_poweron(sensor_info_t *sensor_info)
{
        int gpio, ret = RET_OK;
        vin_dbg("%s gpio_num = %d\n", sensor_info->sensor_name, sensor_info->gpio_num);
        if (sensor_info->gpio_num > 0) {
                for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
                        vin_dbg("%s gpio_pin[%d] = %d\n", sensor_info->sensor_name,
                                gpio, sensor_info->gpio_pin[gpio]);
                        if (sensor_info->gpio_pin[gpio] != -1) {
                                ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
                                        sensor_info->gpio_level[gpio]);
                                usleep(100 * 1000);
                                ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
                                        1 - sensor_info->gpio_level[gpio]);
                                if (ret < 0) {
                                        vin_err("vin_power_ctrl fail\n");
                                        return -HB_CAM_SENSOR_POWERON_FAIL;
                                }
                                usleep(100 * 1000);
                        }
                }
        }

        return ret;
}

static int sc533hgs_write_init_setting(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        int setting_size = 0;
        uint32_t *setting = NULL;

        if (sensor_info->width == SC533HGS_WIDTH &&
            sensor_info->height == SC533HGS_HEIGHT && sensor_info->fps == 60) {
                setting = sc533hgs_linear_60fps_4lane_init_setting;
                setting_size = sizeof(sc533hgs_linear_60fps_4lane_init_setting) /
                        sizeof(uint32_t) / 2;
        } else if (sensor_info->sensor_mode == SLAVE_M) {
                if (sensor_info->config_index == 0) {
                        setting = sc533hgs_linear_30fps_2lane_slave_init_setting;
                        setting_size = sizeof(sc533hgs_linear_30fps_2lane_slave_init_setting) /
                                sizeof(uint32_t) / 2;
                } else {
                        setting = sc533hgs_linear_30fps_4lane_slave_init_setting;
                        setting_size = sizeof(sc533hgs_linear_30fps_4lane_slave_init_setting) /
                                sizeof(uint32_t) / 2;
                }
        } else {
                if (sensor_info->config_index == 0) {
                        setting = sc533hgs_linear_30fps_2lane_init_setting;
                        setting_size = sizeof(sc533hgs_linear_30fps_2lane_init_setting) /
                                sizeof(uint32_t) / 2;
                } else {
                        setting = sc533hgs_linear_30fps_4lane_init_setting;
                        setting_size = sizeof(sc533hgs_linear_30fps_4lane_init_setting) /
                                sizeof(uint32_t) / 2;
                }
        }

        ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
                REG_WIDTH, setting_size, setting);
        if (ret < 0) {
                vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
                return -HB_CAM_I2C_WRITE_FAIL;
        }

        return sc533hgs_linear_data_init(sensor_info);
}

int sensor_init(sensor_info_t *sensor_info)
{
        int ret = RET_OK;

        ret = sensor_poweron(sensor_info);
        if (ret < 0) {
                vin_err("%d : sensor reset %s fail\n", __LINE__, sensor_info->sensor_name);
                return ret;
        }

        switch (sensor_info->sensor_mode) {
                case NORMAL_M:
                case SLAVE_M:
                        vin_info("sc533hgs linear mode, fps=%d, config_index=%d\n",
                                sensor_info->fps, sensor_info->config_index);
                        if (sensor_info->fps != SC533HGS_FPS &&
                            !(sensor_info->fps == 60 && sensor_info->sensor_mode == NORMAL_M &&
                              sensor_info->width == SC533HGS_WIDTH &&
                              sensor_info->height == SC533HGS_HEIGHT)) {
                                vin_err("sc533hgs supports 30fps or 2592x1944 normal 60fps\n");
                                return -HB_CAM_INIT_FAIL;
                        }
                        ret = sc533hgs_write_init_setting(sensor_info);
                        if (ret < 0) {
                                vin_err("%d : linear data init %s fail\n",
                                        __LINE__, sensor_info->sensor_name);
                                return -HB_CAM_INIT_FAIL;
                        }
                        break;
                case DOL2_M:
                default:
                        vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
                        ret = -HB_CAM_INIT_FAIL;
                        break;
        }

        vin_info("sc533hgs config success under %d mode\n", sensor_info->sensor_mode);
        return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        int setting_size = sizeof(sc533hgs_stream_on_setting) / sizeof(uint32_t) / 2;

        switch (sensor_info->sensor_mode) {
                case NORMAL_M:
                case SLAVE_M:
                        vin_info("%s start normal/slave linear mode\n", sensor_info->sensor_name);
                        ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
                                REG_WIDTH, setting_size, sc533hgs_stream_on_setting);
                        if (ret < 0) {
                                vin_err("start %s fail\n", sensor_info->sensor_name);
                                return -HB_CAM_I2C_WRITE_FAIL;
                        }
                        break;
                default:
                        vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
                        ret = -HB_CAM_START_FAIL;
                        break;
        }
        return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        int setting_size = sizeof(sc533hgs_stream_off_setting) / sizeof(uint32_t) / 2;

        vin_info("%s sensor stop\n", sensor_info->sensor_name);
        ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
                REG_WIDTH, setting_size, sc533hgs_stream_off_setting);
        if (ret < 0) {
                vin_err("stop %s fail\n", sensor_info->sensor_name);
                return -HB_CAM_I2C_WRITE_FAIL;
        }

        return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
        int ret = sensor_poweroff(sensor_info);
        if (ret < 0) {
                vin_err("%d : deinit %s fail\n", __LINE__, sensor_info->sensor_name);
                return ret;
        }
        return ret;
}

static int sc533hgs_linear_data_init(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        uint16_t vts_high;
        uint16_t vts_low;
        uint32_t vts_value;
        uint32_t lines_per_second;
        sensor_turning_data_t turning_data;
        uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
        uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

        memset(&turning_data, 0, sizeof(sensor_turning_data_t));

        turning_data.bus_num = sensor_info->bus_num;
        turning_data.bus_type = sensor_info->bus_type;
        turning_data.port = sensor_info->port;
        turning_data.reg_width = sensor_info->reg_width;
        turning_data.mode = sensor_info->sensor_mode;
        if (sensor_info->sensor_mode == SLAVE_M) {
                turning_data.mode = NORMAL_M;
        }
        turning_data.sensor_addr = sensor_info->sensor_addr;
        strncpy(turning_data.sensor_name, sensor_info->sensor_name,
                sizeof(turning_data.sensor_name));

        turning_data.sensor_data.active_width = SC533HGS_WIDTH;
        turning_data.sensor_data.active_height = SC533HGS_HEIGHT;
        vts_high = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
                sensor_info->sensor_addr, SC533HGS_VTS_HIGH);
        vts_low = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
                sensor_info->sensor_addr, SC533HGS_VTS_LOW);
        vts_value = (vts_high << 8) | vts_low;
        if (vts_value == 0) {
                vts_value = (sensor_info->fps == 60) ? 2000 :
                        ((sensor_info->config_index == 0) ?
                         SC533HGS_2LANE_VTS : SC533HGS_4LANE_VTS);
        }
        lines_per_second = (sensor_info->config_index == 0) ?
                SC533HGS_2LANE_LINES_PER_SECOND : SC533HGS_4LANE_LINES_PER_SECOND;

        turning_data.sensor_data.lines_per_second = lines_per_second;
        turning_data.sensor_data.exposure_time_max =
                vts_value - SC533HGS_EXPOSURE_MAX_MARGIN;
        turning_data.sensor_data.exposure_time_min = SC533HGS_EXPOSURE_MIN;
        turning_data.sensor_data.exposure_time_long_max =
                2 * vts_value - SC533HGS_EXPOSURE_MAX_MARGIN;
        turning_data.sensor_data.analog_gain_max =
                sizeof(sc533hgs_gain_lut) / sizeof(uint32_t) - 1;
        turning_data.sensor_data.digital_gain_max = 0;
        turning_data.sensor_data.analog_gain_init = 0;
        turning_data.sensor_data.digital_gain_init = 0;
        turning_data.sensor_data.exposure_time_init = 1956;

        sensor_data_bayer_fill(&turning_data.sensor_data, 10,
                (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
        sensor_data_bits_fill(&turning_data.sensor_data, 10);

        turning_data.stream_ctrl.data_length = 1;
        if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc533hgs_stream_on_setting)) {
                memcpy(stream_on, sc533hgs_stream_on_setting, sizeof(sc533hgs_stream_on_setting));
        } else {
                vin_err("Number of registers on stream over limit\n");
                return -RET_ERROR;
        }
        if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc533hgs_stream_off_setting)) {
                memcpy(stream_off, sc533hgs_stream_off_setting,
                        sizeof(sc533hgs_stream_off_setting));
        } else {
                vin_err("Number of registers off stream over limit\n");
                return -RET_ERROR;
        }

        turning_data.normal.again_lut = malloc(sizeof(sc533hgs_gain_lut));
        if (turning_data.normal.again_lut != NULL) {
                memcpy(turning_data.normal.again_lut, sc533hgs_gain_lut,
                        sizeof(sc533hgs_gain_lut));
        }

        ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);

        if (turning_data.normal.again_lut) {
                free(turning_data.normal.again_lut);
                turning_data.normal.again_lut = NULL;
        }

        if (ret < 0) {
                vin_err("%s sync turning data ioctl fail %d\n", sensor_info->sensor_name, ret);
                return -RET_ERROR;
        }

        return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode,
        uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
        char ana_gain = 0;
        char ana_fine_gain = 0;
        char dig_gain = 0;
        char dig_fine_gain = 0x80;
        int again_index = 0;

        if (mode == NORMAL_M || mode == SLAVE_M) {
                if (again[0] >= sizeof(sc533hgs_gain_lut) / sizeof(uint32_t)) {
                        again_index = sizeof(sc533hgs_gain_lut) / sizeof(uint32_t) - 1;
                } else {
                        again_index = again[0];
                }

                ana_gain = (sc533hgs_gain_lut[again_index] >> 8) & 0x0f;
                ana_fine_gain = sc533hgs_gain_lut[again_index] & 0x3f;

                vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
                        SC533HGS_AGAIN_COARSE, ana_gain);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
                        SC533HGS_AGAIN_FINE, ana_fine_gain);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
                        SC533HGS_DGAIN_COARSE, dig_gain);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
                        SC533HGS_DGAIN_FINE, dig_fine_gain);
        } else {
                vin_err("unsupport mode %d\n", mode);
        }

        return 0;
}

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode,
        uint32_t *line, uint32_t line_num)
{
        uint32_t sline = line[0];
        uint16_t vts_high;
        uint16_t vts_low;
        uint32_t vts_value;
        uint32_t max_line;
        char temp0 = 0;
        char temp1 = 0;
        char temp2 = 0;

        if (mode == NORMAL_M || mode == SLAVE_M) {
                vts_high = hb_vin_i2c_read_reg16_data8(info->bus_num,
                        info->sensor_addr, SC533HGS_VTS_HIGH);
                vts_low = hb_vin_i2c_read_reg16_data8(info->bus_num,
                        info->sensor_addr, SC533HGS_VTS_LOW);
                vts_value = (vts_high << 8) | vts_low;
                if (vts_value == 0) {
                        vts_value = SC533HGS_2LANE_VTS;
                }
                max_line = vts_value - SC533HGS_EXPOSURE_MAX_MARGIN;

                if (sline < SC533HGS_EXPOSURE_MIN) {
                        sline = SC533HGS_EXPOSURE_MIN;
                }
                if (sline > max_line) {
                        sline = max_line;
                }

                temp0 = (sline >> 12) & 0x0f;
                temp1 = (sline >> 4) & 0xff;
                temp2 = (sline & 0x0f) << 4;
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
                        SC533HGS_EXP_LINE0, temp0);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
                        SC533HGS_EXP_LINE1, temp1);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr,
                        SC533HGS_EXP_LINE2, temp2);
        } else {
                vin_err("unsupport mode %d\n", mode);
        }

        return 0;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
        vin_info("enable userspace gain control and line control\n");
        *enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
        return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(sc533hgs, CAM_MODULE_FLAG_A16D8);
sensor_module_t sc533hgs = {
        .module = SENSOR_MNAME(sc533hgs),
#else
sensor_module_t sc533hgs = {
        .module = "sc533hgs",
#endif
        .init = sensor_init,
        .start = sensor_start,
        .stop = sensor_stop,
        .deinit = sensor_deinit,
        .power_on = sensor_poweron,
        .power_off = sensor_poweroff,
        .aexp_gain_control = sensor_aexp_gain_control,
        .aexp_line_control = sensor_aexp_line_control,
        .userspace_control = sensor_userspace_control,
};
