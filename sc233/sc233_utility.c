// Copyright (c) 2024锛孌-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)             "[sc233]:" fmt

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
#include "inc/sc233_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

//#define AE_DBG

#define SC233_VTS_HI (0x320E)
#define SC233_VTS_LO (0x320F)

// 16鈥檋3e03[3]璁剧疆涓�1鈥檅0鏃讹細澧炵泭鍊� = {16鈥檋3e08,16鈥檋3e09}/8鈥檋40
// 璇ュ鐩婂€煎寘鍚暟瀛楀鐩婂拰妯℃嫙澧炵泭锛宻ensor鍐呴儴鑷姩浼樺厛鍒嗛厤妯℃嫙澧炵泭锛屽綋妯℃嫙澧炵泭杈惧埌鏈€澶фā鎷熷鐩婂悗鑷姩寮€鍚暟瀛楀鐩娿€�
// 2)
// 16鈥檋3e03[3]璁剧疆涓�1鈥檅1鏃�,鏁板瓧澧炵泭鍜屾ā鎷熷鐩婂彲浠ュ垎鍒嫭绔嬫帶鍒�
#define SC233_GAIN_MODE (0x3e03)
#define SC233_AGAIN_C (0x3e08) //bit[3:0]
#define SC233_AGAIN_F (0x3e09) //bit[7:0]
#define SC233_DGAIN_C (0x3e06) //bit[2:0]
#define SC233_DGAIN_F (0x3e07) //bit[7:0]

/* 120fps VTS 1250, max exposure = VTS - 6 */
static uint32_t sc233_exposure_max = 1244;

int sc233_linear_data_init(sensor_info_t *sensor_info);

int sensor_poweroff(sensor_info_t *sensor_info)
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

int sensor_poweron(sensor_info_t *sensor_info)
{
        int gpio, ret = RET_OK;
        vin_dbg("%s gpio_num = %d \n", sensor_info->sensor_name, sensor_info->gpio_num);
        if(sensor_info->gpio_num > 0) {
                for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
                        vin_dbg("%s gpio_pin[%d] = %d \n", sensor_info->sensor_name, gpio, sensor_info->gpio_pin[gpio]);
                        if(sensor_info->gpio_pin[gpio] != -1) {
                                ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
                                        sensor_info->gpio_level[gpio]);
                                //usleep(100 * 1000);  //100ms
                                ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
                                        1 - sensor_info->gpio_level[gpio]);
                                if(ret < 0) {
                                        vin_err("vin_power_ctrl fail\n");
                                        return -HB_CAM_SENSOR_POWERON_FAIL;
                                }
                                //usleep(100 * 1000);  //100ms
                        }
                }
        }

        return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        int setting_size = 0;

        ret = sensor_poweron(sensor_info);
        if (ret < 0) {
                vin_err("%d : sensor reset %s fail\n",
                        __LINE__, sensor_info->sensor_name);
                return ret;  //-HB_CAM_SENSOR_POWERON_FAIL
        }

        vin_info("sc233 in normal linear mode\n");
        vin_info("bus_num = %d, sensor_addr = 0x%0x\n", sensor_info->bus_num, sensor_info->sensor_addr);

        switch(sensor_info->sensor_mode) {
                case NORMAL_M:  // 1: normal
                        {
                        uint32_t *setting = NULL;

                        if (sensor_info->fps == 30) {
                                setting_size = sizeof(sc233_24Minput_4lane_540Mbps_10bit_1920x1080_30fps_setting) / sizeof(uint32_t) / 2;
                                setting = sc233_24Minput_4lane_540Mbps_10bit_1920x1080_30fps_setting;
                        } else if (sensor_info->fps == 60) {
                                setting_size = sizeof(sc233_24Minput_4lane_540Mbps_10bit_1920x1080_60fps_setting) / sizeof(uint32_t) / 2;
                                setting = sc233_24Minput_4lane_540Mbps_10bit_1920x1080_60fps_setting;
                        } else if (sensor_info->fps == 240) {
                                setting_size = sizeof(sc233_24Minput_4lane_810Mbps_10bit_960x600_240fps_setting) / sizeof(uint32_t) / 2;
                                setting = sc233_24Minput_4lane_810Mbps_10bit_960x600_240fps_setting;
                        } else {
                                setting_size = sizeof(sc233_24Minput_4lane_810Mbps_10bit_1920x1080_120fps_setting) / sizeof(uint32_t) / 2;
                                setting = sc233_24Minput_4lane_810Mbps_10bit_1920x1080_120fps_setting;
                        }
                        vin_info("sc233 fps = %u\n", sensor_info->fps);
                        ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
                                setting_size, setting);
                        if (ret < 0) {
                                vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
                                return -HB_CAM_I2C_WRITE_FAIL;
                        }
                        }
                        if(sensor_info->fps != 30 && sensor_info->fps != 60 &&
                                        sensor_info->fps != 120 && sensor_info->fps != 240) {

                                int vts_hi = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, SC233_VTS_HI);
                                int vts_lo = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, SC233_VTS_LO);
                                uint32_t vts = vts_hi;
                                vts = vts << 8 | vts_lo;
                                printf("SC233 read: vts_hi:0x%x,vts_lo:0x%x,vts:0x%x\n", vts_hi, vts_lo, vts);
                                vts = vts * 120 / sensor_info->fps;
                                vts_hi = vts >> 8;
                                vts_lo = vts & 0xff;
                                printf("SC233 write: vts_hi:0x%x,vts_lo:0x%x,vts:0x%x\n", vts_hi, vts_lo, vts);
                                vin_i2c_write8(sensor_info->bus_num, 16, sensor_info->sensor_addr, SC233_VTS_HI, vts_hi);
                                vin_i2c_write8(sensor_info->bus_num, 16, sensor_info->sensor_addr, SC233_VTS_LO, vts_lo);
                        }

                        break;
                default:
                        vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
                        ret = -HB_CAM_INIT_FAIL;
                        break;
        }

        ret = sc233_linear_data_init(sensor_info);
        if (ret < 0) {
                vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
                return -HB_CAM_INIT_FAIL;
        }

        vin_info("sc233 config success under %d mode\n", sensor_info->sensor_mode);

        return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        int setting_size = 0;

        switch(sensor_info->sensor_mode) {
                case NORMAL_M:
                case SLAVE_M:
                        setting_size = sizeof(sc233_stream_on_setting)/sizeof(uint32_t)/2;
                        vin_info("%s start normal / slave linear mode\n", sensor_info->sensor_name);
                        ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
                                        setting_size, sc233_stream_on_setting);
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

int sensor_stop(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        int setting_size = 0;

        setting_size =
                sizeof(sc233_stream_off_setting) / sizeof(uint32_t) / 2;
        vin_info("%s sensor stop\n", sensor_info->sensor_name);
        ret = vin_write_array(sensor_info->bus_num,
                        sensor_info->sensor_addr, 2,
                        setting_size, sc233_stream_off_setting);
        if (ret < 0) {
                vin_err("start %s fail\n", sensor_info->sensor_name);
                return -HB_CAM_I2C_WRITE_FAIL;
        }

        return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
        int ret = RET_OK;

        ret = sensor_poweroff(sensor_info);
        if (ret < 0) {
                vin_err("%d : deinit %s fail\n", __LINE__, sensor_info->sensor_name);
                return ret; //-HB_CAM_SENSOR_POWEROFF_FAIL
        }
        return ret;
}

int sc233_linear_data_init(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        uint32_t  open_cnt = 0;
        sensor_turning_data_t turning_data;
        uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
        uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

        uint16_t VTS_HIGH;
        uint16_t VTS_LOW;
        uint32_t VTS_VALUE;

        memset(&turning_data, 0, sizeof(sensor_turning_data_t));

        // common data
        turning_data.bus_num = sensor_info->bus_num;
        turning_data.bus_type = sensor_info->bus_type;
        turning_data.port = sensor_info->port;
        turning_data.reg_width = sensor_info->reg_width;
        turning_data.mode = sensor_info->sensor_mode;
        if (sensor_info->sensor_mode == SLAVE_M)
                turning_data.mode = NORMAL_M;
        turning_data.sensor_addr = sensor_info->sensor_addr;
        strncpy(turning_data.sensor_name, sensor_info->sensor_name,
                sizeof(turning_data.sensor_name));

        if (sensor_info->fps == 30) {
                turning_data.sensor_data.active_width = 1920;
                turning_data.sensor_data.active_height = 1080;
                /* VTS 0x0c80 = 3200, max exposure = VTS - 6 */
                turning_data.sensor_data.lines_per_second = 3200 * 30;
                turning_data.sensor_data.exposure_time_max = 3194;
        } else if (sensor_info->fps == 60) {
                turning_data.sensor_data.active_width = 1920;
                turning_data.sensor_data.active_height = 1080;
                /* VTS 0x0640 = 1600, max exposure = VTS - 6 */
                turning_data.sensor_data.lines_per_second = 1600 * 60;
                turning_data.sensor_data.exposure_time_max = 1594;
        } else if (sensor_info->fps == 240) {
                turning_data.sensor_data.active_width = 960;
                turning_data.sensor_data.active_height = 600;
                /* VTS 0x0272 = 626, max exposure = VTS - 6 */
                turning_data.sensor_data.lines_per_second = 626 * 240;
                turning_data.sensor_data.exposure_time_max = 620;
        } else {
                turning_data.sensor_data.active_width = 1920;
                turning_data.sensor_data.active_height = 1080;
                turning_data.sensor_data.lines_per_second = 150000;//vts * fps
                turning_data.sensor_data.exposure_time_max = 1244; //from customer, max 8.3ms
        }
        sc233_exposure_max = turning_data.sensor_data.exposure_time_max;
        turning_data.sensor_data.exposure_time_min = 1;
        turning_data.sensor_data.analog_gain_max = 192; //we use again + dig fine gain
        turning_data.sensor_data.digital_gain_max = 0;

        //sensor bit && bayer
        sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
        // sensor exposure_max_bit, maybe not used ?  //FIXME
        sensor_data_bits_fill(&turning_data.sensor_data, 12);

        //some stress test case, we need kernel stream_ctrl.
        turning_data.stream_ctrl.data_length = 1;

        if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc233_stream_on_setting)) {
                memcpy(stream_on, sc233_stream_on_setting, sizeof(sc233_stream_on_setting));
        } else {
                vin_err("Number of registers on stream over 10\n");
                return -RET_ERROR;
        }
        if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc233_stream_off_setting)) {
                memcpy(stream_off, sc233_stream_off_setting, sizeof(sc233_stream_off_setting));
        } else {
                vin_err("Number of registers on stream over 10\n");
                return -RET_ERROR;
        }

        // sync gain lut to kernel driver.
        turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
        if (turning_data.normal.again_lut != NULL) {
                memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
                memcpy(turning_data.normal.again_lut, sc233_gain_lut,
                        sizeof(sc233_gain_lut));
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

/* input value:
 * again, dgain should be lut index
 * gain_num, linear mode: 1; dol2 mode: 2
 */
static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
        printf("test %s, mode = %d gain_num = %d again[0] = %d, dgain[0] = %d\n", __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif

        char again_reg_value_h = 0,again_reg_value_l = 0;
        char dgain_reg_value_h = 0,dgain_reg_value_l = 0;
        int gain_index = 0;

        if (mode == NORMAL_M || mode == SLAVE_M) {
                if (again[0] >= sizeof(sc233_gain_lut)/sizeof(uint32_t))
                        gain_index = sizeof(sc233_gain_lut)/sizeof(uint32_t) - 1;
                else
                        gain_index = again[0];

                dgain_reg_value_h = (sc233_gain_lut[gain_index] >> 24) & 0x000000FF;
                dgain_reg_value_l = (sc233_gain_lut[gain_index] >> 16) & 0x000000FF;
                again_reg_value_h = (sc233_gain_lut[gain_index] >> 8) & 0x000000FF;
                again_reg_value_l = (sc233_gain_lut[gain_index]) & 0x000000FF;

#ifdef AE_DBG
                printf("%s, gain_index: %d gain_lut %08x, dgain: 0x%02x 0x%02x again: 0x%02x 0x%02x\n",
                                __FUNCTION__, gain_index, sc233_gain_lut[gain_index],dgain_reg_value_h, dgain_reg_value_l, again_reg_value_h, again_reg_value_l);
#endif
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, SC233_AGAIN_C, again_reg_value_h);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, SC233_AGAIN_F, again_reg_value_l);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, SC233_DGAIN_C, dgain_reg_value_h);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, SC233_DGAIN_F, dgain_reg_value_l);
        } else	{
                vin_err(" unsupport mode %d\n", mode);
        }

    return 0;
}

/* input value:
 * line: exposure time value
 * line_num: linear mode: 1; dol2 mode: 2
 * */
static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
#ifdef AE_DBG
        printf("line mode %d, --line %d , line_num:%d \n", mode, line[0], line_num);
#endif
        const uint16_t EXP_LINE0 = 0x3e00;
        const uint16_t EXP_LINE1 = 0x3e01;
        const uint16_t EXP_LINE2 = 0x3e02;
        char temp0 = 0, temp1 = 0, temp2 = 0;

        if (mode == NORMAL_M || mode == SLAVE_M) {
                uint32_t sline = line[0];

                if ( sline > sc233_exposure_max) {
                        sline = sc233_exposure_max;
                }
                if ( sline < 1) {
                        sline = 1;
                }

                temp0 = (sline >> 12) & 0x0F;
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
                temp1 = (sline >> 4) & 0xFF;
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
                temp2 = (sline & 0x0F) << 4;
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE2, temp2);
#ifdef AE_DBG
        printf("write sline = %d, 0x3e00 = 0x%x, 0x3e01 = 0x%x, 0x3e02 = 0x%x \n",
                        sline, temp0, temp1, temp2);
#endif

        } else {
                vin_err(" unsupport mode %d\n", mode);
        }

        return 0;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
        vin_info("enable userspace gain control and line control\n");
        *enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
        //*enable = HAL_LINE_CONTROL;
        return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(sc233, CAM_MODULE_FLAG_A16D8);
sensor_module_t sc233 = {
        .module = SENSOR_MNAME(sc233),
#else
sensor_module_t sc233 = {
        .module = "sc233",
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
