/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)             "[sc835hai]:" fmt

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
#include "inc/sc835hai_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"


int sc835hai_linear_data_init(sensor_info_t *sensor_info);
static int32_t sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps);
static int32_t sensor_update_fps_notify_driver(sensor_info_t *sensor_info);

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

        switch(sensor_info->sensor_mode) {
                case NORMAL_M:  // 1: normal
                        vin_info("sc835hai in normal linear mode\n");
                        vin_info("bus_num = %d, sensor_addr = 0x%0x\n", sensor_info->bus_num, sensor_info->sensor_addr);
				if(sensor_info->config_index == 0){ // 0: 2lane (default is 0)

                        setting_size = sizeof(sc835hai_linear_30fps_init_setting) / sizeof(uint32_t) / 2;
                        ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
                                                    setting_size, sc835hai_linear_30fps_init_setting);
				}else{// 1: 4lane
					setting_size = sizeof(sc835hai_linear_30fps_4lane_init_setting) / sizeof(uint32_t) / 2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, sc835hai_linear_30fps_4lane_init_setting);
				}
                        if (ret < 0) {
                                vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
                                return -HB_CAM_I2C_WRITE_FAIL;
                        }
                        ret = sc835hai_linear_data_init(sensor_info);
				if (ret < 0) {
					vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_INIT_FAIL;
				}

                        if (ret < 0) {
                                vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
                                return -HB_CAM_INIT_FAIL;
                        }
                        break;
                case DOL2_M:
                default:
                        vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
                        ret = -HB_CAM_INIT_FAIL;
                        break;
        }
        vin_info("sc835hai config success under %d mode\n", sensor_info->sensor_mode);

        // Default 30fps
        // Switch frame rate based on application configuration
        //switch fps should be setted by user program, API: <hbn_camera_change_fps> !


        return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        int setting_size = 0;

        switch(sensor_info->sensor_mode) {
                case NORMAL_M:
                case SLAVE_M:
                        setting_size = sizeof(sc835hai_stream_on_setting)/sizeof(uint32_t)/2;
                        vin_info("%s start normal / slave linear mode\n", sensor_info->sensor_name);
                        ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
                                        setting_size, sc835hai_stream_on_setting);
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
                sizeof(sc835hai_stream_off_setting) / sizeof(uint32_t) / 2;
        vin_info("%s sensor stop\n", sensor_info->sensor_name);
        ret = vin_write_array(sensor_info->bus_num,
                        sensor_info->sensor_addr, 2,
                        setting_size, sc835hai_stream_off_setting);
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

int sc835hai_linear_data_init(sensor_info_t *sensor_info)
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
        turning_data.mode = sensor_info->sensor_mode;

        turning_data.sensor_addr = sensor_info->sensor_addr;
        strncpy(turning_data.sensor_name, sensor_info->sensor_name,
                sizeof(turning_data.sensor_name));

        turning_data.sensor_data.active_width = 3840;
        turning_data.sensor_data.active_height = 2160;

        //read vts = frame_length = HMAX, default value is 0x0465 = 1125
        VTS_HIGH = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x320e);
        VTS_HIGH = VTS_HIGH & 0x7f; //0x320e[0:6]
        VTS_LOW = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x320f);
        VTS_VALUE = (VTS_HIGH << 8 | VTS_LOW);
#ifdef AE_DBG
        printf("%s read VTS_HIGH = 0x%x, VTS_LOW = 0x%x, VTS = 0x%x \n",
                __FUNCTION__, VTS_HIGH, VTS_LOW, VTS_VALUE);
#endif
        turning_data.sensor_data.lines_per_second = 33750;//vts * fps, should be fixed = 1125 * 30
        //lines_per_second/fps = vts
        // from customer, max 10ms
        // 1000ms -lines_per_second - 33750
        // 10ms - 337, 33ms - 1125
        turning_data.sensor_data.exposure_time_max = 1125; //from customer, max 33ms
        turning_data.sensor_data.exposure_time_min = 1;
         turning_data.sensor_data.exposure_time_long_max = 2 * VTS_VALUE - 8;  //2*frame_length - 8  //linear not use
        turning_data.sensor_data.analog_gain_max = 241; //we use again + dig fine gain
        turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.analog_gain_init = 64;
	turning_data.sensor_data.digital_gain_init = 0;
	turning_data.sensor_data.exposure_time_init = 377;

        //sensor bit && bayer
        sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
        // sensor exposure_max_bit, maybe not used ?  //FIXME
        sensor_data_bits_fill(&turning_data.sensor_data, 12);

        //some stress test case, we need kernel stream_ctrl.
        turning_data.stream_ctrl.data_length = 1;

        if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc835hai_stream_on_setting)) {
                memcpy(stream_on, sc835hai_stream_on_setting, sizeof(sc835hai_stream_on_setting));
        } else {
                vin_err("Number of registers on stream over 10\n");
                return -RET_ERROR;
        }
        if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc835hai_stream_off_setting)) {
                memcpy(stream_off, sc835hai_stream_off_setting, sizeof(sc835hai_stream_off_setting));
        } else {
                vin_err("Number of registers on stream over 10\n");
                return -RET_ERROR;
        }

        // sync gain lut to kernel driver.
        turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
        if (turning_data.normal.again_lut != NULL) {
                memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
                memcpy(turning_data.normal.again_lut, sc835hai_gain_lut,
                        sizeof(sc835hai_gain_lut));
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
        const uint16_t AGAIN = 0x3e08;
        const uint16_t AFINE_GAIN = 0x3e09;
        const uint16_t DGAIN = 0x3e06;
        const uint16_t DFINE_GAIN = 0x3e07;
        char again_reg_value = 0,a_fine_gain_reg_value = 0;
        char dgain_reg_value = 0, d_fine_gain_reg_value = 0;
        int gain_index = 0;

        if (mode == NORMAL_M || mode == SLAVE_M) {
                if (again[0] >= sizeof(sc835hai_gain_lut)/sizeof(uint32_t))
                        gain_index = sizeof(sc835hai_gain_lut)/sizeof(uint32_t) - 1;
                else
                        gain_index = again[0];

                again_reg_value = (sc835hai_gain_lut[gain_index] >> 24) & 0x000000FF;
                a_fine_gain_reg_value = (sc835hai_gain_lut[gain_index] >> 16) & 0x000000FF;
                dgain_reg_value = (sc835hai_gain_lut[gain_index] >> 8) & 0x000000FF;
                d_fine_gain_reg_value = sc835hai_gain_lut[gain_index] & 0x000000FF;

#ifdef AE_DBG
                printf("%s, gain_index: %d, again: 0x3e08 = 0x%x adfine: 0x3e09 = 0x%x dgain: 0x3e06 = 0x%x dfine gain: 0x3e07 = 0x%x\n",
                                __FUNCTION__, gain_index, again_reg_value, a_fine_gain_reg_value,dgain_reg_value, d_fine_gain_reg_value);
        #endif
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN, again_reg_value);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AFINE_GAIN, a_fine_gain_reg_value);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN, dgain_reg_value);
                vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DFINE_GAIN, d_fine_gain_reg_value);
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
                uint32_t sline = 2 * line[0];
                /*
                        * NOTICE: sensor exposure half line, so sline = 2 * line(from isp)
                        * exposure line max, from customer:
                        * exposure_time_max = 10ms, line = 337, result = 337 * 2 = 674
                        * form spec:
                        * exposure_time_max = 2 * VTS - 8, 10fps, result = 11250 * 2 - 8
                        * so, we should limit sline = 674
                        */
                if ( sline > 2242) {
                        sline = 2242;
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
        // *enable = 0;
        return 0;
}

static int32_t sensor_update_fps_notify_driver(sensor_info_t *sensor_info)
{
        int32_t ret = RET_OK;

        switch(sensor_info->sensor_mode) {
                case (uint32_t)NORMAL_M:
                case (uint32_t)SLAVE_M:
                        ret = sc835hai_linear_data_init(sensor_info);
                        if (ret < 0) {
                                vin_err("update fps sc835hai_linear_data_init fail\n");
                                return ret;
                        }
                        break;
                case (uint32_t)DOL2_M:
                default:
                        vin_err("update fps not support %d mode \n", sensor_info->sensor_mode);
                        break;
        }

        return ret;
}

/* input value:
 * fps: set fps
 *
 * we can use this function to dynamic switch fps in our program.
 * int32_t hbn_camera_change_fps(camera_handle_t cam_fd, int32_t fps)
 */
static int32_t sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
        int32_t ret = RET_OK;
        const uint16_t SC835HAI_VTS_LOW = 0x320f;
        const uint16_t SC835HAI_VTS_HIGH = 0x320e;
        int32_t vts;

        vin_info("%s %s %dfps \n", __FUNCTION__, sensor_info->sensor_name, fps);

        if (fps < 1 || sensor_info->fps > 30) {
                vin_err("%s %s %dfps not support\n", __FUNCTION__, sensor_info->sensor_name, fps);
                return -RET_ERROR;
        }

        switch (sensor_info->sensor_mode) {
                case NORMAL_M:
                case SLAVE_M:
                        //NOTICE:
                        //vts = frame_length = lines_per_second / fps
                        vts = 33750 / fps;
                        break;
                case DOL2_M:
                default:
                        vin_err("%s not support mode %d \n", __FUNCTION__, sensor_info->sensor_mode);
                        return -RET_ERROR;
        }

#ifdef AE_DBG
        printf("%s set fps = %d, vts = 0x%x \n", __FUNCTION__, fps, vts);
#endif
        ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                        SC835HAI_VTS_HIGH, ((vts >> 8) & 0x7f)); //0x320e[0:6] 0x0d for 10fps
        ret |= hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                        SC835HAI_VTS_LOW, (vts & 0xff) - 2); // 减小2行加点裕度 0x2d for 10fps
        if (ret < 0) {
                vin_err("%s %s write vts=0x%x fail \n", __FUNCTION__, sensor_info->sensor_name, vts);
                return -HB_CAM_I2C_WRITE_FAIL;
        }

        sensor_info->fps = fps;
        sensor_update_fps_notify_driver(sensor_info);
        vin_info("%s dynamic switch to %dfps success \n", sensor_info->sensor_name, fps);
        return RET_OK;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(sc835hai, CAM_MODULE_FLAG_A16D8);
sensor_module_t sc835hai = {
        .module = SENSOR_MNAME(sc835hai),
#else
sensor_module_t sc835hai = {
        .module = "sc835hai",
#endif
        .init = sensor_init,
        .start = sensor_start,
        .stop = sensor_stop,
        .deinit = sensor_deinit,
        .power_on = sensor_poweron,
        .power_off = sensor_poweroff,
        .aexp_gain_control = sensor_aexp_gain_control,
        .aexp_line_control = sensor_aexp_line_control,
        .dynamic_switch_fps = sensor_dynamic_switch_fps,
        .userspace_control = sensor_userspace_control,
};
