/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MSM_EEPROM_H
#define MSM_EEPROM_H

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_camera_i2c.h"
#include "msm_camera_spi.h"
#include "msm_camera_io_util.h"
#include "msm_camera_dt_util.h"

struct msm_eeprom_ctrl_t;

#define DEFINE_MSM_MUTEX(mutexname) \
	static struct mutex mutexname = __MUTEX_INITIALIZER(mutexname)

#define PROPERTY_MAXSIZE 32

#define OTP_CAMERA_BUFF_SIZE 0x1800

struct msm_eeprom_ctrl_t {
	struct platform_device *pdev;
	struct mutex *eeprom_mutex;

	struct v4l2_subdev sdev;
	struct v4l2_subdev_ops *eeprom_v4l2_subdev_ops;
	enum msm_camera_device_type_t eeprom_device_type;
	struct msm_sd_subdev msm_sd;
	enum cci_i2c_master_t cci_master;
	enum i2c_freq_mode_t i2c_freq_mode;

	struct msm_camera_i2c_client i2c_client;
	struct msm_eeprom_memory_block_t cal_data;
	uint8_t is_supported;
	struct msm_eeprom_board_info *eboard_info;
	uint32_t subdev_id;
};

typedef int (*msm_otp_func)(uint8_t*, uint32_t, uint16_t);

struct msm_otp_data {
	uint8_t data_type;
	msm_otp_func otp_data_func;
};

struct msm_otp_type {
	uint32_t otp_camera_num;
	uint8_t module_id;
	uint8_t vcm_id[4];
	uint8_t eeprom_name[19];
	uint8_t otp_camera_buf[OTP_CAMERA_BUFF_SIZE];
	struct msm_otp_data *otp_data_tbl;
} msm_front_otp_type_str, msm_rear_otp_type_str;

static int msm_get_otp_front_camera_id(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_front_camera_awb(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_front_camera_date(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_front_camera_module_id(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_rear_camera_id(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_rear_camera_awb(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_rear_camera_af(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_rear_camera_ois(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_rear_camera_date(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_rear_camera_claf(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_rear_camera_pdaf(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_rear_camera_module_id(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_rear_camera_vcm_id(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_front_camera_all(uint8_t*, uint32_t, uint16_t);
static int msm_get_otp_rear_camera_all(uint8_t*, uint32_t, uint16_t);

#endif
