/*
 * drivers/media/platform/msm/camera_v2/sensor/flash/lm3643.c
 * General device driver for LETV LM3643, FLASH LED Driver
 *
 * Copyright (C) 2015 LETV
 *
 * Contact: Gao,Feng <gaofeng1@letv.com>
 *          Xiao,Peng <xiaopeng@letv.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/module.h>
#include <linux/debugfs.h>
#include "msm_camera_io_util.h"
#include "../msm_sensor.h"
#include "msm_flash.h"
#include "../cci/msm_cci.h"

#undef CDBG
#define CDBG(fmt, args...) pr_err(fmt, ##args)

/* registers definitions */
#define REG_ENABLE 0x01
#define REG_FLASH_LED0_BR 0x03
#define REG_FLASH_LED1_BR 0x04
#define REG_TORCH_LED0_BR 0x05
#define REG_TORCH_LED1_BR 0x06
#define REG_FLASH_TOUT 0x08
#define REG_FLAG0 0x0a
#define REG_FLAG1 0x0b

int lm3643_flash_led_init(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int rc = 0, value = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	if (MSM_CAMERA_POWER_INIT == flash_ctrl->power_state)
		return 0;

	power_info = &flash_ctrl->power_info;
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL)
		pr_err("%s:%d mux install\n", __func__, __LINE__);

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	value = gpio_get_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN]);
	if (GPIO_OUT_LOW == value)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_HIGH);

	flash_ctrl->power_state = MSM_CAMERA_POWER_INIT;

	return rc;
}

int lm3643_flash_led_release(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	if (MSM_CAMERA_POWER_RELEASE == flash_ctrl->power_state)
		return 0;

	power_info = &flash_ctrl->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);

	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	flash_ctrl->power_state = MSM_CAMERA_POWER_RELEASE;

	return 0;
}

int lm3643_flash_led_off(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int rc = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	power_info = &flash_ctrl->power_info;
	flash_ctrl->flash_i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	rc = msm_camera_cci_i2c_write(
		&flash_ctrl->flash_i2c_client, REG_ENABLE, 0x00,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s:%d camera_flash_off failed rc = %d",
			__func__, __LINE__, rc);
		return rc;
	}

	return rc;
}

int lm3643_flash_led_low(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int rc = 0;
	int32_t flash_current_2200K = 0, flash_current_5500K = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	power_info = &flash_ctrl->power_info;
	flash_ctrl->flash_i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	flash_current_2200K = flash_data->flash_current[0];
	flash_current_5500K = flash_data->flash_current[1];
	rc = msm_camera_cci_i2c_write(&flash_ctrl->flash_i2c_client ,
		REG_TORCH_LED1_BR, (uint16_t)flash_current_2200K,
		MSM_CAMERA_I2C_BYTE_DATA);
	rc = msm_camera_cci_i2c_write(&flash_ctrl->flash_i2c_client,
		REG_TORCH_LED0_BR, (uint16_t)flash_current_5500K,
		MSM_CAMERA_I2C_BYTE_DATA);
	rc = msm_camera_cci_i2c_write(&flash_ctrl->flash_i2c_client,
		REG_ENABLE, 0x8B, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s:%d camera_flash_init failed rc = %d",
			__func__, __LINE__, rc);
		return rc;
	}

	return rc;
}

int lm3643_flash_led_high(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int rc = 0;
	int32_t flash_current_2200K = 0, flash_current_5500K = 0;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	power_info = &flash_ctrl->power_info;
	flash_ctrl->flash_i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	flash_current_2200K = flash_data->flash_current[0];
	flash_current_5500K = flash_data->flash_current[1];
	rc = msm_camera_cci_i2c_write(&flash_ctrl->flash_i2c_client,
		REG_FLASH_LED1_BR, (uint16_t)flash_current_2200K,
		MSM_CAMERA_I2C_BYTE_DATA);
	rc = msm_camera_cci_i2c_write(&flash_ctrl->flash_i2c_client,
		REG_FLASH_LED0_BR, (uint16_t)flash_current_5500K,
		MSM_CAMERA_I2C_BYTE_DATA);
	rc = msm_camera_cci_i2c_write(&flash_ctrl->flash_i2c_client,
		REG_FLASH_TOUT, 0x1F, MSM_CAMERA_I2C_BYTE_DATA);
	rc = msm_camera_cci_i2c_write(&flash_ctrl->flash_i2c_client,
		REG_ENABLE, 0x8F, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s:%d msm_camera_cci_i2c_write failed rc = %d",
			__func__, __LINE__, rc);
		return rc;
	}

	return rc;
}
