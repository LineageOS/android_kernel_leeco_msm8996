/*
*
* Simple driver for LETV LM3643 LED Flash driver chip
* Copyright (C) 2015 LETV
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/

#ifndef __LINUX_LM3643_H
#define __LINUX_LM3643_H

#define LM3643_NAME "leds-lm3643"

int lm3643_flash_led_init(
		struct msm_flash_ctrl_t     *flash_ctrl,
		struct msm_flash_cfg_data_t *flash_data);
int lm3643_flash_led_release(
		struct msm_flash_ctrl_t *flash_ctrl);
int lm3643_flash_led_off(
		struct msm_flash_ctrl_t *flash_ctrl,
		struct msm_flash_cfg_data_t *flash_data);
int lm3643_flash_led_low(
		struct msm_flash_ctrl_t     *flash_ctrl,
		struct msm_flash_cfg_data_t *flash_data);
int lm3643_flash_led_high(
		struct msm_flash_ctrl_t     *flash_ctrl,
		struct msm_flash_cfg_data_t *flash_data);

#endif /* __LINUX_LM3643_H */
