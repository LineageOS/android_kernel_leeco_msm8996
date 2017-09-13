/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_LETV_H
#define __LINUX_ATMEL_MXT_TS_LETV_H

#include <linux/types.h>

/* To store the config info in DTS file */
struct mxt_config_info{
	u8 type;
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 config_year;
	u8 config_month;
	u8 config_date;
	u8 *config;
	int config_length;
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned long irqflags;
	u8 t19_num_keys;
	const unsigned int *t19_keymap;
	int t15_num_keys;
	int num_x_lines;
	int num_y_lines;
	int threshold_max;
	int threshold_min;
	int rawdata_shift;
	int threshold_tkey_max;
	int threshold_tkey_min;
	int rawdata_tkey_shift;
	const unsigned int *t15_keymap;
	unsigned long gpio_reset;
	unsigned long gpio_vdd;
	const char *cfg_name;
	const char *fw_version;
	const char *input_name;
	struct mxt_config_info info;
	bool report_pressure_hideep;
};

#endif /* __LINUX_ATMEL_MXT_TS_LETV_H */
