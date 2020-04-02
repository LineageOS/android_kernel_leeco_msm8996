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

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

#include "atmel_tk_key_letv.h"

enum {
	TS_KEY = 0,
	TS_KEY_2,
	NUM_KEY_TYPE
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct ts_platform_data {
	unsigned long irqflags;
	const u8 *num_keys;  //len is NUM_KEY_TYPE
	const unsigned int (*keymap)[MAX_KEYS_SUPPORTED_IN_DRIVER];

	const char *fw_name;
	const char *fw_version;

	u8 reportid_min;
	u8 reportid_max;

	int gpio_reset;
};

#endif /* __LINUX_ATMEL_MXT_TS_H */
