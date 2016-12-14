/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _SLIMPORT_H
#define _SLIMPORT_H

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/async.h>
#include <slimport.h>
#include <linux/clk.h>

#include "slimport_tx_drv.h"
#include "slimport_tx_reg.h"

#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/delay.h>

#define SSC_EN


#define AUX_ERR  1
#define AUX_OK   0

#define LOG_TAG "SlimPort Colorado3"
#define __func__  ""

extern enum SP_TX_System_State sp_tx_system_state;
extern enum RX_CBL_TYPE sp_tx_rx_type;

struct msm_hdmi_device_ops{
	int (*hdmi_tx_device_show)(char *buf);
};
extern int msm_hdmi_device_show_register(int (*func)(char*));

extern bool is_slimport_vga(void);
extern bool is_slimport_dp(void);

int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf);
int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value);
void sp_tx_hardware_poweron(void);
void sp_tx_hardware_powerdown(void);
int slimport_read_edid_block(int block, uint8_t *edid_buf);
char *slimport_read_edid_block_ext(void);
unchar sp_get_link_bw(void);
void sp_set_link_bw(unchar link_bw);
#ifdef USING_HPD_FOR_POWER_MANAGEMENT
bool sink_is_connected(void);
#endif
#endif
