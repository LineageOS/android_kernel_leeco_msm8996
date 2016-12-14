/*
 * tusb320.h - Driver for TUSB320 USB Type-C connector chip
 *
 * Copyright (C) 2015 HUAWEI, Inc.
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 * Author: HUAWEI, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _TYPEC_TUSB320_H_
#define _TYPEC_TUSB320_H_
#include <linux/bitops.h>
enum typec_current_mode {
	TYPEC_CURRENT_MODE_DEFAULT=0,
	TYPEC_CURRENT_MODE_MID,
	TYPEC_CURRENT_MODE_HIGH,
	TYPEC_CURRENT_MODE_UNSPPORTED,
};

enum typec_attached_state {
	TYPEC_ATTACHED_AS_DFP=0,
	TYPEC_ATTACHED_AS_UFP,
	TYPEC_ATTACHED_TO_ACCESSORY,
	TYPEC_NOT_ATTACHED
};

enum typec_accessory_state {
	TYPEC_NO_ACCESSORY_ATTACHED=0,
	TYPEC_AUDIO_ACCESSORY=4,
	TYPEC_AUDIO_CHARGED_THRU_ACCESSORY,
	TYPEC_DEBUG_ACCESSORY_WITH_DFP,
	TYPEC_DEBUG_ACCESSORY_WITH_UFP
};

enum typec_port_mode {
	TYPEC_DFP_MODE=0,
	TYPEC_UFP_MODE,
	TYPEC_DRP_MODE,
	TYPEC_MODE_ACCORDING_TO_PROT,
};

enum typec_sink_detect {
	TYPEC_SINK_REMOVED=0,
	TYPEC_SINK_DETECTED,
};
/*
struct tusb320_device_info {
	int gpio_enb;
	int gpio_intb;
	int irq_intb;
	int irq_active;
	struct i2c_client *client;
	struct work_struct g_intb_work;
	struct delayed_work g_wdog_work;
	struct device *dev;
	struct dual_role_phy_instance *dual_role;
	struct dual_role_phy_desc *desc;
	bool trysnk_attempt;
	int reverse_state;
	bool sink_attached;
	bool clean_failded;
	bool clean_retry_count;
	struct completion reverse_completion;
	struct mutex mutex;
};
*/
struct typec_device_ops {
	enum typec_current_mode (*current_detect)(void);
	enum typec_attached_state (*attached_state_detect)(void);
	enum typec_current_mode (*current_advertise_get)(void);
	int (*current_advertise_set)(enum typec_current_mode current_mode);
	enum typec_port_mode (*port_mode_get)(void);
	int (*port_mode_set)(enum typec_port_mode port_mode);
	ssize_t (*dump_regs)(char *buf);
};
#define TUSB320_DEBUG
#define REVERSE_ATTEMPT 1
#define REVERSE_COMPLETE 2
#define DISABLE_SET 0
#define DISABLE_CLEAR 1
#define REGISTER_NUM    12
#define TUSB320_RESET_DURATION_MS 25
/* Register address */
#define TUSB320_REG_DEVICE_ID				0x00
#define TUSB320_REG_CURRENT_MODE			0x08
#define TUSB320_REG_ATTACH_STATUS			0x09
#define TUSB320_REG_MODE_SET				0x0a
#define TUSB320_REG_DISABLE				    0x45
/* Register REG_CURRENT_MODE 08 */
#define TUSB320_REG_CUR_MODE_ADVERTISE_MASK		(BIT(7) | BIT(6))
#define TUSB320_REG_CUR_MODE_ADVERTISE_DEFAULT	0x00
#define TUSB320_REG_CUR_MODE_ADVERTISE_MID		BIT(6)
#define TUSB320_REG_CUR_MODE_ADVERTISE_HIGH		BIT(7)
#define TUSB320_REG_CUR_MODE_DETECT_MASK		(BIT(5) | BIT(4))
#define TUSB320_REG_CUR_MODE_DETECT_DEFAULT		0x00
#define TUSB320_REG_CUR_MODE_DETECT_MID			BIT(4)
#define TUSB320_REG_CUR_MODE_DETECT_HIGH		(BIT(5) | BIT(4))
#define TUSB320_REG_ACCESSORY_STATUS			(BIT(3) | BIT(2) | BIT(1))

/* Register REG_ATTACH_STATUS 09 */
#define TUSB320_REG_STATUS_MODE					(BIT(7) | BIT(6))
#define TUSB320_REG_STATUS_NOT_ATTACHED			0x00
#define TUSB320_REG_STATUS_AS_DFP				BIT(6)
#define TUSB320_REG_STATUS_AS_UFP				BIT(7)
#define TUSB320_REG_STATUS_TO_ACCESSORY			(BIT(7) | BIT(6))
#define TUSB320_REG_STATUS_CC					BIT(5)
#define TUSB320_REG_STATUS_INT					BIT(4)
#define TUSB320_REG_STATUS_DRP_DUTY_CYCLE		(BIT(2) | BIT(1))
#define TUSB320_REG_STATUS_DRP_DUTY_CYCLE_30	0x00
#define TUSB320_REG_STATUS_DRP_DUTY_CYCLE_40	BIT(1)
#define TUSB320_REG_STATUS_DRP_DUTY_CYCLE_50	BIT(2)
#define TUSB320_REG_STATUS_DRP_DUTY_CYCLE_60	(BIT(2) | BIT(1))
/* Register REG_MODE_SET 0a */
#define TUSB320_REG_SET_MODE					(BIT(5) | BIT(4))
#define TUSB320_REG_SET_BY_PORT					0x00
#define TUSB320_REG_SET_UFP						BIT(4)
#define TUSB320_REG_SET_DFP						BIT(5)
#define TUSB320_REG_SET_DRP						(BIT(5) | BIT(4))
#define TUSB320_REG_SET_SOFT_RESET				BIT(3)
#define TUSB320_REG_SET_DISABLE_RD_RP			BIT(2)

#endif /*_TYPEC_TUSB320_H_*/
