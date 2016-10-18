/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#ifndef __RESTART_REASON_H__
#define __RESTART_REASON_H__

#define PANIC_LOW_PRIO (0x20)
#define PANIC_HIGH_PRIO (0x40)

#define TRIG_INIT_STATE (0x7F)

enum TRIG_REASON_VAL {
	CMD_REBOOT = 1,
	CMD_POWEROFF,
	SUB_SYSTEM_RESET,
	SYSRQ_CRASH,
	OVER_TEMP,
	KERNEL_BUG_MACRO,
	SLUB_BUG,
	KERN_MEM_FAULT,
	LONG_PWR_KEY,
	BAD_MODE,
	WDOG_BARK,
	GPU_HANG,
};

#define TRIG_REASON_MASK (0x7f)

/* trigger with low prority, it maybe overide by high prority */
#define TRIG_SLUB_BUG (SLUB_BUG | PANIC_LOW_PRIO)
#define TRIG_GPU_HANG (GPU_HANG | PANIC_LOW_PRIO)
#define TRIG_SUB_SYSTEM_RESET (SUB_SYSTEM_RESET | PANIC_LOW_PRIO)

/* trigger with high priority */
#define TRIG_CMD_REBOOT (CMD_REBOOT | PANIC_HIGH_PRIO)
#define TRIG_CMD_POWEROFF (CMD_POWEROFF | PANIC_HIGH_PRIO)
#define TRIG_SYSRQ_CRASH (SYSRQ_CRASH | PANIC_HIGH_PRIO)
#define TRIG_OVER_TEMPERATURE (OVER_TEMP | PANIC_HIGH_PRIO)
#define TRIG_KERNEL_BUG_MACRO (KERNEL_BUG_MACRO | PANIC_HIGH_PRIO)
#define TRIG_KERN_MEM_FAULT (KERN_MEM_FAULT | PANIC_HIGH_PRIO)
#define TRIG_LONG_PRESS_PWR_KEY (LONG_PWR_KEY | PANIC_HIGH_PRIO)
#define TRIG_BAD_MODE (LONG_PWR_KEY | PANIC_HIGH_PRIO)
#define TRIG_WDOG_BARK (WDOG_BARK | PANIC_HIGH_PRIO)

#define TRIG_MAX (0x7f)
#define TRIG_S3_RESET (TRIG_MAX)

#define KERNEL_ALIVE (0x80)

extern void set_panic_trig_rsn(u8 val);
void panic_reason_hook(void *p);
#endif
