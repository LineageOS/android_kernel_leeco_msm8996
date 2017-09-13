/*
  SiI6400 Linux Driver

  Copyright (C) 2012-2013 Silicon Image, Inc.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation version 2.

  This program is distributed "AS-IS" WITHOUT ANY WARRANTY of any
  kind, whether express or implied; INCLUDING without the implied warranty
  of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
  See the GNU General Public License for more details at
  http://www.gnu.org/licenses/gpl-2.0.html.
*/

#ifndef _OSAL_LINUX_KERNEL_H
#define _OSAL_LINUX_KERNEL_H

#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/mempool.h>
#include <linux/device.h>

#include "sii6400.h"

#define MAX_NAME_LEN 32

#define DEBUG_LEVEL_NONE		-1
#define DEBUG_LEVEL_CRITICAL		0
#define DEBUG_LEVEL_WARNINGS		1
#define DEBUG_LEVEL_VERBOSE		2
#define DEBUG_LEVEL_DEFAULT		DEBUG_LEVEL_CRITICAL

struct SiiOsSemaphore {
	char name[MAX_NAME_LEN];
	struct semaphore semaphore;
	uint32_t count;
	uint32_t maxCount;
	spinlock_t lock;
	bool usable;
	int tryuse;
};

struct SiiOsQueue {
	char name[MAX_NAME_LEN];
	struct kfifo queue;
	uint32_t esize;
	spinlock_t lock;
	struct semaphore semaphore;
	wait_queue_head_t wait;
	bool usable;
};

struct SiiOsTimer {
	char name[MAX_NAME_LEN];
	struct timer_list timer;
	unsigned long timeout;
	void (*function)(void *pArg);
	void *data;
	bool rearm;
	bool usable;
};

struct SiiOsTime {
	struct timespec time;
};

struct SiiOsBlockPool {
	mempool_t pool;
};

struct SiiOsBytePool {
	mempool_t pool;
};

/* Macros for printk */
#if 1
/* Use these for now, since the dev_dbg() macro is not working on ICS Android */
#define dbg(format, arg...) \
	do { \
		if ((DEBUG_LEVEL_NONE != debug_level) && \
		    (DEBUG_LEVEL_VERBOSE <= debug_level)) { \
			pr_info("video sii6400: %s: " format, \
				__func__,  ## arg); \
		} \
	} while (0)

#define err(format, arg...) \
	do { \
		if ((DEBUG_LEVEL_NONE != debug_level) && \
		    (DEBUG_LEVEL_CRITICAL <= debug_level)) { \
			pr_err("video sii6400: %s: " format, \
				__func__,  ## arg); \
		} \
	} while (0)

#define warn(format, arg...) \
	do { \
		if ((DEBUG_LEVEL_NONE != debug_level) && \
		    (DEBUG_LEVEL_WARNINGS <= debug_level)) { \
			pr_warn("video sii6400: %s: " format, \
				__func__,  ## arg); \
		} \
	} while (0)

#define info(format, arg...) \
	do { \
		if ((DEBUG_LEVEL_NONE != debug_level) && \
		    (DEBUG_LEVEL_CRITICAL <= debug_level)) { \
			pr_info("video sii6400: " format,  ## arg); \
		} \
	} while (0)
#else
#define dbg(format, arg...) \
	do { \
		if ((DEBUG_LEVEL_NONE != debug_level) && \
		    (DEBUG_LEVEL_VERBOSE <= debug_level)) { \
			dev_dbg(sii6400_device, "%s: " format, \
				__func__,  ## arg); \
		} \
	} while (0)

#define err(format, arg...) \
	do { \
		if ((DEBUG_LEVEL_NONE != debug_level) && \
		    (DEBUG_LEVEL_CRITICAL <= debug_level)) { \
			dev_err(sii6400_device, "%s: " format, \
				__func__,  ## arg); \
		} \
	} while (0)

#define warn(format, arg...) \
	do { \
		if ((DEBUG_LEVEL_NONE != debug_level) && \
		    (DEBUG_LEVEL_WARNINGS <= debug_level)) { \
			dev_warn(sii6400_device, "%s: " format, \
				__func__,  ## arg); \
		} \
	} while (0)

#define info(format, arg...) \
	do { \
		if ((DEBUG_LEVEL_NONE != debug_level) && \
		    (DEBUG_LEVEL_CRITICAL <= debug_level)) { \
			dev_info(sii6400_device, format,  ## arg); \
		} \
	} while (0)
#endif

int write_to_nv_storage(const void *pData, uint16_t size, uint32_t offset);
int read_from_nv_storage(void *pData, uint16_t size, uint32_t offset);

#endif /* !_OSAL_LINUX_KERNEL_H */

