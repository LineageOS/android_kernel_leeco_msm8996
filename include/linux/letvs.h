/*
 * Driver model for letvs and letv triggers
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LINUX_LETVS_H_INCLUDED
#define __LINUX_LETVS_H_INCLUDED

#include <linux/list.h>
#include <linux/rwsem.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

struct device;

struct letv_classdev {
	const char		*name;
	struct device		*dev;
	const struct attribute_group	**groups;
};

extern int letv_classdev_register(struct device *parent,
				 struct letv_classdev *letv_cdev);
extern void letv_classdev_unregister(struct letv_classdev *letv_cdev);

#endif		/* __LINUX_LETVS_H_INCLUDED */

