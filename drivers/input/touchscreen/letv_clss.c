/*
 * LETV Class Core
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005-2007 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/letvs.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>

static struct class *letvs_class;
/**
 * letv_classdev_register - register a new object of letv_classdev class.
 * @parent: The device to register.
 * @letv_cdev: the letv_classdev structure for this device.
 */
int letv_classdev_register(struct device *parent, struct letv_classdev *letv_cdev)
{
	letv_cdev->dev = device_create_with_groups(letvs_class, parent, 0,
					letv_cdev, letv_cdev->groups,
					"%s", letv_cdev->name);
	if (IS_ERR(letv_cdev->dev))
		return PTR_ERR(letv_cdev->dev);
	dev_dbg(parent, "Registered letv device: %s\n",
			letv_cdev->name);

	return 0;
}
EXPORT_SYMBOL_GPL(letv_classdev_register);

/**
 * letv_classdev_unregister - unregisters a object of letv_properties class.
 * @letv_cdev: the letv device to unregister
 *
 * Unregisters a previously registered via letv_classdev_register object.
 */
void letv_classdev_unregister(struct letv_classdev *letv_cdev)
{

	device_unregister(letv_cdev->dev);
}
EXPORT_SYMBOL_GPL(letv_classdev_unregister);

static int __init letvs_init(void)
{
	letvs_class = class_create(THIS_MODULE, "letvs");
	if (IS_ERR(letvs_class))
		return PTR_ERR(letvs_class);
	//letvs_class->dev_groups = letv_groups;
	return 0;
}

static void __exit letvs_exit(void)
{
	class_destroy(letvs_class);
}

subsys_initcall(letvs_init);
module_exit(letvs_exit);

MODULE_AUTHOR("John Lenz, Richard Purdie");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LETV Class Interface");

