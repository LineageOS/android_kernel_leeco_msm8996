/* Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "ssrmonitor: " fmt 

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/diagchar.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/ratelimit.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>

#include "ssr_monitor.h"

MODULE_DESCRIPTION("SSR Monitor Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");


#define SSR_CRASH_BUF_SIZE (100)
static char ssr_monitor_crash_reason[SSR_CRASH_BUF_SIZE];


static int ssr_monitor_open(struct inode *inode, struct file *file)
{
	pr_info("ssr_monitor_open.\n");
	return 0;
}

static long ssr_monitor_ioctl(struct file * filp, unsigned int iocmd, unsigned long ioarg)
{
	return 0;
}

static int ssr_monitor_release(struct inode *inode, struct file *file)
{
	pr_info("ssr_monitor_release.\n");
	return 0;
}

static ssize_t ssr_monitor_read(struct file *file, char __user *buf, size_t count,loff_t *ppos)
{
	int ret;
	if(!buf)
		return -1;

	if(count<sizeof(ssr_monitor_crash_reason)){
		pr_err("ssr_monitor_read: user buf is too small\n");
		return -1;
	}

	ret = simple_read_from_buffer(buf, count, ppos, (void *)ssr_monitor_crash_reason, strlen(ssr_monitor_crash_reason));

	pr_info("crash reason:%s.\n",ssr_monitor_crash_reason);
	return ret;
}

static const struct file_operations ssrcharfops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ssr_monitor_ioctl,
	.open = ssr_monitor_open,
	.read = ssr_monitor_read,
	.release = ssr_monitor_release
};

void ssr_monitor_store_crashreason(char* reason)
{
	if (!reason)
		return;
	memset(ssr_monitor_crash_reason,0,SSR_CRASH_BUF_SIZE);
	snprintf(ssr_monitor_crash_reason, SSR_CRASH_BUF_SIZE, "%s\n", reason);
}

static int __init ssr_monitor_init(void)
{
	proc_create("ssr_monitor_dev", S_IRUGO, NULL, &ssrcharfops);

	return 0;
}

static void ssr_monitor_exit(void)
{
	pr_info("ssr_monitor_exit..\n");
}

EXPORT_SYMBOL(ssr_monitor_store_crashreason);

module_init(ssr_monitor_init);
module_exit(ssr_monitor_exit);
