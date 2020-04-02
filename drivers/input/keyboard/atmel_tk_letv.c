/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2012 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

/****************************************************************
	Pitter Liao add for macro for the global platform
		email:  pitter.liao@atmel.com
		mobile: 13244776877
-----------------------------------------------------------------*/
#define DRIVER_VERSION 0x001
/*----------------------------------------------------------------
0.01
1 initialize ts key
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/kthread.h>

#include "atmel_tk_letv.h"
#include "atmel_tk_platform_letv.h"
#if defined(CONFIG_FB_PM)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#include "atomic_op_tk_letv.h"

/* Configuration file */

/* Registers */
#define TS_MAX_BLOCK_WRITE	32
#define TS_INFO_CHECKSUM_SIZE	3
#define TS_OBJECT_START 0x2

#define TS_REG_SIZE 0x40
#define TS_EEPROM_SIZE (7<<10)

/* MXT_GEN_MESSAGE_T5 object */
#define TS_RPTID_NOMSG		0xff

/* Object types */
#define		REG_GPIO_OUTPUT		0x00
#define		REG_KEY1_THRESHOLD		0X02
#define		REG_KEY2_THRESHOLD		0X03
#define		REG_KEY3_THRESHOLD		0X04
#define		REG_KEY4_THRESHOLD		0X05
#define		REG_SLIDE1_THRESHOLD		0X06
#define		REG_SLIDE2_THRESHOLD		0X07
#define		REG_PALM_REJECTION_MS_H		0X08
#define		REG_PALM_REJECTION_MS_L		0X09
#define		REG_SLIDE_REJECTION_MS_H		0X0A
#define		REG_SLIDE_REJECTION_MS_L		0X0B
#define		REG_GPIO_OUTPUT_DLY_MS_H		0X0C
#define		REG_GPIO_OUTPUT_DLY_MS_L		0X0D
#define		REG_PWM_WIDTH_MS_H_L		0X0E
#define		REG_PWM_WIDTH_MS_L		0X0F
#define		REG_INTO_DEEP_SLEEP_DLY_S_H		0X10
#define		REG_INTO_DEEP_SLEEP_DLY_S_L		0X11
#define		REG_PWM_OUTPUT_ENABLE		0X12
#define		REG_KEY_INPUT_ENABLE		0X13
#define		REG_PWM_ACTIVE_LEVEL		0X14
#define		REG_GPIO_ACTIVE_LEVEL		0X15
#define		REG_INPUT_ACTIVE_LEVEL		0X16
#define		REG_PWM_DUTY		0X17
#define		REG_PWM_GAP_MS		0X18
#define		REG_AWAKE_PERIOD		0X19
#define		REG_SLEEP_PERIOD		0X1A
#define		REG_DISABLE_HK_PERIOD		0X1B
#define		REG_FW_VERSIONS		0X1C
#define		REG_REPORT_ENABLE		0X1D
#define		REG_DEBUG_ENABLE		0X1E
#define		REG_MEM_SIZE		0X1F
#define		REG_KEY1_REF_H		0X20
#define		REG_KEY1_REF_L		0X21
#define		REG_KEY2_REF_H		0X22
#define		REG_KEY2_REF_L		0X23
#define		REG_KEY3_REF_H		0X24
#define		REG_KEY3_REF_L		0X25
#define		REG_KEY4_REF_H		0X26
#define		REG_KEY4_REF_L		0X27
#define		REG_KEY1_SIG_H		0X28
#define		REG_KEY1_SIG_L		0X29
#define		REG_KEY2_SIG_H		0X2A
#define		REG_KEY2_SIG_L		0X2B
#define		REG_KEY3_SIG_H		0X2C
#define		REG_KEY3_SIG_L		0X2D
#define		REG_KEY4_SIG_H		0X2E
#define		REG_KEY4_SIG_L		0X2F
#define		REG_KEY1_DLT_H		0X30
#define		REG_KEY1_DLT_L		0X31
#define		REG_KEY2_DLT_H		0X32
#define		REG_KEY2_DLT_L		0X33
#define		REG_KEY3_DLT_H		0X34
#define		REG_KEY3_DLT_L		0X35
#define		REG_KEY4_DLT_H		0X36
#define		REG_KEY4_DLT_L		0X37
#define		REG_SLIDE1_DIFF_H		0X38
#define		REG_SLIDE1_DIFF_L		0X39
#define		REG_SLIDE2_DIFF_H		0X3A
#define		REG_SLIDE2_DIFF_L		0X3B
#define		REG_GPIO_OUTPUT2		0X3C
#define         REG_KEY_STATUS                      0x3E

#define		REG_FW_VERSIONS2		0X40
#define		REG_FW_VERSIONS_EX		0X41
#define		REG_IC_INFO_H	0X42
#define		REG_IC_INFO_L	0X43
#define		REG_CFG_CRC		0X44

#define TS_POWER_CFG_DEEPSLEEP		1
#define TS_POWER_CFG_RUN 0

/* Delay times */
#define TS_BACKUP_TIME		50	/* msec */
#define TS_RESET_TIME		200	/* msec */
#define TS_RESET_TIMEOUT	3000	/* msec */
#define TS_CRC_TIMEOUT		1000	/* msec */
#define TS_FW_RESET_TIME	3000	/* msec */
#define TS_FW_CHG_TIMEOUT	30/*300*/	/* msec */
#define TS_WAKEUP_TIME		25	/* msec */
#define TS_REGULATOR_DELAY	150	/* msec */
#define TS_POWERON_DELAY	150	/* msec */

/* Command to bootloader */
enum {
	TS_ENTER_BOOTLOADER,
	TS_BOOT_RESET,
	TS_ENTER_APP,
	TS_GET_BOOTLOADER_VER,
	TS_BOOT_RW,
	TS_COMMAND_MAX
};

struct ts_command{
	u8 buf[4];
	u8 len;
	unsigned short addr;
	unsigned long flag;
};

struct ts_command ts_command_list[TS_COMMAND_MAX] ={
	//TS_ENTER_BOOTLOADER,
	{{0x0},1},
	//TS_BOOT_RESET,
	{{0xB0,0xAD,0x00},3},
	//TS_ENTER_APP,
	{{0x21,0x80},2},
	//TS_GET_BOOTLOADER_VER,
	{{0x01},1},
	//TS_BOOT_RW,
	{{0x02,0x1},4},
};

#define BOOTLOADER_WRITE_LEN 32
#define BOOT_ADDRESS_FLASH (1 << 5)

/* Touchscreen absolute values */
#define TS_MAX_AREA		0xff

#define TS_PIXELS_PER_MM	20

#define DEBUG_MSG_MAX		200

struct ts_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 object_num;
};

/* Each client has this additional data */
struct ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	struct ts_platform_data *pdata;
	struct ts_info *info;
	void *raw_info_block;
	unsigned int irq;
	atomic_t depth;
	bool in_bootloader;
	u16 mem_size;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	bool debug_v2_enabled;
	bool test_enabled;
	u8 *debug_msg_data;
	u16 debug_msg_count;
	struct bin_attribute debug_msg_attr;
	struct mutex debug_msg_lock;
	u8 max_reportid;
	u32 info_crc;
	unsigned short addr;
	unsigned short bootloader_addr;
#if defined(CONFIG_TS_I2C_DMA)
	void *i2c_dma_va;
	dma_addr_t i2c_dma_pa;
	struct mutex dma_access_mutex;
#endif
	u8 *msg_buf;
	unsigned long key_status;
	bool use_regulator;
	struct regulator *reg_vdd;
	struct regulator *reg_avdd;
	char *fw_name;

	const struct ts_command *cmd_tab;

	/* Cached parameters from object table */
	u8 T5_address;
	u8 T5_msg_size;

	u8 T15_address;
	u8 T15_msg_size;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u8 T37_address;
	u8 T37_msg_size;

	int last_message_count;
	/* for fw update in bootloader */
	struct completion bl_completion;

	/* for reset handling */
	struct completion reset_completion;

	/* Enable reporting of input events */
	bool enable_reporting;

	/* Indicates whether device is in suspend */
	bool suspended;

	struct mutex access_mutex;

#if defined(CONFIG_TS_IRQ_WORKQUEUE)
	struct task_struct *irq_tsk;
	wait_queue_head_t wait;
#endif
#define TS_EVENT_IRQ 1
#define TS_EVENT_EXTERN 2
#define TS_EVENT_IRQ_FLAG 5
	unsigned long busy;

#if defined(CONFIG_FB_PM)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

	struct ts_command dbg_command;
};

#if defined(CONFIG_FB_PM)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ts_early_suspend(struct early_suspend *es);
static void ts_late_resume(struct early_suspend *es);
#endif

static int ts_bootloader_read(struct i2c_client *client,
				u8 idx, u16 reg, void *val, unsigned int count);
static int ts_bootloader_write(struct i2c_client *client,
				u8 idx, u16 reg, void *val, unsigned int count);

inline u8 TO_VAL(void *buf, int reg)
{
	return ((u8 *)buf)[reg & 0xff];
}

static void ts_dump_message(struct ts_data *data, u8 *message)
{
	print_hex_dump(KERN_DEBUG, "TS MSG:", DUMP_PREFIX_NONE, 16, 1,
				message, data->T5_msg_size, false);
}

static void ts_debug_msg_enable(struct ts_data *data)
{
	struct device *dev = &data->client->dev;

	if (data->debug_v2_enabled)
		return;

	mutex_lock(&data->debug_msg_lock);

	data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->debug_msg_data) {
		dev_err(&data->client->dev, "Failed to allocate buffer\n");
		return;
	}

	data->debug_v2_enabled = true;
	mutex_unlock(&data->debug_msg_lock);

	dev_info(dev, "Enabled message output\n");
}

static void ts_debug_msg_disable(struct ts_data *data)
{
	struct device *dev = &data->client->dev;

	if (!data->debug_v2_enabled)
		return;

	dev_info(dev, "disabling message output\n");
	data->debug_v2_enabled = false;

	mutex_lock(&data->debug_msg_lock);
	kfree(data->debug_msg_data);
	data->debug_msg_data = NULL;
	data->debug_msg_count = 0;
	mutex_unlock(&data->debug_msg_lock);
	dev_info(dev, "Disabled message output\n");
}

static void ts_debug_msg_add(struct ts_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	mutex_lock(&data->debug_msg_lock);

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return;
	}

	if (data->debug_msg_count < DEBUG_MSG_MAX) {
		memcpy(data->debug_msg_data + data->debug_msg_count * data->T5_msg_size,
				msg,
				data->T5_msg_size);
		data->debug_msg_count++;
	} else {
		dev_dbg(dev, "Discarding %u messages\n", data->debug_msg_count);
		data->debug_msg_count = 0;
	}

	mutex_unlock(&data->debug_msg_lock);

	sysfs_notify(&data->client->dev.kobj, NULL, "debug_notify");
}

static ssize_t ts_debug_msg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	return -EIO;
}

static ssize_t ts_debug_msg_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct ts_data *data = dev_get_drvdata(dev);
	int count;
	size_t bytes_read;

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return 0;
	}

	count = bytes / data->T5_msg_size;

	if (count > DEBUG_MSG_MAX)
		count = DEBUG_MSG_MAX;

	mutex_lock(&data->debug_msg_lock);

	if (count > data->debug_msg_count)
		count = data->debug_msg_count;

	bytes_read = count * data->T5_msg_size;

	memcpy(buf, data->debug_msg_data, bytes_read);
	data->debug_msg_count = 0;

	mutex_unlock(&data->debug_msg_lock);

	return bytes_read;
}

static int ts_debug_msg_init(struct ts_data *data)
{
	sysfs_bin_attr_init(&data->debug_msg_attr);
	data->debug_msg_attr.attr.name = "debug_msg";
	data->debug_msg_attr.attr.mode = 0666;
	data->debug_msg_attr.read = ts_debug_msg_read;
	data->debug_msg_attr.write = ts_debug_msg_write;
	data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

	if (sysfs_create_bin_file(&data->client->dev.kobj,
				  &data->debug_msg_attr) < 0) {
		dev_err(&data->client->dev, "Failed to create %s\n",
			data->debug_msg_attr.attr.name);
		return -EINVAL;
	}

	return 0;
}

static void ts_debug_msg_remove(struct ts_data *data)
{
	if (data->debug_msg_attr.attr.name)
		sysfs_remove_bin_file(&data->client->dev.kobj,
					  &data->debug_msg_attr);
}

static int ts_wait_for_completion(struct ts_data *data,
			struct completion *comp, unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		dev_err(dev, "Wait for completion interrupted.\n");
		return -EINTR;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int ts_lookup_bootloader_address(struct ts_data *data, bool retry)
{
	u8 appmode = data->addr & 0x7F;
	u8 bootloader;

	switch (appmode) {
	case 0x81:
	default:
		bootloader = 0x29;
	}

	data->bootloader_addr = bootloader;
#if defined(CONFIG_TS_I2C_DMA)
	data->bootloader_addr |= I2C_RS_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG;
#endif

	dev_info(&data->client->dev,
			"Appmode i2c address 0x%02x, bootloader 0x%02x\n",appmode,bootloader);

	return 0;
}

static u8 ts_get_bootloader_version(struct ts_data *data, u8 *val)
{
	int ret;

	ret = ts_bootloader_read(data->client,TS_GET_BOOTLOADER_VER, 0, val, 4);
	if (ret)
		return ret;

	return 0;
}

static int ts_probe_bootloader(struct ts_data *data, bool retry)
{
	u8 val[4];
	int ret;

	ret = ts_lookup_bootloader_address(data, retry);
	if (ret)
		return ret;

	ret = ts_get_bootloader_version(data, val);
	if (ret)
		return ret;

	return 0;
}

static int ts_check_bootloader(struct ts_data *data, u16 *out_len)
{
	struct device *dev = &data->client->dev;
	u8 val[5];
	u16 len;
	int ret;

	ret = ts_get_bootloader_version(data, val);
	if (ret) {
		dev_err(dev, "%s: read bootloader version failed(%d)\n",
			__func__, ret);
		return ret;
	}

	if (out_len) {
		val[4] = '\0';
		if (!strncmp(val,"PA.3",4))
			len = (7 << 10);
		else if (!strncmp(val,"88.3",4))
			len = (7 << 10);
		else if (!strncmp(val,"m8.3",4))
			len = (7 << 10);
		else
			len = (7 << 10);

		*out_len = len;
	}

	return 0;
}

static int ts_send_bootloader_cmd(struct i2c_client *client, u8 idx)
{
	int ret;

	ret = ts_bootloader_write(client, idx, 0, NULL ,0);
	if (ret)
		return ret;

	return 0;
}

#if defined(CONFIG_TS_I2C_DMA)

static int __ts_read(struct i2c_client *client,
				void *val, u16 len)
{
	struct ts_data *data = i2c_get_clientdata(client);

	u16 transfer_len,left_len;
	int retval;

	left_len = len;
	while(left_len) {
		transfer_len = left_len > 255 ? 255 : left_len;
		retval = i2c_master_recv(client, (char *)data->i2c_dma_pa + PAGE_SIZE, transfer_len);
		if (retval != transfer_len) {
			dev_err(&client->dev, "%s: i2c read failed (%d)\n",
				__func__, retval);
			break;
		}
		memcpy(val, data->i2c_dma_va + PAGE_SIZE, transfer_len);
		left_len -= transfer_len;
		val += transfer_len;
	}

	return len - left_len;
}

static int __ts_write(struct i2c_client *client,
				const void *val, u16 len)
{
	struct ts_data *data = i2c_get_clientdata(client);

	u16 transfer_len,left_len;
	int retval;

	left_len = len;
	while(left_len) {
		transfer_len = left_len > 255 ? 255 : left_len;
		memcpy(data->i2c_dma_va, val, transfer_len);
		retval = i2c_master_send(client, (char *)data->i2c_dma_pa, transfer_len);
		if (retval != transfer_len) {
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, retval);
			break;
		}

		left_len -= transfer_len;
		val += transfer_len;
	}
	return len - left_len;
}

static int __ts_raw_read(struct i2c_client *client, u8 *val, unsigned int count)
{
	struct ts_data *data = i2c_get_clientdata(client);
	unsigned short addr;
	int retry = 3;
	int ret;

retry_read:
	mutex_lock(&data->dma_access_mutex);
	client->addr = data->addr;
	ret = __ts_read(client,val,count);
	client->addr = addr;

	mutex_unlock(&data->dma_access_mutex);

	if (ret != count) {
		if (--retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(TS_WAKEUP_TIME);
			goto retry_read;
		}else {
			dev_err(&client->dev, "reads raw data error %d!!\n",ret);
			return -EFAULT;
		}
	}

	return 0;
}

static int __ts_read_reg(struct i2c_client *client,
					const struct ts_command *t_cmd, u16 len, void *val)
{
	struct ts_data *data = i2c_get_clientdata(client);

	int ret = 0;
	int retval = 0;
	int retry = 3;
	u8 * buf;
	u16 extend = 0;

	if (test_flag(I2C_M_RECV_LEN,&t_cmd->flag))
		extend = TS_OBJECT_START;

	if (t_cmd->buf[0] < extend)
		return -EINVAL;

	buf = kmalloc(len + extend, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

retry_read:
	mutex_lock(&data->dma_access_mutex);
	client->addr = t_cmd->addr;
	ret = __ts_write(client, t_cmd->buf, t_cmd->len);
	if (ret != t_cmd->len) {
		dev_err(&client->dev, "send command error!!\n");
		retval = -EFAULT;
		goto out;
	}

	ret = __ts_read(client, buf, len + extend);
	if (ret != len + extend) {
		dev_err(&client->dev, "reads data error ret = %d!!\n",ret);
		retval = -EFAULT;
		goto out;
	}

out:
	mutex_unlock(&data->dma_access_mutex);
	if(retval) {
		if (--retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(TS_WAKEUP_TIME);
			retval = 0;
			goto retry_read;
		} else {
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
		}
	}else {
		memcpy(val, buf + extend, len);
	}

	kfree(buf);
	return retval;
}

static int __ts_write_reg(struct i2c_client *client, const struct ts_command *t_cmd, u16 len,
				const void *val)
{
	struct ts_data *data = i2c_get_clientdata(client);

	u8 *buf;
	size_t count;
	int ret;
	int retry = 3;

	if (test_flag(I2C_M_RECV_LEN,&t_cmd->flag)) {
		if (t_cmd->buf[0] < TS_OBJECT_START)
			return -EINVAL;
	}

	count = t_cmd->len + len;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, t_cmd->buf, t_cmd->len);
	memcpy(buf + t_cmd->len, val, len);

retry_write:
	mutex_lock(&data->dma_access_mutex);
	client->addr = t_cmd->addr;
	ret = __ts_write(client, buf, count);
	mutex_unlock(&data->dma_access_mutex);
	if (ret == count) {
		ret = 0;
	} else {
		if (--retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(TS_WAKEUP_TIME);
			goto retry_write;
		} else {
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
			ret = -EIO;
		}
	}

	kfree(buf);
	return ret;
}

#else

static int __ts_raw_read(struct i2c_client *client,
					u8 *val, unsigned int count)
{
	struct ts_data *data = (struct ts_data *)i2c_get_clientdata(client);
	struct i2c_msg msg;
	int retry = 3;
	int ret;

retry_read:
	msg.addr = data->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret == 1) {
		ret = 0;
	} else {
		if (--retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(TS_WAKEUP_TIME);
			goto retry_read;
		}else {
			ret = (ret < 0) ? ret : -EIO;
			dev_err(&client->dev, "%s: i2c recv failed (%d)\n",
				__func__, ret);
		}
	}

	return ret;
}

static int __ts_read_reg(struct i2c_client *client,
				struct ts_command *t_cmd, u16 len, void *val)
{
	struct i2c_msg xfer[1];
	int ret;
	int retry = 3;
	u8 *buf;
	u16 extend = 0;
	bool done = false;

	if (test_flag(I2C_M_RECV_LEN,&t_cmd->flag))
		extend = TS_OBJECT_START;

	buf = kmalloc(len + extend, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (t_cmd->buf[0] < extend) {
		kfree(buf);
		return -EINVAL;
	}

	/* Write register */
	xfer[0].addr = t_cmd->addr;
	xfer[0].flags = 0;
	xfer[0].len = t_cmd->len;
	xfer[0].buf = t_cmd->buf;

i2c_read:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		if (--retry) {
			dev_err(&client->dev, "%s: i2c retry reg %02x len %d\n", __func__,t_cmd->buf[0],len);
			msleep(TS_WAKEUP_TIME);
			goto i2c_read;
		} else {
			dev_err(&client->dev, "%s: i2c (0x%2x) transfer failed (%d) reg %d\n",
				__func__, t_cmd->addr, ret, xfer[0].buf[0]);
			ret = -EIO;
			goto i2c_failed;
		}
	}

	if (!done) {
		xfer[0].addr = t_cmd->addr;
		xfer[0].flags = I2C_M_RD;
		xfer[0].len = len + extend;
		xfer[0].buf = buf;
		done = true;
		retry = 3;
		goto i2c_read;
	}

	/* store the reading data */
	memcpy(val, buf + extend, len);
	ret = 0;

i2c_failed:
	kfree(buf);
	return ret;
}

static int __ts_write_reg(struct i2c_client *client, const struct ts_command *t_cmd, u16 len,
				const void *val)
{
	struct i2c_msg xfer[1];
	u8 *buf;
	size_t count;
	int ret;
	int retry = 3;

	if (test_flag(I2C_M_RECV_LEN,&t_cmd->flag)) {
		if (t_cmd->buf[0] < TS_OBJECT_START)
			return -EINVAL;
	}

	count = t_cmd->len + len;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, t_cmd->buf, t_cmd->len);
	memcpy(buf + t_cmd->len, val, len);

	/* Write register */
	xfer[0].addr = t_cmd->addr;
	xfer[0].flags = 0;
	xfer[0].len = count;
	xfer[0].buf = buf;

i2c_write:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		if (--retry) {
			dev_dbg(&client->dev, "%s: i2c retry reg %02x len %d\n", __func__,t_cmd->buf[0],len);
			msleep(TS_WAKEUP_TIME);
			goto i2c_write;
		} else {
			dev_err(&client->dev, "%s: i2c (0x%02x) transfer failed (%d) reg %d\n",
				__func__,t_cmd->addr, ret, xfer[0].buf[0]);
			ret = -EIO;
		}
	} else {
		ret = 0;
	}

	kfree(buf);
	return ret;
}
#endif

static int ts_read_reg(struct i2c_client *client,
				u16 reg, void *val)
{
	struct ts_data *data = (struct ts_data *)i2c_get_clientdata(client);
	struct ts_command command;

	command.addr = data->addr;
	command.buf[0] = (u8)reg;
	command.buf[1] = 1;
	command.len = 2;
	command.flag = I2C_M_RECV_LEN;

	return __ts_read_reg(client, &command, 1, val);
}

/* Used in ts_set_ts_power_cfg(), writing 2 registers one time. */
static int ts_write_report_enflag(struct i2c_client *client, u16 reg, void *val)
{
	struct ts_data *data = (struct ts_data *)i2c_get_clientdata(client);
	struct ts_command command;

	command.addr = data->addr;
	command.buf[0] = (u8)reg;
	command.buf[1] = 1;
	command.len = 2;
	command.flag = I2C_M_RECV_LEN;

	return __ts_write_reg(client, &command, 2, val);
}

static int ts_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	struct ts_data *data = (struct ts_data *)i2c_get_clientdata(client);
	struct ts_command command;

	command.addr = data->addr;
	command.buf[0] = (u8)reg;
	command.buf[1] = 1;
	command.len = 2;
	command.flag = I2C_M_RECV_LEN;

	return __ts_write_reg(client, &command, 1, &val);
}

static int ts_bootloader_read(struct i2c_client *client,
				u8 idx, u16 reg, void *val, unsigned int count)
{
	struct ts_data *data = (struct ts_data *)i2c_get_clientdata(client);
	struct ts_command command;

	if (!data->cmd_tab)
		return -ENODEV;

	if (idx >= TS_COMMAND_MAX)
		return -EINVAL;

	memcpy(&command, &data->cmd_tab[idx], sizeof(command));
	if (idx == TS_BOOT_RW) {
		command.buf[2] = (u8)(reg >> 8);
		command.buf[3] = (u8)reg;
	}

	command.addr = data->bootloader_addr;
	command.flag = 0;

	return __ts_read_reg(client, &command, count, val);
}

static int ts_bootloader_write(struct i2c_client *client,
				u8 idx, u16 reg, void *val, unsigned int count)
{
	struct ts_data *data = (struct ts_data *)i2c_get_clientdata(client);
	struct ts_command command;

	if (!data->cmd_tab)
		return -ENODEV;

	if (idx >= TS_COMMAND_MAX)
		return -EINVAL;

	memcpy(&command, &data->cmd_tab[idx], sizeof(command));
	if (idx == TS_BOOT_RESET) {
		command.addr = data->addr;
		command.flag = I2C_M_RECV_LEN;
	}else {
		command.addr = data->bootloader_addr;
		command.flag = 0;
	}

	if (idx == TS_BOOT_RW) {
		command.buf[2] = (u8)(reg >> 8);
		command.buf[3] = (u8)reg;
	}

	return __ts_write_reg(client, &command, count, val);
}

#if defined(CONFIG_TS_PROBE_ALTERNATIVE_CHIP)
static unsigned short ts_lookup_chip_address(struct ts_data *data, int retry)
{
	unsigned short address = data->addr & 0x7F;  //7 bit address

	if (retry && !(retry & 0x1)) {
		address++;
	}

	dev_err(&data->client->dev, "[ts] try %d chip address 0x%x\n",retry,address);

	return address;
}
#endif

static void ts_reset_slots(struct ts_data *data);

static bool ts_proc_t15_messages(struct ts_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	struct device *dev = &data->client->dev;
	int key;
	bool curr_state, new_state;
	bool sync = false;
	unsigned long keystates = le32_to_cpu(msg[0]);
	u8 num_keys;
	const unsigned int *keymap;
	u8 pressed = 0;

	/* do not report events if input device not yet registered */
	if (!input_dev)
		return 0;

	if (!data->enable_reporting)
		return 0;

	if (!data->pdata->keymap || !data->pdata->num_keys)
		return 0;

	if (data->test_enabled == 1) {
		num_keys = data->pdata->num_keys[TS_KEY_2];
		keymap = data->pdata->keymap[TS_KEY_2];
	} else {
		num_keys = data->pdata->num_keys[TS_KEY];
		keymap = data->pdata->keymap[TS_KEY];
	}

	for (key = 0; key < num_keys; key++) {
		curr_state = test_bit(key, &data->key_status);
		new_state = test_bit(key, &keystates);

		if (!curr_state && new_state) {
			dev_info(dev, "T15 key press: %u\n", key);
			__set_bit(key, &data->key_status);
			input_event(input_dev, EV_KEY,
					keymap[key], 1);
			sync = true;
		} else if (curr_state && !new_state) {
			dev_info(dev, "T15 key release: %u\n", key);
			__clear_bit(key, &data->key_status);
			input_event(input_dev, EV_KEY,
					keymap[key], 0);
			sync = true;
		}

		if (new_state)
			pressed++;
	}

	if (sync)
		input_sync(input_dev);

	return pressed;
}

static int ts_proc_message(struct ts_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;
	int ret = 0;

	if (report_id == TS_RPTID_NOMSG)
		return 0;

	if (report_id >= data->T15_reportid_min
			&& report_id <= data->T15_reportid_max) {
		ret += ts_proc_t15_messages(data, message);
	} else {
		dump = true;
	}

	if (dump)
		ts_dump_message(data, message);

	if (data->debug_v2_enabled)
		ts_debug_msg_add(data, message);

	return ret;
}

static int ts_read_and_process_messages(struct ts_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;
	//struct ts_command command;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */

	ret = __ts_raw_read(data->client, data->msg_buf, data->T5_msg_size * count);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

/*
	command.addr = data->addr;
	command.buf[0] = (u8)data->T5_address;
	command.buf[1] = data->T5_msg_size * count;
	command.len = 1;
	command.flag = I2C_M_RECV_LEN;

	ret = __ts_read_reg(data->client, &command, data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}
*/
	dev_info(dev, "messages (%x)\n", data->msg_buf[0]);

	for (i = 0;  i < count; i++) {
		ret = ts_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret >= 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

static irqreturn_t ts_process_messages(struct ts_data *data)
{
	int total_handled = 0, num_handled;

	/* read two at a time until an invalid message or else we reach
	 * reportid limit */
	num_handled = ts_read_and_process_messages(data, 1);
	if (num_handled < 0)
		return IRQ_NONE;

	total_handled += num_handled;
/*
	if (data->suspended)
		break;
	if (!data->enable_reporting)
		break;
*/
	data->last_message_count = total_handled;
	return IRQ_HANDLED;
}

static irqreturn_t ts_interrupt(int irq, void *dev_id)
{
	struct ts_data *data = dev_id;
	irqreturn_t ret;

	if (data->in_bootloader) {
		/* bootloader state transition completion */
		complete(&data->bl_completion);
		return IRQ_HANDLED;
	}

	if (!data->info)
		return IRQ_NONE;

	if (!data->msg_buf)
		return IRQ_NONE;

	ret = ts_process_messages(data);
	//free_irq(irq, dev_id);

	return IRQ_HANDLED;
}

#if defined(CONFIG_TS_IRQ_WORKQUEUE)

static void ts_active_proc_thread(void *dev_id, unsigned int event)
{
	struct ts_data *data;

	data = (struct ts_data *)dev_id;
	if (!data)
		return;

	set_bit(event,&data->busy);
	dev_dbg(&data->client->dev, "ts_active_proc_thread event %d busy %lx depth %d\n",
		event, data->busy,atomic_read(&data->depth));
	wake_up_interruptible(&data->wait);
}

#if defined(CONFIG_TS_EXTERNAL_TRIGGER_IRQ_WORKQUEUE)

static void board_pulse_irq_thread(void)
{
	struct ts_data *data = ts_g_data;

	if (!data) {
		printk(KERN_ERR "TS_EXTERNAL_TRIGGER_IRQ: ts_g_data is not prepared \n");
		return;
	}

	if(atomic_read(&data->depth)) {
		board_disable_irq(data->pdata,data->irq);
		atomic_dec(&data->depth);
	}
	ts_active_proc_thread(data,TS_EVENT_IRQ);
}

#else

static irqreturn_t ts_interrupt_pulse_workqueue(int irq, void *dev_id)
{
	struct ts_data *data = (struct ts_data *)dev_id;

#if !defined(CONFIG_TS_EXTERNAL_TRIGGER_IRQ_WORKQUEUE)
	if(atomic_read(&data->depth)) {
		disable_irq_nosync(irq);
		atomic_dec(&data->depth);
	}
#endif
	if (data) {
		//cancel_delayed_work_sync(&data->irq_work);
		ts_active_proc_thread(data,TS_EVENT_IRQ);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}
#endif

//warning: don't put any write/read code in this function
static int ts_process_message_thread(void *dev_id)
{
	struct ts_data *data = dev_id;
	struct ts_platform_data *pdata = data->pdata;
	long interval = MAX_SCHEDULE_TIMEOUT;
	int post = false;
	irqreturn_t iret;

	while (!kthread_should_stop()) {

#if defined(CONFIG_TS_PLUGIN_SUPPORT)
		ts_plugin_pre_process(&data->plug,data->in_bootloader);
#endif
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible_timeout(
			data->wait,
			test_bit(TS_EVENT_IRQ,&data->busy)||test_bit(TS_EVENT_EXTERN,&data->busy)||
				kthread_should_stop(),
			interval);

		dev_dbg(&data->client->dev, "ts_process_message_thread busy %lx suspend %d  boot %d interval %ld(0x%lx)\n",
			data->busy,
			data->suspended,
			data->in_bootloader,
			interval,interval);

		if (kthread_should_stop()) {
#if defined(CONFIG_TS_PLUGIN_SUPPORT)
			ts_plugin_thread_stopped(&data->plug);
#endif
			break;
		}

		if (test_and_clear_bit(TS_EVENT_IRQ,&data->busy)) {
			iret = ts_interrupt(data->irq, (void *)data);
			if(iret == IRQ_NONE) {
				if (data->pdata->irqflags & IRQF_TRIGGER_LOW) {
					dev_err(&data->client->dev, "Invalid irq: busy 0x%lx depth %d, sleep\n",
						data->busy,atomic_read(&data->depth));
						msleep(TS_RESET_TIME);
				}
			}

			if(!atomic_read(&data->depth)) {
				atomic_inc(&data->depth);
				board_enable_irq(pdata,data->irq);
			}
		}

		if (data->suspended) {
			interval = MAX_SCHEDULE_TIMEOUT;
			if (test_bit(TS_EVENT_EXTERN,&data->busy))
				post = true;
		}
#if defined(CONFIG_TS_PLUGIN_SUPPORT)
		else{
			post = true;
		}

		if (post) {
			clear_bit(TS_EVENT_EXTERN,&data->busy);
			interval = (long)ts_plugin_post_process(&data->plug,data->in_bootloader);
		}
#endif
	}

	set_current_state(TASK_RUNNING);

	return 0;
}

#endif

/*
static int ts_xxxcommand(struct ts_data *data, u16 cmd,
			u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	ret = ts_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(TS_WAKEUP_TIME);
		ret = __ts_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}
*/

static int ts_soft_reset(struct ts_data *data)
{
	struct device *dev = &data->client->dev;

	dev_info(dev, "Resetting chip\n");

/*
	init_completion(data->reset_completion);

	ret = ts_wait_for_completion(data, &data->reset_completion,
					  TS_RESET_TIMEOUT);
	if (ret)
		return ret;
*/

	msleep(TS_WAKEUP_TIME);

	return 0;
}

static void ts_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 ts_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		ts_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		ts_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static u32 ts_calculate_checksum(u8 *base, off_t len)
{
	int i;
	u32 sum = 0;

	for (i = 0; i < len; i++)
		sum += base[i];

	return sum;
}

static int ts_check_retrigen(struct ts_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	int val;

	if (data->pdata->irqflags & IRQF_TRIGGER_LOW)
		return 0;

	error = ts_read_reg(client, REG_PWM_OUTPUT_ENABLE, &val);
	if (error)
		return error;

	if (!val) {
		dev_warn(&client->dev, "Enabling PWM output\n");
		error = ts_write_reg(client, REG_PWM_OUTPUT_ENABLE, 1);
	}

	return error;
}

static int ts_set_ts_power_cfg(struct ts_data *data, u8 sleep)
{
	u8 enable, reg_data[2];
	int error;
	int i;

	if (sleep == TS_POWER_CFG_DEEPSLEEP)
		enable = 0;
	else
		enable = 0x1;

	/* Write 2 registers instead of only writing REG_REPORT_ENABLE
	 * to avoid the touch key wakeup unusable issue, MOBILEP-3204,
	 * the following registuer is REG_DEBUG_ENABLE, nomally it shouldn't
	 * be enable */
	reg_data[0] = enable; /* Value of REG_REPORT_ENABLE */
	reg_data[1] = 0; /* Value of REG_DEBUG_ENABLE */

	for(i = 0; i< 5; i++)	{
		error = ts_write_report_enflag(data->client, REG_REPORT_ENABLE, reg_data);
	 	if (!error)
			 break;
		msleep(10);
	}

	return 0;
}

static int ts_init_ts_power_cfg(struct ts_data *data)
{
	struct device *dev = &data->client->dev;
	int error;
	u8 enable,debug;
	bool retry = false;

recheck:
	error = ts_read_reg(data->client, REG_REPORT_ENABLE, &enable);
	if (error)
		return error;

	error = ts_read_reg(data->client, REG_DEBUG_ENABLE, &debug);
	if (error)
		return error;

	if (!enable || debug) {
		if (!retry) {
			dev_info(dev, "TS cfg zero, resetting\n");
			ts_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
			dev_info(dev, "TS cfg zero after reset, overriding\n");
			return ts_set_ts_power_cfg(data, TS_POWER_CFG_RUN);
		}
	}

	return 0;
}

static int ts_acquire_irq(struct ts_data *data)
{
	struct ts_platform_data *pdata = data->pdata;

	if(!atomic_read(&data->depth)) {
		atomic_inc(&data->depth);
		board_enable_irq(pdata,data->irq);
	}

	return 0;
}

static void ts_free_input_device(struct ts_data *data)
{
	ts_debug_msg_remove(data);

	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void ts_free_object_table(struct ts_data *data)
{
	kfree(data->raw_info_block);
	data->info = NULL;
	data->raw_info_block = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;

	ts_free_input_device(data);

	data->enable_reporting = false;
	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T15_address = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->max_reportid = 0;
}

static int ts_parse_object_table(struct ts_data *data,
				  char *object_table)
{
	struct i2c_client *client = data->client;

	/* Valid Report IDs start counting from 1 */
	data->T5_address = /*REG_KEY_STATUS*/TS_OBJECT_START;
	data->T5_msg_size = 1;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0xF;
	data->T15_address = 0x2;

	/* Store maximum reportid */
	data->max_reportid = data->T15_reportid_max;

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf) {
		dev_err(&client->dev, "Failed to allocate message buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static int ts_read_info_block(struct ts_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size,mem_size;
	void *id_buf, *buf;
	u32 calculated_crc;
	u8 *crc_ptr, val;
	struct ts_command command;

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		ts_free_object_table(data);

	/* Read 1-byte len information block starting at address 0x1F */
	error = ts_read_reg(client, REG_MEM_SIZE, &val);
	if (error) {
		return error;
	}
	dev_notice(&client->dev, "chip memory %d(%02x)\n", val, val);
	if (val == 0 || val == 0xFF)
		val = TS_REG_SIZE;

	mem_size = val;
	size = mem_size + sizeof(struct ts_info);
	id_buf = kzalloc(size, GFP_KERNEL);
	if (!id_buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	buf = id_buf + sizeof(struct ts_info);
	size = mem_size - TS_OBJECT_START;
	command.addr = data->addr;
	command.buf[0] = TS_OBJECT_START;
	command.buf[1] = size;
	command.len = 2;
	command.flag = I2C_M_RECV_LEN;
	error = __ts_read_reg(client, &command, size, buf + TS_OBJECT_START);
	if (error) {
		kfree(id_buf);
		return error;
	}

	/* Extract & calculate checksum */
	crc_ptr = buf + mem_size - TS_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = ts_calculate_crc(buf, 0,
						size - TS_INFO_CHECKSUM_SIZE);

	/* CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol */
	if ((data->info_crc == 0) || (data->info_crc != calculated_crc)) {
		dev_err(&client->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, data->info_crc);
		//error = -EIO;
		//goto err_free_mem;
	}

	/* Save pointers in device data structure */
	data->raw_info_block = id_buf;
	data->info = id_buf;
	if (mem_size > REG_FW_VERSIONS_EX) {
		data->info->family_id = TO_VAL(buf,REG_IC_INFO_H);
		data->info->variant_id = TO_VAL(buf,REG_IC_INFO_L);
		data->info->version = TO_VAL(buf,REG_FW_VERSIONS2);
		data->info->build = TO_VAL(buf,REG_FW_VERSIONS);
	}else {
		data->info->family_id = 0;
		data->info->variant_id = 0;
		data->info->version = 0;
		data->info->build = TO_VAL(buf,REG_FW_VERSIONS);
	}
	data->info->object_num = mem_size;
	data->mem_size = mem_size;

	dev_info(&client->dev,
		 "Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id,
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build, data->info->object_num);

	/* Parse object table information */
	error = ts_parse_object_table(data, buf);
	if (error) {
		dev_err(&client->dev, "Error %d parsing object table\n", error);
		ts_free_object_table(data);
		return error;
	}

	return 0;

//err_free_mem:
	kfree(id_buf);
	return error;
}

static void ts_regulator_enable(struct ts_data *data)
{
	int ret;
	gpio_set_value(data->pdata->gpio_reset, 0);

	ret = regulator_enable(data->reg_vdd);
	ret = regulator_enable(data->reg_avdd);
	msleep(TS_REGULATOR_DELAY);

	init_completion(&data->bl_completion);
	gpio_set_value(data->pdata->gpio_reset, 1);
	ts_wait_for_completion(data, &data->bl_completion, TS_POWERON_DELAY);
}

static void ts_regulator_disable(struct ts_data *data)
{
	regulator_disable(data->reg_vdd);
	regulator_disable(data->reg_avdd);
}

static void ts_probe_regulators(struct ts_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	/* According to maXTouch power sequencing specification, RESET line
	 * must be kept low until some time after regulators come up to
	 * voltage */

	board_gpio_init(data->pdata);
	if(atomic_read(&data->depth)) {
		board_disable_irq(data->pdata,data->irq);
		atomic_dec(&data->depth);
	}

	if (!data->pdata->gpio_reset) {
		dev_warn(dev, "Must have reset GPIO to use regulator support\n");
		goto fail;
	}

	data->reg_vdd = regulator_get(dev, "vdd");
	if (IS_ERR(data->reg_vdd)) {
		error = PTR_ERR(data->reg_vdd);
		dev_err(dev, "Error %d getting vdd regulator\n", error);
		goto fail;
	}

	data->reg_avdd = regulator_get(dev, "avdd");
	if (IS_ERR(data->reg_vdd)) {
		error = PTR_ERR(data->reg_vdd);
		dev_err(dev, "Error %d getting avdd regulator\n", error);
		goto fail_release;
	}

	//no need use regulator for suspend and resume, set false here
	data->use_regulator = true;
	ts_regulator_enable(data);

	dev_dbg(dev, "Initialised regulators\n");
	return;

fail_release:
	regulator_put(data->reg_vdd);
fail:
	data->reg_vdd = NULL;
	data->reg_avdd = NULL;
	data->use_regulator = false;
}

static int ts_input_open(struct input_dev *dev);
static void ts_input_close(struct input_dev *dev);

static int ts_configure_objects(struct ts_data *data);
static int ts_initialize_ts_input_device(struct ts_data *data);

static int ts_initialize(struct ts_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	bool alt_bootloader_addr = false;
	bool retry = false;
#if defined(CONFIG_TS_PROBE_ALTERNATIVE_CHIP)
	unsigned short addr_bak = 0;
	int probe_retry = 0;
#endif
retry_info:
	error = ts_read_info_block(data);
#if defined(CONFIG_TS_PROBE_ALTERNATIVE_CHIP)
	while(error) {
#	if defined(CONFIG_TS_POWER_CONTROL_SUPPORT_AT_PROBE)
		board_gpio_init(data->pdata);
#	endif
		data->addr = ts_lookup_chip_address(data, probe_retry);
#	if defined(CONFIG_TS_I2C_DMA)
		data->addr |= I2C_RS_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG;
#	endif
		if (!addr_bak)
			addr_bak = data->addr;
		error = ts_read_info_block(data);

		if (++probe_retry > CONFIG_TS_PROBE_ALTERNATIVE_CHIP)
			break;
	}
	if (error)
		data->addr = addr_bak;
#endif
	if (error) {
retry_bootloader:
		error = ts_probe_bootloader(data, alt_bootloader_addr);
		if (error) {
			if (alt_bootloader_addr) {
				/* Chip is not in appmode or bootloader mode */
				return error;
			}

			dev_info(&client->dev, "Trying alternate bootloader address\n");
			alt_bootloader_addr = true;
			goto retry_bootloader;
		} else {
			if (retry) {
				dev_err(&client->dev,
						"Could not recover device from "
						"bootloader mode\n");
				/* this is not an error state, we can reflash
				 * from here */
				data->in_bootloader = true;
				return 0;
			}

			/* Attempt to exit bootloader into app mode */
			ts_send_bootloader_cmd(data->client, TS_ENTER_APP);
			msleep(TS_FW_RESET_TIME);
			retry = true;
			goto retry_info;
		}
	}

	error = ts_check_retrigen(data);
	if (error) {
		dev_err(&client->dev,
				"%s, ts_check_retrigen returns error(%d)\n",
				__func__, error);
		return error;
	}

	error = ts_configure_objects(data);
	if (error) {
		dev_err(&client->dev,
				"%s, ts_configure_objects returns error(%d)\n",
				__func__, error);
		return error;
	}

	board_init_irq(data->pdata);

	//board_disable_irq(data->pdata,data->irq);
	error = ts_acquire_irq(data);
	if (error)
		return error;

	return 0;
}

static int ts_configure_objects(struct ts_data *data)
{
	struct i2c_client *client = data->client;
	int error;

	error = ts_debug_msg_init(data);
	if (error)
		return error;

	error = ts_init_ts_power_cfg(data);
	if (error) {
		dev_err(&client->dev, "Failed to initialize power cfg\n");
		return error;
	}

	error = ts_initialize_ts_input_device(data);
	if (error)
		return error;

	data->enable_reporting = true;
	return 0;
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t ts_fw_version_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct ts_data *data = dev_get_drvdata(dev);

	if (!data->info)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X",
			 data->info->version >> 4, data->info->version & 0xf,
			 data->info->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t ts_hw_version_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct ts_data *data = dev_get_drvdata(dev);

	if (!data->info)
		return 0;

	return scnprintf(buf, PAGE_SIZE, "%u.%u",
			data->info->family_id, data->info->variant_id);
}

#if 0

static ssize_t ts_show_reg(char *buf, int len,
				 const u8 *val)
{
	int i;
	int count = 0;

	for (i = 0; i < len; i++) {
		if(!(i % 16))
			count += scnprintf(buf + i, PAGE_SIZE - count,
					"\t[%2u]: ", i);

		count += scnprintf(buf + i, PAGE_SIZE - count,
				"%02x \n", val[i]);
		if(i && !((i + 1) % 16))
			count += scnprintf(buf + count, PAGE_SIZE - count, "\n");
	}
	return count;
}


static ssize_t ts_object_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct ts_data *data = dev_get_drvdata(dev);
	int count = 0;
	int error;
	u8 *obuf;

	if (!data->info)
		return -ENODEV;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(data->info->object_num, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = __ts_read_reg(data->client, 0, data->info->object_num, obuf);
	if (error)
		goto done;

	count+= ts_show_reg(buf, data->info->object_num, obuf);

done:
	kfree(obuf);
	return error ?: count;
}
#endif

static int ts_check_firmware_format(struct device *dev,
					 const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/* To convert file try
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw */
	dev_err(dev, "Aborting: firmware file must be in binary format\n");

	return -1;
}

struct h_record_t {
	u8 count;
	u16 offset;
	u8 type;
}__packed;

#define MAX_HEX_RECORDER_LEN (sizeof(struct h_record_t) + TS_MAX_BLOCK_WRITE)

struct frame_data {
	int idx;
	u16 addr;
	u8 size;
};

static int ts_load_fw(struct device *dev)
{
	struct ts_data *data = dev_get_drvdata(dev);
	struct ts_platform_data *pdata = data->pdata;
	const struct firmware *fw = NULL;
	unsigned int retry = 0;
	unsigned int frame;
	unsigned int pos = 0;
	//struct frame_data frame;
	struct h_record_t *hrec;
	void *frame_buf = NULL;
	u16 ofs,flash_size = TS_EEPROM_SIZE;
	u32 check_sum;
	int ret;

	if (!data->fw_name)
		return -EEXIST;

	dev_info(dev, "ts_load_fw %s\n", data->fw_name);

	ret = request_firmware(&fw, data->fw_name, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", data->fw_name);
		return ret;
	}

	/* Check for incorrect enc file */
	ret = ts_check_firmware_format(dev, fw);
	if (ret)
		goto release_firmware;

	if (data->suspended) {
		if (data->use_regulator)
			ts_regulator_enable(data);
		data->suspended = false;
	}

	if (data->in_bootloader) {
		ret = ts_check_bootloader(data, &flash_size);
		if(ret) {
			dev_err(dev, "false bootloader check %d\n", ret);
			data->in_bootloader = false;
		}
	}

	if (!data->in_bootloader) {
		/* Change to the bootloader mode */
		/*ts_set_retrigen(data);*/

		data->in_bootloader = true;

		ret = ts_send_bootloader_cmd(data->client, TS_BOOT_RESET);
		if (ret) {
			dev_err(dev, "reset to boot loader mode return %d\n", ret);
			//don't return failed, maybe it's in bootloader mode
			//goto release_firmware;
		}
		msleep(TS_RESET_TIME);

		ts_lookup_bootloader_address(data, 0);

		/* At this stage, do not need to scan since we know family ID */
		do {
			ret = ts_check_bootloader(data, &flash_size);
			if(ret == 0)
				break;
			dev_err(dev, "ts_bootloader_read failed %d retry %d\n", ret,retry);
			ts_lookup_bootloader_address(data, retry);
		}while(++retry < 2);

		if (ret) {
			data->in_bootloader = false;
			goto release_firmware;
		}
	}

	frame_buf = devm_kzalloc(dev, flash_size * 2, GFP_KERNEL);
	if (!frame_buf) {
		dev_err(dev, "Unable to alloc mem %s size %d\n", data->fw_name, flash_size);
		ret = -ENOMEM;
		goto release_firmware;
	}

	if(!atomic_read(&data->depth)) {
		atomic_inc(&data->depth);
		board_disable_irq(pdata,data->irq);
	}

	ts_free_object_table(data);
	init_completion(&data->bl_completion);

	memset(frame_buf, 0xff, flash_size);
	while (pos < fw->size) {
		hrec = (struct h_record_t *)(fw->data + pos);
		ofs = be16_to_cpu(hrec->offset);
		dev_dbg(dev, "get pos %d/%zd ofs 0x%04x count %d type %d\n",
			 pos, fw->size, ofs, hrec->count, hrec->type);

		if (hrec->type == 0x0 &&
			ofs + hrec->count < flash_size) {  //data frame
			memcpy(frame_buf + ofs, hrec + 1, hrec->count);
		}else {
			dev_err(dev, "skip pos %d/%zd ofs 0x%04x count %d type %d\n",
				 pos, fw->size, ofs, hrec->count, hrec->type);
		}

		pos += hrec->count + sizeof(struct h_record_t)+ 1; /* Take account of CRC bytes */
	}

	check_sum = ts_calculate_checksum(frame_buf, flash_size - 2);
	((u8 *)frame_buf)[flash_size - 2] = (u8)((check_sum >> 8) & 0xff);
	((u8 *)frame_buf)[flash_size - 1] = (u8)(check_sum & 0xff);

	for (ofs = 0,frame = 0; ofs < flash_size; ofs += TS_MAX_BLOCK_WRITE) {
		/* Write one frame to device */

		ret = ts_bootloader_write(data->client, TS_BOOT_RW, ofs, frame_buf + ofs,TS_MAX_BLOCK_WRITE);
		if (ret)
			goto disable_irq;

		frame++;
		if (frame % 10 == 0) {
			dev_dbg(dev, "Sent %d frames, %d/%d bytes\n",
				 frame, ofs, flash_size);
		}
	}
	dev_info(dev, "Sent %d frames, %d bytes success\n", frame, ofs);

	msleep(10);
	ts_send_bootloader_cmd(data->client, TS_ENTER_APP);
	msleep(TS_FW_RESET_TIME);

	dev_notice(dev, "update firmware success\n");

	data->in_bootloader = false;

disable_irq:
	if(atomic_read(&data->depth)) {
		board_disable_irq(data->pdata,data->irq);
		atomic_dec(&data->depth);
	}
#if defined(CONFIG_TS_IRQ_NESTED)
	if(test_and_clear_bit(TS_EVENT_IRQ,&data->busy)) {
		ts_acquire_irq(data);
	}
#endif
	data->busy = 0;
release_firmware:
	release_firmware(fw);
	if (frame_buf)
		devm_kfree(dev,frame_buf);
	return ret;
}

static int ts_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count, bool alternative)
{
	struct ts_data *data = dev_get_drvdata(dev);
	char *file_name_tmp;
	char suffix[16];

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1 + sizeof(suffix), GFP_KERNEL);
	if (!file_name_tmp) {
		dev_warn(dev, "no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	if (alternative) {
		snprintf(suffix, sizeof(suffix), ".%02X.fw",
			(u8)(data->addr & 0x7f));
		strncat((*file_name),suffix,count + 1 + sizeof(suffix));
	}

	return 0;
}

static ssize_t ts_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct ts_data *data = dev_get_drvdata(dev);
	int error;

	dev_info(dev, "ts_update_fw_store\n");

	error = ts_update_file_name(dev, &data->fw_name, buf, count, false);
	if (error)
		return error;

	//lock it for disable outside access
	mutex_lock(&data->access_mutex);
	error = ts_load_fw(dev);
	mutex_unlock(&data->access_mutex);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		data->suspended = false;

		error = ts_initialize(data);
		if (error)
			return error;
	}

	return count;
}

static ssize_t ts_debug_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_data *data = dev_get_drvdata(dev);
	char c;

	c = data->debug_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t tk_test_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_data *data = dev_get_drvdata(dev);
	char c;

	c = data->test_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t tk_test_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ts_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		if (i == 1)
			data->test_enabled = 1;
		else
			data->test_enabled = 0;
		return count;
	} else {
		dev_dbg(dev, "test_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t ts_debug_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t ts_debug_v2_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ts_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		if (i == 1)
			ts_debug_msg_enable(data);
		else
			ts_debug_msg_disable(data);

		return count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t ts_debug_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ts_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->debug_enabled = (i == 1);

		dev_dbg(dev, "%s\n", i ? "debug enabled" : "debug disabled");
		return count;
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t ts_bootloader_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ts_data *data = dev_get_drvdata(dev);
	char c;

	c = data->in_bootloader ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t ts_bootloader_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ts_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->in_bootloader = (i == 1);

		dev_dbg(dev, "%s\n", i ? "in bootloader" : "app mode");
		return count;
	} else {
		dev_dbg(dev, "in_bootloader write error\n");
		return -EINVAL;
	}
}

static int ts_check_mem_access_params(struct ts_data *data, loff_t off,
						size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	return 0;
}

static ssize_t ts_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct ts_data *data = dev_get_drvdata(dev);
	struct ts_command command;
	int ret = 0;

	ret = ts_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	mutex_lock(&data->access_mutex);

	if (count > 0) {
		command.addr = data->addr;
		command.buf[0] = (u8)off;
		command.buf[1] = (u8)count;
		command.len = 2;
		command.flag = I2C_M_RECV_LEN;
		ret = __ts_read_reg(data->client, &command, count, buf);
	}
	mutex_unlock(&data->access_mutex);

	return ret == 0 ? count : ret;
}

static ssize_t ts_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct ts_data *data = dev_get_drvdata(dev);
	struct ts_command command;
	int ret = 0;

	ret = ts_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	mutex_lock(&data->access_mutex);

	if (count > 0) {
		command.addr = data->addr;
		command.buf[0] = (u8)off;
		command.buf[1] = 1;
		command.len = 2;
		command.flag = I2C_M_RECV_LEN;
		ret = __ts_write_reg(data->client, &command, count, buf);
	}
	mutex_unlock(&data->access_mutex);

	return ret == 0 ? count : 0;
}

static ssize_t ts_dbg_reg_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct ts_data *data = dev_get_drvdata(dev);
	ssize_t offset = 0;
	char *raw_buf;
	struct ts_command command;
	u16 size;
	int i;
	int error;

	if (!data->raw_info_block)
		return -ENODEV;

	if (data->in_bootloader)
		return -EBUSY;

	raw_buf = data->raw_info_block + sizeof(struct ts_info);
	size = data->mem_size;
	memset(raw_buf, 0, TS_OBJECT_START);
	/*
	raw_buf[REG_REPORT_ENABLE] = 1;
	raw_buf[REG_REPORT_ENABLE + 1] = 0;
	command.addr = data->addr;
	command.buf[0] = REG_REPORT_ENABLE;
	command.buf[1] = 2;
	command.len = 2;
	command.flag = 0;
	error = __ts_write_reg(data->client, &command, 2, raw_buf + REG_REPORT_ENABLE);
	if (error) {
		dev_err(dev, "enable report error %d\n",error);
		return error;
	}
	msleep(10);
	*/
	error = __ts_raw_read(data->client, raw_buf, 1);
	if (error) {
		dev_err(dev, "read status error %d\n",error);
		return error;
	}

	/*
	raw_buf[REG_REPORT_ENABLE] = 1;
	raw_buf[REG_REPORT_ENABLE + 1] = 1;
	command.addr = data->addr;
	command.buf[0] = REG_REPORT_ENABLE;
	command.buf[1] = 2;
	command.len = 2;
	command.flag = I2C_M_RECV_LEN;
	error = __ts_write_reg(data->client, &command, 2, raw_buf + REG_REPORT_ENABLE);
	if (error) {
		dev_err(dev, "enable debug error %d\n",error);
		return error;
	}
	msleep(10);
	*/
	command.addr = data->addr;
	command.buf[0] = TS_OBJECT_START;
	command.buf[1] = size;
	command.len = 2;
	command.flag = I2C_M_RECV_LEN;
	error = __ts_read_reg(data->client, &command, size - TS_OBJECT_START, raw_buf + TS_OBJECT_START);
	if (error) {
		dev_err(dev, "read reg list error %d\n",error);
		return error;
	}

	for (i = 0; i < 16; i++) {
		if (!i)
			offset += scnprintf(buf + offset, PAGE_SIZE, "     ");
		offset += scnprintf(buf + offset, PAGE_SIZE, "%2X ", i);
	}
	offset += scnprintf(buf + offset, PAGE_SIZE, "\n");

	for (i = 0; i < size; i++) {
		if (!i || !(i%16))
			offset += scnprintf(buf + offset, PAGE_SIZE, "%04x: ",i);
		if (i < TS_OBJECT_START) {
			if (raw_buf[i])
				offset += scnprintf(buf + offset, PAGE_SIZE, "%02x ",
						 raw_buf[i]);
			else
				offset += scnprintf(buf + offset, PAGE_SIZE, "-- ");
		}else {
			offset += scnprintf(buf + offset, PAGE_SIZE, "%02x ",
				 raw_buf[i]);
			if (i && !((i + 1)%16))
				offset += scnprintf(buf + offset, PAGE_SIZE, "\n");
		}
	}

	if ((i%16))
		offset += scnprintf(buf + offset, PAGE_SIZE, "\n");
/*
	raw_buf[REG_REPORT_ENABLE] = 1;
	raw_buf[REG_REPORT_ENABLE + 1] = 0;
	command.addr = data->addr;
	command.buf[0] = REG_REPORT_ENABLE;
	command.buf[1] = 2;
	command.len = 2;
	command.flag = 0;
	error = __ts_write_reg(data->client, &command, 2, raw_buf + REG_REPORT_ENABLE);
	if (error) {
		dev_err(dev, "enable report error %d\n",error);
		return error;
	}
	msleep(10);
*/
	return offset;
}

static ssize_t ts_dbg_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ts_data *data = dev_get_drvdata(dev);
	u16 reg,len;
	char *tmp;
	struct ts_command command;

	int offset,ofs,k,val,ret;
	ret = -EINVAL;

	if (!data->raw_info_block)
		return -ENODEV;

	if (!data->mem_size)
		return -ENODEV;

	tmp = devm_kzalloc(dev, data->mem_size, GFP_KERNEL);
	if (!tmp) {
		dev_info(dev, "no memory for store\n");
		return -ENOMEM;
	}

	memset(&command, 0, sizeof(command));
	offset = ofs = 0;

	if (sscanf(buf, "R%hx len %hd: %n",&reg,&len,&offset) == 2) {
		dev_info(dev, "%s\n",buf);
		dev_info(dev, "T%hd len %hd:^(ofs %d)\n",reg, len, offset);

		if (reg > data->mem_size) {
			ret = -EINVAL;
			goto free_mem;
		}

		if (reg + len  > data->mem_size)
			len = data->mem_size - reg;

		for (k = 0; k < len; k++) {
			offset += ofs;
			if (offset < count) {
				dev_info(dev, "%s\n",buf + offset);
				ret = sscanf(buf + offset, "%x %n",
					&val,&ofs);
				if (ret == 1) {
					tmp[k] = (u8)val;
					dev_info(dev, "%x",tmp[k]);
				}else
					break;
			}else
				break;
		}
		if (k && ret > 0) {
			dev_info(dev, "set reg data %d\n", k);
			print_hex_dump(KERN_INFO, "[ts]", DUMP_PREFIX_NONE, 16, 1,
				tmp, k, false);
			command.addr = data->addr;
			command.buf[0] = (u8)reg;
			command.buf[1] = (u8)k;
			command.len = 2;
			command.flag = I2C_M_RECV_LEN;
			ret = __ts_write_reg(data->client, &command, k, tmp);
			if (ret) {
				dev_err(dev, "write reg %02x len %d error %d\n",reg,k,ret);
				goto free_mem;
			}

			ret = count;
		}
	}else if (sscanf(buf, "cmd %hx %hd: %n",&command.addr,&len,&offset) == 2) {
		dev_info(dev, "%s\n",buf);
		dev_info(dev, "cmd %hd:^(ofs %d)\n",len, offset);
		if (len > sizeof(command.buf))
			len = sizeof(command.addr);

		for (k = 0; k < len; k++) {
			offset += ofs;
			if (offset < count) {
				dev_info(dev, "%s\n",buf + offset);
				ret = sscanf(buf + offset, "%x %n",
					&val,&ofs);
				if (ret == 1) {
					tmp[k] = (u8)val;
					dev_info(dev, "%x",tmp[k]);
				}else
					break;
			}else
				break;
		}
		if (k && ret > 0) {
			dev_info(dev, "set cmd data %d\n", k);
			print_hex_dump(KERN_INFO, "[ts]", DUMP_PREFIX_NONE, 16, 1,
				tmp, k, false);
			if (command.addr == 0)
				command.addr = data->addr;
			memcpy(command.buf, tmp, len);
			command.len = len;
			command.flag = 0;
			ret = __ts_write_reg(data->client, &command, 0, NULL);
			if (ret) {
				dev_err(dev, "write cmd %d error %d\n",len,ret);
				goto free_mem;
			}

			ret = count;
		}
	}else{
		dev_info(dev, "invalid string: %s\n", buf + offset);
	}

free_mem:
	devm_kfree(dev,tmp);
	return ret;
}

static DEVICE_ATTR(fw_version, S_IRUGO, ts_fw_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, ts_hw_version_show, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR | S_IRUSR/*S_IWUSR*/ , NULL, ts_update_fw_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR | S_IRUSR /*S_IWUSR | S_IRUSR*/, NULL, ts_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUGO, ts_debug_notify_show, NULL);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, ts_debug_enable_show,
			ts_debug_enable_store);
static DEVICE_ATTR(bootloader, S_IWUSR | S_IRUSR, ts_bootloader_show,
			ts_bootloader_store);
static DEVICE_ATTR(reg, S_IWUSR | S_IRUSR, ts_dbg_reg_show,
			ts_dbg_reg_store);
static DEVICE_ATTR(tk_test_enable, S_IWUSR | S_IRUSR, tk_test_enable_show,
			tk_test_enable_store);

static struct attribute *ts_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_notify.attr,
	&dev_attr_bootloader.attr,
	&dev_attr_reg.attr,
	&dev_attr_tk_test_enable.attr,
	NULL
};

static const struct attribute_group ts_attr_group = {
	.attrs = ts_attrs,
};

static void ts_reset_slots(struct ts_data *data)
{
	struct input_dev *input_dev = data->input_dev;

	if (!input_dev)
		return;

	input_sync(input_dev);

	data->key_status = 0;
}

static void ts_start(struct ts_data *data, bool resume)
{
	struct device *dev = &data->client->dev;

	if (!data->suspended || data->in_bootloader)
		return;

	dev_info(dev, "ts_start\n");

	if (data->use_regulator) {
		ts_regulator_enable(data);
	} else {
		/* Discard any messages still in message buffer from before
		 * chip went to sleep */
		ts_set_ts_power_cfg(data, TS_POWER_CFG_RUN);
		//ts_t6_command(data, TS_COMMAND_CALIBRATE, 1, false);
	}

	ts_reset_slots(data);  //it's strange some platform will resverse some points in last touch
	data->enable_reporting = true;
	data->suspended = false;

	ts_acquire_irq(data);
}

static void ts_stop(struct ts_data *data,bool suspend)
{
	struct device *dev = &data->client->dev;

	if (data->suspended || data->in_bootloader)
		return;

	dev_info(dev, "ts_stop\n");

	if(atomic_read(&data->depth)) {
		board_disable_irq(data->pdata,data->irq);
		atomic_dec(&data->depth);
	}
	data->enable_reporting = false;

	if (data->use_regulator)
		ts_regulator_disable(data);
	else{
		ts_set_ts_power_cfg(data, TS_POWER_CFG_DEEPSLEEP);
	}

	ts_reset_slots(data);

	data->suspended = true;
}

static int ts_input_open(struct input_dev *input_dev)
{
	struct ts_data *data = input_get_drvdata(input_dev);
	struct device *dev = &data->client->dev;

	dev_info(dev, "ts_input_open\n");

	ts_start(data,false);

#if defined(CONFIG_TS_EXTERNAL_TRIGGER_IRQ_WORKQUEUE)
	//board_pulse_irq_thread();
	//ts_active_proc_thread(data, TS_EVENT_EXTERN);
#endif
	return 0;
}

static void ts_input_close(struct input_dev *input_dev)
{
	struct ts_data *data = input_get_drvdata(input_dev);
	struct device *dev = &data->client->dev;

	dev_info(dev, "ts_input_close\n");

	ts_stop(data,false);
}

#ifdef CONFIG_OF

static struct ts_platform_data *ts_parse_dt(struct i2c_client *client)
{
	struct ts_platform_data *pdata;
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct property *prop;
	u8 *num_keys;
	unsigned int (*keymap)[MAX_KEYS_SUPPORTED_IN_DRIVER];
	int proplen, ret;
	char temp[255];
	int i, j, gpio;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	/* reset gpio */
	gpio = of_get_named_gpio(node, "reset-gpio", 0);
	if (gpio_is_valid(gpio)) {
		if (!gpio_request(gpio, client->name)) {
			gpio_direction_output(gpio, 1);
			pdata->gpio_reset = gpio;
		}
	}

	of_property_read_string(dev->of_node, "atmel,fw_name",
				&pdata->fw_name);

	of_property_read_string(dev->of_node, "atmel,fw_version",
				&pdata->fw_version);

	dev_info(dev, "%s: fw_name: %s, fw_version: %s", __func__,
			pdata->fw_name, pdata->fw_version);

	prop = of_find_property(dev->of_node, "gpio-keymap-num", &proplen);
	if (prop) {
		num_keys = devm_kzalloc(dev,
				sizeof(*num_keys) * NUM_KEY_TYPE, GFP_KERNEL);
			if (!num_keys)
				return NULL;
		keymap = devm_kzalloc(dev,
			sizeof(*keymap) * NUM_KEY_TYPE, GFP_KERNEL);
		if (!keymap) {
			devm_kfree(dev,num_keys);
			return NULL;
		}
		ret = of_property_read_u8_array(client->dev.of_node,
			"gpio-keymap-num", num_keys, NUM_KEY_TYPE);
		if (ret) {
			dev_err(dev,
				"Unable to read device tree key codes num keys: %d\n",
				 ret);
			devm_kfree(dev,num_keys);
			devm_kfree(dev,keymap);
			return NULL;
		}
		num_keys[0] = 3;
		num_keys[1] = 3;
		for (i = 0; i < NUM_KEY_TYPE; i++) {
			sprintf(temp,"gpio-keymap_%d",i);
			ret = of_property_read_u32_array(client->dev.of_node,
				temp, keymap[i], num_keys[i]);
			if (ret) {
				dev_err(dev,
					"Unable to read device tree key codes: %d\n",
					 ret);
				//return NULL;
				num_keys[i] = 0;
			}else {
				if (num_keys[i] > MAX_KEYS_SUPPORTED_IN_DRIVER)
					num_keys[i] = MAX_KEYS_SUPPORTED_IN_DRIVER;
			}
		}
		pdata->num_keys = num_keys;
		pdata->keymap = (void *)keymap;
	}else {
		dev_info(&client->dev, "couldn't find gpio-keymap-num\n");
	}

	dev_info(&client->dev, "ts reset %d, irq flags 0x%lx\n",
		pdata->gpio_reset, pdata->irqflags);
	if (pdata->num_keys) {
		for (i = 0; i < NUM_KEY_TYPE; i++) {
			dev_info(&client->dev, "ts type %d (%d): ",
				i, pdata->num_keys[i]);
			for (j = 0; j < pdata->num_keys[i]; j++)
				dev_info(&client->dev, "%d ",
					pdata->keymap[i][j]);
			dev_info(&client->dev, "\n");
		}
	}

	pdata->irqflags = IRQF_TRIGGER_LOW;
	return pdata;
}

static void ts_free_dt(struct ts_data *data)
{
	struct ts_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;

	if (!pdata)
		return;

	if (pdata->gpio_reset) {
		gpio_free(pdata->gpio_reset);
		pdata->gpio_reset = 0;
	}
	if (pdata->num_keys) {
		devm_kfree(dev, (void *)pdata->num_keys);
		pdata->num_keys = NULL;
	}

	if (pdata->keymap) {
		devm_kfree(dev, (void *)pdata->keymap);
		pdata->keymap = NULL;
	}
}

#endif

static int ts_handle_pdata(struct ts_data *data)
{
	data->pdata = dev_get_platdata(&data->client->dev);

	/* Use provided platform data if present */
	if (data->pdata) {
		if (data->pdata->fw_name)
			ts_update_file_name(&data->client->dev,
						 &data->fw_name,
						 data->pdata->fw_name,
						 strlen(data->pdata->fw_name),
						 false);

		return 0;
	}

	data->pdata = devm_kzalloc(&data->client->dev,sizeof(*data->pdata), GFP_KERNEL);
	if (!data->pdata) {
		dev_err(&data->client->dev, "Failed to allocate pdata\n");
		return -ENOMEM;
	}

	/* Set default parameters */
	data->pdata->irqflags = IRQF_TRIGGER_LOW;

	return 0;
}

static int ts_initialize_ts_input_device(struct ts_data *data)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	int error;
	int i,j;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = "atmel_ts_key";
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;  //vid: 03eb pid: 8c25
	input_dev->dev.parent = dev;
	input_dev->open = ts_input_open;
	input_dev->close = ts_input_close;

	__set_bit(EV_KEY, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, MSC_SCAN);

	/* For TS key array */
	data->key_status = 0;
	/* For Key register */
	if (data->pdata->keymap) {
		for (i = 0; i < NUM_KEY_TYPE; i++) {
			for (j = 0; j < data->pdata->num_keys[i]; j++) {
				set_bit(data->pdata->keymap[i][j], input_dev->keybit);
				input_set_capability(input_dev, EV_KEY,
							 data->pdata->keymap[i][j]);
			}
		}
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static bool is_need_to_update_fw(struct ts_data *data)
{
	struct ts_info *info = data->info;
	unsigned int version, build;
	int rc;
	const char *fw_version= data->pdata->fw_version;
	struct device *dev = &data->client->dev;
	unsigned int family_id, variant_id, version_h, version_l;

	if (fw_version) {
		rc = sscanf(fw_version, "%2x%2x", &version, &build);
		if (rc != 2) {
			dev_err(dev, "Can't get the fw version from DTS\n");
			return false;
		}
	} else {
		rc = sscanf(data->pdata->fw_name,
				"Letv_max1_tk_%02X_%02X_%u.%u_%02X.fw",
				&family_id, &variant_id, &version_h,
				&version_l, &build);
		if (rc != 5) {
			dev_err(dev, "Can't get the fw version from DTS\n");
			return false;
		}
		version = ((version_h & 0x0F) << 4) | (version_l & 0x0F);
	}

	if (!info) {
		dev_err(dev, "ts_info is NULL\n");
		return false;
	}

	dev_info(dev, "%s, info version: %02X, info build: %02X\n",
			__func__, info->version, info->build);
	dev_info(dev, "%s, image version: %02X, image build: %02X\n",
			__func__, version, build);

	if (info->version == version && info->build == build) {
		dev_info(dev, "The same fw version\n");
		return false;
	}

	if (info->version < version) {
		dev_info(dev, "TK: version: need to update fw\n");
		return true;
	} else if (info->version == version && info->build < build) {
		dev_info(dev, "TK: build: need to update fw\n");
		return true;
	}

	return false;
}

static int ts_update_fw(struct ts_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	dev_info(dev, "%s\n", __func__);

	if (!data->pdata->fw_name)
		return -EINVAL;

	error = ts_update_file_name(dev, &data->fw_name, data->pdata->fw_name,
			strlen(data->pdata->fw_name), false);
	if (error)
		return error;

	//lock it for disable outside access
	mutex_lock(&data->access_mutex);
	error = ts_load_fw(dev);
	mutex_unlock(&data->access_mutex);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		return error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		data->suspended = false;

		error = ts_initialize(data);
		if (error) {
			dev_err(dev,
				"ts_initialze failed after fw update(%d)\n",
				error);
			return error;
		}
	}

	return 0;
}

static int ts_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct ts_data *data;
	int error;

	dev_info(&client->dev, "%s: driver version 0x%x\n",
			__func__,DRIVER_VERSION);

	error = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE);
	if (!error) {
		dev_err(&client->dev, "%s adapter not supported\n",
			dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}
#if !defined(CONFIG_TS_EXTERNAL_TRIGGER_IRQ_WORKQUEUE)
	if (!client->irq) {
		dev_err(&client->dev, "please assign the irq to this device\n");
		return -EINVAL;
	}
#endif
	data = kzalloc(sizeof(struct ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	data->client = client;
	data->irq = client->irq;
	data->addr = client->addr;
	data->cmd_tab = ts_command_list;
	i2c_set_clientdata(client, data);

#if defined(CONFIG_TS_I2C_DMA)
	data->addr |= I2C_RS_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG;
	mutex_init(&data->dma_access_mutex);
	data->i2c_dma_va = (u8 *)dma_alloc_coherent(NULL, PAGE_SIZE * 2, &data->i2c_dma_pa, GFP_KERNEL);
	if (!data->i2c_dma_va)	{
		error = -ENOMEM;
		dev_err(&client->dev, "Allocate DMA I2C Buffer failed!\n");
		goto err_free_mem;
	}
#endif

#ifdef CONFIG_OF
	if (!data->pdata && client->dev.of_node)
		data->pdata = ts_parse_dt(client);
#endif
	if (!data->pdata) {
		error = ts_handle_pdata(data);
		if (error)
			goto err_free_mem;
	}

	dev_info(&client->dev, "ts irq %d\n",
		data->irq);

	init_completion(&data->bl_completion);
	init_completion(&data->reset_completion);
	mutex_init(&data->debug_msg_lock);
	mutex_init(&data->access_mutex);
	atomic_set(&data->depth,1);

#if defined(CONFIG_TS_IRQ_WORKQUEUE)
	init_waitqueue_head(&data->wait);
	data->irq_tsk = kthread_run(ts_process_message_thread, data,
						"Atmel_ts_ts");
	if (!data->irq_tsk) {
		dev_err(&client->dev, "Error %d Can't create irq thread\n",
			error);
		error = -ESRCH;
		goto err_free_pdata;
	}

#	if !defined(CONFIG_TS_EXTERNAL_TRIGGER_IRQ_WORKQUEUE)
	error = request_irq(client->irq, ts_interrupt_pulse_workqueue,
			data->pdata->irqflags /*IRQF_TRIGGER_LOW*/,
			client->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_irq_workqueue;
	}
#	endif
#else

	error = request_threaded_irq(data->irq, NULL, ts_interrupt,
					 IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					 client->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_pdata;
	}

#endif

	ts_probe_regulators(data);
#if defined(CONFIG_TS_EXTERNAL_TRIGGER_IRQ_WORKQUEUE)
	ts_g_data = data;
#endif
	error = ts_initialize(data);
	if (error)
		goto err_free_irq;

	/* Check if fw update is needed */
	if (data->in_bootloader || is_need_to_update_fw(data))
		ts_update_fw(data);

	error = sysfs_create_group(&client->dev.kobj, &ts_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating sysfs group\n",
			error);
		goto err_free_object;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRWXUGO /*S_IRUGO | S_IWUSR*/;
	data->mem_access_attr.read = ts_mem_access_read;
	data->mem_access_attr.write = ts_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	if (sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr) < 0) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}

#if defined(CONFIG_FB_PM)
	data->fb_notif.notifier_call = fb_notifier_callback;
	error = fb_register_client(&data->fb_notif);
	if (error) {
		dev_err(&client->dev,
			"Unable to register fb_notifier: %d\n",
			error);
		goto err_remove_mem_access_attr;
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = ts_early_suspend;
	data->early_suspend.resume = ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	dev_notice(&client->dev, "Ts probe finished\n");

	return 0;

#if defined(CONFIG_FB_PM)
err_remove_mem_access_attr:
	sysfs_remove_bin_file(&client->dev.kobj,
				  &data->mem_access_attr);
#endif
err_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &ts_attr_group);
err_free_object:
	ts_free_object_table(data);
err_free_irq:
	board_free_irq(data->pdata,data->irq, data);
#if defined(CONFIG_TS_IRQ_WORKQUEUE)
#	if !defined(CONFIG_TS_EXTERNAL_TRIGGER_IRQ_WORKQUEUE)
err_free_irq_workqueue:
#	endif
	kthread_stop(data->irq_tsk);
#endif
err_free_pdata:
	mutex_destroy(&data->access_mutex);
	mutex_destroy(&data->debug_msg_lock);
	if (!dev_get_platdata(&data->client->dev))
		devm_kfree(&client->dev,data->pdata);
err_free_mem:
#if defined(CONFIG_TS_I2C_DMA)
	dma_free_coherent(NULL, PAGE_SIZE * 2, data->i2c_dma_va, data->i2c_dma_pa);
#endif
	kfree(data);
	return error;
}

static int ts_remove(struct i2c_client *client)
{
	struct ts_data *data = i2c_get_clientdata(client);

#if defined(CONFIG_FB_PM)
	fb_unregister_client(&data->fb_notif);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif

#if defined(CONFIG_TS_IRQ_WORKQUEUE)
	kthread_stop(data->irq_tsk);
#endif

	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&client->dev.kobj,
					  &data->mem_access_attr);

	sysfs_remove_group(&client->dev.kobj, &ts_attr_group);
#if defined(CONFIG_TS_I2C_DMA)
	dma_free_coherent(NULL, PAGE_SIZE * 2, data->i2c_dma_va, data->i2c_dma_pa);
#endif

	board_free_irq(data->pdata,data->irq, data);
#if defined(CONFIG_TS_EXTERNAL_TRIGGER_IRQ_WORKQUEUE)
	ts_g_data = NULL;
#endif
	board_gpio_deinit(data->pdata);
	regulator_put(data->reg_avdd);
	regulator_put(data->reg_vdd);
	ts_free_object_table(data);
	mutex_destroy(&data->access_mutex);
	mutex_destroy(&data->debug_msg_lock);
#if defined(CONFIG_OF)
	ts_free_dt(data);
#endif
	if (!dev_get_platdata(&data->client->dev))
		devm_kfree(&client->dev,data->pdata);
	kfree(data);

	return 0;
}

#if defined(CONFIG_PM_SLEEP)
static int ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ts_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	dev_info(dev, "ts_suspend\n");

	if (!input_dev)  //maybe bootup in bootloader mode
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		ts_stop(data,true);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ts_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	dev_info(dev, "ts_resume\n");

	if (!input_dev)  //maybe bootup in bootloader mode
		return 0;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		ts_start(data,true);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

#if defined(CONFIG_FB_PM)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ts_data *ts =
		container_of(self, struct ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
				ts && ts->client) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK) {
				if (ts_resume(&ts->client->dev) != 0)
					dev_err(&ts->client->dev, "%s: failed\n", __func__);
			}else if (*blank == FB_BLANK_POWERDOWN) {
				if (ts_suspend(&ts->client->dev) != 0)
					dev_err(&ts->client->dev, "%s: failed\n", __func__);
			}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ts_early_suspend(struct early_suspend *es)
{
	struct ts_data *ts;
	ts = container_of(es, struct ts_data, early_suspend);

	if (ts_suspend(&ts->client->dev) != 0)
		dev_err(&ts->client->dev, "%s: failed\n", __func__);
}

static void ts_late_resume(struct early_suspend *es)
{
	struct ts_data *ts;
	ts = container_of(es, struct ts_data, early_suspend);

	if (ts_resume(&ts->client->dev) != 0)
		dev_err(&ts->client->dev, "%s: failed\n", __func__);
}
#else
static SIMPLE_DEV_PM_OPS(ts_pm_ops, ts_suspend, ts_resume);
#endif
#endif

static void ts_shutdown(struct i2c_client *client)
{
	struct ts_data *data = i2c_get_clientdata(client);

	if(atomic_read(&data->depth)) {
		board_disable_irq(data->pdata,data->irq);
		atomic_dec(&data->depth);
	}
}

static const struct i2c_device_id ts_id[] = {
	{ "atmel_ts_key", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ts_id);

#if defined(CONFIG_TS_EXTERNAL_MODULE)
#include "atmel_ts_mtk_interface.if"
#else
static struct i2c_driver ts_driver = {
	.driver = {
		.name	= "atmel_ts_key",
		.owner	= THIS_MODULE,
#if !(defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_FB_PM))
		.pm	= &ts_pm_ops,
#endif
	},
	.probe		= ts_probe,
	.remove		= ts_remove,
	.shutdown	= ts_shutdown,
	.id_table	= ts_id,
};

static int __init ts_init(void)
{
	return i2c_add_driver(&ts_driver);
}

static void __exit ts_exit(void)
{
	i2c_del_driver(&ts_driver);
}

module_init(ts_init);
module_exit(ts_exit);
#endif

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");

