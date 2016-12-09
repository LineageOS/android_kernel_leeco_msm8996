/*
 * TI LMU(Lighting Management Unit) Hardware Fault Monitoring Driver
 *
 * Copyright 2015 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hwmon.h>
#include <linux/kernel.h>
#include <linux/mfd/ti-lmu.h>
#include <linux/mfd/ti-lmu-register.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#define LMU_BANK_MASK(n)		(~BIT(n) & 0x07)
#define LMU_BL_MAX_CHANNELS		3
#define LMU_DEFAULT_BANK		0
#define LMU_DELAY_STARTUP		500
#define LMU_DELAY_FEEDBACK		5
#define LMU_ENABLE_FEEDBACK		(BIT(0) | BIT(1) | BIT(2))
#define LMU_MAX_BRIGHTNESS		0xFF
#define LMU_NO_RAMP			0

enum ti_lmu_hwmon_id {
	LMU_HWMON_OPEN,
	LMU_HWMON_SHORT,
};

/**
 * struct ti_lmu_reg
 *
 * @monitor:		Enable monitoring register
 * @bank:		Bank configuration register
 * @ramp:		Ramp(speed) configuration register
 * @imax:		Current limit setting register
 * @feedback:		Feedback enable register
 * @brightness:		Brightness register
 * @enable:		Bank enable register
 * @open_fault:		Detect opened circuit status register
 * @short_fault:	Detect shorted circuit status register
 *
 * To detect hardware fault, several registers are used.
 * Device specific register addresses are configured in this structure.
 */
struct ti_lmu_reg {
	u8 monitor;
	u8 bank;
	u8 ramp;
	u8 imax;
	u8 feedback;
	u8 brightness;
	u8 enable;
	u8 open_fault;
	u8 short_fault;
};

struct ti_lmu_hwmon {
	struct ti_lmu *lmu;
	struct device *dev;	/* hwmon dev */
	const struct ti_lmu_reg *regs;
};

struct ti_lmu_hwmon_data {
	const char *name;
	const struct attribute_group **groups;
	const struct ti_lmu_reg *regs;
};

static void ti_lmu_hwmon_reset_device(struct ti_lmu_hwmon *hwmon)
{
	unsigned int en_gpio = hwmon->lmu->en_gpio;

	/* POR(power on reset) by enable pin control */
	gpio_set_value(en_gpio, 0);
	msleep(LMU_DELAY_STARTUP);

	gpio_set_value(en_gpio, 1);
	msleep(LMU_DELAY_STARTUP);
}

static int ti_lmu_hwmon_enable_monitoring(struct ti_lmu_hwmon *hwmon,
					  enum ti_lmu_hwmon_id id)
{
	struct ti_lmu *lmu = hwmon->lmu;
	u8 reg = hwmon->regs->monitor;

	if (id == LMU_HWMON_OPEN)
		return ti_lmu_write_byte(lmu, reg, BIT(0));
	else if (id == LMU_HWMON_SHORT)
		return ti_lmu_write_byte(lmu, reg, BIT(1));
	else
		return -EINVAL;
}

static int ti_lmu_hwmon_assign_bank(struct ti_lmu_hwmon *hwmon, u8 val)
{
	return ti_lmu_write_byte(hwmon->lmu, hwmon->regs->bank, val);
}

static int ti_lmu_hwmon_channel_config(struct ti_lmu_hwmon *hwmon)
{
	struct ti_lmu *lmu = hwmon->lmu;
	const struct ti_lmu_reg *reg = hwmon->regs;
	int ret;

	/* Set ramp time to the fatest setting */
	ret = ti_lmu_write_byte(lmu, reg->ramp, LMU_NO_RAMP);
	if (ret)
		return ret;

	/* Set max current to 20mA */
	ret = ti_lmu_write_byte(lmu, reg->imax, LMU_IMAX_20mA);
	if (ret)
		return ret;

	/* Enable feedback */
	ret = ti_lmu_write_byte(lmu, reg->feedback, LMU_ENABLE_FEEDBACK);
	if (ret)
		return ret;

	/* Set max brightness */
	ret = ti_lmu_write_byte(lmu, reg->brightness, LMU_MAX_BRIGHTNESS);
	if (ret)
		return ret;

	/* Enable a bank */
	ret = ti_lmu_write_byte(lmu, reg->enable, 1);
	if (ret)
		return ret;

	/* Wait until device completes fault detection */
	msleep(LMU_DELAY_FEEDBACK);

	return 0;
}

static int ti_lmu_hwmon_get_open_fault_result(struct ti_lmu_hwmon *hwmon,
					      char *result)
{
	int ret, channel, len, offset = 0;
	u8 status = 0;

	ret = ti_lmu_read_byte(hwmon->lmu, hwmon->regs->open_fault, &status);
	if (ret)
		return ret;

	for (channel = 0; channel < LMU_BL_MAX_CHANNELS; channel++) {
		if (BIT(channel) & status)
			len = sprintf(&result[offset], "Channel %d is opened\n",
				      channel);
		else
			len = sprintf(&result[offset], "Channel %d works\n",
				      channel);

		offset += len;
	}

	return 0;
}

static ssize_t ti_lmu_hwmon_get_short_fault_result(struct ti_lmu_hwmon *hwmon,
						   char *buf, int channel)
{
	int ret;
	u8 status = 0;

	ret = ti_lmu_read_byte(hwmon->lmu, hwmon->regs->short_fault, &status);
	if (ret)
		return ret;

	if (BIT(channel) & status)
		return sprintf(buf, "Channel %d is shorted\n", channel);
	else
		return sprintf(buf, "Channel %d works\n", channel);
}

static int ti_lmu_hwmon_disable_all_banks(struct ti_lmu_hwmon *hwmon)
{
	return ti_lmu_write_byte(hwmon->lmu, hwmon->regs->enable, 0);
}

static int ti_lmu_hwmon_notifier_call_chain(struct ti_lmu_hwmon *hwmon)
{
	int ret;

	ti_lmu_hwmon_reset_device(hwmon);

	ret = blocking_notifier_call_chain(&hwmon->lmu->notifier,
					   LMU_EVENT_HWMON_DONE, NULL);
	if (ret == NOTIFY_OK || ret == NOTIFY_DONE)
		return 0;
	else
		return -EINVAL;
}

static ssize_t ti_lmu_hwmon_open_fault_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buffer)
{
	struct ti_lmu_hwmon *hwmon = dev_get_drvdata(dev);
	char result[100];
	int ret;

	/* Device should be reset prior to fault detection */
	ti_lmu_hwmon_reset_device(hwmon);

	ret = ti_lmu_hwmon_enable_monitoring(hwmon, LMU_HWMON_OPEN);
	if (ret)
		return ret;

	ret = ti_lmu_hwmon_assign_bank(hwmon, LMU_DEFAULT_BANK);
	if (ret)
		return ret;

	ret = ti_lmu_hwmon_channel_config(hwmon);
	if (ret)
		return ret;

	memset(result, 0, sizeof(result));
	ret = ti_lmu_hwmon_get_open_fault_result(hwmon, result);
	if (ret)
		return ret;

	/* Notify an event */
	ret = ti_lmu_hwmon_notifier_call_chain(hwmon);
	if (ret)
		dev_warn(dev, "Notify hwmon err\n");

	return snprintf(buffer, PAGE_SIZE, "%s\n", result);
}

static ssize_t ti_lmu_hwmon_short_fault_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buffer)
{
	struct ti_lmu_hwmon *hwmon = dev_get_drvdata(dev);
	int ret, i, len, offset = 0;
	char result[100];

	/* Device should be reset prior to fault detection */
	ti_lmu_hwmon_reset_device(hwmon);

	ret = ti_lmu_hwmon_enable_monitoring(hwmon, LMU_HWMON_SHORT);
	if (ret)
		return ret;

	memset(result, 0, sizeof(result));

	/* Shorted circuit detection is done by checking the bank one by one */
	for (i = 0; i < LMU_BL_MAX_CHANNELS; i++) {
		ret = ti_lmu_hwmon_assign_bank(hwmon, LMU_BANK_MASK(i));
		if (ret)
			return ret;

		ret = ti_lmu_hwmon_channel_config(hwmon);
		if (ret)
			return ret;

		len = ti_lmu_hwmon_get_short_fault_result(hwmon,
							  &result[offset], i);
		if (len < 0)
			return len;

		offset += len;

		ret = ti_lmu_hwmon_disable_all_banks(hwmon);
		if (ret)
			return ret;
	}

	/* Notify an event */
	ret = ti_lmu_hwmon_notifier_call_chain(hwmon);
	if (ret)
		dev_warn(dev, "Notify hwmon err\n");

	return snprintf(buffer, PAGE_SIZE, "%s\n", result);
}

static DEVICE_ATTR(open_fault, S_IRUGO, ti_lmu_hwmon_open_fault_show, NULL);
static DEVICE_ATTR(short_fault, S_IRUGO, ti_lmu_hwmon_short_fault_show, NULL);

static struct attribute *ti_lmu_hwmon_attrs[] = {
	&dev_attr_open_fault.attr,
	&dev_attr_short_fault.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ti_lmu_hwmon);

/*
 * Operations are dependent on the device.
 * Device registers configuration is required.
 */
static const struct ti_lmu_reg lm3633_regs = {
	.monitor	= LM3633_REG_MONITOR_ENABLE,
	.bank		= LM3633_REG_HVLED_OUTPUT_CFG,
	.ramp		= LM3633_REG_BL0_RAMPUP,
	.imax		= LM3633_REG_IMAX_HVLED_A,
	.feedback	= LM3633_REG_BL_FEEDBACK_ENABLE,
	.brightness	= LM3633_REG_BRT_HVLED_A_MSB,
	.enable		= LM3633_REG_ENABLE,
	.open_fault	= LM3633_REG_BL_OPEN_FAULT_STATUS,
	.short_fault	= LM3633_REG_BL_SHORT_FAULT_STATUS,
};

static const struct ti_lmu_reg lm3697_regs = {
	.monitor	= LM3697_REG_MONITOR_ENABLE,
	.bank		= LM3697_REG_HVLED_OUTPUT_CFG,
	.ramp		= LM3697_REG_BL0_RAMPUP,
	.imax		= LM3697_REG_IMAX_A,
	.feedback	= LM3697_REG_FEEDBACK_ENABLE,
	.brightness	= LM3697_REG_BRT_A_MSB,
	.enable		= LM3697_REG_ENABLE,
	.open_fault	= LM3697_REG_OPEN_FAULT_STATUS,
	.short_fault	= LM3697_REG_SHORT_FAULT_STATUS,
};

static const struct ti_lmu_hwmon_data lm3633_hwmon_data = {
	.name = "lm3633_fault_status",
	.groups = ti_lmu_hwmon_groups,
	.regs = &lm3633_regs,
};

static const struct ti_lmu_hwmon_data lm3697_hwmon_data = {
	.name = "lm3697_fault_status",
	.groups = ti_lmu_hwmon_groups,
	.regs = &lm3697_regs,
};

static const struct of_device_id ti_lmu_hwmon_of_match[] = {
	{ .compatible = "ti,lm3633-hwmon", .data = &lm3633_hwmon_data },
	{ .compatible = "ti,lm3697-hwmon", .data = &lm3697_hwmon_data },
	{ }
};
MODULE_DEVICE_TABLE(of, ti_lmu_hwmon_of_match);

static int ti_lmu_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ti_lmu *lmu = dev_get_drvdata(dev->parent);
	const struct of_device_id *match;
	struct ti_lmu_hwmon *hwmon;
	const struct ti_lmu_hwmon_data *data;

	match = of_match_device(ti_lmu_hwmon_of_match, dev);
	if (!match)
		return -ENODEV;

	/* To monitor hardware fault, enable pin control should be required. */
	if (!gpio_is_valid(lmu->en_gpio))
		return -EINVAL;

	hwmon = devm_kzalloc(dev, sizeof(*hwmon), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	/*
	 * Get device specific data(name, groups and registers) from
	 * of_match table.
	 */
	data = (struct ti_lmu_hwmon_data *)match->data;
	hwmon->lmu = lmu;
	hwmon->regs = data->regs;

	hwmon->dev = devm_hwmon_device_register_with_groups(dev, data->name,
							hwmon, data->groups);

	return PTR_ERR_OR_ZERO(hwmon->dev);
}

static struct platform_driver ti_lmu_hwmon_driver = {
	.probe = ti_lmu_hwmon_probe,
	.driver = {
		.name = "ti-lmu-hwmon",
		.of_match_table = ti_lmu_hwmon_of_match,
	},
};

module_platform_driver(ti_lmu_hwmon_driver);

MODULE_DESCRIPTION("TI LMU Hardware Fault Monitoring Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ti-lmu-hwmon");
