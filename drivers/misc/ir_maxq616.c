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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/kobject.h>

struct msm8994_ir {
	struct device *dev;
	unsigned int en_gpio;
	struct pinctrl *ir_uart_pins;
	int power_on;
};

static struct msm8994_ir *irdata = NULL;
static struct kobject *remote_kobj = NULL;

static ssize_t name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "ir_remote");
}

static ssize_t enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", irdata->power_on);
}

static ssize_t enable_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int enable = 0;
	sscanf(buf, "%d", &enable);

	if (enable == 0) {
		if (irdata->power_on) {
			gpio_set_value(irdata->en_gpio, 0);
			irdata->power_on = 0;
		}
	} else {
		if (irdata->power_on == 0) {
			gpio_set_value(irdata->en_gpio, 1);
			msleep(1500);
			irdata->power_on = 1;
		}
	}

	return count;
}


static struct kobj_attribute enable_attr = {
	.attr	= {
		.name = __stringify(enable),
		.mode = 0666,
	},
	.show	= enable_show,
	.store	= enable_store,
};

static struct kobj_attribute name_attr = {
	.attr	= {
		.name = __stringify(name),
		.mode = 0444,
	},
	.show	= name_show,
	.store	= NULL,
};

static struct attribute * g[] = {
	&name_attr.attr,
	&enable_attr.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int msm8994_ir_suspend(struct device *dev)
{
	struct pinctrl_state *set_state;
	struct msm8994_ir *ir = dev_get_drvdata(dev);

	if (ir->ir_uart_pins) {
		set_state = pinctrl_lookup_state(ir->ir_uart_pins,
				"ir_sleep");
		if (IS_ERR(set_state))
			dev_err(dev, "IR: cannot get ir pinctrl active state\n");
		else
			pinctrl_select_state(ir->ir_uart_pins, set_state);
	}

	gpio_set_value(irdata->en_gpio, 0);
	dev_info(dev, "IR: msm8994 get into deep sleep\n");

	return 0;
}

static int msm8994_ir_resume(struct device *dev)
{
	struct pinctrl_state *set_state;
	struct msm8994_ir *ir = dev_get_drvdata(dev);

	if (ir->ir_uart_pins) {
		set_state = pinctrl_lookup_state(ir->ir_uart_pins,
				"ir_active");
		if (IS_ERR(set_state))
			dev_err(dev, "IR: cannot get ir pinctrl active state\n");
		else
			pinctrl_select_state(ir->ir_uart_pins, set_state);
	}
	gpio_set_value(irdata->en_gpio, 1);
	dev_info(dev, "IR: msm8994 get into active\n");

	return 0;
}

static int msm8994_ir_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct msm8994_ir *ir;
	int ret;
	struct pinctrl_state *set_state;

	ir = devm_kzalloc(&pdev->dev, sizeof(*ir), GFP_KERNEL);
	if (!ir) {
		dev_err(&pdev->dev, "fail to alloc msm8994 ir dev\n");
		return -ENOMEM;
	}

	irdata = ir;
	ir->dev = &pdev->dev;
	platform_set_drvdata(pdev, ir);

	/* Get pinctrl if target uses pinctrl */
	ir->ir_uart_pins = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(ir->ir_uart_pins)) {
		dev_err(&pdev->dev, "IR: can't get pinctrl state\n");
		ir->ir_uart_pins = NULL;
	}

	if (ir->ir_uart_pins) {
		set_state = pinctrl_lookup_state(ir->ir_uart_pins,
				"ir_active");
		if (IS_ERR(set_state))
			dev_err(&pdev->dev, "IR: cannot get ir pinctrl active state\n");
		else
			pinctrl_select_state(ir->ir_uart_pins, set_state);
	}

	ret = of_get_named_gpio(node, "qcom,ir-en-gpio", 0);
	if (!gpio_is_valid(ret)) {
		dev_err(ir->dev, "unable to get valid gpio\n");
		goto err_free;
	}
	ir->en_gpio = ret;

	ret = gpio_request(ir->en_gpio, "ir_en_gpio");
	if (ret) {
		dev_err(ir->dev, "unable to request ir_en_gpio gpio\n");
		goto err_gpio;
	}

	ret = gpio_direction_output(ir->en_gpio, 0);
	if (ret) {
		dev_err(ir->dev, "unable to set ir_en_gpio gpio\n");
		goto err_gpio;
	}

	remote_kobj = kobject_create_and_add("remote", NULL);
	if (!remote_kobj) {
		dev_err(ir->dev, "unable to create and add kobject\n");
		goto err_gpio;
	}

	ret = sysfs_create_group(remote_kobj, &attr_group);
	if (ret) {
		dev_err(ir->dev, "unable to create group attr\n");
		goto err_kobj;
	}

	return 0;

err_kobj:
	kobject_put(remote_kobj);
err_gpio:
	gpio_free(irdata->en_gpio);
err_free:
	devm_kfree(&pdev->dev, ir);
	irdata = NULL;

	return -EINVAL;
}

static int msm8994_ir_remove(struct platform_device *pdev)
{
	struct msm8994_ir *ir = irdata;
	sysfs_remove_group(remote_kobj, &attr_group);
	kobject_put(remote_kobj);
	gpio_free(ir->en_gpio);
	devm_kfree(&pdev->dev, ir);
	if (ir->ir_uart_pins)
		devm_pinctrl_put(ir->ir_uart_pins);
	irdata = NULL;
	return 0;
}

static SIMPLE_DEV_PM_OPS(ir_pm_ops, msm8994_ir_suspend, msm8994_ir_resume);

static struct of_device_id of_match_table[] = {
	{ .compatible = "qcom,ir-remote", },
	{ },
};

static struct platform_driver msm8994_ir_driver = {
	.driver         = {
		.name   = "ir-remote-driver",
		.of_match_table = of_match_table,
		.pm = &ir_pm_ops,
	},
	.probe          = msm8994_ir_probe,
	.remove		= msm8994_ir_remove,
};

module_platform_driver(msm8994_ir_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM8994 IR remote driver");
