/*
 * Copyright (C) 2016 Zeusis.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.
 *
 */
#define DEBUG
#define pr_fmt(fmt) "[%s:%d] " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#endif
//#include <linux/tusb320.h>
#if defined(CONFIG_LETV_HW_DEV_DCT)
#include <linux/hw_dev_dec.h>
#endif
#include "tusb320.h"

#ifdef DEBUG
#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) \
	printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_debug(fmt, ...) \
	printk(pr_fmt(fmt), ##__VA_ARGS__)
#endif
#endif

#define HD_SWITCH_DBG (1)
#define TUSB320_VID (0x30)

static struct tusb320_dev *g_tusb_dev;
//static int Try_Sink_State = Try_Sink_Idle_DRP;

//extern bool letv_typec_plug_state;
//static bool cyccg_usb_audio_insert = false;
#ifdef HD_SWITCH_DBG
static int switch_cnt = 0;
#endif
static bool tusb320_analog_headset_plugin = false;
static bool i2c_op_fail = false;
static struct mutex typec_headset_lock;
static struct mutex tusb320_i2c_lock;
extern bool typec_set_cc_state;
extern void cclogic_updata_port_state(int state);
extern void cclogic_updata_port_polarity(int polarity);
extern int msm_usb_vbus_set(void *_mdwc, bool on, bool ext_call);
extern int wcd_mbhc_plug_detect(void);
extern void cclogic_set_audio_mode_register(void (*func)(bool));
extern int cclogic_set_vbus(bool on);
extern void cclogic_set_typec_headset_with_analog(int val);
extern bool serial_hw_output_enable(void);
static bool always_uart_debug = false;

struct tusb320_dev	{
//	struct delayed_work trysink_check_work1;
//  struct delayed_work trysink_check_work2;
	struct delayed_work check_work;
	struct i2c_client	*client;
	struct pinctrl *pinctrl_int;
	struct pinctrl_state *intr_active;
	struct pinctrl *pinctrl_switch;
	struct pinctrl_state *switch_active;
	struct pinctrl *pinctrl_ccpwr;
	struct pinctrl_state *ccpwr_active;
	struct miscdevice	tusb320_device;
	struct mutex i2c_lock;
	unsigned int		irq_gpio;
	unsigned int 		switch_gpio1;
	unsigned int 		switch_gpio2;
	unsigned int 		cc1_pwr_gpio;
	unsigned int 		cc2_pwr_gpio;
};

extern int pi5usb_set_msm_usb_host_mode(bool mode);

static int tusb320_i2c_rxdata(struct i2c_client *client, char *rxdata, int length)
{
	int ret = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata + 1,
		},
	};

	if (i2c_transfer(client->adapter, msgs, 2) != 2) {
		ret = -EIO;
		pr_err("%s: i2c read error: %d\n", __func__, ret);
	}

	return ret;
}

static int tusb320_i2c_txdata(struct i2c_client *client, char *txdata, int length)
{
	int ret = 0;

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	if (i2c_transfer(client->adapter, msg, 1) != 1) {
		ret = -EIO;
		pr_err("%s: i2c write error: %d\n", __func__, ret);
	}

	return ret;
}

static int tusb320_write_reg(struct i2c_client *client, u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	mutex_lock(&tusb320_i2c_lock);

	buf[0] = addr;
	buf[1] = para;
	ret = tusb320_i2c_txdata(client, buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		i2c_op_fail = true;
		mutex_unlock(&tusb320_i2c_lock);
		return -1;
	}

	mutex_unlock(&tusb320_i2c_lock);

	return 0;
}

static int tusb320_read_reg(struct i2c_client *client, u8 addr, u8 *pdata)
{
	int ret = -1;
	u8 buf[2] = {addr, 0};

	mutex_lock(&tusb320_i2c_lock);

	ret = tusb320_i2c_rxdata(client, buf, 1);
	if (ret < 0) {
		pr_err("read reg failed! %#x ret: %d", buf[0], ret);
		i2c_op_fail = true;
		mutex_unlock(&tusb320_i2c_lock);
		return -1;
	}
	*pdata = buf[1];

	mutex_unlock(&tusb320_i2c_lock);

	return 0;
}

/*This func is for headset switch AD mode*/
void tusb320_cclogic_set_audio_mode(bool mode)
{
	/*true for Analog mode and false for Digit mode*/
	//struct cyccg_platform_data *pdata = cyccg_pdata;
	struct tusb320_dev *tusb320_dev = g_tusb_dev;
	unsigned char reg_val;

	#ifdef HD_SWITCH_DBG
	switch_cnt++;
	#endif
	mutex_lock(&typec_headset_lock);
	#ifdef HD_SWITCH_DBG
	pr_info("tusb320 switch nyx , c:%d, m:%d\n", switch_cnt, mode);
	#endif

	if(true == always_uart_debug)
	{
		pr_err("Err: Serial HW-Output Enable!\n");
	}

	if (mode) {
		pr_info("tusb320 switch to audio mode nyx\n");
		msleep(50);
		pi5usb_set_msm_usb_host_mode(false);/*simulate plugout*/
		//msm_usb_vbus_on(NULL);
		//msm_usb_vbus_set(NULL, 1, true);
		gpio_set_value(tusb320_dev->switch_gpio2, 1);/*Headphone*/
		gpio_set_value(tusb320_dev->switch_gpio1, 0);/*MIC*/

		//check cc orientation
		tusb320_read_reg(tusb320_dev->client, TUSB320_REG_ATTACH_STATUS, &reg_val);
		if (reg_val & TUSB320_REG_STATUS_CC) {
			pr_info("tusb320 cc1_pwr_gpio 1\n");
			gpio_set_value(tusb320_dev->cc1_pwr_gpio, 1);
		} else  {
			pr_info("tusb320 cc2_pwr_gpio 1\n");
			gpio_set_value(tusb320_dev->cc2_pwr_gpio, 1);
		}

		//cyccg_pdata->mode = AUDIO_MODE;
		//if (!letv_typec_plug_state)
			//letv_typec_plug_state = true;
		//notify audio module for headset plug in
		if (!tusb320_analog_headset_plugin) {
			pr_info("audio headset plug in!!\n");
			tusb320_analog_headset_plugin = true;
			wcd_mbhc_plug_detect();
		}
		cclogic_set_typec_headset_with_analog(1);
		//cyccg_usb_audio_insert = true;
		/*The behavior above */
		cclogic_updata_port_state(3);/*"cc_state: audio"*/
	} else {
		pr_info("tusb320 switch to digital nyx\n");
		gpio_set_value(tusb320_dev->switch_gpio2, 0);/*USB*/
		gpio_set_value(tusb320_dev->switch_gpio1, 0);

		msleep(30);
		pi5usb_set_msm_usb_host_mode(true);
		gpio_set_value(tusb320_dev->cc1_pwr_gpio, 0);
		gpio_set_value(tusb320_dev->cc2_pwr_gpio, 0);
		//pr_info("tusb320 set usb to host mode\n");
		//pdata->mode = DFP_MODE;
		//notify audio module for headset plug out
		if (tusb320_analog_headset_plugin) {
			pr_info("audio headset plug out!!\n");
			tusb320_analog_headset_plugin = false;
			wcd_mbhc_plug_detect();
		}
		cclogic_set_typec_headset_with_analog(0);
		cclogic_updata_port_state(2);/*"cc_state: dfp"*/
	}
	mutex_unlock(&typec_headset_lock);
	return;
}
EXPORT_SYMBOL(tusb320_cclogic_set_audio_mode);

static int tusb320_is_present(struct i2c_client *client)
{
	int ret = 0;
	unsigned char reg_val;

	ret = tusb320_read_reg(client, TUSB320_REG_DEVICE_ID, &reg_val);
	pr_info("reg[0x%x] = 0x%02x.\n", TUSB320_REG_DEVICE_ID, reg_val);
	if(0 != ret)
	{
		return -ENODEV;
	}

	if(TUSB320_VID != reg_val)
	{
		return -ENODEV;
	}

	return ret;
}

static int tusb320_int_clear(void)
{
	struct tusb320_dev *tusb320_dev = g_tusb_dev;
	u8 reg_val;
	int ret;
	ret = tusb320_read_reg(tusb320_dev->client, TUSB320_REG_ATTACH_STATUS, &reg_val);
	if (ret < 0) {
		pr_err("%s: read error\n", __func__);
		return ret;
	}
	reg_val |= TUSB320_REG_STATUS_INT;
	ret = tusb320_write_reg(tusb320_dev->client, TUSB320_REG_ATTACH_STATUS, reg_val);
	if (ret < 0) {
		pr_err("%s: write error\n", __func__);
		return ret;
	}
	return ret;
}

static void tusb320_soft_reset(void)
{
	struct tusb320_dev *tusb320_dev = g_tusb_dev;
	u8 reg_val;
	int ret;
	pr_info("%s\n", __func__);
	ret = tusb320_read_reg(tusb320_dev->client, TUSB320_REG_MODE_SET, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_MODE_SET error\n", __func__);
	}
	reg_val |= TUSB320_REG_SET_SOFT_RESET;
	ret = tusb320_write_reg(tusb320_dev->client, TUSB320_REG_MODE_SET, reg_val);
	if (ret < 0) {
		pr_err("%s: write REG_MODE_SET error\n", __func__);
	}
}

static enum typec_current_mode tusb320_current_mode_detect(void)
{
	struct tusb320_dev *tusb320_dev = g_tusb_dev;
	u8 reg_val, mask_val;
	int ret;
	enum typec_current_mode current_mode = TYPEC_CURRENT_MODE_DEFAULT;
	ret = tusb320_read_reg(tusb320_dev->client, TUSB320_REG_CURRENT_MODE, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_CURRENT_MODE error\n", __func__);
		return current_mode;
	}
	pr_debug("%s: REG_CURRENT_MODE 08H is 0x%x\n", __func__, reg_val);
	mask_val = reg_val & TUSB320_REG_CUR_MODE_DETECT_MASK;
	switch (mask_val) {
	case TUSB320_REG_CUR_MODE_DETECT_DEFAULT:
		current_mode = TYPEC_CURRENT_MODE_DEFAULT;
		break;
	case TUSB320_REG_CUR_MODE_DETECT_MID:
		current_mode = TYPEC_CURRENT_MODE_MID;
		break;
	case TUSB320_REG_CUR_MODE_DETECT_HIGH:
		current_mode = TYPEC_CURRENT_MODE_HIGH;
		break;
	default:
		current_mode = TYPEC_CURRENT_MODE_UNSPPORTED;
	}
	pr_debug("%s: current mode is %d\n", __func__, current_mode);
	return current_mode;
}

static enum typec_attached_state tusb320_attatched_state_detect(void)
{
	struct tusb320_dev *tusb320_dev = g_tusb_dev;
	u8 reg_val, mask_val;
	int ret;
	enum typec_attached_state attached_state = TYPEC_NOT_ATTACHED;
	ret = tusb320_read_reg(tusb320_dev->client, TUSB320_REG_ATTACH_STATUS, &reg_val);
	if (ret < 0) {
		pr_err("%s: read REG_ATTACH_STATUS error\n", __func__);
		return attached_state;
	}
	pr_info("%s: REG_ATTACH_STATUS 09H is 0x%x\n", __func__, reg_val);
	mask_val = reg_val & TUSB320_REG_STATUS_MODE;
	switch (mask_val) {
	case TUSB320_REG_STATUS_AS_DFP:
		attached_state = TYPEC_ATTACHED_AS_DFP;
		break;
	case TUSB320_REG_STATUS_AS_UFP:
		attached_state = TYPEC_ATTACHED_AS_UFP;
		break;
	case TUSB320_REG_STATUS_TO_ACCESSORY:
		attached_state = TYPEC_ATTACHED_TO_ACCESSORY;
		break;
	default:
		attached_state = TYPEC_NOT_ATTACHED;
	}
	pr_info("%s: attached state is %d\n", __func__, attached_state);
	return attached_state;
}
static enum typec_accessory_state tusb320_accessory_state_detect(void)
{
	struct tusb320_dev *tusb320_dev = g_tusb_dev;
	u8 reg_val;
	int ret;
	enum typec_accessory_state accessory_state = TYPEC_NO_ACCESSORY_ATTACHED;
	ret = tusb320_read_reg(tusb320_dev->client, TUSB320_REG_CURRENT_MODE, &reg_val);
	if (ret < 0) {
		pr_err("%s: read TUSB320_REG_CURRENT_MODE error\n", __func__);
		return accessory_state;
	}
	pr_info("%s: TUSB320_REG_CURRENT_MODE 08H is 0x%x\n", __func__, reg_val);
	accessory_state = (reg_val & TUSB320_REG_ACCESSORY_STATUS)>>1;
	pr_info("%s: accessory state is %d\n", __func__, accessory_state);
	return accessory_state;
}

int tusb320_hw_init(struct i2c_client *client)
{
	int ret = 0;
	pr_info("%s starting.\n",__func__);
//	tusb320_int_clear();
	return ret;
}

static void tusb320_dump_regs(struct i2c_client *client)
{
	unsigned char i = 0, data = 0;

	for(i = 0; i <= 0xa; i++){
		tusb320_read_reg(client, i, &data);
		pr_debug("reg[0x%x] = 0x%02x.\n", i, data & 0xff);
	}

		tusb320_read_reg(client, 0x45, &data);
		pr_debug("reg[0x%x] = 0x%02x.\n", 0x45, data & 0xff);
}

#if defined(TUSB320_DEBUG)
/* sysfs interface */
static ssize_t tusb320_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	tusb320_dump_regs(client);
	return sprintf(buf,"%s\n","OK");
}

/* debug fs, "echo @1 > /sys/bus/i2c/devices/xxx/tusb320_debug" @1:debug_flag  */
static ssize_t tusb320_store(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data = 0x0;

	/*clear interrupt register test*/
	tusb320_read_reg(client, 0x03, &data);
	pr_debug("After clear interrupt REG[0x03] = 0x%02x.\n", data);

	return count;
}

static struct device_attribute tusb320_dev_attr =
	__ATTR(tusb320_debug, S_IRUGO | S_IWUGO, tusb320_show, tusb320_store);
#endif

#ifdef CONFIG_PM_SLEEP
static int tusb320_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct tusb320_dev *tusb320_dev = g_tusb_dev;
	enable_irq_wake(tusb320_dev->client->irq);
	return ret;
}
static int tusb320_pm_resume(struct device *dev)
{
	int ret = 0;
	struct tusb320_dev *tusb320_dev = g_tusb_dev;
	disable_irq_wake(tusb320_dev->client->irq);
	return ret;
}
static const struct dev_pm_ops tusb320_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tusb320_pm_suspend, tusb320_pm_resume)
};
#endif /* CONFIG_PM_SLEEP */

static irqreturn_t tusb320_irq_handler(int irq, void *data)
{
	struct tusb320_dev *tusb320_dev = data;

	pr_debug("tusb320 handle irq\n");

	schedule_delayed_work(&tusb320_dev->check_work, 0);

	return IRQ_HANDLED;
}


static void tusb320_status_check(struct work_struct *work)
{
	struct tusb320_dev *tusb320_dev = g_tusb_dev;
	int ret;
	enum typec_current_mode current_mode;
	enum typec_attached_state attached_state;
	enum typec_accessory_state accessory_state;
	u8 reg_val,mask_val;
	pr_info("tusb320_status_check-> start.\n");
	ret = tusb320_int_clear();
	current_mode = tusb320_current_mode_detect();
	attached_state = tusb320_attatched_state_detect();
	accessory_state = tusb320_accessory_state_detect();

	switch (attached_state) {
		case TYPEC_NOT_ATTACHED:
		//disconnect cable
		gpio_set_value(tusb320_dev->switch_gpio2, 0);//USB
		if(true == always_uart_debug)
		{
			gpio_set_value(tusb320_dev->switch_gpio1, 1);//UART
		}
		else
		{
			gpio_set_value(tusb320_dev->switch_gpio1, 0);//close UART > MIC
		}

		//disable Vbus first
		cclogic_set_vbus(0);
		gpio_set_value(tusb320_dev->cc1_pwr_gpio, 0);
		gpio_set_value(tusb320_dev->cc2_pwr_gpio, 0);
		//clear type-c connect status.
		cclogic_set_typec_headset_with_analog(-1);
		if (tusb320_analog_headset_plugin) {
			tusb320_analog_headset_plugin = false;
			pr_debug("%s, audio headset plug out!!\n", __func__);
			wcd_mbhc_plug_detect();
		}
		pi5usb_set_msm_usb_host_mode(false);/*if not simulate disconnect, open*/
		cclogic_updata_port_state(0);/*"cc_state: none"*/
		cclogic_updata_port_polarity(0); /* no typec usb connected*/
		if (typec_set_cc_state) {
			pr_info("%s: clear typec_set_cc_state!\n", __func__);
			typec_set_cc_state = false;
		}
		pr_info("tusb320_status_check TYPEC_NOT_ATTACHED.\n");
		break;

		case TYPEC_ATTACHED_AS_DFP:
		//connect DFP device
		gpio_set_value(tusb320_dev->switch_gpio1, 0);/*MIC*/
		cclogic_set_vbus(1);
		pi5usb_set_msm_usb_host_mode(true);
		cclogic_updata_port_state(2);/*"cc_state: dfp"*/
		pr_info("tusb320_status_check TYPEC_ATTACHED_AS_DFP.\n");
		//check cc orientation
		tusb320_read_reg(tusb320_dev->client, TUSB320_REG_ATTACH_STATUS, &reg_val);
		mask_val = (reg_val & TUSB320_REG_STATUS_CC) >> 5;
		if (0x01 == mask_val) {
			pr_info("tusb320 cc2 connected\n");
			cclogic_updata_port_polarity(2); /* CC2 connected*/
		} else  if(0x00 == mask_val){
			pr_info("tusb320 cc1 connected\n");
			cclogic_updata_port_polarity(1); /* CC1 connected*/
		} else  {
			pr_info("tusb320 cc pin polarity detection ERROR\n");
		}
		cclogic_set_typec_headset_with_analog(0);
		break;

		case TYPEC_ATTACHED_AS_UFP:
		//connect UFP device
		gpio_set_value(tusb320_dev->switch_gpio1, 1);/*UART*/
		cclogic_updata_port_state(1);/*"cc_state: ufp"*/
		pr_info("tusb320_status_check TYPEC_ATTACHED_AS_UFP.\n");
		break;

		case TYPEC_ATTACHED_TO_ACCESSORY:
		//connect accessory device
		if(accessory_state == TYPEC_AUDIO_ACCESSORY) {
		/*switch GPIO to select audio channel*/
		gpio_set_value(tusb320_dev->switch_gpio2, 1);/*Headphone*/
		gpio_set_value(tusb320_dev->switch_gpio1, 0);/*MIC*/
		cclogic_updata_port_state(3);/*"cc_state: audio"*/
		if (!tusb320_analog_headset_plugin) {
				pr_debug("%s, audio headset plug in!!\n", __func__);
				msleep(150);
				tusb320_analog_headset_plugin = true;
				wcd_mbhc_plug_detect();
			}
		}
		pr_info("tusb320_status_check TYPEC_ATTACHED_TO_ACCESSORY.\n");
		break;
	}
	pr_info("tusb320_status_check-> end.\n");

}

static ssize_t tusb320_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	int ret = 0;

	return ret;
}

static ssize_t tusb320_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	int ret = 0;

	return ret;
}

static int tusb320_dev_open(struct inode *inode, struct file *filp)
{
	struct tusb320_dev *tusb320_dev = container_of(filp->private_data,
						struct tusb320_dev,
						tusb320_device);

	filp->private_data = tusb320_dev;

	pr_debug("%d, %d\n", imajor(inode), iminor(inode));

	return 0;
}

static long  tusb320_dev_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	return 0;
}

static const struct file_operations tusb320_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= tusb320_dev_read,
	.write	= tusb320_dev_write,
	.open	= tusb320_dev_open,
	.unlocked_ioctl  = tusb320_dev_ioctl,
};

static int tusb320_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct tusb320_dev *tusb320_dev;
	int ret = 0;

	pr_debug("tusb320 probe start!\n");

	tusb320_dev = kzalloc(sizeof(struct tusb320_dev), GFP_KERNEL);
	if (tusb320_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_allocate_mem;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		goto  err_allocate_mem;
	}

	mutex_init(&tusb320_i2c_lock);

	ret = tusb320_is_present(client);
	if (0 != ret) {
		goto err_allocate_mem;
	}

	if(true == serial_hw_output_enable())
	{
		always_uart_debug = true;
		pr_err("Serial HW-Output Enable!!!!!!\n");
	}

	//Interrupt pin configurate
	tusb320_dev->pinctrl_int = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(tusb320_dev->pinctrl_int)) {
		pr_debug("tusb320 %s: Unable to get pinctrl handle\n", __func__);
	}
	tusb320_dev->intr_active = pinctrl_lookup_state(tusb320_dev->pinctrl_int,
			"m0_ccint_active");
	if (IS_ERR(tusb320_dev->intr_active)) {
		pr_debug("tusb320 %s: could not get intr_active pinstate\n", __func__);
		goto err_irq_gpio;
	}

	ret = pinctrl_select_state(tusb320_dev->pinctrl_int,
					tusb320_dev->intr_active);
	if (ret != 0) {
		pr_debug("tusb320 %s: Disable TLMM pins failed with %d\n",
			__func__, ret);
	}

	tusb320_dev->irq_gpio =
		of_get_named_gpio_flags(client->dev.of_node,
		"irq-gpio", 0, NULL);
	if (gpio_is_valid(tusb320_dev->irq_gpio)) {
		pr_debug("%s:irq gpio=%d\n", __func__, tusb320_dev->irq_gpio);
		ret = gpio_request(tusb320_dev->irq_gpio, "irq-gpio");
		if(ret) {
			pr_debug("typec_irq gpio_request failed!\n");
			goto err_irq_gpio;
		}
	} else {
		pr_debug("%s : irq_gpio failed!\n" , __func__);
		goto err_irq_gpio;
	}
	gpio_direction_input(tusb320_dev->irq_gpio);


	//Switch pin configurate
	tusb320_dev->pinctrl_switch = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(tusb320_dev->pinctrl_switch)) {
		pr_debug("tusb320 %s: Unable to get pinctrl_switch\n", __func__);
	}
	tusb320_dev->switch_active = pinctrl_lookup_state(tusb320_dev->pinctrl_switch,
					"m0_ccswitch_active");
	if (IS_ERR(tusb320_dev->switch_active)) {
		pr_debug("tusb320 %s: could not get switch_active pinstate\n", __func__);
		goto err_switch_gpio;
	}

	ret = pinctrl_select_state(tusb320_dev->pinctrl_switch,
					tusb320_dev->switch_active);
	if (ret != 0) {
		pr_debug("tusb320 %s: Disable switch pins failed with %d\n",
			__func__, ret);
	}

	tusb320_dev->switch_gpio1 =
		of_get_named_gpio_flags(client->dev.of_node,
		"switch_gpio1", 0, NULL);
	if (gpio_is_valid(tusb320_dev->switch_gpio1)) {
		pr_debug("%s:switch_gpio1=%d\n",__func__, tusb320_dev->switch_gpio1);
		ret = gpio_request(tusb320_dev->switch_gpio1, "switch_gpio1");
		if(ret) {
			pr_debug("switch_gpio1 gpio_request failed!\n");
			goto err_switch_gpio;
		}
	} else {
		pr_debug("%s : switch_gpio1 failed!\n" , __func__);
		goto err_switch_gpio;
	}

	tusb320_dev->switch_gpio2 =
		of_get_named_gpio_flags(client->dev.of_node,
		"switch_gpio2", 0, NULL);
	if (gpio_is_valid(tusb320_dev->switch_gpio2)) {
		pr_debug("%s:switch_gpio2=%d\n",__func__, tusb320_dev->switch_gpio2);
		ret = gpio_request(tusb320_dev->switch_gpio2, "switch_gpio2");
		if(ret) {
			pr_debug("switch_gpio2 gpio_request failed!\n");
			goto err_switch_gpio;
		}
	} else {
		pr_debug("%s : switch_gpio2 failed!\n" , __func__);
		goto err_switch_gpio;
	}

	if (tusb320_dev->switch_gpio1) {
		ret = gpio_direction_output(tusb320_dev->switch_gpio1, 0);
		if (ret < 0) {
			pr_err("%s : not able to set switch_gpio1 as output\n",
				 __func__);
			goto err_switch_gpio;
		}
	}

	if (tusb320_dev->switch_gpio2) {
		ret = gpio_direction_output(tusb320_dev->switch_gpio2, 0);
		if (ret < 0) {
			pr_err("%s : not able to set switch_gpio2 as output\n",
				 __func__);
			goto err_switch_gpio;
		}
	}

	/* init switch_gpio state */
	gpio_set_value(tusb320_dev->switch_gpio2, 0);/*USB*/  //dfl:0
	if(true == always_uart_debug)
	{
		gpio_set_value(tusb320_dev->switch_gpio1, 1);/*UART*/
	}
	else
	{
		gpio_set_value(tusb320_dev->switch_gpio1, 0);
	}

	//CC Power pin configurate
	tusb320_dev->pinctrl_ccpwr = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(tusb320_dev->pinctrl_ccpwr)) {
		pr_debug("tusb320 %s: Unable to get pinctrl_ccpwr\n", __func__);
	}
	tusb320_dev->ccpwr_active = pinctrl_lookup_state(tusb320_dev->pinctrl_ccpwr,
			"m0_ccpwr_active");
	if (IS_ERR(tusb320_dev->ccpwr_active)) {
		pr_debug("tusb320 %s: could not get ccpwr_active pinstate\n", __func__);
		goto err_cc_pwr_gpio;
	}

	ret = pinctrl_select_state(tusb320_dev->pinctrl_ccpwr,
					tusb320_dev->ccpwr_active);
	if (ret != 0) {
		pr_debug("tusb320 %s: Disable ccpwr pins failed with %d\n",
			__func__, ret);
	}

	tusb320_dev->cc1_pwr_gpio =
		of_get_named_gpio_flags(client->dev.of_node,
		"cc1_pwr_gpio", 0, NULL);
	if (gpio_is_valid(tusb320_dev->cc1_pwr_gpio)) {
		pr_debug("%s: cc1_pwr_gpio=%d\n",__func__, tusb320_dev->cc1_pwr_gpio);
		ret = gpio_request(tusb320_dev->cc1_pwr_gpio, "cc1_pwr_gpio");
		if(ret) {
			pr_debug("cc1_pwr_gpio gpio_request failed!\n");
			goto err_cc_pwr_gpio;
		}
	} else {
		pr_debug("%s: cc1_pwr_gpio failed!\n" , __func__);
		goto err_cc_pwr_gpio;
	}

	tusb320_dev->cc2_pwr_gpio =
		of_get_named_gpio_flags(client->dev.of_node,
		"cc2_pwr_gpio", 0, NULL);
	if (gpio_is_valid(tusb320_dev->cc2_pwr_gpio)) {
		pr_debug("%s: cc2_pwr_gpio=%d\n",__func__, tusb320_dev->cc2_pwr_gpio);
		ret = gpio_request(tusb320_dev->cc2_pwr_gpio, "cc2_pwr_gpio");
		if(ret) {
			pr_debug("cc2_pwr_gpio gpio_request failed!\n");
			goto err_cc_pwr_gpio;
		}
	} else {
		pr_debug("%s: cc2_pwr_gpio failed!\n" , __func__);
		goto err_cc_pwr_gpio;
	}

	if (tusb320_dev->cc1_pwr_gpio) {
		ret = gpio_direction_output(tusb320_dev->cc1_pwr_gpio, 0);
		if (ret < 0) {
			pr_err("%s : not able to set cc1_pwr_gpio as output\n",
				 __func__);
			goto err_cc_pwr_gpio;
		}
	}

	if (tusb320_dev->cc2_pwr_gpio) {
		ret = gpio_direction_output(tusb320_dev->cc2_pwr_gpio, 0);
		if (ret < 0) {
			pr_err("%s : not able to set cc2_pwr_gpio as output\n",
				 __func__);
			goto err_cc_pwr_gpio;
		}
	}

	/* init switch_gpio state */
	gpio_set_value(tusb320_dev->cc1_pwr_gpio, 0);
	gpio_set_value(tusb320_dev->cc2_pwr_gpio, 0);

	tusb320_dev->client = client;

	INIT_DELAYED_WORK(&tusb320_dev->check_work, tusb320_status_check);

	/* init mutex */
	//spin_lock_init(&tusb320_dev->irq_enabled_lock);
	//letv_typec_headset
	mutex_init(&typec_headset_lock);
	//letv_typec_headset
	//mutex_init(&tusb320_i2c_lock);


	tusb320_dev->tusb320_device.minor = MISC_DYNAMIC_MINOR;
	tusb320_dev->tusb320_device.name = "tusb320";
	tusb320_dev->tusb320_device.fops = &tusb320_dev_fops;

	ret = misc_register(&tusb320_dev->tusb320_device);
	if (ret) {
		pr_debug("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_debug("requesting IRQ %d\n", client->irq);

	ret = request_threaded_irq(client->irq, NULL,\
		/*tusb320_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,*/
		tusb320_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,\
		client->name, tusb320_dev);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to request irq.\n");
		goto err_request_irq_failed;
	}

	disable_irq(client->irq);

	i2c_set_clientdata(client, tusb320_dev);

#if defined(TUSB320_DEBUG)
	ret = device_create_file(&client->dev, &tusb320_dev_attr);
	if (ret) {
		pr_err("%s: sysfs registration failed, error %d \n", __func__, ret);
		goto err_create_dev_file;
	}
#endif

	g_tusb_dev = tusb320_dev;
	/* reset the chip and reset it again after 25ms */
	tusb320_soft_reset();
	mdelay(TUSB320_RESET_DURATION_MS);
	tusb320_soft_reset();
	mdelay(50);
	tusb320_hw_init(client);

	cclogic_set_audio_mode_register(tusb320_cclogic_set_audio_mode);
	schedule_delayed_work(&tusb320_dev->check_work,1*HZ);
	cclogic_updata_port_state(0);/*"cc_state: none"*/
	cclogic_updata_port_polarity(0); /* no typec usb connected*/
	msleep(50);
	enable_irq(client->irq);
#if defined(CONFIG_LETV_HW_DEV_DCT)
	if(false == i2c_op_fail) {
		set_hw_dev_flag(DEV_PERIPHIAL_USB_SWITCH);
	}
#endif
	pr_debug("tusb320 probe ok!\n");

	return 0;

err_create_dev_file:
	free_irq(client->irq, tusb320_dev);
err_request_irq_failed:
	misc_deregister(&tusb320_dev->tusb320_device);
	cancel_delayed_work_sync(&tusb320_dev->check_work);
err_misc_register:
err_switch_gpio:
	devm_pinctrl_put(tusb320_dev->pinctrl_switch);

	if (tusb320_dev->switch_gpio1) {
		gpio_free(tusb320_dev->switch_gpio1);
	}

	if (tusb320_dev->switch_gpio2) {
		gpio_free(tusb320_dev->switch_gpio2);
	}
err_cc_pwr_gpio:
	devm_pinctrl_put(tusb320_dev->pinctrl_ccpwr);

	if (tusb320_dev->cc1_pwr_gpio) {
		gpio_free(tusb320_dev->cc1_pwr_gpio);
	}

	if (tusb320_dev->cc2_pwr_gpio) {
		gpio_free(tusb320_dev->cc2_pwr_gpio);
	}
err_irq_gpio:
	devm_pinctrl_put(tusb320_dev->pinctrl_int);
	if (tusb320_dev->irq_gpio) {
		gpio_free(tusb320_dev->irq_gpio);
	}
err_allocate_mem:
	kfree(tusb320_dev);
	pr_debug("%s %d: failed! -\n", __func__, __LINE__);
	return ret;
}

static int tusb320_remove(struct i2c_client *client)
{
#if 1
	struct tusb320_dev *tusb320_dev;

	tusb320_dev = i2c_get_clientdata(client);
	free_irq(client->irq, tusb320_dev);
	misc_deregister(&tusb320_dev->tusb320_device);
	gpio_free(tusb320_dev->irq_gpio);
	mutex_destroy(&typec_headset_lock);
	mutex_destroy(&tusb320_i2c_lock);
#if defined(tusb320_DEBUG)
	device_remove_file(&client->dev, &tusb320_dev_attr);
#endif
	kfree(tusb320_dev);
#endif

	return 0;
}

static const struct i2c_device_id tusb320_id[] = {
	{ "tusb320", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tusb320_id);

#ifdef CONFIG_OF
static struct of_device_id tusb320_match_table[] = {
	{ .compatible = "tusb320",},
	{ },
};
#else
#define tusb320_match_table NULL
#endif

static struct i2c_driver tusb320_driver = {
	.id_table	= tusb320_id,
	.probe		= tusb320_probe,
	.remove		= tusb320_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tusb320",
		.of_match_table = tusb320_match_table,
	},
};

/*
 * module load/unload record keeping
 */

static int __init tusb320_dev_init(void)
{
	pr_debug("Loading tusb320 driver\n");
	return i2c_add_driver(&tusb320_driver);
}
module_init(tusb320_dev_init);

static void __exit tusb320_dev_exit(void)
{
#if 0
	struct tusb320_dev *tusb320_dev = g_tusb_dev;
	struct i2c_client *client = tusb320_dev->client;

	//tusb320_dev = i2c_get_clientdata(client);
	free_irq(client->irq, tusb320_dev);
	misc_deregister(&tusb320_dev->tusb320_device);
	gpio_free(tusb320_dev->irq_gpio);
	mutex_destroy(&typec_headset_lock);
	mutex_destroy(&tusb320_i2c_lock);
#if defined(tusb320_DEBUG)
	device_remove_file(&client->dev, &tusb320_dev_attr);
#endif
	kfree(tusb320_dev);
#endif

	printk("Unloading tusb320 driver!\n");
	i2c_del_driver(&tusb320_driver);
}
module_exit(tusb320_dev_exit);

MODULE_AUTHOR("TI");
MODULE_DESCRIPTION("TI Type-C tusb320 driver");
MODULE_LICENSE("GPL");
