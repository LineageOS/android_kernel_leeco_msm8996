/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
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
#include <linux/fs.h>
#include "slimport.h"

/* Enable or Disable HDCP by default */
/* hdcp_enable = 1: Enable,  0: Disable */
static int hdcp_enable = 1;

/* HDCP switch for external block*/
/* external_block_en = 1: enable, 0: disable*/
//int external_block_en = 1;
int external_block_en = 0;

extern uint dev_id;
extern uint lane_err_v;
extern unchar dongle_type;
extern unchar pre_emphasis_v;

/* to access global platform data */
static struct anx7816_platform_data *g_pdata;

/* Use device tree structure data when defined "CONFIG_OF"  */
/*//#define SP_REGISTER_SET_TEST*/

#ifdef SP_REGISTER_SET_TEST
/*//For Slimport swing&pre-emphasis test*/
unchar val_SP_TX_LT_CTRL_REG0;
unchar val_SP_TX_LT_CTRL_REG10;
unchar val_SP_TX_LT_CTRL_REG11;
unchar val_SP_TX_LT_CTRL_REG2;
unchar val_SP_TX_LT_CTRL_REG12;
unchar val_SP_TX_LT_CTRL_REG1;
unchar val_SP_TX_LT_CTRL_REG6;
unchar val_SP_TX_LT_CTRL_REG16;
unchar val_SP_TX_LT_CTRL_REG5;
unchar val_SP_TX_LT_CTRL_REG8;
unchar val_SP_TX_LT_CTRL_REG15;
unchar val_SP_TX_LT_CTRL_REG18;
#endif

#define TRUE 1
#define FALSE 0

struct regulator *dp_reg;
struct regulator *anx7418_boost_5v;
struct clk *dp_clk_base = NULL;
struct i2c_client *anx7816_client;

struct anx7816_platform_data {
	int gpio_p_dwn;
	int gpio_reset;
	int gpio_cbl_det;
	int gpio_v10_ctrl;
	int gpio_v33_ctrl;
#if 0
	int external_ldo_control;
#endif
	int (*avdd_power)(unsigned int onoff);
	int (*dvdd_power)(unsigned int onoff);
	struct regulator *avdd_10;
	struct regulator *dvdd_10;
	spinlock_t lock;
};

struct anx7816_data {
	struct anx7816_platform_data *pdata;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
};

struct slimport_device_info{
	struct device *device;
	struct class  *dev_class;
	dev_t slimport_dev_t;
};

#ifdef USING_HPD_FOR_POWER_MANAGEMENT
bool sink_is_connected(void)
{
	bool result = false;
#ifdef CONFIG_OF
	struct anx7816_platform_data *pdata = g_pdata;
#else
	struct anx7816_platform_data *pdata = anx7816_client->dev.platform_data;
#endif

	if (!pdata)
		return FALSE;

	if (gpio_get_value(pdata->gpio_cbl_det) == 0x01) {
			pr_info("%s %s : Downstream HPD is detected\n",
				LOG_TAG, __func__);
			result = TRUE;
	}

	return result;
}
#endif

enum gpio_direction_types {
	GPIO_OUTPUT,
	GPIO_INPUT
};

static int dp_set_gpio(const char *gpio_name, int gpio_number,
		enum gpio_direction_types direction, int out_val)
{
	int ret = -EBUSY;
	pr_info("%s()\n", __func__);
	pr_info("%s:%s=[%d]\n", __func__, gpio_name, gpio_number);

	if (gpio_number < 0)
		return -EINVAL;

	if (gpio_is_valid(gpio_number)) {
		ret = gpio_request((unsigned int)gpio_number, gpio_name);
		if (ret < 0) {
			pr_err("%s:%s=[%d] req failed:%d\n",
				__func__, gpio_name, gpio_number, ret);
			return -EBUSY;
		}
		if (direction == GPIO_OUTPUT) {
			pr_debug("%s:gpio output\n", __func__);
			ret = gpio_direction_output(
				(unsigned int)gpio_number, out_val);
		} else if (direction == GPIO_INPUT) {
			pr_debug("%s:gpio input\n", __func__);
			ret = gpio_direction_input((unsigned int)gpio_number);
		} else {
			pr_err("%s:%s=[%d] invalid direction type :%d\n",
				__func__, gpio_name, gpio_number, ret);
			return -EINVAL;
		}
		if (ret < 0) {
			pr_err("%s: set dirn %s failed: %d\n",
				__func__, gpio_name, ret);
			return -EBUSY;
		}
	}

	return 0;
}

static ssize_t slimport_dp_dev_type_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	unsigned int dev_type = dongle_type;
	const char *dev_type_str = NULL;

	switch (dev_type) {
		case 1:
			dev_type_str = "TV\n";
			break;
		case 2:
			dev_type_str = "DP\n";
			break;
		case 3:
			dev_type_str = "VGA\n";
			break;
		default:
			dev_type_str = "None\n";
			break;
	}

	if (NULL != dev_type_str)
		rv = scnprintf(buf, PAGE_SIZE, "%s", dev_type_str);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");

	return retval ? retval : rv;
}

static ssize_t slimport_dp_dev_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;

	if (0 != dev_id)
		rv = scnprintf(buf, PAGE_SIZE, "ANX%x\n", dev_id);
	else
		rv = scnprintf(buf, PAGE_SIZE, "ANX%s\n", "");

	return retval ? retval : rv;
}

static ssize_t slimport_dp_lane_err_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;

	rv = scnprintf(buf, PAGE_SIZE, "%d\n", lane_err_v);

	return retval ? retval : rv;
}

static ssize_t slimport_dp_pre_emphasis_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	const char *dev_pre_emphasis = NULL;

	pre_emphasis_v = (pre_emphasis_v & SWINT_EMPHASIS_MASK1)|((pre_emphasis_v & SWINT_EMPHASIS_MASK2) >> 1);
	switch (pre_emphasis_v) {
		case SWING0_EMPHASIS_0DB:
			dev_pre_emphasis = "swing0 pre-emphasis 0 dB\n";
			break;
		case SWING1_EMPHASIS_0DB:
			dev_pre_emphasis = "swing1 pre-emphasis 0 dB\n";
			break;
		case SWING2_EMPHASIS_0DB:
			dev_pre_emphasis = "swing2 pre-emphasis 0 dB\n";
			break;
		case SWING3_EMPHASIS_0DB:
			dev_pre_emphasis = "swing3 pre-emphasis 0 dB\n";
			break;
		case SWING0_EMPHASIS_35DB:
			dev_pre_emphasis = "swing0 pre-emphasis 3.5 dB\n";
			break;
		case SWING1_EMPHASIS_35DB:
			dev_pre_emphasis = "swing1 pre-emphasis 3.5 dB\n";
			break;
		case SWING2_EMPHASIS_35DB:
			dev_pre_emphasis = "swing2 pre-emphasis 3.5 dB\n";
			break;
		case SWING0_EMPHASIS_6DB:
			dev_pre_emphasis = "swing0 pre-emphasis 6 dB\n";
			break;
		case SWING1_EMPHASIS_6DB:
			dev_pre_emphasis = "swing1 pre-emphasis 6 dB\n";
			break;
		case SWING0_EMPHASIS_95DB_1:
		case SWING0_EMPHASIS_95DB_2:
		case SWING0_EMPHASIS_95DB_3:
		case SWING0_EMPHASIS_95DB_4:
		case SWING0_EMPHASIS_95DB_5:
		case SWING0_EMPHASIS_95DB_6:
		case SWING0_EMPHASIS_95DB_7:
			dev_pre_emphasis = "swing0 pre-emphasis 9.5 dB\n";
			break;
		default:
			dev_pre_emphasis = "swing and pre-emphasis error!\n";
			break;
	}

	if (NULL != dev_pre_emphasis)
		rv = scnprintf(buf, PAGE_SIZE, "%s", dev_pre_emphasis);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");

	return retval ? retval : rv;
}

struct slimport_device_info *slimport_dp_device_info = NULL;

static DEVICE_ATTR(devtype, (S_IRUGO),
			slimport_dp_dev_type_show, NULL);

static DEVICE_ATTR(devid, (S_IRUGO),
			slimport_dp_dev_id_show, NULL);

static DEVICE_ATTR(lane_err, (S_IRUGO),
			slimport_dp_lane_err_show, NULL);

static DEVICE_ATTR(pre_emphasis, (S_IRUGO),
			slimport_dp_pre_emphasis_show, NULL);

static struct attribute *slimport_dp_attrs[] = {
	&dev_attr_devtype.attr,
	&dev_attr_devid.attr,
	&dev_attr_lane_err.attr,
	&dev_attr_pre_emphasis.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group slimport_dp_attr_group = {
	.attrs = slimport_dp_attrs,
};

static int slimport_dp_dev_init(void)
{
	int ret = -1;
	struct slimport_device_info *dev_info;

	if (NULL != slimport_dp_device_info){
		pr_info("%s: slimport_dp_device_info already init\n", __func__);
		return 0;
	}

	slimport_dp_device_info = kzalloc(sizeof(struct slimport_device_info), GFP_KERNEL);
	if (!slimport_dp_device_info) {
		ret = -ENOMEM;
		return ret;
	}
	dev_info = slimport_dp_device_info;

	ret = alloc_chrdev_region(&dev_info->slimport_dev_t, 0, 1, "slimport_dp_t");
	if (ret < 0) {
		pr_err("%s:Fail to alloc char dev region\n", __func__);
		return ret;
	}

	dev_info->dev_class = class_create(THIS_MODULE, "slimport_dp_class");
	if (IS_ERR(dev_info->dev_class)) {
		printk("%s: create class fail \n", __func__);
		return -1;
	}

	dev_info->device = device_create(dev_info->dev_class,
					NULL, dev_info->slimport_dev_t, NULL,
					"slimport_dp_device");

	ret = sysfs_create_group(&dev_info->device->kobj,
					&slimport_dp_attr_group);

	return ret;
}

static int slimport_dp_dev_exit(void)
{
	int ret = 0;
	struct slimport_device_info *dev_info;

	if (NULL == slimport_dp_device_info)
		return -1;
	dev_info = slimport_dp_device_info;

	sysfs_remove_group(&dev_info->device->kobj,
                &slimport_dp_attr_group);

	device_unregister(dev_info->device);

	device_destroy(dev_info->dev_class, dev_info->slimport_dev_t);

	class_destroy(dev_info->dev_class);

	unregister_chrdev_region(dev_info->slimport_dev_t,1);

	kfree(dev_info);
	return ret;
}

static ssize_t slimport7816_rev_check_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t slimport7816_rev_check_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	switch (cmd) {
	case 1:
		/*//sp_tx_chip_located();*/
		break;
	}
	return count;
}

/*sysfs interface : Enable or Disable HDCP by default*/
static ssize_t sp_hdcp_feature_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hdcp_enable);
}

static ssize_t sp_hdcp_feature_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	hdcp_enable = val;
	pr_info(" hdcp_enable = %d\n", hdcp_enable);
	return count;
}

/*sysfs  interface : HDCP switch for VGA dongle*/
static ssize_t sp_external_block_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", external_block_en);
}

static ssize_t sp_external_block_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	external_block_en = val;
	return count;
}

/*sysfs  interface : i2c_master_read_reg, i2c_master_write_reg
anx7730 addr id:: DP_rx(0x50:0, 0x8c:1) HDMI_tx(0x72:5, 0x7a:6, 0x70:7)
ex:read ) 05df   = read:0  id:5 reg:0xdf
ex:write) 15df5f = write:1 id:5 reg:0xdf val:0x5f
*/
static ssize_t anx7730_write_reg_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	char op, i;
	char r[3];
	char v[3];
	unchar tmp;
	int id, reg, val = 0;

	if (sp_tx_cur_states() != STATE_PLAY_BACK) {
		pr_err("%s: error!, Not STATE_PLAY_BACK\n", LOG_TAG);
		return -EINVAL;
	}

	if (count != 7 && count != 5) {
		pr_err("%s: cnt:%zu, invalid input!\n", LOG_TAG, count - 1);
		pr_err("%s: ex) 05df   -> op:0(read)  id:5 reg:0xdf\n",
		       LOG_TAG);
		pr_err("%s: ex) 15df5f -> op:1(wirte) id:5 reg:0xdf val:0x5f\n",
		       LOG_TAG);
		return -EINVAL;
	}

	ret = snprintf(&op, 2, buf);
	ret = snprintf(&i, 2, buf + 1);
	ret = snprintf(r, 3, buf + 2);

	id = simple_strtoul(&i, NULL, 10);
	reg = simple_strtoul(r, NULL, 16);

	if ((id != 0 && id != 1 && id != 5 && id != 6 && id != 7)) {
		pr_err("%s: invalid addr id! (id:0,1,5,6,7)\n", LOG_TAG);
		return -EINVAL;
	}

	switch (op) {
	case 0x30:		/* "0" -> read */
		i2c_master_read_reg(id, reg, &tmp);
		pr_info("%s: anx7730 read(%d,0x%x)= 0x%x\n", LOG_TAG, id, reg,
			tmp);
		break;

	case 0x31:		/* "1" -> write */
		ret = snprintf(v, 3, buf + 4);
		val = simple_strtoul(v, NULL, 16);

		i2c_master_write_reg(id, reg, val);
		i2c_master_read_reg(id, reg, &tmp);
		pr_info("%s: anx7730 write(%d,0x%x,0x%x)\n", LOG_TAG, id, reg,
			tmp);
		break;

	default:
		pr_err("%s: invalid operation code! (0:read, 1:write)\n",
		       LOG_TAG);
		return -EINVAL;
	}

	return count;
}

/*sysfs  interface : sp_read_reg, sp_write_reg
anx7816 addr id:: HDMI_rx(0x7e:0, 0x80:1) DP_tx(0x72:5, 0x7a:6, 0x70:7)
ex:read ) 05df   = read:0  id:5 reg:0xdf
ex:write) 15df5f = write:1 id:5 reg:0xdf val:0x5f
*/
static int anx7816_id_change(int id)
{
	int chg_id = 0;

	switch (id) {
	case 0:
		chg_id = RX_P0;
		break;
	case 1:
		chg_id = RX_P1;
		break;
	case 5:
		chg_id = TX_P2;
		break;
	case 6:
		chg_id = TX_P1;
		break;
	case 7:
		chg_id = TX_P0;
		break;
	}
	return chg_id;
}

static ssize_t anx7816_write_reg_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	char op, i;
	char r[3];
	char v[3];
	unchar tmp;
	int id, reg, val = 0;

	if (sp_tx_cur_states() != STATE_PLAY_BACK) {
		pr_err("%s: error!, Not STATE_PLAY_BACK\n", LOG_TAG);
		return -EINVAL;
	}

	if (count != 7 && count != 5) {
		pr_err("%s: cnt:%zu, invalid input!\n", LOG_TAG, count - 1);
		pr_err("%s: ex) 05df   -> op:0(read)  id:5 reg:0xdf\n",
		       LOG_TAG);
		pr_err("%s: ex) 15df5f -> op:1(wirte) id:5 reg:0xdf val:0x5f\n",
		       LOG_TAG);
		return -EINVAL;
	}

	ret = snprintf(&op, 2, buf);
	ret = snprintf(&i, 2, buf + 1);
	ret = snprintf(r, 3, buf + 2);

	id = simple_strtoul(&i, NULL, 10);
	reg = simple_strtoul(r, NULL, 16);

	if ((id != 0 && id != 1 && id != 5 && id != 6 && id != 7)) {
		pr_err("%s: invalid addr id! (id:0,1,5,6,7)\n", LOG_TAG);
		return -EINVAL;
	}

	id = anx7816_id_change(id);	/*//ex) 5 -> 0x72*/

	switch (op) {
	case 0x30:		/* "0" -> read */
		sp_read_reg(id, reg, &tmp);
		pr_info("%s: anx7816 read(0x%x,0x%x)= 0x%x\n", LOG_TAG, id,
			reg, tmp);
		break;

	case 0x31:		/* "1" -> write */
		ret = snprintf(v, 3, buf + 4);
		val = simple_strtoul(v, NULL, 16);

		sp_write_reg(id, reg, val);
		sp_read_reg(id, reg, &tmp);
		pr_info("%s: anx7816 write(0x%x,0x%x,0x%x)\n", LOG_TAG, id, reg,
			tmp);
		break;

	default:
		pr_err("%s: invalid operation code! (0:read, 1:write)\n",
		       LOG_TAG);
		return -EINVAL;
	}

	return count;
}

#ifdef SP_REGISTER_SET_TEST	/*//Slimport test*/
/*sysfs read interface*/
static int ctrl_reg0_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG0);
}

/*sysfs write interface*/
static int ctrl_reg0_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG0 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg10_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG10);
}

/*sysfs write interface*/
static int ctrl_reg10_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG10 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg11_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG11);
}

/*sysfs write interface*/
static int ctrl_reg11_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG11 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg2_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG2);
}

/*sysfs write interface*/
static int ctrl_reg2_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG2 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg12_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG12);
}

/*sysfs write interface*/
static int ctrl_reg12_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG12 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg1_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG1);
}

/*sysfs write interface*/
static int ctrl_reg1_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG1 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg6_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG6);
}

/*sysfs write interface*/
static int ctrl_reg6_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG6 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg16_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG16);
}

/*sysfs write interface*/
static int ctrl_reg16_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG16 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg5_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG5);
}

/*sysfs write interface*/
static int ctrl_reg5_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG5 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg8_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG8);
}

/*sysfs write interface*/
static int ctrl_reg8_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG8 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg15_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG15);
}

/*sysfs write interface*/
static int ctrl_reg15_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG15 = val;
	return count;
}

/*sysfs read interface*/
static int ctrl_reg18_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sprintf(buf, "0x%x\n", val_SP_TX_LT_CTRL_REG18);
}

/*sysfs write interface*/
static int ctrl_reg18_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, int count)
{
	int ret;
	long val;
	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;
	val_SP_TX_LT_CTRL_REG18 = val;
	return count;
}
#endif
/* for debugging */
static struct device_attribute slimport_device_attrs[] = {
	__ATTR(rev_check, S_IRUGO | S_IWUSR, slimport7816_rev_check_show,
	       slimport7816_rev_check_store),
	__ATTR(hdcp, S_IRUGO | S_IWUSR, sp_hdcp_feature_show,
	       sp_hdcp_feature_store),
	__ATTR(hdcp_switch, S_IRUGO | S_IWUSR, sp_external_block_show,
	       sp_external_block_store),
	__ATTR(anx7730, S_IRUGO | S_IWUSR, slimport7816_rev_check_show,
	       anx7730_write_reg_store),
	__ATTR(anx7816, S_IRUGO | S_IWUSR, slimport7816_rev_check_show,
	       anx7816_write_reg_store),
#ifdef SP_REGISTER_SET_TEST	/*//slimport test*/
	__ATTR(ctrl_reg0, S_IRUGO | S_IWUSR, ctrl_reg0_show, ctrl_reg0_store),
	__ATTR(ctrl_reg10, S_IRUGO | S_IWUSR, ctrl_reg10_show,
	       ctrl_reg10_store),
	__ATTR(ctrl_reg11, S_IRUGO | S_IWUSR, ctrl_reg11_show,
	       ctrl_reg11_store),
	__ATTR(ctrl_reg2, S_IRUGO | S_IWUSR, ctrl_reg2_show, ctrl_reg2_store),
	__ATTR(ctrl_reg12, S_IRUGO | S_IWUSR, ctrl_reg12_show,
	       ctrl_reg12_store),
	__ATTR(ctrl_reg1, S_IRUGO | S_IWUSR, ctrl_reg1_show, ctrl_reg1_store),
	__ATTR(ctrl_reg6, S_IRUGO | S_IWUSR, ctrl_reg6_show, ctrl_reg6_store),
	__ATTR(ctrl_reg16, S_IRUGO | S_IWUSR, ctrl_reg16_show,
	       ctrl_reg16_store),
	__ATTR(ctrl_reg5, S_IRUGO | S_IWUSR, ctrl_reg5_show, ctrl_reg5_store),
	__ATTR(ctrl_reg8, S_IRUGO | S_IWUSR, ctrl_reg8_show, ctrl_reg8_store),
	__ATTR(ctrl_reg15, S_IRUGO | S_IWUSR, ctrl_reg15_show,
	       ctrl_reg15_store),
	__ATTR(ctrl_reg18, S_IRUGO | S_IWUSR, ctrl_reg18_show,
	       ctrl_reg18_store),
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(slimport_device_attrs); i++)
		if (device_create_file(dev, &slimport_device_attrs[i]))
			goto error;
	return 0;
error:
	for (; i >= 0; i--)
		device_remove_file(dev, &slimport_device_attrs[i]);
	pr_err("%s %s: Unable to create interface", LOG_TAG, __func__);
	return -EINVAL;
}

int sp_read_reg_byte(uint8_t slave_addr, uint8_t offset)
{
	int ret = 0;

	anx7816_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_read_byte_data(anx7816_client, offset);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x\n", LOG_TAG,
		       __func__, slave_addr);
		return ret;
	}
	return 0;
}

int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf)
{
	int ret = 0;

	anx7816_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_read_byte_data(anx7816_client, offset);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x\n", LOG_TAG,
		       __func__, slave_addr);
		return ret;
	}
	*buf = (uint8_t) ret;

	return 0;
}

int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value)
{
	int ret = 0;

	anx7816_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_write_byte_data(anx7816_client, offset, value);
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,
		       __func__, slave_addr);
	}
	return ret;
}

void sp_tx_hardware_poweron(void)
{
    int ret;
#ifdef CONFIG_OF
	struct anx7816_platform_data *pdata = g_pdata;
#else
	struct anx7816_platform_data *pdata = anx7816_client->dev.platform_data;
#endif

	gpio_set_value(pdata->gpio_reset, 0);
	usleep_range(1000, 2000);
    #if 0
	if (pdata->external_ldo_control) {
		/* Enable 1.0V LDO */
		gpio_set_value(pdata->gpio_v10_ctrl, 1);
		usleep_range(1000, 2000);
	}
    #endif
    ret = regulator_enable(dp_reg);
    if (ret < 0){
	    pr_info("%s %s: failed to enable regulator 1.0v\n", LOG_TAG, __func__);
        regulator_disable(dp_reg);
    }
	usleep_range(1000, 2000);
	gpio_set_value(pdata->gpio_p_dwn, 0);
    usleep_range(2000, 4000);
    if (dp_clk_base){
	    ret = clk_prepare_enable(dp_clk_base);
	    if (ret) {
	        pr_info("%s %s: failed to enable dp_clk_base\n", LOG_TAG, __func__);
            clk_disable_unprepare(dp_clk_base);
	    }
    }
	usleep_range(8000, 9000);
	gpio_set_value(pdata->gpio_reset, 1);

	usleep_range(2000, 4000);
    ret = regulator_enable(anx7418_boost_5v);
    if (ret < 0){
	    pr_info("%s %s: failed to enable anx7418_boost_5v\n", LOG_TAG, __func__);
        regulator_disable(anx7418_boost_5v);
    }

    pr_info("%s %s: anx7816 power on\n", LOG_TAG, __func__);
}

void sp_tx_hardware_powerdown(void)
{
//    int ret;
#ifdef CONFIG_OF
	struct anx7816_platform_data *pdata = g_pdata;
#else
	struct anx7816_platform_data *pdata = anx7816_client->dev.platform_data;
#endif

	gpio_set_value(pdata->gpio_reset, 0);
	usleep_range(1000, 2000);
    #if 0
	if (pdata->external_ldo_control) {
		gpio_set_value(pdata->gpio_v10_ctrl, 0);
		usleep_range(1000, 2000);
	}
    #endif
	gpio_set_value(pdata->gpio_p_dwn, 1);
	usleep_range(1000, 2000);
	#if 0
    pr_info("%s %s: begin to disable dp regulator 1.0v\n", LOG_TAG, __func__);
    ret = regulator_disable(dp_reg);
    if (ret < 0)
	    pr_info("%s %s: regulator disable failed\n", LOG_TAG, __func__);
    usleep_range(1000, 2000);
    pr_info("%s %s: begin to disable dp_clk_base\n", LOG_TAG, __func__);
	if (dp_clk_base){
	    pr_info("%s %s: failed to disable dp_clk_base\n", LOG_TAG, __func__);
        clk_disable_unprepare(dp_clk_base);
    }
    usleep_range(1000, 2000);
    pr_info("%s %s: begin to disable anx7418_boost_5v\n", LOG_TAG, __func__);
    ret = regulator_disable(anx7418_boost_5v);
    if (ret < 0)
	    pr_info("%s %s: anx7418_boost_5v disable failed\n", LOG_TAG, __func__);
    #endif
	pr_info("%s %s: anx7816 power down\n", LOG_TAG, __func__);
}

int slimport_read_edid_block(int block, uint8_t *edid_buf)
{
	if (block == 0) {
		memcpy(edid_buf, edid_blocks, 128 * sizeof(char));
	} else if (block == 1) {
		memcpy(edid_buf, (edid_blocks + 128), 128 * sizeof(char));
	} else {
		pr_err("%s %s: block number %d is invalid\n",
		       LOG_TAG, __func__, block);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(slimport_read_edid_block);

char *slimport_read_edid_block_ext(void)
{
	return edid_blocks;
}
EXPORT_SYMBOL(slimport_read_edid_block_ext);


static void anx7816_free_gpio(struct anx7816_data *anx7816)
{
    #ifdef USING_HPD_FOR_POWER_MANAGEMENT
	gpio_free(anx7816->pdata->gpio_cbl_det);
	#endif
	gpio_free(anx7816->pdata->gpio_reset);
	gpio_free(anx7816->pdata->gpio_p_dwn);
    #if 0
	if (anx7816->pdata->external_ldo_control)
		gpio_free(anx7816->pdata->gpio_v10_ctrl);
    #endif
}

static int anx7816_init_gpio(struct anx7816_data *anx7816)
{
	int ret = 0;

	/*  gpio for chip power down  */
	ret = gpio_request(anx7816->pdata->gpio_p_dwn, "anx7816_p_dwn_ctl");
	if (ret) {
        pr_info("%s %s: failed to request gpio_p_dwn : %d\n", LOG_TAG, __func__,anx7816->pdata->gpio_p_dwn);
		goto err0;
	}
	gpio_direction_output(anx7816->pdata->gpio_p_dwn, 1);
	/*  gpio for chip reset  */
	ret = gpio_request(anx7816->pdata->gpio_reset, "anx7816_reset_n");
	if (ret) {
        pr_info("%s %s: failed to request gpio_reset : %d\n", LOG_TAG, __func__,anx7816->pdata->gpio_reset);
		goto err1;
	}
	gpio_direction_output(anx7816->pdata->gpio_reset, 0);
	/*  gpio for slimport cable detect  */
	#ifdef USING_HPD_FOR_POWER_MANAGEMENT
	ret = gpio_request(anx7816->pdata->gpio_cbl_det, "anx7816_cbl_det");
	if (ret) {
        pr_info("%s %s: failed to request gpio_cbl_det : %d\n", LOG_TAG, __func__,anx7816->pdata->gpio_cbl_det);
		goto err2;
	}
	gpio_direction_input(anx7816->pdata->gpio_cbl_det);
	#endif
    #if 0
	/*  gpios for power control */
	if (anx7816->pdata->external_ldo_control) {
		/* V10 power control */
		ret = gpio_request(anx7816->pdata->gpio_v10_ctrl,
				   "anx7816_v10_ctrl");
		if (ret) {
			pr_err("%s : failed to request gpio %d\n",
			       __func__, anx7816->pdata->gpio_v10_ctrl);
			goto err3;
		}
		gpio_direction_output(anx7816->pdata->gpio_v10_ctrl, 0);
		/* V33 power control */
		ret = gpio_request(anx7816->pdata->gpio_v33_ctrl,
				   "anx7816_v33_ctrl");
		if (ret) {
			pr_err("%s : failed to request gpio %d\n",
			       __func__, anx7816->pdata->gpio_v33_ctrl);
			goto err4;
		}
		gpio_direction_output(anx7816->pdata->gpio_v33_ctrl, 0);

	}
    #endif
	goto out;
    #if 0
err3:
	gpio_free(anx7816->pdata->gpio_v10_ctrl);
	#endif
    #ifdef USING_HPD_FOR_POWER_MANAGEMENT
err2:
	gpio_free(anx7816->pdata->gpio_cbl_det);	
	#endif
err1:
	gpio_free(anx7816->pdata->gpio_reset);
err0:
	gpio_free(anx7816->pdata->gpio_p_dwn);
out:
	return ret;
}

static int anx7816_system_init(void)
{
	int ret = 0;

	ret = slimport_chip_detect();
	if (ret == 0) {
		sp_tx_hardware_powerdown();
		pr_err("%s : failed to detect anx7816\n", __func__);
		return -ENODEV;
	}

	slimport_chip_initial();
	return 0;
}

#ifdef USING_HPD_FOR_POWER_MANAGEMENT
void sp_cable_disconnect(void *data)
{
	struct anx7816_data *anx7816 = data;
	cancel_delayed_work_sync(&anx7816->work);
	flush_workqueue(anx7816->workqueue);
	sp_tx_hardware_powerdown();
	sp_tx_clean_state_machine();
}

static int confirmed_cable_det(void *data)
{
	struct anx7816_data *anx7816 = data;
	int count = 10;
	int cable_det_count = 0;
	do {
		if (gpio_get_value(anx7816->pdata->gpio_cbl_det)
				== 0x01)
			cable_det_count++;
		mdelay(5);
	} while (count--);

	return (cable_det_count > 5) ? 1 : 0;
}


static irqreturn_t anx7816_cbl_det_isr(int irq, void *data)
{
	/***********************************************************************
	HPD IRQ Event: HPD pulse width greater than 0.25ms but narrower than 2ms;
	Hot Unplug Event: HPD pulse stays low longer than 2ms;
	So, AP just monitor HPD pulse high in this irq, if HPD is high,
	the driver will power on chip,
	and then the driver control when to powerdown the chip.
	If HPD event is HPD IRQ, the driver deal with IRQ event from downstream.
	If HPD event is Hot Plug, ther drive power down the chip.
	***********************************************************************/
	struct anx7816_data *anx7816 = data;
	int cable_connected = 0;
	cable_connected = confirmed_cable_det(data);
	pr_info("%s %s : detect cable insertion, cable_connected = %d\n",
					LOG_TAG, __func__, cable_connected);
	if (cable_connected == 0x01) {
		pr_info("%s %s : detect HPD High\n", LOG_TAG, __func__);
		queue_delayed_work(anx7816->workqueue, &anx7816->work, 0);
	} else {
		pr_info("%s %s : detect HPD LOW\n", LOG_TAG, __func__);
		sp_cable_disconnect(anx7816);
	}
	return IRQ_HANDLED;
}
#endif

static void anx7816_work_func(struct work_struct *work)
{
	struct anx7816_data *td = container_of(work, struct anx7816_data,
					       work.work);
	int workqueu_timer = 0;
	if (sp_tx_cur_states() >= STATE_PLAY_BACK)
		workqueu_timer = 500;
	else
		workqueu_timer = 100;
	mutex_lock(&td->lock);
	slimport_main_process();
	mutex_unlock(&td->lock);
	queue_delayed_work(td->workqueue, &td->work,
			   msecs_to_jiffies(workqueu_timer));
}

#ifdef CONFIG_OF
int anx7816_regulator_configure(struct device *dev,
				struct anx7816_platform_data *pdata)
{
	int rc = 0;
/* To do : regulator control after H/W change */
	return rc;

	pdata->avdd_10 = regulator_get(dev, "vdd_10");

	if (IS_ERR(pdata->avdd_10)) {
		rc = PTR_ERR(pdata->avdd_10);
		pr_err("%s : Regulator get failed avdd_10 rc=%d\n",
		       __func__, rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->avdd_10) > 0) {
		rc = regulator_set_voltage(pdata->avdd_10, 1000000, 1000000);
		if (rc) {
			pr_err("%s : Regulator set_vtg failed rc=%d\n",
			       __func__, rc);
			goto error_set_vtg_avdd_10;
		}
	}

	pdata->dvdd_10 = regulator_get(dev, "analogix,vdd_dig");
	if (IS_ERR(pdata->dvdd_10)) {
		rc = PTR_ERR(pdata->dvdd_10);
		pr_err("%s : Regulator get failed dvdd_10 rc=%d\n",
		       __func__, rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->dvdd_10) > 0) {
		rc = regulator_set_voltage(pdata->dvdd_10, 1000000, 1000000);
		if (rc) {
			pr_err("%s : Regulator set_vtg failed rc=%d\n",
			       __func__, rc);
			goto error_set_vtg_dvdd_10;
		}
	}

	return 0;

error_set_vtg_dvdd_10:
	regulator_put(pdata->dvdd_10);
error_set_vtg_avdd_10:
	regulator_put(pdata->avdd_10);

	return rc;
}

static int anx7816_parse_dt(struct device *dev,
			    struct anx7816_platform_data *pdata)
{
	int ret = 0;
	int error;
	int dp_clock_gpio;
	struct device_node *np = dev->of_node;

	pdata->gpio_p_dwn =
	    of_get_named_gpio_flags(np, "analogix,p-dwn-gpio", 0, NULL);

	pdata->gpio_reset =
	    of_get_named_gpio_flags(np, "analogix,reset-gpio", 0, NULL);

	#ifdef USING_HPD_FOR_POWER_MANAGEMENT
    pdata->gpio_cbl_det =
	    of_get_named_gpio_flags(np, "analogix,cbl-det-gpio", 0, NULL);
	#endif
    pr_info(
	       "%s gpio p_dwn : %d, reset : %d,  gpio_cbl_det %d\n",
	       LOG_TAG, pdata->gpio_p_dwn,
	       pdata->gpio_reset, pdata->gpio_cbl_det);
	/*
	 * if "external-ldo-control" property is not exist, we
	 * assume that it is used in board.
	 * if don't use external ldo control,
	 * please use "external-ldo-control=<0>" in dtsi
	 */
    #if 0
	rc = of_property_read_u32(np, "analogix,external-ldo-control",
				  &pdata->external_ldo_control);
	if (rc == -EINVAL)
		pdata->external_ldo_control = 1;

	if (pdata->external_ldo_control) {
		pdata->gpio_v10_ctrl =
		    of_get_named_gpio_flags(np, "analogix,v10-ctrl-gpio", 0,
					    NULL);

		pdata->gpio_v33_ctrl =
		    of_get_named_gpio_flags(np, "analogix,v33-ctrl-gpio", 0,
					    NULL);
		pr_info("%s gpio_v10_ctrl %d avdd33-en-gpio %d\n",
		       LOG_TAG, pdata->gpio_v10_ctrl, pdata->gpio_v33_ctrl);
	}
	if (anx7816_regulator_configure(dev, pdata) < 0) {
		pr_err("%s %s: parsing dt for anx7816 is failed.\n",
		       LOG_TAG, __func__);
		return rc;
	}

	/* connects function nodes which are not provided with dts */
	pdata->avdd_power = slimport7816_avdd_power;
	pdata->dvdd_power = slimport7816_dvdd_power;
	#endif

    /* add Letv DP regulator 1.0v */
    dp_reg = regulator_get(dev, "vdd_10");
	if (IS_ERR(dp_reg)) {
		error = PTR_ERR(dp_reg);
		dev_err(dev, "Error %d getting vdd_10 regulator\n", error);
        regulator_put(dp_reg);
	}

    /* add Letv DP regulator 5.0v */
    anx7418_boost_5v = regulator_get(dev, "pmi8994_boost_5v");
	if (IS_ERR(anx7418_boost_5v)) {
		error = PTR_ERR(anx7418_boost_5v);
		dev_err(dev, "Error %d getting anx7418_boost_5v regulator\n", error);
        regulator_put(anx7418_boost_5v);
	}
	
    /* add Letv DP Clock */
    dp_clk_base = clk_get(dev, "dp_clk");
	if (IS_ERR(dp_clk_base)) {
		error = PTR_ERR(dp_clk_base);
		dev_err(dev, "Error %d getting dp_clk_base\n", error);
        clk_put(dp_clk_base);
	}
	dp_clock_gpio = of_get_named_gpio(np, "analogix,dp-clk-gpio", 0);
	if (dp_clock_gpio < 0) {
		pr_info("%s: failed to get dp_clock_gpio\n", __func__);
		return -EINVAL;
	}
	ret = dp_set_gpio("dp-clk-gpio", dp_clock_gpio, GPIO_OUTPUT, 0);
	gpio_set_value(dp_clock_gpio, 1);

    return 0;
}
#else
static int anx7816_parse_dt(struct device *dev,
			    struct anx7816_platform_data *pdata)
{
	return -ENODEV;
}
#endif

int slimport_dp_hdmi_device_show (char *buf)
{
	const char *dev_type_str = "dp";

	scnprintf(buf, PAGE_SIZE, "%s", dev_type_str);

	return 0;
}
static int anx7816_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{

	struct anx7816_data *anx7816;
	struct anx7816_platform_data *pdata;
	int ret = 0;

	pr_info("%s %s begin anx7816_i2c_probe\n", LOG_TAG, __func__);

#ifdef SP_REGISTER_SET_TEST
	val_SP_TX_LT_CTRL_REG0 = 0x19;
	val_SP_TX_LT_CTRL_REG10 = 0x00;
	val_SP_TX_LT_CTRL_REG11 = 0x00;
	val_SP_TX_LT_CTRL_REG2 = 0x36;
	val_SP_TX_LT_CTRL_REG12 = 0x00;
	val_SP_TX_LT_CTRL_REG1 = 0x26;
	val_SP_TX_LT_CTRL_REG6 = 0x3c;
	val_SP_TX_LT_CTRL_REG16 = 0x18;
	val_SP_TX_LT_CTRL_REG5 = 0x28;
	val_SP_TX_LT_CTRL_REG8 = 0x2F;
	val_SP_TX_LT_CTRL_REG15 = 0x10;
	val_SP_TX_LT_CTRL_REG18 = 0x1F;
#endif
	if (!i2c_check_functionality(client->adapter,
	I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s: i2c bus does not support the anx7816\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	anx7816 = kzalloc(sizeof(struct anx7816_data), GFP_KERNEL);
	if (!anx7816) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct anx7816_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		client->dev.platform_data = pdata;
		/* device tree parsing function call */
		ret = anx7816_parse_dt(&client->dev, pdata);
		if (ret != 0)	/* if occurs error */
			goto err0;

		anx7816->pdata = pdata;
	} else {
		anx7816->pdata = client->dev.platform_data;
	}

	/* to access global platform data */
	g_pdata = anx7816->pdata;

	anx7816_client = client;

	mutex_init(&anx7816->lock);

	if (!anx7816->pdata) {
		ret = -EINVAL;
		goto err0;
	}
	ret = anx7816_init_gpio(anx7816);
	if (ret) {
		pr_err("%s: failed to initialize gpio\n", __func__);
		goto err0;
	}

	INIT_DELAYED_WORK(&anx7816->work, anx7816_work_func);

	anx7816->workqueue = create_singlethread_workqueue("anx7816_work");
	if (anx7816->workqueue == NULL) {
		pr_err("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	#if 0
	anx7816->pdata->avdd_power(1);
	anx7816->pdata->dvdd_power(1);
	#endif

	ret = anx7816_system_init();
	if (ret) {
		pr_err("%s: failed to initialize anx7816\n", __func__);
		goto err2;
	}

	#ifdef USING_HPD_FOR_POWER_MANAGEMENT
	client->irq = gpio_to_irq(anx7816->pdata->gpio_cbl_det);
	if (client->irq < 0) {
		pr_err("%s : failed to get gpio irq\n", __func__);
		goto err2;
	}

	ret = request_threaded_irq(client->irq, NULL, anx7816_cbl_det_isr,
				   IRQF_TRIGGER_RISING
				   | IRQF_TRIGGER_FALLING
				   | IRQF_ONESHOT, "anx7816", anx7816);
	if (ret < 0) {
		pr_err("%s : failed to request irq\n", __func__);
		goto err2;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for cable detect", __func__);
		pr_err("interrupt wake set fail\n");
		goto err3;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for cable detect", __func__);
		pr_err("interrupt wake enable fail\n");
		goto err3;
	}
	#endif
	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("%s : sysfs register failed", __func__);
		goto err3;
	}
	i2c_set_clientdata(client, anx7816);
	/*enable driver*/
	queue_delayed_work(anx7816->workqueue, &anx7816->work, 0);

	ret = slimport_dp_dev_init();
	if (ret < 0) {
		pr_err("%s : slimport_dp_dev_init failed", __func__);
		goto exit;
	}

	if(msm_hdmi_device_show_register(slimport_dp_hdmi_device_show)){
		pr_err("%s: func ret err\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	pr_info("%s %s end anx7816_i2c_probe\n", LOG_TAG, __func__);
	goto exit;

err3:
	#ifdef USING_HPD_FOR_POWER_MANAGEMENT
	free_irq(client->irq, anx7816);
	#endif
err2:
	destroy_workqueue(anx7816->workqueue);
err1:
	anx7816_free_gpio(anx7816);
err0:
	anx7816_client = NULL;
	kfree(anx7816);
exit:
	return ret;
}

static int anx7816_i2c_remove(struct i2c_client *client)
{
	struct anx7816_data *anx7816 = i2c_get_clientdata(client);
	int i = 0;
	slimport_dp_dev_exit();
	for (i = 0; i < ARRAY_SIZE(slimport_device_attrs); i++)
		device_remove_file(&client->dev, &slimport_device_attrs[i]);
	#ifdef USING_HPD_FOR_POWER_MANAGEMENT
	free_irq(client->irq, anx7816);
	#endif
	anx7816_free_gpio(anx7816);
	destroy_workqueue(anx7816->workqueue);
	kfree(anx7816);
	return 0;
}

bool is_slimport_vga(void)
{
	return 0;
}

/* 0x01: hdmi device is attached
    0x02: DP device is attached
    0x03: Old VGA device is attached // RX_VGA_9832
    0x04: new combo VGA device is attached // RX_VGA_GEN
    0x00: unknow device            */
EXPORT_SYMBOL(is_slimport_vga);
bool is_slimport_dp(void)
{
	return 0;
}
EXPORT_SYMBOL(is_slimport_dp);
unchar sp_get_link_bw(void)
{
	return sp_tx_cur_bw();
}
EXPORT_SYMBOL(sp_get_link_bw);
void sp_set_link_bw(unchar link_bw)
{
	sp_tx_set_bw(link_bw);
}
static int anx7816_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	struct anx7816_data *anx7816 = i2c_get_clientdata(client);

	pr_info("anx7816_i2c_suspend\n");
	cancel_delayed_work_sync(&anx7816->work);
	flush_workqueue(anx7816->workqueue);
	sp_tx_hardware_powerdown();
	sp_tx_clean_state_machine();

	return 0;
}

static int anx7816_i2c_resume(struct i2c_client *client)
{
	struct anx7816_data *anx7816 = i2c_get_clientdata(client);

	pr_info("anx7816_i2c_resume\n");
	queue_delayed_work(anx7816->workqueue, &anx7816->work, 0);

	return 0;
}
static const struct i2c_device_id anx7816_id[] = {
	{"anx7816", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, anx7816_id);

#ifdef CONFIG_OF
static struct of_device_id anx_match_table[] = {
	{.compatible = "analogix,anx7816",},
	{},
};
#endif

static struct i2c_driver anx7816_driver = {
	.driver = {
		   .name = "anx7816",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = anx_match_table,
#endif
		   },
	.probe = anx7816_i2c_probe,
	.remove = anx7816_i2c_remove,
	.suspend = anx7816_i2c_suspend,
	.resume = anx7816_i2c_resume,
	.id_table = anx7816_id,
};

static void __init anx7816_init_async(void *data, async_cookie_t cookie)
{
	int ret = 0;

	ret = i2c_add_driver(&anx7816_driver);
	if (ret < 0)
		pr_err("%s: failed to register anx7816 i2c drivern", __func__);
}

static int __init anx7816_init(void)
{
    async_schedule(anx7816_init_async, NULL);
	return 0;
}

static void __exit anx7816_exit(void)
{
	i2c_del_driver(&anx7816_driver);
}

module_init(anx7816_init);
module_exit(anx7816_exit);

MODULE_DESCRIPTION("Slimport  transmitter ANX7816 driver");
MODULE_AUTHOR("Junhua Xia <jxia@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
