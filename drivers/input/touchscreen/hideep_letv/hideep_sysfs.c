/*******************************************************************************
 * Copyright (C) 2014 HiDeep, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *******************************************************************************/

#include "hideep3d.h"
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#define HIDEEP3D_CHIP_ID_IST2001Z 0x44
#define HIDEEP3D_CHIP_ID_IST3001Z 0x42

static ssize_t fuse_ucode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct hideep3d_t *h3d_drv = dev_get_drvdata(dev);
	int ret;

	h3d_drv->manually_update = true;
	h3d_drv->h3d_fw_update_state=1;
	ret = hideep3d_load_ucode(dev, HIDEEP3D_MAN_FW);

	if (ret) {
		HIDEEP3D_ERR("The firmware update failed(%d)", ret);
		count = ret;
	}

	h3d_drv->manually_update = false;
	h3d_drv->h3d_fw_update_state=0;
	return count;
}

static ssize_t read_ucode(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct hideep3d_t *h3d = dev_get_drvdata(dev);

	len = scnprintf(buf, PAGE_SIZE, "%d\n", h3d->dwz_info->ver_c);
	return len;
}

static ssize_t loglevel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len;

	HIDEEP3D_INFO("enter");
	len = scnprintf(buf, PAGE_SIZE, "loglevel = %d, [0~2(normal log), 3(debug), 4(xy), 5(i2c)]\n", loglevel3d);
	HIDEEP3D_INFO("loglevel =%d ",loglevel3d);

	return len;
}

static ssize_t loglevel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct hideep3d_t *h3d = dev_get_drvdata(dev);

	HIDEEP3D_INFO("enter, loglevel =%d", loglevel3d);

	mutex_lock(&h3d->dev_mutex);
	sscanf (buf,"%d", &loglevel3d);
	mutex_unlock(&h3d->dev_mutex);

    return count;
}

static ssize_t r_vr_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 len = 0;
	u32 vr_data = 0;
	struct hideep3d_t *h3d = dev_get_drvdata(dev);

	hideep3d_i2c_read(h3d, h3d->vr_addr, h3d->vr_size, (u8*)&vr_data);

	len = scnprintf(buf, PAGE_SIZE, "vr : %d %d(%02x)\n", h3d->vr_addr, vr_data, vr_data);
	return len;
}

static ssize_t w_vr_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct hideep3d_t *h3d = dev_get_drvdata(dev);
	u32 vr_addr = 0;
	u32 vr_data = 0;
	u32 vr_size = 0;
	u32 vr_flag = 0;

	sscanf (buf,"%d %d %d %d", &vr_addr, &vr_data, &vr_size, &vr_flag);

	if (vr_addr >= HIDEEP3D_EVENT_COUNT_ADDR)
		return 0;
	if (vr_size >  sizeof(vr_data))
		return 0;

	h3d->vr_addr = vr_addr;
	h3d->vr_size = vr_size;

	if (vr_flag != 0)
		hideep3d_i2c_write(h3d, vr_addr, vr_size, (u8*)&vr_data);

	return count;
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 len = 0;
	char *panel_name = "";
	struct hideep3d_t *h3d = dev_get_drvdata(dev);

	mutex_lock(&h3d->dev_mutex);
	hideep3d_load_dwz(h3d);

	HIDEEP3D_INFO("boot version : %04x", h3d->dwz_info->ver_b);
	HIDEEP3D_INFO("core version : %04x", h3d->dwz_info->ver_c);
	HIDEEP3D_INFO("custom version : %04x", h3d->dwz_info->ver_d);
	HIDEEP3D_INFO("vr version : %04x", h3d->dwz_info->ver_v);
	HIDEEP3D_INFO("factory ID : %02x", h3d->dwz_info->factory_id);

	panel_name = "None";

	len = scnprintf(buf, PAGE_SIZE,
		"boot ver: %04x\ncore ver: %04x\ncustom ver: %04x\nvr ver: %04x\nfactory ID : %02x\npanel company : %s\nD/D ver : %d.%02d\nDescription : %s\n",
		h3d->dwz_info->ver_b, h3d->dwz_info->ver_c, h3d->dwz_info->ver_d,
		h3d->dwz_info->ver_v, h3d->dwz_info->factory_id, panel_name, HIDEEP3D_DD_VERSION_MAJOR, HIDEEP3D_DD_VERSION_MINOR, HIDEEP3D_DD_DESCRIPTION);
	mutex_unlock(&h3d->dev_mutex);
	return len;
}


static ssize_t power_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hideep3d_t *h3d = dev_get_drvdata(dev);
	int len;

	len = scnprintf(buf, PAGE_SIZE, "power status : %s\n", (h3d->dev_state==power_init)?"off":"on");

	return len;
}

static ssize_t power_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct hideep3d_t *h3d = dev_get_drvdata(dev);
	int on;

	sscanf(buf, "%d", &on);
	if (on) {
		hideep3d_power(h3d, on);
		h3d->dev_state = power_normal;
		hideep3d_reset_ic(h3d);
	} else {
		hideep3d_power(h3d, on);
		h3d->dev_state = power_init;
	}

	return count;
}

static ssize_t self_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct hideep3d_t *h3d = dev_get_drvdata(dev);
	int mode, ret;
	unsigned char val;
	unsigned short addr = 0;

	sscanf(buf, "%d", &mode);
	HIDEEP3D_DBG("Mode : %d", mode);

	switch (mode) {
		case 0:
			val = 0x01;
			addr = HIDEEP_SELF_MODE1;
		break;
		case 1:
			val = 0x06;
			addr = HIDEEP_SELF_MODE2;
		break;
		case 2:
			val = 0x07;
			addr = HIDEEP_SELF_MODE2;
		break;
		case 3:
			val = 0x08;
			addr = HIDEEP_SELF_MODE2;
		break;
		default:
			HIDEEP3D_ERR("Undefine mode");
		break;
	}

	ret = hideep3d_i2c_write(h3d, addr, 1, &val);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t chipid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u32 len = 0;
	unsigned int p_code;
	char *product_name;
	struct hideep3d_t *h3d = dev_get_drvdata(dev);

	p_code = h3d->dwz_info->product_code;

	if (p_code == HIDEEP3D_CHIP_ID_IST2001Z) {
		product_name = "iST2001Z";
	}else if (p_code == HIDEEP3D_CHIP_ID_IST3001Z) {
		product_name = "iST3001Z";
	} else {
		product_name = "None";
	}

	len = scnprintf(buf, PAGE_SIZE, "Chip ID : %s\n", product_name);
    return len;
}

static DEVICE_ATTR(update, 0644, read_ucode, fuse_ucode);
static DEVICE_ATTR(vr_data, 0644, r_vr_data, w_vr_data);
static DEVICE_ATTR(version, 0444, version_show, NULL);
static DEVICE_ATTR(loglevel, 0644, loglevel_show, loglevel_store);
static DEVICE_ATTR(power_en, 0644, power_status, power_control);
static DEVICE_ATTR(selftest, 0660, NULL, self_mode);
static DEVICE_ATTR(chipid, 0444, chipid_show, NULL);

static struct attribute *hideep_3d_sysfs_entries[] = {
	&dev_attr_update.attr,
	&dev_attr_vr_data.attr,
	&dev_attr_version.attr,
	&dev_attr_loglevel.attr,
	&dev_attr_power_en.attr,
	&dev_attr_selftest.attr,
	&dev_attr_chipid.attr,
	NULL
};

static struct attribute_group hideep_3d_attr_group = {
	.attrs  = hideep_3d_sysfs_entries,
};

int hideep3d_sysfs_init(struct hideep3d_t *h3d)
{
	int ret;
	struct  i2c_client *client = h3d->client;

	/* Create the files associated with this kobject */
	ret = sysfs_create_group(&client->dev.kobj, &hideep_3d_attr_group);

	HIDEEP3D_INFO("device : %s ", client->dev.kobj.name);
	ret = sysfs_create_link(NULL, &client->dev.kobj,"hideep3d");

	if (ret) {
		HIDEEP3D_ERR("%s: Fail create link error = %d\n", __func__, ret);
	}

	return ret;
}

int hideep3d_sysfs_exit(struct hideep3d_t *h3d)
{
	struct  i2c_client *client = h3d->client;

	sysfs_remove_link(&client->dev.kobj, "hideep3d");
	sysfs_remove_group(&client->dev.kobj, &hideep_3d_attr_group);

	return 0;
}

