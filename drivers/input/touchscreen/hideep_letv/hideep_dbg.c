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
#include "hideep_dbg.h"
#include <linux/poll.h>

#ifdef HIDEEP_DEBUG_DEVICE

static int hideep_get_image(struct hideep3d_t *h3d, u32 addr, u32 len)
{
	int ret = 0;
	struct hideep_debug_dev_t *debug_dev = &h3d->debug_dev;

	mutex_lock(&h3d->dev_mutex);
	ret = hideep3d_i2c_read(h3d, addr, len, debug_dev->im_buff);
	mutex_unlock(&h3d->dev_mutex);
	if (ret < 0)
		goto i2c_err;

	return ret;
i2c_err:
	HIDEEP3D_ERR("%s(%d) : i2c_err", __func__, __LINE__);
	return ret;
}

static int hideep_get_vreg(struct hideep3d_t *h3d, u32 addr, u32 len)
{
	int32_t ret = 0;
	struct hideep_debug_dev_t *debug_dev = &h3d->debug_dev;

	mutex_lock(&h3d->dev_mutex);
	ret = hideep3d_i2c_read(h3d, addr, len, debug_dev->vr_buff);
	mutex_unlock(&h3d->dev_mutex);
	if (ret < 0)
		goto i2c_err;

	HIDEEP3D_INFO("hideep_get_vreg(0x%02x:%d)", addr, len);
	return ret;

i2c_err:
	HIDEEP3D_ERR("%s(%d) : i2c_err", __func__, __LINE__);
	return ret;
}

static int hideep_set_vreg(struct hideep3d_t *h3d, u32 addr, u32 len)
{
	int32_t ret = 0;
	struct hideep_debug_dev_t *debug_dev = &h3d->debug_dev;
	int32_t wr_remain = len;
	u32 vr_addr = addr;
	int32_t wr_len = len;
	uint8_t *buff = debug_dev->vr_buff;

	do {
		if (wr_remain >=  MAX_VR_BUFF)
			wr_len = MAX_VR_BUFF;
		else
			wr_len = wr_remain;

		ret = hideep3d_i2c_write(h3d, vr_addr, wr_len, buff);
		if (ret < 0)
			goto i2c_err;

		wr_remain -= MAX_VR_BUFF;
		vr_addr += MAX_VR_BUFF;
		buff += MAX_VR_BUFF;
	} while(wr_remain > 0);

	HIDEEP3D_INFO("hideep_set_vreg(0x%02x:%d)", addr, len);
	return ret;

i2c_err:
	HIDEEP3D_ERR("i2c_err");
	return ret;
}

static size_t
hideep_download_uc(struct hideep3d_t *h3d, const char __user *uc, size_t count, int offset)
{
	int ret;
	unsigned char *ucode;
	int len = offset;

	HIDEEP3D_INFO("%s count = %d, offset = %d", __func__, (int)count, offset);

	len += count;
	len -= 47 * 1024;
	if(len > 1024)
		return -1;

	ucode = kmalloc((47 * 1024) + len + count, GFP_KERNEL);

	ret = copy_from_user(ucode + offset, uc, count);
	if (ret < 0) {
		HIDEEP3D_ERR("ADDR_UC : copy_to_user");
		kfree(ucode);
		return 0;
	}

	hideep3d_fuse_ucode(h3d->client, ucode, count, offset);
	kfree(ucode);

	HIDEEP3D_INFO("Download_uc(%d)", (int)count);

	return count;
}

static int hideep_debug_open(struct inode *inode, struct file *file)
{
	struct hideep_debug_dev_t *dev_info;

	dev_info = container_of(inode->i_cdev, struct hideep_debug_dev_t, cdev);
	if (dev_info == NULL) {
		HIDEEP3D_ERR("No such char device node");
		return -ENODEV;
	}

	dev_info->release_flag = false;
	dev_info->h3d->z_flag_calib2 = false;

	file->private_data = dev_info;
	HIDEEP3D_INFO("hideep_debug_open");

	return 0;
}

static int hideep_debug_release(struct inode *inode, struct file *file)
{
	struct hideep_debug_dev_t *dev_info;
	dev_info = container_of(inode->i_cdev, struct hideep_debug_dev_t, cdev);

	HIDEEP3D_INFO("%s", __func__);
	if (!dev_info->release_flag)
		return -1;
	dev_info->release_flag = false;
	file->private_data = NULL;
	return 0;
}

static unsigned int hideep_debug_poll(struct file *file, struct poll_table_struct *wait)
{
	u32 mask = 0;

	HIDEEP3D_INFO("%s", __func__);
	if (file->private_data == NULL)
		return 0;

	return mask;
}

static ssize_t hideep_debug_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	struct hideep_debug_dev_t *drv_info = file->private_data;
	ssize_t rd_len = 0;
	uint8_t* rd_buffer = NULL;
	unsigned short ver = 0x0;
	u16* p_z;

	HIDEEP3D_INFO("%s count = %d", __func__, (int)count);
	if (file->private_data == NULL)
		return 0;

	drv_info->vr_size = count;
	rd_buffer = drv_info->vr_buff;
	rd_len = count;

	if ((*offset) < ADDR_VR_END) {
		if ((*offset) == ADDR_IMG) {
			drv_info->im_size = count;
			ret = hideep_get_image(drv_info->h3d, ADDR_IMG, drv_info->im_size);
			rd_buffer = drv_info->im_buff;
			rd_len = count;
			ret = 0;
		} else if ((HIDEEP_Z_VALUE == (*offset)) && (count == 2) && (drv_info->h3d->z_status)) {
			drv_info->h3d->z_status = false;
            p_z = (u16*)drv_info->vr_buff;
			*p_z = drv_info->h3d->z_buffer;
			rd_len = 2;
			ret = 0;
		} else if ((HIDEEP_Z_CALIB2 == (*offset)) &&
				(((drv_info->h3d->z_calib_end-drv_info->h3d->z_calib_start+1)*2+2) == count) && (drv_info->h3d->z_flag_ready)){
			rd_buffer = drv_info->vr_buff;
			drv_info->h3d->z_flag_calib2 = false;
			drv_info->h3d->z_flag_ready = false;
			memcpy(rd_buffer+0, &drv_info->h3d->z_index, 2);
			memcpy(rd_buffer+2, &drv_info->h3d->z_data[drv_info->h3d->z_calib_start], count-2);
			rd_len = count;
			ret = 0;
		} else {
			ret = hideep_get_vreg(drv_info->h3d, *offset, rd_len);
		}
		if(ret < 0)
			rd_len = 0;
	} else if ((*offset) == HIDEEP_VERSION_INFO) {
		rd_buffer = drv_info->vr_buff;
		// DD, boot, core, custom, vr :total 10 byte
		// vr
		memcpy(rd_buffer+0, &drv_info->h3d->dwz_info->ver_v, 2);
		// custom
		memcpy(rd_buffer+2, &drv_info->h3d->dwz_info->ver_d, 2);
		// core
		memcpy(rd_buffer+4, &drv_info->h3d->dwz_info->ver_c, 2);
		// boot
		memcpy(rd_buffer+6, &drv_info->h3d->dwz_info->ver_b, 2);
		// DD
		ver = (HIDEEP3D_DD_VERSION_MAJOR & 0xff) << 8;
		ver = ver | (HIDEEP3D_DD_VERSION_MINOR & 0xf);
		memcpy(rd_buffer+8, &ver, 2);
		rd_len = count;
		ret = 0;
	} else {
		HIDEEP3D_ERR("hideep_read : undefined address");
		return 0;
	}

	ret = copy_to_user(buf, rd_buffer, rd_len);
	if (ret < 0) {
		HIDEEP3D_ERR("error : copy_to_user");
		return -EFAULT;
	}

	return rd_len;
}

static ssize_t hideep_debug_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	struct hideep_debug_dev_t *drv_info = file->private_data;
	int wr_len = 0;

	HIDEEP3D_INFO("%s count = %d", __func__, (int)count);
	if (file->private_data == NULL)
		return 0;

	if ((*offset) < ADDR_VR_END) {
        wr_len = count;

		ret = copy_from_user(drv_info->vr_buff, buf, wr_len);
		if (ret < 0) {
			HIDEEP3D_ERR("error : copy_to_user");
			return -EFAULT;
		}

		ret = hideep_set_vreg(drv_info->h3d, *offset, wr_len);
		if(ret < 0)
			wr_len = 0;
	} else if ((*offset & ADDR_UC) == ADDR_UC) {
		wr_len = hideep_download_uc(drv_info->h3d, buf, count, *offset&0xffff);
    } else {
		HIDEEP3D_ERR("hideep_write : undefined address, 0x%08x", (int)*offset);

		return 0;
    }

	return wr_len;
}

static loff_t hideep_debug_llseek(struct file *file, loff_t off, int whence)
{
	loff_t newpos;
	struct hideep_debug_dev_t *drv_info = file->private_data;

	HIDEEP3D_INFO("%s off = 0x%08x, whence = %d", __func__, (unsigned int)off, whence);
	if(file->private_data == NULL)
		return -EFAULT;

	switch (whence) {
		/* SEEK_SET */
		case 0:
			newpos = off;
			break;
		/* SEEK_CUR */
		case 1:
			HIDEEP3D_INFO("%s set mode off = 0x%08x", __func__, (unsigned int)off);
			if (off == 0x1000) {
				drv_info->vr_buff[0] = 0x8f;  // select frame mode...
				hideep_set_vreg(drv_info->h3d, 0x00, 1);
			} else if (off == 0x240) {
				drv_info->vr_buff[0] = HIDEEP_OPM_TOUCH_A;
				hideep_set_vreg(drv_info->h3d, 0x00, 1);
			} else if (off == HIDEEP_RELEASE_FLAG) {
				HIDEEP3D_DBG("set release flag");
				drv_info->release_flag = true;
				newpos = file->f_pos;
			} else if ((off & HIDEEP_Z_CALIB2_READ) == HIDEEP_Z_CALIB2_READ) {
				drv_info->h3d->z_calib_start = off & 0x0fff;
				drv_info->h3d->z_calib_end = (off >> 16) & 0x0fff;
				if ((drv_info->h3d->z_calib_end -drv_info->h3d->z_calib_start+1) > (1024*4)) {
					HIDEEP3D_ERR(" set the frames is oversize\n");
					return -EINVAL;
				}
				HIDEEP3D_DBG("set calib2 start = %d, end = %d\n", drv_info->h3d->z_calib_start, drv_info->h3d->z_calib_end);
				drv_info->h3d->z_flag_calib2 = true;
				drv_info->h3d->z_flag_ready = false;
				drv_info->h3d->z_index = 0;
				memset(drv_info->h3d->z_data, 0x0, 4096*2);
				newpos=off;
			} else {
				newpos = file->f_pos;
			}
			break;
		/* SEEK_END */
		case 2:
			drv_info->im_size = off & 0xffff;
			drv_info->vr_size = (off >> 16) & 0xffff;
			HIDEEP3D_INFO("%s HIDEEP_DEBUG_CFG : %d", __func__, drv_info->im_size);
			HIDEEP3D_INFO("%s HIDEEP_DEBUG_CFG : %d", __func__, drv_info->vr_size);
			newpos = file->f_pos;
			break;
		default:
			return -EINVAL;
	}

	if (newpos < 0)
		return -EINVAL;

	file->f_pos = newpos;

	return newpos;
}

static long hideep_debug_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct hideep_debug_dev_t *drv_info = file->private_data;
	struct hideep_debug_cfg_t config;
	int err = 0;
	int ret = 0;
	int opmode;

	void __user *argp = (void __user *) arg;

	/*
	* extract the type and number bitfields, and don't decode
	* wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	*/
	HIDEEP3D_ERR("%s", __func__);

	if (file->private_data == NULL)
		return 0;

	if (_IOC_TYPE(cmd) != HIDEEP_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > HIDEEP_IOC_MAXNR)
		return -ENOTTY;

	/*
	* the direction is a bitmask, and VERIFY_WRITE catches R/W
	* transfers. `Type' is user-oriented, while
	* access_ok is kernel-oriented, so the concept of "read" and
	* "write" is reversed
	*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case HIDEEP_CFG:
			ret = copy_from_user(&config, argp, sizeof(struct hideep_debug_cfg_t));

			drv_info->im_size = config.im_size;
			drv_info->vr_size = config.vr_size;

			HIDEEP3D_INFO("HIDEEP_DEBUG_CFG : %d", drv_info->im_size);
			HIDEEP3D_INFO("HIDEEP_DEBUG_CFG : %d", drv_info->vr_size);
			break;
		case HIDEEP_DEBUG_MODE:
			//opmode 8f setting
			ret = copy_from_user(&opmode, argp, sizeof(argp));
			if (opmode) {
				//op mode 8f on
				drv_info->vr_buff[0] = HIDEEP_OPM_RAW;
				drv_info->enable = true;
				HIDEEP3D_INFO("RAW_MODE");
			} else {
				//op mode 00 on
				drv_info->vr_buff[0] = HIDEEP_OPM_TOUCH_A;
				drv_info->enable =  false;
				HIDEEP3D_INFO("TOUCH_MODE");
			}
			ret = hideep_set_vreg(drv_info->h3d, 0x00, 1);
			break;
		case HIDEEP_GET_DATA:
			// read vr and copy to user
			ret = hideep_get_image(drv_info->h3d, ADDR_IMG, drv_info->im_size);
			ret = copy_to_user(argp, drv_info->im_buff, drv_info->im_size);
			break;
		default:
			return -ENOTTY;
	}

	return ret;
}

static const struct file_operations hideep_debug_fops = {
	.owner = THIS_MODULE,
	.open = hideep_debug_open,
	.poll = hideep_debug_poll,
	.release = hideep_debug_release,
	.read = hideep_debug_read,
	.write = hideep_debug_write,
	.llseek = hideep_debug_llseek,
	.unlocked_ioctl = hideep_debug_ioctl,
};

static void hideep_debug_unregister(struct hideep3d_t *h3d)
{
	struct hideep_debug_dev_t *dev  = &h3d->debug_dev;

	device_destroy(h3d->debug_class, h3d->debug_dev_no);
	cdev_del(&dev->cdev);

	if(dev->im_buff)
		kfree(dev->im_buff);
	if(dev->vr_buff)
		kfree(dev->vr_buff);
#ifdef HIDEEP_SELFTEST_MODE
	if(dev->self_buff)
		kfree(dev->self_buff);
#endif

	return;
}

static int hideep_debug_register(struct hideep3d_t *h3d, u32 minor)
{
	int err = 0;
	struct device *device = NULL;
	struct hideep_debug_dev_t *dev = &h3d->debug_dev;
	dev_t devno = h3d->debug_dev_no;

	cdev_init(&dev->cdev, &hideep_debug_fops);
	dev->cdev.owner = THIS_MODULE;

	err = cdev_add(&dev->cdev, h3d->debug_dev_no, 1);
	if (err) {
		goto err;
	}

    device = device_create(h3d->debug_class, NULL, devno, NULL, HIDEEP3D_DEBUG_DEVICE_NAME);
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		cdev_del(&dev->cdev);
		goto err;
	}

	return 0;

err:
	HIDEEP3D_ERR("hideep_debug_register failed");
	return err;
}

void hideep3d_debug_uninit(struct hideep3d_t *h3d)
{
	/* Get rid of character devices (if any exist) */
	hideep_debug_unregister(h3d);

	if(h3d->debug_class)
		class_destroy(h3d->debug_class);

	unregister_chrdev_region(h3d->debug_dev_no, 1);
	return;
}

int hideep3d_debug_init(struct hideep3d_t *h3d)
{
	int ret = 0;
	dev_t dev_id = 0;
	struct hideep_debug_dev_t *debug_drv = &h3d->debug_dev;

	ret = alloc_chrdev_region(&dev_id, 0, 1, HIDEEP3D_DEBUG_DEVICE_NAME);
	if (ret < 0) {
		return ret;
	}

	h3d->debug_dev_no = dev_id;
	h3d->debug_class  = class_create(THIS_MODULE, HIDEEP3D_DEBUG_DEVICE_NAME);
	if (IS_ERR(h3d->debug_class)) {
		ret = PTR_ERR(h3d->debug_class);
		goto fail;
	}

	ret = hideep_debug_register(h3d, 0);
	if (ret) {
		goto fail;
	}

	debug_drv->h3d = h3d;
	hideep3d_i2c_read(h3d, 0x8000, 1, (unsigned char*)&debug_drv->tx_num);
	hideep3d_i2c_read(h3d, 0x8001, 1, (unsigned char*)&debug_drv->rx_num);
	HIDEEP3D_DBG("TX_NUM : %d, RX_NUM : %d", debug_drv->tx_num, debug_drv->rx_num);
	debug_drv->im_size = (debug_drv->tx_num * debug_drv->rx_num * 2 * 3) + 12;
	debug_drv->vr_size = 4096;
	debug_drv->im_buff = kmalloc(debug_drv->im_size, GFP_KERNEL);
	debug_drv->vr_buff = kmalloc(debug_drv->vr_size, GFP_KERNEL);

	if (!debug_drv->im_buff || !debug_drv->vr_buff) {
		goto fail;
	}

	HIDEEP3D_DBG("hideep_debug_init....");
	return 0;

fail:
	hideep3d_debug_uninit(h3d);

	pr_err("%s failed", __func__);

	return ret;
}
#endif

