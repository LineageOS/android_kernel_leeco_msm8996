/*
  SiI6400 Linux Driver

  Copyright (C) 2012-2013 Silicon Image, Inc.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation version 2.

  This program is distributed "AS-IS" WITHOUT ANY WARRANTY of any
  kind, whether express or implied; INCLUDING without the implied warranty
  of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
  See the GNU General Public License for more details at
  http://www.gnu.org/licenses/gpl-2.0.html.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/mdss_io_util.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <linux/stat.h>

#ifdef ENABLE_MHL_UEVENT
#include <linux/power/mhl_uevent.h>
#endif

#include "osal.h"
#include "hal.h"
#include "device.h"
#include "sii6400.h"
#include "state_machine.h"
#include "wihd_sm.h"
#include "mhl_sm.h"
#include "diag_sm.h"
#include "host_msg.h"

/* Module information */
MODULE_DESCRIPTION("Silicon Image 6400 driver");
MODULE_AUTHOR("Silicon Image <http://www.siliconimage.com>");
MODULE_LICENSE("GPL");

/* Default version string */
#ifndef SII6400_DRIVER_VERSION
#define SII6400_DRIVER_VERSION "unknown"
#endif

/* Device variables */
static struct sii6400_module_info sii6400_modinfo;
struct device *sii6400_device;

struct wake_lock sii6400_change_mode_wakelock;
static bool module_is_exiting;

/* A mutex will ensure that only one process accesses our device */
static DEFINE_MUTEX(sii6400_device_mutex);

#ifdef SII6400_COMBINED_FIRMWARE
/* WiHD_MHL firmware filename */
char *wihd_mhl_firmware = DEFAULT_WIHD_MHL_FIRMWARE_FILENAME;
#else /* SII6400_COMBINED_FIRMWARE */
/* WiHD firmware filename */
char *wihd_firmware = DEFAULT_WIHD_FIRMWARE_FILENAME;
/* MHL firmware filename */
char *mhl_firmware = DEFAULT_MHL_FIRMWARE_FILENAME;
#endif /* SII6400_COMBINED_FIRMWARE */

/* Module parameters that can be provided on insmod */

/* control debug messages sent to kernel log */
int debug_level = DEBUG_LEVEL_WARNINGS;
module_param(debug_level, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "debug level (default: 0)");

/* Non-Volatile Storage filepath */
#ifdef SII6400_CONFIGDATA_FILE
char *configdata_file = (char *)SII6400_CONFIGDATA_FILE;
#else
char *configdata_file = "";
#endif
module_param(configdata_file, charp, S_IRUGO);
MODULE_PARM_DESC(configdata_file,
			"Non-Volatile Storage filepath (no default value)");

/* Support for transcode mode */
static bool transcode_mode;
module_param(transcode_mode, bool, S_IRUGO);
MODULE_PARM_DESC(transcode_mode, "Support transcode mode (default: false)");

/* Remote Control Input Device */
static int input_dev_remote_control = INPUT_DEVICE_DEFAULT_VALUE;
module_param(input_dev_remote_control, int, S_IRUGO);
MODULE_PARM_DESC(input_dev_remote_control,
			"Remote Control Input Device (default: 1)");

/* RAP Input Device */
static int input_dev_rap = INPUT_DEVICE_DEFAULT_VALUE;
module_param(input_dev_rap, int, S_IRUGO);
MODULE_PARM_DESC(input_dev_rap, "RAP Input Device (default: 1)");

/* RCP Input Device */
static int input_dev_rcp = INPUT_DEVICE_DEFAULT_VALUE;
module_param(input_dev_rcp, int, S_IRUGO);
MODULE_PARM_DESC(input_dev_rcp, "RCP Input Device (default: 1)");

/* UCP Input Device */
static int input_dev_ucp = INPUT_DEVICE_DEFAULT_VALUE;
module_param(input_dev_ucp, int, S_IRUGO);
MODULE_PARM_DESC(input_dev_ucp, "UCP Input Device (default: 1)");

/* Firmware Debug Log filepath */
static char *debug_logfile;

/* static function prototypes */
static int sii6400_device_open(struct inode *inode, struct file *filp);
static int sii6400_device_close(struct inode *inode, struct file *filp);
static ssize_t get_sii6400_chip_version(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_fw_version(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_mode(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_debug_level(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_debug_level(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_diag_cmd(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t send_sii6400_diag_cmd(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_diag_cmd_block(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t send_sii6400_diag_cmd_block(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_diag_cmd_output(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_diag_logfile(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_diag_logfile(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#ifdef SII6400_COMBINED_FIRMWARE
static enum sii_os_status send_output_mode_to_firmware(
						enum sii6400_mode new_mode);
#endif /* SII6400_COMBINED_FIRMWARE */
static int __init sii6400_module_init(void);
static void __exit sii6400_module_exit(void);


int send_sii6400_uevent(struct device *device, const char *event_cat,
			const char *event_type, const char *event_data)
{
	int retval = 0;
	int rv = 0;
	char *event_string = NULL;

	dbg("");

	if ((NULL == device) || (NULL == event_cat) || (NULL == event_type)) {
		err("Invalid parameters\n");
		retval = -EINVAL;
		goto failed_nullptr;
	}

	event_string = SiiOsCalloc("UEvent", MAX_EVENT_STRING_LEN, 0);
	if (NULL == event_string) {
		err("Out of memory\n");
		retval = -ENOMEM;
		goto failed_memalloc;
	}

	rv = scnprintf(event_string, MAX_EVENT_STRING_LEN,
			"%s={\"event\":\"%s\",\"data\":%s}",
			event_cat, event_type,
			(NULL != event_data) ? event_data : "null");
	if (0 < rv) {
		char *envp[] = {event_string, NULL};

		warn("%s: event_string=%s\n", __func__, event_string);
		(void)kobject_uevent_env(&device->kobj, KOBJ_CHANGE, envp);
	} else {
		err("scnprintf failed\n");
		retval = -EFAULT;
	}

	SiiOsFree(event_string);
failed_memalloc:
failed_nullptr:
	return retval;
}

/*
 * Get pointer to device info structure.
 */
struct sii6400_device_info *get_sii6400_devinfo(void)
{
	return sii6400_modinfo.sii6400_devinfo;
}

/*
 * Format the WIHD state string.
 */
const char *get_wihd_state_string(enum sii6400_wihd_state wihd_state)
{
	const char *wihd_state_str = "";

	switch (wihd_state) {
	case WIHD_STATE_IDLE:
		wihd_state_str = "idle";
		break;

	case WIHD_STATE_WVAN_SCAN:
		wihd_state_str = "wvan_scan";
		break;

	case WIHD_STATE_ASSOCIATING:
		wihd_state_str = "associating";
		break;

	case WIHD_STATE_ASSOCIATED:
		wihd_state_str = "associated";
		break;

	case WIHD_STATE_CONNECTING:
		wihd_state_str = "connecting";
		break;

	case WIHD_STATE_CONNECTED:
		wihd_state_str = "av_enabled";
		break;

	default:
	case WIHD_STATE_NONE:
		wihd_state_str = "none";
		break;
	}

	return wihd_state_str;
}

/*
 * Format the category string.
 */
const char *get_category_string(struct sii6400_dev_category *device_category)
{
	const char *category_str = "";

	if (NULL == device_category)
		goto done;

	switch (device_category->category) {
	case WIHD_CATEGORY_DTV:
		category_str = "DTV";
		break;

	case WIHD_CATEGORY_BD_DVD_PLAYER:
		category_str = "BD/DVD player";
		break;

	case WIHD_CATEGORY_BD_DVD_RECORDER:
		category_str = "BD/DVD recorder";
		break;

	case WIHD_CATEGORY_STB:
		category_str = "STB";
		break;

	case WIHD_CATEGORY_AUDIO_AMP:
		category_str = "Audio amplifier";
		break;

	case WIHD_CATEGORY_HDD_RECORDER:
		category_str = "HDD recorder";
		break;

	case WIHD_CATEGORY_MONITOR:
		category_str = "Monitor";
		break;

	case WIHD_CATEGORY_PROJECTOR:
		category_str = "Projector";
		break;

	case WIHD_CATEGORY_PC:
		category_str = "PC";
		break;

	case WIHD_CATEGORY_CAMCORDER:
		category_str = "Camcorder";
		break;

	case WIHD_CATEGORY_GAME:
		category_str = "Game";
		break;

	case WIHD_CATEGORY_ADAPTER:
		category_str = "Adapter";
		break;

	case WIHD_CATEGORY_MOBILE_PHONE:
		category_str = "Mobile phone";
		break;

	case WIHD_CATEGORY_PDA:
		category_str = "PDA";
		break;

	case WIHD_CATEGORY_PMP:
		category_str = "PMP";
		break;

	case WIHD_CATEGORY_DSC:
		category_str = "DSC";
		break;

	case WIHD_CATEGORY_OTHER:
		category_str = "Other";
		break;

	case WIHD_CATEGORY_RESERVED:
	default:
		category_str = "Reserved";
		break;
	}

done:
	return category_str;
}

bool sii6400_mode_is_off(enum sii6400_mode mode)
{
	return (SII6400_MODE_OFF == mode) ? true : false;
}

bool sii6400_mode_is_config(enum sii6400_mode mode)
{
	return (SII6400_MODE_CONFIG == mode) ? true : false;
}

bool sii6400_mode_is_wihd(enum sii6400_mode mode)
{
	return (SII6400_MODE_WIHD == mode) ? true : false;
}

bool sii6400_mode_is_mhl(enum sii6400_mode mode)
{
	return (SII6400_MODE_MHL == mode) ? true : false;
}

bool sii6400_char_string_is_valid(const char *data, size_t max_strlen)
{
	bool retval = false;
	const char *cur = data;

	if (NULL == data)
		goto done;

	if (max_strlen < strlen(data))
		goto done;

	for (cur = data; '\0' != *cur; ++cur) {
		if (isprint(*cur))
			continue;
		if (isspace(*cur))
			continue;
		goto done; /* not acceptable */
	}
	retval = true;

done:
	return retval;
}

bool sii6400_utf8_data_is_valid(const unsigned char *data, size_t size)
{
	bool retval = false;
	const unsigned char *cur = NULL;
	const unsigned char *end = NULL;

	if (NULL == data)
		goto done;
	cur = data;
	end = data + size;

	for (cur = data; cur < end; ++cur) {
		if (0xf5 <= *cur)
			goto done;
		if (0xc0 == *cur)
			goto done;
		if (0xc1 == *cur)
			goto done;
	}
	retval = true;

done:
	return retval;
}

static int sii6400_device_open(struct inode *inode, struct file *filp)
{
	int retval = 0;

	dbg("");

#ifndef SII6400_DEBUG
	if (NULL == filp) {
		retval = -EFAULT;
		goto done;
	}

	/* The Sii6400 device does not allow write access */
	if (((filp->f_flags & O_ACCMODE) == O_WRONLY) ||
	    ((filp->f_flags & O_ACCMODE) == O_RDWR)) {
		retval = -EACCES;
		goto done;
	}
#endif

	/* Ensure that only one process has access to the
	 * Sii6400 device at any one time. */
	if (!mutex_trylock(&sii6400_device_mutex)) {
		retval = -EBUSY;
		goto done;
	}

#ifdef SII6400_DEBUG
	retval = sii6400_device_dbg_open(inode, filp);
#endif

done:
	return retval;
}

static int sii6400_device_close(struct inode *inode, struct file *filp)
{
	int retval = 0;

	dbg("");

#ifdef SII6400_DEBUG
	retval = sii6400_device_dbg_close(inode, filp);
#endif

	mutex_unlock(&sii6400_device_mutex);
	return retval;
}

static const struct file_operations sii6400_fops = {
	.open = sii6400_device_open,
	.release = sii6400_device_close
};

/*
 * Sii6400 Attributes
 */
static ssize_t get_sii6400_chip_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_chip_version *chip_version = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		goto done;
	}

	switch (devinfo->mode) {
	case SII6400_MODE_WIHD:
	{
		sm_rv = sm_send_request(wihd_sm_queue,
					SII_SM_REQ_GET_CHIP_VERSION,
					NULL, 0,
					get_wihd_chip_version_resp_q,
					WAIT_TIME_FOR_GET_CHIP_VERSION_RESPONSE,
					&resp_data, &resp_data_size);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("Timed out waiting for WiHD State Machine resp\n");
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("WiHD State Machine request send failure\n");
			goto done;
		}

		if ((NULL != resp_data) &&
		    (sizeof(*chip_version) == resp_data_size)) {
			chip_version = resp_data;
		} else {
			err("Invalid WiHD State Machine response\n");
			goto done;
		}
	}
		break;

	case SII6400_MODE_MHL:
	{
		sm_rv = sm_send_request(mhl_sm_queue,
					SII_SM_REQ_GET_CHIP_VERSION,
					NULL, 0,
					get_mhl_chip_version_resp_q,
					WAIT_TIME_FOR_GET_CHIP_VERSION_RESPONSE,
					&resp_data, &resp_data_size);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("Timed out waiting for MHL State Machine resp\n");
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("MHL State Machine request send failure\n");
			goto done;
		}

		if ((NULL != resp_data) &&
		    (sizeof(*chip_version) == resp_data_size)) {
			chip_version = resp_data;
		} else {
			err("Invalid MHL State Machine response\n");
			goto done;
		}
	}
		break;

	case SII6400_MODE_CONFIG:
	case SII6400_MODE_OFF:
	case SII6400_MODE_UNDEFINED:
	default:
		break;
	}

done:
	if (NULL != chip_version) {
		if (NULL != chip_version->chip_version) {
			rv = scnprintf(buf, PAGE_SIZE, "%s",
						chip_version->chip_version);
			SiiOsFree(chip_version->chip_version);
			chip_version->chip_version = NULL;
		} else {
			rv = scnprintf(buf, PAGE_SIZE, "%s", "");
		}
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_fw_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_firmware_version *firmware_version = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		goto done;
	}

	switch (devinfo->mode) {
	case SII6400_MODE_WIHD:
	{
		sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_FIRMWARE_VERSION,
				NULL, 0,
				get_wihd_firmware_version_resp_q,
				WAIT_TIME_FOR_GET_FIRMWARE_VERSION_RESPONSE,
				&resp_data, &resp_data_size);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("Timed out waiting for WiHD State Machine resp\n");
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("WiHD State Machine request send failure\n");
			goto done;
		}

		if ((NULL != resp_data) &&
		    (sizeof(*firmware_version) == resp_data_size)) {
			firmware_version = resp_data;
		} else {
			err("Invalid WiHD State Machine response\n");
			goto done;
		}
	}
		break;

	case SII6400_MODE_MHL:
	{
		sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_FIRMWARE_VERSION,
				NULL, 0,
				get_mhl_firmware_version_resp_q,
				WAIT_TIME_FOR_GET_FIRMWARE_VERSION_RESPONSE,
				&resp_data, &resp_data_size);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("Timed out waiting for MHL State Machine resp\n");
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("MHL State Machine request send failure\n");
			goto done;
		}

		if ((NULL != resp_data) &&
		    (sizeof(*firmware_version) == resp_data_size)) {
			firmware_version = resp_data;
		} else {
			err("Invalid MHL State Machine response\n");
			goto done;
		}
	}
		break;

	case SII6400_MODE_CONFIG:
	case SII6400_MODE_OFF:
	case SII6400_MODE_UNDEFINED:
	default:
		break;
	}

done:
	if (NULL != firmware_version) {
		if (NULL != firmware_version->fw_version) {
			rv = scnprintf(buf, PAGE_SIZE, "%s",
					firmware_version->fw_version);
			SiiOsFree(firmware_version->fw_version);
			firmware_version->fw_version = NULL;
		} else {
			rv = scnprintf(buf, PAGE_SIZE, "%s", "");
		}
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const char *mode_str = "";
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
	} else {
		switch (devinfo->mode) {
		case SII6400_MODE_OFF:
			mode_str = "off";
			break;

		case SII6400_MODE_WIHD:
			mode_str = "wihd";
			break;

		case SII6400_MODE_MHL:
			mode_str = "mhl";
			break;
		case SII6400_MODE_CONFIG:
			mode_str = "config";
			break;
		case SII6400_MODE_UNDEFINED:
		default:
			break;
		}
	}
	return scnprintf(buf, PAGE_SIZE, "%s", mode_str);
}

#ifdef MHL_DEBUG_TEST
int debug_mhl_notify(char *buf, int count)
{
	ssize_t retval = count;
	enum sii6400_mode new_mode = SII6400_MODE_UNDEFINED;
	char *new_mode_str = NULL;
	struct sii6400_device_info *devinfo = NULL;

	printk("+++sii6400, debug_mhl_notify\n");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -EFAULT;
		goto done;
	}

	new_mode = devinfo->mode;

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (sysfs_streq(skip_spaces(buf), "off")) {
		new_mode = SII6400_MODE_OFF;
		new_mode_str = "off";
	} else if (sysfs_streq(skip_spaces(buf), "config")) {
		new_mode = SII6400_MODE_CONFIG;
		new_mode_str = "config";
	} else if (sysfs_streq(skip_spaces(buf), "wihd")) {
		new_mode = SII6400_MODE_WIHD;
		new_mode_str = "wihd";
	} else if (sysfs_streq(skip_spaces(buf), "mhl")) {
		new_mode = SII6400_MODE_MHL;
		new_mode_str = "mhl";
	} else {
		warn("Invalid Sii6400 driver mode %s", buf);
		retval = -EINVAL;
		goto done;
	}

	if ((SII6400_MODE_CONFIG == devinfo->mode) &&
	    (SII6400_MODE_WIHD == new_mode)) {
		err("Could not change to wihd mode from config mode\n");
		retval = -EINVAL;
		goto done;
	}

	if ((SII6400_MODE_CONFIG == devinfo->mode) &&
	    (SII6400_MODE_MHL == new_mode)) {
		err("Could not change to mhl mode from config mode\n");
		retval = -EINVAL;
		goto done;
	}

	if ((SII6400_MODE_WIHD == devinfo->mode) &&
	    (SII6400_MODE_CONFIG == new_mode)) {
		err("Could not change to config mode from wihd mode\n");
		retval = -EINVAL;
		goto done;
	}

	if ((SII6400_MODE_MHL == devinfo->mode) &&
	    (SII6400_MODE_CONFIG == new_mode)) {
		err("Could not change to config mode from mhl mode\n");
		retval = -EINVAL;
		goto done;
	}

	if (devinfo->mode != new_mode) {
		change_mode(devinfo, new_mode);

		if (devinfo->mode == new_mode) {
			char event_data[MAX_DRIVER_MODE_LEN + 4] = {0};
			int rv = 0;

			devinfo->mode = new_mode;

			//if ((NULL != dev) && (NULL != attr))
			//	sysfs_notify(&dev->kobj, NULL, attr->attr.name);

			rv = scnprintf(event_data, MAX_DRIVER_MODE_LEN + 4,
					"\"%s\"", new_mode_str);
			if (0 < rv) {
				(void)send_sii6400_uevent(devinfo->device,
						DEVICE_EVENT, MODE_CHANGE_EVENT,
						event_data);
			}
		}
	}

done:
	return retval;
}

EXPORT_SYMBOL(debug_mhl_notify);
#endif

static ssize_t set_sii6400_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	enum sii6400_mode new_mode = SII6400_MODE_UNDEFINED;
	char *new_mode_str = NULL;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -EFAULT;
		goto done;
	}

	new_mode = devinfo->mode;

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (sysfs_streq(skip_spaces(buf), "off")) {
		new_mode = SII6400_MODE_OFF;
		new_mode_str = "off";
	} else if (sysfs_streq(skip_spaces(buf), "config")) {
		new_mode = SII6400_MODE_CONFIG;
		new_mode_str = "config";
	} else if (sysfs_streq(skip_spaces(buf), "wihd")) {
		new_mode = SII6400_MODE_WIHD;
		new_mode_str = "wihd";
	} else if (sysfs_streq(skip_spaces(buf), "mhl")) {
		new_mode = SII6400_MODE_MHL;
		new_mode_str = "mhl";
	} else {
		warn("Invalid Sii6400 driver mode %s", buf);
		retval = -EINVAL;
		goto done;
	}

	if ((SII6400_MODE_CONFIG == devinfo->mode) &&
	    (SII6400_MODE_WIHD == new_mode)) {
		err("Could not change to wihd mode from config mode\n");
		retval = -EINVAL;
		goto done;
	}

	if ((SII6400_MODE_CONFIG == devinfo->mode) &&
	    (SII6400_MODE_MHL == new_mode)) {
		err("Could not change to mhl mode from config mode\n");
		retval = -EINVAL;
		goto done;
	}

	if ((SII6400_MODE_WIHD == devinfo->mode) &&
	    (SII6400_MODE_CONFIG == new_mode)) {
		err("Could not change to config mode from wihd mode\n");
		retval = -EINVAL;
		goto done;
	}

	if ((SII6400_MODE_MHL == devinfo->mode) &&
	    (SII6400_MODE_CONFIG == new_mode)) {
		err("Could not change to config mode from mhl mode\n");
		retval = -EINVAL;
		goto done;
	}

	if (devinfo->mode != new_mode) {
		change_mode(devinfo, new_mode);

		if (devinfo->mode == new_mode) {
			char event_data[MAX_DRIVER_MODE_LEN + 4] = {0};
			int rv = 0;

			devinfo->mode = new_mode;

			if ((NULL != dev) && (NULL != attr))
				sysfs_notify(&dev->kobj, NULL, attr->attr.name);

			rv = scnprintf(event_data, MAX_DRIVER_MODE_LEN + 4,
					"\"%s\"", new_mode_str);
			if (0 < rv) {
				(void)send_sii6400_uevent(devinfo->device,
						DEVICE_EVENT, MODE_CHANGE_EVENT,
						event_data);
			}
		}
	}

done:
	return retval;
}

static ssize_t get_sii6400_debug_level(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	dbg("");

	return scnprintf(buf, PAGE_SIZE, "%d", debug_level);
}

static ssize_t set_sii6400_debug_level(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	long new_debug_level = 0;

	dbg("");

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtol(buf, 0, &new_debug_level);
	if (rv) {
		warn("Invalid Debug Level input: %s", buf);
		retval = rv;
		goto done;
	}

	if ((DEBUG_LEVEL_NONE != new_debug_level) &&
	    (DEBUG_LEVEL_CRITICAL != new_debug_level) &&
	    (DEBUG_LEVEL_WARNINGS != new_debug_level) &&
	    (DEBUG_LEVEL_VERBOSE != new_debug_level)) {
		warn("Invalid Debug Level input: %d\n", (int)new_debug_level);
		retval = -EINVAL;
		goto done;
	}

	debug_level = (int)new_debug_level;

done:
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 Attributes.
 * These macros create instances of:
 *   dev_attr_chip_version
 *   dev_attr_fw_version
 *   dev_attr_mode
 *   dev_attr_debug_level
 */
static DEVICE_ATTR(chip_version, (S_IRUGO), get_sii6400_chip_version, NULL);
static DEVICE_ATTR(fw_version, (S_IRUGO), get_sii6400_fw_version, NULL);
static DEVICE_ATTR(mode, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
			get_sii6400_mode, set_sii6400_mode);
static DEVICE_ATTR(debug_level, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
			get_sii6400_debug_level, set_sii6400_debug_level);

static struct attribute *sii6400_attrs[] = {
	&dev_attr_chip_version.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_mode.attr,
	&dev_attr_debug_level.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_attr_group = {
	.attrs = sii6400_attrs,
};

/*
 * Sii6400 Diag Group Attributes
 */
static ssize_t get_sii6400_diag_cmd(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_diag_cmd *diag_cmd = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		goto done;
	}

	if ((SII6400_MODE_WIHD != devinfo->mode) &&
	    (SII6400_MODE_MHL != devinfo->mode) &&
	    (SII6400_MODE_CONFIG != devinfo->mode)) {
		dbg("Diagnostic commands not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(diag_sm_queue,
				SII_SM_REQ_GET_DIAG_COMMAND,
				NULL, 0,
				get_diag_cmd_resp_q,
				WAIT_TIME_FOR_GET_DIAG_COMMAND_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for Diag State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("Diag State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*diag_cmd) != resp_data_size)) {
		err("Invalid Diag State Machine response\n");
		goto done;
	}

	diag_cmd = resp_data;

done:
	if (NULL != diag_cmd) {
		if (NULL != diag_cmd->diag_cmd) {
			rv = scnprintf(buf, PAGE_SIZE, "%s",
					diag_cmd->diag_cmd);
			SiiOsFree(diag_cmd->diag_cmd);
			diag_cmd->diag_cmd = NULL;
		} else {
			rv = scnprintf(buf, PAGE_SIZE, "%s", "");
		}
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t send_sii6400_diag_cmd(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	char *diag_cmd_str = NULL;
	int length = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_diag_cmd *diag_cmd = NULL;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if ((SII6400_MODE_WIHD != devinfo->mode) &&
	    (SII6400_MODE_MHL != devinfo->mode) &&
	    (SII6400_MODE_CONFIG != devinfo->mode)) {
		dbg("Diagnostic commands not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	length = strnlen(skip_spaces(buf), count);
	diag_cmd_str = SiiOsCalloc("DiagCmdStr", length + 1, 0);
	if (NULL == diag_cmd_str) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	strlcpy(diag_cmd_str, skip_spaces(buf), length + 1);
	strim(diag_cmd_str);
	length = strlen(diag_cmd_str);
	if (MAX_DIAG_CMD_STR_LEN < length) {
		warn("Invalid Diagnostic Command %s", buf);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	diag_cmd = SiiOsCalloc("DiagCmd", sizeof(*diag_cmd), 0);
	if (NULL == diag_cmd) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	diag_cmd->is_sync_cmd = false;
	diag_cmd->length = length;
	diag_cmd->diag_cmd = diag_cmd_str;
	diag_cmd_str = NULL;

	sm_rv = sm_send_request(diag_sm_queue,
				SII_SM_REQ_SEND_DIAG_COMMAND,
				diag_cmd, sizeof(*diag_cmd),
				send_diag_cmd_resp_q,
				WAIT_TIME_FOR_SEND_DIAG_COMMAND_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_QUEUE_FULL == sm_rv) {
		err("Diag Command Queue is full\n");
		retval = -ENOMEM;
		goto done;
	} else if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for Diag State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("Diag State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

done:
	SiiOsFree(diag_cmd_str);
	return retval;
}

static ssize_t get_sii6400_diag_cmd_block(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_diag_cmd *diag_cmd = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		goto done;
	}

	if ((SII6400_MODE_WIHD != devinfo->mode) &&
	    (SII6400_MODE_MHL != devinfo->mode) &&
	    (SII6400_MODE_CONFIG != devinfo->mode)) {
		dbg("Diagnostic commands not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(diag_sm_queue,
				SII_SM_REQ_GET_DIAG_COMMAND_SYNC,
				NULL, 0,
				get_diag_cmd_sync_resp_q,
				WAIT_TIME_FOR_GET_DIAG_COMMAND_SYNC_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for Diag State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("Diag State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*diag_cmd) != resp_data_size)) {
		err("Invalid Diag State Machine response\n");
		goto done;
	}

	diag_cmd = resp_data;

done:
	if (NULL != diag_cmd) {
		if (NULL != diag_cmd->diag_cmd) {
			rv = scnprintf(buf, PAGE_SIZE, "%s",
					diag_cmd->diag_cmd);
			SiiOsFree(diag_cmd->diag_cmd);
			diag_cmd->diag_cmd = NULL;
		} else {
			rv = scnprintf(buf, PAGE_SIZE, "%s", "");
		}
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t send_sii6400_diag_cmd_block(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	char *diag_cmd_str = NULL;
	int length = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_diag_cmd *diag_cmd = NULL;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if ((SII6400_MODE_WIHD != devinfo->mode) &&
	    (SII6400_MODE_MHL != devinfo->mode) &&
	    (SII6400_MODE_CONFIG != devinfo->mode)) {
		dbg("Diagnostic commands not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	length = strnlen(skip_spaces(buf), count);
	diag_cmd_str = SiiOsCalloc("DiagCmdStr", length + 1, 0);
	if (NULL == diag_cmd_str) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	strlcpy(diag_cmd_str, skip_spaces(buf), length + 1);
	strim(diag_cmd_str);
	length = strlen(diag_cmd_str);
	if (MAX_DIAG_CMD_STR_LEN < length) {
		warn("Invalid Diagnostic Command %s", buf);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	diag_cmd = SiiOsCalloc("DiagCmd", sizeof(*diag_cmd), 0);
	if (NULL == diag_cmd) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	diag_cmd->is_sync_cmd = true;
	diag_cmd->length = length;
	diag_cmd->diag_cmd = diag_cmd_str;
	diag_cmd_str = NULL;

	sm_rv = sm_send_request(diag_sm_queue,
				SII_SM_REQ_SEND_DIAG_COMMAND_SYNC,
				diag_cmd, sizeof(*diag_cmd),
				send_diag_cmd_sync_resp_q,
				WAIT_TIME_FOR_SEND_DIAG_COMMAND_SYNC_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_QUEUE_FULL == sm_rv) {
		err("Diag Command Queue is full\n");
		retval = -ENOMEM;
		goto done;
	} else if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for Diag State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("Diag State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

done:
	SiiOsFree(diag_cmd_str);
	return retval;
}

static ssize_t get_sii6400_diag_cmd_output(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_diag_cmd_output *diag_cmd_output = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		goto done;
	}

	if ((SII6400_MODE_WIHD != devinfo->mode) &&
	    (SII6400_MODE_MHL != devinfo->mode) &&
	    (SII6400_MODE_CONFIG != devinfo->mode)) {
		dbg("Diagnostic commands not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(diag_sm_queue,
				SII_SM_REQ_GET_DIAG_COMMAND_OUTPUT,
				NULL, 0,
				get_diag_cmd_output_resp_q,
				WAIT_TIME_FOR_GET_DIAG_COMMAND_OUTPUT_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for Diag State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("Diag State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*diag_cmd_output) != resp_data_size)) {
		err("Invalid Diag State Machine response\n");
		goto done;
	}

	diag_cmd_output = resp_data;

done:
	if (NULL != diag_cmd_output) {
		if (NULL != diag_cmd_output->output) {
			rv = scnprintf(buf, PAGE_SIZE, "%s",
					diag_cmd_output->output);
			SiiOsFree(diag_cmd_output->output);
			diag_cmd_output->output = NULL;
		} else {
			rv = scnprintf(buf, PAGE_SIZE, "%s", "");
		}
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_diag_logfile(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	char *diag_logfile = NULL;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	diag_logfile = debug_logfile;

done:
	if (NULL != diag_logfile)
		rv = scnprintf(buf, PAGE_SIZE, "%s", diag_logfile);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	return retval ? retval : rv;
}

static ssize_t set_sii6400_diag_logfile(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	char *sysfs_diag_logfile = NULL;
	size_t sysfs_diag_logfile_len = 0;
	size_t actual_diag_logfile_len = 0;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t prev_debug_output_path = 0;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	sysfs_diag_logfile_len = strnlen(skip_spaces(buf), count);
	if (0 != sysfs_diag_logfile_len) {
		sysfs_diag_logfile = SiiOsCalloc("SysfsDiagLogfile",
						sysfs_diag_logfile_len + 1, 0);
		if (NULL == sysfs_diag_logfile) {
			err("Out of memory\n");
			retval = -ENODEV;
			goto done;
		}
		strlcpy(sysfs_diag_logfile, skip_spaces(buf),
			sysfs_diag_logfile_len + 1);
		strim(sysfs_diag_logfile);
		actual_diag_logfile_len = strlen(sysfs_diag_logfile);
		if (0 == actual_diag_logfile_len) {
			SiiOsFree(sysfs_diag_logfile);
			sysfs_diag_logfile = NULL;
		} else if (FILEPATH_NAME_LEN < actual_diag_logfile_len) {
			sysfs_diag_logfile[FILEPATH_NAME_LEN] = '\0';
			actual_diag_logfile_len = FILEPATH_NAME_LEN;
		}
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (NULL != sysfs_diag_logfile) {
		/* If the new debug logfile is not the same file as the
		 * previous debug logfile, close prev and open new logfile */
		if ((NULL == debug_logfile) ||
		    (actual_diag_logfile_len != strlen(debug_logfile)) ||
		    (0 != strncmp(debug_logfile, sysfs_diag_logfile,
					actual_diag_logfile_len))) {
			struct file *new_debug_log_filp = NULL;
			struct file *prev_debug_log_filp = NULL;

			/* Try to open the new debug logfile to append */
			new_debug_log_filp = filp_open(sysfs_diag_logfile,
				O_CREAT | O_RDWR | O_APPEND, S_IRUSR | S_IWUSR);
			if (IS_ERR(new_debug_log_filp) ||
			    (NULL == new_debug_log_filp)) {
				err("Error opening file %s\n",
				    sysfs_diag_logfile);
				retval = -ENODEV;
				goto done;
			}

			/* If file opened OK, close previous log file */
			prev_debug_log_filp = devinfo->debug_log_filp;
			devinfo->debug_log_filp = new_debug_log_filp;
			if (NULL != prev_debug_log_filp) {
				if (filp_close(prev_debug_log_filp, NULL)) {
					err("Error closing file %s\n",
					    debug_logfile);
				}
			}

			/* If file opened OK, set the current log file path */
			SiiOsFree(debug_logfile);
			debug_logfile = sysfs_diag_logfile;
			sysfs_diag_logfile = NULL;
		}
	} else {
		/* Close the open log file */
		if (NULL != devinfo->debug_log_filp) {
			if (filp_close(devinfo->debug_log_filp, NULL))
				err("Error closing file %s\n", debug_logfile);
			devinfo->debug_log_filp = NULL;
		}

		/* Clear the current log file path */
		SiiOsFree(debug_logfile);
		debug_logfile = NULL;
	}

	/* If the value of the Debug Output Path has changed,
	 * send it to the firmware */
	prev_debug_output_path =
			devinfo->my_debug_output_path.debug_output_path;
	if (NULL != debug_logfile) {
		devinfo->my_debug_output_path.debug_output_path =
							DEBUG_OUTPUT_PATH_SPI;
	} else {
		devinfo->my_debug_output_path.debug_output_path =
							DEBUG_OUTPUT_PATH_UART;
	}

	if (((SII6400_MODE_WIHD == devinfo->mode) ||
	     (SII6400_MODE_MHL == devinfo->mode) ||
	     (SII6400_MODE_CONFIG == devinfo->mode)) &&
	    (prev_debug_output_path !=
			devinfo->my_debug_output_path.debug_output_path)) {
		struct sii6400_debug_output_path *debug_output_path = NULL;
		enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

		/* This memory is freed in the state machine. */
		debug_output_path = SiiOsCalloc("DebugOutputPath",
						sizeof(*debug_output_path), 0);
		if (NULL == debug_output_path) {
			err("Out of memory\n");
			retval = -ENODEV;
			goto done;
		}
		memcpy(debug_output_path, &devinfo->my_debug_output_path,
			sizeof(*debug_output_path));

		sm_rv = sm_send_request(diag_sm_queue,
				SII_SM_REQ_SET_DEBUG_OUTPUT_PATH,
				debug_output_path, sizeof(*debug_output_path),
				set_debug_output_path_resp_q,
				WAIT_TIME_FOR_SET_DEBUG_OUTPUT_PATH_RESPONSE,
				NULL, NULL);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("Timed out waiting for Diag State Machine resp\n");
			retval = -ETIME;
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("Diag State Machine request send failure\n");
			retval = -ENODEV;
			goto done;
		}
	}

done:
	SiiOsFree(sysfs_diag_logfile);
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 Diag Group Attributes.
 */
static struct device_attribute dev_attr_diag_cmd =
	__ATTR(cmd, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_diag_cmd,
		send_sii6400_diag_cmd);
static struct device_attribute dev_attr_diag_cmd_block =
	__ATTR(cmd_block, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_diag_cmd_block,
		send_sii6400_diag_cmd_block);
static struct device_attribute dev_attr_diag_cmd_output =
	__ATTR(cmd_output, (S_IRUGO), get_sii6400_diag_cmd_output, NULL);
static struct device_attribute dev_attr_diag_logfile =
	__ATTR(logfile, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_diag_logfile,
		set_sii6400_diag_logfile);

static struct attribute *sii6400_diag_attrs[] = {
	&dev_attr_diag_cmd.attr,
	&dev_attr_diag_cmd_block.attr,
	&dev_attr_diag_cmd_output.attr,
	&dev_attr_diag_logfile.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_diag_attr_group = {
	.name = __stringify(diag),
	.attrs = sii6400_diag_attrs,
};

#ifdef SII6400_COMBINED_FIRMWARE
static enum sii_os_status send_output_mode_to_firmware(
						enum sii6400_mode new_mode)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t *output_mode_data = NULL;
	uint8_t mode = (uint8_t)new_mode;

	dbg("");

	if ((SII6400_MODE_WIHD != new_mode) &&
	    (SII6400_MODE_MHL != new_mode) &&
	    (SII6400_MODE_CONFIG != new_mode)) {
		err("illegal parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	output_mode_data = SiiOsCalloc("SetOutputMode",
					HM_SET_OUTPUT_MODE_CMD_MLEN, 0);
	if (NULL == output_mode_data) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	field_ua_set(mode, output_mode_data, HM_SET_OUTPUT_MODE_CMD_MODE);

	retval = send_hostmsg_command((uint16_t)HM_SET_OUTPUT_MODE,
					HM_SET_OUTPUT_MODE_CMD_MLEN,
					output_mode_data,
					HM_SET_OUTPUT_MODE_RESPONSE_TIMEOUT,
					NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set Output Mode command not sent\n");
		goto done;
	}
	dbg("Set Output Mode successful\n");

done:
	SiiOsFree(output_mode_data);
	return retval;
}
#endif /* SII6400_COMBINED_FIRMWARE */

#if 0
static int sii6400_reg_config(struct sii6400_device_info *devinfo, bool enable)
{
	int rc = -EINVAL;
	struct regulator *reg_pma8084_l14 = NULL;
	bool reg_enabled = false;

	dbg("");

  	if (NULL == devinfo) {
		err("NULL devinfo buffer\n");
		goto failed;
	}

	// get PMA8084_L14 regulator which provide V1.8 as SiI6400 requested
	reg_pma8084_l14 = devinfo->vregs[SII6400_1V8_VREG]->vreg;
	if (NULL == reg_pma8084_l14) {
		err("NULL pma8084_l14 regulator resource\n");
		goto failed;
	}

	dbg("vreg: uV %d\n",  regulator_get_voltage(reg_pma8084_l14));
	// to check PMA8084_L14 regulator status
	rc = regulator_is_enabled(reg_pma8084_l14);
	if (rc < 0) {
		err("Failed to access regulator_is_enabled - %d", rc);
		goto failed;
	}
	reg_enabled = rc > 0 ? true : false;

	rc = 0;
	// disable or enable PMA8084_L14 regulator as requested
	if (!enable) {
		if (reg_enabled) {
			rc = regulator_disable(reg_pma8084_l14);
		} else {
			warn("reg_pma8084_l14 is already disabled\n");
		}
	} else {
		if (!reg_enabled) {
			rc = regulator_enable(reg_pma8084_l14);
		} else {
			warn("reg_pma8084_l14 is already enabled\n");
		}
	}

	if (rc) {
		err("'%s' regulator config[%u] failed, rc=%d\n", "avcc_1.8V", enable, rc);
		goto failed;
	}

	dbg("vreg PMA8084_L14 %s\n", (enable ? "enabled" : "disabled"));
	return 0;

failed:
	return -EINVAL;
}
#endif

static int sii6400_vreg_config(struct sii6400_device_info *devinfo, bool on)
{
	int ret;
	struct dss_gpio *pwr_gpio = NULL;
	struct dss_gpio *hdt_gpio = NULL;

	dbg("");

	if (NULL == devinfo) {
		err("NULL devinfo buffer\n");
		goto vreg_config_failed;
	}

	pwr_gpio = devinfo->gpios[SII6400_PMIC_PWR_GPIO];
	if (NULL == pwr_gpio) {
		err("NULL PWR GPIO buffer\n");
		goto vreg_config_failed;
	}

	hdt_gpio = devinfo->gpios[SII6400_HDMI_DET_GPIO];
	if (NULL == hdt_gpio) {
		err("NULL HDT GPIO buffer\n");
		goto vreg_config_failed;
	}

	if (on) {
		// configure WHD_PWR_EN to enable WHD_1V1 from VPH_PWR
		ret = gpio_direction_output(pwr_gpio->gpio, 1);
		if (ret < 0) {
			err("set gpio MHL_PWR_EN dircn failed: %d\n", ret);
			goto vreg_config_failed;
		}

#if 0
		ret = sii6400_reg_config(devinfo, true);
		if (ret) {
			err("regulator enable failed\n");
			goto vreg_config_failed;
		}
#endif

		// configure HDMI_5V_DET to enable HDMI detecting
		ret = gpio_direction_output(hdt_gpio->gpio, 1);
		if (ret < 0) {
			err("set gpio HDMI_5V_DET dircn failed: %d\n", ret);
			goto vreg_config_failed;
		}

		dbg("sii6400 power on successful\n");
	} else {
		warn("turning off pwr controls\n");

#if 0
		sii6400_reg_config(devinfo, false);
#endif

		// put WHD_PWR_EN to down
		ret = gpio_direction_output(pwr_gpio->gpio, 0);
		if (ret < 0) {
			err("set gpio WHD_PWR_EN dircn failed: %d\n", ret);
			goto vreg_config_failed;
		}

		// put HDMI_5V_DET to down
		ret = gpio_direction_output(hdt_gpio->gpio, 0);
		if (ret < 0) {
			err("set gpio HDMI_5V_DET dircn failed: %d\n", ret);
			goto vreg_config_failed;
		}
	}

	/*dbg("successful\n");*/
	return 0;

vreg_config_failed:
    return -EINVAL;
}

static int sii6400_mhl_switch(struct sii6400_device_info *devinfo, bool on)
{
	int ret;
	struct dss_gpio *mhl_sel1_gpio = NULL;

	dbg("");

	if (NULL == devinfo) {
		err("NULL devinfo buffer\n");
		goto mhl_config_failed1;
	}

	mhl_sel1_gpio = devinfo->gpios[SII6400_MHL_SEL1_GPIO];
	if (NULL == mhl_sel1_gpio) {
		err("NULL MHL SEL1 GPIO buffer\n");
		goto mhl_config_failed1;
	}

	if (on) {
		// configure MHL_USB_SEL1 to enable MHL path
		ret = gpio_request_one(mhl_sel1_gpio->gpio, GPIOF_OUT_INIT_HIGH,
				"SII6400 mhl sel1");
		if (ret < 0) {
			err("Failled GPIO MHL SEL1 request: %d\n", ret);
			goto mhl_config_failed1;
		}
		mhl_sel1_gpio->value = 1;

	} else {

		if (mhl_sel1_gpio->value) {
			// configure MHL_USB_SEL1 to enable USB path
			ret = gpio_direction_output(mhl_sel1_gpio->gpio, 0);
			if (ret < 0) {
				err("set gpio MHL-SEL1 direct failed: %d\n", ret);
			}
			gpio_free(mhl_sel1_gpio->gpio);

			mhl_sel1_gpio->value = 0;
		}

	}

	dbg("successful\n");
	return 0;

mhl_config_failed1:
	return -EINVAL;
}

#ifdef MHL_POWER_OUT
static void sii6400_mhl_tx_vbus_control(struct sii6400_device_info *devinfo, bool power_state)
{
	if (devinfo == NULL)
		return;

	err("newstate %d received!VBUS_power_state=%d\n ",
		power_state, devinfo->vbus_power_state);

	if (power_state == devinfo->vbus_power_state)
		return;

	if (power_state) {
		if (!dwc3_otg_set_mhl_power(1))
			devinfo->vbus_power_state = 1;
	} else {
		if (!dwc3_otg_set_mhl_power(0))
			devinfo->vbus_power_state = 0;
	}
}
#endif

#ifdef CONFIG_LIMIT_ON_HDMI
void hdmi_tx_hpd_done_callback(void *data)
{
	struct sii6400_device_info *devinfo = NULL;
	int retval = -1;

	struct msm_hdmi_mhl_ops *hdmi_mhl_ops = NULL;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo)
		err("Invalid devinfo pointer\n");

	hdmi_mhl_ops = devinfo->hdmi_mhl_ops;
	if (hdmi_mhl_ops) {
		retval = hdmi_mhl_ops->set_upstream_hpd(devinfo->hdmi_pdev, 0);
		err("set hpd 0\n");
	}
}
#endif

void change_mode(struct sii6400_device_info *devinfo,
			enum sii6400_mode new_mode)
{
	int retval = 0;
	int retry_cnt;
	enum sii_status sii_rv;
	enum sii_os_status sii_os_rv;
	enum sii6400_mode old_mode;
	static int is_changing_mode_off;

#ifdef CONFIG_LIMIT_ON_HDMI
	struct msm_hdmi_mhl_ops *hdmi_mhl_ops = NULL;
#endif


	if (NULL == devinfo)
		goto done;
	old_mode = devinfo->mode;

#ifdef CONFIG_LIMIT_ON_HDMI
	hdmi_mhl_ops = devinfo->hdmi_mhl_ops;
#endif

	warn("change_mode: %d -> %d\n", old_mode, new_mode);
	switch (new_mode) {
	case SII6400_MODE_OFF:
		if (is_changing_mode_off) {
			warn("%s: it's changing mode off, ignore!!!\n",
								__func__);
			break;
		}
		is_changing_mode_off++;
#ifdef MHL_POWER_OUT
		sii6400_mhl_tx_vbus_control(devinfo, false);
#endif
		if (sii6400_vreg_config(devinfo, false)) {
			err("Failed sii6400_vreg_config to power down");
		}

		if (sii6400_mhl_switch(devinfo, false)) {
			err("Failed sii6400_mhl_switch to USB path\n");
		}

#ifdef CONFIG_LIMIT_ON_HDMI
		if (hdmi_mhl_ops) {
			retval = hdmi_mhl_ops->set_upstream_hpd(devinfo->hdmi_pdev, 0);
			dbg("%s: hdmi unset hpd %s\n", __func__, retval ? "failed" : "passed");
		}
#endif


		sii6400_wihd_sm_disable();
		sii6400_mhl_sm_disable();
		sii6400_diag_sm_disable();

		retval = HostMsgTerm();
		if (retval < 0)
			err("HostMsg failed to terminate: 0x%x\n", retval);

		/* Put the device in "reset" */
		sii_rv = SiiHalReset(true);
		if (SII_STATUS_SUCCESS != sii_rv)
			err("Reset failed\n");

		/* Change the mode to "off" */
		devinfo->mode = SII6400_MODE_OFF;
		if (old_mode == SII6400_MODE_WIHD) {
			(void)send_sii6400_uevent(devinfo->wihd->device,
				WIHD_EVENT, MODE_CHANGE_EVENT_WIHD_OFF, NULL);
		} else if (old_mode == SII6400_MODE_MHL) {
			(void)send_sii6400_uevent(devinfo->mhl->device,
				MHL_EVENT, MODE_CHANGE_EVENT_MHL_OFF, NULL);
		}

		dbg("Mode changed to off\n");
		is_changing_mode_off--;
		break;

	case SII6400_MODE_CONFIG:
	case SII6400_MODE_WIHD:
	case SII6400_MODE_MHL:
#ifdef CONFIG_LIMIT_ON_HDMI
		if (SII6400_MODE_OFF == old_mode) {
			if (hdmi_mhl_ops) {
				wake_lock(&sii6400_change_mode_wakelock);
				retval = hdmi_mhl_ops->set_upstream_hpd(
						devinfo->hdmi_pdev, 1);
				wake_unlock(&sii6400_change_mode_wakelock);
				dbg("%s:hdmi set hpd %s\n", __func__,
						retval ? "failed" : "passed");
			}
		}
#endif

		if (new_mode == SII6400_MODE_MHL) {
#ifdef MHL_POWER_OUT
			dwc3_otg_start_mhl_power();
			err("set VBUS to power on\n");
			sii6400_mhl_tx_vbus_control(devinfo, true);
#endif
			if (sii6400_mhl_switch(devinfo, true)) {
				err("Failed sii6400_mhl_switch to MHL path\n");
			}
		}

		if (sii6400_vreg_config(devinfo, true)) {
			err("Failed sii6400_vreg_config to power on\n");
		}


		sii6400_wihd_sm_disable();
		sii6400_mhl_sm_disable();
		if (SII6400_MODE_OFF == old_mode)
			sii6400_diag_sm_disable();

		if (SII6400_MODE_WIHD == new_mode) {
			sii6400_wihd_sm_enable();

			sii_os_rv = sii6400_wihd_sm_start();
			if (SII_OS_STATUS_SUCCESS != sii_os_rv)
				err("WiHD State Machine not started\n");
		} else if (SII6400_MODE_MHL == new_mode) {
			sii6400_mhl_sm_enable();

			sii_os_rv = sii6400_mhl_sm_start();
			if (SII_OS_STATUS_SUCCESS != sii_os_rv)
				err("MHL State Machine not started\n");
		}

		if (SII6400_MODE_OFF == old_mode) {
			sii6400_diag_sm_enable();

			sii_os_rv = sii6400_diag_sm_start();
			if (SII_OS_STATUS_SUCCESS != sii_os_rv)
				err("DIAG State Machine not started\n");

/*#ifdef CONFIG_LIMIT_ON_HDMI
			if (hdmi_mhl_ops) {
				retval = hdmi_mhl_ops->set_upstream_hpd(devinfo->hdmi_pdev, 1);
				dbg("%s: hdmi set hpd %s\n", __func__, retval ? "failed" : "passed");
			}
#endif*/
		}

#ifdef SII6400_COMBINED_FIRMWARE
		if (SII6400_MODE_OFF == old_mode) {
			wake_lock(&sii6400_change_mode_wakelock);
			retval = SiiDeviceFirmwareBoot(wihd_mhl_firmware);
			wake_unlock(&sii6400_change_mode_wakelock);
			if (retval < 0) {
				err("firmware load failed the first time\n");
				wake_lock(&sii6400_change_mode_wakelock);
				msleep(400);
				retval = SiiDeviceFirmwareBoot(wihd_mhl_firmware);
				wake_unlock(&sii6400_change_mode_wakelock);
				if (retval < 0) {
					err("firmware load failed the second time\n");
					change_mode(devinfo, SII6400_MODE_OFF);

#ifdef ENABLE_MHL_UEVENT
					dbg("report_mhl_disconnected notify the mhl switch dev for load firmware fail\n");
					mhl_state_uevent(0, 1);
#endif

					goto done;
				}
			}
		}
		wake_lock(&sii6400_change_mode_wakelock);
		sii_os_rv = send_output_mode_to_firmware(new_mode);
		wake_unlock(&sii6400_change_mode_wakelock);
		if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
			err("Failed to Set Output Mode\n");
			change_mode(devinfo, SII6400_MODE_OFF);
			goto done;
		}
#else
		retval = SiiDeviceFirmwareBoot((SII6400_MODE_WIHD == new_mode) ?
						wihd_firmware : mhl_firmware);
		if (retval < 0) {
			err("firmware load failed\n");
			change_mode(devinfo, SII6400_MODE_OFF);
			goto done;
		}
#endif

		/* Change the mode */
		devinfo->mode = new_mode;
		switch (new_mode) {
		case SII6400_MODE_WIHD:
			dbg("Mode changed to wihd\n");
			break;
		case SII6400_MODE_MHL:
			dbg("Mode changed to mhl\n");
			break;
		case SII6400_MODE_CONFIG:
			dbg("Mode changed to config\n");
			break;
		default:
			err("Mode changed failed\n");
			break;
		}

		if (SII6400_MODE_WIHD == new_mode) {
			(void)sii6400_wihd_init(devinfo);
			(void)send_sii6400_uevent(devinfo->wihd->device,
				WIHD_EVENT, MODE_CHANGE_EVENT_WIHD_ON, NULL);
		}

		if (SII6400_MODE_MHL == new_mode) {
			(void)sii6400_mhl_init(devinfo);
			for (retry_cnt = 0; retry_cnt < 4; retry_cnt++) {
				msleep(25);
				sii_os_rv = get_sii6400_mhl_id_impedance_after_mode_changed();
				if (sii_os_rv == SII_OS_STATUS_SUCCESS)
					break;
			}
			(void)send_sii6400_uevent(devinfo->mhl->device,
				MHL_EVENT, MODE_CHANGE_EVENT_MHL_ON, NULL);
		}

		if (SII6400_MODE_OFF == old_mode)
			(void)sii6400_diag_init(devinfo);

		break;

	case SII6400_MODE_UNDEFINED:
	default:
		break;
	}

done:
	return;
}

bool sii6400_module_is_exiting(void)
{
	return module_is_exiting;
}

static ssize_t set_sii6400_mhl_cbus_low(uint8_t high, uint16_t period)
{
	ssize_t retval = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_cbus_voltage *cbus_voltage = NULL;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	err("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		err("the modules is not exiting\n");
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL clock swing not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}


	/* This memory is freed in the state machine. */
	cbus_voltage = SiiOsCalloc("", sizeof(*cbus_voltage), 0);
	if (NULL == cbus_voltage) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	cbus_voltage->high = high;
	cbus_voltage->peroid = period;
	err("set the high =%d,the period= %d\n", high, period);

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_SET_MHL_CBUS_VOLTAGE,
				cbus_voltage, sizeof(*cbus_voltage),
				set_mhl_cbus_voltage_resp_q,
				HM_MHL_SET_CBUS_VOLTAGE_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

done:
	return retval;
}

static int sii6400_reboot_cbus(struct notifier_block *nb, unsigned long event,
		void *buf)
{
	dbg("system reboot\n");
	set_sii6400_mhl_cbus_low(0, 100);

	return NOTIFY_OK;
}

static struct notifier_block sii6400_reboot_notifier = {
	.notifier_call = sii6400_reboot_cbus,
};

static void fb_notify_sii6400_resume_work(struct work_struct *work);
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);

static int sii6400_suspend(void)
{
	struct sii6400_device_info *devinfo = NULL;
	dbg("sii6400 suspend start\n");
	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo)
		return -ENODEV;

	if ((devinfo->mode == SII6400_MODE_WIHD) ||
			(devinfo->mode == SII6400_MODE_MHL)) {
		change_mode(devinfo, SII6400_MODE_OFF);
		dbg("sii6400 suspend into off mode\n");
	}

	dbg("sii6400 suspend stop\n");
	return 0;
}

static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	struct sii6400_device_info *devinfo = NULL;
	devinfo = get_sii6400_devinfo();

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
				devinfo) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			pr_err("the fb is unblank\n");
			schedule_work(&devinfo->fb_notify_work);
		}
	}

	return 0;
}

static void fb_notify_sii6400_resume_work(struct work_struct *work)
{
	struct sii6400_device_info *devinfo = NULL;
	int rv = 0;
	char event_data[11] = {0};
	dbg("sii6400 resume start\n");
	devinfo = get_sii6400_devinfo();
	if (NULL != devinfo) {
		rv = scnprintf(event_data, 11,
			"\"%s\"", "resume");
		if (0 < rv) {
			(void)send_sii6400_uevent(devinfo->device,
				DEVICE_EVENT, MODE_CHANGE_EVENT,
					event_data);
		}
		dbg("into set wihd mode\n");
	}
	dbg("sii6400 resume stop\n");
}

static struct sii6400_firmware_memory firmware_memory;

struct sii6400_firmware_memory *get_firmware_memory(void)
{
	return &firmware_memory;
}

static void sii6400_init_firmware_memory(void)
{
	uint32_t i;
	uint32_t num_firmware_bufs = 0;

	memset(&firmware_memory, 0, sizeof(struct sii6400_firmware_memory));

	firmware_memory.cfg_buf = SiiOsCalloc("configbuff",
			NV_STORAGE_SIZE * sizeof(unsigned char *), 0);
	if (NULL == firmware_memory.cfg_buf)
		err("Out of memory\n");

	num_firmware_bufs = MAX_DEVICE_MEMORY / FIRMWARE_BUF_SIZE;
	if (MAX_DEVICE_MEMORY % FIRMWARE_BUF_SIZE)
		num_firmware_bufs++;

	firmware_memory.firmware_buf = SiiOsCalloc("FWBufArray",
			num_firmware_bufs * sizeof(uint32_t *), 0);
	if (NULL == firmware_memory.firmware_buf)
		err("Out of memory\n");

	if (firmware_memory.firmware_buf) {
		for (i = 0; i < num_firmware_bufs; i++) {
			firmware_memory.firmware_buf[i] =
				SiiOsCalloc("", FIRMWARE_BUF_SIZE, 0);
			if (NULL == firmware_memory.firmware_buf[i])
				err("Out of memory\n");
		}
	}

	return;
}

static void sii6400_release_firmware_memory(void)
{
	uint32_t i;
	uint32_t num_firmware_bufs = 0;

	if (firmware_memory.cfg_buf != NULL)
		SiiOsFree(firmware_memory.cfg_buf);

	num_firmware_bufs = MAX_DEVICE_MEMORY / FIRMWARE_BUF_SIZE;
	if (MAX_DEVICE_MEMORY % FIRMWARE_BUF_SIZE)
		num_firmware_bufs++;

	if (firmware_memory.firmware_buf != NULL) {
		for (i = 0; i < num_firmware_bufs; i++) {
			if (firmware_memory.firmware_buf[i] != NULL)
				SiiOsFree(firmware_memory.firmware_buf[i]);
		}
		SiiOsFree(firmware_memory.firmware_buf);
	}
}

static int sii6400_suspend_pm_event(struct notifier_block *notifier,
					unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		sii6400_suspend();
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block sii6400_pm_notifier_block = {
	.notifier_call = sii6400_suspend_pm_event,
};
/* Module initialization and release */
static int sii6400_probe(struct platform_device *pdev)
{
	int retval = 0;
	enum sii_status sii_rv = SII_STATUS_SUCCESS;
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;

	/* display driver version information in kernel log */
	info("Version "SII6400_DRIVER_VERSION);

	module_is_exiting = false;

	sii_os_rv = SiiOsInit(1);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		retval = -EFAULT;
		goto init_failed;
	}

	devinfo = SiiOsCalloc("DevInfo", sizeof(*devinfo), 0);
	if (NULL == devinfo) {
		err("Out of memory\n");
		retval = -ENOMEM;
		goto init_failed;
	}
	sii6400_modinfo.sii6400_devinfo = devinfo;

	retval = alloc_chrdev_region(&devinfo->devnum, 0,
				SII6400_NUMBER_OF_DEVS, SII6400_DEVNAME);
	if (retval < 0)
		goto init_failed;

	devinfo->cdev = cdev_alloc();
	devinfo->cdev->owner = THIS_MODULE;
	devinfo->cdev->ops = &sii6400_fops;
	retval = cdev_add(devinfo->cdev, devinfo->devnum, 1);
	if (retval < 0)
		goto init_failed;

	retval = sii6400_wihd_dev_add(devinfo);
	if (retval < 0)
		goto init_failed;

	retval = sii6400_mhl_dev_add(devinfo);
	if (retval < 0)
		goto init_failed;

	devinfo->dev_class = class_create(THIS_MODULE, SII6400_CLASS_NAME);
	if (IS_ERR(devinfo->dev_class)) {
		retval = PTR_ERR(devinfo->dev_class);
		goto init_failed;
	}

	devinfo->device = device_create(devinfo->dev_class, NULL,
					devinfo->devnum, NULL, SII6400_DEVNAME);
	if (IS_ERR(devinfo->device)) {
		retval = PTR_ERR(devinfo->device);
		goto init_failed;
	}
	sii6400_device = devinfo->device;

	retval = sysfs_create_group(&devinfo->device->kobj,
					&sii6400_attr_group);
	if (retval < 0)
		warn("failed to create sii6400 attribute group\n");

	retval = sysfs_create_group(&devinfo->device->kobj,
					&sii6400_diag_attr_group);
	if (retval < 0)
		warn("failed to create sii6400 diag attribute group\n");

#ifdef SII6400_DEBUG
	retval = sii6400_dbg_init(devinfo);
	if (retval < 0)
		goto init_failed;
#endif

	retval = sii6400_wihd_dev_init(devinfo);
	if (retval < 0)
		goto init_failed;

	retval = sii6400_mhl_dev_init(devinfo);
	if (retval < 0)
		goto init_failed;

	/* Set various settings to default values */
	devinfo->my_connect_timeout.connect_timeout =
						SELF_CONNECT_TIMEOUT_DEFAULT;
	devinfo->my_hdcp_policy.policy = HDCP_POLICY_DEFAULT;
	devinfo->my_hdcp_stream_type.stream_type = HDCP_STREAM_TYPE_DEFAULT;

	devinfo->my_debug_output_path.debug_output_path =
						DEBUG_OUTPUT_PATH_DEFAULT;

	devinfo->my_rc_input_device = input_dev_remote_control;
	devinfo->my_rap_input_device = input_dev_rap;
	devinfo->my_rcp_input_device = input_dev_rcp;
	devinfo->my_ucp_input_device = input_dev_ucp;

	sii_rv = SiiDeviceInit();
	if (SII_STATUS_SUCCESS != sii_rv) {
		retval = -EFAULT;
		goto init_failed;
	}

	sii_rv = SiiHalInit();
	if (SII_STATUS_SUCCESS != sii_rv) {
		retval = -EFAULT;
		goto init_failed;
	}

	retval = sii6400_sm_init();
	if (retval < 0)
		goto init_failed;

	wake_lock_init(&sii6400_change_mode_wakelock,
		WAKE_LOCK_SUSPEND, "sii6400_change_mode_wakelock");
	change_mode(devinfo, SII6400_MODE_OFF);
	register_reboot_notifier(&sii6400_reboot_notifier);

	INIT_WORK(&devinfo->fb_notify_work, fb_notify_sii6400_resume_work);
	devinfo->fb_notify.notifier_call = fb_notifier_callback;

	retval = fb_register_client(&devinfo->fb_notify);
	if (retval)
		pr_err("unable to register fb_notifier:%d\n", retval);

	retval = register_pm_notifier(&sii6400_pm_notifier_block);
	if (retval)
		pr_err("unable to register pm_notifier:%d\n", retval);

	sii6400_init_firmware_memory();

	return 0;

init_failed:
	cleanup_and_exit();
	return retval;
}

void cleanup_and_exit(void)
{
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	unregister_reboot_notifier(&sii6400_reboot_notifier);

	if (unregister_pm_notifier(&sii6400_pm_notifier_block))
		pr_err("unable to unregister the pm_notifier\n");
	if (fb_unregister_client(&devinfo->fb_notify))
		pr_err("unable to unregister the fb_notifier\n");

	devinfo = get_sii6400_devinfo();

	change_mode(devinfo, SII6400_MODE_OFF);

	module_is_exiting = true;

	msleep(500);

	(void)HostMsgTerm();

	sii6400_sm_exit();

	SiiHalTerm();

	SiiDeviceTerm();

	SiiOsFree(debug_logfile);

	if (NULL != devinfo) {
		if (NULL != devinfo->debug_log_filp) {
			if (filp_close(devinfo->debug_log_filp, NULL))
				err("Error closing file %s\n", debug_logfile);
			devinfo->debug_log_filp = NULL;
		}

		sii6400_mhl_dev_exit(devinfo);
		sii6400_wihd_dev_exit(devinfo);
#ifdef SII6400_DEBUG
		sii6400_dbg_exit(devinfo);
#endif

		if (NULL != devinfo->device) {
			sysfs_remove_group(&devinfo->device->kobj,
						&sii6400_attr_group);
			sysfs_remove_group(&devinfo->device->kobj,
						&sii6400_diag_attr_group);
			device_unregister(devinfo->device);
			devinfo->device = NULL;
		}
		sii6400_device = NULL;

		if (NULL != devinfo->dev_class) {
			device_destroy(devinfo->dev_class, devinfo->devnum);
			class_destroy(devinfo->dev_class);
			devinfo->dev_class = NULL;
		}

		sii6400_mhl_dev_remove(devinfo);
		sii6400_wihd_dev_remove(devinfo);

		if (NULL != devinfo->cdev) {
			cdev_del(devinfo->cdev);
			devinfo->cdev = NULL;
		}

		unregister_chrdev_region(devinfo->devnum,
						SII6400_NUMBER_OF_DEVS);

		SiiOsFree(devinfo);
	}

	(void)SiiOsTerm();
}

static int sii6400_remove(struct platform_device *pdev)
{
	dbg("");

	cleanup_and_exit();
	wake_lock_destroy(&sii6400_change_mode_wakelock);
	sii6400_release_firmware_memory();
	return 0;
}
struct of_device_id sii6400_match_table[] = {
	{.compatible = SII6400_COMPATIBLE_NAME,},
	{ },
};

static struct platform_driver sii6400_platform_driver = {
	.driver = {
		.name = "sii6400_platform_driver",
		.of_match_table = sii6400_match_table,
	},
	.probe = sii6400_probe,
	.remove = sii6400_remove,
};

static int __init sii6400_module_init(void)
{
	int ret = -1;

	ret = platform_driver_register(&sii6400_platform_driver);
	if (ret) {
		pr_err("%s(): Failed to register sii6400"
				 "platform driver\n", __func__);
	}
	return ret;
}

static void __exit sii6400_module_exit(void)
{
	platform_driver_unregister(&sii6400_platform_driver);
	return;
}


module_init(sii6400_module_init);
module_exit(sii6400_module_exit);

