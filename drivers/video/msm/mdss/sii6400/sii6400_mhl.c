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
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/stat.h>

#include "osal.h"
#include "hal.h"
#include "sii6400.h"
#include "state_machine.h"
#include "wihd_sm.h"
#include "mhl_sm.h"

/* A mutex will ensure that only one process accesses our device */
static DEFINE_MUTEX(sii6400_mhl_device_mutex);

/* static function prototypes */
static int sii6400_mhl_device_open(struct inode *inode, struct file *filp);
static int sii6400_mhl_device_close(struct inode *inode, struct file *filp);
static ssize_t get_sii6400_mhl_dev_type(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_mhl_connection_state(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_mhl_id_impedance(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_mhl_clock_swing(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_clock_swing(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t set_sii6400_mhl_cbus_voltage(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_devcap_local(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_devcap_local(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_devcap_local_offset(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_devcap_local_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_devcap_remote(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_mhl_devcap_remote_offset(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_devcap_remote_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_rap_in(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_rap_in_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_rap_out(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_rap_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_rap_out_status(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_mhl_rap_input_dev(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_rap_input_dev(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_rcp_in(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_rcp_in_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_rcp_out(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_rcp_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_rcp_out_status(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_mhl_rcp_input_dev(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_rcp_input_dev(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_ucp_in(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_ucp_in_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_ucp_out(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_ucp_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_mhl_ucp_out_status(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_mhl_ucp_input_dev(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_mhl_ucp_input_dev(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);


static int sii6400_mhl_device_open(struct inode *inode, struct file *filp)
{
	int retval = 0;

	dbg("");

	if (NULL == filp) {
		err("Invalid filp pointer\n");
		retval = -EINVAL;
		goto done;
	}

	/* The Sii6400 MHL device does not allow write access */
	if (((filp->f_flags & O_ACCMODE) == O_WRONLY) ||
	    ((filp->f_flags & O_ACCMODE) == O_RDWR)) {
		retval = -EACCES;
		goto done;
	}

	/* Ensure that only one process has access to the
	 * Sii6400 MHL device at any one time. */
	if (!mutex_trylock(&sii6400_mhl_device_mutex)) {
		retval = -EBUSY;
		goto done;
	}

done:
	return retval;
}

static int sii6400_mhl_device_close(struct inode *inode, struct file *filp)
{
	dbg("");

	mutex_unlock(&sii6400_mhl_device_mutex);
	return 0;
}

static const struct file_operations fops_sii6400_mhl = {
	.open = sii6400_mhl_device_open,
	.release = sii6400_mhl_device_close
};

/*
 * Sii6400 MHL Attributes
 */
static ssize_t get_sii6400_mhl_dev_type(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_dev_type *dev_type = NULL;
	const char *dev_type_str = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dev_type_str = "TV\n";
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_DEV_TYPE,
				NULL, 0,
				get_mhl_connection_state_resp_q,
				WAIT_TIME_FOR_GET_MHL_CONNECTION_STATE_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*dev_type) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		goto done;
	}

	dev_type = resp_data;

done:
	if (NULL != dev_type) {
		switch (dev_type->type) {
		case MHL_DEV_TYPE_ARMET:
			dev_type_str = "ARMET\n";
			break;

		default:
			dev_type_str = "TV\n";
			break;
		}
	}

	if (NULL != dev_type_str)
		rv = scnprintf(buf, PAGE_SIZE, "%s", dev_type_str);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_mhl_connection_state(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_connection_state *connect_state = NULL;
	const char *connect_state_str = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		connect_state_str = "not connected";
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_CONNECTION_STATE,
				NULL, 0,
				get_mhl_connection_state_resp_q,
				WAIT_TIME_FOR_GET_MHL_CONNECTION_STATE_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*connect_state) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		goto done;
	}

	connect_state = resp_data;

done:
	if (NULL != connect_state) {
		switch (connect_state->connect_state) {
		case MHL_STATE_CONNECTED:
			connect_state_str = "connected";
			break;

		default:
		case MHL_STATE_NONE:
		case MHL_STATE_DISCONNECTED:
			connect_state_str = "not connected";
			break;
		}
	}

	if (NULL != connect_state_str)
		rv = scnprintf(buf, PAGE_SIZE, "%s", connect_state_str);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

int get_sii6400_mhl_id_impedance_after_mode_changed(void)
{
	struct sii6400_id_impedance *id_impedance = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	err("");

	sm_rv = sm_send_request(mhl_sm_queue,
			SII_SM_REQ_GET_MHL_ID_IMPEDANCE,
			NULL, 0,
			get_mhl_id_impedance_resp_q,
			WAIT_TIME_FOR_GET_MHL_ID_IMPEDANCE_RESPONSE,
			&resp_data, &resp_data_size);

	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("MHL State Machine response time out\n");
		return sm_rv;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		return sm_rv;
	}

	if ((NULL != resp_data) &&
		(sizeof(*id_impedance) == resp_data_size)) {
		id_impedance = resp_data;
	} else {
		err("Invalid MHL State Machine response\n");
		return SII_OS_STATUS_ERR_NOT_AVAIL;
	}

	if ((0x00000320>= id_impedance->ohms) ||
		(id_impedance->ohms>=0x000004B0)) {
		err("no MHL peer device response\n");
		(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_DISCONNECTED,
					NULL, 0, NULL, 0, NULL, NULL);
	} else
		err("MHL peer device response\n");

	return sm_rv;
}

static ssize_t get_sii6400_mhl_id_impedance(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_id_impedance *id_impedance = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	switch (devinfo->mode) {
	case SII6400_MODE_WIHD:
	{
		sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_WIHD_ID_IMPEDANCE,
				NULL, 0,
				get_wihd_id_impedance_resp_q,
				WAIT_TIME_FOR_GET_WIHD_ID_IMPEDANCE_RESPONSE,
				&resp_data, &resp_data_size);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("MHL State Machine response time out\n");
			retval = -ETIME;
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("MHL State Machine request send failure\n");
			retval = -ENODEV;
			goto done;
		}

		if ((NULL != resp_data) &&
		    (sizeof(*id_impedance) == resp_data_size)) {
			id_impedance = resp_data;
		} else {
			err("Invalid MHL State Machine response\n");
			retval = -ENODEV;
			goto done;
		}
	}
		break;

	case SII6400_MODE_MHL:
	{
		sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_ID_IMPEDANCE,
				NULL, 0,
				get_mhl_id_impedance_resp_q,
				WAIT_TIME_FOR_GET_MHL_ID_IMPEDANCE_RESPONSE,
				&resp_data, &resp_data_size);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("MHL State Machine response time out\n");
			retval = -ETIME;
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("MHL State Machine request send failure\n");
			retval = -ENODEV;
			goto done;
		}

		if ((NULL != resp_data) &&
		    (sizeof(*id_impedance) == resp_data_size)) {
			id_impedance = resp_data;
		} else {
			err("Invalid MHL State Machine response\n");
			retval = -ENODEV;
			goto done;
		}
	}
		break;

	case SII6400_MODE_CONFIG:
	case SII6400_MODE_OFF:
	case SII6400_MODE_UNDEFINED:
	default:
		dbg("ID Impedance not available in current mode\n");
		retval = -ENODEV;
		break;
	}

done:
	if (NULL != id_impedance)
		rv = scnprintf(buf, PAGE_SIZE, "0x%08x", id_impedance->ohms);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_mhl_clock_swing(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_clock_swing *clock_swing = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
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

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_CLOCK_SWING,
				NULL, 0,
				get_mhl_clock_swing_resp_q,
				HM_MHL_GET_CLOCK_SWING_RESPONSE_TIMEOUT,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*clock_swing) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		retval = -ENODEV;
		goto done;
	}

	clock_swing = resp_data;

done:
	if (NULL != clock_swing)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x",
					clock_swing->clock_swing);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_clock_swing(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t clock_value = 0;
	struct sii6400_clock_swing *clock_swing = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL clock swing not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 != sscanf(buf, "0x%hhx", &clock_value)) {
		warn("Invalid MHL clock swing value input: %s", buf);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	clock_swing = SiiOsCalloc("", sizeof(*clock_swing), 0);
	if (NULL == clock_swing) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	clock_swing->clock_swing = clock_value;

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_SET_MHL_CLOCK_SWING,
				clock_swing, sizeof(*clock_swing),
				set_mhl_clock_swing_resp_q,
				HM_MHL_SET_CLOCK_SWING_RESPONSE_TIMEOUT,
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

static ssize_t set_sii6400_mhl_cbus_voltage(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t high = 0;
	uint16_t period = 0;
	struct sii6400_cbus_voltage *cbus_voltage = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL clock swing not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 != sscanf(buf, "0x%hhx 0x%hx", &high, &period)) {
		warn("Invalid MHL cbus voltage input: %s", buf);
		retval = -EINVAL;
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


/*
 * Declare the sysfs entries for Sii6400 MHL Attributes.
 * These macros create instances of:
 *   dev_attr_connection_state
 *   dev_attr_id_impedance
 */
static DEVICE_ATTR(connection_state, (S_IRUGO),
			get_sii6400_mhl_connection_state, NULL);
static DEVICE_ATTR(id_impedance, (S_IRUGO),
			get_sii6400_mhl_id_impedance, NULL);
//static DEVICE_ATTR(clock_swing, ((S_IRUGO) | (S_IWU)),
static DEVICE_ATTR(clock_swing, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
			get_sii6400_mhl_clock_swing,
			set_sii6400_mhl_clock_swing);
//leijun
//static DEVICE_ATTR(cbus_voltage, (S_IWU),
static DEVICE_ATTR(cbus_voltage, (S_IWUSR|S_IWGRP),
			NULL, set_sii6400_mhl_cbus_voltage);
static DEVICE_ATTR(devtype, (S_IRUGO),
			get_sii6400_mhl_dev_type, NULL);

static struct attribute *sii6400_mhl_attrs[] = {
	&dev_attr_connection_state.attr,
	&dev_attr_id_impedance.attr,
	&dev_attr_clock_swing.attr,
	&dev_attr_cbus_voltage.attr,
	&dev_attr_devtype.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_mhl_attr_group = {
	.attrs = sii6400_mhl_attrs,
};

/*
 * Sii6400 MHL Devcap Group Attributes
 */
static ssize_t get_sii6400_mhl_devcap_local(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_devcap *devcap = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL Local Device Cap not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_LOCAL_DEVCAP,
				NULL, 0,
				get_mhl_local_devcap_resp_q,
				WAIT_TIME_FOR_GET_MHL_LOCAL_DEVCAP_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*devcap) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		goto done;
	}

	devcap = resp_data;

done:
	if (NULL != devcap)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", devcap->data[0]);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_devcap_local(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	unsigned int devcap = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_devcap *local_devcap = NULL;
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

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 != sscanf(buf, "0x%x", &devcap)) {
		warn("Invalid MHL Local Device Capability %s", buf);
		retval = -EINVAL;
		goto done;
	}

	if (0xff < devcap) {
		warn("Invalid MHL Local Device Capability %u\n", devcap);
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL Local Device Cap not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	/* This memory is freed in the state machine. */
	local_devcap = SiiOsCalloc("MHLLocalDevCap", sizeof(*local_devcap), 0);
	if (NULL == local_devcap) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	local_devcap->data[0] = (uint8_t)devcap;
	local_devcap->length = 1;
	local_devcap->offset = 0;

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_SET_MHL_LOCAL_DEVCAP,
				local_devcap, sizeof(*local_devcap),
				set_mhl_local_devcap_resp_q,
				WAIT_TIME_FOR_SET_MHL_LOCAL_DEVCAP_RESPONSE,
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

static ssize_t get_sii6400_mhl_devcap_local_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_devcap_offset *devcap_offset = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL Local Dev Cap Offset unavailable in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
			SII_SM_REQ_GET_MHL_LOCAL_DEVCAP_OFFSET,
			NULL, 0,
			get_mhl_local_devcap_offset_resp_q,
			WAIT_TIME_FOR_GET_MHL_LOCAL_DEVCAP_OFFSET_RESPONSE,
			&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*devcap_offset) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		goto done;
	}

	devcap_offset = resp_data;

done:
	if (NULL != devcap_offset)
		rv = scnprintf(buf, PAGE_SIZE, "%u", devcap_offset->offset);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_devcap_local_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	unsigned long offset = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_devcap_offset *local_devcap_offset = NULL;
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

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &offset);
	if (rv) {
		warn("Invalid MHL Local Device Capability Offset %s", buf);
		retval = rv;
		goto done;
	}

	if (MAX_DEVCAP_OFFSET < offset) {
		warn("Invalid MHL Local Device Capability Offset %lu\n",
		     offset);
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL Local Dev Cap Offset unavailable in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	/* This memory is freed in the state machine. */
	local_devcap_offset = SiiOsCalloc("MHLLocalDevCapOffset",
					sizeof(*local_devcap_offset), 0);
	if (NULL == local_devcap_offset) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	local_devcap_offset->offset = offset;

	sm_rv = sm_send_request(mhl_sm_queue,
			SII_SM_REQ_SET_MHL_LOCAL_DEVCAP_OFFSET,
			local_devcap_offset, sizeof(*local_devcap_offset),
			set_mhl_local_devcap_offset_resp_q,
			WAIT_TIME_FOR_SET_MHL_LOCAL_DEVCAP_OFFSET_RESPONSE,
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

static ssize_t get_sii6400_mhl_devcap_remote(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_devcap *devcap = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL Remote Dev Cap not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_REMOTE_DEVCAP,
				NULL, 0,
				get_mhl_remote_devcap_resp_q,
				WAIT_TIME_FOR_GET_MHL_REMOTE_DEVCAP_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_ERR_NOT_AVAIL == sm_rv) {
		err("MHL Remote Device Capablities not available\n");
		retval = -ENODEV;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*devcap) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		goto done;
	}

	devcap = resp_data;

done:
	if (NULL != devcap)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", devcap->data[0]);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_mhl_devcap_remote_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_devcap_offset *devcap_offset = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL Remote Dev Cap Offset unavailable in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
			SII_SM_REQ_GET_MHL_REMOTE_DEVCAP_OFFSET,
			NULL, 0,
			get_mhl_remote_devcap_offset_resp_q,
			WAIT_TIME_FOR_GET_MHL_REMOTE_DEVCAP_OFFSET_RESPONSE,
			&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*devcap_offset) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		goto done;
	}

	devcap_offset = resp_data;

done:
	if (NULL != devcap_offset)
		rv = scnprintf(buf, PAGE_SIZE, "%u", devcap_offset->offset);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_devcap_remote_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	unsigned long offset = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_devcap_offset *remote_devcap_offset = NULL;
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

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &offset);
	if (rv) {
		warn("Invalid MHL Remote Device Capability Offset %s", buf);
		retval = rv;
		goto done;
	}

	if (MAX_DEVCAP_OFFSET < offset) {
		warn("Invalid MHL Remote Device Capability Offset %lu\n",
		     offset);
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL Remote Dev Cap Offset unavailable in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	/* This memory is freed in the state machine. */
	remote_devcap_offset = SiiOsCalloc("MHLRemoteDevCapOffset",
					sizeof(*remote_devcap_offset), 0);
	if (NULL == remote_devcap_offset) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	remote_devcap_offset->offset = offset;

	sm_rv = sm_send_request(mhl_sm_queue,
			SII_SM_REQ_SET_MHL_REMOTE_DEVCAP_OFFSET,
			remote_devcap_offset, sizeof(*remote_devcap_offset),
			set_mhl_remote_devcap_offset_resp_q,
			WAIT_TIME_FOR_SET_MHL_REMOTE_DEVCAP_OFFSET_RESPONSE,
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

/*
 * Declare the sysfs entries for Sii6400 MHL Devcap Group Attributes.
 */
static struct device_attribute dev_attr_devcap_local =
	__ATTR(local, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_mhl_devcap_local,
		set_sii6400_mhl_devcap_local);
static struct device_attribute dev_attr_devcap_local_offset =
	__ATTR(local_offset, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_mhl_devcap_local_offset,
		set_sii6400_mhl_devcap_local_offset);
static struct device_attribute dev_attr_devcap_remote =
	__ATTR(remote, (S_IRUGO), get_sii6400_mhl_devcap_remote, NULL);
static struct device_attribute dev_attr_devcap_remote_offset =
	__ATTR(remote_offset, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_mhl_devcap_remote_offset,
		set_sii6400_mhl_devcap_remote_offset);

static struct attribute *sii6400_mhl_devcap_attrs[] = {
	&dev_attr_devcap_local.attr,
	&dev_attr_devcap_local_offset.attr,
	&dev_attr_devcap_remote.attr,
	&dev_attr_devcap_remote_offset.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_mhl_devcap_attr_group = {
	.name = __stringify(devcap),
	.attrs = sii6400_mhl_devcap_attrs,
};

/*
 * Sii6400 MHL RAP Group Attributes
 */
static ssize_t get_sii6400_mhl_rap_in(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_rap *rap = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RAP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (INPUT_DEVICE_APP != devinfo->my_rap_input_device) {
		dbg("MHL RAP Input not available (input_dev != 0)\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
			SII_SM_REQ_GET_MHL_RAP_RCVD_ACTIONCODE,
			NULL, 0,
			get_mhl_rap_rcvd_actioncode_resp_q,
			WAIT_TIME_FOR_GET_MHL_RAP_RCVD_ACTIONCODE_RESPONSE,
			&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*rap) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		retval = -ENODEV;
		goto done;
	}

	rap = resp_data;

done:
	if (NULL != rap)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", rap->actioncode);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_rap_in_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t rap_status = 0;
	struct sii6400_mhl_rap_status *rap_ack = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RAP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (INPUT_DEVICE_APP != devinfo->my_rap_input_device) {
		dbg("RAP Input Status not available (input_dev != 0)\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 != sscanf(buf, "0x%hhx", &rap_status)) {
		warn("Invalid Send MHL RAP Status input: %s", buf);
		retval = -EINVAL;
		goto done;
	}

	if (MAX_MHL_RAP_STATUS < rap_status) {
		warn("Invalid MHL MHL RAP Status input %u\n", rap_status);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	rap_ack = SiiOsCalloc("SendMHLRAPACK", sizeof(*rap_ack), 0);
	if (NULL == rap_ack) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	rap_ack->status = rap_status;

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_SEND_MHL_RAP_ACK,
				rap_ack, sizeof(*rap_ack),
				send_mhl_rap_ack_resp_q,
				WAIT_TIME_FOR_SEND_MHL_RAP_ACK_RESPONSE,
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

static ssize_t get_sii6400_mhl_rap_out(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_rap *rap = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RAP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
			SII_SM_REQ_GET_MHL_RAP_SENT_ACTIONCODE,
			NULL, 0,
			get_mhl_rap_sent_actioncode_resp_q,
			WAIT_TIME_FOR_GET_MHL_RAP_SENT_ACTIONCODE_RESPONSE,
			&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*rap) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		retval = -ENODEV;
		goto done;
	}

	rap = resp_data;

done:
	if (NULL != rap)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", rap->actioncode);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_rap_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t actioncode = 0;
	struct sii6400_mhl_rap *rap = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RAP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 != sscanf(buf, "0x%hhx", &actioncode)) {
		warn("Invalid Send MHL RAP actioncode input: %s", buf);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	rap = SiiOsCalloc("SendMHLRAPActioncode", sizeof(*rap), 0);
	if (NULL == rap) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	rap->actioncode = actioncode;

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_SEND_MHL_RAP_ACTIONCODE,
				rap, sizeof(*rap),
				send_mhl_rap_actioncode_resp_q,
				WAIT_TIME_FOR_SEND_MHL_RAP_ACTIONCODE_RESPONSE,
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

static ssize_t get_sii6400_mhl_rap_out_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_rap_status *rap_ack = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RAP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_RAP_ACK,
				NULL, 0,
				get_mhl_rap_ack_resp_q,
				WAIT_TIME_FOR_GET_MHL_RAP_ACK_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*rap_ack) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		retval = -ENODEV;
		goto done;
	}

	rap_ack = resp_data;

done:
	if (NULL != rap_ack)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", rap_ack->status);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_mhl_rap_input_dev(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RAP Input Device not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

done:
	if (NULL != devinfo) {
		rv = scnprintf(buf, PAGE_SIZE, "%u",
				devinfo->my_rap_input_device);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_rap_input_dev(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	unsigned long input_dev_type = 0;

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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RAP Input Device not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &input_dev_type);
	if (rv) {
		warn("Invalid RAP Input Device input: %s", buf);
		retval = rv;
		goto done;
	}

	if ((INPUT_DEVICE_APP != input_dev_type) &&
	    (INPUT_DEVICE_DRIVER != input_dev_type)) {
		warn("Invalid RAP Input Device input: %lu\n", input_dev_type);
		retval = -EINVAL;
		goto done;
	}

	devinfo->my_rap_input_device = input_dev_type;

done:
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 MHL RAP Group Attributes.
 */
static struct device_attribute dev_attr_rap_in =
	__ATTR(in, (S_IRUGO), get_sii6400_mhl_rap_in, NULL);
static struct device_attribute dev_attr_rap_in_status =
	__ATTR(in_status, (S_IWUSR|S_IWGRP), NULL, set_sii6400_mhl_rap_in_status);
static struct device_attribute dev_attr_rap_out =
	__ATTR(out, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_mhl_rap_out,
		set_sii6400_mhl_rap_out);
static struct device_attribute dev_attr_rap_out_status =
	__ATTR(out_status, (S_IRUGO), get_sii6400_mhl_rap_out_status, NULL);
static struct device_attribute dev_attr_rap_input_dev =
	__ATTR(input_dev, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_mhl_rap_input_dev,
		set_sii6400_mhl_rap_input_dev);

static struct attribute *sii6400_mhl_rap_attrs[] = {
	&dev_attr_rap_in.attr,
	&dev_attr_rap_in_status.attr,
	&dev_attr_rap_out.attr,
	&dev_attr_rap_out_status.attr,
	&dev_attr_rap_input_dev.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_mhl_rap_attr_group = {
	.name = __stringify(rap),
	.attrs = sii6400_mhl_rap_attrs,
};

/*
 * Sii6400 MHL RCP Group Attributes
 */
static ssize_t get_sii6400_mhl_rcp_in(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_rcp *rcp = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (INPUT_DEVICE_APP != devinfo->my_rcp_input_device) {
		dbg("RCP Input not available (input_dev != 0)\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_RCP_RCVD_KEYCODE,
				NULL, 0,
				get_mhl_rcp_rcvd_keycode_resp_q,
				WAIT_TIME_FOR_GET_MHL_RCP_RCVD_KEYCODE_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*rcp) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		retval = -ENODEV;
		goto done;
	}

	rcp = resp_data;

done:
	if (NULL != rcp)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", rcp->keycode);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_rcp_in_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t rcp_status = 0;
	struct sii6400_mhl_rcp_status *rcp_ack = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (INPUT_DEVICE_APP != devinfo->my_rcp_input_device) {
		dbg("RCP Input Status not available (input_dev != 0)\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 != sscanf(buf, "0x%hhx", &rcp_status)) {
		warn("Invalid Send MHL RCP Status input: %s", buf);
		retval = -EINVAL;
		goto done;
	}

	if (MAX_MHL_RCP_STATUS < rcp_status) {
		warn("Invalid MHL MHL RCP Status input %u\n", rcp_status);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	rcp_ack = SiiOsCalloc("SendMHLRCPACK", sizeof(*rcp_ack), 0);
	if (NULL == rcp_ack) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	rcp_ack->status = rcp_status;

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_SEND_MHL_RCP_ACK,
				rcp_ack, sizeof(*rcp_ack),
				send_mhl_rcp_ack_resp_q,
				WAIT_TIME_FOR_SEND_MHL_RCP_ACK_RESPONSE,
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

static ssize_t get_sii6400_mhl_rcp_out(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_rcp *rcp = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_RCP_SENT_KEYCODE,
				NULL, 0,
				get_mhl_rcp_sent_keycode_resp_q,
				WAIT_TIME_FOR_GET_MHL_RCP_SENT_KEYCODE_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*rcp) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		retval = -ENODEV;
		goto done;
	}

	rcp = resp_data;

done:
	if (NULL != rcp)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", rcp->keycode);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_rcp_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t keycode = 0;
	struct sii6400_mhl_rcp *rcp = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 != sscanf(buf, "0x%hhx", &keycode)) {
		warn("Invalid Send MHL RCP keycode input: %s", buf);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	rcp = SiiOsCalloc("SendMHLRCPKeycode", sizeof(*rcp), 0);
	if (NULL == rcp) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	rcp->keycode = keycode;

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_SEND_MHL_RCP_KEYCODE,
				rcp, sizeof(*rcp),
				send_mhl_rcp_keycode_resp_q,
				WAIT_TIME_FOR_SEND_MHL_RCP_KEYCODE_RESPONSE,
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

static ssize_t get_sii6400_mhl_rcp_out_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_rcp_status *rcp_ack = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_RCP_ACK,
				NULL, 0,
				get_mhl_rcp_ack_resp_q,
				WAIT_TIME_FOR_GET_MHL_RCP_ACK_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*rcp_ack) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		retval = -ENODEV;
		goto done;
	}

	rcp_ack = resp_data;

done:
	if (NULL != rcp_ack)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", rcp_ack->status);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_mhl_rcp_input_dev(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

done:
	if (NULL != devinfo) {
		rv = scnprintf(buf, PAGE_SIZE, "%u",
				devinfo->my_rcp_input_device);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_rcp_input_dev(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	unsigned long input_dev_type = 0;

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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL RCP Input Device not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &input_dev_type);
	if (rv) {
		warn("Invalid RCP Input Device input: %s", buf);
		retval = rv;
		goto done;
	}

	if ((INPUT_DEVICE_APP != input_dev_type) &&
	    (INPUT_DEVICE_DRIVER != input_dev_type)) {
		warn("Invalid RCP Input Device input: %lu\n", input_dev_type);
		retval = -EINVAL;
		goto done;
	}

	devinfo->my_rcp_input_device = input_dev_type;

done:
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 MHL RCP Group Attributes.
 */
static struct device_attribute dev_attr_rcp_in =
	__ATTR(in, (S_IRUGO), get_sii6400_mhl_rcp_in, NULL);
static struct device_attribute dev_attr_rcp_in_status =
	__ATTR(in_status, (S_IWUSR|S_IWGRP), NULL, set_sii6400_mhl_rcp_in_status);
static struct device_attribute dev_attr_rcp_out =
	__ATTR(out, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_mhl_rcp_out,
		set_sii6400_mhl_rcp_out);
static struct device_attribute dev_attr_rcp_out_status =
	__ATTR(out_status, (S_IRUGO), get_sii6400_mhl_rcp_out_status, NULL);
static struct device_attribute dev_attr_rcp_input_dev =
	__ATTR(input_dev, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_mhl_rcp_input_dev,
		set_sii6400_mhl_rcp_input_dev);

static struct attribute *sii6400_mhl_rcp_attrs[] = {
	&dev_attr_rcp_in.attr,
	&dev_attr_rcp_in_status.attr,
	&dev_attr_rcp_out.attr,
	&dev_attr_rcp_out_status.attr,
	&dev_attr_rcp_input_dev.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_mhl_rcp_attr_group = {
	.name = __stringify(rcp),
	.attrs = sii6400_mhl_rcp_attrs,
};

/*
 * Sii6400 MHL UCP Group Attributes
 */
static ssize_t get_sii6400_mhl_ucp_in(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_ucp *ucp = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL UCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (INPUT_DEVICE_APP != devinfo->my_ucp_input_device) {
		dbg("UCP Input not available (input_dev != 0)\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
			SII_SM_REQ_GET_MHL_UCP_RCVD_CHARCODE,
			NULL, 0,
			get_mhl_ucp_rcvd_charcode_resp_q,
			WAIT_TIME_FOR_GET_MHL_UCP_RCVD_CHARCODE_RESPONSE,
			&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*ucp) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		retval = -ENODEV;
		goto done;
	}

	ucp = resp_data;

done:
	if (NULL != ucp)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", ucp->charcode);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_ucp_in_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t ucp_status = 0;
	struct sii6400_mhl_ucp_status *ucp_ack = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL UCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (INPUT_DEVICE_APP != devinfo->my_ucp_input_device) {
		dbg("UCP Input Status not available (input_dev != 0)\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 != sscanf(buf, "0x%hhx", &ucp_status)) {
		warn("Invalid Send MHL UCP Status input: %s", buf);
		retval = -EINVAL;
		goto done;
	}

	if (MAX_MHL_UCP_STATUS < ucp_status) {
		warn("Invalid MHL MHL UCP Status input %u\n", ucp_status);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	ucp_ack = SiiOsCalloc("SendMHLUCPACK", sizeof(*ucp_ack), 0);
	if (NULL == ucp_ack) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	ucp_ack->status = ucp_status;

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_SEND_MHL_UCP_ACK,
				ucp_ack, sizeof(*ucp_ack),
				send_mhl_ucp_ack_resp_q,
				WAIT_TIME_FOR_SEND_MHL_UCP_ACK_RESPONSE,
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

static ssize_t get_sii6400_mhl_ucp_out(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_ucp *ucp = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL UCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
			SII_SM_REQ_GET_MHL_UCP_SENT_CHARCODE,
			NULL, 0,
			get_mhl_ucp_sent_charcode_resp_q,
			WAIT_TIME_FOR_GET_MHL_UCP_SENT_CHARCODE_RESPONSE,
			&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*ucp) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		retval = -ENODEV;
		goto done;
	}

	ucp = resp_data;

done:
	if (NULL != ucp)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", ucp->charcode);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_ucp_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t charcode = 0;
	struct sii6400_mhl_ucp *ucp = NULL;
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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL UCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 != sscanf(buf, "0x%hhx", &charcode)) {
		warn("Invalid Send MHL UCP charcode input: %s", buf);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	ucp = SiiOsCalloc("SendMHLUCPCharcode", sizeof(*ucp), 0);
	if (NULL == ucp) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	ucp->charcode = charcode;

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_SEND_MHL_UCP_CHARCODE,
				ucp, sizeof(*ucp),
				send_mhl_ucp_charcode_resp_q,
				WAIT_TIME_FOR_SEND_MHL_UCP_CHARCODE_RESPONSE,
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

static ssize_t get_sii6400_mhl_ucp_out_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mhl_ucp_status *ucp_ack = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL UCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_UCP_ACK,
				NULL, 0,
				get_mhl_ucp_ack_resp_q,
				WAIT_TIME_FOR_GET_MHL_UCP_ACK_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*ucp_ack) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		retval = -ENODEV;
		goto done;
	}

	ucp_ack = resp_data;

done:
	if (NULL != ucp_ack)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", ucp_ack->status);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_mhl_ucp_input_dev(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if (sii6400_module_is_exiting()) {
		retval = -ENODEV;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL UCP not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

done:
	if (NULL != devinfo) {
		rv = scnprintf(buf, PAGE_SIZE, "%u",
				devinfo->my_ucp_input_device);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	return retval ? retval : rv;
}

static ssize_t set_sii6400_mhl_ucp_input_dev(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	unsigned long input_dev_type = 0;

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

	if (SII6400_MODE_MHL != devinfo->mode) {
		dbg("MHL UCP Input Device not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &input_dev_type);
	if (rv) {
		warn("Invalid UCP Input Device input: %s", buf);
		retval = rv;
		goto done;
	}

	if ((INPUT_DEVICE_APP != input_dev_type) &&
	    (INPUT_DEVICE_DRIVER != input_dev_type)) {
		warn("Invalid UCP Input Device input: %lu\n", input_dev_type);
		retval = -EINVAL;
		goto done;
	}

	devinfo->my_ucp_input_device = input_dev_type;

done:
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 MHL UCP Group Attributes.
 */
static struct device_attribute dev_attr_ucp_in =
	__ATTR(in, (S_IRUGO), get_sii6400_mhl_ucp_in, NULL);
static struct device_attribute dev_attr_ucp_in_status =
	__ATTR(in_status, (S_IWUSR|S_IWGRP), NULL, set_sii6400_mhl_ucp_in_status);
static struct device_attribute dev_attr_ucp_out =
	__ATTR(out, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_mhl_ucp_out,
		set_sii6400_mhl_ucp_out);
static struct device_attribute dev_attr_ucp_out_status =
	__ATTR(out_status, (S_IRUGO), get_sii6400_mhl_ucp_out_status, NULL);
static struct device_attribute dev_attr_ucp_input_dev =
	__ATTR(input_dev, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_mhl_ucp_input_dev,
		set_sii6400_mhl_ucp_input_dev);

static struct attribute *sii6400_mhl_ucp_attrs[] = {
	&dev_attr_ucp_in.attr,
	&dev_attr_ucp_in_status.attr,
	&dev_attr_ucp_out.attr,
	&dev_attr_ucp_out_status.attr,
	&dev_attr_ucp_input_dev.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_mhl_ucp_attr_group = {
	.name = __stringify(ucp),
	.attrs = sii6400_mhl_ucp_attrs,
};

/* MHL device initialization and release */
int sii6400_mhl_dev_add(struct sii6400_device_info *devinfo)
{
	int retval = 0;
	struct sii6400_mhl_device_info *devinfo_mhl = NULL;

	dbg("");

	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -EFAULT;
		goto failed_nullptr;
	}

	devinfo_mhl = SiiOsCalloc("MHLDevAdd", sizeof(*devinfo_mhl), 0);
	if (NULL == devinfo_mhl) {
		err("Out of memory\n");
		retval = -ENOMEM;
		goto failed_memalloc;
	}

	devinfo_mhl->devnum = MKDEV(MAJOR(devinfo->devnum), 2);

	devinfo_mhl->cdev = cdev_alloc();
	devinfo_mhl->cdev->owner = THIS_MODULE;
	devinfo_mhl->cdev->ops = &fops_sii6400_mhl;
	retval = cdev_add(devinfo_mhl->cdev, devinfo_mhl->devnum, 1);
	if (retval)
		goto failed_chrmhladd;

	devinfo->mhl = devinfo_mhl;

	return 0;

failed_chrmhladd:
	SiiOsFree(devinfo_mhl);
failed_memalloc:
failed_nullptr:
	return retval;
}

int sii6400_mhl_dev_init(struct sii6400_device_info *devinfo)
{
	int retval = 0;

	dbg("");

	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		retval = -EFAULT;
		goto failed_nullptr;
	}

	devinfo->mhl->device = device_create(devinfo->dev_class,
					devinfo->device, devinfo->mhl->devnum,
					NULL, SII6400_MHL_DEVNAME);
	if (IS_ERR(devinfo->mhl->device)) {
		retval = PTR_ERR(devinfo->mhl->device);
		goto failed_mhlreg;
	}

	retval = sysfs_create_group(&devinfo->mhl->device->kobj,
					&sii6400_mhl_attr_group);
	if (retval < 0)
		warn("failed to create MHL attribute group\n");

	retval = sysfs_create_group(&devinfo->mhl->device->kobj,
					&sii6400_mhl_devcap_attr_group);
	if (retval < 0)
		warn("failed to create MHL devcap attribute group\n");

	retval = sysfs_create_group(&devinfo->mhl->device->kobj,
					&sii6400_mhl_rap_attr_group);
	if (retval < 0)
		warn("failed to create MHL rap attribute group\n");

	retval = sysfs_create_group(&devinfo->mhl->device->kobj,
					&sii6400_mhl_rcp_attr_group);
	if (retval < 0)
		warn("failed to create MHL rcp attribute group\n");

	retval = sysfs_create_group(&devinfo->mhl->device->kobj,
					&sii6400_mhl_ucp_attr_group);
	if (retval < 0)
		warn("failed to create MHL ucp attribute group\n");

	return 0;

failed_mhlreg:
failed_nullptr:
	return retval;
}

void sii6400_mhl_dev_exit(struct sii6400_device_info *devinfo)
{
	dbg("");

	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		return;
	}

	sysfs_remove_group(&devinfo->mhl->device->kobj,
				&sii6400_mhl_attr_group);
	sysfs_remove_group(&devinfo->mhl->device->kobj,
				&sii6400_mhl_devcap_attr_group);
	sysfs_remove_group(&devinfo->mhl->device->kobj,
				&sii6400_mhl_rap_attr_group);
	sysfs_remove_group(&devinfo->mhl->device->kobj,
				&sii6400_mhl_rcp_attr_group);
	sysfs_remove_group(&devinfo->mhl->device->kobj,
				&sii6400_mhl_ucp_attr_group);

	device_unregister(devinfo->mhl->device);
	device_destroy(devinfo->dev_class, devinfo->mhl->devnum);
}

void sii6400_mhl_dev_remove(struct sii6400_device_info *devinfo)
{
	dbg("");

	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		return;
	}

	cdev_del(devinfo->mhl->cdev);
	SiiOsFree(devinfo->mhl);
	devinfo->mhl = NULL;
}

