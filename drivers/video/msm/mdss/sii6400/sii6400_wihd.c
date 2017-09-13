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

#include "osal.h"
#include "hal.h"
#include "sii6400.h"
#include "state_machine.h"
#include "wihd_sm.h"

/* A mutex will ensure that only one process accesses our device */
static DEFINE_MUTEX(sii6400_wihd_device_mutex);

/* static function prototypes */
static int sii6400_wihd_device_open(struct inode *inode, struct file *filp);
static int sii6400_wihd_device_close(struct inode *inode, struct file *filp);
static ssize_t request_sii6400_wihd_connect(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_state(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_cec_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_hdcp_policy(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_hdcp_policy(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_hdcp_stream_type(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_hdcp_stream_type(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_remote_control_in(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_wihd_remote_control_out(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_remote_control_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_remote_control_input_dev(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_remote_control_input_dev(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_remote_device_category(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_wihd_remote_device_mac_addr(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_wihd_remote_device_manufacturer(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_wihd_remote_device_monitor_name(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_wihd_remote_device_name(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t
get_sii6400_wihd_remote_device_signal_strength(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_wihd_remote_device_type(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t
set_sii6400_wihd_remote_device_fw_update_status(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t get_sii6400_wihd_self_connect_timeout(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_self_connect_timeout(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_self_mac_addr(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_wihd_self_name(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_self_name(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_wvan_connection_status(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_wihd_wvan_device_list(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t get_sii6400_wihd_wvan_join(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t request_sii6400_wihd_wvan_join(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_wvan_scan_enable(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_wvan_scan_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_wvan_scan_duration(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_wvan_scan_duration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_wvan_scan_interval(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_wvan_scan_interval(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t set_sii6400_wihd_vendor_msg_recv_filter(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t get_sii6400_wihd_vendor_msg_recv_filter(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t set_sii6400_wihd_vendor_msg_vendor_id(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t set_sii6400_wihd_vendor_msg_dest_mac_addr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
static ssize_t set_sii6400_wihd_vendor_msg_send(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);


static int sii6400_wihd_device_open(struct inode *inode, struct file *filp)
{
	int retval = 0;

	dbg("");

	if (NULL == filp) {
		err("Invalid filp pointer\n");
		retval = -EINVAL;
		goto done;
	}

	/* The Sii6400 WiHD device does not allow write access */
	if (((filp->f_flags & O_ACCMODE) == O_WRONLY) ||
	    ((filp->f_flags & O_ACCMODE) == O_RDWR)) {
		retval = -EACCES;
		goto done;
	}

	/* Ensure that only one process has access to the
	 * Sii6400 WiHD device at any one time. */
	if (!mutex_trylock(&sii6400_wihd_device_mutex)) {
		retval = -EBUSY;
		goto done;
	}

done:
	return retval;
}

static int sii6400_wihd_device_close(struct inode *inode, struct file *filp)
{
	dbg("");

	mutex_unlock(&sii6400_wihd_device_mutex);
	return 0;
}

static const struct file_operations fops_sii6400_wihd = {
	.open = sii6400_wihd_device_open,
	.release = sii6400_wihd_device_close
};

/*
 * Sii6400 WiHD Attributes
 */
static ssize_t request_sii6400_wihd_connect(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	char mac_addr_str[MAC_ADDRESS_STR_1_LEN + 1] = {0};
	uint64_t mac_addr = 0;
	struct sii6400_device_info *devinfo = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WiHD Connect / Disconnect not allowed in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	/* Use a max string length that is 1 longer than allowed to check for
	 * illegal (too long) entries */
	if (1 != sscanf(buf, " %" __stringify(MAC_ADDRESS_STR_1_LEN) "s",
			mac_addr_str)) {
		warn("Invalid Sii6400 WiHD Connect MAC Address %s", buf);
		retval = -EINVAL;
		goto done;
	}
	if (mac_addr_str[MAC_ADDRESS_STR_1_LEN - 1]) {
		warn("Invalid Sii6400 WiHD Connect MAC Address %s", buf);
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoull(mac_addr_str, 16, &mac_addr);
	if (rv) {
		warn("Invalid Sii6400 WiHD Connect MAC Address %s\n",
		     mac_addr_str);
		retval = rv;
		goto done;
	}

	if (0xffffffffffffULL < mac_addr) {
		warn("Invalid Sii6400 WiHD Connect MAC Address %s\n",
		     mac_addr_str);
		retval = -EINVAL;
		goto done;
	}

	if (0 != mac_addr) {
		struct sii6400_connect *connect_data = NULL;

		/* This memory is freed in the state machine. */
		connect_data = SiiOsCalloc("ConnectReqData",
						sizeof(*connect_data), 0);
		if (NULL == connect_data) {
			err("Out of memory\n");
			retval = -ENODEV;
			goto done;
		}

		connect_data->mac_addr[0] = (mac_addr >> 40) & 0xff;
		connect_data->mac_addr[1] = (mac_addr >> 32) & 0xff;
		connect_data->mac_addr[2] = (mac_addr >> 24) & 0xff;
		connect_data->mac_addr[3] = (mac_addr >> 16) & 0xff;
		connect_data->mac_addr[4] = (mac_addr >> 8) & 0xff;
		connect_data->mac_addr[5] = mac_addr & 0xff;

		connect_data->connect_timeout =
				devinfo->my_connect_timeout.connect_timeout;

		sm_rv = sm_send_request(wihd_sm_queue,
					SII_SM_REQ_WIHD_CONNECT,
					connect_data, sizeof(*connect_data),
					wihd_connect_resp_q,
					WAIT_TIME_FOR_WIHD_CONNECT_RESPONSE,
					NULL, NULL);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("WiHD State Machine response time out\n");
			retval = -ETIME;
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("WiHD State Machine request send failure\n");
			retval = -ENODEV;
			goto done;
		}
	} else {
		sm_rv = sm_send_request(wihd_sm_queue,
					SII_SM_REQ_WIHD_DISCONNECT,
					NULL, 0,
					wihd_disconnect_resp_q,
					WAIT_TIME_FOR_WIHD_DISCONNECT_RESPONSE,
					NULL, NULL);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("WiHD State Machine response time out\n");
			retval = -ETIME;
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("WiHD State Machine request send failure\n");
			retval = -ENODEV;
			goto done;
		}
	}

done:
	return retval;
}

static ssize_t get_sii6400_wihd_state(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	enum sii6400_wihd_state wihd_state = WIHD_STATE_NONE;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd))
		err("Invalid devinfo pointer\n");
	else if (SII6400_MODE_WIHD != devinfo->mode)
		dbg("No WiHD state info available in current mode\n");
	else
		wihd_state = devinfo->wihd->state;

	return scnprintf(buf, PAGE_SIZE, "%s",
			 get_wihd_state_string(wihd_state));
}

/*
 * Declare the sysfs entries for Sii6400 WiHD Attributes.
 * These macros create instances of:
 *   dev_attr_connect
 *   dev_attr_search
 *   dev_attr_state
 */
static DEVICE_ATTR(connect, ((S_IWUSR|S_IWGRP) | (S_IRUGO)),
			get_sii6400_wihd_remote_device_mac_addr,
			request_sii6400_wihd_connect);
static DEVICE_ATTR(state, (S_IRUGO), get_sii6400_wihd_state, NULL);

static struct attribute *sii6400_wihd_attrs[] = {
	&dev_attr_connect.attr,
	&dev_attr_state.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_wihd_attr_group = {
	.attrs = sii6400_wihd_attrs,
};

/*
 * Sii6400 WiHD CEC Group Attributes
 */
static ssize_t set_sii6400_wihd_cec_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int i;
	char cec_msg_str[CEC_MESSAGE_STR_1_LEN + 1] = {0};
	char *cur_buf = NULL;
	int length = 0;
	struct sii6400_cec_message *cec_message = NULL;
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

	/* Use a max string length that is 1 longer than allowed to check for
	 * illegal (too long) entries */
	if (1 != sscanf(buf, " %" __stringify(CEC_MESSAGE_STR_1_LEN) "s",
								cec_msg_str)) {
		warn("Invalid Sii6400 WiHD CEC Message %s", buf);
		retval = -EINVAL;
		goto done;
	}
	if (cec_msg_str[CEC_MESSAGE_STR_1_LEN - 1]) {
		warn("Invalid Sii6400 WiHD CEC Message %s", buf);
		retval = -EINVAL;
		goto done;
	}

	length = strlen(cec_msg_str) >> 1;
	if ((0 == length) || (MAX_CEC_MESSAGE_LEN < length)) {
		warn("Invalid Sii6400 WiHD CEC Message %s\n", cec_msg_str);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	cec_message = SiiOsCalloc("CECMessage", sizeof(*cec_message), 0);
	if (NULL == cec_message) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	cec_message->length = length;
	cur_buf = cec_msg_str;
	for (i = 0; i < length; i++) {
		char byte_str[3];

		byte_str[0] = *cur_buf++;
		byte_str[1] = *cur_buf++;
		byte_str[2] = 0;
		if (1 != sscanf(byte_str, "%hhx",
				&cec_message->cec_message[i])) {
			warn("Invalid Sii6400 WiHD CEC Message %s\n",
			     cec_msg_str);
			retval = -EINVAL;
			goto done;
		}
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_CEC_SEND_MSG, cec_message,
				sizeof(*cec_message),
				cec_send_message_resp_q,
				WAIT_TIME_FOR_CEC_SEND_MSG_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

done:
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 WiHD CEC Group Attributes.
 */
static struct device_attribute dev_attr_cec_out =
	__ATTR(out, (S_IWUSR|S_IWGRP), NULL, set_sii6400_wihd_cec_out);

static struct attribute *sii6400_wihd_cec_attrs[] = {
	&dev_attr_cec_out.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_wihd_cec_attr_group = {
	.name = __stringify(cec),
	.attrs = sii6400_wihd_cec_attrs,
};

/*
 * Sii6400 WiHD HDCP Group Attributes
 */
static ssize_t get_sii6400_wihd_hdcp_policy(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_hdcp_policy *hdcp_policy = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		hdcp_policy = &devinfo->my_hdcp_policy;
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_HDCP_POLICY,
				NULL, 0,
				get_hdcp_policy_resp_q,
				WAIT_TIME_FOR_GET_HDCP_POLICY_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*hdcp_policy) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	hdcp_policy = resp_data;

done:
	if (NULL != hdcp_policy)
		rv = scnprintf(buf, PAGE_SIZE, "%u", hdcp_policy->policy);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_wihd_hdcp_policy(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	unsigned long policy = HDCP_POLICY_DEFAULT;
	struct sii6400_device_info *devinfo = NULL;

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

	rv = strict_strtoul(buf, 0, &policy);
	if (rv) {
		warn("Invalid Sii6400 WIHD HDCP Policy %s", buf);
		retval = rv;
		goto done;
	}

	if (1 < policy) {
		warn("Invalid Sii6400 WIHD HDCP Policy %lu\n", policy);
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	devinfo->my_hdcp_policy.policy = policy;

	if (SII6400_MODE_WIHD == devinfo->mode) {
		struct sii6400_hdcp_policy *hdcp_policy = NULL;
		enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

		/* This memory is freed in the state machine. */
		hdcp_policy = SiiOsCalloc("HDCPPolicy",
						sizeof(*hdcp_policy), 0);
		if (NULL == hdcp_policy) {
			err("Out of memory\n");
			retval = -ENODEV;
			goto done;
		}
		memcpy(hdcp_policy, &devinfo->my_hdcp_policy,
			sizeof(*hdcp_policy));

		sm_rv = sm_send_request(wihd_sm_queue,
					SII_SM_REQ_SET_HDCP_POLICY,
					hdcp_policy, sizeof(*hdcp_policy),
					set_hdcp_policy_resp_q,
					WAIT_TIME_FOR_SET_HDCP_POLICY_RESPONSE,
					NULL, NULL);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("WiHD State Machine response time out\n");
			retval = -ETIME;
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("WiHD State Machine request send failure\n");
			retval = -ENODEV;
			goto done;
		}
	}

done:
	return retval;
}

static ssize_t get_sii6400_wihd_hdcp_stream_type(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_hdcp_stream_type *hdcp_stream_type = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		hdcp_stream_type = &devinfo->my_hdcp_stream_type;
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_HDCP_STREAM_TYPE,
				NULL, 0,
				get_hdcp_stream_type_resp_q,
				WAIT_TIME_FOR_GET_HDCP_STREAM_TYPE_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*hdcp_stream_type) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	hdcp_stream_type = resp_data;

done:
	if (NULL != hdcp_stream_type) {
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x",
			       hdcp_stream_type->stream_type);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_wihd_hdcp_stream_type(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	unsigned long stream_type = HDCP_STREAM_TYPE_DEFAULT;
	struct sii6400_device_info *devinfo = NULL;

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

	rv = strict_strtoul(buf, 0, &stream_type);
	if (rv) {
		warn("Invalid Sii6400 WIHD HDCP Stream Type %s", buf);
		retval = rv;
		goto done;
	}

	if (1 < stream_type) {
		warn("Invalid Sii6400 WIHD HDCP Stream Type %lu\n",
		     stream_type);
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	devinfo->my_hdcp_stream_type.stream_type = stream_type;

	if (SII6400_MODE_WIHD == devinfo->mode) {
		struct sii6400_hdcp_stream_type *hdcp_stream_type = NULL;
		enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

		/* This memory is freed in the state machine. */
		hdcp_stream_type = SiiOsCalloc("HDCPStreamType",
						sizeof(*hdcp_stream_type), 0);
		if (NULL == hdcp_stream_type) {
			err("Out of memory\n");
			retval = -ENODEV;
			goto done;
		}
		memcpy(hdcp_stream_type, &devinfo->my_hdcp_stream_type,
			sizeof(*hdcp_stream_type));

		sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_SET_HDCP_STREAM_TYPE,
				hdcp_stream_type, sizeof(*hdcp_stream_type),
				set_hdcp_stream_type_resp_q,
				WAIT_TIME_FOR_SET_HDCP_STREAM_TYPE_RESPONSE,
				NULL, NULL);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("WiHD State Machine response time out\n");
			retval = -ETIME;
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("WiHD State Machine request send failure\n");
			retval = -ENODEV;
			goto done;
		}
	}

done:
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 WiHD HDCP Group Attributes.
 */
static struct device_attribute dev_attr_hdcp_policy =
	__ATTR(policy, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_wihd_hdcp_policy,
		set_sii6400_wihd_hdcp_policy);
static struct device_attribute dev_attr_hdcp_stream_type =
	__ATTR(stream_type, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_wihd_hdcp_stream_type,
		set_sii6400_wihd_hdcp_stream_type);

static struct attribute *sii6400_wihd_hdcp_attrs[] = {
	&dev_attr_hdcp_policy.attr,
	&dev_attr_hdcp_stream_type.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_wihd_hdcp_attr_group = {
	.name = __stringify(hdcp),
	.attrs = sii6400_wihd_hdcp_attrs,
};

/*
 * Sii6400 WiHD Remote Control Group Attributes
 */
static ssize_t get_sii6400_wihd_remote_control_in(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_wihd_rc *rc = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WiHD Remote Control not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (INPUT_DEVICE_APP != devinfo->my_rc_input_device) {
		dbg("WiHD Remote Control Input not allowed (input_dev != 0)\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
			SII_SM_REQ_GET_WIHD_RC_RCVD_CTRLCODE,
			NULL, 0,
			get_wihd_rc_rcvd_ctrlcode_resp_q,
			WAIT_TIME_FOR_GET_WIHD_RC_RCVD_CTRLCODE_RESPONSE,
			&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_ERR_NOT_AVAIL == sm_rv) {
		err("WiHD Remote Control not available\n");
		retval = -ENODEV;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*rc) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	rc = resp_data;

done:
	if (NULL != rc)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", rc->ctrlcode);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_wihd_remote_control_out(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_wihd_rc *rc = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WiHD Remote Control not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
			SII_SM_REQ_GET_WIHD_RC_SENT_CTRLCODE,
			NULL, 0,
			get_wihd_rc_sent_ctrlcode_resp_q,
			WAIT_TIME_FOR_GET_WIHD_RC_SENT_CTRLCODE_RESPONSE,
			&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_ERR_NOT_AVAIL == sm_rv) {
		err("WiHD Remote Control not available\n");
		retval = -ENODEV;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*rc) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	rc = resp_data;

done:
	if (NULL != rc)
		rv = scnprintf(buf, PAGE_SIZE, "0x%02x", rc->ctrlcode);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_wihd_remote_control_out(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t ctrlcode = 0;
	struct sii6400_wihd_rc *rc = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WiHD Remote Control not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 != sscanf(buf, "0x%hhx", &ctrlcode)) {
		warn("Invalid Send WiHD Remote Control Code input: %s", buf);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	rc = SiiOsCalloc("SendWIHDRCCtrlcode", sizeof(*rc), 0);
	if (NULL == rc) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	rc->ctrlcode = ctrlcode;

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_SEND_WIHD_RC_CTRLCODE,
				rc, sizeof(*rc),
				send_wihd_rc_ctrlcode_resp_q,
				WAIT_TIME_FOR_SEND_WIHD_RC_CTRLCODE_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

done:
	return retval;
}

static ssize_t get_sii6400_wihd_remote_control_input_dev(struct device *dev,
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WiHD Remote Control not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

done:
	if (NULL != devinfo) {
		rv = scnprintf(buf, PAGE_SIZE, "%u",
			       devinfo->my_rc_input_device);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	return retval ? retval : rv;
}

static ssize_t set_sii6400_wihd_remote_control_input_dev(struct device *dev,
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WiHD Remote Control not available in current mode\n");
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
		warn("Invalid Remote Control Input Device input: %s", buf);
		retval = rv;
		goto done;
	}

	if ((INPUT_DEVICE_APP != input_dev_type) &&
	    (INPUT_DEVICE_DRIVER != input_dev_type)) {
		warn("Invalid Remote Control Input Device input: %lu\n",
		     input_dev_type);
		retval = -EINVAL;
		goto done;
	}

	devinfo->my_rc_input_device = input_dev_type;

done:
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 WiHD Remote Control Group Attributes.
 */
static struct device_attribute dev_attr_remote_control_in =
	__ATTR(in, (S_IRUGO), get_sii6400_wihd_remote_control_in, NULL);
static struct device_attribute dev_attr_remote_control_out =
	__ATTR(out, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_wihd_remote_control_out,
		set_sii6400_wihd_remote_control_out);
static struct device_attribute dev_attr_remote_control_input_dev =
	__ATTR(input_dev, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_wihd_remote_control_input_dev,
		set_sii6400_wihd_remote_control_input_dev);

static struct attribute *sii6400_wihd_remote_control_attrs[] = {
	&dev_attr_remote_control_in.attr,
	&dev_attr_remote_control_out.attr,
	&dev_attr_remote_control_input_dev.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_wihd_remote_control_attr_group = {
	.name = __stringify(remote_control),
	.attrs = sii6400_wihd_remote_control_attrs,
};

/*
 * Sii6400 WiHD Remote Device Group Attributes
 */
static ssize_t get_sii6400_wihd_remote_device_category(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_dev_category *remote_device_category = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Remote Devive Category not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_REMOTE_CATEGORY,
				NULL, 0,
				get_remote_category_resp_q,
				WAIT_TIME_FOR_GET_REMOTE_CATEGORY_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*remote_device_category) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	remote_device_category = resp_data;

done:
	if (NULL != remote_device_category) {
		rv = scnprintf(buf, PAGE_SIZE, "%s",
			       get_category_string(remote_device_category));
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_wihd_remote_device_mac_addr(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mac_addr *remote_device_mac_addr = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Remote Devive MAC Address not allowed in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_REMOTE_MAC_ADDR,
				NULL, 0,
				get_remote_mac_addr_resp_q,
				WAIT_TIME_FOR_GET_REMOTE_MAC_ADDR_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*remote_device_mac_addr) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	remote_device_mac_addr = resp_data;

done:
	if (NULL != remote_device_mac_addr) {
		rv = scnprintf(buf, PAGE_SIZE, "%02x%02x%02x%02x%02x%02x",
			       remote_device_mac_addr->mac_addr[0],
			       remote_device_mac_addr->mac_addr[1],
			       remote_device_mac_addr->mac_addr[2],
			       remote_device_mac_addr->mac_addr[3],
			       remote_device_mac_addr->mac_addr[4],
			       remote_device_mac_addr->mac_addr[5]);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_wihd_remote_device_manufacturer(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_manufacturer *remote_device_manufacturer = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Remote Devive Manufacturer not allowed in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_REMOTE_MANUFACTURER,
				NULL, 0,
				get_remote_manufacturer_resp_q,
				WAIT_TIME_FOR_GET_REMOTE_MANUFACTURER_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*remote_device_manufacturer) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	remote_device_manufacturer = resp_data;

done:
	if (NULL != remote_device_manufacturer) {
		rv = scnprintf(buf, PAGE_SIZE, "%s",
			       remote_device_manufacturer->manufacturer);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_wihd_remote_device_monitor_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_monitor_name *remote_device_monitor_name = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Remote Devive Monitor Name not allowed in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_REMOTE_MONITOR_NAME,
				NULL, 0,
				get_remote_monitor_name_resp_q,
				WAIT_TIME_FOR_GET_REMOTE_MONITOR_NAME_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*remote_device_monitor_name) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	remote_device_monitor_name = resp_data;

done:
	if (NULL != remote_device_monitor_name) {
		rv = scnprintf(buf, PAGE_SIZE, "%s",
			       remote_device_monitor_name->monitor_name);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_wihd_remote_device_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_name *remote_device_name = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Remote Devive Name not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_REMOTE_NAME,
				NULL, 0,
				get_remote_name_resp_q,
				WAIT_TIME_FOR_GET_REMOTE_NAME_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*remote_device_name) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	remote_device_name = resp_data;

done:
	if (NULL != remote_device_name)
		rv = scnprintf(buf, PAGE_SIZE, "%s", remote_device_name->name);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t
get_sii6400_wihd_remote_device_signal_strength(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_signal_strength *remote_device_signal_strength = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Remote Dev Signal Strength not allowed in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_SIGNAL_STRENGTH,
				NULL, 0,
				get_signal_strength_resp_q,
				WAIT_TIME_FOR_GET_SIGNAL_STRENGTH_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*remote_device_signal_strength) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	remote_device_signal_strength = resp_data;

done:
	if (NULL != remote_device_signal_strength) {
		if (remote_device_signal_strength->signal_strength <= 100) {
			rv = scnprintf(buf, PAGE_SIZE, "%u",
				remote_device_signal_strength->signal_strength);
		} else {
			rv = scnprintf(buf, PAGE_SIZE, "%s", "");
		}
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_wihd_remote_device_type(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_dev_type *remote_device_type = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Remote Devive Type not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_REMOTE_TYPE,
				NULL, 0,
				get_remote_type_resp_q,
				WAIT_TIME_FOR_GET_REMOTE_TYPE_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*remote_device_type) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	remote_device_type = resp_data;

done:
	if (NULL != remote_device_type) {
		rv = scnprintf(buf, PAGE_SIZE,
			"{\"video_source\":%s,\"video_sink\":%s,"
			"\"audio_source\":%s,\"audio_sink\":%s}",
			remote_device_type->video_source ? "true" : "false",
			remote_device_type->video_sink ? "true" : "false",
			remote_device_type->audio_source ? "true" : "false",
			remote_device_type->audio_sink ? "true" : "false");
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t
set_sii6400_wihd_remote_device_fw_update_status(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	unsigned long enabled = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_remote_fw_update_status *remote_fw_update_status = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Remote Dev Firmware Update not allowed in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &enabled);
	if (rv) {
		warn("Bad Sii6400 Remote Device Firmware Update request: %s",
		     buf);
		retval = rv;
		goto done;
	}

	if (1 < enabled) {
		warn("Bad Sii6400 Remote Device Firmware Update request: %s",
		     buf);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	remote_fw_update_status = SiiOsCalloc("RemoteDevFwUpdateStatus",
				sizeof(*remote_fw_update_status), 0);
	if (NULL == remote_fw_update_status) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	remote_fw_update_status->remote_fw_update_enabled =
				(0 == enabled) ? false : true;
	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_SET_REMOTE_FW_UPDATE,
				remote_fw_update_status,
				sizeof(*remote_fw_update_status),
				set_remote_fw_update_resp_q,
				WAIT_TIME_FOR_SET_REMOTE_FW_UPDATE_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

done:
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 WiHD Connection Group Attributes.
 */
static struct device_attribute dev_attr_rd_category =
	__ATTR(category, (S_IRUGO),
		get_sii6400_wihd_remote_device_category, NULL);
static struct device_attribute dev_attr_rd_mac_addr =
	__ATTR(mac_addr, (S_IRUGO),
		get_sii6400_wihd_remote_device_mac_addr, NULL);
static struct device_attribute dev_attr_rd_manufacturer =
	__ATTR(manufacturer, (S_IRUGO),
		get_sii6400_wihd_remote_device_manufacturer, NULL);
static struct device_attribute dev_attr_rd_monitor_name =
	__ATTR(monitor_name, (S_IRUGO),
		get_sii6400_wihd_remote_device_monitor_name, NULL);
static struct device_attribute dev_attr_rd_name =
	__ATTR(name, (S_IRUGO),
		get_sii6400_wihd_remote_device_name, NULL);
static struct device_attribute dev_attr_rd_signal_strength =
	__ATTR(signal_strength, (S_IRUGO),
		get_sii6400_wihd_remote_device_signal_strength, NULL);
static struct device_attribute dev_attr_rd_type =
	__ATTR(type, (S_IRUGO),
		get_sii6400_wihd_remote_device_type, NULL);
static struct device_attribute dev_attr_wr_fw_update =
	__ATTR(fw_update, (S_IWUSR|S_IWGRP),
		NULL, set_sii6400_wihd_remote_device_fw_update_status);

static struct attribute *sii6400_wihd_remote_device_attrs[] = {
	&dev_attr_rd_category.attr,
	&dev_attr_rd_mac_addr.attr,
	&dev_attr_rd_manufacturer.attr,
	&dev_attr_rd_monitor_name.attr,
	&dev_attr_rd_name.attr,
	&dev_attr_rd_signal_strength.attr,
	&dev_attr_rd_type.attr,
	&dev_attr_wr_fw_update.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_wihd_remote_device_attr_group = {
	.name = __stringify(remote_device),
	.attrs = sii6400_wihd_remote_device_attrs,
};

/*
 * Sii6400 WiHD Self Group Attributes
 */
static ssize_t get_sii6400_wihd_self_connect_timeout(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_connect_timeout *self_connect_timeout = NULL;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		goto done;
	}

	self_connect_timeout = &devinfo->my_connect_timeout;

done:
	if (NULL != self_connect_timeout) {
		rv = scnprintf(buf, PAGE_SIZE, "%u",
			       self_connect_timeout->connect_timeout);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	return retval ? retval : rv;
}

static ssize_t set_sii6400_wihd_self_connect_timeout(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	unsigned long connect_timeout = SELF_CONNECT_TIMEOUT_DEFAULT;
	struct sii6400_device_info *devinfo = NULL;

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

	rv = strict_strtoul(buf, 0, &connect_timeout);
	if (rv) {
		warn("Invalid Sii6400 WIHD Self Connect Timeout %s", buf);
		retval = rv;
		goto done;
	}

	if (0 == connect_timeout) {
		warn("Invalid Sii6400 WIHD Self Connect Timeout %lu\n",
		     connect_timeout);
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	devinfo->my_connect_timeout.connect_timeout = connect_timeout;

done:
	return retval;
}

static ssize_t get_sii6400_wihd_self_mac_addr(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mac_addr *self_mac_addr = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		err("Get mac address not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_MAC_ADDR,
				NULL, 0,
				get_mac_addr_resp_q,
				WAIT_TIME_FOR_GET_MAC_ADDR_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*self_mac_addr) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	self_mac_addr = resp_data;

done:
	if (NULL != self_mac_addr) {
		rv = scnprintf(buf, PAGE_SIZE, "%02x%02x%02x%02x%02x%02x",
			       self_mac_addr->mac_addr[0],
			       self_mac_addr->mac_addr[1],
			       self_mac_addr->mac_addr[2],
			       self_mac_addr->mac_addr[3],
			       self_mac_addr->mac_addr[4],
			       self_mac_addr->mac_addr[5]);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_wihd_self_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_name *self_name = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		err("Get device name not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_NAME,
				NULL, 0,
				get_name_resp_q,
				WAIT_TIME_FOR_GET_NAME_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*self_name) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	self_name = resp_data;

done:
	if (NULL != self_name)
		rv = scnprintf(buf, PAGE_SIZE, "%.16s", self_name->name);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_wihd_self_name(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	char *sysfs_self_name = NULL;
	size_t sysfs_self_name_len = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_name *self_name = NULL;
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

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (SII6400_MODE_WIHD != devinfo->mode) {
		err("Could not set device name in current mode\n");
		retval = -EINVAL;
		goto done;
	}

	sysfs_self_name_len = strnlen(skip_spaces(buf), count);
	sysfs_self_name = SiiOsCalloc("SysfsSelfName",
					sysfs_self_name_len + 1, 0);
	if (NULL == sysfs_self_name) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	strlcpy(sysfs_self_name, skip_spaces(buf), sysfs_self_name_len + 1);
	strim(sysfs_self_name);
	if (DEVICE_NAME_LEN < strlen(sysfs_self_name))
		sysfs_self_name[DEVICE_NAME_LEN] = '\0';

	/* This memory is freed in the state machine. */
	self_name = SiiOsCalloc("SelfName", sizeof(*self_name), 0);
	if (NULL == self_name) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	memset(self_name->name, 0, sizeof(self_name->name));
	strlcpy(self_name->name, sysfs_self_name,
				sizeof(self_name->name));

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_SET_NAME,
				self_name, sizeof(*self_name),
				set_name_resp_q,
				WAIT_TIME_FOR_SET_NAME_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

done:
	SiiOsFree(sysfs_self_name);
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 WiHD Self Group Attributes.
 */
static struct device_attribute dev_attr_my_connect_timeout =
	__ATTR(connect_timeout, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_wihd_self_connect_timeout,
		set_sii6400_wihd_self_connect_timeout);
static struct device_attribute dev_attr_my_mac_addr =
	__ATTR(mac_addr, (S_IRUGO), get_sii6400_wihd_self_mac_addr, NULL);
static struct device_attribute dev_attr_my_name =
	__ATTR(name, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_wihd_self_name,
		set_sii6400_wihd_self_name);
static struct attribute *sii6400_wihd_self_attrs[] = {
	&dev_attr_my_connect_timeout.attr,
	&dev_attr_my_mac_addr.attr,
	&dev_attr_my_name.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_wihd_self_attr_group = {
	.name = __stringify(self),
	.attrs = sii6400_wihd_self_attrs,
};

/*
 * Sii6400 WiHD WVAN Group Attributes
 */
static ssize_t get_sii6400_wihd_wvan_connection_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_wvan_connection_status *connection_status = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("connection_status not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_GET_CONNECTION_STATUS,
				NULL, 0,
				get_wvan_connection_status_resp_q,
				WAIT_TIME_FOR_GET_CONNECTION_STATUS_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
		(resp_data_size != sizeof(*connection_status))) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}
	connection_status = resp_data;

done:
	if (NULL != connection_status)
		rv = scnprintf(buf, PAGE_SIZE, "%u", connection_status->status);
	else
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_wihd_wvan_device_list(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;
	struct sii6400_remote_device_list *device_list = NULL;
	uint8_t i = 0;
	size_t buf_size = PAGE_SIZE;

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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WiHD WVAN Device List not available in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_GET_DEVICE_LIST,
				NULL, 0,
				get_device_list_resp_q,
				WAIT_TIME_FOR_GET_DEVICE_LIST_RESPONSE,
				&resp_data, &resp_data_size);

	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

	if ((NULL == resp_data) ||
		(resp_data_size != sizeof(*device_list))) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}
	device_list = resp_data;

done:
	if ((NULL != device_list) && (NULL != buf)) {
		rv = scnprintf(buf, buf_size, "[");
		for (i = 0; i < device_list->num_dev; i++) {
			struct sii6400_remote_device *remote_dev =
					&device_list->remote_devices[i];
			if (i)
				rv += scnprintf(buf + rv, buf_size - rv, ",");
			rv += scnprintf(buf + rv, buf_size - rv, "{");
			rv += scnprintf(buf + rv, buf_size - rv,
					"\"mac_addr\":\"");
			rv += scnprintf(buf + rv, buf_size - rv,
					"%02x%02x%02x%02x%02x%02x",
					remote_dev->mac_addr.mac_addr[0],
					remote_dev->mac_addr.mac_addr[1],
					remote_dev->mac_addr.mac_addr[2],
					remote_dev->mac_addr.mac_addr[3],
					remote_dev->mac_addr.mac_addr[4],
					remote_dev->mac_addr.mac_addr[5]);
			rv += scnprintf(buf + rv, buf_size - rv,
					"\",\"name\":\"");
			rv += scnprintf(buf + rv, buf_size - rv,
					"%s", remote_dev->name.name);
			rv += scnprintf(buf + rv, buf_size - rv,
					"\",\"manufacturer\":\"");
			rv += scnprintf(buf + rv, buf_size - rv, "%s",
					remote_dev->manufacturer.manufacturer);
			rv += scnprintf(buf + rv, buf_size - rv,
					"\",\"category\":\"");
			rv += scnprintf(buf + rv, buf_size - rv, "%s",
				get_category_string(&remote_dev->category));
			rv += scnprintf(buf + rv, buf_size - rv,
					"\",\"type\":");
			rv += scnprintf(buf + rv, buf_size - rv,
				"{\"video_source\":%s,\"video_sink\":%s,"
				"\"audio_source\":%s,\"audio_sink\":%s}",
				remote_dev->type.video_source ?
						"true" : "false",
				remote_dev->type.video_sink ? "true" : "false",
				remote_dev->type.audio_source ?
						"true" : "false",
				remote_dev->type.audio_sink ? "true" : "false");
			rv += scnprintf(buf + rv, buf_size - rv, "}");
		}
		rv += scnprintf(buf + rv, buf_size - rv, "]");
	} else {
		rv = scnprintf(buf, buf_size, "%s", "[]");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t get_sii6400_wihd_wvan_join(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_wvan_info *wvan_info = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WVAN Info not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_WVAN_INFO,
				NULL, 0,
				get_wvan_info_resp_q,
				WAIT_TIME_FOR_GET_WVAN_INFO_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*wvan_info) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	wvan_info = resp_data;

done:
	if (NULL != wvan_info) {
		rv = scnprintf(buf, PAGE_SIZE, "[%u,%u,%u]", wvan_info->wvan_id,
			       wvan_info->hr_channel, wvan_info->lr_channel);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t request_sii6400_wihd_wvan_join(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sii6400_device_info *devinfo = NULL;
	ssize_t retval = count;
	uint8_t wvan_id = 0;
	uint8_t hr_channel = 0;
	uint8_t lr_channel = 0;
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

	warn("start!\n");
	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	if (3 != sscanf(buf, " [%hhu,%hhu,%hhu]",
				&wvan_id, &hr_channel, &lr_channel)) {
		warn("Invalid Sii6400 WiHD WVAN join/leave request: %s", buf);
		retval = -EINVAL;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -ENODEV;
		goto done;
	}

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Join/Leave WVAN not possible in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if ((0 == wvan_id) && (0 == hr_channel) && (0 == lr_channel)) {
		sm_rv = sm_send_request(wihd_sm_queue,
					SII_SM_REQ_LEAVE_WVAN,
					NULL, 0,
					leave_wvan_resp_q,
					WAIT_TIME_FOR_LEAVE_WVAN_RESPONSE,
					NULL, NULL);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("WiHD State Machine response time out\n");
			retval = -ETIME;
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("WiHD State Machine request send failure\n");
			retval = -ENODEV;
			goto done;
		}
	} else {
		struct sii6400_wvan_info *wvan_info = NULL;

		if (/*(wvan_id < 0) || (255 < wvan_id) ||*/
		    (hr_channel < 2) || (3 < hr_channel) ||
		    /*(lr_channel < 0) ||*/ (4 < lr_channel)) {
			warn("Invalid Sii6400 WiHD Join WVAN request: %s", buf);
			retval = -EINVAL;
			goto done;
		}

		/* This memory is freed in the state machine. */
		wvan_info = SiiOsCalloc("WVANInfo", sizeof(*wvan_info), 0);
		if (NULL == wvan_info) {
			err("Out of memory\n");
			retval = -ENODEV;
			goto done;
		}

		wvan_info->wvan_id = wvan_id;
		wvan_info->hr_channel = hr_channel;
		wvan_info->lr_channel = lr_channel;

		sm_rv = sm_send_request(wihd_sm_queue,
					SII_SM_REQ_JOIN_WVAN,
					wvan_info, sizeof(*wvan_info),
					join_wvan_resp_q,
					WAIT_TIME_FOR_JOIN_WVAN_RESPONSE,
					NULL, NULL);
		if (SII_OS_STATUS_TIMEOUT == sm_rv) {
			err("WiHD State Machine response time out\n");
			retval = -ETIME;
			goto done;
		} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
			err("WiHD State Machine request send failure\n");
			retval = -ENODEV;
			goto done;
		}
	}

done:
	return retval;
}

static ssize_t get_sii6400_wihd_wvan_scan_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_scan_status *scan_status = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WVAN Scan Status not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_WVAN_SCAN_STATUS,
				NULL, 0,
				get_wvan_scan_status_resp_q,
				WAIT_TIME_FOR_GET_WVAN_SCAN_STATUS_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*scan_status) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	scan_status = resp_data;

done:
	if (NULL != scan_status) {
		rv = scnprintf(buf, PAGE_SIZE, "%s",
			       (scan_status->scan_enabled ? "1" : "0"));
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_wihd_wvan_scan_enable(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	unsigned long scan_enabled = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_scan_status *scan_status = NULL;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;

#ifdef CONFIG_LIMIT_ON_HDMI
	struct msm_hdmi_mhl_ops *hdmi_mhl_ops = NULL;
#endif

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

#ifdef CONFIG_LIMIT_ON_HDMI
	hdmi_mhl_ops = devinfo->hdmi_mhl_ops;
#endif

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WVAN Scan Enable/Disable not possible in current mode\n");
		retval = -ENODEV;
		goto done;
	} else {
#ifdef CONFIG_LIMIT_ON_HDMI
		if (hdmi_mhl_ops) {
			retval = hdmi_mhl_ops->set_upstream_hpd(
					devinfo->hdmi_pdev, 1);
			dbg("%s:hdmi set hpd %s\n", __func__,
					retval ? "failed" : "passed");
		}
#endif
	}


	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &scan_enabled);
	if (rv) {
		warn("Invalid WiHD WVAN Scan Enable/Disable request: %s",
		     buf);
		retval = rv;
		goto done;
	}

	if (1 < scan_enabled) {
		warn("Invalid WiHD WVAN Scan Enable/Disable request: %s",
		     buf);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	scan_status = SiiOsCalloc("ScanStatus", sizeof(*scan_status), 0);
	if (NULL == scan_status) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	warn("start! scan_enabled=%ld\n", scan_enabled);
	scan_status->scan_enabled = (0 == scan_enabled) ? false : true;

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_SCAN_FOR_WVANS, scan_status,
				sizeof(*scan_status),
				scan_for_wvans_resp_q,
				WAIT_TIME_FOR_SCAN_FOR_WVANS_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

done:
	return retval;
}

static ssize_t get_sii6400_wihd_wvan_scan_duration(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_scan_duration *scan_duration = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WVAN Scan Duration not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_WVAN_SCAN_DURATION,
				NULL, 0,
				get_wvan_scan_duration_resp_q,
				WAIT_TIME_FOR_GET_WVAN_SCAN_DURATION_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*scan_duration) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	scan_duration = resp_data;

done:
	if (NULL != scan_duration) {
		rv = scnprintf(buf, PAGE_SIZE, "%u",
			       scan_duration->scan_duration);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_wihd_wvan_scan_duration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	unsigned long duration = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_scan_duration *scan_duration = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Set WVAN Scan Duration not possible in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &duration);
	if (rv) {
		warn("Invalid WVAN Scan Duration %s", buf);
		retval = rv;
		goto done;
	}

	warn("start! duration=%ld\n", duration);
	if (MAX_SCAN_DURATION < duration)
		duration = MAX_SCAN_DURATION;

	/* This memory is freed in the state machine. */
	scan_duration = SiiOsCalloc("ScanDuration", sizeof(*scan_duration), 0);
	if (NULL == scan_duration) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	scan_duration->scan_duration = duration;

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_SET_WVAN_SCAN_DURATION,
				scan_duration, sizeof(*scan_duration),
				set_wvan_scan_duration_resp_q,
				WAIT_TIME_FOR_SET_WVAN_SCAN_DURATION_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

done:
	return retval;
}

static ssize_t get_sii6400_wihd_wvan_scan_interval(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_scan_interval *scan_interval = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("WVAN Scan Interval not available in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_WVAN_SCAN_INTERVAL,
				NULL, 0,
				get_wvan_scan_interval_resp_q,
				WAIT_TIME_FOR_GET_WVAN_SCAN_INTERVAL_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
	    (sizeof(*scan_interval) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	scan_interval = resp_data;

done:
	if (NULL != scan_interval) {
		rv = scnprintf(buf, PAGE_SIZE, "%u",
			       scan_interval->scan_interval);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_wihd_wvan_scan_interval(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	unsigned long interval = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_scan_interval *scan_interval = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Set WVAN Scan Interval not possible in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &interval);
	if (rv) {
		warn("Invalid WVAN Scan Interval %s", buf);
		retval = rv;
		goto done;
	}

	warn("start! interval=%ld\n", interval);
	if (MAX_SCAN_INTERVAL < interval)
		interval = MAX_SCAN_INTERVAL;

	/* This memory is freed in the state machine. */
	scan_interval = SiiOsCalloc("ScanInterval", sizeof(*scan_interval), 0);
	if (NULL == scan_interval) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	scan_interval->scan_interval = interval;

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_SET_WVAN_SCAN_INTERVAL,
				scan_interval, sizeof(*scan_interval),
				set_wvan_scan_interval_resp_q,
				WAIT_TIME_FOR_SET_WVAN_SCAN_INTERVAL_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}

done:
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 WiHD WVAN Group Attributes.
 * These macros create instances of:
 *   dev_attr_connection_status
 *   dev_attr_device_list
 *   dev_attr_join
 *   dev_attr_scan
 *   dev_attr_scan_duration
 *   dev_attr_scan_interval
 */
static DEVICE_ATTR(connection_status, (S_IRUGO),
			get_sii6400_wihd_wvan_connection_status, NULL);
static DEVICE_ATTR(device_list, (S_IRUGO),
			get_sii6400_wihd_wvan_device_list, NULL);
static DEVICE_ATTR(join, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
			get_sii6400_wihd_wvan_join,
			request_sii6400_wihd_wvan_join);
static DEVICE_ATTR(scan, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
			get_sii6400_wihd_wvan_scan_enable,
			set_sii6400_wihd_wvan_scan_enable);
static DEVICE_ATTR(scan_duration, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
			get_sii6400_wihd_wvan_scan_duration,
			set_sii6400_wihd_wvan_scan_duration);
static DEVICE_ATTR(scan_interval, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
			get_sii6400_wihd_wvan_scan_interval,
			set_sii6400_wihd_wvan_scan_interval);

static struct attribute *sii6400_wihd_wvan_attrs[] = {
	&dev_attr_connection_status.attr,
	&dev_attr_device_list.attr,
	&dev_attr_join.attr,
	&dev_attr_scan.attr,
	&dev_attr_scan_duration.attr,
	&dev_attr_scan_interval.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_wihd_wvan_attr_group = {
	.name = __stringify(wvan),
	.attrs = sii6400_wihd_wvan_attrs,
};

static ssize_t set_sii6400_wihd_vendor_msg_recv_filter(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	char vendor_id_str[MAX_VENDOR_ID_STR_LEN + 1] = {0};
	unsigned long vendor_id = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_vendor_id *vendor_msg_filter = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Cannot set vendor msg filter in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	/* Use a max string length that is 1 longer than allowed to check for
	 * illegal (too long) entries */
	if (1 != sscanf(buf, " %" __stringify(MAX_VENDOR_ID_STR_LEN) "s",
			vendor_id_str)) {
		warn("Invalid Sii6400 Vendor Msg Filter ID %s", buf);
		retval = -EINVAL;
		goto done;
	}
	if (vendor_id_str[MAX_VENDOR_ID_STR_LEN - 1]) {
		warn("Invalid Sii6400 Vendor Msg Filter ID %s", buf);
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(vendor_id_str, 16, &vendor_id);
	if (rv) {
		warn("Invalid Sii6400 Vendor Msg Filter ID %s\n",
		     vendor_id_str);
		retval = rv;
		goto done;
	}
	/* This memory is freed in the state machine. */
	vendor_msg_filter = SiiOsCalloc("vendorfilter",
					sizeof(*vendor_msg_filter), 0);
	if (NULL == vendor_msg_filter) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	vendor_msg_filter->vendor_id[0] = (vendor_id >> 16) & 0xff;
	vendor_msg_filter->vendor_id[1] = (vendor_id >> 8) & 0xff;
	vendor_msg_filter->vendor_id[2] = (vendor_id) & 0xff;
	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_SET_VENDOR_MSG_RECV_FILTER,
				vendor_msg_filter, sizeof(*vendor_msg_filter),
				set_vendor_msg_recv_filter_resp_q,
				WAIT_TIME_FOR_SET_VENDOR_MSG_FILTER_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}
done:
	return retval;
}

static ssize_t get_sii6400_wihd_vendor_msg_recv_filter(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_vendor_id *vendor_msg_filter = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Cannot get vendor msg filter in current mode\n");
		goto done;
	}

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_GET_VENDOR_MSG_RECV_FILTER,
				NULL, 0,
				get_vendor_msg_recv_filter_resp_q,
				WAIT_TIME_FOR_GET_VENDOR_MSG_FILTER_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		goto done;
	}

	if ((NULL == resp_data) ||
		(sizeof(*vendor_msg_filter) != resp_data_size)) {
		err("Invalid WiHD State Machine response\n");
		goto done;
	}

	vendor_msg_filter = resp_data;

done:
	if (NULL != vendor_msg_filter) {
		rv = scnprintf(buf, PAGE_SIZE, "%02x%02x%02x",
			       vendor_msg_filter->vendor_id[0],
			       vendor_msg_filter->vendor_id[1],
			       vendor_msg_filter->vendor_id[2]);
	} else {
		rv = scnprintf(buf, PAGE_SIZE, "%s", "");
	}
	SiiOsFree(resp_data);
	return retval ? retval : rv;
}

static ssize_t set_sii6400_wihd_vendor_msg_vendor_id(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	char vendor_id_str[MAX_VENDOR_ID_STR_LEN + 1] = {0};
	unsigned long vendor_id = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_vendor_id *dest_vendor_id = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Cannot set vendor msg vendor ID in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	/* Use a max string length that is 1 longer than allowed to check for
	 * illegal (too long) entries */
	if (1 != sscanf(buf, " %" __stringify(MAX_VENDOR_ID_STR_LEN) "s",
			vendor_id_str)) {
		warn("Invalid Sii6400 Vendor ID %s", buf);
		retval = -EINVAL;
		goto done;
	}
	if (vendor_id_str[MAX_VENDOR_ID_STR_LEN - 1]) {
		warn("Invalid Sii6400 Vendor ID %s", buf);
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(vendor_id_str, 16, &vendor_id);
	if (rv) {
		warn("Invalid Sii6400 Vendor ID %s\n",
		     vendor_id_str);
		retval = rv;
		goto done;
	}
	/* This memory is freed in the state machine. */
	dest_vendor_id = SiiOsCalloc("vendorid", sizeof(*dest_vendor_id), 0);
	if (NULL == dest_vendor_id) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}
	dest_vendor_id->vendor_id[0] = (vendor_id >> 16) & 0xff;
	dest_vendor_id->vendor_id[1] = (vendor_id >> 8) & 0xff;
	dest_vendor_id->vendor_id[2] = (vendor_id) & 0xff;
	sm_rv = sm_send_request(wihd_sm_queue,
			SII_SM_REQ_SET_VENDOR_MSG_VENDOR_ID,
			dest_vendor_id, sizeof(*dest_vendor_id),
			set_vendor_msg_vendor_id_resp_q,
			WAIT_TIME_FOR_SET_VENDOR_MSG_VENDOR_ID_RESPONSE,
			NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}
done:
	return retval;
}

static ssize_t set_sii6400_wihd_vendor_msg_dest_mac_addr(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	int rv = 0;
	char mac_addr_str[MAC_ADDRESS_STR_1_LEN + 1] = {0};
	uint64_t mac_addr = 0;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_mac_addr *dest_mac_addr = NULL;
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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Cannot set vendor msg dest MAC addr in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}

	/* Use a max string length that is 1 longer than allowed to check for
	 * illegal (too long) entries */
	if (1 != sscanf(buf, " %" __stringify(MAC_ADDRESS_STR_1_LEN) "s",
			mac_addr_str)) {
		warn("Invalid Sii6400 Vendor Msg Dest MAC Addr %s", buf);
		retval = -EINVAL;
		goto done;
	}
	if (mac_addr_str[MAC_ADDRESS_STR_1_LEN - 1]) {
		warn("Invalid Sii6400 Vendor Msg Dest MAC Addr %s", buf);
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoull(mac_addr_str, 16, &mac_addr);
	if (rv) {
		warn("Invalid Sii6400 Vendor Msg Dest MAC Addr %s\n",
		     mac_addr_str);
		retval = rv;
		goto done;
	}

	if (0xffffffffffffULL < mac_addr) {
		warn("Invalid Sii6400 Vendor Msg Dest MAC Addr %s\n",
		     mac_addr_str);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	dest_mac_addr = SiiOsCalloc("dest_mac_addr",
					sizeof(*dest_mac_addr), 0);
	if (NULL == dest_mac_addr) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	dest_mac_addr->mac_addr[0] = (mac_addr >> 40) & 0xff;
	dest_mac_addr->mac_addr[1] = (mac_addr >> 32) & 0xff;
	dest_mac_addr->mac_addr[2] = (mac_addr >> 24) & 0xff;
	dest_mac_addr->mac_addr[3] = (mac_addr >> 16) & 0xff;
	dest_mac_addr->mac_addr[4] = (mac_addr >> 8) & 0xff;
	dest_mac_addr->mac_addr[5] = mac_addr & 0xff;
	sm_rv = sm_send_request(wihd_sm_queue,
			SII_SM_REQ_SET_VENDOR_MSG_MAC_ADDR,
			dest_mac_addr, sizeof(*dest_mac_addr),
			set_vendor_msg_mac_addr_resp_q,
			WAIT_TIME_FOR_SET_VENDOR_MSG_MAC_ADDR_RESPONSE,
			NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}
done:
	return retval;
}

static ssize_t set_sii6400_wihd_vendor_msg_send(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t retval = count;
	char vendor_msg_str[MAX_VENDOR_MSG_STR_LEN + 1] = {0};
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_vendor_msg_payload *msg_payload = NULL;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;
	uint32_t i, j;
	unsigned char value;

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

	if (SII6400_MODE_WIHD != devinfo->mode) {
		dbg("Cannot send vendor msg in current mode\n");
		retval = -ENODEV;
		goto done;
	}

	if (!sii6400_char_string_is_valid(buf, count)) {
		err("Invalid input string\n");
		retval = -EINVAL;
		goto done;
	}
	/* Use a max string length that is 1 longer than allowed to check for
	 * illegal (too long) entries */
	if (1 != sscanf(buf, " %" __stringify(MAX_VENDOR_MSG_STR_LEN) "s",
			vendor_msg_str)) {
		warn("Invalid Sii6400 Vendor Msg string %s", buf);
		retval = -EINVAL;
		goto done;
	}
	if (vendor_msg_str[MAX_VENDOR_MSG_STR_LEN - 1]) {
		warn("Invalid Sii6400 Vendor Msg string %s", buf);
		retval = -EINVAL;
		goto done;
	}

	if (0 != strlen(vendor_msg_str) % 2) {
		warn("Invalid Sii6400 Vendor Msg string %s", buf);
		retval = -EINVAL;
		goto done;
	}

	/* This memory is freed in the state machine. */
	msg_payload = SiiOsCalloc("vendormsgpayload", sizeof(*msg_payload), 0);
	if (NULL == msg_payload) {
		err("Out of memory\n");
		retval = -ENODEV;
		goto done;
	}

	for (i = 0, j = 0; i < strlen(vendor_msg_str); i++) {
		if ('0' <= vendor_msg_str[i] && vendor_msg_str[i] <= '9') {
			value = vendor_msg_str[i] - '0';
		} else if ('a' <= vendor_msg_str[i] &&
			    vendor_msg_str[i] <= 'f') {
			value = vendor_msg_str[i] - 'a' + 10;
		} else {
			warn("Invalid Sii6400 Vendor Msg string %s", buf);
			SiiOsFree(msg_payload);
			retval = -EINVAL;
			goto done;
		}
		msg_payload->data[j] += value;
		if (i % 2)
			j++;
		else
			msg_payload->data[j] <<= 4;
	}
	msg_payload->dataLength = j;

	sm_rv = sm_send_request(wihd_sm_queue,
				SII_SM_REQ_SEND_VENDOR_MSG,
				msg_payload, sizeof(*msg_payload),
				send_vendor_msg_resp_q,
				WAIT_TIME_FOR_SEND_VENDOR_MSG_RESPONSE,
				NULL, NULL);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for WiHD State Machine response\n");
		retval = -ETIME;
		goto done;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("WiHD State Machine request send failure\n");
		retval = -ENODEV;
		goto done;
	}
done:
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 WiHD Vendor Msg Group Attributes.
 */

static struct device_attribute dev_attr_recv_filter =
	__ATTR(recv_filter, ((S_IRUGO) | (S_IWUSR|S_IWGRP)),
		get_sii6400_wihd_vendor_msg_recv_filter,
		set_sii6400_wihd_vendor_msg_recv_filter);
static struct device_attribute dev_attr_vendor_id =
	__ATTR(vendor_id, (S_IWUSR|S_IWGRP),
		NULL, set_sii6400_wihd_vendor_msg_vendor_id);
static struct device_attribute dev_attr_dest_mac_addr =
	__ATTR(dest_mac_addr, (S_IWUSR|S_IWGRP),
		NULL, set_sii6400_wihd_vendor_msg_dest_mac_addr);
static struct device_attribute dev_attr_vm_send =
	__ATTR(send, (S_IWUSR|S_IWGRP),
		NULL, set_sii6400_wihd_vendor_msg_send);

static struct attribute *sii6400_wihd_vendor_msg_attrs[] = {
	&dev_attr_dest_mac_addr.attr,
	&dev_attr_recv_filter.attr,
	&dev_attr_vm_send.attr,
	&dev_attr_vendor_id.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_wihd_vendor_msg_attr_group = {
	.name = __stringify(vendor_msg),
	.attrs = sii6400_wihd_vendor_msg_attrs,
};
/* WiHD initialization and release */
int sii6400_wihd_dev_add(struct sii6400_device_info *devinfo)
{
	int retval = 0;
	struct sii6400_wihd_device_info *devinfo_wihd = NULL;

	dbg("");

	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -EFAULT;
		goto failed_nullptr;
	}

	devinfo_wihd = SiiOsCalloc("WIHDDevAdd", sizeof(*devinfo_wihd), 0);
	if (NULL == devinfo_wihd) {
		err("Out of memory\n");
		retval = -ENOMEM;
		goto failed_memalloc;
	}

	devinfo_wihd->devnum = MKDEV(MAJOR(devinfo->devnum), 1);

	devinfo_wihd->cdev = cdev_alloc();
	devinfo_wihd->cdev->owner = THIS_MODULE;
	devinfo_wihd->cdev->ops = &fops_sii6400_wihd;
	retval = cdev_add(devinfo_wihd->cdev, devinfo_wihd->devnum, 1);
	if (retval)
		goto failed_chrwihdadd;

	devinfo->wihd = devinfo_wihd;

	return 0;

failed_chrwihdadd:
	SiiOsFree(devinfo_wihd);
failed_memalloc:
failed_nullptr:
	return retval;
}

int sii6400_wihd_dev_init(struct sii6400_device_info *devinfo)
{
	int retval = 0;

	dbg("");

	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		retval = -EFAULT;
		goto failed_nullptr;
	}

	devinfo->wihd->device = device_create(devinfo->dev_class,
					devinfo->device, devinfo->wihd->devnum,
					NULL, SII6400_WIHD_DEVNAME);
	if (IS_ERR(devinfo->wihd->device)) {
		retval = PTR_ERR(devinfo->wihd->device);
		goto failed_wihdreg;
	}

	retval = sysfs_create_group(&devinfo->wihd->device->kobj,
					&sii6400_wihd_attr_group);
	if (retval < 0)
		warn("failed to create WiHD attribute group\n");

	retval = sysfs_create_group(&devinfo->wihd->device->kobj,
					&sii6400_wihd_cec_attr_group);
	if (retval < 0)
		warn("failed to create WiHD cec attribute group\n");

	retval = sysfs_create_group(&devinfo->wihd->device->kobj,
					&sii6400_wihd_hdcp_attr_group);
	if (retval < 0)
		warn("failed to create WiHD hdcp attribute group\n");

	retval = sysfs_create_group(&devinfo->wihd->device->kobj,
				&sii6400_wihd_remote_control_attr_group);
	if (retval < 0)
		warn("failed to create WiHD remote control attribute group\n");

	retval = sysfs_create_group(&devinfo->wihd->device->kobj,
					&sii6400_wihd_remote_device_attr_group);
	if (retval < 0)
		warn("failed to create WiHD remote device attribute group\n");

	retval = sysfs_create_group(&devinfo->wihd->device->kobj,
					&sii6400_wihd_self_attr_group);
	if (retval < 0)
		warn("failed to create WiHD self attribute group\n");

	retval = sysfs_create_group(&devinfo->wihd->device->kobj,
					&sii6400_wihd_wvan_attr_group);
	if (retval < 0)
		warn("failed to create WiHD WVAN attribute group\n");

	retval = sysfs_create_group(&devinfo->wihd->device->kobj,
					&sii6400_wihd_vendor_msg_attr_group);
	if (retval < 0)
		warn("failed to create WiHD Vendor Msg attribute group\n");

	return 0;

failed_wihdreg:
failed_nullptr:
	return retval;
}

void sii6400_wihd_dev_exit(struct sii6400_device_info *devinfo)
{
	dbg("");

	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		return;
	}

	sysfs_remove_group(&devinfo->wihd->device->kobj,
				&sii6400_wihd_attr_group);
	sysfs_remove_group(&devinfo->wihd->device->kobj,
				&sii6400_wihd_cec_attr_group);
	sysfs_remove_group(&devinfo->wihd->device->kobj,
				&sii6400_wihd_hdcp_attr_group);
	sysfs_remove_group(&devinfo->wihd->device->kobj,
				&sii6400_wihd_remote_control_attr_group);
	sysfs_remove_group(&devinfo->wihd->device->kobj,
				&sii6400_wihd_remote_device_attr_group);
	sysfs_remove_group(&devinfo->wihd->device->kobj,
				&sii6400_wihd_self_attr_group);
	sysfs_remove_group(&devinfo->wihd->device->kobj,
				&sii6400_wihd_wvan_attr_group);
	sysfs_remove_group(&devinfo->wihd->device->kobj,
				&sii6400_wihd_vendor_msg_attr_group);

	device_unregister(devinfo->wihd->device);
	device_destroy(devinfo->dev_class, devinfo->wihd->devnum);
}

void sii6400_wihd_dev_remove(struct sii6400_device_info *devinfo)
{
	dbg("");

	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		return;
	}

	cdev_del(devinfo->wihd->cdev);
	SiiOsFree(devinfo->wihd);
	devinfo->wihd = NULL;
}

