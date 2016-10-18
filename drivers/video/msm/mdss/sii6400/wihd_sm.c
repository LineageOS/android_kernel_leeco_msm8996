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

#include <linux/device.h>

#include "osal.h"
#include "state_machine.h"
#include "wihd_sm.h"
#include "sii6400.h"
#include "host_msg.h"
#include "rcp_inputdev.h"
#include "remote_fw_update.h"

struct SiiOsQueue *get_wihd_chip_version_resp_q;
struct SiiOsQueue *get_wihd_firmware_version_resp_q;
struct SiiOsQueue *get_wihd_id_impedance_resp_q;
struct SiiOsQueue *get_remote_mac_addr_resp_q;
struct SiiOsQueue *get_remote_category_resp_q;
struct SiiOsQueue *get_remote_manufacturer_resp_q;
struct SiiOsQueue *get_remote_monitor_name_resp_q;
struct SiiOsQueue *get_remote_name_resp_q;
struct SiiOsQueue *get_remote_type_resp_q;
struct SiiOsQueue *get_signal_strength_resp_q;
struct SiiOsQueue *get_mac_addr_resp_q;
struct SiiOsQueue *set_name_resp_q;
struct SiiOsQueue *get_name_resp_q;
struct SiiOsQueue *set_wvan_scan_duration_resp_q;
struct SiiOsQueue *get_wvan_scan_duration_resp_q;
struct SiiOsQueue *set_wvan_scan_interval_resp_q;
struct SiiOsQueue *get_wvan_scan_interval_resp_q;
struct SiiOsQueue *scan_for_wvans_resp_q;
struct SiiOsQueue *get_wvan_scan_status_resp_q;
struct SiiOsQueue *join_wvan_resp_q;
struct SiiOsQueue *leave_wvan_resp_q;
struct SiiOsQueue *get_wvan_info_resp_q;
struct SiiOsQueue *get_wvan_connection_status_resp_q;
struct SiiOsQueue *get_device_list_resp_q;
struct SiiOsQueue *wihd_connect_resp_q;
struct SiiOsQueue *wihd_disconnect_resp_q;
struct SiiOsQueue *cec_send_message_resp_q;
struct SiiOsQueue *get_wihd_rc_rcvd_ctrlcode_resp_q;
struct SiiOsQueue *get_wihd_rc_sent_ctrlcode_resp_q;
struct SiiOsQueue *send_wihd_rc_ctrlcode_resp_q;
struct SiiOsQueue *set_hdcp_policy_resp_q;
struct SiiOsQueue *get_hdcp_policy_resp_q;
struct SiiOsQueue *set_hdcp_stream_type_resp_q;
struct SiiOsQueue *get_hdcp_stream_type_resp_q;
struct SiiOsQueue *set_remote_fw_update_resp_q;
struct SiiOsQueue *get_vendor_msg_recv_filter_resp_q;
struct SiiOsQueue *set_vendor_msg_recv_filter_resp_q;
struct SiiOsQueue *set_vendor_msg_vendor_id_resp_q;
struct SiiOsQueue *set_vendor_msg_mac_addr_resp_q;
struct SiiOsQueue *send_vendor_msg_resp_q;

static struct SiiOsTimer *wvan_scan_duration_timer;
static struct SiiOsTimer *wvan_scan_interval_timer;
static struct SiiOsTimer *wvan_scan_response_timer;

static struct sii6400_wihd_sm_info wihd_sm_info;

static bool wihd_sm_is_enabled;

static struct sii6400_mac_addr vendor_msg_dest_addr;
static struct sii6400_vendor_id vendor_msg_vendor_id;

static void disable_scanning(struct sii6400_device_info *devinfo);
static void enable_scanning(struct sii6400_device_info *devinfo);
static void restart_scanning(struct sii6400_device_info *devinfo);
static void wvan_scan_duration_timer_function(void *data);
static void wvan_scan_duration_work_function(struct work_struct *work);
static void wvan_scan_interval_timer_function(void *data);
static void wvan_scan_interval_work_function(struct work_struct *work);
static void wvan_scan_response_timer_function(void *data);
static void wvan_scan_response_work_function(struct work_struct *work);
static void associate_to_wvan(struct sii6400_device_info *devinfo);
static enum sii_os_status receive_remote_device_info(
			struct sii6400_remote_device *remote_device_info);
static enum sii_os_status send_my_hdcp_policy(
			struct sii6400_hdcp_policy *hdcp_policy);
static enum sii_os_status send_my_hdcp_stream_type(
			struct sii6400_hdcp_stream_type *hdcp_stream_type);
static const char *get_wihd_uevent_reason_string(
			enum SiI_Notification_Reason reason);

static DECLARE_WORK(wvan_scan_duration_work, wvan_scan_duration_work_function);
static DECLARE_WORK(wvan_scan_interval_work, wvan_scan_interval_work_function);
static DECLARE_WORK(wvan_scan_response_work, wvan_scan_response_work_function);


bool wihd_is_idle(enum sii6400_wihd_state state)
{
	return (WIHD_STATE_IDLE == state) ? true : false;
}

bool wihd_is_scanning(enum sii6400_wihd_state state)
{
	return (WIHD_STATE_WVAN_SCAN == state) ? true : false;
}

bool wihd_is_associating(enum sii6400_wihd_state state)
{
	return (WIHD_STATE_ASSOCIATING == state) ? true : false;
}

bool wihd_is_associated(enum sii6400_wihd_state state)
{
	return (WIHD_STATE_ASSOCIATED == state) ? true : false;
}

bool wihd_is_connecting(enum sii6400_wihd_state state)
{
	return (WIHD_STATE_CONNECTING == state) ? true : false;
}

bool wihd_is_connected(enum sii6400_wihd_state state)
{
	return (WIHD_STATE_CONNECTED == state) ? true : false;
}

enum sii_os_status get_wihd_chip_version(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_chip_version *chip_version = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_GET_CHIP_VERSION, 0, NULL,
					HM_GET_CHIP_VERSION_RESPONSE_TIMEOUT,
					&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get Chip Version command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get Chip Version successful\n");

	if ((resp_data_size < HM_GET_CHIP_VERSION_CMD_RSP_MLEN_MIN) ||
	    (HM_GET_CHIP_VERSION_CMD_RSP_MLEN_MAX < resp_data_size)) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	chip_version = SiiOsCalloc("GetChipVersion", sizeof(*chip_version), 0);
	if (NULL == chip_version) {
		err("Out of memory\n");
		goto send_response;
	}

	chip_version->length = resp_data_size -
					HM_GET_CHIP_VERSION_CMD_RSP_MLEN_MIN;

	/* This memory is freed by the response handler */
	chip_version->chip_version = SiiOsCalloc("GetChipVersionData",
						chip_version->length + 1, 0);
	if (NULL == chip_version->chip_version) {
		err("Out of memory\n");
		goto send_response;
	}

	if (chip_version->length) {
		memcpy(chip_version->chip_version,
			(void *)FIELD_UA_BYTE_ADDRESS(resp_data,
					HM_GET_CHIP_VERSION_CMD_RSP_STRING0),
			chip_version->length);
	}

send_response:
	resp.request_type = SII_SM_REQ_GET_CHIP_VERSION;
	resp.resp_status = resp_status;
	resp.data = chip_version;
	resp.data_size = (NULL != chip_version) ? sizeof(*chip_version) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		if (NULL != chip_version)
			SiiOsFree(chip_version->chip_version);
		SiiOsFree(chip_version);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status get_wihd_firmware_version(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_firmware_version *firmware_version = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_GET_FW_VERSION, 0, NULL,
					HM_GET_FW_VERSION_RESPONSE_TIMEOUT,
					&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get Firmware Version command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get Firmware Version successful\n");

	if ((resp_data_size < HM_GET_FW_VERSION_CMD_RSP_MLEN_MIN) ||
	    (HM_GET_FW_VERSION_CMD_RSP_MLEN_MAX < resp_data_size)) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	firmware_version = SiiOsCalloc("GetFWVersion",
					sizeof(*firmware_version), 0);
	if (NULL == firmware_version) {
		err("Out of memory\n");
		goto send_response;
	}

	firmware_version->length = resp_data_size -
					HM_GET_FW_VERSION_CMD_RSP_MLEN_MIN;

	/* This memory is freed by the response handler */
	firmware_version->fw_version = SiiOsCalloc("GetFWVersionData",
					firmware_version->length + 1, 0);
	if (NULL == firmware_version->fw_version) {
		err("Out of memory\n");
		goto send_response;
	}

	if (firmware_version->length) {
		memcpy(firmware_version->fw_version,
			(void *)FIELD_UA_BYTE_ADDRESS(resp_data,
						HM_GET_FW_VERSION_STRING_0),
			firmware_version->length);
	}

send_response:
	resp.request_type = SII_SM_REQ_GET_FIRMWARE_VERSION;
	resp.resp_status = resp_status;
	resp.data = firmware_version;
	resp.data_size = (NULL != firmware_version) ?
				sizeof(*firmware_version) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		if (NULL != firmware_version)
			SiiOsFree(firmware_version->fw_version);
		SiiOsFree(firmware_version);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status get_wihd_id_impedance(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_id_impedance *id_impedance = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_MHL_MEASURE_ID_IMPEDANCE,
				0, NULL,
				HM_MHL_MEASURE_ID_IMPEDANCE_RESPONSE_TIMEOUT,
				&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get WiHD ID Impedance command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get WiHD ID Impedance successful\n");

	if (HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	id_impedance = SiiOsCalloc("GetWiHDIDImpedance",
					sizeof(*id_impedance), 0);
	if (NULL == id_impedance) {
		err("Out of memory\n");
		goto send_response;
	}

	id_impedance->ohms = (uint32_t)field_ua_extract(resp_data,
				HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP_IMPEDANCE);

send_response:
	resp.request_type = SII_SM_REQ_GET_WIHD_ID_IMPEDANCE;
	resp.resp_status = resp_status;
	resp.data = id_impedance;
	resp.data_size = (NULL != id_impedance) ? sizeof(*id_impedance) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(id_impedance);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status get_remote_mac_addr(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mac_addr *mac_addr = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("Remote Device MAC Address not available in state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	mac_addr = SiiOsCalloc("GetRemoteMacAddr", sizeof(*mac_addr), 0);
	if (NULL == mac_addr) {
		err("Out of memory\n");
		goto send_response;
	}

	memcpy(mac_addr->mac_addr,
		wihd_sm_info.remote_device.mac_addr.mac_addr,
		sizeof(mac_addr->mac_addr));

send_response:
	resp.request_type = SII_SM_REQ_GET_REMOTE_MAC_ADDR;
	resp.resp_status = resp_status;
	resp.data = mac_addr;
	resp.data_size = (NULL != mac_addr) ? sizeof(*mac_addr) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(mac_addr);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_remote_category(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_dev_category *category = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("Remote Device Category not available in state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	category = SiiOsCalloc("GetRemoteCat", sizeof(*category), 0);
	if (NULL == category) {
		err("Out of memory\n");
		goto send_response;
	}

	category->category = wihd_sm_info.remote_device.category.category;

send_response:
	resp.request_type = SII_SM_REQ_GET_REMOTE_CATEGORY;
	resp.resp_status = resp_status;
	resp.data = category;
	resp.data_size = (NULL != category) ? sizeof(*category) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(category);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_remote_manufacturer(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_manufacturer *manufacturer = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("Remote Device Manufacturer not available in state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	manufacturer = SiiOsCalloc("GetRemoteManu", sizeof(*manufacturer), 0);
	if (NULL == manufacturer) {
		err("Out of memory\n");
		goto send_response;
	}

	memcpy(manufacturer->manufacturer,
		wihd_sm_info.remote_device.manufacturer.manufacturer,
		sizeof(manufacturer->manufacturer) - 1);

send_response:
	resp.request_type = SII_SM_REQ_GET_REMOTE_MANUFACTURER;
	resp.resp_status = resp_status;
	resp.data = manufacturer;
	resp.data_size = (NULL != manufacturer) ? sizeof(*manufacturer) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(manufacturer);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_remote_monitor_name(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_monitor_name *monitor_name = NULL;
	struct sii6400_device_info *devinfo = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("Remote Device Monitor Name not available in state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	retval = send_hostmsg_command((uint16_t)HM_REMOTE_DEV_GET_MONITOR_NAME,
				0, NULL,
				HM_REMOTE_DEV_GET_MONITOR_NAME_RESPONSE_TIMEOUT,
				&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get Remote Monitor Name command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get Remote Monitor Name successful\n");

	if (HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	monitor_name = SiiOsCalloc("GetRemoteMonName",
					sizeof(*monitor_name), 0);
	if (NULL == monitor_name) {
		err("Out of memory\n");
		goto send_response;
	}

	memcpy(monitor_name->monitor_name,
		(void *)FIELD_UA_BYTE_ADDRESS(resp_data,
				HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_0),
		sizeof(monitor_name->monitor_name) - 1);

send_response:
	resp.request_type = SII_SM_REQ_GET_REMOTE_MONITOR_NAME;
	resp.resp_status = resp_status;
	resp.data = monitor_name;
	resp.data_size = (NULL != monitor_name) ? sizeof(*monitor_name) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(monitor_name);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status get_remote_name(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_name *name = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("Remote Device Name not available in current state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	name = SiiOsCalloc("GetRemoteName", sizeof(*name), 0);
	if (NULL == name) {
		err("Out of memory\n");
		goto send_response;
	}

	memcpy(name->name, wihd_sm_info.remote_device.name.name,
		sizeof(name->name) - 1);

send_response:
	resp.request_type = SII_SM_REQ_GET_REMOTE_NAME;
	resp.resp_status = resp_status;
	resp.data = name;
	resp.data_size = (NULL != name) ? sizeof(*name) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(name);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_remote_type(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_dev_type *type = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("Remote Device Type not available in current state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	type = SiiOsCalloc("GetRemoteType", sizeof(*type), 0);
	if (NULL == type) {
		err("Out of memory\n");
		goto send_response;
	}

	type->video_source = wihd_sm_info.remote_device.type.video_source;
	type->video_sink = wihd_sm_info.remote_device.type.video_sink;
	type->audio_source = wihd_sm_info.remote_device.type.audio_source;
	type->audio_sink = wihd_sm_info.remote_device.type.audio_sink;

send_response:
	resp.request_type = SII_SM_REQ_GET_REMOTE_TYPE;
	resp.resp_status = resp_status;
	resp.data = type;
	resp.data_size = (NULL != type) ? sizeof(*type) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(type);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_signal_strength(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_signal_strength *signal_strength = NULL;
	struct sii6400_device_info *devinfo = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("Signal Strength not available in current state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	retval = send_hostmsg_command(
			(uint16_t)HM_REMOTE_DEV_GET_SIGNAL_STRENGTH,
			0, NULL,
			HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_RESPONSE_TIMEOUT,
			&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get Signal Strength command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get Signal Strength successful\n");

	if (HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	signal_strength = SiiOsCalloc("GetRemoteSigStr",
					sizeof(*signal_strength), 0);
	if (NULL == signal_strength) {
		err("Out of memory\n");
		goto send_response;
	}

	signal_strength->signal_strength = (uint8_t)field_ua_extract(resp_data,
			HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD_RSP_STRENGTH);
	if (100 < signal_strength->signal_strength) {
		dbg("Illegal Signal Strength (%u) received from device\n",
		    signal_strength->signal_strength);
		signal_strength->signal_strength = 0;
	}

send_response:
	resp.request_type = SII_SM_REQ_GET_SIGNAL_STRENGTH;
	resp.resp_status = resp_status;
	resp.data = signal_strength;
	resp.data_size = (NULL != signal_strength) ?
				sizeof(*signal_strength) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(signal_strength);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status get_mac_addr(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mac_addr *mac_addr = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_LOCAL_DEV_GET_MAC_ADDR,
				0, NULL,
				HM_LOCAL_DEV_GET_MAC_ADDR_RESPONSE_TIMEOUT,
				&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get MAC Address command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get MAC Address successful\n");

	if (HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	mac_addr = SiiOsCalloc("GetMacAddr", sizeof(*mac_addr), 0);
	if (NULL == mac_addr) {
		err("Out of memory\n");
		goto send_response;
	}

	memcpy(mac_addr->mac_addr,
		(void *)FIELD_UA_BYTE_ADDRESS(resp_data,
				HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP_MAC_ADDR0),
		sizeof(mac_addr->mac_addr));

send_response:
	resp.request_type = SII_SM_REQ_GET_MAC_ADDR;
	resp.resp_status = resp_status;
	resp.data = mac_addr;
	resp.data_size = (NULL != mac_addr) ? sizeof(*mac_addr) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(mac_addr);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status set_name(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_name *name = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t name_data_size = 0;
	uint8_t *name_data = NULL;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	name = req_data;

	if (sizeof(*name) != req_data_size) {
		err("Illegal name for Set Name request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	name_data_size = strlen(name->name);
	if ((name_data_size < HM_LOCAL_DEV_SET_NAME_CMD_MLEN_MIN) ||
	    (HM_LOCAL_DEV_SET_NAME_CMD_MLEN_MAX < name_data_size)) {
		err("Illegal name for Set Name request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	name_data = SiiOsCalloc("SetName", name_data_size, 0);
	if (NULL == name_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	memcpy(name_data, name->name, name_data_size);

	retval = send_hostmsg_command((uint16_t)HM_LOCAL_DEV_SET_NAME,
					name_data_size, name_data,
					HM_LOCAL_DEV_SET_NAME_RESPONSE_TIMEOUT,
					NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set Name command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Set Name successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SET_NAME;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(name_data);
	return retval;
}

enum sii_os_status get_name(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_name *name = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_LOCAL_DEV_GET_NAME, 0, NULL,
					HM_LOCAL_DEV_GET_NAME_RESPONSE_TIMEOUT,
					&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get Name command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get Name successful\n");

	if (HM_LOCAL_DEV_GET_NAME_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	name = SiiOsCalloc("GetName", sizeof(*name), 0);
	if (NULL == name) {
		err("Out of memory\n");
		goto send_response;
	}

	memcpy(name->name,
		(void *)FIELD_UA_BYTE_ADDRESS(resp_data,
					HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_0),
		sizeof(name->name) - 1);

send_response:
	resp.request_type = SII_SM_REQ_GET_NAME;
	resp.resp_status = resp_status;
	resp.data = name;
	resp.data_size = (NULL != name) ? sizeof(*name) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(name);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status set_wvan_scan_duration(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_scan_duration *scan_duration = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	scan_duration = req_data;

	if (sizeof(*scan_duration) != req_data_size) {
		err("Illegal Set Scan Duration request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	if (wihd_sm_info.wvan_scan_enabled &&
	    !wihd_sm_info.disable_scanning_pending) {
		dbg("Set Scan Duration not allowed while scanning enabled\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	if ((0 != scan_duration->scan_duration)) {
		if (NULL == wvan_scan_duration_timer) {
			retval = SiiOsTimerCreate("wvan_scan_duration_timer",
					wvan_scan_duration_timer_function,
					NULL, false, 0, false,
					&wvan_scan_duration_timer);
			if (SII_OS_STATUS_SUCCESS != retval) {
				err("wvan_scan_duration_timer create failed\n");
				resp_status = SII_OS_STATUS_ERR_FAILED;
				goto send_response;
			}
		}
	} else {
		if (NULL != wvan_scan_duration_timer) {
			retval = SiiOsTimerDelete(wvan_scan_duration_timer);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Could not delete Scan Duration timer\n");
			else
				wvan_scan_duration_timer = NULL;
		}
	}

	wihd_sm_info.wvan_scan_duration = scan_duration->scan_duration;

send_response:
	resp.request_type = SII_SM_REQ_SET_WVAN_SCAN_DURATION;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_wvan_scan_duration(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_scan_duration *scan_duration = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* This memory is freed by the response handler */
	scan_duration = SiiOsCalloc("GetScanDuration",
					sizeof(*scan_duration), 0);
	if (NULL == scan_duration) {
		err("Out of memory\n");
		goto send_response;
	}

	scan_duration->scan_duration = wihd_sm_info.wvan_scan_duration;

send_response:
	resp.request_type = SII_SM_REQ_GET_WVAN_SCAN_DURATION;
	resp.resp_status = resp_status;
	resp.data = scan_duration;
	resp.data_size = (NULL != scan_duration) ? sizeof(*scan_duration) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(scan_duration);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status set_wvan_scan_interval(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_scan_interval *scan_interval = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	scan_interval = req_data;

	if (sizeof(*scan_interval) != req_data_size) {
		err("Illegal Set Scan Interval request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	if (wihd_sm_info.wvan_scan_enabled &&
	    !wihd_sm_info.disable_scanning_pending) {
		dbg("Set Scan Interval not allowed while scanning enabled\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	wihd_sm_info.wvan_scan_interval = scan_interval->scan_interval;

send_response:
	resp.request_type = SII_SM_REQ_SET_WVAN_SCAN_INTERVAL;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_wvan_scan_interval(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_scan_interval *scan_interval = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* This memory is freed by the response handler */
	scan_interval = SiiOsCalloc("GetScanInterval",
					sizeof(*scan_interval), 0);
	if (NULL == scan_interval) {
		err("Out of memory\n");
		goto send_response;
	}

	scan_interval->scan_interval = wihd_sm_info.wvan_scan_interval;

send_response:
	resp.request_type = SII_SM_REQ_GET_WVAN_SCAN_INTERVAL;
	resp.resp_status = resp_status;
	resp.data = scan_interval;
	resp.data_size = (NULL != scan_interval) ? sizeof(*scan_interval) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(scan_interval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status scan_for_wvans(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_scan_status *scan_status = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	scan_status = req_data;

	if (sizeof(*scan_status) != req_data_size) {
		err("Illegal Scan Status for Scan for WVANs request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_idle(devinfo->wihd->state) &&
	    !wihd_is_scanning(devinfo->wihd->state)) {
		dbg("Enable/Disable WiHD Scan not allowed in state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	if (wihd_sm_info.wvan_scan_enabled != scan_status->scan_enabled) {
		if (!scan_status->scan_enabled)
			disable_scanning(devinfo);
		else
			enable_scanning(devinfo);
	}

send_response:
	resp.request_type = SII_SM_REQ_SCAN_FOR_WVANS;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_wvan_scan_status(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_scan_status *scan_status = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* This memory is freed by the response handler */
	scan_status = SiiOsCalloc("GetScanStatus", sizeof(*scan_status), 0);
	if (NULL == scan_status) {
		err("Out of memory\n");
		goto send_response;
	}

	scan_status->scan_enabled = wihd_sm_info.wvan_scan_enabled;

send_response:
	resp.request_type = SII_SM_REQ_GET_WVAN_SCAN_STATUS;
	resp.resp_status = resp_status;
	resp.data = scan_status;
	resp.data_size = (NULL != scan_status) ? sizeof(*scan_status) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(scan_status);
		goto done;
	}

done:
	return retval;
}

static void disable_scanning(struct sii6400_device_info *devinfo)
{
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	sii_os_rv = SiiOsSemaphoreTake(wihd_sm_info.scan_sem,
					MAX_WAIT_FOR_SCAN_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not take scan semaphore: %x\n", sii_os_rv);
		goto done_no_release;
	}

	wihd_sm_info.restart_scanning_pending = false;

	/* Stop scan duration timer */
	if (NULL != wvan_scan_duration_timer) {
		sii_os_rv = SiiOsTimerDelete(wvan_scan_duration_timer);
		if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
			err("Could not stop Scan Duration timer: %x\n",
			    sii_os_rv);
		} else {
			wvan_scan_duration_timer = NULL;
			if ((0 != wihd_sm_info.wvan_scan_duration) &&
			    sii6400_wihd_sm_is_enabled()) {
				/* Recreate the scan duration timer */
				sii_os_rv = SiiOsTimerCreate(
					"wvan_scan_duration_timer",
					wvan_scan_duration_timer_function,
					NULL, false, 0, false,
					&wvan_scan_duration_timer);
				if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
					err("Unable to create timer: %x\n",
						sii_os_rv);
					wvan_scan_duration_timer = NULL;
				}
			}
		}
	}

	/* Stop scan interval timer */
	if (NULL != wvan_scan_interval_timer) {
		/* Delete the scan interval timer */
		sii_os_rv = SiiOsTimerDelete(wvan_scan_interval_timer);
		if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
			err("Could not stop Scan Interval timer: %x\n",
			    sii_os_rv);
		} else {
			wvan_scan_interval_timer = NULL;

			if (sii6400_wihd_sm_is_enabled()) {
				/* Recreate the scan interval timer */
				sii_os_rv = SiiOsTimerCreate(
					"wvan_scan_interval_timer",
					wvan_scan_interval_timer_function,
					NULL, false, 0, false,
					&wvan_scan_interval_timer);
				if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
					err("Unable to create timer: %x\n",
						sii_os_rv);
					wvan_scan_interval_timer = NULL;
				}
			}
		}
	}

	if ((NULL != devinfo) && (NULL != devinfo->wihd)) {
		if (wihd_is_scanning(devinfo->wihd->state)) {
			/* Disable scanning when current scan completes */
			wihd_sm_info.disable_scanning_pending = true;
			goto done;
		}

		wihd_sm_info.disable_scanning_pending = false;

		if (wihd_sm_info.wvan_scan_enabled) {
			wihd_sm_info.wvan_scan_enabled = false;

			/* 'Scan Stopped' uevent not sent for one-shot scans */
			if ((0 != wihd_sm_info.wvan_scan_interval) ||
			    (0 != wihd_sm_info.wvan_scan_duration)) {
				(void)send_sii6400_uevent(devinfo->wihd->device,
					WIHD_EVENT, WIHD_SCAN_STOPPED_EVENT,
					NULL);
			}
		}
	}

done:
	sii_os_rv = SiiOsSemaphoreGive(wihd_sm_info.scan_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv)
		err("Could not release scan semaphore: %x\n", sii_os_rv);
done_no_release:
	return;
}

static void enable_scanning(struct sii6400_device_info *devinfo)
{
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	sii_os_rv = SiiOsSemaphoreTake(wihd_sm_info.scan_sem,
					MAX_WAIT_FOR_SCAN_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not take scan semaphore: %x\n", sii_os_rv);
		goto done_no_release;
	}

	wihd_sm_info.disable_scanning_pending = false;
	wihd_sm_info.restart_scanning_pending = false;

	if ((NULL == devinfo) || (NULL == devinfo->wihd))
		goto done;

	if (!wihd_is_idle(devinfo->wihd->state) ||
	    !sii6400_wihd_sm_is_enabled()) {
		/* Nothing to be done unless in idle state */
		goto done;
	}

	wihd_sm_info.wvan_scan_enabled = true;

	if ((0 == wihd_sm_info.wvan_scan_interval) &&
	    (0 == wihd_sm_info.wvan_scan_duration)) {
		/* One-shot scan disables scanning when it completes */
		wihd_sm_info.disable_scanning_pending = true;
	}

	/* Start scanning */
	sii_os_rv = HostMsgSendCommandAsync((uint16_t)HM_WVAN_SCAN, 0, NULL);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Scan For WVANs command not sent: %x\n", sii_os_rv);
		/* don't exit without starting timers */
	} else {
		/* Enter scanning state if scan cmd sent OK */
		devinfo->wihd->state = WIHD_STATE_WVAN_SCAN;
		sysfs_notify(&devinfo->wihd->device->kobj,
				NULL, __stringify(state));
	}

	(void)send_sii6400_uevent(devinfo->wihd->device, WIHD_EVENT,
					WIHD_SCAN_STARTED_EVENT, NULL);

	/* Start the scan response timer */
	if (NULL != wvan_scan_response_timer) {
		sii_os_rv = SiiOsTimerSchedule(wvan_scan_response_timer,
						MAX_WAIT_FOR_SCAN_RESPONSE);
		if (SII_OS_STATUS_SUCCESS != sii_os_rv)
			err("Scan response timer not started: %x\n", sii_os_rv);
	}

	/* Start the scan duration timer */
	if (0 != wihd_sm_info.wvan_scan_duration) {
		if (NULL != wvan_scan_duration_timer) {
			/* If periodic scan, start the scan duration timer */
			sii_os_rv = SiiOsTimerSchedule(wvan_scan_duration_timer,
					wihd_sm_info.wvan_scan_duration * 1000);
			if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
				err("Scan duration timer not started: %x\n",
				    sii_os_rv);
			}
		}
	}

	/* Start the scan interval timer */
	if ((0 != wihd_sm_info.wvan_scan_duration) ||
	    (0 != wihd_sm_info.wvan_scan_interval)) {
		/* If the scan interval is 0, then the timer will expire
		 * in the next tick. */
		if (NULL != wvan_scan_interval_timer) {
			sii_os_rv = SiiOsTimerSchedule(wvan_scan_interval_timer,
				wihd_sm_info.wvan_scan_interval ?
				wihd_sm_info.wvan_scan_interval * 1000 : 100);
			if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
				err("Scan interval timer not started: %x\n",
				    sii_os_rv);
			}
		}
	}

done:
	sii_os_rv = SiiOsSemaphoreGive(wihd_sm_info.scan_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv)
		err("Could not release scan semaphore: %x\n", sii_os_rv);
done_no_release:
	return;
}

static void restart_scanning(struct sii6400_device_info *devinfo)
{
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	sii_os_rv = SiiOsSemaphoreTake(wihd_sm_info.scan_sem,
					MAX_WAIT_FOR_SCAN_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not take scan semaphore: %x\n", sii_os_rv);
		goto done_no_release;
	}

	if (!sii6400_wihd_sm_is_enabled())
		wihd_sm_info.wvan_scan_enabled = false;

	if (!wihd_sm_info.wvan_scan_enabled ||
	    wihd_sm_info.disable_scanning_pending) {
		wihd_sm_info.restart_scanning_pending = false;
		goto done;
	}

	if ((NULL == devinfo) || (NULL == devinfo->wihd))
		goto done;

	if (wihd_is_scanning(devinfo->wihd->state)) {
		wihd_sm_info.restart_scanning_pending = true;
		goto done;
	}

	if (!wihd_is_idle(devinfo->wihd->state)) {
		/* Nothing to be done unless in idle state */
		goto done;
	}

	wihd_sm_info.restart_scanning_pending = false;

	/* Restart scanning */
	sii_os_rv = HostMsgSendCommandAsync((uint16_t)HM_WVAN_SCAN, 0, NULL);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Scan For WVANs command not sent: %x\n", sii_os_rv);
		/* don't exit without restarting timers */
	} else {
		/* Enter scanning state if scan cmd sent OK */
		devinfo->wihd->state = WIHD_STATE_WVAN_SCAN;
		sysfs_notify(&devinfo->wihd->device->kobj,
				NULL, __stringify(state));
	}

	/* Start the scan response timer */
	if (NULL != wvan_scan_response_timer) {
		sii_os_rv = SiiOsTimerSchedule(wvan_scan_response_timer,
						MAX_WAIT_FOR_SCAN_RESPONSE);
		if (SII_OS_STATUS_SUCCESS != sii_os_rv)
			err("Scan response timer not started: %x\n", sii_os_rv);
	}

	/* Start the scan interval timer */
	/* If the scan interval is 0, then the timer will expire
	 * in the next tick. */
	if (NULL != wvan_scan_interval_timer) {
		sii_os_rv = SiiOsTimerSchedule(wvan_scan_interval_timer,
				wihd_sm_info.wvan_scan_interval ?
				wihd_sm_info.wvan_scan_interval * 1000 : 100);
		if (SII_OS_STATUS_SUCCESS != sii_os_rv)
			err("Scan interval timer not started: %x\n", sii_os_rv);
	}

done:
	sii_os_rv = SiiOsSemaphoreGive(wihd_sm_info.scan_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv)
		err("Could not release scan semaphore: %x\n", sii_os_rv);
done_no_release:
	return;
}

static void wvan_scan_duration_timer_function(void *data)
{
	schedule_work(&wvan_scan_duration_work);
}

static void wvan_scan_duration_work_function(struct work_struct *work)
{
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if (!sii6400_wihd_sm_is_enabled())
		wihd_sm_info.wvan_scan_enabled = false;

	devinfo = get_sii6400_devinfo();
	if (NULL != devinfo)
		disable_scanning(devinfo);
}

static void wvan_scan_interval_timer_function(void *data)
{
	schedule_work(&wvan_scan_interval_work);
}

static void wvan_scan_interval_work_function(struct work_struct *work)
{
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if (!sii6400_wihd_sm_is_enabled())
		wihd_sm_info.wvan_scan_enabled = false;

	devinfo = get_sii6400_devinfo();
	if (NULL != devinfo)
		restart_scanning(devinfo);
}

static void wvan_scan_response_timer_function(void *data)
{
	schedule_work(&wvan_scan_response_work);
}

static void wvan_scan_response_work_function(struct work_struct *work)
{
	struct sii6400_device_info *devinfo = NULL;

	dbg("timed out waiting for scan results\n");

	if (!sii6400_wihd_sm_is_enabled())
		wihd_sm_info.wvan_scan_enabled = false;

	devinfo = get_sii6400_devinfo();
	if (NULL != devinfo) {
		devinfo->wihd->state = WIHD_STATE_IDLE;
		sysfs_notify(&devinfo->wihd->device->kobj,
				NULL, __stringify(state));

		if (wihd_sm_info.disable_scanning_pending)
			disable_scanning(devinfo);

		if (wihd_sm_info.restart_scanning_pending)
			restart_scanning(devinfo);

		if (wihd_sm_info.associate_pending)
			associate_to_wvan(devinfo);
	}
}

static void associate_to_wvan(struct sii6400_device_info *devinfo)
{
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;
	uint8_t *wvan_info_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if ((NULL == devinfo) || (NULL == devinfo->wihd))
		goto done;

	sii_os_rv = SiiOsSemaphoreTake(wihd_sm_info.scan_sem,
					MAX_WAIT_FOR_SCAN_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS == sii_os_rv) {
		if (sii6400_wihd_sm_is_enabled() &&
		    !wihd_is_scanning(devinfo->wihd->state)) {
			wihd_sm_info.associate_pending = false;
		} else {
			/* Join WVAN when current scan completes */
			wihd_sm_info.associate_pending = true;
		}
		sii_os_rv = SiiOsSemaphoreGive(wihd_sm_info.scan_sem);
		if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
			err("Could not release scan semaphore: %x\n",
				sii_os_rv);
		}
	} else {
		err("Could not take scan semaphore: %x\n", sii_os_rv);
	}

	disable_scanning(devinfo);

	if (wihd_sm_info.associate_pending)
		goto done;

	wvan_info_data = SiiOsCalloc("JoinWVAN", HM_WVAN_JOIN_CMD_MLEN, 0);
	if (NULL == wvan_info_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	field_ua_set(wihd_sm_info.next_wvan_info.wvan_id, wvan_info_data,
			HM_WVAN_JOIN_CMD_WVAN_ID);
	field_ua_set(wihd_sm_info.next_wvan_info.hr_channel, wvan_info_data,
			HM_WVAN_JOIN_CMD_HR_CHANNEL);
	field_ua_set(wihd_sm_info.next_wvan_info.lr_channel, wvan_info_data,
			HM_WVAN_JOIN_CMD_LR_CHANNEL);

	sii_os_rv = send_hostmsg_command((uint16_t)HM_WVAN_JOIN,
					HM_WVAN_JOIN_CMD_MLEN, wvan_info_data,
					HM_WVAN_JOIN_RESPONSE_TIMEOUT,
					NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("WiHD Join WVAN command not sent\n");
		resp_status = sii_os_rv;
		goto send_response;
	}
	dbg("WiHD Join WVAN successful\n");

	devinfo->wihd->state = WIHD_STATE_ASSOCIATING;
	sysfs_notify(&devinfo->wihd->device->kobj, NULL, __stringify(state));

	/* Association is happening, wait for success to set these values. */
	wihd_sm_info.wvan_info.wvan_id = 0;
	wihd_sm_info.wvan_info.hr_channel = 0;
	wihd_sm_info.wvan_info.lr_channel = 0;

send_response:
	resp.request_type = SII_SM_REQ_JOIN_WVAN;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	sii_os_rv = SiiOsQueueSend(join_wvan_resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not send response: err=%d\n", sii_os_rv);
		goto done;
	}

done:
	SiiOsFree(wvan_info_data);
}

enum sii_os_status join_wvan(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_wvan_info *wvan_info = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if ((NULL == req_data) ||
	    (sizeof(*wvan_info) != req_data_size)) {
		err("Illegal Join WVAN request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	wvan_info = req_data;

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_idle(devinfo->wihd->state) &&
	    !wihd_is_scanning(devinfo->wihd->state)) {
		dbg("WiHD WVAN Join request not allowed from state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	wihd_sm_info.next_wvan_info.wvan_id = wvan_info->wvan_id;
	wihd_sm_info.next_wvan_info.hr_channel = wvan_info->hr_channel;
	wihd_sm_info.next_wvan_info.lr_channel = wvan_info->lr_channel;

	associate_to_wvan(devinfo); /* The response will be sent here */
	goto done;

send_response:
	resp.request_type = SII_SM_REQ_JOIN_WVAN;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status leave_wvan(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	wihd_sm_info.associate_pending = false;

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_associating(devinfo->wihd->state) &&
	    !wihd_is_associated(devinfo->wihd->state) &&
	    !wihd_is_connecting(devinfo->wihd->state) &&
	    !wihd_is_connected(devinfo->wihd->state)) {
		dbg("WiHD WVAN disassociate not allowed from state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		/* Ignore request */
		goto send_response;
	}

	retval = send_hostmsg_command((uint16_t)HM_WVAN_LEAVE, 0, NULL,
				HM_WVAN_LEAVE_RESPONSE_TIMEOUT, NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("WiHD Leave WVAN command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("WiHD Leave WVAN successful\n");

send_response:
	resp.request_type = SII_SM_REQ_LEAVE_WVAN;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_wvan_info(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_wvan_info *wvan_info = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* This memory is freed by the response handler */
	wvan_info = SiiOsCalloc("GetWVANInfo", sizeof(*wvan_info), 0);
	if (NULL == wvan_info) {
		err("Out of memory\n");
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if ((!wihd_is_associated(devinfo->wihd->state)) &&
	    (!wihd_is_connecting(devinfo->wihd->state)) &&
	    (!wihd_is_connected(devinfo->wihd->state))) {
		wvan_info->wvan_id = 0;
		wvan_info->hr_channel = 0;
		wvan_info->lr_channel = 0;
	} else {
		wvan_info->wvan_id = wihd_sm_info.wvan_info.wvan_id;
		wvan_info->hr_channel = wihd_sm_info.wvan_info.hr_channel;
		wvan_info->lr_channel = wihd_sm_info.wvan_info.lr_channel;
	}

send_response:
	resp.request_type = SII_SM_REQ_GET_WVAN_INFO;
	resp.resp_status = resp_status;
	resp.data = wvan_info;
	resp.data_size = (NULL != wvan_info) ? sizeof(*wvan_info) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(wvan_info);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_connection_status(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_wvan_connection_status *connection_status = NULL;
	struct state_machine_response resp;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_associated(devinfo->wihd->state) &&
		!wihd_is_connecting(devinfo->wihd->state) &&
		!wihd_is_connected(devinfo->wihd->state)) {
		dbg("Get Connection Status not allowed from state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	retval = send_hostmsg_command((uint16_t)HM_WVAN_GET_CONNECTION_STATUS,
				0, NULL,
				HM_REMOTE_CONNECTION_STATUS_RESPONSE_TIMEOUT,
				&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get Connection Status command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get Connection Status successful\n");

	if (HM_WVAN_GET_CONNECTION_STATUS_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	connection_status = SiiOsCalloc("GetConnectionStatus",
					sizeof(*connection_status), 0);
	if (NULL == connection_status) {
		err("Out of memory\n");
		goto send_response;
	}

	connection_status->status = (uint8_t)field_ua_extract(resp_data,
		HM_WVAN_GET_CONNECTION_STATUS_CMD_RSP_CONNECTION_STATUS);

send_response:
	resp.request_type = SII_SM_GET_CONNECTION_STATUS;
	resp.resp_status = resp_status;
	resp.data = connection_status;
	resp.data_size = (NULL != connection_status) ?
		sizeof(*connection_status) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		SiiOsFree(connection_status);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status get_device_list(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_remote_device_list *device_list = NULL;
	struct sii6400_device_info *devinfo = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t num_dev = 0;
	uint8_t i;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_associated(devinfo->wihd->state) &&
	    !wihd_is_connecting(devinfo->wihd->state) &&
	    !wihd_is_connected(devinfo->wihd->state)) {
		dbg("Get Device List not possible from current state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	retval = send_hostmsg_command((uint16_t)HM_GET_DEV_LIST, 0, NULL,
					HM_GET_DEV_LIST_RESPONSE_TIMEOUT,
					&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get Device List command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get Device List successful\n");

	dbg("resp_data_size = %u\n", resp_data_size);

	if (HM_GET_DEV_LIST_CMD_RSP_MLEN_MAX < resp_data_size) {
		dbg("Invalid search results: data size too large (%d)\n",
		    resp_data_size);
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (resp_data_size <= HM_GET_DEV_LIST_CMD_RSP_MLEN_MIN) {
		dbg("Invalid search results: data size too small (%d)\n",
		    resp_data_size);
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if ((resp_data_size - HM_GET_DEV_LIST_CMD_RSP_MLEN_MIN) %
				HM_GET_DEV_LIST_CMD_RSP_DEV_LEN) {
		err("Invalid search results: data size is incorrect\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	num_dev = (resp_data_size - HM_GET_DEV_LIST_CMD_RSP_MLEN_MIN) /
				HM_GET_DEV_LIST_CMD_RSP_DEV_LEN;
	if (HM_GET_DEV_LIST_CMD_RSP_MAX_DEVS < num_dev) {
		err("Invalid search results: too many results %d\n", num_dev);
		retval = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if ((HM_GET_DEV_LIST_CMD_RSP_MLEN_MIN == resp_data_size) ||
	    (0 == num_dev)) {
		/* No search results to parse */
		goto send_response;
	}

	device_list = SiiOsCalloc("GetDeviceList", sizeof(*device_list), 0);
	if (NULL == device_list) {
		err("Out of memory\n");
		goto send_response;
	}

	device_list->num_dev = num_dev;

	device_list->remote_devices = SiiOsCalloc("RemoteDevices",
			num_dev * sizeof(struct sii6400_remote_device), 0);
	if (NULL == device_list->remote_devices) {
		err("Out of memory\n");
		goto send_response;
	}

	for (i = 0; i < device_list->num_dev; i++) {
		struct sii6400_remote_device *remote_device_info =
					&device_list->remote_devices[i];
		void *cur_dev_info = ((uint8_t *)resp_data +
				HM_GET_DEV_LIST_CMD_RSP_MLEN_MIN +
				(i * HM_GET_DEV_LIST_CMD_RSP_DEV_LEN));
		uint8_t type = 0;

		memcpy(remote_device_info->mac_addr.mac_addr,
			(void *)FIELD_UA_BYTE_ADDRESS(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_MAC_ADDR0),
			sizeof(remote_device_info->mac_addr.mac_addr));
		memcpy(remote_device_info->name.name,
			(void *)FIELD_UA_BYTE_ADDRESS(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME0),
			sizeof(remote_device_info->name.name) - 1);
		remote_device_info->manufacturer.manufacturer[0] =
			(char)field_ua_extract(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG0) +
					'A' - 1;
		remote_device_info->manufacturer.manufacturer[1] =
			(char)field_ua_extract(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG1) +
					'A' - 1;
		remote_device_info->manufacturer.manufacturer[2] =
			(char)field_ua_extract(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG2) +
					'A' - 1;
		remote_device_info->category.category =
			(enum sii6400_wihd_category)field_ua_extract(
				cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_CATEGORY);
		type = (uint8_t)field_ua_extract(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_AV_TYPE);
		remote_device_info->type.video_source = type & 0x01;
		remote_device_info->type.video_sink = (type >> 1) & 0x01;
		remote_device_info->type.audio_source = (type >> 2) & 0x01;
		remote_device_info->type.audio_sink = (type >> 3) & 0x01;
	}

send_response:
	resp.request_type = SII_SM_GET_DEVICE_LIST;
	resp.resp_status = resp_status;
	resp.data = device_list;
	resp.data_size = (NULL != device_list) ? sizeof(*device_list) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		if (NULL != device_list)
			SiiOsFree(device_list->remote_devices);
		SiiOsFree(device_list);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status wihd_connect(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_connect *connect = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *connect_data = NULL;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	connect = req_data;

	if (sizeof(*connect) != req_data_size) {
		err("Illegal data for WiHD connect request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	if ((0 == connect->mac_addr[0]) &&
	    (0 == connect->mac_addr[1]) &&
	    (0 == connect->mac_addr[2]) &&
	    (0 == connect->mac_addr[3]) &&
	    (0 == connect->mac_addr[4]) &&
	    (0 == connect->mac_addr[5])) {
		err("Illegal MAC address for WiHD connect request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_associated(devinfo->wihd->state)) {
		dbg("Unable to connect from current state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	connect_data = SiiOsCalloc("ConnectData",
					HM_REMOTE_DEV_CONNECT_CMD_MLEN, 0);
	if (NULL == connect_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	memcpy(connect_data, connect->mac_addr,
		HM_REMOTE_DEV_CONNECT_CMD_MAC_ADDR_LEN);
	field_ua_set(connect->connect_timeout, connect_data,
			HM_REMOTE_DEV_CONNECT_CMD_TIMEOUT);

	retval = send_hostmsg_command((uint16_t)HM_REMOTE_DEV_CONNECT,
					HM_REMOTE_DEV_CONNECT_CMD_MLEN,
					connect_data,
					HM_REMOTE_DEV_CONNECT_RESPONSE_TIMEOUT,
					NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("WiHD Connect command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("WiHD Connect successful\n");

	devinfo->wihd->state = WIHD_STATE_CONNECTING;
	sysfs_notify(&devinfo->wihd->device->kobj, NULL, __stringify(state));

send_response:
	resp.request_type = SII_SM_REQ_WIHD_CONNECT;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(connect_data);
	return retval;
}

enum sii_os_status wihd_disconnect(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connecting(devinfo->wihd->state) &&
	    !wihd_is_connected(devinfo->wihd->state)) {
		dbg("Unable to disconnect from current state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	retval = send_hostmsg_command((uint16_t)HM_REMOTE_DEV_DISCONNECT,
				0, NULL,
				HM_REMOTE_DEV_DISCONNECT_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("WiHD Disconnect command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("WiHD Disconnect successful\n");

send_response:
	resp.request_type = SII_SM_REQ_WIHD_DISCONNECT;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status cec_send_message(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_cec_message *cec_message = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *cec_message_data = NULL;
	uint8_t *pHmPayload;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	cec_message = req_data;

	if (sizeof(*cec_message) != req_data_size) {
		err("Illegal data for CEC send message request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	if ((cec_message->length < HM_CEC_MSG_SEND_CMD_CEC_DATALEN_MIN) ||
	    (HM_CEC_MSG_SEND_CMD_CEC_DATALEN_MAX < cec_message->length)) {
		err("Illegal data for CEC send message request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("Unable to send CEC message from current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	cec_message_data = SiiOsCalloc("CECMessage",
		HM_CEC_MSG_SEND_CMD_CEC_HDRLEN + cec_message->length, 0);
	if (NULL == cec_message_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	field_ua_set(cec_message->length, cec_message_data,
			HM_CEC_MSG_SEND_CMD_CEC_DATALEN);

	pHmPayload = FIELD_UA_BYTE_ADDRESS(cec_message_data,
						HM_CEC_MSG_SEND_CMD_CEC_DATA0);
	memcpy((void *)pHmPayload, cec_message->cec_message,
		cec_message->length);

	retval = send_hostmsg_command((uint16_t)HM_CEC_MSG_SEND,
			HM_CEC_MSG_SEND_CMD_CEC_HDRLEN + cec_message->length,
			cec_message_data,
			HM_CEC_MSG_SEND_RESPONSE_TIMEOUT,
			NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("CEC Send Message command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("CEC Send Message successful\n");

send_response:
	resp.request_type = SII_SM_REQ_CEC_SEND_MSG;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(cec_message_data);
	return retval;
}

enum sii_os_status get_wihd_rc_rcvd_ctrlcode(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_wihd_rc *rc = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("WiHD Remote Control not available in current state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	rc = SiiOsCalloc("GetWIHDRCRcvdCtrl", sizeof(*rc), 0);
	if (NULL == rc) {
		err("Out of memory\n");
		goto send_response;
	}

	rc->ctrlcode = wihd_sm_info.rc_received;

send_response:
	resp.request_type = SII_SM_REQ_GET_WIHD_RC_RCVD_CTRLCODE;
	resp.resp_status = resp_status;
	resp.data = rc;
	resp.data_size = (NULL != rc) ? sizeof(*rc) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(rc);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_wihd_rc_sent_ctrlcode(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_wihd_rc *rc = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("WiHD Remote Control not available in current state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	rc = SiiOsCalloc("GetWIHDRCSentCtrl", sizeof(*rc), 0);
	if (NULL == rc) {
		err("Out of memory\n");
		goto send_response;
	}

	rc->ctrlcode = wihd_sm_info.rc_sent;

send_response:
	resp.request_type = SII_SM_REQ_GET_WIHD_RC_SENT_CTRLCODE;
	resp.resp_status = resp_status;
	resp.data = rc;
	resp.data_size = (NULL != rc) ? sizeof(*rc) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(rc);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status send_wihd_rc_ctrlcode(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_wihd_rc *rc = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *rc_data = NULL;
	uint8_t *pHmPayload = NULL;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	rc = req_data;

	if (sizeof(*rc) != req_data_size) {
		err("Illegal data for Send WIHD Remote Control Code request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_connected(devinfo->wihd->state)) {
		dbg("WiHD Remote Control not available in current state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	rc_data = SiiOsCalloc("SendWIHDRCCtrlData",
				HM_CEC_MSG_SEND_CMD_CEC_HDRLEN + 3, 0);
	if (NULL == rc_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	field_ua_set(3, rc_data, HM_CEC_MSG_SEND_CMD_CEC_DATALEN);
	pHmPayload = FIELD_UA_BYTE_ADDRESS(rc_data,
						HM_CEC_MSG_SEND_CMD_CEC_DATA0);
	pHmPayload[0] = CEC_MESSAGE_HEADER;
	pHmPayload[1] = CEC_REMOTE_CONTROL_PRESSED;
	pHmPayload[2] = rc->ctrlcode;

	retval = send_hostmsg_command((uint16_t)HM_CEC_MSG_SEND,
				HM_CEC_MSG_SEND_CMD_CEC_HDRLEN + 3, rc_data,
				HM_CEC_MSG_SEND_RESPONSE_TIMEOUT, NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send WiHD Remote Control Code command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Send WiHD Remote Control Code successful\n");

	wihd_sm_info.rc_sent = rc->ctrlcode;

send_response:
	resp.request_type = SII_SM_REQ_SEND_WIHD_RC_CTRLCODE;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(rc_data);
	return retval;
}

enum sii_os_status set_hdcp_policy(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_hdcp_policy *hdcp_policy = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *hdcp_policy_data = NULL;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	hdcp_policy = req_data;

	if (sizeof(*hdcp_policy) != req_data_size) {
		err("Illegal Policy for Set HDCP Policy request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (wihd_is_connecting(devinfo->wihd->state) ||
	    wihd_is_connected(devinfo->wihd->state)) {
		dbg("Set HDCP Policy command not allowed in state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	hdcp_policy_data = SiiOsCalloc("SetHDCPPolicy",
					HM_HDCP_SET_POLICY_CMD_MLEN, 0);
	if (NULL == hdcp_policy_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(hdcp_policy->policy, hdcp_policy_data,
			HM_HDCP_SET_POLICY_CMD_POLICY);

	retval = send_hostmsg_command((uint16_t)HM_HDCP_SET_POLICY,
					HM_HDCP_SET_POLICY_CMD_MLEN,
					hdcp_policy_data,
					HM_HDCP_SET_POLICY_RESPONSE_TIMEOUT,
					NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set HDCP Policy command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Set HDCP Policy successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SET_HDCP_POLICY;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(hdcp_policy_data);
	return retval;
}

enum sii_os_status get_hdcp_policy(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_hdcp_policy *hdcp_policy = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_HDCP_GET_POLICY, 0, NULL,
					HM_HDCP_GET_POLICY_RESPONSE_TIMEOUT,
					&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get HDCP Policy command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get HDCP Policy successful\n");

	if (HM_HDCP_GET_POLICY_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	hdcp_policy = SiiOsCalloc("GetHDCPPolicy", sizeof(*hdcp_policy), 0);
	if (NULL == hdcp_policy) {
		err("Out of memory\n");
		goto send_response;
	}

	hdcp_policy->policy = (uint8_t)field_ua_extract(resp_data,
					HM_HDCP_GET_POLICY_CMD_RSP_POLICY);

send_response:
	resp.request_type = SII_SM_REQ_GET_HDCP_POLICY;
	resp.resp_status = resp_status;
	resp.data = hdcp_policy;
	resp.data_size = (NULL != hdcp_policy) ? sizeof(*hdcp_policy) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(hdcp_policy);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status set_hdcp_stream_type(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_hdcp_stream_type *hdcp_stream_type = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *hdcp_stream_type_data = NULL;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	hdcp_stream_type = req_data;

	if (sizeof(*hdcp_stream_type) != req_data_size) {
		err("Illegal Stream Type for Set HDCP Stream Type request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	hdcp_stream_type_data = SiiOsCalloc("SetHDCPStreamType",
					HM_HDCP_SET_STREAM_TYPE_CMD_MLEN, 0);
	if (NULL == hdcp_stream_type_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(hdcp_stream_type->stream_type, hdcp_stream_type_data,
			HM_HDCP_SET_STREAM_TYPE_CMD_STREAM_TYPE);

	retval = send_hostmsg_command((uint16_t)HM_HDCP_SET_STREAM_TYPE,
				HM_HDCP_SET_STREAM_TYPE_CMD_MLEN,
				hdcp_stream_type_data,
				HM_HDCP_SET_STREAM_TYPE_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set HDCP Stream Type command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Set HDCP Stream Type successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SET_HDCP_STREAM_TYPE;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(hdcp_stream_type_data);
	return retval;
}

enum sii_os_status get_hdcp_stream_type(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_hdcp_stream_type *hdcp_stream_type = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_HDCP_GET_STREAM_TYPE,
				0, NULL,
				HM_HDCP_GET_STREAM_TYPE_RESPONSE_TIMEOUT,
				&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get HDCP Stream Type command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get HDCP Stream Type successful\n");

	if (HM_HDCP_GET_STREAM_TYPE_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	hdcp_stream_type = SiiOsCalloc("GetHDCPStreamType",
					sizeof(*hdcp_stream_type), 0);
	if (NULL == hdcp_stream_type) {
		err("Out of memory\n");
		goto send_response;
	}

	hdcp_stream_type->stream_type = (uint8_t)field_ua_extract(resp_data,
				HM_HDCP_GET_STREAM_TYPE_CMD_RSP_STREAM_TYPE);

send_response:
	resp.request_type = SII_SM_REQ_GET_HDCP_STREAM_TYPE;
	resp.resp_status = resp_status;
	resp.data = hdcp_stream_type;
	resp.data_size = (NULL != hdcp_stream_type) ?
					sizeof(*hdcp_stream_type) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(hdcp_stream_type);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status report_scan_results(void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_scan_results *scan_results = NULL;
	uint8_t i;
	int offset = 0;
	char *buf = NULL;
	size_t buf_size = PAGE_SIZE;
	char *scan_results_str = NULL;
	enum CmdRspResult cmd_resp;
	uint8_t num_wvan = 0;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	devinfo->wihd->state = WIHD_STATE_IDLE;
	sysfs_notify(&devinfo->wihd->device->kobj, NULL, __stringify(state));

	retval = SiiOsSemaphoreTake(wihd_sm_info.scan_sem,
					MAX_WAIT_FOR_SCAN_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not take scan semaphore: %x\n", retval);
	} else {
		/* Stop scan response timer */
		if (NULL != wvan_scan_response_timer) {
			/* Delete the scan response timer */
			retval = SiiOsTimerDelete(wvan_scan_response_timer);
			if (SII_OS_STATUS_SUCCESS != retval) {
				err("Could not stop scan response timer: %x\n",
				    retval);
			} else {
				wvan_scan_response_timer = NULL;
				/* Recreate the scan response timer */
				retval = SiiOsTimerCreate(
					"wvan_scan_response_timer",
					wvan_scan_response_timer_function,
					NULL, false, 0, false,
					&wvan_scan_response_timer);
				if (SII_OS_STATUS_SUCCESS != retval) {
					err("Unable to create timer: %x\n",
						retval);
					wvan_scan_response_timer = NULL;
				}
			}
		}

		retval = SiiOsSemaphoreGive(wihd_sm_info.scan_sem);
		if (SII_OS_STATUS_SUCCESS != retval)
			err("Could not release scan semaphore: %x\n", retval);
	}

	if (wihd_sm_info.disable_scanning_pending)
		disable_scanning(devinfo);

	if (wihd_sm_info.restart_scanning_pending)
		restart_scanning(devinfo);

	if (wihd_sm_info.associate_pending)
		associate_to_wvan(devinfo);

	scan_results = SiiOsCalloc("ScanResults", sizeof(*scan_results), 0);
	if (NULL == scan_results)
		err("Out of memory\n");

	if ((NULL == req_data) || (0 == req_data_size)) {
		err("Missing scan data\n");
		goto data_parse_done;
	}

	cmd_resp = (enum CmdRspResult)field_ua_extract(req_data,
						HM_COMMON_CMD_RSP_RESULT_CODE);
	switch (cmd_resp) {
	case CrrSuccess:
		dbg("Scan successful\n");
		retval = SII_OS_STATUS_SUCCESS;
		break;

	case CrrErrInvalidParam:
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		break;

	case CrrErrFail:
	default:
		err("Scan failed or nothing found\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		break;
	}

	if (SII_OS_STATUS_SUCCESS != retval)
		goto data_parse_done;

	dbg("req_data_size = %u\n", req_data_size);

	if (HM_WVAN_SCAN_CMD_RSP_MLEN_MAX < req_data_size) {
		dbg("Invalid scan results: data size too large (%d)\n",
		    req_data_size);
		retval = SII_OS_STATUS_ERR_FAILED;
		goto data_parse_done;
	}

	if (req_data_size <= HM_WVAN_SCAN_CMD_RSP_MLEN_MIN) {
		dbg("Invalid scan results: data size too small (%d)\n",
		    req_data_size);
		retval = SII_OS_STATUS_ERR_FAILED;
		goto data_parse_done;
	}

	if ((req_data_size - HM_WVAN_SCAN_CMD_RSP_MLEN_MIN) %
				HM_WVAN_SCAN_CMD_RSP_DEV_LEN) {
		err("Invalid scan results: data size is incorrect\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto data_parse_done;
	}

	num_wvan = (req_data_size - HM_WVAN_SCAN_CMD_RSP_MLEN_MIN) /
				HM_WVAN_SCAN_CMD_RSP_DEV_LEN;
	if (HM_WVAN_SCAN_CMD_RSP_MAX_DEVS < num_wvan) {
		err("Invalid scan results: too many results %d\n", num_wvan);
		retval = SII_OS_STATUS_ERR_FAILED;
		goto data_parse_done;
	}

	if ((HM_WVAN_SCAN_CMD_RSP_MLEN_MIN == req_data_size) ||
	    (NULL == scan_results)) {
		/* No data to parse */
		goto data_parse_done;
	}

	scan_results->num_wvan = num_wvan;
	scan_results->wvans = SiiOsCalloc("ScanResultsWVANs",
		scan_results->num_wvan * sizeof(struct sii6400_wvan), 0);
	if (NULL == scan_results->wvans) {
		err("Out of memory\n");
		goto data_parse_done;
	}

	for (i = 0; i < num_wvan; i++) {
		struct sii6400_wvan *wvan = &scan_results->wvans[i];
		void *cur_wvan_info = ((uint8_t *)req_data +
					HM_WVAN_SCAN_CMD_RSP_MLEN_MIN +
					(i * HM_WVAN_SCAN_CMD_RSP_DEV_LEN));

		memcpy(wvan->wvan_name,
			(void *)FIELD_UA_BYTE_ADDRESS(cur_wvan_info,
					HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME0),
			sizeof(wvan->wvan_name) - 1);
		wvan->wvan_id = (uint8_t)field_ua_extract(cur_wvan_info,
					HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_ID);
		wvan->hr_channel = (uint8_t)field_ua_extract(cur_wvan_info,
					HM_WVAN_SCAN_CMD_RSP_WVAN_X_HR_CHANNEL);
		wvan->lr_channel = (uint8_t)field_ua_extract(cur_wvan_info,
					HM_WVAN_SCAN_CMD_RSP_WVAN_X_LR_CHANNEL);
		wvan->signal_strength = (uint8_t)field_ua_extract(cur_wvan_info,
					HM_WVAN_SCAN_CMD_RSP_WVAN_X_BEACON_STR);
	}

data_parse_done:
	scan_results_str = SiiOsCalloc("ScanResultsStr", buf_size, 0);
	if (NULL == scan_results_str) {
		err("Out of memory\n");
		goto done;
	}
	buf = scan_results_str;
	offset += scnprintf(buf, buf_size, "[");
	if ((NULL != scan_results) && (NULL != scan_results->wvans)) {
		for (i = 0; i < scan_results->num_wvan; i++) {
			struct sii6400_wvan *wvan = &scan_results->wvans[i];

			if (i) {
				offset += scnprintf(buf + offset,
							buf_size - offset, ",");
			}
			offset += scnprintf(buf + offset, buf_size - offset,
					"{");
			offset += scnprintf(buf + offset, buf_size - offset,
					"\"name\":\"");
			offset += scnprintf(buf + offset, buf_size - offset,
					"%s", wvan->wvan_name);
			offset += scnprintf(buf + offset, buf_size - offset,
					"\",\"id\":");
			offset += scnprintf(buf + offset, buf_size - offset,
					"%u", wvan->wvan_id);
			offset += scnprintf(buf + offset, buf_size - offset,
					",\"hr\":");
			offset += scnprintf(buf + offset, buf_size - offset,
					"%u", wvan->hr_channel);
			offset += scnprintf(buf + offset, buf_size - offset,
					",\"lr\":");
			offset += scnprintf(buf + offset, buf_size - offset,
					"%u", wvan->lr_channel);
			offset += scnprintf(buf + offset, buf_size - offset,
					",\"strength\":");
			offset += scnprintf(buf + offset, buf_size - offset,
					"%u", wvan->signal_strength);
			offset += scnprintf(buf + offset, buf_size - offset,
					"}");
		}
	}
	offset += scnprintf(buf + offset, buf_size - offset, "]");

	(void)send_sii6400_uevent(devinfo->wihd->device, WIHD_EVENT,
				WIHD_SCAN_COMPLETE_EVENT, scan_results_str);
	SiiOsFree(scan_results_str);

done:
	if (NULL != scan_results)
		SiiOsFree(scan_results->wvans);
	SiiOsFree(scan_results);
	return retval;
}

enum sii_os_status report_associated(void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	wihd_sm_info.wvan_info.wvan_id = wihd_sm_info.next_wvan_info.wvan_id;
	wihd_sm_info.wvan_info.hr_channel =
					wihd_sm_info.next_wvan_info.hr_channel;
	wihd_sm_info.wvan_info.lr_channel =
					wihd_sm_info.next_wvan_info.lr_channel;

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	devinfo->wihd->state = WIHD_STATE_ASSOCIATED;
	(void)send_sii6400_uevent(devinfo->wihd->device, WIHD_EVENT,
					WIHD_ASSOCIATED_EVENT, NULL);
	sysfs_notify(&devinfo->wihd->device->kobj, NULL, __stringify(state));

done:
	return retval;
}

enum sii_os_status report_dev_list_changed(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	char *char_report_data_str = NULL;
	char *buf = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_remote_device_list *device_list = NULL;
	uint8_t dev_num = 0;
	uint8_t i = 0;
	size_t buf_size = PAGE_SIZE;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if (NULL == req_data) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto data_parse_done;
	}

	if (HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MAX < req_data_size) {
		err("Invalid search results: data size too large (%d)\n",
		    req_data_size);
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto data_parse_done;
	}

	if (HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MIN >= req_data_size) {
		err("Invalid search results: data size too small (%d)\n",
		    req_data_size);
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto data_parse_done;
	}

	if ((req_data_size - HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MIN) %
				HM_DEV_LIST_CHANGED_DEV_LEN) {
		err("Invalid search results: data size is incorrect\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto data_parse_done;
	}

	dev_num = (req_data_size - HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MIN) /
				HM_DEV_LIST_CHANGED_DEV_LEN;

	if (HM_DEV_LIST_CHANGED_MAX_DEVS < dev_num) {
		err("Invalid search results: too many results %d\n", dev_num);
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto data_parse_done;
	}

	if ((HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MIN == req_data_size) ||
	    (0 == dev_num)) {
		/* No search results to parse */
		goto data_parse_done;
	}

	device_list = SiiOsCalloc("DeviceList",
					sizeof(*device_list), 0);
	if (NULL == device_list) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto data_parse_done;
	}
	device_list->num_dev = dev_num;
	device_list->remote_devices = SiiOsCalloc("RemoteDevList",
			dev_num * sizeof(struct sii6400_remote_device), 0);
	if (NULL == device_list->remote_devices) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto data_parse_done;
	}

	for (i = 0; i < dev_num; i++) {
		struct sii6400_remote_device *remote_device_info =
			&device_list->remote_devices[i];
		void *cur_dev_info = ((uint8_t *)req_data +
				HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MIN +
				i * HM_DEV_LIST_CHANGED_DEV_LEN);
		uint8_t type = 0;

		memcpy(remote_device_info->mac_addr.mac_addr,
			(void *)FIELD_UA_BYTE_ADDRESS(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_MAC_ADDR0),
			sizeof(remote_device_info->mac_addr.mac_addr));

		memcpy(remote_device_info->name.name,
			(void *)FIELD_UA_BYTE_ADDRESS(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME0),
			sizeof(remote_device_info->name.name) - 1);

		remote_device_info->category.category =
			(enum sii6400_wihd_category)field_ua_extract(
				cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_CATEGORY);
		type = (uint8_t)field_ua_extract(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_AV_TYPE);
		remote_device_info->type.video_source = type & 0x01;
		remote_device_info->type.video_sink = (type >> 1) & 0x01;
		remote_device_info->type.audio_source = (type >> 2) & 0x01;
		remote_device_info->type.audio_sink = (type >> 3) & 0x01;
		remote_device_info->manufacturer.manufacturer[0] =
			(char)field_ua_extract(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG0) +
					'A' - 1;
		remote_device_info->manufacturer.manufacturer[1] =
			(char)field_ua_extract(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG1) +
					'A' - 1;
		remote_device_info->manufacturer.manufacturer[2] =
			(char)field_ua_extract(cur_dev_info,
				HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG2) +
					'A' - 1;
	}

data_parse_done:
	if ((NULL == device_list) ||
	    (NULL == device_list->remote_devices)) {
		(void)send_sii6400_uevent(devinfo->wihd->device, WIHD_EVENT,
					WIHD_DEVICE_LIST_COMPLETE_EVENT, "");
		goto done;
	}

	char_report_data_str = SiiOsCalloc("DevListChanged", buf_size, 0);
	if (NULL == char_report_data_str) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	buf = char_report_data_str;

	for (i = 0; i < dev_num; i++) {
		int offset = 0;
		struct sii6400_remote_device *remote_device =
					&device_list->remote_devices[i];
		const char *event_str = (i == device_list->num_dev - 1) ?
					WIHD_DEVICE_LIST_COMPLETE_EVENT :
					WIHD_DEVICE_LIST_ENTRY_EVENT;

		if (i) {
			offset += scnprintf(buf + offset, buf_size - offset,
						",");
		}
		offset += scnprintf(buf + offset, buf_size - offset, "{");
		offset += scnprintf(buf + offset, buf_size - offset,
					"\"mac_addr\":\"");
		offset += scnprintf(buf + offset, buf_size - offset,
					"%02x%02x%02x%02x%02x%02x",
					remote_device->mac_addr.mac_addr[0],
					remote_device->mac_addr.mac_addr[1],
					remote_device->mac_addr.mac_addr[2],
					remote_device->mac_addr.mac_addr[3],
					remote_device->mac_addr.mac_addr[4],
					remote_device->mac_addr.mac_addr[5]);
		offset += scnprintf(buf + offset, buf_size - offset,
					"\",\"name\":\"");
		offset += scnprintf(buf + offset, buf_size - offset,
					"%s", remote_device->name.name);
		offset += scnprintf(buf + offset, buf_size - offset,
					"\",\"manufacturer\":\"");
		offset += scnprintf(buf + offset, buf_size - offset, "%s",
				remote_device->manufacturer.manufacturer);
		offset += scnprintf(buf + offset, buf_size - offset,
					"\",\"category\":\"");
		offset += scnprintf(buf + offset, buf_size - offset, "%s",
				get_category_string(&remote_device->category));
		offset += scnprintf(buf + offset, buf_size - offset,
					"\",\"type\":");
		offset += scnprintf(buf + offset, buf_size - offset,
			"{\"video_source\":%s,\"video_sink\":%s,"
			"\"audio_source\":%s,\"audio_sink\":%s}",
			remote_device->type.video_source ? "true" : "false",
			remote_device->type.video_sink ? "true" : "false",
			remote_device->type.audio_source ? "true" : "false",
			remote_device->type.audio_sink ? "true" : "false");
		offset += scnprintf(buf + offset, buf_size - offset, "}");

		(void)send_sii6400_uevent(devinfo->wihd->device, WIHD_EVENT,
					event_str, char_report_data_str);
	}
	SiiOsFree(char_report_data_str);

done:
	if (NULL != device_list)
		SiiOsFree(device_list->remote_devices);
	SiiOsFree(device_list);
	return retval;
}

enum sii_os_status report_disassociated(void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;
	const char *reason_str = NULL;

	dbg("");

	/* Clear the WVAN Info now that we are disassociated */
	memset(&wihd_sm_info.wvan_info, 0, sizeof(struct sii6400_wvan_info));
	memset(&wihd_sm_info.next_wvan_info, 0,
		sizeof(struct sii6400_wvan_info));

	if ((NULL != req_data) && (0 != req_data_size)) {
		enum SiI_Notification_Reason reason =
			(enum SiI_Notification_Reason)field_ua_extract(req_data,
					HM_WVAN_DISASSOCIATED_NOTIFY_REASON);
		reason_str = get_wihd_uevent_reason_string(reason);
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	devinfo->wihd->state = WIHD_STATE_IDLE;
	(void)send_sii6400_uevent(devinfo->wihd->device, WIHD_EVENT,
					WIHD_DISASSOCIATED_EVENT, reason_str);
	sysfs_notify(&devinfo->wihd->device->kobj, NULL, __stringify(state));

done:
	return retval;
}

enum sii_os_status report_connected(void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	/* Get the Remote Device Info now that we are connected */
	retval = receive_remote_device_info(&wihd_sm_info.remote_device);
	if (SII_OS_STATUS_SUCCESS != retval)
		err("Failed to receive Remote Device Info\n");

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	devinfo->wihd->state = WIHD_STATE_CONNECTED;
	(void)send_sii6400_uevent(devinfo->wihd->device, WIHD_EVENT,
					WIHD_CONNECTED_EVENT, NULL);
	sysfs_notify(&devinfo->wihd->device->kobj, NULL, __stringify(state));

done:
	return retval;
}

enum sii_os_status report_disconnected(void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;
	const char *reason_str = NULL;

	dbg("");

	/* Clear the Remote Device Info now that we are disconnected */
	memset(&wihd_sm_info.remote_device, 0,
		sizeof(struct sii6400_remote_device));

	if ((NULL != req_data) && (0 != req_data_size)) {
		enum SiI_Notification_Reason reason =
			(enum SiI_Notification_Reason)field_ua_extract(req_data,
				HM_REMOTE_DEV_DISCONNECTED_NOTIFY_REASON);
		reason_str = get_wihd_uevent_reason_string(reason);
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	devinfo->wihd->state = WIHD_STATE_ASSOCIATED;
	(void)send_sii6400_uevent(devinfo->wihd->device, WIHD_EVENT,
					WIHD_DISCONNECTED_EVENT, reason_str);
	sysfs_notify(&devinfo->wihd->device->kobj, NULL, __stringify(state));

done:
	return retval;
}

enum sii_os_status report_cec_msg_received(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t keycode = 0;
	struct sii6400_device_info *devinfo = NULL;
	char *key_report_data_str = NULL;

	dbg("");

	if ((NULL == req_data) ||
	    (HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY_MLEN != req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* Only the keycode (third byte) is needed. */
	keycode = (uint8_t)field_ua_extract(req_data,
				HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY_CEC_MSG2);

	wihd_sm_info.rc_received = keycode;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if (INPUT_DEVICE_APP == devinfo->my_rc_input_device) {
		key_report_data_str = SiiOsCalloc("ReportRCKey",
				MAX_WIHD_KEY_REPORT_DATA_STRING_SIZE, 0);
		if (NULL == key_report_data_str) {
			err("Out of memory\n");
			retval = SII_OS_STATUS_ERR_FAILED;
			goto done;
		}

		scnprintf(key_report_data_str,
				MAX_WIHD_KEY_REPORT_DATA_STRING_SIZE,
				"0x%02x", keycode);

		if (NULL != devinfo->wihd) {
			(void)send_sii6400_uevent(devinfo->wihd->device,
					WIHD_EVENT, WIHD_RECEIVED_RC_EVENT,
					key_report_data_str);
		}
	} else {
		if (rcp_support_table[keycode & RCP_KEY_ID_MASK].rcp_support &
							LOGICAL_DEVICE_MAP) {
			if (0 == generate_rcp_input_event(keycode))
				dbg("0x%hhx", keycode);
			else
				dbg("-0x%hhx", keycode);
		} else {
			dbg("-0x%hhx", keycode);
		}
	}

done:
	SiiOsFree(key_report_data_str);
	return retval;
}

enum sii_os_status report_vendor_msg_received(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_vendor_msg *vendor_msg_rcvd = NULL;
	char *char_report_data_str = NULL;
	char *buf = NULL;
	int offset = 0;
	uint8_t i = 0;
	size_t buf_size = PAGE_SIZE;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if (NULL == req_data) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto data_parse_done;
	}

	if (HM_VENDOR_MSG_RCVD_NOTIFY_MLEN_MAX < req_data_size) {
		err("Invalid vendor msg rcvd: data size too large (%d)\n",
		    req_data_size);
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto data_parse_done;
	}

	if (HM_VENDOR_MSG_RCVD_NOTIFY_MLEN_MIN > req_data_size) {
		err("Invalid vendor msg rcvd: data size too small (%d)\n",
		    req_data_size);
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto data_parse_done;
	}

	dbg("req_data_size = %u\n", req_data_size);

	vendor_msg_rcvd = SiiOsCalloc("VendorMsgRcvd",
					sizeof(*vendor_msg_rcvd), 0);
	if (NULL == vendor_msg_rcvd) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto data_parse_done;
	}

	vendor_msg_rcvd->vendor_id[0] = (uint8_t)field_ua_extract(req_data,
		HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_VID0);
	vendor_msg_rcvd->vendor_id[1] = (uint8_t)field_ua_extract(req_data,
		HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_VID1);
	vendor_msg_rcvd->vendor_id[2] = (uint8_t)field_ua_extract(req_data,
		HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_VID2);
	vendor_msg_rcvd->mac_addr[0] = (uint8_t)field_ua_extract(req_data,
		HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR0);
	vendor_msg_rcvd->mac_addr[1] = (uint8_t)field_ua_extract(req_data,
		HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR1);
	vendor_msg_rcvd->mac_addr[2] = (uint8_t)field_ua_extract(req_data,
		HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR2);
	vendor_msg_rcvd->mac_addr[3] = (uint8_t)field_ua_extract(req_data,
		HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR3);
	vendor_msg_rcvd->mac_addr[4] = (uint8_t)field_ua_extract(req_data,
		HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR4);
	vendor_msg_rcvd->mac_addr[5] = (uint8_t)field_ua_extract(req_data,
		HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR5);
	vendor_msg_rcvd->dataLength = req_data_size -
		HM_VENDOR_MSG_RCVD_NOTIFY_MLEN_MIN + 1;
	memcpy(vendor_msg_rcvd->data, (void *)FIELD_UA_BYTE_ADDRESS(req_data,
		HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MSG_DATA0),
		vendor_msg_rcvd->dataLength);

data_parse_done:
	if (NULL == vendor_msg_rcvd) {
		(void)send_sii6400_uevent(devinfo->wihd->device, WIHD_EVENT,
					WIHD_VENDOR_MSG_RCVD_EVENT, "");
		goto done;
	}

	char_report_data_str = SiiOsCalloc("VendorMsgRecvStr", buf_size, 0);
	if (NULL == char_report_data_str) {
		err("Out of memory\n");
		goto done;
	}
	buf = char_report_data_str;
	offset += scnprintf(buf + offset, buf_size - offset, "{");
	offset += scnprintf(buf + offset, buf_size - offset,
				"\"vendor_id\":\"");
	offset += scnprintf(buf + offset, buf_size - offset,
				"%02x%02x%02x",
				vendor_msg_rcvd->vendor_id[0],
				vendor_msg_rcvd->vendor_id[1],
				vendor_msg_rcvd->vendor_id[2]);
	offset += scnprintf(buf + offset, buf_size - offset,
				"\",\"mac_addr\":\"");
	offset += scnprintf(buf + offset, buf_size - offset,
				"%02x%02x%02x%02x%02x%02x",
				vendor_msg_rcvd->mac_addr[0],
				vendor_msg_rcvd->mac_addr[1],
				vendor_msg_rcvd->mac_addr[2],
				vendor_msg_rcvd->mac_addr[3],
				vendor_msg_rcvd->mac_addr[4],
				vendor_msg_rcvd->mac_addr[5]);
	offset += scnprintf(buf + offset, buf_size - offset,
				"\",\"vendor_msg\":\"");
	for (i = 0; i < vendor_msg_rcvd->dataLength; i++)
		offset += scnprintf(buf+offset, buf_size - offset,
					"%02x", vendor_msg_rcvd->data[i]);
	offset += scnprintf(buf + offset, buf_size - offset, "\"}");

	(void)send_sii6400_uevent(devinfo->wihd->device, WIHD_EVENT,
			WIHD_VENDOR_MSG_RCVD_EVENT, char_report_data_str);
	SiiOsFree(char_report_data_str);

done:
	if (NULL != vendor_msg_rcvd)
		SiiOsFree(vendor_msg_rcvd);
	return retval;
}

enum sii_os_status set_remote_fw_update(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_remote_fw_update_status *remote_fw_update_status = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	remote_fw_update_status = req_data;
	if (sizeof(*remote_fw_update_status) != req_data_size) {
		err("Illegal remote device firmware update status\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_associated(devinfo->wihd->state) &&
	    !wihd_is_connecting(devinfo->wihd->state) &&
	    !wihd_is_connected(devinfo->wihd->state)) {
		dbg("Remote device firmware update not allowed in state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	if (true == remote_fw_update_status->remote_fw_update_enabled)
		resp_status = sii6400_remote_fw_update_start();
	else
		resp_status = sii6400_remote_fw_update_abort();

send_response:
	resp.request_type = SII_SM_REQ_SET_REMOTE_FW_UPDATE;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status set_vendor_msg_recv_filter(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_vendor_id *vendor_id = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *vendor_msg_recv_filter_data = NULL;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	vendor_id = req_data;
	if (sizeof(*vendor_id) != req_data_size) {
		err("Illegal vendor msg recv filter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	vendor_msg_recv_filter_data = SiiOsCalloc("SetVendorMsgFilter",
					HM_SET_VENDOR_MSG_FILTER_CMD_MLEN, 0);
	if (NULL == vendor_msg_recv_filter_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	field_ua_set(vendor_id->vendor_id[0], vendor_msg_recv_filter_data,
			HM_SET_VENDOR_MSG_FILTER_CMD_VID0);
	field_ua_set(vendor_id->vendor_id[1], vendor_msg_recv_filter_data,
			HM_SET_VENDOR_MSG_FILTER_CMD_VID1);
	field_ua_set(vendor_id->vendor_id[2], vendor_msg_recv_filter_data,
			HM_SET_VENDOR_MSG_FILTER_CMD_VID2);
	retval = send_hostmsg_command((uint16_t)HM_SET_VENDOR_MSG_FILTER,
			HM_SET_VENDOR_MSG_FILTER_CMD_MLEN,
			vendor_msg_recv_filter_data,
			HM_SET_VENDOR_MSG_FILTER_RESPONSE_TIMEOUT,
			NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set Vendor Msg Recv Filter command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Set Vendor Msg Recv Filter successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SET_VENDOR_MSG_RECV_FILTER;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(vendor_msg_recv_filter_data);
	return retval;
}

enum sii_os_status get_vendor_msg_recv_filter(struct SiiOsQueue *resp_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_vendor_id *vendor_id = NULL;
	struct sii6400_device_info *devinfo = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	retval = send_hostmsg_command((uint16_t)HM_GET_VENDOR_MSG_FILTER,
			0,
			NULL,
			HM_GET_VENDOR_MSG_FILTER_RESPONSE_TIMEOUT,
			&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get Vendor Msg Recv Filter command not sent\n");
		resp_status = retval;
		goto send_response;
	}

	dbg("Get Vendor Msg Recv Filter successful\n");

	if (HM_GET_VENDOR_MSG_FILTER_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	vendor_id = SiiOsCalloc("GetVendorMsgFilter", sizeof(*vendor_id), 0);
	if (NULL == vendor_id) {
		err("Out of memory\n");
		goto send_response;
	}

	vendor_id->vendor_id[0] = (uint8_t)field_ua_extract(resp_data,
			HM_GET_VENDOR_MSG_FILTER_CMD_RSP_VID0);
	vendor_id->vendor_id[1] = (uint8_t)field_ua_extract(resp_data,
			HM_GET_VENDOR_MSG_FILTER_CMD_RSP_VID1);
	vendor_id->vendor_id[2] = (uint8_t)field_ua_extract(resp_data,
			HM_GET_VENDOR_MSG_FILTER_CMD_RSP_VID2);

send_response:
	resp.request_type = SII_SM_REQ_GET_VENDOR_MSG_RECV_FILTER;
	resp.resp_status = resp_status;
	resp.data = vendor_id;
	resp.data_size = (NULL != vendor_id) ? sizeof(*vendor_id) : 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(vendor_id);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status set_vendor_msg_vendor_id(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_vendor_id *vendor_id = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	vendor_id = req_data;
	if (sizeof(*vendor_id) != req_data_size) {
		err("Illegal vendor msg vendor id\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	memcpy(vendor_msg_vendor_id.vendor_id,
					vendor_id->vendor_id, VENDOR_ID_LEN);

	dbg("Set Vendor Msg Vendor ID successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SET_VENDOR_MSG_VENDOR_ID;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status set_vendor_msg_mac_addr(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mac_addr *dest_mac_addr = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	dest_mac_addr = req_data;
	if (sizeof(*dest_mac_addr) != req_data_size) {
		err("Illegal vendor msg dest MAC addr\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	memcpy(vendor_msg_dest_addr.mac_addr,
		dest_mac_addr->mac_addr, MAC_ADDR_LEN);

	dbg("Set Vendor Msg Dest MAC Addr successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SET_VENDOR_MSG_MAC_ADDR;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status send_vendor_msg(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_vendor_msg_payload *msg_payload = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *vendor_msg_send_data = NULL;
	dbg("");

	if (NULL == resp_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	msg_payload = req_data;
	if (sizeof(*msg_payload) != req_data_size) {
		err("Illegal vendor msg payload\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->wihd)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!wihd_is_associated(devinfo->wihd->state) &&
	    !wihd_is_connecting(devinfo->wihd->state) &&
	    !wihd_is_connected(devinfo->wihd->state)) {
		dbg("Send Vendor Msg not possible from current state (%s)\n",
		    get_wihd_state_string(devinfo->wihd->state));
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	vendor_msg_send_data = SiiOsCalloc("SendVendorMsg",
			HM_VENDOR_MSG_HDR_LEN + msg_payload->dataLength, 0);
	if (NULL == vendor_msg_send_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	field_ua_set(vendor_msg_vendor_id.vendor_id[0], vendor_msg_send_data,
			HM_VENDOR_MSG_SEND_CMD_VID0);
	field_ua_set(vendor_msg_vendor_id.vendor_id[1], vendor_msg_send_data,
			HM_VENDOR_MSG_SEND_CMD_VID1);
	field_ua_set(vendor_msg_vendor_id.vendor_id[2], vendor_msg_send_data,
			HM_VENDOR_MSG_SEND_CMD_VID2);
	field_ua_set(vendor_msg_dest_addr.mac_addr[0], vendor_msg_send_data,
			HM_VENDOR_MSG_SEND_CMD_MAC_ADDR0);
	field_ua_set(vendor_msg_dest_addr.mac_addr[1], vendor_msg_send_data,
			HM_VENDOR_MSG_SEND_CMD_MAC_ADDR1);
	field_ua_set(vendor_msg_dest_addr.mac_addr[2], vendor_msg_send_data,
			HM_VENDOR_MSG_SEND_CMD_MAC_ADDR2);
	field_ua_set(vendor_msg_dest_addr.mac_addr[3], vendor_msg_send_data,
			HM_VENDOR_MSG_SEND_CMD_MAC_ADDR3);
	field_ua_set(vendor_msg_dest_addr.mac_addr[4], vendor_msg_send_data,
			HM_VENDOR_MSG_SEND_CMD_MAC_ADDR4);
	field_ua_set(vendor_msg_dest_addr.mac_addr[5], vendor_msg_send_data,
			HM_VENDOR_MSG_SEND_CMD_MAC_ADDR5);
	memcpy((void *)FIELD_UA_BYTE_ADDRESS(vendor_msg_send_data,
			HM_VENDOR_MSG_SEND_CMD_MSG_DATA0),
			msg_payload->data, msg_payload->dataLength);
	retval = send_hostmsg_command((uint16_t)HM_VENDOR_MSG_SEND,
					HM_VENDOR_MSG_HDR_LEN +
					msg_payload->dataLength,
					vendor_msg_send_data,
					HM_SEND_VENDOR_MSG_RESPONSE_TIMEOUT,
					NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send Vendor Msg command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Send Vendor Msg successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SEND_VENDOR_MSG;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(vendor_msg_send_data);
	return retval;
}

static enum sii_os_status receive_remote_device_info(
			struct sii6400_remote_device *remote_device_info)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	uint16_t resp_data_mfg_id = 0;
	uint8_t type = 0;

	dbg("");

	if (NULL == remote_device_info) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_REMOTE_DEV_GET_INFO,
					0, NULL,
					HM_REMOTE_DEV_GET_INFO_RESPONSE_TIMEOUT,
					&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get Remote Device Info command not sent\n");
		goto done;
	}
	dbg("Get Remote Device Info successful\n");

	if (HM_REMOTE_DEV_GET_INFO_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto done;
	}

	memcpy(remote_device_info->mac_addr.mac_addr,
		(void *)FIELD_UA_BYTE_ADDRESS(resp_data,
				HM_REMOTE_DEV_GET_INFO_CMD_RSP_MAC_ADDR0),
		sizeof(remote_device_info->mac_addr.mac_addr));

	memcpy(remote_device_info->name.name,
		(void *)FIELD_UA_BYTE_ADDRESS(resp_data,
				HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_0),
		sizeof(remote_device_info->name.name) - 1);

	/* handle big endian packed MFG ID */
	resp_data_mfg_id = ((uint16_t)field_ua_extract(resp_data,
				HM_REMOTE_DEV_GET_INFO_CMD_RSP_MFG_MSB) << 8);
	resp_data_mfg_id |= (uint16_t)field_ua_extract(resp_data,
				HM_REMOTE_DEV_GET_INFO_CMD_RSP_MFG_LSB);
	remote_device_info->manufacturer.manufacturer[0] =
			((char)((resp_data_mfg_id >> 10) & 0x1f)) + 'A' - 1;
	remote_device_info->manufacturer.manufacturer[1] =
			((char)((resp_data_mfg_id >> 5) & 0x1f)) + 'A' - 1;
	remote_device_info->manufacturer.manufacturer[2] =
			((char)((resp_data_mfg_id >> 0) & 0x1f)) + 'A' - 1;

	remote_device_info->category.category =
			(enum sii6400_wihd_category)field_ua_extract(resp_data,
				HM_REMOTE_DEV_GET_INFO_CMD_RSP_CATEGORY);

	type = (uint8_t)field_ua_extract(resp_data,
					HM_REMOTE_DEV_GET_INFO_CMD_RSP_AV_TYPE);
	remote_device_info->type.video_source = type & 0x01;
	remote_device_info->type.video_sink = (type >> 1) & 0x01;
	remote_device_info->type.audio_source = (type >> 2) & 0x01;
	remote_device_info->type.audio_sink = (type >> 3) & 0x01;

done:
	SiiOsFree(resp_data);
	return retval;
}

static enum sii_os_status send_my_hdcp_policy(
				struct sii6400_hdcp_policy *hdcp_policy)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t *hdcp_policy_data = NULL;

	dbg("");

	if (NULL == hdcp_policy) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	hdcp_policy_data = SiiOsCalloc("SendHDCP_Policy",
				HM_HDCP_SET_POLICY_CMD_MLEN, 0);
	if (NULL == hdcp_policy_data) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	field_ua_set(hdcp_policy->policy, hdcp_policy_data,
			HM_HDCP_SET_POLICY_CMD_POLICY);

	retval = send_hostmsg_command((uint16_t)HM_HDCP_SET_POLICY,
					HM_HDCP_SET_POLICY_CMD_MLEN,
					hdcp_policy_data,
					HM_HDCP_SET_POLICY_RESPONSE_TIMEOUT,
					NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set HDCP Policy command not sent\n");
		goto done;
	}
	dbg("Set HDCP Policy successful\n");

done:
	SiiOsFree(hdcp_policy_data);
	return retval;
}

static enum sii_os_status send_my_hdcp_stream_type(
			struct sii6400_hdcp_stream_type *hdcp_stream_type)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t *hdcp_stream_type_data = NULL;

	dbg("");

	if (NULL == hdcp_stream_type) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	hdcp_stream_type_data = SiiOsCalloc("SendHDCPStreamType",
				HM_HDCP_SET_STREAM_TYPE_CMD_MLEN, 0);
	if (NULL == hdcp_stream_type_data) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	field_ua_set(hdcp_stream_type->stream_type, hdcp_stream_type_data,
			HM_HDCP_SET_STREAM_TYPE_CMD_STREAM_TYPE);

	retval = send_hostmsg_command((uint16_t)HM_HDCP_SET_STREAM_TYPE,
				HM_HDCP_SET_STREAM_TYPE_CMD_MLEN,
				hdcp_stream_type_data,
				HM_HDCP_SET_STREAM_TYPE_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set HDCP Stream Type command not sent\n");
		goto done;
	}
	dbg("Set HDCP Stream Type successful\n");

done:
	SiiOsFree(hdcp_stream_type_data);
	return retval;
}

/*
 * Format the WiHD UEvent reason string.
 */
static const char *
get_wihd_uevent_reason_string(enum SiI_Notification_Reason reason)
{
	const char *reason_str = NULL;

	switch (reason) {
	case HM_NOTIFY_REASON_NETWORK_ERROR:
		reason_str = "{\"reason\":\"network_error\"}";
		break;

	case HM_NOTIFY_REASON_REJECTED:
		reason_str = "{\"reason\":\"rejected\"}";
		break;

	case HM_NOTIFY_REASON_REQUESTED:
		reason_str = "{\"reason\":\"requested\"}";
		break;

	case HM_NOTIFY_REASON_HDCP:
		reason_str = "{\"reason\":\"hdcp\"}";
		break;

	case HM_NOTIFY_REASON_INPUT_ERROR:
		reason_str = "{\"reason\":\"input_error\"}";
		break;

	case HM_NOTIFY_REASON_TIMEOUT:
		reason_str = "{\"reason\":\"timeout\"}";
		break;

	case HM_NOTIFY_REASON_UNKNOWN:
	default:
		reason_str = "{\"reason\":\"unknown\"}";
		break;
	}

	return reason_str;
}

/*
 * Initialize the WiHD firmware when it is first loaded
 */
enum sii_os_status sii6400_wihd_init(struct sii6400_device_info *devinfo)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	enum sii_os_status rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	/* Force the state to "idle" */
	devinfo->wihd->state = WIHD_STATE_IDLE;
	dbg("WiHD state changed to idle\n");

	/* Set the MAC address, name, regulatory ID, HDCP Policy, and
	 * HDCP Streamn Type in the firmware */

	rv = send_my_hdcp_policy(&devinfo->my_hdcp_policy);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("Failed to send my HDCP Policy\n");
		retval = rv;
	}

	rv = send_my_hdcp_stream_type(&devinfo->my_hdcp_stream_type);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("Failed to send my HDCP Stream Type\n");
		retval = rv;
	}

done:
	return retval;
}

bool sii6400_wihd_sm_is_enabled(void)
{
	return wihd_sm_is_enabled;
}

void sii6400_wihd_sm_enable(void)
{
	enum sii_os_status rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == wvan_scan_interval_timer) {
		rv = SiiOsTimerCreate("wvan_scan_interval_timer",
					wvan_scan_interval_timer_function,
					NULL, false, 0, false,
					&wvan_scan_interval_timer);
		if (SII_OS_STATUS_SUCCESS != rv)
			err("Unable to create scan interval timer\n");
	}

	if ((0 != wihd_sm_info.wvan_scan_duration) &&
	    (NULL == wvan_scan_duration_timer)) {
		rv = SiiOsTimerCreate("wvan_scan_duration_timer",
					wvan_scan_duration_timer_function,
					NULL, false, 0, false,
					&wvan_scan_duration_timer);
		if (SII_OS_STATUS_SUCCESS != rv)
			err("Unable to create scan duration timer\n");
	}

	if (NULL == wvan_scan_response_timer) {
		rv = SiiOsTimerCreate("wvan_scan_response_timer",
					wvan_scan_response_timer_function,
					NULL, false, 0, false,
					&wvan_scan_response_timer);
		if (SII_OS_STATUS_SUCCESS != rv)
			err("Unable to create scan response timer\n");
	}

	/* Set WiHD parameters to startup state */
	wihd_sm_info.wvan_scan_enabled = false;
	wihd_sm_info.disable_scanning_pending = false;
	wihd_sm_info.restart_scanning_pending = false;
	wihd_sm_info.associate_pending = false;
	wihd_sm_info.wvan_info.wvan_id = 0;
	wihd_sm_info.wvan_info.hr_channel = 0;
	wihd_sm_info.wvan_info.lr_channel = 0;
	wihd_sm_info.next_wvan_info.wvan_id = 0;
	wihd_sm_info.next_wvan_info.hr_channel = 0;
	wihd_sm_info.next_wvan_info.lr_channel = 0;
	memset(&wihd_sm_info.remote_device, 0,
		sizeof(struct sii6400_remote_device));
	(void)SiiOsSemaphoreGive(wihd_sm_info.scan_sem);

	wihd_sm_is_enabled = true;
}

void sii6400_wihd_sm_disable(void)
{
	dbg("");

	wihd_sm_is_enabled = false;

	if (NULL != wvan_scan_duration_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(wvan_scan_duration_timer)) {
			err("Failed to delete scan duration timer\n");
		}
		wvan_scan_duration_timer = NULL;
	}

	if (NULL != wvan_scan_interval_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(wvan_scan_interval_timer)) {
			err("Failed to delete scan interval timer\n");
		}
		wvan_scan_interval_timer = NULL;
	}

	if (NULL != wvan_scan_response_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(wvan_scan_response_timer)) {
			err("Failed to delete scan response timer\n");
		}
		wvan_scan_response_timer = NULL;
	}

}

int sii6400_wihd_sm_init(void)
{
	int retval = 0;
	enum sii_os_status rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	memset(&wihd_sm_info, 0, sizeof(struct sii6400_wihd_sm_info));

	rv = SiiOsQueueCreate("get_wihd_chip_ver_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_wihd_chip_version_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_wihd_chip_version_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_wihd_fw_ver_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_wihd_firmware_version_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_wihd_firmware_version_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_wihd_id_impedance_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_wihd_id_impedance_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_wihd_id_impedance_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_rem_mac_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_remote_mac_addr_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_remote_mac_addr_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_rem_cat_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_remote_category_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_remote_category_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_rem_manu_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_remote_manufacturer_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_remote_manufacturer_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_rem_moni_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_remote_monitor_name_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_remote_monitor_name_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_rem_name_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_remote_name_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_remote_name_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_rem_type_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_remote_type_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_remote_type_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_sig_str_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_signal_strength_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_signal_strength_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mac_addr_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mac_addr_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mac_addr_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_name_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_name_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_name_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_name_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_name_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_name_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_scan_dur_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_wvan_scan_duration_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_wvan_scan_duration_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_scan_dur_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_wvan_scan_duration_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_wvan_scan_duration_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_scan_int_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_wvan_scan_interval_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_wvan_scan_interval_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_scan_int_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_wvan_scan_interval_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_wvan_scan_interval_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("scan_for_wvans_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&scan_for_wvans_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("scan_for_wvans_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_scan_stat_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_wvan_scan_status_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_wvan_scan_status_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("join_wvan_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&join_wvan_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("join_wvan_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("leave_wvan_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&leave_wvan_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("leave_wvan_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_wvan_info_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_wvan_info_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_wvan_info_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_wvan_connection_status_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_wvan_connection_status_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_wvan_connection_status_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_search_res_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_device_list_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_device_list_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("wihd_connect_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&wihd_connect_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("wihd_connect_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("wihd_disconnect_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&wihd_disconnect_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("wihd_disconnect_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("cec_send_message_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&cec_send_message_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("cec_send_message_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_wihd_rc_rcvd_ctrlcode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_wihd_rc_rcvd_ctrlcode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_wihd_rc_rcvd_ctrlcode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_wihd_rc_sent_ctrlcode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_wihd_rc_sent_ctrlcode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_wihd_rc_sent_ctrlcode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("send_wihd_rc_ctrlcode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&send_wihd_rc_ctrlcode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("send_wihd_rc_ctrlcode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_hdcp_policy_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_hdcp_policy_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_hdcp_policy_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_hdcp_policy_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_hdcp_policy_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_hdcp_policy_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_hdcp_stream_type_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_hdcp_stream_type_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_hdcp_stream_type_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_hdcp_stream_type_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_hdcp_stream_type_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_hdcp_stream_type_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("remote_fw_update_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_remote_fw_update_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_remote_fw_update_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_vendor_msg_recv_filter_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_vendor_msg_recv_filter_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_vendor_msg_recv_filter_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_vendor_msg_recv_filter_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_vendor_msg_recv_filter_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_vendor_msg_recv_filter_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_vendor_msg_vendor_id_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_vendor_msg_vendor_id_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_vendor_msg_vendor_id_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_vendor_msg_mac_addr_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_vendor_msg_mac_addr_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_vendor_msg_mac_addr_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("send_vendor_msg_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&send_vendor_msg_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("send_vendor_msg_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsSemaphoreCreate("wvan_scan_semaphore", 1, 1,
					&wihd_sm_info.scan_sem);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("Unable to create scan semaphore\n");
		retval = -EFAULT;
		goto done;
	}

done:
	return retval;
}

void sii6400_wihd_sm_exit(void)
{
	dbg("");

	if (NULL != get_wihd_chip_version_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_wihd_chip_version_resp_q)) {
			err("get_wihd_chip_version_resp_q delete failed\n");
		}
		get_wihd_chip_version_resp_q = NULL;
	}

	if (NULL != get_wihd_firmware_version_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_wihd_firmware_version_resp_q)) {
			err("get_wihd_firmware_version_resp_q delete failed\n");
		}
		get_wihd_firmware_version_resp_q = NULL;
	}

	if (NULL != get_wihd_id_impedance_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_wihd_id_impedance_resp_q)) {
			err("get_wihd_id_impedance_resp_q delete failed\n");
		}
		get_wihd_id_impedance_resp_q = NULL;
	}

	if (NULL != get_remote_mac_addr_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_remote_mac_addr_resp_q)) {
			err("get_remote_mac_addr_resp_q delete failed\n");
		}
		get_remote_mac_addr_resp_q = NULL;
	}

	if (NULL != get_remote_category_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_remote_category_resp_q)) {
			err("get_remote_category_resp_q delete failed\n");
		}
		get_remote_category_resp_q = NULL;
	}

	if (NULL != get_remote_manufacturer_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_remote_manufacturer_resp_q)) {
			err("get_remote_manufacturer_resp_q delete failed\n");
		}
		get_remote_manufacturer_resp_q = NULL;
	}

	if (NULL != get_remote_monitor_name_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_remote_monitor_name_resp_q)) {
			err("get_remote_monitor_name_resp_q delete failed\n");
		}
		get_remote_monitor_name_resp_q = NULL;
	}

	if (NULL != get_remote_name_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_remote_name_resp_q)) {
			err("get_remote_name_resp_q delete failed\n");
		}
		get_remote_name_resp_q = NULL;
	}

	if (NULL != get_remote_type_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_remote_type_resp_q)) {
			err("get_remote_type_resp_q delete failed\n");
		}
		get_remote_type_resp_q = NULL;
	}

	if (NULL != get_signal_strength_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_signal_strength_resp_q)) {
			err("get_signal_strength_resp_q delete failed\n");
		}
		get_signal_strength_resp_q = NULL;
	}

	if (NULL != get_mac_addr_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(get_mac_addr_resp_q)) {
			err("get_mac_addr_resp_q delete failed\n");
		}
		get_mac_addr_resp_q = NULL;
	}

	if (NULL != set_name_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
					SiiOsQueueDelete(set_name_resp_q)) {
			err("set_name_resp_q delete failed\n");
		}
		set_name_resp_q = NULL;
	}

	if (NULL != get_name_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
					SiiOsQueueDelete(get_name_resp_q)) {
			err("get_name_resp_q delete failed\n");
		}
		get_name_resp_q = NULL;
	}

	if (NULL != set_wvan_scan_duration_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_wvan_scan_duration_resp_q)) {
			err("set_wvan_scan_duration_resp_q delete failed\n");
		}
		set_wvan_scan_duration_resp_q = NULL;
	}

	if (NULL != get_wvan_scan_duration_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_wvan_scan_duration_resp_q)) {
			err("get_wvan_scan_duration_resp_q delete failed\n");
		}
		get_wvan_scan_duration_resp_q = NULL;
	}

	if (NULL != set_wvan_scan_interval_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_wvan_scan_interval_resp_q)) {
			err("set_wvan_scan_interval_resp_q delete failed\n");
		}
		set_wvan_scan_interval_resp_q = NULL;
	}

	if (NULL != get_wvan_scan_interval_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_wvan_scan_interval_resp_q)) {
			err("get_wvan_scan_interval_resp_q delete failed\n");
		}
		get_wvan_scan_interval_resp_q = NULL;
	}

	if (NULL != scan_for_wvans_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(scan_for_wvans_resp_q)) {
			err("scan_for_wvans_resp_q delete failed\n");
		}
		scan_for_wvans_resp_q = NULL;
	}

	if (NULL != get_wvan_scan_status_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_wvan_scan_status_resp_q)) {
			err("get_wvan_scan_status_resp_q delete failed\n");
		}
		get_wvan_scan_status_resp_q = NULL;
	}

	if (NULL != join_wvan_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(join_wvan_resp_q)) {
			err("join_wvan_resp_q delete failed\n");
		}
		join_wvan_resp_q = NULL;
	}

	if (NULL != leave_wvan_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(leave_wvan_resp_q)) {
			err("leave_wvan_resp_q delete failed\n");
		}
		leave_wvan_resp_q = NULL;
	}

	if (NULL != get_wvan_info_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(get_wvan_info_resp_q)) {
			err("get_wvan_info_resp_q delete failed\n");
		}
		get_wvan_info_resp_q = NULL;
	}

	if (NULL != get_wvan_connection_status_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_wvan_connection_status_resp_q)) {
			err("get_wvan_connection_status_resp_q del failed\n");
		}
		get_wvan_connection_status_resp_q = NULL;
	}

	if (NULL != get_device_list_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(get_device_list_resp_q)) {
			err("get_device_list_resp_q delete failed\n");
		}
		get_device_list_resp_q = NULL;
	}

	if (NULL != wihd_connect_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(wihd_connect_resp_q)) {
			err("wihd_connect_resp_q delete failed\n");
		}
		wihd_connect_resp_q = NULL;
	}

	if (NULL != wihd_disconnect_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(wihd_disconnect_resp_q)) {
			err("wihd_disconnect_resp_q delete failed\n");
		}
		wihd_disconnect_resp_q = NULL;
	}

	if (NULL != cec_send_message_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(cec_send_message_resp_q)) {
			err("cec_send_message_resp_q delete failed\n");
		}
		cec_send_message_resp_q = NULL;
	}

	if (NULL != get_wihd_rc_rcvd_ctrlcode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_wihd_rc_rcvd_ctrlcode_resp_q)) {
			err("get_wihd_rc_rcvd_ctrlcode_resp_q delete failed\n");
		}
		get_wihd_rc_rcvd_ctrlcode_resp_q = NULL;
	}

	if (NULL != get_wihd_rc_sent_ctrlcode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_wihd_rc_sent_ctrlcode_resp_q)) {
			err("get_wihd_rc_sent_ctrlcode_resp_q delete failed\n");
		}
		get_wihd_rc_sent_ctrlcode_resp_q = NULL;
	}

	if (NULL != send_wihd_rc_ctrlcode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(send_wihd_rc_ctrlcode_resp_q)) {
			err("send_wihd_rc_ctrlcode_resp_q delete failed\n");
		}
		send_wihd_rc_ctrlcode_resp_q = NULL;
	}

	if (NULL != set_hdcp_policy_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(set_hdcp_policy_resp_q)) {
			err("set_hdcp_policy_resp_q delete failed\n");
		}
		set_hdcp_policy_resp_q = NULL;
	}

	if (NULL != get_hdcp_policy_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(get_hdcp_policy_resp_q)) {
			err("get_hdcp_policy_resp_q delete failed\n");
		}
		get_hdcp_policy_resp_q = NULL;
	}

	if (NULL != set_hdcp_stream_type_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_hdcp_stream_type_resp_q)) {
			err("set_hdcp_stream_type_resp_q delete failed\n");
		}
		set_hdcp_stream_type_resp_q = NULL;
	}

	if (NULL != get_hdcp_stream_type_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_hdcp_stream_type_resp_q)) {
			err("get_hdcp_stream_type_resp_q delete failed\n");
		}
		get_hdcp_stream_type_resp_q = NULL;
	}

	if (NULL != set_remote_fw_update_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_remote_fw_update_resp_q)) {
			err("set_remote_fw_update_resp_q delete failed\n");
		}
		set_remote_fw_update_resp_q = NULL;
	}

	if (NULL != get_vendor_msg_recv_filter_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_vendor_msg_recv_filter_resp_q)) {
			err("get_vendor_msg_recv_filter_resp_q del failed\n");
		}
		get_vendor_msg_recv_filter_resp_q = NULL;
	}

	if (NULL != set_vendor_msg_recv_filter_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_vendor_msg_recv_filter_resp_q)) {
			err("set_vendor_msg_recv_filter_resp_q del failed\n");
		}
		set_vendor_msg_recv_filter_resp_q = NULL;
	}

	if (NULL != set_vendor_msg_vendor_id_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_vendor_msg_vendor_id_resp_q)) {
			err("set_vendor_msg_vendor_id_resp_q delete failed\n");
		}
		set_vendor_msg_recv_filter_resp_q = NULL;
	}

	if (NULL != set_vendor_msg_mac_addr_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_vendor_msg_mac_addr_resp_q)) {
			err("set_vendor_msg_mac_addr_resp_q delete failed\n");
		}
		set_vendor_msg_recv_filter_resp_q = NULL;
	}

	if (NULL != send_vendor_msg_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(send_vendor_msg_resp_q)) {
			err("send_vendor_msg_resp_q delete failed\n");
		}
		send_vendor_msg_resp_q = NULL;
	}

	if (NULL != wvan_scan_interval_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(wvan_scan_interval_timer)) {
			err("Failed to delete scan interval timer\n");
		}
		wvan_scan_interval_timer = NULL;
	}

	if (NULL != wvan_scan_duration_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(wvan_scan_duration_timer)) {
			err("Failed to delete scan duration timer\n");
		}
		wvan_scan_duration_timer = NULL;
	}

	if (NULL != wvan_scan_response_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(wvan_scan_response_timer)) {
			err("Failed to delete scan response timer\n");
		}
		wvan_scan_response_timer = NULL;
	}

	if (NULL != wihd_sm_info.scan_sem) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsSemaphoreDelete(wihd_sm_info.scan_sem)) {
			err("Failed to delete scan semaphore\n");
		}
		wihd_sm_info.scan_sem = NULL;
	}
}

