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
#include <linux/delay.h>

#ifdef ENABLE_MHL_UEVENT
#include <linux/power/mhl_uevent.h>
#endif

#include "osal.h"
#include "state_machine.h"
#include "mhl_sm.h"
#include "sii6400.h"
#include "host_msg.h"
#include "rcp_inputdev.h"

struct SiiOsQueue *get_mhl_chip_version_resp_q;
struct SiiOsQueue *get_mhl_firmware_version_resp_q;
struct SiiOsQueue *get_mhl_connection_state_resp_q;
struct SiiOsQueue *get_mhl_id_impedance_resp_q;
struct SiiOsQueue *set_mhl_clock_swing_resp_q;
struct SiiOsQueue *get_mhl_clock_swing_resp_q;
struct SiiOsQueue *set_mhl_cbus_voltage_resp_q;
struct SiiOsQueue *set_mhl_local_devcap_resp_q;
struct SiiOsQueue *get_mhl_local_devcap_resp_q;
struct SiiOsQueue *set_mhl_local_devcap_offset_resp_q;
struct SiiOsQueue *get_mhl_local_devcap_offset_resp_q;
struct SiiOsQueue *get_mhl_remote_devcap_resp_q;
struct SiiOsQueue *set_mhl_remote_devcap_offset_resp_q;
struct SiiOsQueue *get_mhl_remote_devcap_offset_resp_q;
struct SiiOsQueue *send_mhl_ucp_charcode_resp_q;
struct SiiOsQueue *get_mhl_ucp_rcvd_charcode_resp_q;
struct SiiOsQueue *get_mhl_ucp_sent_charcode_resp_q;
struct SiiOsQueue *send_mhl_ucp_ack_resp_q;
struct SiiOsQueue *get_mhl_ucp_ack_resp_q;
struct SiiOsQueue *send_mhl_rcp_keycode_resp_q;
struct SiiOsQueue *get_mhl_rcp_rcvd_keycode_resp_q;
struct SiiOsQueue *get_mhl_rcp_sent_keycode_resp_q;
struct SiiOsQueue *send_mhl_rcp_ack_resp_q;
struct SiiOsQueue *get_mhl_rcp_ack_resp_q;
struct SiiOsQueue *send_mhl_rap_actioncode_resp_q;
struct SiiOsQueue *get_mhl_rap_rcvd_actioncode_resp_q;
struct SiiOsQueue *get_mhl_rap_sent_actioncode_resp_q;
struct SiiOsQueue *send_mhl_rap_ack_resp_q;
struct SiiOsQueue *get_mhl_rap_ack_resp_q;

static struct sii6400_mhl_sm_info mhl_sm_info;

static bool mhl_sm_is_enabled;
static bool is_armet;

static enum sii_os_status send_rap_ack_or_nak(uint8_t action_code,
						uint8_t status);
static enum sii_os_status send_ucp_ack_or_nak(uint8_t charcode,
						uint8_t status);
static enum sii_os_status send_rcp_ack_or_nak(uint8_t keycode,
						uint8_t status);

static void auto_get_dcap(struct work_struct *work);

static DECLARE_WORK(auto_get_dcap_work, auto_get_dcap);


static void auto_get_dcap(struct work_struct *work)
{
	struct sii6400_mhl_devcap_offset *remote_devcap_offset = NULL;
	enum sii_os_status sm_rv = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_devcap *devcap = NULL;
	void *resp_data = NULL;
	uint32_t resp_data_size = 0;

	err("auto_get_dcap, otification Received\n");

	/***********here set dev cap offset********************/
	/* This memory is freed in the state machine. */
	remote_devcap_offset = SiiOsCalloc("MHLRemoteDevCapOffsetAuto",
				sizeof(*remote_devcap_offset), 0);

	if (NULL == remote_devcap_offset) {
		err("Out of memory\n");
		return;
	}

	remote_devcap_offset->offset = 0x0F; // here set 0x0F is for armlet judgement
	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_SET_MHL_REMOTE_DEVCAP_OFFSET,
				remote_devcap_offset, sizeof(*remote_devcap_offset),
				set_mhl_remote_devcap_offset_resp_q,
				WAIT_TIME_FOR_SET_MHL_REMOTE_DEVCAP_OFFSET_RESPONSE,
				NULL, NULL);

	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		return;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		return;
	}
	/***********here read remote dev cap of 0x0F********************/
	sm_rv = sm_send_request(mhl_sm_queue,
				SII_SM_REQ_GET_MHL_REMOTE_DEVCAP,
				NULL, 0,
				get_mhl_remote_devcap_resp_q,
				WAIT_TIME_FOR_GET_MHL_REMOTE_DEVCAP_RESPONSE,
				&resp_data, &resp_data_size);
	if (SII_OS_STATUS_TIMEOUT == sm_rv) {
		err("Timed out waiting for MHL State Machine response\n");
		return;
	} else if (SII_OS_STATUS_ERR_NOT_AVAIL == sm_rv) {
		err("MHL Remote Device Capablities not available\n");
		return;
	} else if (SII_OS_STATUS_SUCCESS != sm_rv) {
		err("MHL State Machine request send failure\n");
		return;
	}

	if ((NULL == resp_data) ||
		(sizeof(*devcap) != resp_data_size)) {
		err("Invalid MHL State Machine response\n");
		return;
	}
	devcap = resp_data;

	if (NULL != devcap && devcap->data[0] == 0xba) {
		is_armet = true;
		err("The mhl sink device is armet.\n");
	} else
		is_armet = false;

	SiiOsFree(resp_data);
	/***********here read remote dev cap of 0x0F********************/
	return;
}

bool mhl_is_connected(enum sii6400_mhl_state state)
{
	return (MHL_STATE_CONNECTED == state) ? true : false;
}

enum sii_os_status get_mhl_chip_version(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_chip_version *chip_version = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
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

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
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

enum sii_os_status get_mhl_firmware_version(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_firmware_version *firmware_version = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
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

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
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

enum sii_os_status get_mhl_dev_type(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_dev_type *dev_type = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	/* This memory is freed by the response handler */
	dev_type = SiiOsCalloc("GetMHLDevType",
					sizeof(*dev_type), 0);
	if (NULL == dev_type) {
		err("Out of memory\n");
		goto send_response;
	}

	if (is_armet)
		dev_type->type = MHL_DEV_TYPE_ARMET;
	else
		dev_type->type = MHL_DEV_TYPE_TV;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_DEV_TYPE;
	resp.resp_status = resp_status;
	resp.data = dev_type;
	resp.data_size = (NULL != dev_type) ? sizeof(*dev_type) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(dev_type);
	}

	return retval;
}

enum sii_os_status get_mhl_connection_state(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_connection_state *connect_state = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t state = 0;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_MHL_GET_CONNECTION_STATE,
				0, NULL,
				HM_MHL_GET_CONNECTION_STATE_RESPONSE_TIMEOUT,
				&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get MHL Connection State command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get MHL Connection State successful\n");

	if (HM_MHL_GET_CONNECTION_STATE_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	connect_state = SiiOsCalloc("GetMHLConnectState",
					sizeof(*connect_state), 0);
	if (NULL == connect_state) {
		err("Out of memory\n");
		goto send_response;
	}

	connect_state->connect_state = MHL_STATE_NONE;

	state = (uint8_t)field_ua_extract(resp_data,
			HM_MHL_GET_CONNECTION_STATE_CMD_RSP_CONNECT_STATE);
	if (1 == state)
		connect_state->connect_state = MHL_STATE_DISCONNECTED;
	else if (2 == state)
		connect_state->connect_state = MHL_STATE_CONNECTED;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_CONNECTION_STATE;
	resp.resp_status = resp_status;
	resp.data = connect_state;
	resp.data_size = (NULL != connect_state) ? sizeof(*connect_state) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(connect_state);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	return retval;
}

enum sii_os_status get_mhl_id_impedance(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_id_impedance *id_impedance = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_MHL_MEASURE_ID_IMPEDANCE,
				0, NULL,
				HM_MHL_MEASURE_ID_IMPEDANCE_RESPONSE_TIMEOUT,
				&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get MHL ID Impedance command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get MHL ID Impedance successful\n");

	if (HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP_MLEN != resp_data_size) {
		warn("Unexpected response data size %d\n", resp_data_size);
		goto send_response;
	}

	/* This memory is freed by the response handler */
	id_impedance = SiiOsCalloc("GetMHLIDImpedance",
					sizeof(*id_impedance), 0);
	if (NULL == id_impedance) {
		err("Out of memory\n");
		goto send_response;
	}

	id_impedance->ohms = (uint32_t)field_ua_extract(resp_data,
				HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP_IMPEDANCE);

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_ID_IMPEDANCE;
	resp.resp_status = resp_status;
	resp.data = id_impedance;
	resp.data_size = (NULL != id_impedance) ? sizeof(*id_impedance) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
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

enum sii_os_status set_mhl_cbus_voltage(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_cbus_voltage *cbus_voltage = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *cbus_voltage_data = NULL;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	cbus_voltage = req_data;

	if (sizeof(*cbus_voltage) != req_data_size) {
		err("Illegal data for Set Cbus Voltage request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	cbus_voltage_data = SiiOsCalloc("SetMHLCbusVoltage",
				HM_MHL_SET_CBUS_VOLTAGE_CMD_MLEN, 0);
	if (NULL == cbus_voltage_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(cbus_voltage->high, cbus_voltage_data,
				HM_MHL_SET_CBUS_VOLTAGE_CMD_VALUE);
	field_ua_set(cbus_voltage->peroid & 0xFF, cbus_voltage_data,
				HM_MHL_SET_CBUS_VOLTAGE_CMD_PERIOD_LO);
	field_ua_set((cbus_voltage->peroid >> 8) & 0xFF, cbus_voltage_data,
				HM_MHL_SET_CBUS_VOLTAGE_CMD_PERIOD_HI);

	retval = send_hostmsg_command((uint16_t)HM_MHL_SET_CBUS_VOLTAGE,
				HM_MHL_SET_CBUS_VOLTAGE_CMD_MLEN, cbus_voltage_data,
				HM_MHL_SET_CBUS_VOLTAGE_RESPONSE_TIMEOUT,
				NULL, NULL);

	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set MHL Cbus Voltage command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Set MHL Cbus Voltage successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SET_MHL_CBUS_VOLTAGE;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(cbus_voltage_data);
	return retval;
}

enum sii_os_status set_mhl_clock_swing(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_clock_swing *clock_swing = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *clock_value = NULL;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	clock_swing = req_data;

	if (sizeof(*clock_swing) != req_data_size) {
		err("Illegal data for Set MHL Clock Swing request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	clock_value = SiiOsCalloc("SetMHLClockSwing",
				HM_MHL_SET_CLOCK_SWING_CMD_MLEN, 0);
	if (NULL == clock_value) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(clock_swing->clock_swing, clock_value,
				HM_MHL_SET_CLOCK_SWING_CMD_VALUE);

	retval = send_hostmsg_command((uint16_t)HM_MHL_SET_CLOCK_SWING,
				HM_MHL_SET_CLOCK_SWING_CMD_MLEN, clock_value,
				HM_MHL_SET_CLOCK_SWING_RESPONSE_TIMEOUT,
				NULL, NULL);

	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set MHL Clock Swing command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Set MHL Clock Swing successful\n");

	mhl_sm_info.swing_set = clock_swing->clock_swing;

send_response:
	resp.request_type = SII_SM_REQ_SET_MHL_CLOCK_SWING;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(clock_value);
	return retval;
}

enum sii_os_status get_mhl_clock_swing(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_clock_swing *clock_swing = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	clock_swing = SiiOsCalloc("GetMHLClockSwing",
				sizeof(*clock_swing), 0);
	if (NULL == clock_swing) {
		err("Out of memory\n");
		goto send_response;
	}

	clock_swing->clock_swing = mhl_sm_info.swing_set;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_CLOCK_SWING;
	resp.resp_status = resp_status;
	resp.data = clock_swing;
	resp.data_size = (NULL != clock_swing) ? sizeof(*clock_swing) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(clock_swing);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status set_mhl_local_devcap(struct SiiOsQueue *resp_q,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_devcap *local_devcap = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *local_devcap_data = NULL;
	uint8_t offset = mhl_sm_info.local_devcap_offset;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	local_devcap = req_data;

	if (sizeof(*local_devcap) != req_data_size) {
		err("Illegal Set MHL Local Device Capablities request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	if ((MAX_MHL_DEVCAP_SIZE < local_devcap->length) ||
	    (HM_MHL_DEVCAP_LOCAL_WRITE_CMD_DATALEN_MAX <
				local_devcap->length)) {
		err("Device Capability Length exceeds maximum allowed\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	if (HM_MHL_DEVCAP_MAX_CAPABILITY_OFFSET < offset) {
		err("Device Capability Offset exceeds maximum allowed\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	local_devcap_data = SiiOsCalloc("SetLocalDevCap",
		HM_MHL_DEVCAP_LOCAL_WRITE_CMD_MLEN_MIN + local_devcap->length,
		0);
	if (NULL == local_devcap_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(offset, local_devcap_data,
			HM_MHL_DEVCAP_LOCAL_WRITE_CMD_OFFSET);
	memcpy(local_devcap_data + HM_MHL_DEVCAP_LOCAL_WRITE_CMD_MLEN_MIN,
		local_devcap->data, local_devcap->length);

	retval = send_hostmsg_command((uint16_t)HM_MHL_DEVCAP_LOCAL_WRITE,
		HM_MHL_DEVCAP_LOCAL_WRITE_CMD_MLEN_MIN + local_devcap->length,
		local_devcap_data,
		HM_MHL_DEVCAP_LOCAL_WRITE_RESPONSE_TIMEOUT,
		NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Write MHL Local Device Capabilities command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Write MHL Local Device Capabilities successful\n");

#if 0
	/* Update local copy of data */
	if (offset < MAX_MHL_DEVCAP_SIZE) {
		memcpy(&mhl_sm_info.local_devcap[offset],
			local_devcap->data,
			(local_devcap->length < MAX_MHL_DEVCAP_SIZE - offset) ?
					local_devcap->length :
					MAX_MHL_DEVCAP_SIZE - offset);
	} else {
		warn("Local DevCap offset too large: %u\n", offset);
	}
#endif

send_response:
	resp.request_type = SII_SM_REQ_SET_MHL_LOCAL_DEVCAP;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(local_devcap_data);
	return retval;
}

enum sii_os_status get_mhl_local_devcap(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_devcap *local_devcap = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *local_devcap_data = NULL;
	int data_size = 0;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	local_devcap_data = SiiOsCalloc("LocalDevCapData",
					HM_MHL_DEVCAP_LOCAL_READ_CMD_MLEN, 0);
	if (NULL == local_devcap_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(mhl_sm_info.local_devcap_offset, local_devcap_data,
			HM_MHL_DEVCAP_LOCAL_READ_CMD_OFFSET);
	field_ua_set(1, local_devcap_data,
			HM_MHL_DEVCAP_LOCAL_READ_CMD_DATALEN);

	retval = send_hostmsg_command((uint16_t)HM_MHL_DEVCAP_LOCAL_READ,
				HM_MHL_DEVCAP_LOCAL_READ_CMD_MLEN,
				local_devcap_data,
				HM_MHL_DEVCAP_LOCAL_READ_RESPONSE_TIMEOUT,
				&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get MHL Local Device Capabilities command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get MHL Local Device Capabilities successful\n");

	dbg("resp_data_size = %u\n", resp_data_size);

	if (HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_MLEN_MAX < resp_data_size) {
		dbg("Local devcap data size too large (%d)\n",
		    resp_data_size);
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (resp_data_size < HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_MLEN_MIN) {
		dbg("Local devcap data size too small (%d)\n",
		    resp_data_size);
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	local_devcap = SiiOsCalloc("GetMHLLocalDevCap",
					sizeof(*local_devcap), 0);
	if (NULL == local_devcap) {
		err("Out of memory\n");
		goto send_response;
	}

	data_size = resp_data_size - HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_MLEN_MIN;
	if (MAX_MHL_DEVCAP_SIZE < data_size)
		data_size = MAX_MHL_DEVCAP_SIZE;
	if (0 < data_size) {
		local_devcap->offset = mhl_sm_info.local_devcap_offset;
		local_devcap->length = data_size;
		memcpy(local_devcap->data,
			(uint8_t *)resp_data +
				HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_MLEN_MIN,
			data_size);
	}

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_LOCAL_DEVCAP;
	resp.resp_status = resp_status;
	resp.data = local_devcap;
	resp.data_size = (NULL != local_devcap) ? sizeof(*local_devcap) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(local_devcap);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	SiiOsFree(local_devcap_data);
	return retval;
}

enum sii_os_status set_mhl_local_devcap_offset(struct SiiOsQueue *resp_q,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_devcap_offset *local_devcap_offset = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	local_devcap_offset = req_data;

	if (sizeof(*local_devcap_offset) != req_data_size) {
		err("Illegal Set MHL Local Device Cap Offset request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	mhl_sm_info.local_devcap_offset = local_devcap_offset->offset;

send_response:
	resp.request_type = SII_SM_REQ_SET_MHL_LOCAL_DEVCAP_OFFSET;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_mhl_local_devcap_offset(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_devcap_offset *local_devcap_offset = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* This memory is freed by the response handler */
	local_devcap_offset = SiiOsCalloc("GetMHLLocalDevCapOffset",
					sizeof(*local_devcap_offset), 0);
	if (NULL == local_devcap_offset) {
		err("Out of memory\n");
		goto send_response;
	}

	local_devcap_offset->offset = mhl_sm_info.local_devcap_offset;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_LOCAL_DEVCAP_OFFSET;
	resp.resp_status = resp_status;
	resp.data = local_devcap_offset;
	resp.data_size = (NULL != local_devcap_offset) ?
				sizeof(*local_devcap_offset) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(local_devcap_offset);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_mhl_remote_devcap(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_devcap *remote_devcap = NULL;
	struct sii6400_device_info *devinfo = NULL;
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *remote_devcap_data = NULL;
	int data_size = 0;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL Remote Device Cap not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	remote_devcap_data = SiiOsCalloc("RemoteDevCapData",
					HM_MHL_DEVCAP_REMOTE_READ_CMD_MLEN, 0);
	if (NULL == remote_devcap_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(mhl_sm_info.remote_devcap_offset, remote_devcap_data,
			HM_MHL_DEVCAP_REMOTE_READ_CMD_OFFSET);
	field_ua_set(1, remote_devcap_data,
			HM_MHL_DEVCAP_REMOTE_READ_CMD_DATALEN);

	retval = send_hostmsg_command((uint16_t)HM_MHL_DEVCAP_REMOTE_READ,
				HM_MHL_DEVCAP_REMOTE_READ_CMD_MLEN,
				remote_devcap_data,
				HM_MHL_DEVCAP_REMOTE_READ_RESPONSE_TIMEOUT,
				&resp_data_size, &resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Get MHL Remote Device Capabilities command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Get MHL Remote Device Capabilities successful\n");

	dbg("resp_data_size = %u\n", resp_data_size);

	if (HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP_MLEN_MAX < resp_data_size) {
		dbg("Remote devcap data size too large (%d)\n",
		    resp_data_size);
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (resp_data_size < HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP_MLEN_MIN) {
		dbg("Remote devcap data size too small (%d)\n",
		    resp_data_size);
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	remote_devcap = SiiOsCalloc("GetMHLRemoteDevCap",
					sizeof(*remote_devcap), 0);
	if (NULL == remote_devcap) {
		err("Out of memory\n");
		goto send_response;
	}

	data_size = resp_data_size - HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP_MLEN_MIN;
	if (MAX_MHL_DEVCAP_SIZE < data_size)
		data_size = MAX_MHL_DEVCAP_SIZE;
	if (0 < data_size) {
		remote_devcap->offset = mhl_sm_info.remote_devcap_offset;
		remote_devcap->length = data_size;
		memcpy(remote_devcap->data,
			(uint8_t *)resp_data +
				HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP_MLEN_MIN,
			data_size);
	}

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_REMOTE_DEVCAP;
	resp.resp_status = resp_status;
	resp.data = remote_devcap;
	resp.data_size = (NULL != remote_devcap) ? sizeof(*remote_devcap) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(remote_devcap);
		goto done;
	}

done:
	SiiOsFree(resp_data);
	SiiOsFree(remote_devcap_data);
	return retval;
}

enum sii_os_status set_mhl_remote_devcap_offset(struct SiiOsQueue *resp_q,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_devcap_offset *remote_devcap_offset = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	remote_devcap_offset = req_data;

	if (sizeof(*remote_devcap_offset) != req_data_size) {
		err("Illegal Set MHL Remote Device Cap Offset request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	mhl_sm_info.remote_devcap_offset = remote_devcap_offset->offset;

send_response:
	resp.request_type = SII_SM_REQ_SET_MHL_REMOTE_DEVCAP_OFFSET;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_mhl_remote_devcap_offset(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_devcap_offset *remote_devcap_offset = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* This memory is freed by the response handler */
	remote_devcap_offset = SiiOsCalloc("GetMHLRemoteDevCapOffset",
					sizeof(*remote_devcap_offset), 0);
	if (NULL == remote_devcap_offset) {
		err("Out of memory\n");
		goto send_response;
	}

	remote_devcap_offset->offset = mhl_sm_info.remote_devcap_offset;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_REMOTE_DEVCAP_OFFSET;
	resp.resp_status = resp_status;
	resp.data = remote_devcap_offset;
	resp.data_size = (NULL != remote_devcap_offset) ?
				sizeof(*remote_devcap_offset) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(remote_devcap_offset);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status send_mhl_ucp_charcode(struct SiiOsQueue *resp_q,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_ucp *ucp = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *ucp_data = NULL;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	ucp = req_data;

	if (sizeof(*ucp) != req_data_size) {
		err("Illegal data for Send MHL UCP Charcode request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL UCP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	ucp_data = SiiOsCalloc("SendMHLUCPCharcodeData",
				HM_MHL_SEND_UCP_CHAR_CMD_MLEN, 0);
	if (NULL == ucp_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(ucp->charcode, ucp_data, HM_MHL_SEND_UCP_CHAR_CMD_CHAR);

	retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_UCP_CHAR,
				HM_MHL_SEND_UCP_CHAR_CMD_MLEN, ucp_data,
				HM_MHL_SEND_UCP_CHAR_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send MHL UCP Charcode command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Send MHL UCP Charcode successful\n");

	mhl_sm_info.ucp_sent = ucp->charcode;
	mhl_sm_info.ucp_sent_status = 0;

send_response:
	resp.request_type = SII_SM_REQ_SEND_MHL_UCP_CHARCODE;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(ucp_data);
	return retval;
}

enum sii_os_status get_mhl_ucp_rcvd_charcode(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_ucp *ucp = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL UCP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	ucp = SiiOsCalloc("GetMHLUCPRcvdChar", sizeof(*ucp), 0);
	if (NULL == ucp) {
		err("Out of memory\n");
		goto send_response;
	}

	ucp->charcode = mhl_sm_info.ucp_received;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_UCP_RCVD_CHARCODE;
	resp.resp_status = resp_status;
	resp.data = ucp;
	resp.data_size = (NULL != ucp) ? sizeof(*ucp) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(ucp);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_mhl_ucp_sent_charcode(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_ucp *ucp = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL UCP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	ucp = SiiOsCalloc("GetMHLUCPSentChar", sizeof(*ucp), 0);
	if (NULL == ucp) {
		err("Out of memory\n");
		goto send_response;
	}

	ucp->charcode = mhl_sm_info.ucp_sent;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_UCP_SENT_CHARCODE;
	resp.resp_status = resp_status;
	resp.data = ucp;
	resp.data_size = (NULL != ucp) ? sizeof(*ucp) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(ucp);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status send_mhl_ucp_ack(struct SiiOsQueue *resp_q,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_ucp_status *ucp_ack = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *ucp_ack_data = NULL;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	ucp_ack = req_data;

	if (sizeof(*ucp_ack) != req_data_size) {
		err("Illegal data for Send MHL UCP ACK request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL UCP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	ucp_ack_data = SiiOsCalloc("SendMHLUCPACKData",
					HM_MHL_SEND_UCP_ACK_CMD_MLEN, 0);
	if (NULL == ucp_ack_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (ucp_ack->status) {
		field_ua_set(1, ucp_ack_data, HM_MHL_SEND_UCP_ACK_CMD_ACK);
		field_ua_set(ucp_ack->status, ucp_ack_data,
				HM_MHL_SEND_UCP_ACK_CMD_ERR);
	} else {
		field_ua_set(0, ucp_ack_data, HM_MHL_SEND_UCP_ACK_CMD_ACK);
		field_ua_set(mhl_sm_info.ucp_received, ucp_ack_data,
				HM_MHL_SEND_UCP_ACK_CMD_ERR);
	}

	retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_UCP_ACK,
				HM_MHL_SEND_UCP_ACK_CMD_MLEN, ucp_ack_data,
				HM_MHL_SEND_UCP_ACK_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send MHL UCP ACK/NAK command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Send MHL UCP ACK/NAK successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SEND_MHL_UCP_ACK;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(ucp_ack_data);
	return retval;
}

enum sii_os_status get_mhl_ucp_ack(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_ucp_status *ucp_ack = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL UCP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	ucp_ack = SiiOsCalloc("GetMHLUCPACK", sizeof(*ucp_ack), 0);
	if (NULL == ucp_ack) {
		err("Out of memory\n");
		goto send_response;
	}

	ucp_ack->status = mhl_sm_info.ucp_sent_status;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_UCP_ACK;
	resp.resp_status = resp_status;
	resp.data = ucp_ack;
	resp.data_size = (NULL != ucp_ack) ? sizeof(*ucp_ack) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(ucp_ack);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status send_mhl_rcp_keycode(struct SiiOsQueue *resp_q,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_rcp *rcp = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *rcp_data = NULL;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	rcp = req_data;

	if (sizeof(*rcp) != req_data_size) {
		err("Illegal data for Send MHL RCP Keycode request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL RCP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	rcp_data = SiiOsCalloc("SendMHLRCPKeycodeData",
				HM_MHL_SEND_RCP_KEY_CMD_MLEN, 0);
	if (NULL == rcp_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(rcp->keycode, rcp_data, HM_MHL_SEND_RCP_KEY_CMD_KEY);

	retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_RCP_KEY,
				HM_MHL_SEND_RCP_KEY_CMD_MLEN, rcp_data,
				HM_MHL_SEND_RCP_KEY_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send MHL RCP Keycode command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Send MHL RCP Keycode successful\n");

	mhl_sm_info.rcp_sent = rcp->keycode;
	mhl_sm_info.rcp_sent_status = 0;

send_response:
	resp.request_type = SII_SM_REQ_SEND_MHL_RCP_KEYCODE;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(rcp_data);
	return retval;
}

enum sii_os_status get_mhl_rcp_rcvd_keycode(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_rcp *rcp = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL RCP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	rcp = SiiOsCalloc("GetMHLRCPRcvdKey", sizeof(*rcp), 0);
	if (NULL == rcp) {
		err("Out of memory\n");
		goto send_response;
	}

	rcp->keycode = mhl_sm_info.rcp_received;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_RCP_RCVD_KEYCODE;
	resp.resp_status = resp_status;
	resp.data = rcp;
	resp.data_size = (NULL != rcp) ? sizeof(*rcp) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(rcp);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_mhl_rcp_sent_keycode(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_rcp *rcp = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL RCP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	rcp = SiiOsCalloc("GetMHLRCPSentKey", sizeof(*rcp), 0);
	if (NULL == rcp) {
		err("Out of memory\n");
		goto send_response;
	}

	rcp->keycode = mhl_sm_info.rcp_sent;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_RCP_SENT_KEYCODE;
	resp.resp_status = resp_status;
	resp.data = rcp;
	resp.data_size = (NULL != rcp) ? sizeof(*rcp) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(rcp);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status send_mhl_rcp_ack(struct SiiOsQueue *resp_q,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_rcp_status *rcp_ack = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *rcp_ack_data = NULL;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	rcp_ack = req_data;

	if (sizeof(*rcp_ack) != req_data_size) {
		err("Illegal data for Send MHL RCP ACK request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL RCP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	rcp_ack_data = SiiOsCalloc("SendMHLRCPACKData",
					HM_MHL_SEND_RCP_ACK_CMD_MLEN, 0);
	if (NULL == rcp_ack_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (RCPE_NO_ERROR != rcp_ack->status) {
		field_ua_set(1, rcp_ack_data, HM_MHL_SEND_RCP_ACK_CMD_ACK);
		field_ua_set(rcp_ack->status, rcp_ack_data,
				HM_MHL_SEND_RCP_ACK_CMD_ERR);
	} else {
		field_ua_set(0, rcp_ack_data, HM_MHL_SEND_RCP_ACK_CMD_ACK);
		field_ua_set(mhl_sm_info.rcp_received, rcp_ack_data,
				HM_MHL_SEND_RCP_ACK_CMD_ERR);
	}

	retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_RCP_ACK,
				HM_MHL_SEND_RCP_ACK_CMD_MLEN, rcp_ack_data,
				HM_MHL_SEND_RCP_ACK_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send MHL RCP ACK/NAK command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Send MHL RCP ACK/NAK successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SEND_MHL_RCP_ACK;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(rcp_ack_data);
	return retval;
}

enum sii_os_status get_mhl_rcp_ack(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_rcp_status *rcp_ack = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL RCP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	rcp_ack = SiiOsCalloc("GetMHLRCPACK", sizeof(*rcp_ack), 0);
	if (NULL == rcp_ack) {
		err("Out of memory\n");
		goto send_response;
	}

	rcp_ack->status = mhl_sm_info.rcp_sent_status;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_RCP_ACK;
	resp.resp_status = resp_status;
	resp.data = rcp_ack;
	resp.data_size = (NULL != rcp_ack) ? sizeof(*rcp_ack) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(rcp_ack);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status send_mhl_rap_actioncode(struct SiiOsQueue *resp_q,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_rap *rap = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *rap_data = NULL;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	rap = req_data;

	if (sizeof(*rap) != req_data_size) {
		err("Illegal data for Send MHL RAP Actioncode request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL RAP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	rap_data = SiiOsCalloc("SendMHLRAPActioncodeData",
				HM_MHL_SEND_RAP_ACTION_CMD_MLEN, 0);
	if (NULL == rap_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(rap->actioncode, rap_data,
			HM_MHL_SEND_RAP_ACTION_CMD_ACTION);

	retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_RAP_ACTION,
				HM_MHL_SEND_RAP_ACTION_CMD_MLEN, rap_data,
				HM_MHL_SEND_RAP_ACTION_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send MHL RAP Actioncode command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Send MHL RAP Actioncode successful\n");

	mhl_sm_info.rap_sent = rap->actioncode;
	mhl_sm_info.rap_sent_status = 0;

send_response:
	resp.request_type = SII_SM_REQ_SEND_MHL_RAP_ACTIONCODE;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(rap_data);
	return retval;
}

enum sii_os_status get_mhl_rap_rcvd_actioncode(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_rap *rap = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL RAP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	rap = SiiOsCalloc("GetMHLRAPRcvdAction", sizeof(*rap), 0);
	if (NULL == rap) {
		err("Out of memory\n");
		goto send_response;
	}

	rap->actioncode = mhl_sm_info.rap_received;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_RAP_RCVD_ACTIONCODE;
	resp.resp_status = resp_status;
	resp.data = rap;
	resp.data_size = (NULL != rap) ? sizeof(*rap) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(rap);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_mhl_rap_sent_actioncode(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_rap *rap = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL RAP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	rap = SiiOsCalloc("GetMHLRAPAction", sizeof(*rap), 0);
	if (NULL == rap) {
		err("Out of memory\n");
		goto send_response;
	}

	rap->actioncode = mhl_sm_info.rap_sent;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_RAP_SENT_ACTIONCODE;
	resp.resp_status = resp_status;
	resp.data = rap;
	resp.data_size = (NULL != rap) ? sizeof(*rap) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(rap);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status send_mhl_rap_ack(struct SiiOsQueue *resp_q,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_rap_status *rap_ack = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *rap_ack_data = NULL;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	rap_ack = req_data;

	if (sizeof(*rap_ack) != req_data_size) {
		err("Illegal data for Send MHL RAP ACK request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL RAP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	rap_ack_data = SiiOsCalloc("SendMHLRAPACKData",
					HM_MHL_SEND_RAP_ACK_CMD_MLEN, 0);
	if (NULL == rap_ack_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	/* If errcode is not 0, send NAK. Send all zeros for an ACK. */
	if (RAPK_NO_ERROR != rap_ack->status) {
		field_ua_set(rap_ack->status, rap_ack_data,
				HM_MHL_SEND_RAP_ACK_CMD_ERR);
	}

	retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_RAP_ACK,
				HM_MHL_SEND_RAP_ACK_CMD_MLEN, rap_ack_data,
				HM_MHL_SEND_RAP_ACK_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send MHL RAP ACK/NAK command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Send MHL RAP ACK/NAK successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SEND_MHL_RAP_ACK;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(rap_ack_data);
	return retval;
}

enum sii_os_status get_mhl_rap_ack(struct SiiOsQueue *resp_q)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_mhl_rap_status *rap_ack = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == resp_q) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}

	if (!mhl_is_connected(devinfo->mhl->state)) {
		dbg("MHL RAP not available in current state\n");
		resp_status = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto send_response;
	}

	/* This memory is freed by the response handler */
	rap_ack = SiiOsCalloc("GetMHLRAPACK", sizeof(*rap_ack), 0);
	if (NULL == rap_ack) {
		err("Out of memory\n");
		goto send_response;
	}

	rap_ack->status = mhl_sm_info.rap_sent_status;

send_response:
	resp.request_type = SII_SM_REQ_GET_MHL_RAP_ACK;
	resp.resp_status = resp_status;
	resp.data = rap_ack;
	resp.data_size = (NULL != rap_ack) ? sizeof(*rap_ack) : 0;

	retval = SiiOsQueueSend(resp_q, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		SiiOsFree(rap_ack);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status report_mhl_connected(void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	devinfo->mhl->state = MHL_STATE_CONNECTED;
	(void)send_sii6400_uevent(devinfo->mhl->device, MHL_EVENT,
					MHL_CONNECTED_EVENT, NULL);
	sysfs_notify(&devinfo->mhl->device->kobj, NULL, __stringify(state));

done:
	return retval;
}

enum sii_os_status report_mhl_disconnected(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	devinfo->mhl->state = MHL_STATE_DISCONNECTED;
	(void)send_sii6400_uevent(devinfo->mhl->device, MHL_EVENT,
					MHL_DISCONNECTED_EVENT, NULL);
	sysfs_notify(&devinfo->mhl->device->kobj, NULL, __stringify(state));


#ifdef ENABLE_MHL_UEVENT
	dbg("report_mhl_disconnected notify the mhl switch dev\n");
	mhl_state_uevent(0, 1);
#endif


done:
	return retval;
}

enum sii_os_status report_mhl_spad_msg_rcv(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	char *spad_msg_str = NULL;
	char *spad_data_str = NULL;
	uint8_t length = 0;
	uint8_t spad_offset = 0;

	dbg("");

	if ((NULL == req_data) ||
	    (HM_MHL_SPAD_DATA_CHG_NOTIFY_MLEN_MIN > req_data_size) ||
	    (HM_MHL_SPAD_DATA_CHG_NOTIFY_MLEN_MAX < req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	length = req_data_size - HM_MHL_SPAD_DATA_CHG_NOTIFY_MLEN_MIN;
	if (MAX_SCRATCHPAD_REGISTER < length) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (length) {
		uint8_t spad_data[MAX_SCRATCHPAD_REGISTER];
		int offset = 0;
		char *buf = NULL;
		size_t buf_size = length * 5;
		int i;

		spad_data_str = SiiOsCalloc("ReportSpadData", buf_size, 0);
		if (NULL == spad_data_str) {
			err("Out of memory\n");
			retval = SII_OS_STATUS_ERR_FAILED;
			goto done;
		}

		memcpy(spad_data,
			(void *)FIELD_UA_BYTE_ADDRESS(req_data,
				HM_MHL_SPAD_DATA_CHG_NOTIFY_DATA_0),
			length);

		buf = spad_data_str;
		for (i = 0; i < length; i++) {
			if (i) {
				offset += scnprintf(buf + offset,
							buf_size - offset, " ");
			}
			offset += scnprintf(buf + offset, buf_size - offset,
						"0x%02x", spad_data[i]);
		}
	}

	spad_offset = (uint8_t)field_ua_extract(req_data,
				HM_MHL_SPAD_DATA_CHG_NOTIFY_OFFSET);

	spad_msg_str = SiiOsCalloc("ReportSpadMsg",
				MAX_MHL_SPAD_REPORT_DATA_STRING_SIZE, 0);
	if (NULL == spad_msg_str) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if (spad_data_str) {
		scnprintf(spad_msg_str, MAX_MHL_SPAD_REPORT_DATA_STRING_SIZE,
				"length=%u offset=%u data=%s",
				length, spad_offset, spad_data_str);
	} else {
		scnprintf(spad_msg_str, MAX_MHL_SPAD_REPORT_DATA_STRING_SIZE,
				"length=0");
	}

	dbg("Local SPAD changed: %s\n", spad_msg_str);

done:
	SiiOsFree(spad_msg_str);
	SiiOsFree(spad_data_str);
	return retval;
}

enum sii_os_status report_mhl_ucp_char_rcv(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t charcode = 0;
	char *char_report_data_str = NULL;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if ((NULL == req_data) ||
	    (HM_MHL_UCP_CHAR_RCV_NOTIFY_MLEN != req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	charcode = (uint8_t)field_ua_extract(req_data,
					HM_MHL_UCP_CHAR_RCV_NOTIFY_CHAR);

	mhl_sm_info.ucp_received = charcode;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if (INPUT_DEVICE_APP == devinfo->my_ucp_input_device) {
		char_report_data_str = SiiOsCalloc("ReportUCPChar",
				MAX_MHL_CHAR_REPORT_DATA_STRING_SIZE, 0);
		if (NULL == char_report_data_str) {
			err("Out of memory\n");
			retval = SII_OS_STATUS_ERR_FAILED;
			goto done;
		}

		scnprintf(char_report_data_str,
				MAX_MHL_CHAR_REPORT_DATA_STRING_SIZE,
				"0x%02x", charcode);

		if (NULL != devinfo->mhl) {
			(void)send_sii6400_uevent(devinfo->mhl->device,
					MHL_EVENT, MHL_UCP_RECEIVED_EVENT,
					char_report_data_str);
		}
	} else {
		/* Valid UTF8 charcodes are 0x00 -> 0xbf, 0xc2 -> 0xf4 */
		if ((charcode < 0xf5) &&
		    (0xc0 != charcode) &&
		    (0xc1 != charcode)) {
			(void)send_ucp_ack_or_nak(charcode, UCPE_NO_ERROR);
			dbg("0x%hhx", charcode);
		} else {
			(void)send_ucp_ack_or_nak(charcode,
						UCPE_INEFFECTIVE_KEYCODE);
			dbg("-0x%hhx", charcode);
		}
	}

done:
	SiiOsFree(char_report_data_str);
	return retval;
}

enum sii_os_status report_mhl_ucp_ack_rcv(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t charcode = 0;
	uint8_t ack = 0;
	uint8_t errorcode = 0;
	char *char_report_data_str = NULL;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if ((NULL == req_data) ||
	    (HM_MHL_UCP_ACK_RCV_NOTIFY_MLEN != req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	charcode = (uint8_t)field_ua_extract(req_data,
					HM_MHL_UCP_ACK_RCV_NOTIFY_CHAR);
	ack = (uint8_t)field_ua_extract(req_data,
					HM_MHL_UCP_ACK_RCV_NOTIFY_ACK);

	if (mhl_sm_info.ucp_sent != charcode) {
		err("Received an ACK/NAK for wrong UCP charcode\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (0 == ack) {
		mhl_sm_info.ucp_sent_status = 0;
	} else {
		errorcode = (uint8_t)field_ua_extract(req_data,
					HM_MHL_UCP_ACK_RCV_NOTIFY_ERROR);
		mhl_sm_info.ucp_sent_status = errorcode;
	}

	char_report_data_str = SiiOsCalloc("ReportUCPACK",
				MAX_MHL_CHAR_REPORT_DATA_STRING_SIZE, 0);
	if (NULL == char_report_data_str) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if (0 == ack) {
		scnprintf(char_report_data_str,
				MAX_MHL_CHAR_REPORT_DATA_STRING_SIZE,
				"0x%02x", charcode);

		devinfo = get_sii6400_devinfo();
		if ((NULL != devinfo) && (NULL != devinfo->mhl)) {
			(void)send_sii6400_uevent(devinfo->mhl->device,
						MHL_EVENT, MHL_UCP_ACKED_EVENT,
						char_report_data_str);
		}
	} else {
		scnprintf(char_report_data_str,
				MAX_MHL_CHAR_REPORT_DATA_STRING_SIZE,
				"0x%02x", errorcode);

		devinfo = get_sii6400_devinfo();
		if ((NULL != devinfo) && (NULL != devinfo->mhl)) {
			(void)send_sii6400_uevent(devinfo->mhl->device,
						MHL_EVENT, MHL_UCP_ERROR_EVENT,
						char_report_data_str);
		}
	}

done:
	SiiOsFree(char_report_data_str);
	return retval;
}

enum sii_os_status report_mhl_rcp_key_rcv(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t keycode = 0;
	struct sii6400_device_info *devinfo = NULL;
	char *key_report_data_str = NULL;

	dbg("");

	if ((NULL == req_data) ||
	    (HM_MHL_RCP_KEY_RCV_NOTIFY_MLEN != req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	keycode = (uint8_t)field_ua_extract(req_data,
						HM_MHL_RCP_KEY_RCV_NOTIFY_KEY);

	mhl_sm_info.rcp_received = keycode;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if (INPUT_DEVICE_APP == devinfo->my_rcp_input_device) {
		dbg("devinfo->my_rcp_input_device 111111 = 0x%06x\n", devinfo->my_rcp_input_device);
		key_report_data_str = SiiOsCalloc("ReportRCPKey",
					MAX_MHL_KEY_REPORT_DATA_STRING_SIZE, 0);
		if (NULL == key_report_data_str) {
			err("Out of memory\n");
			retval = SII_OS_STATUS_ERR_FAILED;
			goto done;
		}

		scnprintf(key_report_data_str,
				MAX_MHL_KEY_REPORT_DATA_STRING_SIZE,
				"0x%02x", keycode);

		if (NULL != devinfo->mhl) {
			(void)send_sii6400_uevent(devinfo->mhl->device,
					MHL_EVENT, MHL_RCP_RECEIVED_EVENT,
					key_report_data_str);
		}
	} else {
		dbg("devinfo->my_rcp_input_device 222222 = 0x%06x\n", devinfo->my_rcp_input_device);
		if (rcp_support_table[keycode & RCP_KEY_ID_MASK].rcp_support &
							LOGICAL_DEVICE_MAP) {
			if (0 == generate_rcp_input_event(keycode)) {
				(void)send_rcp_ack_or_nak(keycode,
								RCPE_NO_ERROR);
				dbg("0x%hhx", keycode);
			} else {
				(void)send_rcp_ack_or_nak(keycode,
						RCPE_INEFFECTIVE_KEYCODE);
				dbg("-0x%hhx", keycode);
			}
		} else {
			(void)send_rcp_ack_or_nak(keycode,
						RCPE_INEFFECTIVE_KEYCODE);
			dbg("-0x%hhx", keycode);
		}
	}

done:
	SiiOsFree(key_report_data_str);
	return retval;
}

enum sii_os_status report_mhl_rcp_ack_rcv(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t keycode = 0;
	uint8_t ack = 0;
	uint8_t errorcode = 0;
	char *key_report_data_str = NULL;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if ((NULL == req_data) ||
	    (HM_MHL_RCP_ACK_RCV_NOTIFY_MLEN != req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	keycode = (uint8_t)field_ua_extract(req_data,
						HM_MHL_RCP_ACK_RCV_NOTIFY_KEY);
	ack = (uint8_t)field_ua_extract(req_data,
						HM_MHL_RCP_ACK_RCV_NOTIFY_ACK);

	if (mhl_sm_info.rcp_sent != keycode) {
		err("Received an ACK/NAK for wrong RCP keycode\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (0 == ack) {
		mhl_sm_info.rcp_sent_status = 0;
	} else {
		errorcode = (uint8_t)field_ua_extract(req_data,
					HM_MHL_RCP_ACK_RCV_NOTIFY_ERROR);
		mhl_sm_info.rcp_sent_status = errorcode;
	}

	key_report_data_str = SiiOsCalloc("ReportRCPACK",
					MAX_MHL_KEY_REPORT_DATA_STRING_SIZE, 0);
	if (NULL == key_report_data_str) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if (0 == ack) {
		scnprintf(key_report_data_str,
				MAX_MHL_KEY_REPORT_DATA_STRING_SIZE,
				"0x%02x", keycode);

		devinfo = get_sii6400_devinfo();
		if ((NULL != devinfo) && (NULL != devinfo->mhl)) {
			(void)send_sii6400_uevent(devinfo->mhl->device,
					MHL_EVENT, MHL_RCP_ACKED_EVENT,
					key_report_data_str);
		}
	} else {
		scnprintf(key_report_data_str,
				MAX_MHL_KEY_REPORT_DATA_STRING_SIZE,
				"0x%02x", errorcode);

		devinfo = get_sii6400_devinfo();
		if ((NULL != devinfo) && (NULL != devinfo->mhl)) {
			(void)send_sii6400_uevent(devinfo->mhl->device,
						MHL_EVENT, MHL_RCP_ERROR_EVENT,
						key_report_data_str);
		}
	}

done:
	SiiOsFree(key_report_data_str);
	return retval;
}

enum sii_os_status report_mhl_rap_action_rcv(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t action_code = 0;
	char *rap_data_str = NULL;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if ((NULL == req_data) ||
	    (HM_MHL_RAP_ACTION_RCV_NOTIFY_MLEN != req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	action_code = (uint8_t)field_ua_extract(req_data,
					HM_MHL_RAP_ACTION_RCV_NOTIFY_ACTION);

	mhl_sm_info.rap_received = action_code;

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if (INPUT_DEVICE_APP == devinfo->my_rap_input_device) {
		rap_data_str = SiiOsCalloc("ReportRAPActionCode",
				MAX_MHL_ACTION_REPORT_DATA_STRING_SIZE, 0);
		if (NULL == rap_data_str) {
			err("Out of memory\n");
			retval = SII_OS_STATUS_ERR_FAILED;
			goto done;
		}

		scnprintf(rap_data_str, MAX_MHL_ACTION_REPORT_DATA_STRING_SIZE,
				"0x%02x", action_code);

		if (NULL != devinfo->mhl) {
			(void)send_sii6400_uevent(devinfo->mhl->device,
					MHL_EVENT, MHL_RAP_RECEIVED_EVENT,
					rap_data_str);
		}
	} else {
		/*
		 * Valid RAP action codes are:
		 *   RAP_POLL        0x00
		 *   RAP_CONTENT_ON  0x10
		 *   RAP_CONTENT_OFF 0x11
		 * This driver only supports action code RAP_POLL.
		 */
		if (RAP_POLL == action_code) {
			(void)send_rap_ack_or_nak(action_code, RAPK_NO_ERROR);
			dbg("0x%hhx", action_code);
		} else if ((RAP_CONTENT_ON == action_code) ||
			   (RAP_CONTENT_OFF == action_code)) {
			(void)send_rap_ack_or_nak(action_code,
						RAPK_UNSUPPORTED_ACTION_CODE);
			dbg("-0x%hhx", action_code);
		} else {
			(void)send_rap_ack_or_nak(action_code,
						RAPK_UNRECOGNIZED_ACTION_CODE);
			dbg("-0x%hhx", action_code);
		}
	}

done:
	SiiOsFree(rap_data_str);
	return retval;
}

enum sii_os_status report_mhl_rap_ack_rcv(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t status_code = 0;
	char *rap_ack_data_str = NULL;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	if ((NULL == req_data) ||
	    (HM_MHL_RAP_ACK_RCV_NOTIFY_MLEN != req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	status_code = (uint8_t)field_ua_extract(req_data,
					HM_MHL_RAP_ACK_RCV_NOTIFY_ERROR);
	mhl_sm_info.rap_sent_status = status_code;

	rap_ack_data_str = SiiOsCalloc("ReportRAPACK",
				MAX_MHL_ACTION_REPORT_DATA_STRING_SIZE, 0);
	if (NULL == rap_ack_data_str) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	scnprintf(rap_ack_data_str, MAX_MHL_ACTION_REPORT_DATA_STRING_SIZE,
			"0x%02x", status_code);

	devinfo = get_sii6400_devinfo();
	if ((NULL != devinfo) && (NULL != devinfo->mhl)) {
		(void)send_sii6400_uevent(devinfo->mhl->device, MHL_EVENT,
					MHL_RAP_ACKED_EVENT, rap_ack_data_str);
	}

done:
	SiiOsFree(rap_ack_data_str);
	return retval;
}

enum sii_os_status report_mhl_vbus_power_request_rcv(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;
	uint8_t power_request = 0;
#ifdef MHL_POWER_OUT
	extern int dwc3_otg_set_mhl_power(bool enable);
#endif
	dbg("");

	if ((NULL == req_data) ||
	    (HM_MHL_VBUS_POWER_REQUEST_NOTIFY_MLEN != req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	/* Get VBUS Power Request */
	power_request = (uint8_t)field_ua_extract(req_data,
				HM_MHL_VBUS_POWER_REQUEST_NOTIFY_PWR_REQ);
	if (power_request) {
		(void)send_sii6400_uevent(devinfo->mhl->device, MHL_EVENT,
						MHL_VBUS_ON_EVENT, MHL_VBUS_ON_EVENT);/*NULL);*/
		dbg("MHL_VBUS_ON_EVENT power_request = 0x%06x\n", power_request);
#ifdef MHL_POWER_OUT
		if (devinfo->vbus_power_state != 1 &&
				!dwc3_otg_set_mhl_power(1))
			devinfo->vbus_power_state = 1;
#endif
	} else {
		(void)send_sii6400_uevent(devinfo->mhl->device, MHL_EVENT,
						MHL_VBUS_OFF_EVENT, MHL_VBUS_OFF_EVENT);/*NULL);*/
		dbg("MHL_VBUS_OFF_EVENT power_request = 0x%06x\n", power_request);
#ifdef MHL_POWER_OUT
		if (devinfo->vbus_power_state != 0 &&
				!dwc3_otg_set_mhl_power(0))
			devinfo->vbus_power_state = 0;
#endif
	}

done:
	return retval;
}

enum sii_os_status report_dcap_chg(void *req_data, uint32_t req_data_size)

{
         enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
         struct sii6400_device_info *devinfo = NULL;

         err("");

         devinfo = get_sii6400_devinfo();

         if ((NULL == devinfo) || (NULL == devinfo->mhl)) {
                   err("Invalid devinfo pointer\n");
                   retval = SII_OS_STATUS_ERR_FAILED;
                   goto done;
         }

         (void)send_sii6400_uevent(devinfo->mhl->device, MHL_EVENT,
                                               MHL_DCAP_CHG_EVENT, NULL);

         sysfs_notify(&devinfo->mhl->device->kobj, NULL, __stringify(state));
done:
         schedule_work(&auto_get_dcap_work);
         return retval;
}

static enum sii_os_status send_rap_ack_or_nak(uint8_t action_code,
						uint8_t status)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t *rap_ack_data = NULL;

	dbg("");

	rap_ack_data = SiiOsCalloc("SendMHLRAPACKData",
					HM_MHL_SEND_RAP_ACK_CMD_MLEN, 0);
	if (NULL == rap_ack_data) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	field_ua_set(status, rap_ack_data, HM_MHL_SEND_RAP_ACK_CMD_ERR);

	retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_RAP_ACK,
				HM_MHL_SEND_RAP_ACK_CMD_MLEN, rap_ack_data,
				HM_MHL_SEND_RAP_ACK_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send MHL RAP ACK/NAK command not sent\n");
		goto done;
	}
	dbg("Send MHL RAP ACK/NAK successful\n");

done:
	SiiOsFree(rap_ack_data);
	return retval;
}

static enum sii_os_status send_ucp_ack_or_nak(uint8_t charcode, uint8_t status)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t *ucp_ack_data = NULL;

	dbg("");

	ucp_ack_data = SiiOsCalloc("SendMHLUCPACKData",
					HM_MHL_SEND_UCP_ACK_CMD_MLEN, 0);
	if (NULL == ucp_ack_data) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	/* If NAK, send UCPE with status code */
	if (UCPE_NO_ERROR != status) {
		field_ua_set(1, ucp_ack_data, HM_MHL_SEND_UCP_ACK_CMD_ACK);
		field_ua_set(status, ucp_ack_data, HM_MHL_SEND_UCP_ACK_CMD_ERR);

		retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_UCP_ACK,
				HM_MHL_SEND_UCP_ACK_CMD_MLEN, ucp_ack_data,
				HM_MHL_SEND_UCP_ACK_RESPONSE_TIMEOUT,
				NULL, NULL);
		if (SII_OS_STATUS_SUCCESS != retval) {
			err("Send MHL UCPE command not sent\n");
			goto done_with_ucpe;
		}
		dbg("Send MHL UCPE successful\n");

done_with_ucpe:
		if (SII_OS_STATUS_SUCCESS != retval)
			goto done;

		/* Add a delay before sending UCPK after UCPE */
		msleep(DELAY_BETWEEN_UCPE_AND_UCPK);
	}

	/* Both ACK and NAK send UCPK with character code */
	field_ua_set(0, ucp_ack_data, HM_MHL_SEND_UCP_ACK_CMD_ACK);
	field_ua_set(charcode, ucp_ack_data, HM_MHL_SEND_UCP_ACK_CMD_ERR);

	retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_UCP_ACK,
				HM_MHL_SEND_UCP_ACK_CMD_MLEN, ucp_ack_data,
				HM_MHL_SEND_UCP_ACK_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send MHL UCPK command not sent\n");
		goto done;
	}
	dbg("Send MHL UCPK successful\n");

done:
	SiiOsFree(ucp_ack_data);
	return retval;
}

static enum sii_os_status send_rcp_ack_or_nak(uint8_t keycode, uint8_t status)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t *rcp_ack_data = NULL;

	dbg("");

	rcp_ack_data = SiiOsCalloc("SendMHLRCPACKData",
					HM_MHL_SEND_RCP_ACK_CMD_MLEN, 0);
	if (NULL == rcp_ack_data) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	/* If NAK, send RCPE with status code */
	if (RCPE_NO_ERROR != status) {
		field_ua_set(1, rcp_ack_data, HM_MHL_SEND_RCP_ACK_CMD_ACK);
		field_ua_set(status, rcp_ack_data, HM_MHL_SEND_RCP_ACK_CMD_ERR);

		retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_RCP_ACK,
				HM_MHL_SEND_RCP_ACK_CMD_MLEN, rcp_ack_data,
				HM_MHL_SEND_RCP_ACK_RESPONSE_TIMEOUT,
				NULL, NULL);
		if (SII_OS_STATUS_SUCCESS != retval) {
			err("Send MHL RCPE command not sent\n");
			goto done_with_rcpe;
		}
		dbg("Send MHL RCPE successful\n");

done_with_rcpe:
		if (SII_OS_STATUS_SUCCESS != retval)
			goto done;

		/* Add a delay before sending RCPK after RCPE */
		msleep(DELAY_BETWEEN_RCPE_AND_RCPK);
	}

	/* Both ACK and NAK send RCPK with key code */
	field_ua_set(0, rcp_ack_data, HM_MHL_SEND_RCP_ACK_CMD_ACK);
	field_ua_set(keycode, rcp_ack_data, HM_MHL_SEND_RCP_ACK_CMD_ERR);

	retval = send_hostmsg_command((uint16_t)HM_MHL_SEND_RCP_ACK,
				HM_MHL_SEND_RCP_ACK_CMD_MLEN, rcp_ack_data,
				HM_MHL_SEND_RCP_ACK_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Send MHL RCPK command not sent\n");
		goto done;
	}
	dbg("Send MHL RCPK successful\n");

done:
	SiiOsFree(rcp_ack_data);
	return retval;
}

/*
 * Initialize the MHL firmware when it is first loaded
 */
enum sii_os_status sii6400_mhl_init(struct sii6400_device_info *devinfo)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	/* Force the state to "disconnected" */
	devinfo->mhl->state = MHL_STATE_DISCONNECTED;
	dbg("MHL state changed to disconnected\n");

done:
	return retval;
}

bool sii6400_mhl_sm_is_enabled(void)
{
	return mhl_sm_is_enabled;
}

void sii6400_mhl_sm_enable(void)
{
	dbg("");

	mhl_sm_is_enabled = true;
}

void sii6400_mhl_sm_disable(void)
{
	dbg("");

	mhl_sm_is_enabled = false;
}

int sii6400_mhl_sm_init(void)
{
	int retval = 0;
	enum sii_os_status rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	memset(&mhl_sm_info, 0, sizeof(struct sii6400_mhl_sm_info));
	mhl_sm_info.local_devcap_offset = MHL_LOCAL_DEVCAP_OFFSET_DEFAULT;
	mhl_sm_info.remote_devcap_offset = MHL_REMOTE_DEVCAP_OFFSET_DEFAULT;

	rv = SiiOsQueueCreate("get_mhl_chip_ver_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_chip_version_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_chip_version_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_fw_ver_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_firmware_version_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_firmware_version_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_connection_state_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_connection_state_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_connection_state_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_id_impedance_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_id_impedance_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_id_impedance_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_mhl_clock_swing_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_mhl_clock_swing_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_mhl_clock_swing_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_clock_swing_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_clock_swing_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_clock_swing_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_mhl_cbus_voltage_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_mhl_cbus_voltage_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_mhl_cbus_voltage_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_mhl_local_devcap_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_mhl_local_devcap_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_mhl_local_devcap_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_local_devcap_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_local_devcap_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_local_devcap_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_mhl_local_devcap_offset_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_mhl_local_devcap_offset_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_mhl_local_devcap_offset_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_local_devcap_offset_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_local_devcap_offset_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_local_devcap_offset_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_remote_devcap_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_remote_devcap_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_remote_devcap_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_mhl_remote_devcap_offset_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_mhl_remote_devcap_offset_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_mhl_remote_devcap_offset_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_remote_devcap_offset_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_remote_devcap_offset_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_remote_devcap_offset_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("send_mhl_ucp_charcode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&send_mhl_ucp_charcode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("send_mhl_ucp_charcode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_ucp_rcvd_charcode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_ucp_rcvd_charcode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_ucp_rcvd_charcode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_ucp_sent_charcode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_ucp_sent_charcode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_ucp_sent_charcode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("send_mhl_ucp_ack_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&send_mhl_ucp_ack_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("send_mhl_ucp_ack_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_ucp_ack_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_ucp_ack_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_ucp_ack_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("send_mhl_rcp_keycode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&send_mhl_rcp_keycode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("send_mhl_rcp_keycode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_rcp_rcvd_keycode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_rcp_rcvd_keycode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_rcp_rcvd_keycode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_rcp_sent_keycode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_rcp_sent_keycode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_rcp_sent_keycode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("send_mhl_rcp_ack_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&send_mhl_rcp_ack_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("send_mhl_rcp_ack_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_rcp_ack_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_rcp_ack_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_rcp_ack_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("send_mhl_rap_actioncode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&send_mhl_rap_actioncode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("send_mhl_rap_actioncode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_rap_rcvd_actioncode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_rap_rcvd_actioncode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_rap_rcvd_actioncode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_rap_sent_actioncode_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_rap_sent_actioncode_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_rap_sent_actioncode_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("send_mhl_rap_ack_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&send_mhl_rap_ack_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("send_mhl_rap_ack_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_mhl_rap_ack_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_mhl_rap_ack_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_mhl_rap_ack_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	init_rcp_input_dev();

done:
	return retval;
}

void sii6400_mhl_sm_exit(void)
{
	dbg("");

	destroy_rcp_input_dev();

	if (NULL != get_mhl_chip_version_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_chip_version_resp_q)) {
			err("get_mhl_chip_version_resp_q delete failed\n");
		}
		get_mhl_chip_version_resp_q = NULL;
	}

	if (NULL != get_mhl_firmware_version_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_firmware_version_resp_q)) {
			err("get_mhl_firmware_version_resp_q delete failed\n");
		}
		get_mhl_firmware_version_resp_q = NULL;
	}

	if (NULL != get_mhl_connection_state_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_connection_state_resp_q)) {
			err("get_mhl_connection_state_resp_q delete failed\n");
		}
		get_mhl_connection_state_resp_q = NULL;
	}

	if (NULL != get_mhl_id_impedance_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_id_impedance_resp_q)) {
			err("get_mhl_id_impedance_resp_q delete failed\n");
		}
		get_mhl_id_impedance_resp_q = NULL;
	}

	if (NULL != set_mhl_clock_swing_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_mhl_clock_swing_resp_q)) {
			err("set_mhl_clock_swing_resp_q delete failed\n");
		}
		set_mhl_clock_swing_resp_q = NULL;
	}

	if (NULL != get_mhl_clock_swing_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_clock_swing_resp_q)) {
			err("get_mhl_clock_swing_resp_q delete failed\n");
		}
		get_mhl_clock_swing_resp_q = NULL;
	}

    if (NULL != set_mhl_cbus_voltage_resp_q) {
        if (SII_OS_STATUS_SUCCESS !=
            SiiOsQueueDelete(set_mhl_cbus_voltage_resp_q)) {
            err("set_mhl_cbus_voltage_resp_q delete failed\n");
        }
        set_mhl_cbus_voltage_resp_q = NULL;
    }

	if (NULL != set_mhl_local_devcap_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_mhl_local_devcap_resp_q)) {
			err("set_mhl_local_devcap_resp_q delete failed\n");
		}
		set_mhl_local_devcap_resp_q = NULL;
	}

	if (NULL != get_mhl_local_devcap_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_local_devcap_resp_q)) {
			err("get_mhl_local_devcap_resp_q delete failed\n");
		}
		get_mhl_local_devcap_resp_q = NULL;
	}

	if (NULL != set_mhl_local_devcap_offset_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_mhl_local_devcap_offset_resp_q)) {
			err("set_mhl_local_devcap_offset_resp_q del failed\n");
		}
		set_mhl_local_devcap_offset_resp_q = NULL;
	}

	if (NULL != get_mhl_local_devcap_offset_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_local_devcap_offset_resp_q)) {
			err("get_mhl_local_devcap_offset_resp_q del failed\n");
		}
		get_mhl_local_devcap_offset_resp_q = NULL;
	}

	if (NULL != get_mhl_remote_devcap_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_remote_devcap_resp_q)) {
			err("get_mhl_remote_devcap_resp_q delete failed\n");
		}
		get_mhl_remote_devcap_resp_q = NULL;
	}

	if (NULL != set_mhl_remote_devcap_offset_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_mhl_remote_devcap_offset_resp_q)) {
			err("set_mhl_remote_devcap_offset_resp_q del failed\n");
		}
		set_mhl_remote_devcap_offset_resp_q = NULL;
	}

	if (NULL != get_mhl_remote_devcap_offset_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_remote_devcap_offset_resp_q)) {
			err("get_mhl_remote_devcap_offset_resp_q del failed\n");
		}
		get_mhl_remote_devcap_offset_resp_q = NULL;
	}

	if (NULL != send_mhl_ucp_charcode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(send_mhl_ucp_charcode_resp_q)) {
			err("send_mhl_ucp_charcode_resp_q delete failed\n");
		}
		send_mhl_ucp_charcode_resp_q = NULL;
	}

	if (NULL != get_mhl_ucp_rcvd_charcode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_ucp_rcvd_charcode_resp_q)) {
			err("get_mhl_ucp_rcvd_charcode_resp_q delete failed\n");
		}
		get_mhl_ucp_rcvd_charcode_resp_q = NULL;
	}

	if (NULL != get_mhl_ucp_sent_charcode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_ucp_sent_charcode_resp_q)) {
			err("get_mhl_ucp_sent_charcode_resp_q delete failed\n");
		}
		get_mhl_ucp_sent_charcode_resp_q = NULL;
	}

	if (NULL != send_mhl_ucp_ack_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(send_mhl_ucp_ack_resp_q)) {
			err("send_mhl_ucp_ack_resp_q delete failed\n");
		}
		send_mhl_ucp_ack_resp_q = NULL;
	}

	if (NULL != get_mhl_ucp_ack_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(get_mhl_ucp_ack_resp_q)) {
			err("get_mhl_ucp_ack_resp_q delete failed\n");
		}
		get_mhl_ucp_ack_resp_q = NULL;
	}

	if (NULL != send_mhl_rcp_keycode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(send_mhl_rcp_keycode_resp_q)) {
			err("send_mhl_rcp_keycode_resp_q delete failed\n");
		}
		send_mhl_rcp_keycode_resp_q = NULL;
	}

	if (NULL != get_mhl_rcp_rcvd_keycode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_rcp_rcvd_keycode_resp_q)) {
			err("get_mhl_rcp_rcvd_keycode_resp_q delete failed\n");
		}
		get_mhl_rcp_rcvd_keycode_resp_q = NULL;
	}

	if (NULL != get_mhl_rcp_sent_keycode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_rcp_sent_keycode_resp_q)) {
			err("get_mhl_rcp_sent_keycode_resp_q delete failed\n");
		}
		get_mhl_rcp_sent_keycode_resp_q = NULL;
	}

	if (NULL != send_mhl_rcp_ack_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(send_mhl_rcp_ack_resp_q)) {
			err("send_mhl_rcp_ack_resp_q delete failed\n");
		}
		send_mhl_rcp_ack_resp_q = NULL;
	}

	if (NULL != get_mhl_rcp_ack_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(get_mhl_rcp_ack_resp_q)) {
			err("get_mhl_rcp_ack_resp_q delete failed\n");
		}
		get_mhl_rcp_ack_resp_q = NULL;
	}

	if (NULL != send_mhl_rap_actioncode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(send_mhl_rap_actioncode_resp_q)) {
			err("send_mhl_rap_actioncode_resp_q delete failed\n");
		}
		send_mhl_rap_actioncode_resp_q = NULL;
	}

	if (NULL != get_mhl_rap_rcvd_actioncode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_rap_rcvd_actioncode_resp_q)) {
			err("get_mhl_rap_rcvd_actioncode_resp_q del failed\n");
		}
		get_mhl_rap_rcvd_actioncode_resp_q = NULL;
	}

	if (NULL != get_mhl_rap_sent_actioncode_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_mhl_rap_sent_actioncode_resp_q)) {
			err("get_mhl_rap_sent_actioncode_resp_q del failed\n");
		}
		get_mhl_rap_sent_actioncode_resp_q = NULL;
	}

	if (NULL != send_mhl_rap_ack_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(send_mhl_rap_ack_resp_q)) {
			err("send_mhl_rap_ack_resp_q delete failed\n");
		}
		send_mhl_rap_ack_resp_q = NULL;
	}

	if (NULL != get_mhl_rap_ack_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(get_mhl_rap_ack_resp_q)) {
			err("get_mhl_rap_ack_resp_q delete failed\n");
		}
		get_mhl_rap_ack_resp_q = NULL;
	}
}

