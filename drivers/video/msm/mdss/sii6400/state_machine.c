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

#include <linux/delay.h>

#include "osal.h"
#include "state_machine.h"
#include "wihd_sm.h"
#include "mhl_sm.h"
#include "diag_sm.h"
#include "sii6400.h"
#include "host_msg.h"
#include "rcp_inputdev.h"
#include "remote_fw_update.h"

struct SiiOsQueue *wihd_sm_queue;
struct SiiOsQueue *mhl_sm_queue;
struct SiiOsQueue *diag_sm_queue;

static struct workqueue_struct *wihd_sm_wq;
static struct workqueue_struct *mhl_sm_wq;
static struct workqueue_struct *diag_sm_wq;

static struct sm_work *my_wihd_sm_work;
static struct sm_work *my_mhl_sm_work;
static struct sm_work *my_diag_sm_work;

static void wihd_state_machine(struct work_struct *work);
static void mhl_state_machine(struct work_struct *work);
static void diag_state_machine(struct work_struct *work);

/*
 * Send a request to a state machine and possibly wait for a response.
 *
 * parameters:
 *	sm_queue is a required value.
 *	request_type is a required value.
 *	req_data will be freed by the state_machine (must be allocated memory).
 *	If an error is returned, req_data is guaranteed to be freed.
 *	If req_data is NULL, no data will be sent in the request.
 *	If resp_queue is NULL, no response is expected.
 *	resp_timeout is the time (in msec) to wait for the resp to the request.
 *	*resp_data will be freed by the caller (must be allocated memory).
 *	If resp_data and/or resp_data_size are NULL, then resp data will not be
 *		returned to the caller (memory will be freed in this routine).
 *
 * return values:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_QUEUE_FULL if the queue has no free space
 *	SII_OS_STATUS_TIMEOUT on a timeout
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status sm_send_request(struct SiiOsQueue *sm_queue,
				enum sm_request request_type,
				void *req_data, uint32_t req_data_size,
				struct SiiOsQueue *resp_queue,
				int32_t resp_timeout,
				void **resp_data, uint32_t *resp_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct state_machine_request req;
	struct state_machine_response resp;
	uint32_t resp_size;

	dbg("");

	if (NULL != req_data && 0 == req_data_size) {
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		err("Invalid parameter\n");
		goto done;
	}

	/* Throw away any old responses. */
	if (NULL != resp_queue) {
		resp_size = sizeof(resp);
		memset(&resp, 0, sizeof(resp));
		while (SII_OS_STATUS_SUCCESS ==
				SiiOsQueueReceive(resp_queue, &resp,
						SII_OS_NO_WAIT, &resp_size)) {
			switch (request_type) {
			case SII_SM_REQ_GET_CHIP_VERSION:
			{
				struct sii6400_chip_version *chip_version =
							resp.data;

				if ((NULL != chip_version) &&
				    (sizeof(*chip_version) == resp.data_size) &&
				    (NULL != chip_version->chip_version)) {
					SiiOsFree(chip_version->chip_version);
					chip_version->chip_version = NULL;
				}
			}
				break;

			case SII_SM_REQ_GET_FIRMWARE_VERSION:
			{
				struct sii6400_firmware_version *fw_version =
							resp.data;

				if ((NULL != fw_version) &&
				    (sizeof(*fw_version) == resp.data_size) &&
				    (NULL != fw_version->fw_version)) {
					SiiOsFree(fw_version->fw_version);
					fw_version->fw_version = NULL;
				}
			}
				break;

			case SII_SM_REQ_GET_DIAG_COMMAND:
			case SII_SM_REQ_GET_DIAG_COMMAND_SYNC:
			{
				struct sii6400_diag_cmd *diag_cmd = resp.data;

				if ((NULL != diag_cmd) &&
				    (sizeof(*diag_cmd) == resp.data_size) &&
				    (NULL != diag_cmd->diag_cmd)) {
					SiiOsFree(diag_cmd->diag_cmd);
					diag_cmd->diag_cmd = NULL;
				}
			}
				break;

			case SII_SM_REQ_GET_DIAG_COMMAND_OUTPUT:
			{
				struct sii6400_diag_cmd_output *
						diag_cmd_output = resp.data;

				if ((NULL != diag_cmd_output) &&
				    (sizeof(*diag_cmd_output) ==
							resp.data_size) &&
				    (NULL != diag_cmd_output->output)) {
					SiiOsFree(diag_cmd_output->output);
					diag_cmd_output->output = NULL;
				}
			}
				break;

			default:
				break;
			}
			SiiOsFree(resp.data);
			resp_size = sizeof(resp);
			memset(&resp, 0, sizeof(resp));
		}
	}

	req.request_type = request_type;
	/* This memory will be freed by the state machine */
	req.data = req_data;
	req.data_size = req_data_size;
	req.response_queue = resp_queue;
	retval = SiiOsQueueSend(sm_queue, &req, sizeof(req));
	if (SII_OS_STATUS_SUCCESS != retval) {
		SiiOsFree(req_data);
		err("Could not send request to state machine: err=%d\n",
		    retval);
		goto done;
	}

	if (NULL != resp_queue) {
		resp_size = sizeof(resp);
		memset(&resp, 0, sizeof(resp));
		retval = SiiOsQueueReceive(resp_queue, &resp, resp_timeout,
						&resp_size);
		if (SII_OS_STATUS_SUCCESS != retval) {
			err("Cannot receive state machine response: err = %d\n",
			    retval);
			goto done;
		}
		retval = resp.resp_status;
		if ((NULL != resp_data) && (NULL != resp_data_size)) {
			/* This memory will be freed by the caller */
			*resp_data = resp.data;
			*resp_data_size = resp.data_size;
		} else {
			SiiOsFree(resp.data);
		}
	}

done:
	return retval;
}

/*
 * Send a command to hostmsg and check the response.
 *
 * parameters:
 *	cmdcode is the hostmsg command ID.
 *	cmddatalen is the length of the cmddata buffer.
 *	cmddata is the command buffer.
 *	resp_timeout is the time (in msec) to wait for the command response.
 *	resp_data_size and resp_data can be NULL if this info is not needed.
 *	If resp_data is NULL, this routine will free the response data memory.
 *	If resp_data is not NULL, *resp_data must be freed by the caller.
 *
 * return values:
 *	SII_OS_STATUS_SUCCESS on success
 *	SII_OS_STATUS_ERR_INVALID_PARAM if any parameter is invalid
 *	SII_OS_STATUS_ERR_NOT_AVAIL if that command is not enabled
 *	SII_OS_STATUS_ERR_FAILED for all other errors
 */
enum sii_os_status send_hostmsg_command(uint16_t cmdcode, uint16_t cmddatalen,
				void *cmddata, uint32_t resp_timeout,
				uint16_t *resp_data_size, void **resp_data)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint16_t local_resp_data_size = 0;
	void *local_resp_data = NULL;
	enum CmdRspResult cmd_resp;

	retval = HostMsgSendCommand(cmdcode, cmddatalen, cmddata, resp_timeout,
				&local_resp_data_size, &local_resp_data);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Hostmsg command not sent\n");
		goto done;
	}

	if ((NULL == local_resp_data) || (0 == local_resp_data_size)) {
		err("Hostmsg response missing data\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	cmd_resp = (enum CmdRspResult)field_ua_extract(local_resp_data,
					HM_COMMON_CMD_RSP_RESULT_CODE);
	switch (cmd_resp) {
	case CrrSuccess:
		dbg("Send command successful\n");
		retval = SII_OS_STATUS_SUCCESS;
		break;

	case CrrErrInvalidParam:
		err("Command not sent, invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		break;

	case CrrErrModeDisabled:
		err("Command not sent, disabled\n");
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
		break;
	case CrrErrNetworkErr:
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
		break;
	case CrrErrFail:
	default:
		err("Send command failed\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		break;
	}

done:
	if (NULL != resp_data)
		*resp_data = local_resp_data;
	else
		SiiOsFree(local_resp_data);
	if (NULL != resp_data_size)
		*resp_data_size = local_resp_data_size;
	return retval;
}

/*
 * Task that handles the WiHD state machine request queue.
 */
static void wihd_state_machine(struct work_struct *work)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct state_machine_request req;
	uint32_t req_size;

	dbg("");

	req.data = NULL;
	while (!sii6400_module_is_exiting() && sii6400_wihd_sm_is_enabled()) {
		req_size = sizeof(req);
		memset(&req, 0, sizeof(req));
		retval = SiiOsQueueReceive(wihd_sm_queue, &req,
				WAIT_TIME_FOR_STATE_MACHINE_REQUEST, &req_size);
		if ((SII_OS_STATUS_QUEUE_EMPTY == retval) ||
		    (SII_OS_STATUS_TIMEOUT == retval)) {
			continue;
		} else if (SII_OS_STATUS_SUCCESS != retval) {
			err("WiHD state machine request queue error = %d\n",
			    retval);
			msleep(WAIT_TIME_FOR_STATE_MACHINE_QUEUE_FAILURE);
			continue;
		}

		dbg("Handling request %d\n", req.request_type);
		switch (req.request_type) {
		case SII_SM_REQ_GET_CHIP_VERSION:
			retval = get_wihd_chip_version(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get WiHD Chip Version failed\n");
			break;

		case SII_SM_REQ_GET_FIRMWARE_VERSION:
			retval = get_wihd_firmware_version(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get WiHD Firmware Version failed\n");
			break;

		case SII_SM_REQ_GET_WIHD_ID_IMPEDANCE:
			retval = get_wihd_id_impedance(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get WiHD ID Impedance failed\n");
			break;

		case SII_SM_REQ_GET_REMOTE_MAC_ADDR:
			retval = get_remote_mac_addr(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Remote MAC Addr failed\n");
			break;

		case SII_SM_REQ_GET_REMOTE_CATEGORY:
			retval = get_remote_category(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Remote Category failed\n");
			break;

		case SII_SM_REQ_GET_REMOTE_MANUFACTURER:
			retval = get_remote_manufacturer(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Remote Manufacturer failed\n");
			break;

		case SII_SM_REQ_GET_REMOTE_MONITOR_NAME:
			retval = get_remote_monitor_name(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Remote Monitor Name failed\n");
			break;

		case SII_SM_REQ_GET_REMOTE_NAME:
			retval = get_remote_name(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Remote Name failed\n");
			break;

		case SII_SM_REQ_GET_REMOTE_TYPE:
			retval = get_remote_type(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Remote Type failed\n");
			break;

		case SII_SM_REQ_GET_SIGNAL_STRENGTH:
			retval = get_signal_strength(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Signal Strength failed\n");
			break;

		case SII_SM_REQ_GET_MAC_ADDR:
			retval = get_mac_addr(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MAC Address failed\n");
			break;

		case SII_SM_REQ_SET_NAME:
			retval = set_name(req.response_queue, req.data,
						req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set Name failed\n");
			break;

		case SII_SM_REQ_GET_NAME:
			retval = get_name(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Name failed\n");
			break;

		case SII_SM_REQ_SET_WVAN_SCAN_DURATION:
			retval = set_wvan_scan_duration(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set WVAN Scan Duration failed\n");
			break;

		case SII_SM_REQ_GET_WVAN_SCAN_DURATION:
			retval = get_wvan_scan_duration(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get WVAN Scan Duration failed\n");
			break;

		case SII_SM_REQ_SET_WVAN_SCAN_INTERVAL:
			retval = set_wvan_scan_interval(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set WVAN Scan Interval failed\n");
			break;

		case SII_SM_REQ_GET_WVAN_SCAN_INTERVAL:
			retval = get_wvan_scan_interval(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get WVAN Scan Interval failed\n");
			break;

		case SII_SM_REQ_SCAN_FOR_WVANS:
			retval = scan_for_wvans(req.response_queue, req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Scan For WVANs failed\n");
			break;

		case SII_SM_REQ_GET_WVAN_SCAN_STATUS:
			retval = get_wvan_scan_status(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get WVAN Scan Status failed\n");
			break;

		case SII_SM_REQ_JOIN_WVAN:
			retval = join_wvan(req.response_queue, req.data,
						req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Join WVAN failed\n");
			break;

		case SII_SM_REQ_LEAVE_WVAN:
			retval = leave_wvan(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Leave WVAN failed\n");
			break;

		case SII_SM_REQ_GET_WVAN_INFO:
			retval = get_wvan_info(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get WVAN Info failed\n");
			break;

		case SII_SM_GET_CONNECTION_STATUS:
			retval = get_connection_status(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Connection Status failed\n");
			break;

		case SII_SM_GET_DEVICE_LIST:
			retval = get_device_list(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Search Results failed\n");
			break;

		case SII_SM_REQ_WIHD_CONNECT:
			retval = wihd_connect(req.response_queue, req.data,
						req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("WiHD Connect failed\n");
			break;

		case SII_SM_REQ_WIHD_DISCONNECT:
			retval = wihd_disconnect(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("WiHD Disconnect failed\n");
			break;

		case SII_SM_REQ_CEC_SEND_MSG:
			retval = cec_send_message(req.response_queue, req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("CEC Send Message failed\n");
			break;

		case SII_SM_REQ_GET_WIHD_RC_RCVD_CTRLCODE:
			retval = get_wihd_rc_rcvd_ctrlcode(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Received Remote Ctrl Code failed\n");
			break;

		case SII_SM_REQ_GET_WIHD_RC_SENT_CTRLCODE:
			retval = get_wihd_rc_sent_ctrlcode(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Sent Remote Control Code failed\n");
			break;

		case SII_SM_REQ_SEND_WIHD_RC_CTRLCODE:
			retval = send_wihd_rc_ctrlcode(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Send Remote Control Code failed\n");
			break;

		case SII_SM_REQ_SET_HDCP_POLICY:
			retval = set_hdcp_policy(req.response_queue, req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set HDCP Policy failed\n");
			break;

		case SII_SM_REQ_GET_HDCP_POLICY:
			retval = get_hdcp_policy(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get HDCP Policy failed\n");
			break;

		case SII_SM_REQ_SET_HDCP_STREAM_TYPE:
			retval = set_hdcp_stream_type(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set HDCP Stream Type failed\n");
			break;

		case SII_SM_REQ_GET_HDCP_STREAM_TYPE:
			retval = get_hdcp_stream_type(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get HDCP Stream Type failed\n");
			break;

		case SII_SM_REQ_REPORT_SCAN_RESULTS:
			retval = report_scan_results(req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report Scan Results failed\n");
			break;

		case SII_SM_REQ_REPORT_ASSOCIATED:
			retval = report_associated(req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report Associated failed\n");
			break;

		case SII_SM_REQ_REPORT_DEV_LIST_CHANGED:
			retval = report_dev_list_changed(req.data,
								req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report Device List Changed failed\n");
			break;

		case SII_SM_REQ_REPORT_DISASSOCIATED:
			retval = report_disassociated(req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report Disassociated failed\n");
			break;

		case SII_SM_REQ_REPORT_CONNECTED:
			retval = report_connected(req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report Connected failed\n");
			break;

		case SII_SM_REQ_REPORT_DISCONNECTED:
			retval = report_disconnected(req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report Disconnected failed\n");
			break;

		case SII_SM_REQ_REPORT_CEC_MSG_RECEIVED:
			retval = report_cec_msg_received(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report CEC_MSG_RECEIVED failed\n");
			break;

		case SII_SM_REQ_REPORT_VENDOR_MSG_RECEIVED:
			retval = report_vendor_msg_received(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report Vendor Msg Received failed\n");
			break;

		case SII_SM_REQ_SET_REMOTE_FW_UPDATE:
			retval = set_remote_fw_update(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set Remote Firmware Update failed\n");
			break;

		case SII_SM_REQ_GET_VENDOR_MSG_RECV_FILTER:
			retval = get_vendor_msg_recv_filter(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Vendor Msg Recv Filter failed\n");
			break;

		case SII_SM_REQ_SET_VENDOR_MSG_RECV_FILTER:
			retval = set_vendor_msg_recv_filter(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set Vendor Msg Recv Filter failed\n");
			break;

		case SII_SM_REQ_SET_VENDOR_MSG_VENDOR_ID:
			retval = set_vendor_msg_vendor_id(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set Vendor Msg Vendor ID failed\n");
			break;

		case SII_SM_REQ_SET_VENDOR_MSG_MAC_ADDR:
			retval = set_vendor_msg_mac_addr(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set Vendor Msg Dest MAC Addr failed\n");
			break;

		case SII_SM_REQ_SEND_VENDOR_MSG:
			retval = send_vendor_msg(req.response_queue, req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Send Vendor Msg failed\n");
			break;

		case SII_SM_REQ_NOP:
		default:
			break;

		}

		SiiOsFree(req.data);
		req.data = NULL;
	}

	/* Clear out the request queue */
	req_size = sizeof(req);
	while (SII_OS_STATUS_SUCCESS == SiiOsQueueReceive(wihd_sm_queue, &req,
						SII_OS_NO_WAIT, &req_size)) {
		SiiOsFree(req.data);
		req.data = NULL;
		req_size = sizeof(req);
	}
}

/*
 * Task that handles the MHL state machine request queue.
 */
static void mhl_state_machine(struct work_struct *work)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct state_machine_request req;
	uint32_t req_size;

	dbg("");

	req.data = NULL;
	while (!sii6400_module_is_exiting() && sii6400_mhl_sm_is_enabled()) {
		req_size = sizeof(req);
		memset(&req, 0, sizeof(req));
		retval = SiiOsQueueReceive(mhl_sm_queue, &req,
				WAIT_TIME_FOR_STATE_MACHINE_REQUEST, &req_size);
		if ((SII_OS_STATUS_QUEUE_EMPTY == retval) ||
		    (SII_OS_STATUS_TIMEOUT == retval)) {
			continue;
		} else if (SII_OS_STATUS_SUCCESS != retval) {
			err("MHL state machine request queue error = %d\n",
			    retval);
			msleep(WAIT_TIME_FOR_STATE_MACHINE_QUEUE_FAILURE);
			continue;
		}

		dbg("Handling request %d\n", req.request_type);
		switch (req.request_type) {
		case SII_SM_REQ_GET_CHIP_VERSION:
			retval = get_mhl_chip_version(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL Chip Version failed\n");
			break;

		case SII_SM_REQ_GET_FIRMWARE_VERSION:
			retval = get_mhl_firmware_version(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL Firmware Version failed\n");
			break;

		case SII_SM_REQ_GET_MHL_DEV_TYPE:
			retval = get_mhl_dev_type(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL dev type failed\n");
			break;

		case SII_SM_REQ_GET_MHL_CONNECTION_STATE:
			retval = get_mhl_connection_state(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL Connection State failed\n");
			break;

		case SII_SM_REQ_GET_MHL_ID_IMPEDANCE:
			retval = get_mhl_id_impedance(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL ID Impedance failed\n");
			break;

		case SII_SM_REQ_SET_MHL_CLOCK_SWING:
			retval = set_mhl_clock_swing(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set MHL Clock Swing failed\n");
			break;

		case SII_SM_REQ_GET_MHL_CLOCK_SWING:
			retval = get_mhl_clock_swing(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL Clock Swing failed\n");
			break;

		case SII_SM_REQ_SET_MHL_CBUS_VOLTAGE:
			retval = set_mhl_cbus_voltage(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set MHL Clock Swing failed\n");
			break;
		case SII_SM_REQ_SET_MHL_LOCAL_DEVCAP:
			retval = set_mhl_local_devcap(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set MHL Local Device Cap failed\n");
			break;

		case SII_SM_REQ_GET_MHL_LOCAL_DEVCAP:
			retval = get_mhl_local_devcap(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL Local Device Cap failed\n");
			break;

		case SII_SM_REQ_SET_MHL_LOCAL_DEVCAP_OFFSET:
			retval = set_mhl_local_devcap_offset(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set MHL Local Device Cap Offset failed\n");
			break;

		case SII_SM_REQ_GET_MHL_LOCAL_DEVCAP_OFFSET:
			retval =
				get_mhl_local_devcap_offset(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL Local Device Cap Offset failed\n");
			break;

		case SII_SM_REQ_GET_MHL_REMOTE_DEVCAP:
			retval = get_mhl_remote_devcap(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL Remote Device Cap failed\n");
			break;

		case SII_SM_REQ_SET_MHL_REMOTE_DEVCAP_OFFSET:
			retval = set_mhl_remote_devcap_offset(
							req.response_queue,
							req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set MHL_REMOTE_DEVCAP_OFFSET failed\n");
			break;

		case SII_SM_REQ_GET_MHL_REMOTE_DEVCAP_OFFSET:
			retval = get_mhl_remote_devcap_offset(
							req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL_REMOTE_DEVCAP_OFFSET failed\n");
			break;

		case SII_SM_REQ_SEND_MHL_UCP_CHARCODE:
			retval = send_mhl_ucp_charcode(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Send MHL UCP Charcode failed\n");
			break;

		case SII_SM_REQ_GET_MHL_UCP_RCVD_CHARCODE:
			retval = get_mhl_ucp_rcvd_charcode(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL UCP Received Charcode failed\n");
			break;

		case SII_SM_REQ_GET_MHL_UCP_SENT_CHARCODE:
			retval = get_mhl_ucp_sent_charcode(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL UCP Sent Charcode failed\n");
			break;

		case SII_SM_REQ_SEND_MHL_UCP_ACK:
			retval = send_mhl_ucp_ack(req.response_queue, req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Send MHL UCP ACK failed\n");
			break;

		case SII_SM_REQ_GET_MHL_UCP_ACK:
			retval = get_mhl_ucp_ack(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL UCP ACK failed\n");
			break;

		case SII_SM_REQ_SEND_MHL_RCP_KEYCODE:
			retval = send_mhl_rcp_keycode(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Send MHL RCP Keycode failed\n");
			break;

		case SII_SM_REQ_GET_MHL_RCP_RCVD_KEYCODE:
			retval = get_mhl_rcp_rcvd_keycode(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL RCP Received Keycode failed\n");
			break;

		case SII_SM_REQ_GET_MHL_RCP_SENT_KEYCODE:
			retval = get_mhl_rcp_sent_keycode(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL RCP Sent Keycode failed\n");
			break;

		case SII_SM_REQ_SEND_MHL_RCP_ACK:
			retval = send_mhl_rcp_ack(req.response_queue, req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Send MHL RCP ACK failed\n");
			break;

		case SII_SM_REQ_GET_MHL_RCP_ACK:
			retval = get_mhl_rcp_ack(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL RCP ACK failed\n");
			break;

		case SII_SM_REQ_SEND_MHL_RAP_ACTIONCODE:
			retval = send_mhl_rap_actioncode(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Send MHL RAP Actioncode failed\n");
			break;

		case SII_SM_REQ_GET_MHL_RAP_RCVD_ACTIONCODE:
			retval =
				get_mhl_rap_rcvd_actioncode(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL RAP Received Actioncode failed\n");
			break;

		case SII_SM_REQ_GET_MHL_RAP_SENT_ACTIONCODE:
			retval =
				get_mhl_rap_sent_actioncode(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL RAP Sent Actioncode failed\n");
			break;

		case SII_SM_REQ_SEND_MHL_RAP_ACK:
			retval = send_mhl_rap_ack(req.response_queue, req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Send MHL RAP ACK failed\n");
			break;

		case SII_SM_REQ_GET_MHL_RAP_ACK:
			retval = get_mhl_rap_ack(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get MHL RAP ACK failed\n");
			break;

		case SII_SM_REQ_REPORT_MHL_CONNECTED:
			retval = report_mhl_connected(req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report MHL Connected failed\n");
			break;

		case SII_SM_REQ_REPORT_MHL_DISCONNECTED:
			retval = report_mhl_disconnected(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report MHL Disconnected failed\n");
			break;

		case SII_SM_REQ_REPORT_MHL_SPAD_MSG_RCV:
			retval = report_mhl_spad_msg_rcv(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report MHL_SPAD_MSG_RCV failed\n");
			break;

		case SII_SM_REQ_REPORT_MHL_UCP_CHAR_RCV:
			retval = report_mhl_ucp_char_rcv(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report MHL UCP Char received failed\n");
			break;

		case SII_SM_REQ_REPORT_MHL_UCP_ACK_RCV:
			retval = report_mhl_ucp_ack_rcv(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report MHL UCP ACK received failed\n");
			break;

		case SII_SM_REQ_REPORT_MHL_RCP_KEY_RCV:
			retval = report_mhl_rcp_key_rcv(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report MHL RCP Key received failed\n");
			break;

		case SII_SM_REQ_REPORT_MHL_RCP_ACK_RCV:
			retval = report_mhl_rcp_ack_rcv(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report MHL RCP ACK received failed\n");
			break;

		case SII_SM_REQ_REPORT_MHL_RAP_ACTION_RCV:
			retval = report_mhl_rap_action_rcv(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report MHL RAP Action received failed\n");
			break;

		case SII_SM_REQ_REPORT_MHL_RAP_ACK_RCV:
			retval = report_mhl_rap_ack_rcv(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report MHL RAP ACK received failed\n");
			break;

		case SII_SM_REQ_REPORT_MHL_VBUS_POWER_REQUEST_RCV:
			retval = report_mhl_vbus_power_request_rcv(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report MHL_VBUS_POWER_REQ_RCV failed\n");
			break;
		case SII_SM_REQ_REPORT_MHL_DCAP_CHG:
			retval = report_dcap_chg(req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report Dcap Change failed\n");
			break;
		case SII_SM_REQ_NOP:
		default:
			break;

		}

		SiiOsFree(req.data);
		req.data = NULL;
	}

	/* Clear out the request queue */
	req_size = sizeof(req);
	while (SII_OS_STATUS_SUCCESS == SiiOsQueueReceive(mhl_sm_queue, &req,
						SII_OS_NO_WAIT, &req_size)) {
		SiiOsFree(req.data);
		req.data = NULL;
		req_size = sizeof(req);
	}
}

/*
 * Task that handles the DIAG state machine request queue.
 */
static void diag_state_machine(struct work_struct *work)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct state_machine_request req;
	uint32_t req_size;

	dbg("");

	req.data = NULL;
	while (!sii6400_module_is_exiting() && sii6400_diag_sm_is_enabled()) {
		req_size = sizeof(req);
		memset(&req, 0, sizeof(req));
		retval = SiiOsQueueReceive(diag_sm_queue, &req,
				WAIT_TIME_FOR_STATE_MACHINE_REQUEST, &req_size);
		if ((SII_OS_STATUS_QUEUE_EMPTY == retval) ||
		    (SII_OS_STATUS_TIMEOUT == retval)) {
			continue;
		} else if (SII_OS_STATUS_SUCCESS != retval) {
			err("DIAG state machine request queue error = %d\n",
			    retval);
			msleep(WAIT_TIME_FOR_STATE_MACHINE_QUEUE_FAILURE);
			continue;
		}

		dbg("Handling request %d\n", req.request_type);
		switch (req.request_type) {
		case SII_SM_REQ_GET_DIAG_COMMAND:
			retval = get_diag_command(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Diag Command failed\n");
			break;

		case SII_SM_REQ_SEND_DIAG_COMMAND:
		{
			struct sii6400_diag_cmd *diag_cmd = req.data;

			retval = send_diag_command(req.response_queue, req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Send Diag Command failed\n");
			if ((NULL != diag_cmd) &&
			    (sizeof(*diag_cmd) == req.data_size)) {
				SiiOsFree(diag_cmd->diag_cmd);
				diag_cmd->diag_cmd = NULL;
			}
		}
			break;

		case SII_SM_REQ_GET_DIAG_COMMAND_SYNC:
			retval = get_diag_cmd_sync(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Diag Command Sync failed\n");
			break;

		case SII_SM_REQ_SEND_DIAG_COMMAND_SYNC:
		{
			struct sii6400_diag_cmd *diag_cmd = req.data;

			retval = send_diag_cmd_sync(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Send Diag Command Sync failed\n");
			if ((NULL != diag_cmd) &&
			    (sizeof(*diag_cmd) == req.data_size)) {
				SiiOsFree(diag_cmd->diag_cmd);
				diag_cmd->diag_cmd = NULL;
			}
		}
			break;

		case SII_SM_REQ_GET_DIAG_COMMAND_OUTPUT:
			retval = get_diag_cmd_output(req.response_queue);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Get Diag Command Output failed\n");
			break;

		case SII_SM_REQ_SET_DEBUG_OUTPUT_PATH:
			retval = set_debug_output_path(req.response_queue,
						req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Set Debug Output Path failed\n");
			break;

		case SII_SM_REQ_REPORT_DIAG_CMD_DONE:
			retval = report_diag_cmd_done(req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report DIAG Command Done failed\n");
			break;

		case SII_SM_REQ_REPORT_DIAG_CMD_OUTPUT:
			retval = report_diag_cmd_output(req.data,
							req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report DIAG Command Output failed\n");
			break;

		case SII_SM_REQ_REPORT_DEBUG_OUTPUT:
			retval = report_debug_output(req.data, req.data_size);
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Report Debug Output failed\n");
			break;

		case SII_SM_REQ_NOP:
		default:
			break;

		}

		SiiOsFree(req.data);
		req.data = NULL;
	}

	/* Clear out the request queue */
	req_size = sizeof(req);
	while (SII_OS_STATUS_SUCCESS == SiiOsQueueReceive(diag_sm_queue, &req,
						SII_OS_NO_WAIT, &req_size)) {
		switch (req.request_type) {
		case SII_SM_REQ_SEND_DIAG_COMMAND:
		case SII_SM_REQ_SEND_DIAG_COMMAND_SYNC:
		{
			struct sii6400_diag_cmd *diag_cmd = req.data;
			if ((NULL != diag_cmd) &&
			    (sizeof(*diag_cmd) == req.data_size)) {
				SiiOsFree(diag_cmd->diag_cmd);
				diag_cmd->diag_cmd = NULL;
			}
		}
			break;

		default:
			break;
		}
		SiiOsFree(req.data);
		req.data = NULL;
		req_size = sizeof(req);
	}
}

enum sii_os_status sii6400_wihd_sm_start(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	/* Queue WiHD state machine work */
	if (!queue_work(wihd_sm_wq, &my_wihd_sm_work->state_machine_work))
		dbg("WiHD State Machine task already queued\n");

	return retval;
}

enum sii_os_status sii6400_mhl_sm_start(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	/* Queue MHL state machine work */
	if (!queue_work(mhl_sm_wq, &my_mhl_sm_work->state_machine_work))
		dbg("MHL State Machine task already queued\n");

	return retval;
}

enum sii_os_status sii6400_diag_sm_start(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	/* Queue DIAG state machine work */
	if (!queue_work(diag_sm_wq, &my_diag_sm_work->state_machine_work))
		dbg("DIAG State Machine task already queued\n");

	return retval;
}

int sii6400_sm_init(void)
{
	int retval = 0;
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	rcp_input_dev_one_time_init();

	retval = sii6400_wihd_sm_init();
	if (retval < 0) {
		err("Failed to init WiHD state machine\n");
		retval = -EFAULT;
		goto init_failed;
	}

	retval = sii6400_mhl_sm_init();
	if (retval < 0) {
		err("Failed to init MHL state machine\n");
		retval = -EFAULT;
		goto init_failed;
	}

	retval = sii6400_diag_sm_init();
	if (retval < 0) {
		err("Failed to init DIAG state machine\n");
		retval = -EFAULT;
		goto init_failed;
	}

	retval = sii6400_remote_fw_update_init();
	if (retval < 0) {
		err("Failed to init Remote Firmware Update\n");
		retval = -EFAULT;
		goto init_failed;
	}

	sii_os_rv = SiiOsQueueCreate("WiHD_sm_queue",
				sizeof(struct state_machine_request),
				STATE_MACHINE_QUEUE_SIZE, &wihd_sm_queue);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("wihd_sm_queue create failed: err=%d\n",
		    sii_os_rv);
		retval = -EFAULT;
		goto init_failed;
	}

	sii_os_rv = SiiOsQueueCreate("MHL_sm_queue",
				sizeof(struct state_machine_request),
				STATE_MACHINE_QUEUE_SIZE, &mhl_sm_queue);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("mhl_sm_queue create failed: err=%d\n",
		    sii_os_rv);
		retval = -EFAULT;
		goto init_failed;
	}

	sii_os_rv = SiiOsQueueCreate("DIAG_sm_queue",
				sizeof(struct state_machine_request),
				STATE_MACHINE_QUEUE_SIZE, &diag_sm_queue);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("diag_sm_queue create failed: err=%d\n",
		    sii_os_rv);
		retval = -EFAULT;
		goto init_failed;
	}

	wihd_sm_wq = alloc_workqueue("wihd_sm_wq",
					WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (NULL == wihd_sm_wq) {
		err("wihd_sm_wq create failed\n");
		retval = -EFAULT;
		goto init_failed;
	}

	mhl_sm_wq = alloc_workqueue("mhl_sm_wq",
					WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (NULL == mhl_sm_wq) {
		err("mhl_sm_wq create failed\n");
		retval = -EFAULT;
		goto init_failed;
	}

	diag_sm_wq = alloc_workqueue("diag_sm_wq",
					WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (NULL == diag_sm_wq) {
		err("diag_sm_wq create failed\n");
		retval = -EFAULT;
		goto init_failed;
	}

	my_wihd_sm_work = SiiOsCalloc("wihd_sm_work",
					sizeof(struct sm_work), 0);
	if (NULL == my_wihd_sm_work) {
		err("Out of Memory!\n");
		retval = -ENOMEM;
		goto init_failed;
	}

	my_mhl_sm_work = SiiOsCalloc("mhl_sm_work",
					sizeof(struct sm_work), 0);
	if (NULL == my_mhl_sm_work) {
		err("Out of Memory!\n");
		retval = -ENOMEM;
		goto init_failed;
	}

	my_diag_sm_work = SiiOsCalloc("diag_sm_work",
					sizeof(struct sm_work), 0);
	if (NULL == my_diag_sm_work) {
		err("Out of Memory!\n");
		retval = -ENOMEM;
		goto init_failed;
	}

	INIT_WORK(&my_wihd_sm_work->state_machine_work, wihd_state_machine);
	INIT_WORK(&my_mhl_sm_work->state_machine_work, mhl_state_machine);
	INIT_WORK(&my_diag_sm_work->state_machine_work, diag_state_machine);

init_failed:
	return retval;
}

void sii6400_sm_exit(void)
{
	enum sii_os_status sii_os_rv;

	dbg("");

	if (NULL != my_wihd_sm_work) {
		cancel_work_sync(&my_wihd_sm_work->state_machine_work);
		SiiOsFree(my_wihd_sm_work);
		my_wihd_sm_work = NULL;
	}
	if (NULL != my_mhl_sm_work) {
		cancel_work_sync(&my_mhl_sm_work->state_machine_work);
		SiiOsFree(my_mhl_sm_work);
		my_mhl_sm_work = NULL;
	}
	if (NULL != my_diag_sm_work) {
		cancel_work_sync(&my_diag_sm_work->state_machine_work);
		SiiOsFree(my_diag_sm_work);
		my_diag_sm_work = NULL;
	}

	if (NULL != wihd_sm_wq) {
		destroy_workqueue(wihd_sm_wq);
		wihd_sm_wq = NULL;
	}
	if (NULL != mhl_sm_wq) {
		destroy_workqueue(mhl_sm_wq);
		mhl_sm_wq = NULL;
	}
	if (NULL != diag_sm_wq) {
		destroy_workqueue(diag_sm_wq);
		diag_sm_wq = NULL;
	}

	if (NULL != wihd_sm_queue) {
		sii_os_rv = SiiOsQueueDelete(wihd_sm_queue);
		if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
			err("wihd_sm_queue delete failed: err=%d\n",
				sii_os_rv);
		}
		wihd_sm_queue = NULL;
	}

	if (NULL != mhl_sm_queue) {
		sii_os_rv = SiiOsQueueDelete(mhl_sm_queue);
		if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
			err("mhl_sm_queue delete failed: err=%d\n",
				sii_os_rv);
		}
		mhl_sm_queue = NULL;
	}

	if (NULL != diag_sm_queue) {
		sii_os_rv = SiiOsQueueDelete(diag_sm_queue);
		if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
			err("diag_sm_queue delete failed: err=%d\n",
				sii_os_rv);
		}
		diag_sm_queue = NULL;
	}

	sii6400_wihd_sm_exit();
	sii6400_mhl_sm_exit();
	sii6400_diag_sm_exit();
	sii6400_remote_fw_update_exit();
}

