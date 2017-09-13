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
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "osal.h"
#include "state_machine.h"
#include "diag_sm.h"
#include "sii6400.h"
#include "host_msg.h"

static struct SiiOsQueue *diag_cmd_q;
static struct SiiOsQueue *diag_cmd_output_q;
struct SiiOsQueue *get_diag_cmd_resp_q;
struct SiiOsQueue *send_diag_cmd_resp_q;
struct SiiOsQueue *get_diag_cmd_sync_resp_q;
struct SiiOsQueue *send_diag_cmd_sync_resp_q;
struct SiiOsQueue *get_diag_cmd_output_resp_q;
struct SiiOsQueue *set_debug_output_path_resp_q;

static struct workqueue_struct *diag_cmd_wq;
static struct diag_cmd_work *my_diag_cmd_work;

static struct SiiOsTimer *diag_cmd_resp_timer;

static struct sii6400_diag_sm_info diag_sm_info;

static bool diag_sm_is_enabled;

static void diag_cmd_resp_timer_function(void *data);
static void diag_cmd_resp_work_function(struct work_struct *work);
static void diag_cmd_handler(struct work_struct *work);
static enum sii_os_status handle_cmd_done(void);
static enum sii_os_status send_my_debug_output_path(
			struct sii6400_debug_output_path *debug_output_path);

static DECLARE_WORK(diag_cmd_response_work, diag_cmd_resp_work_function);


enum sii_os_status get_diag_command(struct SiiOsQueue *response_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_diag_cmd *diag_cmd = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == response_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* This memory is freed by the response handler */
	diag_cmd = SiiOsCalloc("GetDiagCmd", sizeof(*diag_cmd), 0);
	if (NULL == diag_cmd) {
		err("Out of memory\n");
		goto send_response;
	}

	if ((NULL == diag_sm_info.latest_async_queued.diag_cmd) ||
	    (0 == diag_sm_info.latest_async_queued.length)) {
		dbg("No Diagnostic Command was sent\n");
		goto send_response;
	}

	diag_cmd->length = diag_sm_info.latest_async_queued.length;

	/* This memory is freed by the response handler */
	diag_cmd->diag_cmd =
		kstrndup(diag_sm_info.latest_async_queued.diag_cmd,
				diag_cmd->length, GFP_KERNEL);

send_response:
	resp.request_type = SII_SM_REQ_GET_DIAG_COMMAND;
	resp.resp_status = resp_status;
	resp.data = diag_cmd;
	resp.data_size = (NULL != diag_cmd) ? sizeof(*diag_cmd) : 0;

	retval = SiiOsQueueSend(response_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		if (NULL != diag_cmd)
			SiiOsFree(diag_cmd->diag_cmd);
		SiiOsFree(diag_cmd);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status send_diag_command(struct SiiOsQueue *response_queue,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_diag_cmd *diag_cmd = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == response_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	diag_cmd = req_data;

	if (sizeof(*diag_cmd) != req_data_size) {
		err("Illegal Diag Command for Send Diag Command request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	if (MAX_DIAG_CMD_STR_LEN < diag_cmd->length) {
		err("Length of diag cmd exceeds maximum allowed\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	retval = SiiOsQueueSend(diag_cmd_q, diag_cmd, sizeof(*diag_cmd));
	if (SII_OS_STATUS_QUEUE_FULL == retval) {
		warn("Diag Cmd queue is full\n");
		resp_status = SII_OS_STATUS_QUEUE_FULL;
		goto send_response;
	} else if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not queue latest diag cmd output: err=%d\n", retval);
		resp_status = retval;
		goto send_response;
	}

	diag_cmd->diag_cmd = NULL; /* Prevent state machine from freeing */

send_response:
	resp.request_type = SII_SM_REQ_SEND_DIAG_COMMAND;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(response_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_diag_cmd_sync(struct SiiOsQueue *response_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_diag_cmd *diag_cmd = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == response_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* This memory is freed by the response handler */
	diag_cmd = SiiOsCalloc("GetDiagCmdSync", sizeof(*diag_cmd), 0);
	if (NULL == diag_cmd) {
		err("Out of memory\n");
		goto send_response;
	}

	if ((NULL == diag_sm_info.latest_sync_queued.diag_cmd) ||
	    (0 == diag_sm_info.latest_sync_queued.length)) {
		dbg("No Diagnostic Command was sent\n");
		goto send_response;
	}

	diag_cmd->length = diag_sm_info.latest_sync_queued.length;

	/* This memory is freed by the response handler */
	diag_cmd->diag_cmd =
		kstrndup(diag_sm_info.latest_sync_queued.diag_cmd,
				diag_cmd->length, GFP_KERNEL);

send_response:
	resp.request_type = SII_SM_REQ_GET_DIAG_COMMAND_SYNC;
	resp.resp_status = resp_status;
	resp.data = diag_cmd;
	resp.data_size = (NULL != diag_cmd) ? sizeof(*diag_cmd) : 0;

	retval = SiiOsQueueSend(response_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		if (NULL != diag_cmd)
			SiiOsFree(diag_cmd->diag_cmd);
		SiiOsFree(diag_cmd);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status send_diag_cmd_sync(struct SiiOsQueue *response_queue,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_diag_cmd *diag_cmd = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == response_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	diag_cmd = req_data;

	if (sizeof(*diag_cmd) != req_data_size) {
		err("Illegal Command for Send Diag Cmd Sync request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	if (MAX_DIAG_CMD_STR_LEN < diag_cmd->length) {
		err("Length of diag cmd exceeds maximum allowed\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	retval = SiiOsQueueSend(diag_cmd_q, diag_cmd, sizeof(*diag_cmd));
	if (SII_OS_STATUS_QUEUE_FULL == retval) {
		warn("Diag Cmd queue is full\n");
		resp_status = SII_OS_STATUS_QUEUE_FULL;
		goto send_response;
	} else if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not queue latest diag cmd output: err=%d\n", retval);
		resp_status = retval;
		goto send_response;
	}

	diag_cmd->diag_cmd = NULL; /* Prevent state machine from freeing */

	goto done; /* response sent in cmd done handler */

send_response:
	resp.request_type = SII_SM_REQ_SEND_DIAG_COMMAND_SYNC;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(response_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status get_diag_cmd_output(struct SiiOsQueue *response_queue)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_diag_cmd_output *diag_cmd_output = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == response_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* This memory is freed by the response handler */
	diag_cmd_output = SiiOsCalloc("GetDiagCommandOutput",
					sizeof(*diag_cmd_output), 0);
	if (NULL == diag_cmd_output) {
		err("Out of memory\n");
	} else {
		/* This memory is freed by the response handler */
		uint32_t diag_cmd_output_size = sizeof(*diag_cmd_output);

		retval = SiiOsQueueReceive(diag_cmd_output_q,
			diag_cmd_output, SII_OS_NO_WAIT, &diag_cmd_output_size);
		if (SII_OS_STATUS_QUEUE_EMPTY == retval) {
			dbg("Diag Cmd Output queue is empty\n");
			diag_cmd_output->output = NULL;
			goto send_response;
		} else if (SII_OS_STATUS_SUCCESS != retval) {
			err("Diag Cmd Output queue error = %d\n", retval);
			diag_cmd_output->output = NULL;
			goto send_response;
		}
	}

send_response:
	resp.request_type = SII_SM_REQ_GET_DIAG_COMMAND;
	resp.resp_status = resp_status;
	resp.data = diag_cmd_output;
	resp.data_size = (NULL != diag_cmd_output) ?
				sizeof(*diag_cmd_output) : 0;

	retval = SiiOsQueueSend(response_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		/* Free memory here, since response handler won't */
		if (NULL != diag_cmd_output)
			SiiOsFree(diag_cmd_output->output);
		SiiOsFree(diag_cmd_output);
		goto done;
	}

done:
	return retval;
}

enum sii_os_status set_debug_output_path(struct SiiOsQueue *response_queue,
					void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_debug_output_path *debug_output_path = NULL;
	struct state_machine_response resp;
	enum sii_os_status resp_status = SII_OS_STATUS_SUCCESS;
	uint8_t *debug_output_path_data = NULL;

	dbg("");

	if (NULL == response_queue) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (NULL == req_data) {
		err("NULL parameter\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}
	debug_output_path = req_data;

	if (sizeof(*debug_output_path) != req_data_size) {
		err("Illegal path for Set Dbg Output Path request\n");
		resp_status = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto send_response;
	}

	debug_output_path_data = SiiOsCalloc("SetDebugOutputPath",
					HM_DEBUG_OUTPUT_SET_PATH_CMD_MLEN, 0);
	if (NULL == debug_output_path_data) {
		err("Out of memory\n");
		resp_status = SII_OS_STATUS_ERR_FAILED;
		goto send_response;
	}
	field_ua_set(debug_output_path->debug_output_path,
			debug_output_path_data,
			HM_DEBUG_OUTPUT_SET_PATH_CMD_PATH);

	retval = send_hostmsg_command((uint16_t)HM_DEBUG_OUTPUT_SET_PATH,
				HM_DEBUG_OUTPUT_SET_PATH_CMD_MLEN,
				debug_output_path_data,
				HM_DEBUG_OUTPUT_SET_PATH_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set Debug Output Path command not sent\n");
		resp_status = retval;
		goto send_response;
	}
	dbg("Set Debug Output Path successful\n");

send_response:
	resp.request_type = SII_SM_REQ_SET_DEBUG_OUTPUT_PATH;
	resp.resp_status = resp_status;
	resp.data = NULL;
	resp.data_size = 0;

	retval = SiiOsQueueSend(response_queue, &resp, sizeof(resp));
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not send response: err=%d\n", retval);
		goto done;
	}

done:
	SiiOsFree(debug_output_path_data);
	return retval;
}

static void diag_cmd_resp_timer_function(void *data)
{
	schedule_work(&diag_cmd_response_work);
}

static void diag_cmd_resp_work_function(struct work_struct *work)
{
	dbg("timed out waiting for diag cmd done\n");

	if (sii6400_diag_sm_is_enabled())
		(void)handle_cmd_done();
}

static enum sii_os_status handle_cmd_done(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	int rv = 0;
	struct sii6400_device_info *devinfo = NULL;
	char *event_data = NULL;
	size_t diag_cmd_length = 0;

	dbg("");

	retval = SiiOsQueueSend(diag_cmd_output_q,
		&diag_sm_info.cur_being_rcvd,
		sizeof(diag_sm_info.cur_being_rcvd));
	if (SII_OS_STATUS_QUEUE_FULL == retval) {
		struct sii6400_diag_cmd_output diag_cmd_output;
		uint32_t diag_cmd_output_size = sizeof(diag_cmd_output);

		err("Diag Cmd Output queue full, discarding oldest output\n");

		retval = SiiOsQueueReceive(diag_cmd_output_q,
					&diag_cmd_output,
					SII_OS_NO_WAIT, &diag_cmd_output_size);
		if (SII_OS_STATUS_SUCCESS != retval) {
			err("Diag Cmd Output queue error = %d\n", retval);
			err("Could not queue latest diag cmd output: err=%d\n",
			    retval);
			SiiOsFree(diag_sm_info.cur_being_rcvd.output);
			diag_sm_info.cur_being_rcvd.output = NULL;
		} else {
			SiiOsFree(diag_cmd_output.output);
			diag_cmd_output.output = NULL;

			retval = SiiOsQueueSend(diag_cmd_output_q,
				&diag_sm_info.cur_being_rcvd,
				sizeof(diag_sm_info.cur_being_rcvd));
			if (SII_OS_STATUS_SUCCESS != retval) {
				err("Failed q'ing latest diag cmd output:%d\n",
				    retval);
				SiiOsFree(diag_sm_info.cur_being_rcvd.output);
				diag_sm_info.cur_being_rcvd.output = NULL;
			}
		}
	} else if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not queue latest diag cmd output: err=%d\n", retval);
		SiiOsFree(diag_sm_info.cur_being_rcvd.output);
		diag_sm_info.cur_being_rcvd.output = NULL;
	}

	/* Clear out current results in prep for next results */
	diag_sm_info.cur_being_rcvd.output = NULL;
	diag_sm_info.cur_being_rcvd.length = 0;
	diag_sm_info.cur_being_rcvd.cmd_id = 0;

	/* Send Cmd Done uevent */
	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto send_next_command;
	}

	diag_cmd_length = strnlen(diag_sm_info.latest_sent.diag_cmd,
					MAX_DIAG_CMD_STR_LEN);

	event_data = SiiOsCalloc("DiagCmdEventData", diag_cmd_length + 4, 0);
	if (NULL == event_data) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto send_next_command;
	}
	rv = scnprintf(event_data, diag_cmd_length + 4, "\"%s\"",
				diag_sm_info.latest_sent.diag_cmd);
	if (0 < rv) {
		(void)send_sii6400_uevent(devinfo->device, DEVICE_EVENT,
					DIAG_CMD_DONE_EVENT, event_data);
	}

send_next_command:
	if (diag_sm_info.latest_sent.is_sync_cmd) {
		struct state_machine_response resp;

		resp.request_type = SII_SM_REQ_SEND_DIAG_COMMAND_SYNC;
		resp.resp_status = SII_OS_STATUS_SUCCESS;
		resp.data = NULL;
		resp.data_size = 0;

		retval = SiiOsQueueSend(send_diag_cmd_sync_resp_q,
					&resp, sizeof(resp));
		if (SII_OS_STATUS_SUCCESS != retval)
			err("Could not send response: err=%d\n", retval);
	}

	diag_sm_info.diag_cmd_can_be_sent = true;

/*done:*/
	SiiOsFree(event_data);
	return retval;
}

enum sii_os_status report_diag_cmd_done(void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint16_t cmd_id = 0;

	dbg("");

	if ((NULL == req_data) ||
	    (HM_DIAG_CMD_DONE_NOTIFY_MLEN != req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	/* Check the cmd_id matches (throw away mismatched done message) */
	cmd_id = (uint16_t)field_ua_extract(req_data,
						HM_DIAG_CMD_DONE_NOTIFY_CMD_ID);
	if (cmd_id != diag_sm_info.cur_being_rcvd.cmd_id) {
		err("Diag Cmd Output data doesn't match Diag Cmd Done msg!\n");
		goto done;
	}

	/* Stop diag cmd response timer */
	if (NULL != diag_cmd_resp_timer) {
		/* Delete the diag cmd response timer */
		retval = SiiOsTimerDelete(diag_cmd_resp_timer);
		if (SII_OS_STATUS_SUCCESS != retval) {
			err("Could not stop diag cmd response timer: %x\n",
			    retval);
		} else {
			diag_cmd_resp_timer = NULL;
			/* Recreate the diag cmd response timer */
			retval = SiiOsTimerCreate("diag_cmd_resp_timer",
					diag_cmd_resp_timer_function,
					NULL, false, 0, false,
					&diag_cmd_resp_timer);
			if (SII_OS_STATUS_SUCCESS != retval) {
				err("Cannot create diag cmd resp timer: %x\n",
				    retval);
				diag_cmd_resp_timer = NULL;
			}
		}
	}

	retval = handle_cmd_done();

done:
	return retval;
}

enum sii_os_status report_diag_cmd_output(void *req_data,
						uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_device_info *devinfo = NULL;
	char *diag_cmd_output_str = NULL;
	size_t length = 0;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if ((NULL == req_data) ||
	    (req_data_size < HM_DIAG_CMD_OUTPUT_NOTIFY_MLEN_MIN) ||
	    (HM_DIAG_CMD_OUTPUT_NOTIFY_MLEN_MAX < req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	length = req_data_size - HM_DIAG_CMD_OUTPUT_NOTIFY_MLEN_MIN;
	if (length) {
		/* Need to add a '\n' on the end of each line */

		/* That is one for '\n' and one for line terminator. */
		diag_cmd_output_str = SiiOsCalloc("DiagCmdOutput",
							length + 2, 0);
		if (NULL == diag_cmd_output_str) {
			err("Out of memory\n");
			retval = SII_OS_STATUS_ERR_FAILED;
			goto done;
		}

		memcpy(diag_cmd_output_str,
			(void *)FIELD_UA_BYTE_ADDRESS(req_data,
					HM_DIAG_CMD_OUTPUT_NOTIFY_DATA0),
			length);
		diag_cmd_output_str[length] = '\n';
		diag_cmd_output_str[length + 1] = '\0';

		if (NULL == diag_sm_info.cur_being_rcvd.output) {
			diag_sm_info.cur_being_rcvd.length =
							length + 1;
			diag_sm_info.cur_being_rcvd.output =
							diag_cmd_output_str;
			diag_cmd_output_str = NULL;
		} else {
			char *appended_diag_cmd_output_str = NULL;
			size_t new_length = diag_sm_info.cur_being_rcvd.length +
						length;

			if ((new_length + 2) <= PAGE_SIZE) {
				appended_diag_cmd_output_str = SiiOsRealloc(
					"DiagCmdOutput",
					diag_sm_info.cur_being_rcvd.output,
					new_length + 2, 0);
				if (NULL == appended_diag_cmd_output_str) {
					err("Out of memory\n");
					goto done;
				}
				/* Previous buffer was freed in Realloc */
				diag_sm_info.cur_being_rcvd.output = NULL;

				strlcat(appended_diag_cmd_output_str,
					diag_cmd_output_str, new_length + 2);

				/* Save new command output string */
				diag_sm_info.cur_being_rcvd.output =
						appended_diag_cmd_output_str;
				diag_sm_info.cur_being_rcvd.length =
						new_length + 1;
			} else {
				/* Push in the previous buffer */
				retval = SiiOsQueueSend(diag_cmd_output_q,
					&diag_sm_info.cur_being_rcvd,
					sizeof(diag_sm_info.cur_being_rcvd));
				if (SII_OS_STATUS_QUEUE_FULL == retval) {
					warn("Diag Cmd Output queue full\n");
					goto done;
				} else if (SII_OS_STATUS_SUCCESS != retval) {
					err("Failed q'ing latest output:%d\n",
					    retval);
					goto done;
				}

				/* The first result after the push in */
				diag_sm_info.cur_being_rcvd.output =
					diag_cmd_output_str;
				diag_sm_info.cur_being_rcvd.length =
					length + 1;
				diag_sm_info.cur_being_rcvd.cmd_id =
					diag_sm_info.latest_sent.cmd_id;
				diag_cmd_output_str = NULL;
			}
		}
	}

done:
	SiiOsFree(diag_cmd_output_str);
	return retval;
}

enum sii_os_status report_debug_output(void *req_data, uint32_t req_data_size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	int result = -1;
	struct sii6400_device_info *devinfo = NULL;
	struct file *filp = NULL;
	struct debug_output *debug_output = NULL;
	size_t length = 0;
	size_t debug_output_length = 0;
	struct timespec unix_time;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	if ((NULL == req_data) ||
	    (req_data_size < HM_DEBUG_OUTPUT_NOTIFY_MLEN_MIN) ||
	    (HM_DEBUG_OUTPUT_NOTIFY_MLEN_MAX < req_data_size)) {
		err("Invalid parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	length = req_data_size - HM_DEBUG_OUTPUT_NOTIFY_MLEN_MIN;
	if (length) {
		debug_output_length = length + sizeof(struct debug_output);
		debug_output = SiiOsCalloc("DebugOutput",
						debug_output_length, 0);
		if (NULL == debug_output) {
			err("Out of memory\n");
			retval = SII_OS_STATUS_ERR_FAILED;
			goto done;
		}

		getnstimeofday(&unix_time);

		debug_output->magic_number = DEBUG_OUTPUT_MAGIC_NUMBER;
		debug_output->time_stamp = unix_time.tv_sec;
		memcpy(debug_output->debug_msg_data,
			(void *)FIELD_UA_BYTE_ADDRESS(req_data,
					HM_DEBUG_OUTPUT_NOTIFY_NOTIFY_DATA0),
			length);

		filp = devinfo->debug_log_filp;
		if (NULL == filp) {
			warn("No debug output logfile open\n");
		} else if ((NULL == filp->f_op) ||
			   (NULL == filp->f_op->write)) {
			warn("Cannot write to debug output logfile\n");
		} else {
			mm_segment_t oldfs;

			oldfs = get_fs();
			set_fs(KERNEL_DS);

			/* f_pos should always be at the EOF,
			 * since it was opened with O_APPEND */
			result = filp->f_op->write(filp,
				(unsigned char __user __force *)debug_output,
				debug_output_length, &filp->f_pos);
			if (result < 0)
				err("Debug output logfile write failed\n");
			set_fs(oldfs);
		}
	}

done:
	SiiOsFree(debug_output);
	return retval;
}

static enum sii_os_status send_my_debug_output_path(
			struct sii6400_debug_output_path *debug_output_path)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint8_t *debug_output_path_data = NULL;

	dbg("");

	if (NULL == debug_output_path) {
		err("NULL parameter\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	debug_output_path_data = SiiOsCalloc("SendDebugOutputPath",
				HM_DEBUG_OUTPUT_SET_PATH_CMD_MLEN, 0);
	if (NULL == debug_output_path_data) {
		err("Out of memory\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	field_ua_set(debug_output_path->debug_output_path,
			debug_output_path_data,
			HM_DEBUG_OUTPUT_SET_PATH_CMD_PATH);

	retval = send_hostmsg_command((uint16_t)HM_DEBUG_OUTPUT_SET_PATH,
				HM_DEBUG_OUTPUT_SET_PATH_CMD_MLEN,
				debug_output_path_data,
				HM_DEBUG_OUTPUT_SET_PATH_RESPONSE_TIMEOUT,
				NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Set Debug Output Path command not sent\n");
		goto done;
	}
	dbg("Set Debug Output Path successful\n");

done:
	SiiOsFree(debug_output_path_data);
	return retval;
}

/*
 * Task that handles the Diagnostic Command queue.
 */
static void diag_cmd_handler(struct work_struct *work)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	struct sii6400_diag_cmd diag_cmd;
	uint32_t diag_cmd_size = sizeof(diag_cmd);
	uint16_t resp_data_size = 0;
	void *resp_data = NULL;
	uint8_t *diag_cmd_data = NULL;
	char *diag_cmd_to_send = NULL;
	uint16_t diag_cmd_to_send_length = 0;
	bool diag_cmd_to_send_is_sync_cmd = false;

	dbg("");

	diag_cmd.diag_cmd = NULL;
	while (!sii6400_module_is_exiting() && sii6400_diag_sm_is_enabled()) {
		if (!diag_sm_info.diag_cmd_can_be_sent) {
			/* If a command has been sent, need to wait for the
			 * command processing to be completed before sending the
			 * next command on the queue. */
			msleep(WAIT_TIME_FOR_DIAG_CMD_BUSY);
			continue;
		}

		diag_cmd_size = sizeof(diag_cmd);
		memset(&diag_cmd, 0, sizeof(diag_cmd));
		retval = SiiOsQueueReceive(diag_cmd_q, &diag_cmd,
				WAIT_TIME_FOR_DIAG_CMD_QUEUE, &diag_cmd_size);
		if ((SII_OS_STATUS_QUEUE_EMPTY == retval) ||
		    (SII_OS_STATUS_TIMEOUT == retval)) {
			continue;
		} else if (SII_OS_STATUS_SUCCESS != retval) {
			err("Diag Cmd queue error = %d\n", retval);
			msleep(WAIT_TIME_FOR_DIAG_CMD_QUEUE_FAILURE);
			continue;
		}

		diag_cmd_to_send = diag_cmd.diag_cmd;
		diag_cmd_to_send_length = diag_cmd.length;
		diag_cmd_to_send_is_sync_cmd = diag_cmd.is_sync_cmd;

		if (HM_DIAG_COMMAND_CMD_MLEN_MAX < diag_cmd_to_send_length)
			diag_cmd_to_send_length = HM_DIAG_COMMAND_CMD_MLEN_MAX;

		/* Free the previous command in the latest-command-queued and
		 * save the new one. */
		if (diag_cmd_to_send_is_sync_cmd) {
			SiiOsFree(diag_sm_info.latest_sync_queued.diag_cmd);
			diag_sm_info.latest_sync_queued.diag_cmd =
					diag_cmd_to_send;

			diag_sm_info.latest_sync_queued.length =
					diag_cmd_to_send_length;
			diag_sm_info.latest_sync_queued.cmd_id = 0;
		} else {
			SiiOsFree(diag_sm_info.latest_async_queued.diag_cmd);
			diag_sm_info.latest_async_queued.diag_cmd =
					diag_cmd_to_send;

			diag_sm_info.latest_async_queued.length =
					diag_cmd_to_send_length;
			diag_sm_info.latest_async_queued.cmd_id = 0;
		}

		/* Send the diag command to the firmware. */
		diag_cmd_data = SiiOsCalloc("SendDiagCmdData",
						diag_cmd_to_send_length, 0);
		if (NULL == diag_cmd_data) {
			err("Out of memory\n");
			goto done;
		}

		memcpy(diag_cmd_data,
				diag_cmd_to_send, diag_cmd_to_send_length);

		retval = send_hostmsg_command((uint16_t)HM_DIAG_COMMAND,
					diag_cmd_to_send_length,
					diag_cmd_data,
					HM_DIAG_COMMAND_RESPONSE_TIMEOUT,
					&resp_data_size, &resp_data);
		if (SII_OS_STATUS_SUCCESS != retval) {
			err("Send Diagnostic Command not sent\n");
			goto done;
		}
		dbg("Send Diagnostic Command successful\n");

		if (HM_DIAG_COMMAND_CMD_RSP_MLEN != resp_data_size) {
			warn("Unexpected response data size %d\n",
			     resp_data_size);
			goto done;
		}

		/* If the command was sent successfully, disable sending
		 * any other commands from the queue until this command
		 * has sent the cmd done notification. */
		diag_sm_info.diag_cmd_can_be_sent = false;

		/* Start the diag cmd response timer, in case the cmd
		 * done notification is never received. */
		if (NULL != diag_cmd_resp_timer) {
			retval = SiiOsTimerSchedule(diag_cmd_resp_timer,
						MAX_WAIT_FOR_DIAG_CMD_RESPONSE);
			if (SII_OS_STATUS_SUCCESS != retval) {
				err("Diag Cmd response timer not started: %x\n",
				    retval);
			}
		}

		SiiOsFree(diag_sm_info.latest_sent.diag_cmd);

		diag_sm_info.latest_sent.diag_cmd =
					kstrndup(diag_cmd_to_send,
							diag_cmd_to_send_length,
							GFP_KERNEL);
		diag_sm_info.latest_sent.length =
					diag_cmd_to_send_length;
		diag_sm_info.latest_sent.cmd_id =
					(uint16_t)field_ua_extract(resp_data,
						HM_DIAG_COMMAND_CMD_RSP_CMD_ID);
		diag_sm_info.latest_sent.is_sync_cmd =
					diag_cmd_to_send_is_sync_cmd;

		if (NULL != diag_sm_info.cur_being_rcvd.output) {
			/* This should never happen */
			err("Still receiving output from another command!\n");
		} else {
			diag_sm_info.cur_being_rcvd.cmd_id =
				diag_sm_info.latest_sent.cmd_id;
		}

done:
		SiiOsFree(resp_data);
		resp_data = NULL;

		SiiOsFree(diag_cmd_data);
		diag_cmd_data = NULL;

		/* If command sending failed, unblock a sync command */
		if ((diag_sm_info.diag_cmd_can_be_sent) &&
		    (diag_cmd_to_send_is_sync_cmd)) {
			struct state_machine_response resp;

			resp.request_type = SII_SM_REQ_SEND_DIAG_COMMAND_SYNC;
			resp.resp_status = SII_OS_STATUS_ERR_FAILED;
			resp.data = NULL;
			resp.data_size = 0;

			retval = SiiOsQueueSend(send_diag_cmd_sync_resp_q,
							&resp, sizeof(resp));
			if (SII_OS_STATUS_SUCCESS != retval)
				err("Could not send response: err=%d\n",
				    retval);
		}
	}

	/* Clear out the Diag Cmd queue */
	diag_cmd_size = sizeof(diag_cmd);
	while (SII_OS_STATUS_SUCCESS == SiiOsQueueReceive(diag_cmd_q,
				&diag_cmd, SII_OS_NO_WAIT, &diag_cmd_size)) {
		SiiOsFree(diag_cmd.diag_cmd);
		diag_cmd.diag_cmd = NULL;
		diag_cmd_size = sizeof(diag_cmd);
	}
}

/*
 * Initialize the Diag state when firmware is first loaded
 */
enum sii_os_status sii6400_diag_init(struct sii6400_device_info *devinfo)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	enum sii_os_status rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	rv = send_my_debug_output_path(&devinfo->my_debug_output_path);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("Failed to send my Debug Output Path\n");
		retval = rv;
	}

done:
	return retval;
}

bool sii6400_diag_sm_is_enabled(void)
{
	return diag_sm_is_enabled;
}

void sii6400_diag_sm_enable(void)
{
	enum sii_os_status rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == diag_cmd_resp_timer) {
		rv = SiiOsTimerCreate("diag_cmd_resp_timer",
					diag_cmd_resp_timer_function,
					NULL, false, 0, false,
					&diag_cmd_resp_timer);
		if (SII_OS_STATUS_SUCCESS != rv)
			err("diag_cmd_resp_timer create failed\n");
	}

	diag_sm_is_enabled = true;
	diag_sm_info.diag_cmd_can_be_sent = true;

	/* Queue Diag Cmd Handler work (must be after sm is enabled) */
	if (!queue_work(diag_cmd_wq, &my_diag_cmd_work->diagnostic_cmd_work))
		dbg("Diag Cmd Handler task already queued\n");
}

void sii6400_diag_sm_disable(void)
{
	dbg("");

	diag_sm_is_enabled = false;
	diag_sm_info.diag_cmd_can_be_sent = false;

	if (NULL != diag_cmd_resp_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(diag_cmd_resp_timer)) {
			err("Failed to delete diag cmd response timer\n");
		}
		diag_cmd_resp_timer = NULL;
	}

	SiiOsFree(diag_sm_info.latest_async_queued.diag_cmd);
	diag_sm_info.latest_async_queued.diag_cmd = NULL;

	SiiOsFree(diag_sm_info.latest_sync_queued.diag_cmd);
	diag_sm_info.latest_sync_queued.diag_cmd = NULL;

	SiiOsFree(diag_sm_info.latest_sent.diag_cmd);
	diag_sm_info.latest_sent.diag_cmd = NULL;

	SiiOsFree(diag_sm_info.cur_being_rcvd.output);
	diag_sm_info.cur_being_rcvd.output = NULL;

	/* Clear out the diag command output queue */
	if (NULL != diag_cmd_output_q) {
		struct sii6400_diag_cmd_output diag_cmd_output;
		uint32_t diag_cmd_output_size = sizeof(diag_cmd_output);

		while (SII_OS_STATUS_SUCCESS ==
				SiiOsQueueReceive(diag_cmd_output_q,
					&diag_cmd_output, SII_OS_NO_WAIT,
					&diag_cmd_output_size)) {
			SiiOsFree(diag_cmd_output.output);
			diag_cmd_output.output = NULL;
			diag_cmd_output_size = sizeof(diag_cmd_output);
		}
	}
}

int sii6400_diag_sm_init(void)
{
	int retval = 0;
	enum sii_os_status rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	memset(&diag_sm_info, 0, sizeof(struct sii6400_diag_sm_info));

	rv = SiiOsQueueCreate("diag_cmd_q",
				sizeof(struct sii6400_diag_cmd),
				DIAG_CMD_QUEUE_SIZE,
				&diag_cmd_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("diag_cmd_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("diag_cmd_output_q",
				sizeof(struct sii6400_diag_cmd_output),
				DIAG_CMD_OUTPUT_QUEUE_SIZE,
				&diag_cmd_output_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("diag_cmd_output_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_diag_cmd_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_diag_cmd_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_diag_cmd_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("send_diag_cmd_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&send_diag_cmd_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("send_diag_cmd_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_diag_command_sync_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_diag_cmd_sync_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_diag_cmd_sync_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("send_diag_command_sync_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&send_diag_cmd_sync_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("send_diag_cmd_sync_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("get_diag_cmd_output_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&get_diag_cmd_output_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("get_diag_cmd_output_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsQueueCreate("set_debug_output_path_q",
				sizeof(struct state_machine_response),
				RESPONSE_QUEUE_SIZE,
				&set_debug_output_path_resp_q);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("set_debug_output_path_resp_q create failed\n");
		retval = -EFAULT;
		goto done;
	}

	diag_cmd_wq = alloc_workqueue("diag_cmd_wq",
					WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (NULL == diag_cmd_wq) {
		err("diag_cmd_wq create failed\n");
		retval = -EFAULT;
		goto done;
	}

	my_diag_cmd_work = SiiOsCalloc("diag_cmd_work",
					sizeof(struct diag_cmd_work), 0);
	if (NULL == my_diag_cmd_work) {
		err("Out of Memory!\n");
		retval = -ENOMEM;
		goto done;
	}

	INIT_WORK(&my_diag_cmd_work->diagnostic_cmd_work, diag_cmd_handler);

done:
	return retval;
}

void sii6400_diag_sm_exit(void)
{
	dbg("");

	SiiOsFree(diag_sm_info.latest_async_queued.diag_cmd);
	diag_sm_info.latest_async_queued.diag_cmd = NULL;

	SiiOsFree(diag_sm_info.latest_sync_queued.diag_cmd);
	diag_sm_info.latest_sync_queued.diag_cmd = NULL;

	SiiOsFree(diag_sm_info.latest_sent.diag_cmd);
	diag_sm_info.latest_sent.diag_cmd = NULL;

	SiiOsFree(diag_sm_info.cur_being_rcvd.output);
	diag_sm_info.cur_being_rcvd.output = NULL;

	if (NULL != my_diag_cmd_work) {
		cancel_work_sync(&my_diag_cmd_work->diagnostic_cmd_work);
		SiiOsFree(my_diag_cmd_work);
		my_diag_cmd_work = NULL;
	}

	if (NULL != diag_cmd_wq) {
		destroy_workqueue(diag_cmd_wq);
		diag_cmd_wq = NULL;
	}

	if (NULL != diag_cmd_q) {
		if (SII_OS_STATUS_SUCCESS !=
					SiiOsQueueDelete(diag_cmd_q)) {
			err("diag_cmd_q delete failed\n");
		}
		diag_cmd_q = NULL;
	}

	/* Clear out the diag command output queue */
	if (NULL != diag_cmd_output_q) {
		struct sii6400_diag_cmd_output diag_cmd_output;
		uint32_t diag_cmd_output_size = sizeof(diag_cmd_output);

		while (SII_OS_STATUS_SUCCESS ==
			SiiOsQueueReceive(diag_cmd_output_q,
					&diag_cmd_output, SII_OS_NO_WAIT,
					&diag_cmd_output_size)) {
			SiiOsFree(diag_cmd_output.output);
			diag_cmd_output.output = NULL;
			diag_cmd_output_size = sizeof(diag_cmd_output);
		}

		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(diag_cmd_output_q)) {
			err("diag_cmd_output_q delete failed\n");
		}
		diag_cmd_output_q = NULL;
	}

	if (NULL != get_diag_cmd_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsQueueDelete(get_diag_cmd_resp_q)) {
			err("get_diag_cmd_resp_q delete failed\n");
		}
		get_diag_cmd_resp_q = NULL;
	}

	if (NULL != send_diag_cmd_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(send_diag_cmd_resp_q)) {
			err("send_diag_cmd_resp_q delete failed\n");
		}
		send_diag_cmd_resp_q = NULL;
	}

	if (NULL != get_diag_cmd_sync_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_diag_cmd_sync_resp_q)) {
			err("get_diag_cmd_sync_resp_q delete failed\n");
		}
		get_diag_cmd_sync_resp_q = NULL;
	}

	if (NULL != send_diag_cmd_sync_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(send_diag_cmd_sync_resp_q)) {
			err("send_diag_cmd_sync_resp_q delete failed\n");
		}
		send_diag_cmd_sync_resp_q = NULL;
	}

	if (NULL != get_diag_cmd_output_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(get_diag_cmd_output_resp_q)) {
			err("get_diag_cmd_output_resp_q delete failed\n");
		}
		get_diag_cmd_output_resp_q = NULL;
	}

	if (NULL != set_debug_output_path_resp_q) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsQueueDelete(set_debug_output_path_resp_q)) {
			err("set_debug_output_path_resp_q delete failed\n");
		}
		set_debug_output_path_resp_q = NULL;
	}

	if (NULL != diag_cmd_resp_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(diag_cmd_resp_timer)) {
			err("Failed to delete diag cmd response timer\n");
		}
		diag_cmd_resp_timer = NULL;
	}
}

