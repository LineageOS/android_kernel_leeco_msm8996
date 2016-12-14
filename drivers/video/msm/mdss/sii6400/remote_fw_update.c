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

#include <linux/firmware.h>
#include <linux/crc32.h>
#include "remote_fw_update.h"
#include "sii6400.h"
#include "device.h"
#include "state_machine.h"

#define CHUNK_SIZE				(1024)
#define BLOCK_SIZE				(512)
#define MAX_REMOTE_FW_UPDATE_PERCENT_LENGTH	(4)
#define REMOTE_FW_UPDATE_TIME			(10*60*1000)

static struct workqueue_struct *remote_fw_update_wq;
static struct work_struct *remote_fw_update_work;
static enum sii6400_remote_fw_update_state state = REMOTE_FW_UPDATE_STATE_OFF;
static bool remote_fw_update_abort;
static struct SiiOsTimer *remote_fw_update_timer;


static const char *
get_remote_fw_update_failed_reason_string(enum sii_os_status reason)
{
	const char *reason_str = NULL;

	switch (reason) {
	case SII_OS_STATUS_ERR_INVALID_PARAM:
		reason_str = "{\"reason\":\"input_error\"}";
		break;
	case SII_OS_STATUS_ERR_NOT_AVAIL:
	case SII_OS_STATUS_TIMEOUT:
	case SII_OS_STATUS_ERR_FAILED:
	default:
		reason_str = "{\"reason\":\"update_failed\"}";
		break;
	}
	return reason_str;
}

static void remote_fw_update_timer_function(void *data)
{
	const char *reason_str = NULL;
	struct sii6400_device_info *devinfo = NULL;

	dbg("");

	remote_fw_update_abort = true;
	devinfo = get_sii6400_devinfo();
	if (NULL != devinfo) {
		reason_str = get_remote_fw_update_failed_reason_string(
				SII_OS_STATUS_TIMEOUT);
		(void)send_sii6400_uevent(devinfo->device,
					REMOTE_FW_UPDATE_EVENT,
					REMOTE_FW_UPDATE_FAILED, reason_str);
	}
}

static enum sii_os_status
remote_fw_update_verify_sink_firmware(const void *data, uint32_t size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	uint32_t calc_header_crc = ~0;
	uint32_t calc_firmware_crc = ~0;
	uint32_t firmware_size = 0;
	uint32_t firmware_crc = 0;
	struct sii6400_firmware_file_hdr_v02 *fw_header = NULL;

	dbg("");

	if (NULL == data) {
		err("Invalid data passed in\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	if (size < sizeof(struct sii6400_firmware_file_hdr_v02)) {
		err("Invalid size passed in\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	fw_header = (struct sii6400_firmware_file_hdr_v02 *)(data);

	calc_header_crc = ~crc32(calc_header_crc, (uint8_t *)(fw_header),
				sizeof(struct sii6400_firmware_file_hdr_v02) -
					sizeof(uint32_t));
	if (calc_header_crc != fw_header->header_crc32) {
		err("Fw header CRC mismatch: expected = 0x%08x, got = 0x%08x\n",
		    fw_header->header_crc32, calc_header_crc);
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	dbg("Firmware header CRC matches\n");

	firmware_size = fw_header->firmware_size;
	if (size != firmware_size +
			sizeof(struct sii6400_firmware_file_hdr_v02) +
			sizeof(uint32_t)) {
		err("Firmware body size mismatch\n");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	firmware_crc = *(uint32_t *)((uint8_t *)data + size - sizeof(uint32_t));
	calc_firmware_crc = ~crc32(calc_firmware_crc,
			(uint8_t *)((uint8_t *)data +
				sizeof(struct sii6400_firmware_file_hdr_v02)),
			firmware_size);
	if (calc_firmware_crc != firmware_crc) {
		err("Fw body CRC mismatch: expected = 0x%08x, got = 0x%08x\n",
		    firmware_crc, calc_firmware_crc);
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}
	dbg("Firmware body CRC matches\n");

done:
	return retval;
}

static enum sii_os_status
remote_fw_update_send_start_req(uint32_t *customer_key)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == customer_key) {
		err("Invalid customer key pointer\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	retval = send_hostmsg_command((uint16_t)HM_RMT_FW_UPDATE_START_REQ,
			HM_RMT_FW_UPDATE_START_REQ_CMD_MLEN,
			customer_key,
			HM_RMT_FW_UPDATE_START_REQ_RESPONSE_TIMEOUT, NULL,
			NULL);

done:
	return retval;
}

static enum sii_os_status
send_remote_fw_update_info(struct sii6400_firmware_file_hdr_v02 *fw_header)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	void *remote_fw_update_info = NULL;

	dbg("");

	if (NULL == fw_header) {
		err("Invalid firmware header pointer\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	remote_fw_update_info = SiiOsCalloc("RemoteFwUpdateInfo",
				HM_RMT_FW_UPDATE_INFO_CMD_MLEN, 0);
	if (NULL == remote_fw_update_info) {
		err("out of memory");
		retval = SII_OS_STATUS_ERR_FAILED;
		goto done;
	}

	field_ua_set(fw_header->firmware_size, remote_fw_update_info,
			HM_RMT_FW_UPDATE_INFO_CMD_FW_SIZE);
	field_ua_set(fw_header->firmware_entry, remote_fw_update_info,
			HM_RMT_FW_UPDATE_INFO_CMD_ENTRYPOINT);
	memcpy((void *)FIELD_UA_BYTE_ADDRESS(remote_fw_update_info,
			HM_RMT_FW_UPDATE_INFO_CMD_MIC0),
			fw_header->mic, HM_RMT_FW_UPDATE_INFO_CMD_MIC_LEN);
	retval = send_hostmsg_command((uint16_t)HM_RMT_FW_UPDATE_INFO,
			HM_RMT_FW_UPDATE_INFO_CMD_MLEN,
			remote_fw_update_info,
			HM_RMT_FW_UPDATE_INFO_RESPONSE_TIMEOUT, NULL, NULL);

done:
	SiiOsFree(remote_fw_update_info);
	return retval;
}

static enum sii_os_status remote_fw_update_send_chunk_data(uint8_t *data,
	uint32_t offset, uint32_t size)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	void *send_start = NULL;
	void *send_end = NULL;
	uint32_t calc_data_crc = 0;
	uint32_t block_num = 0;
	uint32_t trail_length = 0;
	uint32_t i;

	dbg("");

	if (NULL == data) {
		retval = SII_OS_STATUS_ERR_FAILED;
		err("Invalid chunk data pointer\n");
		goto done;
	}

	send_start = SiiOsCalloc("remote_fw_update_send_start",
				HM_RMT_FW_UPDATE_DATA_SEND_START_CMD_MLEN,
				0);
	if (NULL == send_start) {
		retval = SII_OS_STATUS_ERR_FAILED;
		err("out of memory\n");
		goto done;
	}

	field_ua_set(offset, send_start,
			HM_RMT_FW_UPDATE_DATA_SEND_START_CMD_START_OFFSET);

	retval = send_hostmsg_command(
			(uint16_t)HM_RMT_FW_UPDATE_DATA_SEND_START,
			HM_RMT_FW_UPDATE_DATA_SEND_START_CMD_MLEN,
			send_start,
			HM_RMT_FW_UPDATE_DATA_SEND_START_RESPONSE_TIMEOUT,
			NULL, NULL);
	if (SII_OS_STATUS_SUCCESS != retval)
		goto done;

	block_num = size / BLOCK_SIZE;
	trail_length = size % BLOCK_SIZE;
	for (i = 0; i < block_num; i++) {
		retval = send_hostmsg_command(
				(uint16_t)HM_RMT_FW_UPDATE_DATA_SEND,
				BLOCK_SIZE, data + i * BLOCK_SIZE,
				HM_RMT_FW_UPDATE_DATA_SEND_RESPONSE_TIMEOUT,
				NULL, NULL);
		if (SII_OS_STATUS_SUCCESS != retval)
			goto done;
	}
	if (trail_length > 0) {
		retval = send_hostmsg_command(
				(uint16_t)HM_RMT_FW_UPDATE_DATA_SEND,
				trail_length, data + block_num * BLOCK_SIZE,
				HM_RMT_FW_UPDATE_DATA_SEND_RESPONSE_TIMEOUT,
				NULL, NULL);
		if (SII_OS_STATUS_SUCCESS != retval)
			goto done;
	}

	calc_data_crc = ~crc32(~0, data, size);

	send_end = SiiOsCalloc("remote_fw_update_send_end",
				HM_RMT_FW_UPDATE_DATA_SEND_END_MLEN, 0);
	if (NULL == send_end) {
		retval = SII_OS_STATUS_ERR_FAILED;
		err("out of memory\n");
		goto done;
	}

	field_ua_set(calc_data_crc, send_end,
			HM_RMT_FW_UPDATE_DATA_SEND_END_CRC32);

	retval = send_hostmsg_command(
			(uint16_t)HM_RMT_FW_UPDATE_DATA_SEND_END,
			HM_RMT_FW_UPDATE_DATA_SEND_END_MLEN, send_end,
			HM_RMT_FW_UPDATE_DATA_SEND_END_RESPONSE_TIMEOUT,
			NULL, NULL);

done:
	SiiOsFree(send_start);
	SiiOsFree(send_end);
	return retval;
}

static void remote_fw_update_work_item(struct work_struct *work)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;
	int error = 0;
	const struct firmware *fw_entry = NULL;
	struct sii6400_device_info *devinfo = NULL;
	struct sii6400_firmware_file_hdr_v02 *fw_header = NULL;
	uint32_t chunk_num = 0;
	uint32_t trail_size = 0;
	uint8_t *fw_data = NULL;
	const char *reason_str = NULL;
	uint32_t percentage = 0;
	char event_data[MAX_REMOTE_FW_UPDATE_PERCENT_LENGTH + 4] = {0};
	uint32_t i;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		goto done;
	}

	error = request_firmware(&fw_entry, "firmware_base_sink.fw",
				devinfo->device);
	if ((0 != error) || (NULL == fw_entry)) {
		err("Failed to loading sink firmware\n");
		retval = SII_OS_STATUS_ERR_INVALID_PARAM;
		reason_str = get_remote_fw_update_failed_reason_string(retval);
		goto loadfail;
	}
	/* After this, need to release the firmware... */

	state = REMOTE_FW_UPDATE_STATE_ON;

	/* verify firmware */
	retval = remote_fw_update_verify_sink_firmware(fw_entry->data,
							fw_entry->size);
	if (retval != SII_OS_STATUS_SUCCESS) {
		reason_str = get_remote_fw_update_failed_reason_string(retval);
		goto release;
	}

	if (remote_fw_update_abort)
		goto release;

	/* send Remote Firmware Update start */
	fw_header = (struct sii6400_firmware_file_hdr_v02 *)(fw_entry->data);
	/* sink firmware is always header 2 revision */
	fw_data = (uint8_t *)(fw_entry->data +
				sizeof(struct sii6400_firmware_file_hdr_v02));
	retval = remote_fw_update_send_start_req(&(fw_header->customer_id));
	if (retval != SII_OS_STATUS_SUCCESS) {
		reason_str = get_remote_fw_update_failed_reason_string(retval);
		goto release;
	}
	/* After this, need to inform firmware of Remote Firmware Update end */

	if (remote_fw_update_abort)
		goto abort;

	/* send update info */
	retval = send_remote_fw_update_info(fw_header);
	if (retval != SII_OS_STATUS_SUCCESS) {
		reason_str = get_remote_fw_update_failed_reason_string(retval);
		goto abort;
	}

	/* send chunk data */
	chunk_num = (fw_header->firmware_size) / CHUNK_SIZE;
	trail_size = (fw_header->firmware_size) % CHUNK_SIZE;
	for (i = 0; i < chunk_num; i++) {
		if (remote_fw_update_abort)
			goto abort;
		retval = remote_fw_update_send_chunk_data(fw_data +
						i * CHUNK_SIZE,
						i * CHUNK_SIZE, CHUNK_SIZE);
		if (SII_OS_STATUS_SUCCESS != retval) {
			reason_str = get_remote_fw_update_failed_reason_string(
						retval);
			goto abort;
		}
		percentage = (i * 100) / chunk_num;
		scnprintf(event_data, MAX_REMOTE_FW_UPDATE_PERCENT_LENGTH + 4,
				"\"%d\"", percentage);
		(void)send_sii6400_uevent(devinfo->device,
					REMOTE_FW_UPDATE_EVENT,
					REMOTE_FW_UPDATE_PROGRESS, event_data);
	}
	if (trail_size > 0) {
		if (remote_fw_update_abort)
			goto abort;
		retval = remote_fw_update_send_chunk_data(
					fw_data + chunk_num * CHUNK_SIZE,
					chunk_num * CHUNK_SIZE, trail_size);
		if (SII_OS_STATUS_SUCCESS != retval) {
			reason_str = get_remote_fw_update_failed_reason_string(
						retval);
			goto abort;
		}
		percentage = 100;
		scnprintf(event_data, MAX_REMOTE_FW_UPDATE_PERCENT_LENGTH + 4,
				"\"%d\"", percentage);
		(void)send_sii6400_uevent(devinfo->device,
					REMOTE_FW_UPDATE_EVENT,
					REMOTE_FW_UPDATE_PROGRESS, event_data);
	}

	(void)send_sii6400_uevent(devinfo->device, REMOTE_FW_UPDATE_EVENT,
			REMOTE_FW_UPDATE_COMPLETE, NULL);

abort:
	/* send Remote Firmware Update end */
	(void)send_hostmsg_command((uint16_t)HM_RMT_FW_UPDATE_END,
				HM_RMT_FW_UPDATE_END_MLEN,
				NULL, HM_RMT_FW_UPDATE_END_RESPONSE_TIMEOUT,
				NULL, NULL);

release:
	/* release the firmware data */
	release_firmware(fw_entry);

loadfail:
	if (SII_OS_STATUS_SUCCESS != retval) {
		(void)send_sii6400_uevent(devinfo->device,
				REMOTE_FW_UPDATE_EVENT,
				REMOTE_FW_UPDATE_FAILED, reason_str);
	}

done:
	/* delete the Remote Firmware Update timer */
	if (NULL != remote_fw_update_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(remote_fw_update_timer)) {
			err("Could not delete Remote Firmware Update timer\n");
		} else {
			remote_fw_update_timer = NULL;
		}
	}
	state = REMOTE_FW_UPDATE_STATE_OFF;
}

int sii6400_remote_fw_update_init(void)
{
	int retval = 0;

	dbg("");

	remote_fw_update_wq = alloc_workqueue("remote_fw_update_wq",
					WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (NULL == remote_fw_update_wq) {
		err("remote_fw_update_wq create failed\n");
		retval = -EFAULT;
		goto done;
	}
	remote_fw_update_work = SiiOsCalloc("remote_fw_update_work",
					sizeof(struct work_struct), 0);
	if (NULL == remote_fw_update_work) {
		err("Out of Memory!\n");
		retval = -ENOMEM;
		goto done;
	}

	INIT_WORK(remote_fw_update_work, remote_fw_update_work_item);
done:
	return retval;
}

void sii6400_remote_fw_update_exit(void)
{
	dbg("");

	if (NULL != remote_fw_update_work) {
		cancel_work_sync(remote_fw_update_work);
		SiiOsFree(remote_fw_update_work);
		remote_fw_update_work = NULL;
	}

	if (NULL != remote_fw_update_wq) {
		destroy_workqueue(remote_fw_update_wq);
		remote_fw_update_wq = NULL;
	}
	if (NULL != remote_fw_update_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(remote_fw_update_timer)) {
			err("Failed to delete Remote Firmware Update timer\n");
		}
		remote_fw_update_timer = NULL;
	}
}

enum sii_os_status sii6400_remote_fw_update_start(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (REMOTE_FW_UPDATE_STATE_OFF == state) {
		if (!queue_work(remote_fw_update_wq, remote_fw_update_work)) {
			dbg("remote_fw_update work item already queued\n");
			retval = SII_OS_STATUS_ERR_FAILED;
			goto done;
		}
		if (NULL != remote_fw_update_timer) {
			retval = SiiOsTimerDelete(remote_fw_update_timer);
			if (SII_OS_STATUS_SUCCESS != retval) {
				err("remote_fw_update_timer delete failed\n");
				goto done;
			}
			remote_fw_update_timer = NULL;
		}
		retval = SiiOsTimerCreate("remote_fw_update_timer",
					remote_fw_update_timer_function,
					NULL, false, 0, false,
					&remote_fw_update_timer);
		if (SII_OS_STATUS_SUCCESS != retval) {
			err("Failed to create Remote Firmware Update timer\n");
			goto done;
		}
		retval = SiiOsTimerSchedule(remote_fw_update_timer,
						REMOTE_FW_UPDATE_TIME);
		if (SII_OS_STATUS_SUCCESS != retval) {
			err("Failed to start Remote Firmware Update timer\n");
			goto done;
		}
		remote_fw_update_abort = false;
	} else {
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
	}

done:
	return retval;
}

enum sii_os_status sii6400_remote_fw_update_abort(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (REMOTE_FW_UPDATE_STATE_ON == state)
		remote_fw_update_abort = true;
	else
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
	return retval;
}

