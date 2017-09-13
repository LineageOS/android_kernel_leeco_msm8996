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
/**
 * @file host_msg.c
 *
 * @brief This file implements AP<->BB SPI messaging
 *
 ******************************************************************************/

#include <linux/kernel.h>

#include "osal.h"
#include "device.h"
#include "sii6400.h"
#include "state_machine.h"
#include "struct_bitfield.h"
#include "host_msg_private.h"

/* Semaphore and mutex declarations */

/* Controls sequence number update. */
static DEFINE_MUTEX(gHostMsgSeqNumMutex);
/* Controls Send, only a single message may transmitted at a time. */
static struct SiiOsSemaphore *gHostMsgSendAccessSem;
/* On Send, sender does timed wait for ACK/NAK. */
static struct SiiOsSemaphore *gHostMsgSendAckNakSem;

/* Message receive event handler
 * Scheduled by HostMsgRxPacket() when it completes reception of
 * a message (1-N packets).
 */
static void HostMsgRxMessage(struct work_struct *work);
static struct workqueue_struct *RxMessageHandler_wq;
static struct work_struct *gRxMessageHandler;

struct HostMsgRxMessageInfo {
	struct HostMsgBufHdl *pRxMsg;
};

#define HOST_MSG_RX_MSG_QUEUE_DEPTH	32

static struct SiiOsQueue *gHostMsgRxMessageQueue;

#if 0
#ifndef HM_DO_LOOPBACK
/* enable to turn on loopback testing */
#define HM_DO_LOOPBACK
#endif
#else
#ifdef HM_DO_LOOPBACK
#undef HM_DO_LOOPBACK
#endif
#endif

/* Enable/Disable HostMsg mode */
static bool gIsHostMsgActive;

/* Message sequence counter, initialized to 0 */
static uint8_t gHostMsgSeqNum;

/* Packet memory buffer */
static struct HostMsgBufHdl *gMsgBeingRxd;

/* If either is set, ACK/NAK has been received. */
static bool gHostMsgGotAck; /* ACK/NAK ISR handler sets this true for ACK */
static bool gHostMsgGotNak; /* ACK/NAK ISR handler sets this true for NAK */

struct HostMsgCmdRspPendingInfo {
	struct timeval startTime;
	uint32_t maxWaitMs;
	uint16_t msgType;
	uint8_t msgTag;
	bool isSync;
};

struct HostMsgCmdRspInfo {
	uint16_t rspLen;
	void *pRspData;
};

#define HOST_MSG_CMD_QUEUE_DEPTH	16

static struct SiiOsQueue *gHostMsgPendingCmdQueue;
static struct SiiOsQueue *gHostMsgCmdRspQueue;

/* Wait and timeout values in mSecs */
#define HM_SEND_ACCESS_WAIT_MS		(2000)

/* was 500 mS - increased to compensate for slow serial interrupt handling */
#ifdef SII6400_HAL_SERIAL
#define HM_SEND_ACK_NAK_WAIT_MS		(10000)
#else
#define HM_SEND_ACK_NAK_WAIT_MS		(2000)
#endif

/*
 * The message Type is implicit in the design (will be registered for the
 * correct message type) so we don't need to pass it explicitly.
 */
struct HostMsgHandlerTableEntry {
	uint16_t msgCode;
	void (*msgHandler)(uint16_t msgCode, uint16_t msgLength,
				void *pMsgData, uint8_t msgTag);
};

static void HostMsgUnknownTypeHandler(uint16_t msgCode, uint16_t msgLength,
					void *pMsgData, uint8_t msgTag);
static bool HostMsgCheckTimeout(struct HostMsgCmdRspPendingInfo *pPending);
static enum sii_os_status HostMsgIsPending(uint16_t msgCode, uint8_t msgTag,
			struct HostMsgCmdRspPendingInfo *pReturnPending);


/******************************************************************************
 * HostMsg Message Handler Functions and the message handler function table
 * (at top of file to group all handlers in an easily-located place).
 ******************************************************************************/

/* Print out error message for unknown messages. */
static void HostMsgUnknownTypeHandler(uint16_t msgCode, uint16_t msgLength,
					void *pMsgData, uint8_t msgTag)
{
	dbg("Unknown Msg Code - msgCode(%04x) msgLength(%u)\n",
	    msgCode, msgLength);
	dbg("                   pMsgData(0x%p), msgTag(%02x)\n",
	    pMsgData, msgTag);
}

/* AP->Bow commands (command response handlers) */

/* Check macro for message codes.
 * If wrong code, prints error message and exits. */
#define MSG_CODE_CHECK(codeToCheck) \
	do { \
		if ((codeToCheck) != msgCode) { \
			err("wrong msgCode - got 0x%x, sb 0x%x\n", \
				msgCode, (codeToCheck)); \
			return; \
		} \
	} while (0)

/* Unlike the embedded code, where command responses are tightly coupled with
 * the functional code, the AP side command responses only perform success/error
 * parsing and message length checks before either returning the command
 * response payload to the caller (synchronous commands) or passing the command
 * response payload directly to the AP state machine (asyncronous commands).
 * In both cases, this handler is responsible for verifying that the received
 * command response matches a currently pending command.
 * It is the responsibility of the caller or the AP state machine to free the
 * data allocated to return the payload.
 *
 * Returns "true" if valid, "false" if timed out or passed NULL pointer.
 */
static bool HostMsgCheckTimeout(struct HostMsgCmdRspPendingInfo *pPending)
{
	struct timeval now;
	long elapsed;

	if (NULL == pPending)
		return false;

	/* Get the current time. */
	do_gettimeofday(&now);

	/* Calculate elapsed time in uSecs. */
	elapsed = (USEC_PER_SEC * (now.tv_sec - pPending->startTime.tv_sec)) +
			(now.tv_usec - pPending->startTime.tv_usec);

	if (elapsed > (1000 * (long)pPending->maxWaitMs))
		return false;
	else
		return true;
}

/* Search for matching command on pending command queue.
 * Also, age out pending but timed-out commands.
 * If return value is SII_OS_STATUS_SUCCESS, *pReturnPending will be updated to
 * hold the pending command handling structure. */
static enum sii_os_status HostMsgIsPending(uint16_t msgCode, uint8_t msgTag,
			struct HostMsgCmdRspPendingInfo *pReturnPending)
{
	uint32_t retSize = 0;
	uint16_t firstMsgCode = 0xffff;
	uint8_t firstMsgTag = 0;
	bool firstRead = true;
	enum sii_os_status retStatus = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == pReturnPending) {
		err("pReturnPending is NULL!\n");
		retStatus = SII_OS_STATUS_ERR_INVALID_PARAM;
		return retStatus;
	}

	/* Persist until have found the matching pending command in the
	 * queue or have exhausted all currently pending commands. */
	do {
		if (NULL == gHostMsgPendingCmdQueue) {
			err("gHostMsgPendingCmdQueue is NULL\n");
			retStatus = SII_OS_STATUS_ERR_NOT_AVAIL;
		} else {
			retSize = sizeof(*pReturnPending);
			retStatus = SiiOsQueueReceive(gHostMsgPendingCmdQueue,
				pReturnPending, SII_OS_NO_WAIT, &retSize);
		}

		/* Successfully read a pending data structure from the queue? */
		if ((SII_OS_STATUS_SUCCESS == retStatus) &&
		    (retSize == sizeof(struct HostMsgCmdRspPendingInfo))) {
			/* Check match before pending status in order to
			 * explicitly handle the matching but timed
			 * out condition. */
			if ((msgCode == pReturnPending->msgType) &&
			    (msgTag == pReturnPending->msgTag)) {
				if (HostMsgCheckTimeout(pReturnPending))
					break; /* Success!! */

				/* Timeout will also break out of the
				 * do-while loop. */
				dbg("msgCode 0x%x msgTag 0x%x timed out!\n",
				    msgCode, msgTag);
				retStatus = SII_OS_STATUS_TIMEOUT;
				break;
			}

			if (HostMsgCheckTimeout(pReturnPending)) {
				/* Not the matching code, return to queue. */
				retStatus = SiiOsQueueSend(
						gHostMsgPendingCmdQueue,
						pReturnPending,
						sizeof(*pReturnPending));
				if (SII_OS_STATUS_SUCCESS != retStatus)
					dbg("Can't return cmd to pending q!\n");

				if (firstRead) {
					firstMsgCode = pReturnPending->msgType;
					firstMsgTag = pReturnPending->msgTag;
					firstRead = false;
					continue;
				}

				/* If at least one other pending code
				 * was read and restored to the queue,
				 * should check to see if it matches the
				 * currently restored pending command.
				 * If so, the queue is exhausted, so
				 * just exit successfully. */

				if ((firstMsgCode == pReturnPending->msgType) &&
				    (firstMsgTag == pReturnPending->msgTag)) {
					/* this will break us out of the
					 * do-while loop */
					retStatus = SII_OS_STATUS_ERR_NOT_AVAIL;
				}
			}
		} else if (SII_OS_STATUS_QUEUE_EMPTY != retStatus) {
			err("SiiOsQueueReceive() failed! stat: %d size: %d\n",
			    retStatus, retSize);
		}
	} while (SII_OS_STATUS_SUCCESS == retStatus);

	return retStatus;
}

static void HostMsgStdCmdRspHandler(uint16_t msgCode, uint16_t msgLength,
						void *pMsgData, uint8_t msgTag)
{
	struct HostMsgCmdRspPendingInfo retPending;
	struct HostMsgCmdRspInfo outMsg;
	enum sii_os_status retStatus = SII_OS_STATUS_SUCCESS;
	enum sm_request smRequest = SII_SM_REQ_NOP;

	dbg("");

	/* Print error message and return if not a command response
	 * message type. */
	if (0 == IS_HOST_MSG_CMD_RESP_MSG(msgCode)) {
		err("non Command Response msg 0x%x\n", msgCode);
		return;
	}

	/* Initialize return-data structure. */
	outMsg.rspLen = 0;
	outMsg.pRspData = NULL;

	/* Check list of pending messages for a match.
	 * This routine checks to see if there was a pending and not timed-out
	 * command that this response matches.
	 * If check returns returns "false", log an error message and return. */
	retStatus = HostMsgIsPending(msgCode, msgTag, &retPending);
	if (SII_OS_STATUS_SUCCESS != retStatus) {
		dbg("HostMsgIsPending() returned %d\n", retStatus);
		return;
	}

	/* At this point, the received command response matches a pending
	 * command that has not timed out, so check for response validity. */

	/* Special case - no payload - this is an automatic error. */
	if (0 == msgLength) {
		/* Allocate 1 byte for error message and set code. */
		outMsg.pRspData = SiiOsCalloc(__func__, 1, 0);
		if (NULL == outMsg.pRspData) {
			err("SiiOsCalloc() failed\n");
			return;
		}
		*(uint8_t *)outMsg.pRspData = (uint8_t)SII_OS_STATUS_ERR_FAILED;
		outMsg.rspLen = 1;
	} else {
		uint8_t cmdRspResult = (uint8_t)field_ua_extract(pMsgData,
						HM_COMMON_CMD_RSP_RESULT_CODE);

		if ((uint8_t)CrrSuccess == cmdRspResult) {
			/* TBD - return the chip version number */
			dbg("Command Succeeded\n");
		} else {
			dbg("Command Failed\n");
		}

		switch (msgCode) {
		case HM_GET_CHIP_VERSION_CMD_RSP:
		case HM_GET_FW_VERSION_CMD_RSP:
		case HM_SET_OUTPUT_MODE_CMD_RSP:
		case HM_GET_OUTPUT_MODE_CMD_RSP:
		case HM_REMOTE_DEV_CONNECT_CMD_RSP:
		case HM_REMOTE_DEV_DISCONNECT_CMD_RSP:
		case HM_REMOTE_DEV_GET_INFO_CMD_RSP:
		case HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD_RSP:
		case HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP:
		case HM_GET_DEV_LIST_CMD_RSP:
		case HM_LOCAL_DEV_SET_MAC_ADDR_CMD_RSP:
		case HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP:
		case HM_LOCAL_DEV_SET_NAME_CMD_RSP:
		case HM_LOCAL_DEV_GET_NAME_CMD_RSP:
		case HM_LOCAL_DEV_SET_REGULATORY_ID_CMD_RSP:
		case HM_LOCAL_DEV_GET_REGULATORY_ID_CMD_RSP:
		case HM_WVAN_GET_CONNECTION_STATUS_CMD_RSP:
		case HM_WVAN_JOIN_CMD_RSP:
		case HM_WVAN_LEAVE_CMD_RSP:
		case HM_CEC_MSG_SEND_CMD_RSP:
		case HM_HDCP_SET_POLICY_CMD_RSP:
		case HM_HDCP_GET_POLICY_CMD_RSP:
		case HM_HDCP_SET_STREAM_TYPE_CMD_RSP:
		case HM_HDCP_GET_STREAM_TYPE_CMD_RSP:
		case HM_SET_VENDOR_MSG_FILTER_CMD_RSP:
		case HM_GET_VENDOR_MSG_FILTER_CMD_RSP:
		case HM_VENDOR_MSG_SEND_CMD_RSP:
		case HM_DIAG_COMMAND_CMD_RSP:
		case HM_DEBUG_OUTPUT_SET_PATH_CMD_RSP:
		case HM_MHL_GET_CONNECTION_STATE_CMD_RSP:
		case HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP:
		case HM_MHL_SET_CLOCK_SWING_CMD_RSP:
		case HM_MHL_GET_CLOCK_SWING_CMD_RSP:
		case HM_MHL_SET_CBUS_VOLTAGE_CMD_RSP:
		case HM_MHL_READ_LOCAL_SPAD_CMD_RSP:
		case HM_MHL_WRITE_REMOTE_SPAD_CMD_RSP:
		case HM_MHL_SEND_UCP_CHAR_CMD_RSP:
		case HM_MHL_SEND_UCP_ACK_CMD_RSP:
		case HM_MHL_SEND_RCP_KEY_CMD_RSP:
		case HM_MHL_SEND_RCP_ACK_CMD_RSP:
		case HM_MHL_SEND_RAP_ACTION_CMD_RSP:
		case HM_MHL_SEND_RAP_ACK_CMD_RSP:
		case HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP:
		case HM_MHL_DEVCAP_LOCAL_WRITE_CMD_RSP:
		case HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP:
		case HM_MHL_DEVCAP_REMOTE_WRITE_CMD_RSP:
			smRequest = SII_SM_REQ_NOP;
			if (!retPending.isSync) {
				warn("msgCode 0x%x should be synchronous!\n",
				     msgCode);
			}
			break;
		case HM_WVAN_SCAN_CMD_RSP:
			/* Should we add specific length checks here? */
			smRequest = SII_SM_REQ_REPORT_SCAN_RESULTS;
			if (retPending.isSync) {
				warn("msgCode 0x%x should be asynchronous!\n",
				     msgCode);
			}
			break;
		default:
			/* Unknown code, but was marked as "pending". */
			dbg("Need to add explicit handling for msgCode 0x%x\n",
			    msgCode);
			break;
		}

		/* Allocate buffer to hold message. */
		outMsg.pRspData = SiiOsCalloc(__func__, msgLength, 0);
		if (NULL == outMsg.pRspData) {
			err("SiiOsCalloc() failed\n");
			return;
		}
		outMsg.rspLen = msgLength;

		/* Copy data to new buffer. */
		if (NULL != pMsgData)
			memcpy(outMsg.pRspData, pMsgData, msgLength);
	}

	/* Write buffer to outbound queue or do async notification of
	 * the AP state machine. */
	if (retPending.isSync) {
		retStatus = SII_OS_STATUS_QUEUE_FULL;
		if (NULL != gHostMsgCmdRspQueue) {
			retStatus = SiiOsQueueSend(gHostMsgCmdRspQueue,
						   &outMsg, sizeof(outMsg));
		}
		if (SII_OS_STATUS_SUCCESS != retStatus) {
			/* Unable to write to queue. */
			SiiOsFree(outMsg.pRspData);
			outMsg.pRspData = NULL;
			err("Unable to write to gHostMsgCmdRspQueue\n");
		}
	} else {
		/* Notify async state machine here. */
		if (SII_SM_REQ_NOP == smRequest) {
			SiiOsFree(outMsg.pRspData);
			outMsg.pRspData = NULL;
			dbg("Freeing non-async data for msgCode 0x%x\n",
			    msgCode);
		} else {
			(void)sm_send_request(wihd_sm_queue, smRequest,
						outMsg.pRspData,
						(uint32_t)outMsg.rspLen,
						NULL, 0, NULL, NULL);
		}
	}
}

/* Bow->AP Commands */

static void HostMsgNVStorageWriteCmdHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	uint8_t aRespMsg[HM_NV_STORAGE_WRITE_CMD_RSP_MLEN];
	uint8_t aFirstData[4];
	uint16_t aFirstDataLen = sizeof(aFirstData);
	uint8_t mySeqNum, cmdRspResult = (uint8_t)CrrErrFail;
	uint16_t bytesToWrite = 0, bytesWritten = 0;
	uint32_t offsetAddr = 0;
	int writeStatus;
	void *pWriteData = NULL;
	int rv = 0;

	dbg("");

	/* Print error message and return if not a cmd response message type. */
	if (HM_NV_STORAGE_WRITE_CMD != msgCode) {
		err("Msg 0x%x is not non-volatile write (0x%x)\n",
		    msgCode, HM_NV_STORAGE_WRITE_CMD);
		return;
	}

	/* Initialize buffers */
	memset(aRespMsg, 0, sizeof(aRespMsg));
	memset(aFirstData, 0, sizeof(aFirstData));

	/* Check for underlength or overlength messages. */
	if ((HM_NV_STORAGE_WRITE_CMD_MLEN_MIN > msgLength) ||
	    (HM_NV_STORAGE_WRITE_CMD_MLEN_MAX < msgLength)) {
		err("illegal length NV Write Command %d\n", msgLength);
		goto nv_write_done;
	}

	/* Good message length: parse message. */
	bytesToWrite = (uint16_t)field_ua_extract(pMsgData,
				HM_NV_STORAGE_WRITE_CMD_DATA_LENGTH);
	offsetAddr = (uint32_t)field_ua_extract(pMsgData,
				HM_NV_STORAGE_WRITE_CMD_SRC_OFFSET);
	/* Check to ensure a legal number of bytes to write and that the
		 * length given agrees with the message length. */
	if ((512 < bytesToWrite) ||
	    (HM_NV_STORAGE_WRITE_CMD_MLEN_MIN != (msgLength - bytesToWrite))) {
		err("illegal NV Write Command: writeLength %d, msgLength %d\n",
		    bytesToWrite, msgLength);
		goto nv_write_done;
	}

	/* Special case: no need to attempt to write data if
	 * given a 0 length. */
	if ((0 < bytesToWrite) && (NULL != pMsgData)) {
		pWriteData = (void *)FIELD_UA_BYTE_ADDRESS(pMsgData,
				HM_NV_STORAGE_WRITE_CMD_DATA0);
		/* Also copy first few bytes of data if present
		 * for log message. */
		memcpy((void *)aFirstData, pWriteData,
			min(bytesToWrite, aFirstDataLen));

		writeStatus = (int)write_to_nv_storage(pWriteData,
						bytesToWrite,
						offsetAddr);
		if (0 <= writeStatus) {
			cmdRspResult = (uint8_t)CrrSuccess;
			bytesWritten = (uint16_t)writeStatus;
			dbg("NV Write Cmd - 0x%x/0x%x bytes at offset 0x%x\n",
			    bytesWritten, bytesToWrite, offsetAddr);
			dbg("               (0x%x 0x%x 0x%x 0x%x)\n",
			    aFirstData[0], aFirstData[1],
			    aFirstData[2], aFirstData[3]);
		}
	}

nv_write_done:
	/* Build return message: success/failure, bytes written */
	field_ua_set(cmdRspResult, aRespMsg, HM_COMMON_CMD_RSP_RESULT_CODE);
	field_ua_set(bytesWritten, aRespMsg,
			HM_NV_STORAGE_WRITE_CMD_RSP_DATA_LENGTH);

	/* Sequence number update is semaphore protected. */
	mySeqNum = HostMsgGetSeqNum();

	rv = HostMsgSendInternal(HM_NV_STORAGE_WRITE_CMD_RSP, mySeqNum,
		msgTag, HM_NV_STORAGE_WRITE_CMD_RSP_MLEN, aRespMsg);
	if (rv >= 0)
		dbg("Message Sent: seq # 0x%02x\n", (uint32_t)mySeqNum);
	else
		err("Message NOT Sent: seq # 0x%02x\n", (uint32_t)mySeqNum);
}

static void HostMsgNVStorageReadCmdHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	uint8_t aRspMsg[HM_NV_STORAGE_READ_CMD_RSP_MLEN_MIN];
	uint8_t mySeqNum, cmdRspResult = (uint8_t)CrrErrFail;
	uint16_t bytesToRead = 0, bytesRead = 0;
	uint32_t offsetAddr = 0;
	int readStatus;
	void *pRspMsg = NULL, *pRspData = NULL;
	int rv = 0;

	dbg("");

	/* Print error message and return if not a cmd response message type. */
	if (HM_NV_STORAGE_READ_CMD != msgCode) {
		err("Msg 0x%x is not non-volatile read (0x%x)\n",
		    msgCode, HM_NV_STORAGE_READ_CMD);
		return;
	}

	/* Start with default pointer for short error responses and
	 * 0-length reads. */
	pRspMsg = aRspMsg;
	memset(aRspMsg, 0, sizeof(aRspMsg));

	/* Check for underlength or overlength messages. */
	if (HM_NV_STORAGE_READ_CMD_MLEN != msgLength) {
		err("illegal length NV Read Command %d\n", msgLength);
	} else {
		/* Good message length: parse message. */
		bytesToRead = (uint16_t)field_ua_extract(pMsgData,
				HM_NV_STORAGE_READ_CMD_DATA_LENGTH);
		offsetAddr = (uint32_t)field_ua_extract(pMsgData,
				HM_NV_STORAGE_READ_CMD_SRC_OFFSET);

		/* Check to ensure a legal number of bytes to write, and that
		 * the length given agrees with the message length. */
		if (512 >= bytesToRead) {
			if (0 < bytesToRead) {
				/* Allocate buffer to hold message. */
				pRspMsg = SiiOsCalloc(__func__,
					(HM_NV_STORAGE_READ_CMD_RSP_MLEN_MIN +
						bytesToRead), 0);
				if (NULL == pRspMsg) {
					err("SiiOsCalloc() failed\n");
					pRspMsg = aRspMsg;
				}
			}

			/* Special case: no need to attempt to write data if
			 * given a 0 length. */
			if (0 < bytesToRead) {
				pRspData = (void *)FIELD_UA_BYTE_ADDRESS(
					pRspMsg,
					HM_NV_STORAGE_READ_CMD_RSP_DATA0);

				readStatus = read_from_nv_storage(pRspData,
						bytesToRead, offsetAddr);
				if (0 <= readStatus) {
					/* Read some data back successfully. */
					bytesRead = (uint16_t)readStatus;
					cmdRspResult = (uint8_t)CrrSuccess;
				}
			} else {
				/* Starting value for successfully alloted
				 * response */
				cmdRspResult = (uint8_t)CrrSuccess;
			}
			dbg("NV Read Cmd - 0x%x/0x%x bytes at offset 0x%x\n",
			    bytesRead , bytesToRead , offsetAddr);
		} else {
			err("illegal NV Read Command: writeLength %d\n",
			    bytesToRead);
		}
	}
	/* Build return message: success/failure, bytes written. */
	field_ua_set(cmdRspResult, pRspMsg, HM_COMMON_CMD_RSP_RESULT_CODE);
	field_ua_set(bytesRead, pRspMsg,
			HM_NV_STORAGE_READ_CMD_RSP_DATA_LENGTH);

	/* Sequence number update is semaphore protected. */
	mySeqNum = HostMsgGetSeqNum();

	if (pRspMsg != aRspMsg) {
		rv = HostMsgSendInternal(HM_NV_STORAGE_READ_CMD_RSP,
			mySeqNum, msgTag,
			(HM_NV_STORAGE_READ_CMD_RSP_MLEN_MIN + bytesRead),
			pRspMsg);
	} else {
		rv = HostMsgSendInternal(HM_NV_STORAGE_READ_CMD_RSP,
				mySeqNum, msgTag, sizeof(aRspMsg), aRspMsg);
	}
	if (rv >= 0)
		dbg("Message Sent: seq # 0x%02x\n", (uint32_t)mySeqNum);
	else
		err("Message NOT Sent: seq # 0x%02x\n", (uint32_t)mySeqNum);

	if ((NULL != pRspMsg) && (pRspMsg != aRspMsg)) {
		/* If allocated buffer, free it. */
		SiiOsFree(pRspMsg);
	}
}

/* Bow->AP Notifications */

static void HostMsgWvanAssociationResultNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_WVAN_ASSOCIATION_RESULT_NOTIFY);
	if ((HM_WVAN_ASSOCIATION_RESULT_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine. */
			pReqData = SiiOsCalloc("AssociatedNotify",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed. */
			(void)sm_send_request(wihd_sm_queue,
						SII_SM_REQ_REPORT_ASSOCIATED,
						NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(wihd_sm_queue,
						SII_SM_REQ_REPORT_ASSOCIATED,
						pReqData, reqDataLen,
						NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgWvanDisassociatedNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_WVAN_DISASSOCIATED_NOTIFY);
	if ((msgLength != HM_WVAN_DISASSOCIATED_NOTIFY_MLEN) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("DisassociatedNotify",
							reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(wihd_sm_queue,
						SII_SM_REQ_REPORT_DISASSOCIATED,
						NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(wihd_sm_queue,
						SII_SM_REQ_REPORT_DISASSOCIATED,
						pReqData, reqDataLen,
						NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgDevConnectionReadyNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_REMOTE_DEV_CONNECTION_READY_NOTIFY);
	if ((HM_REMOTE_DEV_CONNECTION_READY_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("ConnectedNotify",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(wihd_sm_queue,
						SII_SM_REQ_REPORT_CONNECTED,
						NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(wihd_sm_queue,
						SII_SM_REQ_REPORT_CONNECTED,
						pReqData, reqDataLen,
						NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgDevDisconnectedNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_REMOTE_DEV_DISCONNECTED_NOTIFY);
	if ((msgLength != HM_REMOTE_DEV_DISCONNECTED_NOTIFY_MLEN) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("DisconnectedNotify",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(wihd_sm_queue,
						SII_SM_REQ_REPORT_DISCONNECTED,
						NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(wihd_sm_queue,
						SII_SM_REQ_REPORT_DISCONNECTED,
						pReqData, reqDataLen,
						NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgCecUserCtrlMsgRcvdNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY);
	if ((HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("CecMsgNotify", reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(wihd_sm_queue,
					SII_SM_REQ_REPORT_CEC_MSG_RECEIVED,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(wihd_sm_queue,
					SII_SM_REQ_REPORT_CEC_MSG_RECEIVED,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgDevListChangedNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_DEV_LIST_CHANGED_NOTIFY);
	if ((HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MAX < msgLength) ||
	    (HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MIN > msgLength) ||
	    (NULL == pMsgData)) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen =  msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("WihdDevListChgMsgRcv",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(wihd_sm_queue,
					SII_SM_REQ_REPORT_DEV_LIST_CHANGED,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(wihd_sm_queue,
					SII_SM_REQ_REPORT_DEV_LIST_CHANGED,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgVendorMsgRcvdNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_VENDOR_MSG_RCVD_NOTIFY);
	if ((HM_VENDOR_MSG_RCVD_NOTIFY_MLEN_MAX < msgLength) ||
	    (HM_VENDOR_MSG_RCVD_NOTIFY_MLEN_MIN > msgLength) ||
	    (NULL == pMsgData)) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen =  msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("VendorMsgRcv",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(wihd_sm_queue,
					SII_SM_REQ_REPORT_VENDOR_MSG_RECEIVED,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(wihd_sm_queue,
					SII_SM_REQ_REPORT_VENDOR_MSG_RECEIVED,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgDiagCmdDoneNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_DIAG_CMD_DONE_NOTIFY);
	if ((HM_DIAG_CMD_DONE_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("DiagCmdDoneNotify",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(diag_sm_queue,
						SII_SM_REQ_REPORT_DIAG_CMD_DONE,
						NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(diag_sm_queue,
						SII_SM_REQ_REPORT_DIAG_CMD_DONE,
						pReqData, reqDataLen,
						NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgDiagCmdOutputNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_DIAG_CMD_OUTPUT_NOTIFY);
	if ((HM_DIAG_CMD_OUTPUT_NOTIFY_MLEN_MIN > msgLength) ||
	    (HM_DIAG_CMD_OUTPUT_NOTIFY_MLEN_MAX < msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("DiagCmdOutputNotify",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(diag_sm_queue,
					SII_SM_REQ_REPORT_DIAG_CMD_OUTPUT,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(diag_sm_queue,
					SII_SM_REQ_REPORT_DIAG_CMD_OUTPUT,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgDebugOutputNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_DEBUG_OUTPUT_NOTIFY);
	if ((HM_DEBUG_OUTPUT_NOTIFY_MLEN_MIN > msgLength) ||
	    (HM_DEBUG_OUTPUT_NOTIFY_MLEN_MAX < msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("DebugOutputNotify",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(diag_sm_queue,
						SII_SM_REQ_REPORT_DEBUG_OUTPUT,
						NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(diag_sm_queue,
						SII_SM_REQ_REPORT_DEBUG_OUTPUT,
						pReqData, reqDataLen,
						NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlSinkConnectedNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_SINK_CONNECTED_NOTIFY);
	if ((HM_MHL_SINK_CONNECTED_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("MHLConnectedNotify",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
						SII_SM_REQ_REPORT_MHL_CONNECTED,
						NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
						SII_SM_REQ_REPORT_MHL_CONNECTED,
						pReqData, reqDataLen,
						NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlSinkDisconnectedNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_SINK_DISCONNECTED_NOTIFY);
	if ((HM_MHL_SINK_DISCONNECTED_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("MHLDisconnectedNotify",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_DISCONNECTED,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_DISCONNECTED,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlUcpCharRcvNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_UCP_CHAR_RCV_NOTIFY);
	if ((HM_MHL_UCP_CHAR_RCV_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("MHLUCPKeyRcv", reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_UCP_CHAR_RCV,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_UCP_CHAR_RCV,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlUcpAckRcvNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_UCP_ACK_RCV_NOTIFY);
	if ((HM_MHL_UCP_ACK_RCV_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("MHLUCPAckRcv", reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_UCP_ACK_RCV,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_UCP_ACK_RCV,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlRcpKeyRcvNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_RCP_KEY_RCV_NOTIFY);
	if ((HM_MHL_RCP_KEY_RCV_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("MHLRCPKeyRcv", reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_RCP_KEY_RCV,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_RCP_KEY_RCV,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlDCapChgNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_DCAP_CHG_NOTIFY);
	if ((HM_MHL_DCAP_CHG_NOTIFY_MLEN != msgLength) ||
		((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
			msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("DcapChg",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
			(NULL == pMsgData) ||
			(0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
						SII_SM_REQ_REPORT_MHL_DCAP_CHG,
						NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
						SII_SM_REQ_REPORT_MHL_DCAP_CHG,
						pReqData, reqDataLen,
						NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlRcpAckRcvNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_RCP_ACK_RCV_NOTIFY);
	if ((HM_MHL_RCP_ACK_RCV_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("MHLRCPAckRcv", reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_RCP_ACK_RCV,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_RCP_ACK_RCV,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlRapActionRcvNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_RAP_ACTION_RCV_NOTIFY);
	if ((HM_MHL_RAP_ACTION_RCV_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("MHLRAPActionCodeRcv",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_RAP_ACTION_RCV,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_RAP_ACTION_RCV,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlRapAckRcvNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_RAP_ACK_RCV_NOTIFY);
	if ((HM_MHL_RAP_ACK_RCV_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("MHLRAPAckRcv", reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_RAP_ACK_RCV,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_RAP_ACK_RCV,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlSpadDataChgNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_SPAD_DATA_CHG_NOTIFY);
	if ((HM_MHL_SPAD_DATA_CHG_NOTIFY_MLEN_MIN > msgLength) ||
	    (HM_MHL_SPAD_DATA_CHG_NOTIFY_MLEN_MAX < msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("MHLSPadMsgRcv", reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_SPAD_MSG_RCV,
					NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
					SII_SM_REQ_REPORT_MHL_SPAD_MSG_RCV,
					pReqData, reqDataLen,
					NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

static void HostMsgMhlVbusPowerRequestNotifyHandler(uint16_t msgCode,
			uint16_t msgLength, void *pMsgData, uint8_t msgTag)
{
	MSG_CODE_CHECK(HM_MHL_VBUS_POWER_REQUEST_NOTIFY);
	if ((HM_MHL_VBUS_POWER_REQUEST_NOTIFY_MLEN != msgLength) ||
	    ((0 != msgLength) && (NULL == pMsgData))) {
		err("bad parameters - msgLength 0x%x pMsgData 0x%p\n",
		    msgLength, pMsgData);
	} else {
		void *pReqData = NULL;
		uint32_t reqDataLen = msgLength;

		if ((NULL != pMsgData) && (0 != msgLength)) {
			/* This data will be freed by the state machine */
			pReqData = SiiOsCalloc("MHLVbusPowerRqst",
						reqDataLen, 0);
		}
		if ((NULL == pReqData) ||
		    (NULL == pMsgData) ||
		    (0 == msgLength)) {
			/* Send the report with no data if alloc failed */
			(void)sm_send_request(mhl_sm_queue,
				SII_SM_REQ_REPORT_MHL_VBUS_POWER_REQUEST_RCV,
				NULL, 0, NULL, 0, NULL, NULL);
		} else {
			memcpy(pReqData, pMsgData, reqDataLen);
			(void)sm_send_request(mhl_sm_queue,
				SII_SM_REQ_REPORT_MHL_VBUS_POWER_REQUEST_RCV,
				pReqData, reqDataLen, NULL, 0, NULL, NULL);
		}
		dbg("Notification Received\n");
	}
}

/******************************************************************************/

/* Table of message handlers
 * By convention, the first entry (msgType 0) is the "unknown type" handler and
 * msgCode "HOST_MSG_UNKNOWN_MSG" (0xFFFF) will be reserved.
 */
static struct HostMsgHandlerTableEntry HostMsgHandlerTable[] = {
	/* Testing and unknown messages */
	{ HOST_MSG_UNKNOWN_MSG, HostMsgUnknownTypeHandler },
	/*{ HM_TEST_MSG_NOTIFY, HostMsgTestTypeHandler },*/

	/* Command responses - all now use a standard handler */
	{ HM_GET_CHIP_VERSION_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_GET_FW_VERSION_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_SET_OUTPUT_MODE_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_GET_OUTPUT_MODE_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_REMOTE_DEV_CONNECT_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_REMOTE_DEV_DISCONNECT_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_REMOTE_DEV_GET_INFO_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_GET_DEV_LIST_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_LOCAL_DEV_SET_MAC_ADDR_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_LOCAL_DEV_SET_NAME_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_LOCAL_DEV_GET_NAME_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_LOCAL_DEV_SET_REGULATORY_ID_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_LOCAL_DEV_GET_REGULATORY_ID_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_SET_VENDOR_MSG_FILTER_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_GET_VENDOR_MSG_FILTER_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_VENDOR_MSG_SEND_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_WVAN_GET_CONNECTION_STATUS_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_WVAN_JOIN_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_WVAN_LEAVE_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_WVAN_SCAN_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_CEC_MSG_SEND_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_HDCP_SET_POLICY_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_HDCP_GET_POLICY_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_HDCP_SET_STREAM_TYPE_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_HDCP_GET_STREAM_TYPE_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_DIAG_COMMAND_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_DEBUG_OUTPUT_SET_PATH_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_RMT_FW_UPDATE_START_REQ_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_RMT_FW_UPDATE_INFO_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_RMT_FW_UPDATE_DATA_SEND_START_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_RMT_FW_UPDATE_DATA_SEND_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_RMT_FW_UPDATE_DATA_SEND_END_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_RMT_FW_UPDATE_END_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_GET_CONNECTION_STATE_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_SET_CLOCK_SWING_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_GET_CLOCK_SWING_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_SET_CBUS_VOLTAGE_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_READ_LOCAL_SPAD_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_WRITE_REMOTE_SPAD_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_SEND_UCP_CHAR_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_SEND_UCP_ACK_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_SEND_RCP_KEY_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_SEND_RCP_ACK_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_SEND_RAP_ACTION_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_SEND_RAP_ACK_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_DEVCAP_LOCAL_WRITE_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP, HostMsgStdCmdRspHandler },
	{ HM_MHL_DEVCAP_REMOTE_WRITE_CMD_RSP, HostMsgStdCmdRspHandler },

	/* Mass storage commands
	 * The only commands the embedded device currently can send to the AP */
	{ HM_NV_STORAGE_WRITE_CMD, HostMsgNVStorageWriteCmdHandler },
	{ HM_NV_STORAGE_READ_CMD, HostMsgNVStorageReadCmdHandler },

	/* Event notifications from embedded device */
	{ HM_WVAN_ASSOCIATION_RESULT_NOTIFY,
			HostMsgWvanAssociationResultNotifyHandler },
	{ HM_WVAN_DISASSOCIATED_NOTIFY, HostMsgWvanDisassociatedNotifyHandler },
	{ HM_REMOTE_DEV_CONNECTION_READY_NOTIFY,
			HostMsgDevConnectionReadyNotifyHandler },
	{ HM_REMOTE_DEV_DISCONNECTED_NOTIFY,
			HostMsgDevDisconnectedNotifyHandler },
	{ HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY,
			HostMsgCecUserCtrlMsgRcvdNotifyHandler },
	{ HM_DEV_LIST_CHANGED_NOTIFY, HostMsgDevListChangedNotifyHandler },
	{ HM_VENDOR_MSG_RCVD_NOTIFY, HostMsgVendorMsgRcvdNotifyHandler },
	{ HM_DIAG_CMD_DONE_NOTIFY, HostMsgDiagCmdDoneNotifyHandler },
	{ HM_DIAG_CMD_OUTPUT_NOTIFY, HostMsgDiagCmdOutputNotifyHandler },
	{ HM_DEBUG_OUTPUT_NOTIFY, HostMsgDebugOutputNotifyHandler },
	{ HM_MHL_SINK_CONNECTED_NOTIFY, HostMsgMhlSinkConnectedNotifyHandler },
	{ HM_MHL_SINK_DISCONNECTED_NOTIFY,
			HostMsgMhlSinkDisconnectedNotifyHandler },
	{ HM_MHL_UCP_CHAR_RCV_NOTIFY, HostMsgMhlUcpCharRcvNotifyHandler },
	{ HM_MHL_UCP_ACK_RCV_NOTIFY, HostMsgMhlUcpAckRcvNotifyHandler },
	{ HM_MHL_RCP_KEY_RCV_NOTIFY, HostMsgMhlRcpKeyRcvNotifyHandler },
	{ HM_MHL_RCP_ACK_RCV_NOTIFY, HostMsgMhlRcpAckRcvNotifyHandler },
	{ HM_MHL_RAP_ACTION_RCV_NOTIFY, HostMsgMhlRapActionRcvNotifyHandler },
	{ HM_MHL_RAP_ACK_RCV_NOTIFY, HostMsgMhlRapAckRcvNotifyHandler },
	{ HM_MHL_SPAD_DATA_CHG_NOTIFY, HostMsgMhlSpadDataChgNotifyHandler },
	{ HM_MHL_VBUS_POWER_REQUEST_NOTIFY,
			HostMsgMhlVbusPowerRequestNotifyHandler },
	{ HM_MHL_DCAP_CHG_NOTIFY, HostMsgMhlDCapChgNotifyHandler },
};
#define NUM_MSG_HANDLER_ENTRIES \
	(sizeof(HostMsgHandlerTable) / sizeof(HostMsgHandlerTable[0]))

/******************************************************************************
 * Top Level HostMsg calls
 ******************************************************************************/

/* Set up for host messaging:
 *      create and allocate all resources
 *      initialize the AP<->BB SPI interrupts and mailboxes
 *      start HostMsg receive task
 */
int HostMsgInit(void)
{
	enum sii_os_status osStatus = SII_OS_STATUS_SUCCESS;

	dbg("");

	/* is HostMsg already active? */
	if (gIsHostMsgActive)
		return 0;

	/* disable SPI HW mode as much as we can (AP can override) */

	/* clear any inbound and outbound interrupts and leave
	 * disabled for now */
	HostMsgClearAndSetupInts(false);

	gHostMsgGotAck = false;     /* by default, neither ACK */
	gHostMsgGotNak = false;     /* nor NAK has been received */

	osStatus = SiiOsQueueCreate("gHostMsgRxMessageQueue",
					sizeof(struct HostMsgRxMessageInfo),
					HOST_MSG_RX_MSG_QUEUE_DEPTH,
					&gHostMsgRxMessageQueue);
	if (SII_OS_STATUS_SUCCESS != osStatus)
		return -1;

	/* create message-receive workqueue */
	RxMessageHandler_wq = alloc_workqueue("RxMessageHandler_wq",
						WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (NULL == RxMessageHandler_wq) {
		err("RxMessageHandler_wq create failed\n");
		return -1;
	}

	gRxMessageHandler = SiiOsCalloc("gRxMessageHandler",
					sizeof(struct work_struct), 0);
	if (NULL == gRxMessageHandler)
		return -1;

	INIT_WORK(gRxMessageHandler, HostMsgRxMessage);

	/* initialize Command and Command Responses queues for use by
	 * HostMsgSendCommand() and HostMsgSendCommandAsync() */
	osStatus = SiiOsQueueCreate("gHostMsgPendingCmdQueue",
					sizeof(struct HostMsgCmdRspPendingInfo),
					HOST_MSG_CMD_QUEUE_DEPTH,
					&gHostMsgPendingCmdQueue);
	if (SII_OS_STATUS_SUCCESS != osStatus)
		return -1;

	osStatus = SiiOsQueueCreate("gHostMsgCmdRspQueue",
					sizeof(struct HostMsgCmdRspInfo),
					HOST_MSG_CMD_QUEUE_DEPTH,
					&gHostMsgCmdRspQueue);
	if (SII_OS_STATUS_SUCCESS != osStatus)
		return -1;

	osStatus = SiiOsSemaphoreCreate("gHostMsgSendAccessSem", 1, 1,
					&gHostMsgSendAccessSem);
	if (SII_OS_STATUS_SUCCESS != osStatus)
		return -1;

	osStatus = SiiOsSemaphoreCreate("gHostMsgSendAckNakSem", 1, 0,
					&gHostMsgSendAckNakSem);
	if (SII_OS_STATUS_SUCCESS != osStatus)
		return -1;

	/* set HostMsgActive status */
	gIsHostMsgActive = true;

#ifndef HM_DO_LOOPBACK
	/* enable HostMsg interrupts */
	HostMsgClearAndSetupInts(true);
#endif

	return 0;
}

/* Stop host message:
 *      stop HostMsg receive task
 *      disable the AP<->BB SPI interrupts and mailboxes
 *      free all easily released allocated resources
 */
int HostMsgTerm(void)
{
	enum sii_os_status osStatus = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (gIsHostMsgActive) {
		/* set HostMsgActive status.
		 * no more outbound messages accepted. */
		gIsHostMsgActive = false;

		/* disable HostMsg interrupts */
		HostMsgClearAndSetupInts(false);

		if (NULL != gRxMessageHandler) {
			cancel_work_sync(gRxMessageHandler);
			SiiOsFree(gRxMessageHandler);
			gRxMessageHandler = NULL;
		}
		if (NULL != RxMessageHandler_wq) {
			destroy_workqueue(RxMessageHandler_wq);
			RxMessageHandler_wq = NULL;
		}

		if (NULL != gHostMsgRxMessageQueue) {
			struct HostMsgRxMessageInfo rxMsgInfo;
			uint32_t rxMsgInfoSize = 0;

			/* Clear out the Rx Message queue */
			rxMsgInfoSize = sizeof(rxMsgInfo);
			while (SII_OS_STATUS_SUCCESS ==
				SiiOsQueueReceive(gHostMsgRxMessageQueue,
						&rxMsgInfo, SII_OS_NO_WAIT,
						&rxMsgInfoSize)) {
				HostMsgFreeBuf(rxMsgInfo.pRxMsg);
				rxMsgInfo.pRxMsg = NULL;
				rxMsgInfoSize = sizeof(rxMsgInfo);
			}

			osStatus = SiiOsQueueDelete(gHostMsgRxMessageQueue);
			if (SII_OS_STATUS_SUCCESS != osStatus)
				err("unable to delete queue!\n");
			gHostMsgRxMessageQueue = NULL;
		}
		if (NULL != gMsgBeingRxd) {
			HostMsgFreeBuf(gMsgBeingRxd);
			gMsgBeingRxd = NULL;
		}

		if (NULL != gHostMsgPendingCmdQueue) {
			struct HostMsgCmdRspPendingInfo pendingInfo;
			uint32_t pendingInfoSize = 0;

			/* Clear out the Pending Cmd queue */
			pendingInfoSize = sizeof(pendingInfo);
			while (SII_OS_STATUS_SUCCESS ==
				SiiOsQueueReceive(gHostMsgPendingCmdQueue,
						&pendingInfo, SII_OS_NO_WAIT,
						&pendingInfoSize)) {
				pendingInfoSize = sizeof(pendingInfo);
			}

			osStatus = SiiOsQueueDelete(gHostMsgPendingCmdQueue);
			if (SII_OS_STATUS_SUCCESS != osStatus)
				err("unable to delete queue!\n");
			gHostMsgPendingCmdQueue = NULL;
		}

		if (NULL != gHostMsgCmdRspQueue) {
			struct HostMsgCmdRspInfo respInfo;
			uint32_t respInfoSize = 0;

			/* Clear out the Cmd Rsp queue */
			respInfoSize = sizeof(respInfo);
			while (SII_OS_STATUS_SUCCESS ==
				SiiOsQueueReceive(gHostMsgCmdRspQueue,
						&respInfo, SII_OS_NO_WAIT,
						&respInfoSize)) {
				SiiOsFree(respInfo.pRspData);
				respInfo.pRspData = NULL;
				respInfoSize = sizeof(respInfo);
			}

			osStatus = SiiOsQueueDelete(gHostMsgCmdRspQueue);
			if (SII_OS_STATUS_SUCCESS != osStatus)
				err("unable to delete queue!\n");
			gHostMsgCmdRspQueue = NULL;
		}

		if (NULL != gHostMsgSendAccessSem) {
			osStatus = SiiOsSemaphoreDelete(gHostMsgSendAccessSem);
			if (SII_OS_STATUS_SUCCESS != osStatus)
				err("unable to delete semaphre!\n");
			gHostMsgSendAccessSem = NULL;
		}

		if (NULL != gHostMsgSendAckNakSem) {
			osStatus = SiiOsSemaphoreDelete(gHostMsgSendAckNakSem);
			if (SII_OS_STATUS_SUCCESS != osStatus)
				err("unable to delete semaphre!\n");
			gHostMsgSendAckNakSem = NULL;
		}
	}
	return 0;
}

/* Call to allocate memory buffer and get handle
 * Always returns msgLength and pMsgBody set correctly (treat as constants) and
 * all other fields 0 or NULL.
 * Returns either valid buffer or NULL */
struct HostMsgBufHdl *HostMsgAllocateBuf(uint16_t expectedSize)
{
	struct HostMsgBufHdl *pRetVal = NULL;

	dbg("");

	/* Too small or too big don't need to be checked */
	if ((HOSTMSG_MSG_HDR_SIZE > expectedSize) ||
	    (HOSTMSG_MSG_MAX_MSGLEN < expectedSize)) {
		return pRetVal;
	}

	/* Allocate buffer of msgLength + msgHeader size.
	 * Address is in pMsgBuffer and zero value. */
	pRetVal = SiiOsCalloc("HostMsgRxBuf",
			(sizeof(struct HostMsgBufHdl) + expectedSize), 0);
	if (NULL != pRetVal) {
		/* initialize the handle */
		pRetVal->msgLength = expectedSize;
		pRetVal->pMsgBody = (uint8_t *)pRetVal +
						sizeof(struct HostMsgBufHdl);
	} else {
		err("failed to allocate buffer\n");
	}

	return pRetVal;
}

/* frees associated memory and returns handle to buffer pool */
void HostMsgFreeBuf(struct HostMsgBufHdl *pBufHdl)
{
	dbg("");

	if (NULL != pBufHdl)
		SiiOsFree(pBufHdl);
	else
		err("passed NULL buffer handle!\n");
}

/* Notification messages only - no response, generates new sequence number */
enum sii_os_status HostMsgSendNotification(uint16_t msgCode,
					uint16_t msgLength, void *pMsgData)
{
	enum sii_os_status retVal = SII_OS_STATUS_ERR_FAILED;
	int rv = 0;
	uint16_t outMsgCode = 0;
	uint8_t mySeqNum = 0;

	dbg("");

	/* can't send message if HostMsg system not active */
	if (!gIsHostMsgActive)
		return SII_OS_STATUS_ERR_FAILED;

	outMsgCode = HOST_MSG_NOTIFY_MSG(msgCode);

	/* sequence number update is semaphore protected */
	mySeqNum = HostMsgGetSeqNum();

	/* send notification */
	rv = HostMsgSendInternal(outMsgCode, mySeqNum, 0, msgLength, pMsgData);
	if (rv >= 0) {
		retVal = SII_OS_STATUS_SUCCESS;
		dbg("Message Sent: seq # 0x%02x\n", (uint32_t)mySeqNum);
	} else {
		retVal = SII_OS_STATUS_ERR_FAILED;
		err("Message NOT Sent: seq # 0x%02x\n", (uint32_t)mySeqNum);
	}

	return retVal;
}

/* Send command with no wait for response (expects responses to use state
 * machine queue to pass events and data to state machine)
 */
enum sii_os_status HostMsgSendCommandAsync(uint16_t msgCode,
					uint16_t msgLength, void *pMsgData)
{
	enum sii_os_status retVal = SII_OS_STATUS_ERR_FAILED;
	int rv = 0;
	struct HostMsgCmdRspPendingInfo pendingInfo, dummyInfo;
	uint16_t outMsgCode = 0;
	uint8_t mySeqNum = 0;
	/*struct HostMsgBufHdl *pBufHdl = NULL;*/

	dbg("");

	/* can't send message if HostMsg system not active */
	if (!gIsHostMsgActive)
		return SII_OS_STATUS_ERR_FAILED;

	outMsgCode = HOST_MSG_CMD_MSG(msgCode);

	/* sequence number update is semaphore protected */
	mySeqNum = HostMsgGetSeqNum();

	/* update the pending info */
	do_gettimeofday(&pendingInfo.startTime);
	pendingInfo.msgTag = mySeqNum;
	pendingInfo.msgType = HOST_MSG_CMD_RSP_MSG(msgCode);
	pendingInfo.maxWaitMs = 500 + 10000;
	pendingInfo.isSync = false;

	if (NULL != gHostMsgPendingCmdQueue) {
		/* force elimination of all timed-out messages in queue */
		(void)HostMsgIsPending(pendingInfo.msgType, pendingInfo.msgTag,
					&dummyInfo);
		retVal = SiiOsQueueSend(gHostMsgPendingCmdQueue, &pendingInfo,
					sizeof(pendingInfo));
		if (SII_OS_STATUS_SUCCESS != retVal)
			err("Could not put command in pending queue!\n");
	}

	/* send notification */
	rv = HostMsgSendInternal(outMsgCode, mySeqNum, 0, msgLength, pMsgData);
	if (rv >= 0) {
		retVal = SII_OS_STATUS_SUCCESS;
		dbg("Message Sent: seq # 0x%02x\n", (uint32_t)mySeqNum);
	} else {
		retVal = SII_OS_STATUS_ERR_FAILED;
		err("Message NOT Sent: seq # 0x%02x\n", (uint32_t)mySeqNum);
	}

	return retVal;
}

/* Send command with wait of up to maxTimeout mS for response.
 * Returns data length to *pRetLength and pointer to malloc'd data buffer to
 * *ppRetData.  Caller is responsible for freeing buffer after use
 * if *pRetLength is != 0.
 */
enum sii_os_status HostMsgSendCommand(uint16_t msgCode, uint16_t msgLength,
				void *pMsgData, uint32_t maxTimeout,
				uint16_t *pRetLength, void **ppRetData)
{
	enum sii_os_status retVal = SII_OS_STATUS_ERR_FAILED;
	int rv = 0;
	struct HostMsgCmdRspPendingInfo pendingInfo, dummyInfo;
	struct HostMsgCmdRspInfo retResp;
	uint32_t retSize = 0;
	uint16_t outMsgCode = 0;
	uint8_t mySeqNum = 0;
	/*struct HostMsgBufHdl *pBufHdl = NULL;*/

	dbg("");

	/* can't send message if HostMsg system not active */
	if (!gIsHostMsgActive)
		return SII_OS_STATUS_ERR_FAILED;

	outMsgCode = HOST_MSG_CMD_MSG(msgCode);

	/* sequence number update is semaphore protected */
	mySeqNum = HostMsgGetSeqNum();

	/* update the pending info */
	do_gettimeofday(&pendingInfo.startTime);
	pendingInfo.msgTag = mySeqNum;
	pendingInfo.msgType = HOST_MSG_CMD_RSP_MSG(msgCode);
	pendingInfo.maxWaitMs = maxTimeout + 500;
	pendingInfo.isSync = true;
	if (NULL != gHostMsgPendingCmdQueue) {
		/* force elimination of all timed-out messages in queue */
		(void)HostMsgIsPending(pendingInfo.msgType, pendingInfo.msgTag,
					&dummyInfo);
		retVal = SiiOsQueueSend(gHostMsgPendingCmdQueue, &pendingInfo,
					sizeof(pendingInfo));
		if (SII_OS_STATUS_SUCCESS != retVal)
			dbg("Could not put command in pending queue!\n");
	}

	/* send notification */
	rv = HostMsgSendInternal(outMsgCode, mySeqNum, 0, msgLength, pMsgData);
	if (rv >= 0) {
		retVal = SII_OS_STATUS_SUCCESS;
		dbg("Message Sent: seq # 0x%02x\n", (uint32_t)mySeqNum);
	} else {
		retVal = SII_OS_STATUS_ERR_FAILED;
		err("Message NOT Sent: seq # 0x%02x\n", (uint32_t)mySeqNum);
	}

	/* clean up returned data parameters */
	if (NULL != pRetLength)
		*pRetLength = 0;

	if (NULL != ppRetData)
		*ppRetData = NULL;

	if (NULL != gHostMsgCmdRspQueue) {
		retSize = sizeof(retResp);
		retVal = SiiOsQueueReceive(gHostMsgCmdRspQueue, &retResp,
						maxTimeout, &retSize);
		if (SII_OS_STATUS_SUCCESS == retVal) {
			if (sizeof(struct HostMsgCmdRspInfo) != retSize) {
				/* error case - read the wrong amount of data */
				retVal = SII_OS_STATUS_ERR_FAILED;
				err("read wrong length (%d) from CmdRspQueue\n",
				    retSize);
			} else {
				/* get the command response result code.
				 * (first byte of returned message) */
				if (NULL != retResp.pRspData) {
					retVal = (enum sii_os_status)(
						*(uint8_t *)retResp.pRspData);
				}

				if (NULL != pRetLength)
					*pRetLength = retResp.rspLen;

				if (NULL != ppRetData) {
					*ppRetData = retResp.pRspData;
				} else {
					SiiOsFree(retResp.pRspData);
					retResp.pRspData = NULL;
				}

			}
		}
	}
	return retVal;
}

/* Single call to send all message types
 *
 * Parameters:
 *    msgCode           Bitwise OR of Message Type
 *    mySeqNum          Sequence number to use
 *    msgTagIn          Ignored except for Cmd Resp
 *                        (IS_HOST_MSG_CMD_RSP_MSG(msgCode) is true)
 *    msgLength         Length of pMsgData buffer
 *    pMsgData          Message payload
 */
int HostMsgSendInternal(uint16_t msgCode, uint8_t mySeqNum, uint8_t msgTagIn,
				uint16_t msgLength, void *pMsgData)
{
	int retVal = -1;
	uint8_t genCrc = 0;
	uint8_t *pMsgBuffer = NULL;
	enum sii_os_status osStatus = SII_OS_STATUS_SUCCESS;

	dbg("");

	/* can't send message if HostMsg system not active */
	if (!gIsHostMsgActive)
		return retVal;

	if (((0 != msgLength) && (NULL == pMsgData)) ||
	    ((0 == msgLength) && (NULL != pMsgData)) ||
	    (HOSTMSG_MSG_MAX_DATALEN < msgLength) ||
	    !(IS_VALID_HOST_MSG_TYPE(msgCode))) {
		err("bad parameter\n");
		return retVal;
	}

	/* take access semaphore */
	osStatus = SiiOsSemaphoreTake(gHostMsgSendAccessSem,
					HM_SEND_ACCESS_WAIT_MS);
	if (SII_OS_STATUS_SUCCESS != osStatus) {
		warn("Could not take HostMsg Send Access semaphore: %x\n",
		     osStatus);
		warn("Another process is accessing the device\n");
		return -EBUSY;
	}

	/* allocate buffer and zero value */
	pMsgBuffer = SiiOsCalloc("HostMsgTxBuf",
					(msgLength + HOSTMSG_MSG_HDR_SIZE), 0);
	if (NULL == pMsgBuffer) {
		err("Unable to allocate buffer\n");

		/* release access semaphore */
		osStatus = SiiOsSemaphoreGive(gHostMsgSendAccessSem);
		if (SII_OS_STATUS_SUCCESS != osStatus) {
			err("Can't release HostMsg Send Access semaphore: %x\n",
			    osStatus);
		}

		return retVal;
	}

	/* set up header, generate CRC, add to header, and bump curFrag */
	field_ua_set(msgCode, pMsgBuffer, HOSTMSG_MSG_OPCODE);
	field_ua_set(msgLength, pMsgBuffer, HOSTMSG_MSG_DATALEN);

	/* command responses use supplied tag - all others
	 * use sequence number */
	if (IS_HOST_MSG_CMD_TO_AP_RESP_MSG(msgCode))
		field_ua_set(msgTagIn, pMsgBuffer, HOSTMSG_MSG_TAG);
	else
		field_ua_set(mySeqNum, pMsgBuffer, HOSTMSG_MSG_TAG);

	if ((0 != msgLength) && (NULL != pMsgData)) {
		/* copy data to outgoing message */
		memcpy((pMsgBuffer + HOSTMSG_MSG_HDR_SIZE), pMsgData,
						msgLength);
	}

	/* generate CRC of header and message payload,
	 * and save to to header CRC field. */
	genCrc = GenCrc8CCITT(pMsgBuffer, (msgLength + HOSTMSG_MSG_HDR_SIZE));
	field_ua_set(genCrc, pMsgBuffer, HOSTMSG_MSG_CRC);

	/* pass formatted message to packet level for transmission */
	retVal = HostMsgFormatAndSendPackets(mySeqNum,
				(msgLength + HOSTMSG_MSG_HDR_SIZE), pMsgBuffer);
	if (retVal < 0)
		err("HostMsgFormatAndSendPackets() failed\n");

	SiiOsFree(pMsgBuffer);

	/* release access semaphore */
	osStatus = SiiOsSemaphoreGive(gHostMsgSendAccessSem);
	if (SII_OS_STATUS_SUCCESS != osStatus) {
		err("Could not release HostMsg Send Access semaphore: %x\n",
		    osStatus);
	}

	return retVal;
}

/******************************************************************************
 * HostMsg Packet level functions
 ******************************************************************************/

/* Upper face of HostMst packet level.
 * Pass in sequence number, length of message, and pointer to formatted message.
 * Will build and transmit the packets needed to carry the message.
 * returns 0 if successfully packetizes and sends the message
 * returns -1 if message is overlength, empty, or transmission errors occur
 */
int HostMsgFormatAndSendPackets(uint8_t seqNum, uint16_t msgLen,
				uint8_t *pMsgData)
{
	uint16_t remainingData = 0;
	uint8_t curFrag = 0, curSize = 0, genCrc = 0;
	uint8_t pktBuffer[HOSTMSG_PKT_MAX_PKTLEN];
	int retVal = -1;

	dbg("");

	/* check for maximum possible size - we can't send more data
	 * than this */
	if (HOSTMSG_MSG_MAX_MSGLEN < msgLen) {
		err("Message is overlength\n");
		return retVal;
	}

	if ((0 == msgLen) || (NULL == pMsgData)) {
		err("Message is empty\n");
		return retVal;
	}

	remainingData = msgLen;
	curFrag = 0;
	while (0 < remainingData) {
		/* how many bytes can we send? */
		if (HOSTMSG_PKT_MAX_DATALEN < remainingData) {
			curSize = HOSTMSG_PKT_MAX_DATALEN;
			remainingData -= HOSTMSG_PKT_MAX_DATALEN;
		} else {
			curSize = (uint8_t)(0xff & remainingData);
			remainingData = 0;
		}

		/* clear header of message */
		/*memset(pktBuffer, 0, HOSTMSG_PKT_HDR_SIZE);*/
		memset(pktBuffer, 0, HOSTMSG_PKT_MAX_PKTLEN);
		/* copy data into payload */
		memcpy(&pktBuffer[HOSTMSG_PKT_HDR_SIZE], pMsgData, curSize);
		/* advance data pointer */
		pMsgData += curSize;

		/* set up header, generate CRC, add to header, bump curFrag */
		field_ua_set(seqNum, pktBuffer, HOSTMSG_PKT_SEQNUM);
		field_ua_set(curFrag, pktBuffer, HOSTMSG_PKT_FRAGNUM);
		field_ua_set(((0 < remainingData) ? 0 : 1), pktBuffer,
				HOSTMSG_PKT_LASTFRAG);
		field_ua_set(curSize, pktBuffer, HOSTMSG_PKT_DATALEN);
		genCrc = GenCrc8CCITT(pktBuffer,
					(HOSTMSG_PKT_HDR_SIZE + curSize));
		field_ua_set(genCrc, pktBuffer, HOSTMSG_PKT_CRC);
		curFrag++;

		/* send the packet now */
		dbg("  seqNum %x curFrag %x curSize %x genCrc %x\n",
		    seqNum, curFrag, curSize, genCrc);
		retVal = HostMsgSendPacket((HOSTMSG_PKT_HDR_SIZE + curSize),
						pktBuffer);
		if (retVal < 0)
			break;
#if 0
		/* test code
		 * Problems seen with large multi-packet messages.
		 * Can adding a delay between packets improve performance? */
		if (0 < remainingData) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout((10 * HZ)/MSEC_PER_SEC);
		}
#endif
	}

	return retVal;
}

/* Message receive event handler.
 * Scheduled by HostMsgRxPacket() when it completes reception of a
 * message (1-N packets).
 */
static void HostMsgRxMessage(struct work_struct *work)
{
	enum sii_os_status retVal = SII_OS_STATUS_SUCCESS;
	struct HostMsgRxMessageInfo rxMsgInfo;
	uint32_t retSize = 0;
	struct HostMsgBufHdl *pCurMsg = NULL;
	void *pMsgData = NULL;
	uint16_t msgCode = 0, msgPayloadLength = 0;
	uint8_t msgTag = 0, msgCrc = 0, genCrc = 0, i;
	bool msg_handled = false;
	void (*pHandler)(uint16_t msgCode, uint16_t msgLength,
				void *pMsgData, uint8_t msgTag);

	dbg("");

	/* exit handler if module not active */
	if (!gIsHostMsgActive)
		return;

	retSize = sizeof(rxMsgInfo);
	while (gIsHostMsgActive) {
		retVal = SiiOsQueueReceive(gHostMsgRxMessageQueue, &rxMsgInfo,
						SII_OS_NO_WAIT, &retSize);
		if (SII_OS_STATUS_SUCCESS != retVal) {
			if ((!msg_handled) ||
			    (SII_OS_STATUS_QUEUE_EMPTY != retVal)) {
				err("Rx Message queue error: %d\n", retVal);
			}
			return;
		}
		msg_handled = true;

		if (sizeof(struct HostMsgRxMessageInfo) != retSize) {
			/* read the wrong amount of data */
			err("read wrong length (%d) from RxMessageQueue\n",
			    retSize);
			continue;
		}

		pCurMsg = rxMsgInfo.pRxMsg;
		if (NULL == pCurMsg) {
			err("Read NULL pointer from Rx Message Queue!!!\n");
			continue;
		}

		/* valid message pointer - handle message and free buffer */
		dbg("Message pointer was 0x%p!\n", pCurMsg);

		/* extract message header data and pointer to payload */
		msgCode = (uint16_t)field_ua_extract(pCurMsg->pMsgBody,
						     HOSTMSG_MSG_OPCODE);
		msgPayloadLength = (uint16_t)field_ua_extract(pCurMsg->pMsgBody,
							HOSTMSG_MSG_DATALEN);
		msgTag = (uint8_t)field_ua_extract(pCurMsg->pMsgBody,
						   HOSTMSG_MSG_TAG);
		msgCrc = (uint8_t)field_ua_extract(pCurMsg->pMsgBody,
						   HOSTMSG_MSG_CRC);
		if (0 < msgPayloadLength)
			pMsgData = (void *)(pCurMsg->pMsgBody +
						HOSTMSG_MSG_HDR_SIZE);
		else
			pMsgData = NULL;

		/* now clear header CRC field and calculate CRC for
		 * header and body */
		field_ua_clear(pCurMsg->pMsgBody, HOSTMSG_MSG_CRC);
		genCrc = GenCrc8CCITT(pCurMsg->pMsgBody, pCurMsg->msgLength);

		/* does the generated CRC match the calculated? */
		if (genCrc != msgCrc) {
			err("Bad CRC - msgCode(%04x) msgPayloadLength(%u)\n",
			    msgCode, msgPayloadLength);
			err("          pMsgData(0x%p), msgTag(%02x)\n",
			    pMsgData, msgTag);
			goto message_done;
		}

		/* Good message
		 * Look for correct handler
		 * By convention, the first entry in the Handler table
		 * will have msgCode 0xFFFF assigned (reserved) and any
		 * messages with no assigned handler will call this
		 * unassigned message handler. */
		pHandler = HostMsgHandlerTable[0].msgHandler;
		for (i = 1; NUM_MSG_HANDLER_ENTRIES > i; i++) {
			/* If we find a valid message handler in the
			 * table with a matching msgCode, select the
			 * new handler and exit the search loop. */
			if ((msgCode == HostMsgHandlerTable[i].msgCode) &&
			    (NULL != HostMsgHandlerTable[i].msgHandler)) {
				pHandler = HostMsgHandlerTable[i].msgHandler;
				break;
			}
		}

		/* call the handler
		 * Should always have a valid handler
		 * (or the unknown message handler). */
		(*pHandler)(msgCode, msgPayloadLength, pMsgData, msgTag);

message_done:
		/* good or bad message, we should free the message here */
		HostMsgFreeBuf(pCurMsg);
	}
}

/* Packet receive event handler - scheduled on "Packet Ready" interrupt */
void HostMsgRxPacketHandler(void)
{
	struct HostMsgRxMessageInfo rxMsgInfo;
	int readStatus = 0;
	uint8_t curFrag = 0, curPktDataLen = 0, curPktSeq = 0, curMsgTag = 0;
	uint8_t pktBuffer[HOSTMSG_PKT_MAX_PKTLEN];
	uint16_t msgLength = 0;
	uint8_t isLastFrag = 0, goodPkt = 0;

	dbg("");

	/* exit handler if module not active */
	if (!gIsHostMsgActive) {
		/* reset interrupts and flag "NAK" on current packet
		 * if driver not enabled but this handler was called */
		HostMsgSetACKStatus(false);
		return;
	}

	/* if we got this far, we have an incoming packet */

	goodPkt = false;
	/* read packet and check CRC */
	readStatus = HostMsgReadMboxPacket(&pktBuffer[0]);
	if (0 != readStatus)
		goto packet_done;

	/* true so far - CRC was OK */
	goodPkt = true;
	curFrag = (uint8_t)field_ua_extract(pktBuffer, HOSTMSG_PKT_FRAGNUM);
	isLastFrag = (uint8_t)field_ua_extract(pktBuffer, HOSTMSG_PKT_LASTFRAG);
	curPktDataLen = (uint8_t)field_ua_extract(pktBuffer,
							HOSTMSG_PKT_DATALEN);
	curPktSeq = (uint8_t)field_ua_extract(pktBuffer, HOSTMSG_PKT_SEQNUM);

	/* Do we need to allocate a matching message?
	 * Only do so if this is the first fragment of the message and
	 * size is at least one byte longer than the message header. */
	if ((NULL == gMsgBeingRxd) &&
	    (0 == curFrag) &&
	    (HOSTMSG_MSG_HDR_SIZE <= curPktDataLen) &&
	    (HOSTMSG_PKT_MAX_DATALEN >= curPktDataLen)) {
		msgLength = (uint16_t)field_ua_extract(
				(pktBuffer + HOSTMSG_PKT_HDR_SIZE),
				HOSTMSG_MSG_DATALEN);
		curMsgTag = (uint8_t)field_ua_extract(
				(pktBuffer + HOSTMSG_PKT_HDR_SIZE),
				HOSTMSG_MSG_TAG);
		/* if legal length message, allocate a buffer for it */
		if (HOSTMSG_MSG_MAX_MSGLEN >=
				(msgLength + HOSTMSG_MSG_HDR_SIZE)) {
			/* gMsgBeingRxd->msgLength will be set by this
			 * call - all other fields set to 0/NULL */
			gMsgBeingRxd = HostMsgAllocateBuf(msgLength +
						HOSTMSG_MSG_HDR_SIZE);
			if (NULL != gMsgBeingRxd) {
				/* only these two fields need setting -
				 * current data length and
				 * fragment are 0 */
				gMsgBeingRxd->msgTag = curMsgTag;
				gMsgBeingRxd->curPktSeq = curPktSeq;
			}
		}
	}

	/* check for error conditions -
	 * wrong fragment, too long, no buffer, etc */
	if ((NULL == gMsgBeingRxd) ||
	    (curFrag != gMsgBeingRxd->curFrag) ||
	    ((curPktDataLen + gMsgBeingRxd->curBytes) >
				gMsgBeingRxd->msgLength) ||
	    (curPktSeq != gMsgBeingRxd->curPktSeq) ||
	    (HOSTMSG_PKT_MAX_DATALEN < curPktDataLen)) {
		goodPkt = false;
	} else if ((((curPktDataLen + gMsgBeingRxd->curBytes) >=
			gMsgBeingRxd->msgLength) && !isLastFrag) ||
		   (((curPktDataLen + gMsgBeingRxd->curBytes) !=
			gMsgBeingRxd->msgLength) && isLastFrag)) {
		goodPkt = false;
	} else {
		/* good message so far -
		 * copy packet data into message structure, adjust
		 * received data length, and increment the currentFrag
		 * field to the next expected fragment */
		memcpy((gMsgBeingRxd->pMsgBody + gMsgBeingRxd->curBytes),
			(pktBuffer + HOSTMSG_PKT_HDR_SIZE),
			curPktDataLen);
		gMsgBeingRxd->curBytes += curPktDataLen;
		gMsgBeingRxd->curFrag++;
	}

	/* goodPkt and isLastFrag together mean that we've received the
	 * message with no errors and it's ready to be passed up the
	 * queue to the message handler task */
	if (goodPkt && isLastFrag) {
		enum sii_os_status retVal = SII_OS_STATUS_SUCCESS;

		rxMsgInfo.pRxMsg = gMsgBeingRxd;
		retVal = SiiOsQueueSend(gHostMsgRxMessageQueue,
					&rxMsgInfo, sizeof(rxMsgInfo));
		if (SII_OS_STATUS_SUCCESS != retVal) {
			err("Unable to queue RX Message!\n");
			HostMsgFreeBuf(gMsgBeingRxd);
		} else {
			dbg("RX Message queued!\n");
		}

		/* set pointer to NULL, we've passed this message off */
		gMsgBeingRxd = NULL;

		/* schedule message handler for completed message
		 * this is a different work queue to allow (if
		 * necessary) greater latency without blocking reception
		 * of new messages */
		if ((NULL != RxMessageHandler_wq) &&
		    (NULL != gRxMessageHandler)) {
			queue_work(RxMessageHandler_wq, gRxMessageHandler);
		}
	} else if (!goodPkt && (NULL != gMsgBeingRxd)) {
		HostMsgFreeBuf(gMsgBeingRxd);
		gMsgBeingRxd = NULL;
	}

packet_done:
	/* send ACK or NAK */
	HostMsgSetACKStatus(goodPkt);
}

/******************************************************************************
 * CCITT CRC8 Implementation
 ******************************************************************************/
/* genCrc8CCITT()
 *     helper function for HostMsg module -
 *       generates 8 bit CCITT CRC over "length" bytes of "*data"
 *     expected usage on send:
 *         1) build complete packet or message, including header, with CRC
 *             field set to 0
 *         2) call genCrc8CCITT() for complete packet or message
 *             (header and payload)
 *         3) set CRC field to generated CRC
 *
 *     expected usage on receive:
 *         1) receive complete packet or message
 *         2) copy CRC header and set header CRC field to 0
 *         3) call genCrc8CCITT() for complete packet or message
 *             (header and payload)
 *         4) compare received CRC with generated. If generated != received,
 *             error detected
 */
uint8_t GenCrc8CCITT(uint8_t *data, int32_t length)
{
	uint8_t myCrc, bit;
	uint16_t genCrc;

	if (NULL == data) {
		err("NULL data pointer!\n");
		return 0;
	}

	myCrc = START_CCITT_CRC8;
	while (0 < length--) {
		/*DO_CCITT8_CRC(myCrc, *data);*/
		genCrc = myCrc ^ *data++;
		for (bit = 8; bit > 0; bit--) {
			genCrc <<= 1;
			if (genCrc & 0xff00)
				genCrc = 0xff & (genCrc ^ POLY_CCITT_CRC8);
		}
		myCrc = (uint8_t)genCrc;
	}
	return myCrc;
}

/******************************************************************************
 * Message Register Access Functions
 ******************************************************************************/

/* low-level routine to send a single already-formated packet.
 * returns 0 if successfully sent and ACK interrupt received
 * returns <0 if length >HOSTMSG_PKT_MAX_PKTLEN, timeout, or retries exceed
 *                 MAX_HOSTMSG_PKT_RETRIES
 */
int HostMsgSendPacket(uint8_t pktLen, uint8_t *pMsgPkt)
{
	int retVal = -1;
	uint8_t retry = 0;
	enum sii_os_status osStatus;

	dbg("");

	/* Ensure that the ACK/NAK semaphore has not already been released. */
	osStatus = SiiOsSemaphoreTake(gHostMsgSendAckNakSem,
					SII_OS_NO_WAIT);
	if (SII_OS_STATUS_SUCCESS == osStatus)
		warn("ACK/NAK semaphore reset\n");

	do {
		/* this should always be cleared by receiver, but just in case
		 * let's clear it here */
		HostMsgSetPktReady(false);
		gHostMsgGotAck = false;
		gHostMsgGotNak = false;
		(void)WriteMailBoxBytes(HOSTMSG_AP_TO_BOW_MBOX_ADDR, pktLen,
						pMsgPkt);

		/* set packet ready bit */
		HostMsgSetPktReady(true);

		/* take ACK/NAK semaphore - exit with error if unable to take */
		osStatus = SiiOsSemaphoreTake(gHostMsgSendAckNakSem,
						HM_SEND_ACK_NAK_WAIT_MS);
		if (SII_OS_STATUS_SUCCESS != osStatus) {
			warn("ACK/NAK timed out! (attempt %d)\n", retry);
			retVal = -EBUSY;
			continue;
		}

		if (gHostMsgGotAck) {
			retVal = 0;
		} else if (gHostMsgGotNak) {
			retVal = -1;
			warn("NAK status set!\n");
		} else {
			retVal = -2;
			warn("ACK/NAK status not set!\n");
		}
	} while ((0 > retVal) && (retry++ <= MAX_HOSTMSG_PKT_RETRIES));
	return retVal;
}

/******************************************************************************
 * Message Register Access Functions
 ******************************************************************************/

/* raw read of packet from mailbox registers
 * Verifies length and CRC, but leaves handling sequence number and
 * fragmentation to upper-level callers */
int HostMsgReadMboxPacket(uint8_t *pPacketBuffer)
{
	uint32_t mboxIn = 0;
	uint8_t *pCurByte = NULL, dataLength = 0, readCRC = 0, calcCRC = 0;
	uint8_t retries = 2;

	dbg("");

TESTTARGET:
	if (NULL == pPacketBuffer)
		return -1;
	pCurByte = pPacketBuffer;

#ifdef HM_DO_LOOPBACK
	/* loopback test - read from our own outgoing registers */
	mboxIn = HOSTMSG_AP_TO_BOW_MBOX_ADDR;
#else
	/* Set pMboxIn to start of Rx mailbox.
	 * Currently REG_GLUE_MBOX_00_ADDR, but use
	 * HOSTMSG_AP_TO_BOW_MBOX_ADDR in case this changes. */
	mboxIn = HOSTMSG_BOW_TO_AP_MBOX_ADDR;
#endif

	/* special case
	 * Need to read packet header (4 bytes).
	 * Since each register read gets 3 bytes, get the first 2 bytes of the
	 * payload at the same time. */
	if (ReadMailBoxBytes(mboxIn, 6, pCurByte))
		return -1;
	mboxIn += 2;
	pCurByte += 6;

	dataLength = (uint8_t)field_ua_extract(pPacketBuffer,
						HOSTMSG_PKT_DATALEN);
	readCRC = (uint8_t)field_ua_extract(pPacketBuffer, HOSTMSG_PKT_CRC);

	/* check to ensure that the datalength is a legal value */
	if (HOSTMSG_PKT_MAX_DATALEN < dataLength)
		return -1;

	/* special case
	 * Have read header and first couple of bytes of payload.
	 * Only need to read as many more registers as needed for rest
	 * of payload. */
	if (2 < dataLength) {
		if (ReadMailBoxBytes(mboxIn, (dataLength - 2), pCurByte))
			return -1;
	}

#if 0
	/* debug info - packet and message header (first 10 bytes of packet) */
	dbg("%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
		pPacketBuffer[0],
		pPacketBuffer[1],
		pPacketBuffer[2],
		pPacketBuffer[3],
		pPacketBuffer[4],
		pPacketBuffer[5],
		pPacketBuffer[6],
		pPacketBuffer[7],
		pPacketBuffer[8],
		pPacketBuffer[9]);
#endif
	/* Calculate CRC now. Start by zeroing received CRC field */
	/*field_ua_clear(pPacketBuffer, HOSTMSG_PKT_CRC);*/
	field_ua_set(0, pPacketBuffer, HOSTMSG_PKT_CRC);
	calcCRC = GenCrc8CCITT(pPacketBuffer,
				(dataLength + HOSTMSG_PKT_HDR_SIZE));

	/* If calculated CRC == read CRC, good packet. */
	if (calcCRC == readCRC) {
#if 0
		dbg("good CRC\n");
#endif
		return 0;
	}

#if 0
	err("bad CRC\n");
	err("  dataLength: %x CRC: %x calcCRC:%x\n",
		dataLength, readCRC, calcCRC);
#endif
	if (0 < retries--)
		goto TESTTARGET;
	return -1;
}

int ReadMailBoxBytes(uint32_t startRegNum, uint8_t byteCount,
			uint8_t *pReadBytes)
{
	uint32_t readData = 0;
	uint8_t numReads = 0, bytesThisRead = 0, *pReadCurByte = NULL;
	int retVal = -1;

	/*dbg("");*/

	/* Always read or write 3 bytes at a time. */
	numReads = (byteCount + 2) / 3;

	if ((startRegNum < 0xa) ||
	    ((startRegNum + (numReads - 1)) > 0x3f)) {
		err("attempted read of non mailbox SPI address!\n");
		return retVal;
	}

	if (NULL == pReadBytes) {
		err("NULL return buffer pointer\n");
		return retVal;
	}

	/* Start doing 3 byte reads. */
	for (; 0 < numReads; numReads--, startRegNum++) {
		readData = 0;
		retVal = SiiDeviceRegRead((enum sii_spi_reg_num)startRegNum,
						&readData);
		if (retVal < 0) {
			err("read error on register 0x%x\n", startRegNum);
			return retVal;
		}
		if (3 < byteCount) {
			bytesThisRead = 3;
			byteCount -= 3;
		} else {
			bytesThisRead = byteCount;
			byteCount = 0;
		}

		pReadCurByte = (uint8_t *)&readData;
		while (0 < bytesThisRead--)
			*pReadBytes++ = *pReadCurByte++;
	}
	return retVal;
}

int WriteMailBoxBytes(uint32_t startRegNum, uint8_t byteCount,
			uint8_t *pWriteBytes)
{
	uint32_t writeData = 0;
	uint8_t numWrites = 0, bytesThisWrite = 0, *pWriteCurByte = NULL;
	int retVal = -1;

	dbg("");

	/* Always read or write 3 bytes at a time. */
	numWrites = (byteCount + 2) / 3;

	if ((startRegNum < 0xa) ||
	    ((startRegNum + (numWrites - 1)) > 0x3f)) {
		err("attempted write of non mailbox SPI address!\n");
		return retVal;
	}

	if (NULL == pWriteBytes) {
		err("NULL data buffer pointer\n");
		return retVal;
	}

	/* Start doing 3 byte writes */
	for (; 0 < numWrites; numWrites--, startRegNum++) {
		writeData = 0;
		if (3 < byteCount) {
			bytesThisWrite = 3;
			byteCount -= 3;
		} else {
			bytesThisWrite = byteCount;
			byteCount = 0;
		}

		pWriteCurByte = (uint8_t *)&writeData;
		while (0 < bytesThisWrite--)
			*pWriteCurByte++ = *pWriteBytes++;

		retVal = SiiDeviceRegWrite((enum sii_spi_reg_num)startRegNum,
						writeData);
		if (retVal < 0) {
			err("write error on register 0x%x\n", startRegNum);
			return retVal;
		}
	}
	return retVal;
}

/* Clear current inbound and outbound HostMsg interrupts, and leave inbound
 * interrupts enabled/disabled per "enable" setting.
 */
void HostMsgClearAndSetupInts(bool enable)
{
	uint32_t mySWIntMask = 0, clearMask = 0, setMask = 0, tempReg = 0;
	int retStatus = 0;

	dbg("");

	/* These bits are set identically for both AP Ctrl Int and SW Int and
	 * their respective enable registers. */
	mySWIntMask = FIELD_SET(HOSTMSG_SW_INT_MASK, 0,
				REG_GLUE_AP_CTRL_INT_STATUS_ADDR_HOST_MSG);

	/* Clear the outbound host message interrupts whether enabling or
	 * disabling module. */
	clearMask = mySWIntMask;

	/* Make certain that HOST_READY and HOST_SUPPORT are both set if
	 * enabling the host protocol.
	 * Only clear HOST_READY if disabling host protocol. */
	if (enable) {
		setMask |= FIELD_SET(1, 0,
				REG_GLUE_AP_CTRL_INT_EN_ADDR_AP_HOST_READY_EN);
		setMask |= FIELD_SET(1, 0,
			REG_GLUE_AP_CTRL_INT_EN_ADDR_AP_HOST_SUPPORT_EN);
	} else {
		clearMask |= FIELD_SET(1, 0,
				REG_GLUE_AP_CTRL_INT_EN_ADDR_AP_HOST_READY_EN);
		setMask |= FIELD_SET(1, 0,
			REG_GLUE_AP_CTRL_INT_EN_ADDR_AP_HOST_SUPPORT_EN);
	}
	retStatus = SiiDeviceApIntStatusReadWrite(clearMask, setMask, &tempReg);
	if (retStatus < 0)
		err("Error modifying AP_CTRL_INT_STATUS: 0x%x\n", retStatus);

	/* Read the incoming interrupt enable field, clear the enable bits, and
	 * write back out. */
	retStatus = SiiDeviceIntEnableReadWrite(mySWIntMask, 0, &tempReg);
	if (retStatus < 0)
		err("Error modifying INT_N_ENABLE (off): 0x%x\n", retStatus);

	/* Clear the inbound interrupts.
	 * Should be no contention from ISR after this point. */
	retStatus = SiiDeviceRegWrite(SII_REG_INT_N_STATUS, mySWIntMask);
	if (retStatus < 0)
		err("Error modifying INT_N_STATUS: 0x%x\n", retStatus);

	/* If "enable" is true, enable incoming interrupts. */
	if (enable) {
		retStatus = SiiDeviceIntEnableReadWrite(0, mySWIntMask,
							&tempReg);
		if (retStatus < 0) {
			err("Error modifying INT_N_ENABLE (on): 0x%x\n",
			    retStatus);
		}
	}
}

void HostMsgAckHandler(void)
{
	enum sii_os_status osStatus = SII_OS_STATUS_SUCCESS;

	dbg("");

	/* Exit handler if module not active. */
	if (gIsHostMsgActive) {
		/* Notify send routine. */
		gHostMsgGotAck = true;
	}

	/* Release access semaphore. */
	osStatus = SiiOsSemaphoreGive(gHostMsgSendAckNakSem);
	if (SII_OS_STATUS_SUCCESS != osStatus) {
		err("Could not release HostMsg Send Ack Nak semaphore: %x\n",
		    osStatus);
	}
}

void HostMsgNakHandler(void)
{
	enum sii_os_status osStatus = SII_OS_STATUS_SUCCESS;

	dbg("");

	/* Exit handler if module not active. */
	if (gIsHostMsgActive) {
		/* Notify send routine. */
		gHostMsgGotNak = true;
	}

	/* Release access semaphore. */
	osStatus = SiiOsSemaphoreGive(gHostMsgSendAckNakSem);
	if (SII_OS_STATUS_SUCCESS != osStatus) {
		err("Could not release HostMsg Send Ack Nak semaphore: %x\n",
			osStatus);
	}
}

/* Return ACK/NAK for packets received. */
void HostMsgSetACKStatus(bool doAck)
{
	uint32_t intStatus = 0, ackBit = 0, nakBit = 0;
	int retStatus = 0;

	dbg("");

	/* Set ACK/NAK mask values for AP_CTRL_INT_STATUS
	 * (interrupts from AP->BOW). */
	ackBit = FIELD_SET(HOSTMSG_SW_INT_ACK, 0,
				REG_GLUE_AP_CTRL_INT_STATUS_ADDR_HOST_MSG);
	nakBit = FIELD_SET(HOSTMSG_SW_INT_NAK, 0,
				REG_GLUE_AP_CTRL_INT_STATUS_ADDR_HOST_MSG);
	if (doAck) {
		/* Set ACK and clear NAK. */
		retStatus = SiiDeviceApIntStatusReadWrite(nakBit, ackBit,
								&intStatus);
	} else {
		/* Set NAK and clear ACK. */
		retStatus = SiiDeviceApIntStatusReadWrite(ackBit, nakBit,
								&intStatus);
	}
	if (retStatus < 0)
		err("Error modifying AP_CTRL_INT_STATUS: 0x%x\n", retStatus);

	/* If we share an access semaphore, give it here. */
#ifdef HM_DO_LOOPBACK
	/*tx_semaphore_put(&gHostMsgGotAckSem);*/
	if (doAck)
		gHostMsgGotAck = true;
	else
		gHostMsgGotNak = true;
#endif
}

/* set or clear PktReady interrupt to AP */
void HostMsgSetPktReady(bool ready)
{
	uint32_t intStatus = 0, fieldVal = 0;
	int retStatus;

	dbg("");

	/* Set "packet ready" mask value for AP_CTRL_INT
	 * (interrupts from AP->Bow). */
	fieldVal = FIELD_SET(HOSTMSG_SW_INT_PKT_RDY, 0,
				REG_GLUE_AP_CTRL_INT_STATUS_ADDR_HOST_MSG);
	if (ready) {
		/* Set packet ready bit. */
		retStatus = SiiDeviceApIntStatusReadWrite(0, fieldVal,
								&intStatus);
	} else {
		/* Clear packet ready bit. */
		retStatus = SiiDeviceApIntStatusReadWrite(fieldVal, 0,
								&intStatus);
	}
	if (retStatus < 0)
		err("Error modifying AP_CTRL_INT_STATUS: 0x%x\n", retStatus);

	/* If we share an access semaphore, give it here. */
#ifdef HM_DO_LOOPBACK
	if (ready) {
		/* Give packet ready semaphore. */
		/*tx_semaphore_put(&gHostMsgRxPktSem);*/
	}
#endif
}

/* Update sequence number and return.
 * New value is mutex-protected.
 */
uint8_t HostMsgGetSeqNum(void)
{
	uint8_t mySeqNum = 0;

	/* Mutex protection to guarantee unique sequence numbers
	 * even if messages are being sent from multiple threads. */
	mutex_lock(&gHostMsgSeqNumMutex);
	mySeqNum = gHostMsgSeqNum++;
	mutex_unlock(&gHostMsgSeqNumMutex);

	return mySeqNum;
}

#if 1
/* Test functions */
void SendHMConnectMsg(void)
{
	enum sii_os_status retStatus = SII_OS_STATUS_SUCCESS;
	uint8_t msgDataArray[HM_REMOTE_DEV_CONNECT_CMD_MLEN];
	uint16_t retLen = 0;
	void *pRetData = NULL;

	dbg("");

	/* Set to known value for now, all 0's. */
	memset(msgDataArray, 0, HM_REMOTE_DEV_CONNECT_CMD_MLEN);

	retStatus = HostMsgSendCommand((uint16_t)HM_REMOTE_DEV_CONNECT,
					HM_REMOTE_DEV_CONNECT_CMD_MLEN,
					(void *)msgDataArray, 500,
					&retLen, &pRetData);
	if (SII_OS_STATUS_SUCCESS == retStatus) {
		dbg("Message Sent! retLen: %d pRedData: 0x%p\n",
		    retLen, pRetData);
	} else {
		dbg("Message Not Sent! retLen: %d pRedData: 0x%p\n",
		    retLen, pRetData);
	}

	SiiOsFree(pRetData);
}

void SendHMDisconnectMsg(void)
{
	enum sii_os_status retStatus = SII_OS_STATUS_SUCCESS;
	uint16_t retLen = 0;
	void *pRetData = NULL;

	dbg("");

	retStatus = HostMsgSendCommand((uint16_t)HM_REMOTE_DEV_DISCONNECT, 0,
					NULL, 500, &retLen, &pRetData);
	if (SII_OS_STATUS_SUCCESS == retStatus) {
		dbg("Message Sent! retLen: %d pRedData: 0x%p\n",
		    retLen, pRetData);
	} else {
		dbg("Message Not Sent! retLen: %d pRedData: 0x%p\n",
		    retLen, pRetData);
	}

	SiiOsFree(pRetData);
}

void SendHMTestMsg(uint16_t dataLength)
{
	uint16_t msgLength = 0;
	uint8_t *pOutputArray = NULL;
	uint8_t msgTag = 0;
	int retStatus = 0;

	dbg("");

	if (HOSTMSG_MSG_MAX_DATALEN < dataLength) {
		err("invalid Data length (must be in range 0-%u bytes)\n",
		    HOSTMSG_MSG_MAX_DATALEN);
		return;
	}
	msgLength = (uint16_t)dataLength;
	if ((0 != msgLength)) {
		/* Allocate memory for the pool. */
		pOutputArray = SiiOsCalloc("OutputArray",
						HOSTMSG_MSG_MAX_DATALEN, 0);
		if (NULL == pOutputArray) {
			err("failed to create pOutputArray\n");
			return;
		}

		/* Fill buffer with output data. */
		while (0 < dataLength) {
			dataLength--;
			pOutputArray[dataLength] = (uint8_t)(dataLength & 0xff);
		}
	} else {
		pOutputArray = NULL;
	}

	msgTag = HostMsgGetSeqNum();

	/* Single call to send all message types.
	 * NOTE: will ignore "msgTagIn" unless
	 * IS_HOST_MSG_CMD_RSP_MSG(msgCode) is true.
	 */
	retStatus = HostMsgSendInternal((uint16_t)HM_TEST_MSG_NOTIFY, msgTag, 0,
					msgLength, (void *)pOutputArray);
	if (retStatus >= 0)
		dbg("Message Sent: return tag 0x%02x\n", (uint32_t)msgTag);
	else
		err("Message NOT Sent: return tag 0x%02x\n", (uint32_t)msgTag);

	SiiOsFree(pOutputArray);
}

/* Below is implementation for mailbox read/write memory tests. */

/* CheckMboxRegs() is general purpose write-read-verify routine
 * that depends on caller to correctly set up source array. */

#define MBOX_TBUF_REGS		(HOSTMSG_MBOX_REG_COUNT * 2)
#define MBOX_TBUF_SIZE		(MBOX_TBUF_REGS * 3)

int CheckMboxRegs(uint8_t *pSource, uint8_t *pRead, uint8_t regCount)
{
	uint16_t byteIndex = 0;
	int retVal = 0;

	dbg("");

	if (MBOX_TBUF_REGS < regCount) {
		err("max number of registers is %d\n", MBOX_TBUF_REGS);
		return -1;
	}

	if ((NULL == pSource) || (NULL == pRead)) {
		err("NULL buffer pointer\n");
		return -1;
	}

	(void)WriteMailBoxBytes(0xa, (regCount * 3), pSource);
	memset(pRead, 0, (regCount * 3));
	(void)ReadMailBoxBytes(0xa, (regCount * 3), pRead);

	for (byteIndex = 0; byteIndex < (regCount * 3); byteIndex++) {
		if (pSource[byteIndex] != pRead[byteIndex]) {
			err("MBox %d Byte %d: sb %02x was %02x\n",
			    (byteIndex/3), (byteIndex % 3),
			    pSource[byteIndex], pRead[byteIndex]);

			retVal = -1;
		}
	}
	return retVal;
}

void DoMailboxTest(uint8_t data)
{
	int retVal = 0;
	uint8_t *pSourceArray = NULL;
	uint8_t *pReadArray = NULL;

	/* Allocate buffers big enough to hold all MBOX registers used by
	 * driver (3 bytes/reg, both buffers). */
	pSourceArray = SiiOsCalloc("SourceArray", MBOX_TBUF_SIZE, 0);
	if (NULL == pSourceArray)
		goto EXIT_POINT;

	pReadArray = SiiOsCalloc("ReadArray", MBOX_TBUF_SIZE, 0);
	if (NULL == pReadArray)
		goto EXIT_POINT;

	memset(pSourceArray, data, MBOX_TBUF_SIZE);
	retVal = CheckMboxRegs(pSourceArray, pReadArray, MBOX_TBUF_REGS);
	dbg("Write/Read mailbox with 0x%02x %s\n",
	    data, retVal ? "FAILED" : "PASSED");

EXIT_POINT:
	SiiOsFree(pSourceArray);
	SiiOsFree(pReadArray);
}
#endif

