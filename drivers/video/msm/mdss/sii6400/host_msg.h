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
 * @file host_msg.h
 *
 * @brief Definition
 *      This file is a common include file to be used by any component
 *      which needs to invoke the HostMsg SPI driver (AP<->BB)
 *
 *****************************************************************************/

#ifndef __HOST_MSG_H__
#define __HOST_MSG_H__

#include "struct_bitfield.h"
#include "osal.h"

#define HOST_MSG_CMD_TYPE               (0x1000)
#define HOST_MSG_CMD_RSP_TYPE           (0x2000)
#define HOST_MSG_CMD_TO_AP_TYPE         (0x3000)
#define HOST_MSG_CMD_TO_AP_RSP_TYPE     (0x4000)
#define HOST_MSG_NOTIFY_TYPE            (0x8000)

/* this message code is reserved */
#define HOST_MSG_UNKNOWN_MSG               (0xffff)
#define HOST_MSG_CMD_MSG(x)                \
		(HOST_MSG_CMD_TYPE | (UDATA(x) & 0xfff))
#define HOST_MSG_CMD_RSP_MSG(x)            \
		(HOST_MSG_CMD_RSP_TYPE | (UDATA(x) & 0xfff))
#define HOST_MSG_CMD_TO_AP_MSG(x)          \
		(HOST_MSG_CMD_TO_AP_TYPE | (UDATA(x) & 0xfff))
#define HOST_MSG_CMD_TO_AP_RSP_MSG(x)      \
		(HOST_MSG_CMD_TO_AP_RSP_TYPE | (UDATA(x) & 0xfff))
#define HOST_MSG_NOTIFY_MSG(x)             \
		(HOST_MSG_NOTIFY_TYPE | (UDATA(x) & 0xfff))
#define HOST_MSG_CMD_TO_RSP(x)             \
		(HOST_MSG_CMD_RSP_MSG(UDATA(x) & 0xfff))
#define HOST_MSG_CMD_TO_AP_TO_RSP(x)       \
		(HOST_MSG_CMD_TO_AP_RSP_MSG(UDATA(x) & 0xfff))
#define IS_HOST_MSG_CMD_MSG(x)             \
		(HOST_MSG_CMD_TYPE == (0xf000 & UDATA(x)))
#define IS_HOST_MSG_CMD_RESP_MSG(x)        \
		(HOST_MSG_CMD_RSP_TYPE == (0xf000 & UDATA(x)))
#define IS_HOST_MSG_CMD_TO_AP_MSG(x)       \
		(HOST_MSG_CMD_TO_AP_TYPE == (0xf000 & UDATA(x)))
#define IS_HOST_MSG_CMD_TO_AP_RESP_MSG(x)  \
		(HOST_MSG_CMD_TO_AP_RSP_TYPE == (0xf000 & UDATA(x)))
#define IS_HOST_MSG_NOTIFY_MSG(x)          \
		(HOST_MSG_NOTIFY_TYPE == (0xf000 & UDATA(x)))
#define IS_VALID_HOST_MSG_TYPE(x)          (0 != (0xf000 & UDATA(x)))

/* usage examples below */
#if 0

/* note that these are 3 distinct messages (0x1001, 0x2001, 0x8001) */
#define GET_VARIABLE_CMD_MSG           HOST_MSG_CMD_MSG(1)

#define GET_VARIABLE_CMD_RSP_MSG       HOST_MSG_CMD_RSP_MSG(1)
or
#define GET_VARIABLE_CMD_RSP_MSG       HOST_MSG_CMD_TO_RSP(GET_VARIABLE_CMD_MSG)

#define ITS_MY_BIRTHDAY_NOTIFY_MSG     HOST_MSG_NOTIFY_MSG(1)

....
msgCode = ITS_MY_BIRTHDAY_NOTIFY_MSG;

if (IS_HOST_MSG_CMD_MSG(msgCode))
	doThis(msgCode);
else
	doThat(msgCode); /* should always call doThat() */

#endif

/* These values are in msec */
#define HM_GET_CHIP_VERSION_RESPONSE_TIMEOUT                    10000
#define HM_GET_FW_VERSION_RESPONSE_TIMEOUT                      10000
#define HM_SET_OUTPUT_MODE_RESPONSE_TIMEOUT                     10000
#define HM_GET_OUTPUT_MODE_RESPONSE_TIMEOUT                     10000
#define HM_REMOTE_DEV_CONNECT_RESPONSE_TIMEOUT                  10000
#define HM_REMOTE_DEV_DISCONNECT_RESPONSE_TIMEOUT               10000
#define HM_REMOTE_DEV_GET_INFO_RESPONSE_TIMEOUT                 10000
#define HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_RESPONSE_TIMEOUT      10000
#define HM_REMOTE_DEV_GET_MONITOR_NAME_RESPONSE_TIMEOUT         10000
#define HM_REMOTE_CONNECTION_STATUS_RESPONSE_TIMEOUT            10000
#define HM_GET_DEV_LIST_RESPONSE_TIMEOUT                        10000
#define HM_LOCAL_DEV_GET_MAC_ADDR_RESPONSE_TIMEOUT              10000
#define HM_LOCAL_DEV_SET_NAME_RESPONSE_TIMEOUT                  20000
#define HM_LOCAL_DEV_GET_NAME_RESPONSE_TIMEOUT                  10000
#define HM_WVAN_JOIN_RESPONSE_TIMEOUT                           10000
#define HM_WVAN_LEAVE_RESPONSE_TIMEOUT                          10000
#define HM_WVAN_SCAN_RESPONSE_TIMEOUT                           10000
#define HM_CEC_MSG_SEND_RESPONSE_TIMEOUT                        10000
#define HM_HDCP_SET_POLICY_RESPONSE_TIMEOUT                     10000
#define HM_HDCP_GET_POLICY_RESPONSE_TIMEOUT                     10000
#define HM_HDCP_SET_STREAM_TYPE_RESPONSE_TIMEOUT                10000
#define HM_HDCP_GET_STREAM_TYPE_RESPONSE_TIMEOUT                10000
#define HM_DIAG_COMMAND_RESPONSE_TIMEOUT                        10000
#define HM_DEBUG_OUTPUT_SET_PATH_RESPONSE_TIMEOUT               10000
#define HM_RMT_FW_UPDATE_START_REQ_RESPONSE_TIMEOUT             10000
#define HM_RMT_FW_UPDATE_INFO_RESPONSE_TIMEOUT                  10000
#define HM_RMT_FW_UPDATE_DATA_SEND_START_RESPONSE_TIMEOUT       10000
#define HM_RMT_FW_UPDATE_DATA_SEND_RESPONSE_TIMEOUT             10000
#define HM_RMT_FW_UPDATE_DATA_SEND_END_RESPONSE_TIMEOUT         20000
#define HM_RMT_FW_UPDATE_END_RESPONSE_TIMEOUT                   10000
#define HM_SET_VENDOR_MSG_FILTER_RESPONSE_TIMEOUT               10000
#define HM_GET_VENDOR_MSG_FILTER_RESPONSE_TIMEOUT               10000
#define HM_SEND_VENDOR_MSG_RESPONSE_TIMEOUT                     10000
#define HM_MHL_GET_CONNECTION_STATE_RESPONSE_TIMEOUT            10000
#define HM_MHL_MEASURE_ID_IMPEDANCE_RESPONSE_TIMEOUT            20000
#define HM_MHL_SET_CLOCK_SWING_RESPONSE_TIMEOUT                 20000
#define HM_MHL_GET_CLOCK_SWING_RESPONSE_TIMEOUT                 20000
#define HM_MHL_SET_CBUS_VOLTAGE_RESPONSE_TIMEOUT                20000
#define HM_MHL_READ_LOCAL_SPAD_RESPONSE_TIMEOUT                 12000
#define HM_MHL_WRITE_REMOTE_SPAD_RESPONSE_TIMEOUT               12000
#define HM_MHL_SEND_UCP_CHAR_RESPONSE_TIMEOUT                   10000
#define HM_MHL_SEND_UCP_ACK_RESPONSE_TIMEOUT                    10000
#define HM_MHL_SEND_RCP_KEY_RESPONSE_TIMEOUT                    10000
#define HM_MHL_SEND_RCP_ACK_RESPONSE_TIMEOUT                    10000
#define HM_MHL_SEND_RAP_ACTION_RESPONSE_TIMEOUT                 10000
#define HM_MHL_SEND_RAP_ACK_RESPONSE_TIMEOUT                    10000
#define HM_MHL_DEVCAP_LOCAL_READ_RESPONSE_TIMEOUT               12000
#define HM_MHL_DEVCAP_LOCAL_WRITE_RESPONSE_TIMEOUT              12000
#define HM_MHL_DEVCAP_REMOTE_READ_RESPONSE_TIMEOUT              12000
#define HM_MHL_DEVCAP_REMOTE_WRITE_RESPONSE_TIMEOUT             12000

/******************************************************************************
 * Message Opcodes and bitfield definitions
 ******************************************************************************/

enum AP_Command_Idx {
	HM_GET_CHIP_VERSION = 0x00,
	HM_GET_FW_VERSION = 0x01,
	HM_SET_OUTPUT_MODE = 0x02,
	HM_GET_OUTPUT_MODE = 0x03,
	HM_REMOTE_DEV_CONNECT = 0x04,
	HM_REMOTE_DEV_DISCONNECT = 0x05,
	HM_REMOTE_DEV_GET_INFO = 0x06,
	HM_REMOTE_DEV_GET_SIGNAL_STRENGTH = 0x07,
	HM_REMOTE_DEV_GET_MONITOR_NAME = 0x08,
	HM_GET_DEV_LIST = 0x09,
	HM_LOCAL_DEV_SET_MAC_ADDR = 0x0a,
	HM_LOCAL_DEV_GET_MAC_ADDR = 0x0b,
	HM_LOCAL_DEV_SET_NAME = 0x0c,
	HM_LOCAL_DEV_GET_NAME = 0x0d,
	HM_LOCAL_DEV_SET_REGULATORY_ID = 0x0e,
	HM_LOCAL_DEV_GET_REGULATORY_ID = 0x0f,
	HM_WVAN_JOIN = 0x10,
	HM_WVAN_LEAVE = 0x11,
	HM_WVAN_SCAN = 0x12,
	HM_CEC_MSG_SEND = 0x13,
	HM_HDCP_SET_POLICY = 0x14,
	HM_HDCP_GET_POLICY = 0x15,
	HM_HDCP_SET_STREAM_TYPE = 0x16,
	HM_HDCP_GET_STREAM_TYPE = 0x17,
	HM_WVAN_GET_CONNECTION_STATUS = 0x18,
	HM_SET_VENDOR_MSG_FILTER = 0x19,
	HM_GET_VENDOR_MSG_FILTER = 0x1a,
	HM_VENDOR_MSG_SEND = 0x1b,
	HM_DIAG_COMMAND = 0x100,
	HM_DEBUG_OUTPUT_SET_PATH = 0x101,
	HM_RMT_FW_UPDATE_START_REQ = 0x200,
	HM_RMT_FW_UPDATE_INFO = 0x201,
	HM_RMT_FW_UPDATE_DATA_SEND_START = 0x202,
	HM_RMT_FW_UPDATE_DATA_SEND = 0x203,
	HM_RMT_FW_UPDATE_DATA_SEND_END = 0x204,
	HM_RMT_FW_UPDATE_END = 0x205,
	HM_MHL_GET_CONNECTION_STATE = 0x800,
	HM_MHL_MEASURE_ID_IMPEDANCE = 0x801,
	HM_MHL_SET_CLOCK_SWING = 0x80f,
	HM_MHL_GET_CLOCK_SWING = 0x80e,
	HM_MHL_READ_LOCAL_SPAD = 0x802,
	HM_MHL_WRITE_REMOTE_SPAD = 0x803,
	HM_MHL_SEND_UCP_CHAR = 0x804,
	HM_MHL_SEND_UCP_ACK = 0x805,
	HM_MHL_SEND_RCP_KEY = 0x806,
	HM_MHL_SEND_RCP_ACK = 0x807,
	HM_MHL_SEND_RAP_ACTION = 0x808,
	HM_MHL_SEND_RAP_ACK = 0x809,
	HM_MHL_DEVCAP_LOCAL_READ = 0x80a,
	HM_MHL_DEVCAP_LOCAL_WRITE = 0x80b,
	HM_MHL_DEVCAP_REMOTE_READ = 0x80c,
	HM_MHL_DEVCAP_REMOTE_WRITE = 0x80d,
	HM_MHL_SET_CBUS_VOLTAGE = 0x810,
};

enum SiI_Command_Idx {
	HM_NV_STORAGE_WRITE = 0x00,
	HM_NV_STORAGE_READ = 0x01
};

enum SiI_Notification_Idx {
	HM_WVAN_ASSOCIATION_RESULT = 0x00,
	HM_WVAN_DISASSOCIATED = 0x01,
	HM_REMOTE_DEV_CONNECTION_READY = 0x02,
	HM_REMOTE_DEV_AV_ENABLED = 0x03,
	HM_REMOTE_DEV_DISCONNECTED = 0x04,
	HM_CEC_USER_CTRL_MSG_RCVD = 0x05,
	HM_DEV_LIST_CHANGED = 0x06,
	HM_VENDOR_MSG_RCVD = 0x07,
	HM_DIAG_CMD_DONE = 0x100,
	HM_DIAG_CMD_OUTPUT = 0x101,
	HM_DEBUG_OUTPUT = 0x102,
	HM_MHL_SINK_CONNECTED = 0x800,
	HM_MHL_SINK_DISCONNECTED = 0x801,
	HM_MHL_UCP_CHAR_RCV = 0x802,
	HM_MHL_UCP_ACK_RCV = 0x803,
	HM_MHL_RCP_KEY_RCV = 0x804,
	HM_MHL_RCP_ACK_RCV = 0x805,
	HM_MHL_RAP_ACTION_RCV = 0x806,
	HM_MHL_RAP_ACK_RCV = 0x807,
	HM_MHL_SPAD_DATA_CHG = 0x808,
	HM_MHL_VBUS_POWER_REQUEST = 0x809,
	HM_MHL_DCAP_CHG = 0x80a,
	HM_TEST_MSG = 0xfff /* special code for any message type */
	/* 0x?fff is reserved for internal testing */
};

enum SiI_Notification_Reason {
	HM_NOTIFY_REASON_UNKNOWN = 0x00,
	HM_NOTIFY_REASON_NETWORK_ERROR = 0x01,
	HM_NOTIFY_REASON_REJECTED = 0x02,
	HM_NOTIFY_REASON_REQUESTED = 0x03,
	HM_NOTIFY_REASON_HDCP = 0x04,
	HM_NOTIFY_REASON_INPUT_ERROR = 0x05,
	HM_NOTIFY_REASON_TIMEOUT = 0x06
};

enum CmdRspResult {
	CrrSuccess = 0,
	CrrErrNetworkErr = 0xfc,
	CrrErrModeDisabled = 0xfd,
	CrrErrInvalidParam = 0xfe,
	CrrErrFail = 0xff
};

/* the size and offset values are common to all MHL capability access
 * commands, so are defined here */
#define HM_MHL_DEVCAP_MAX_CAPABILITY_SIZE           0x10
#define HM_MHL_DEVCAP_MAX_CAPABILITY_OFFSET         0x0F

/* these message codes are used for internal testing only */
#define HM_TEST_MSG_CMD                             \
		(HOST_MSG_CMD_MSG(HM_TEST_MSG))
#define HM_TEST_MSG_CMD_RSP                         \
		(HOST_MSG_CMD_RSP_MSG(HM_TEST_MSG))
#define HM_TEST_MSG_NOTIFY                          \
		(HOST_MSG_NOTIFY_MSG(HM_TEST_MSG))

/* implementation defined message codes */

/* AP->Bow WIHD commands */
#define HM_GET_CHIP_VERSION_CMD                     \
		(HOST_MSG_CMD_MSG(HM_GET_CHIP_VERSION))
#define HM_GET_FW_VERSION_CMD                       \
		(HOST_MSG_CMD_MSG(HM_GET_FW_VERSION))
#define HM_SET_OUTPUT_MODE_CMD                      \
		(HOST_MSG_CMD_MSG(HM_SET_OUTPUT_MODE))
#define HM_GET_OUTPUT_MODE_CMD                      \
		(HOST_MSG_CMD_MSG(HM_GET_OUTPUT_MODE))
#define HM_REMOTE_DEV_CONNECT_CMD                   \
		(HOST_MSG_CMD_MSG(HM_REMOTE_DEV_CONNECT))
#define HM_REMOTE_DEV_DISCONNECT_CMD                \
		(HOST_MSG_CMD_MSG(HM_REMOTE_DEV_DISCONNECT))
#define HM_REMOTE_DEV_GET_INFO_CMD                  \
		(HOST_MSG_CMD_MSG(HM_REMOTE_DEV_GET_INFO))
#define HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD       \
		(HOST_MSG_CMD_MSG(HM_REMOTE_DEV_GET_SIGNAL_STRENGTH))
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD          \
		(HOST_MSG_CMD_MSG(HM_REMOTE_DEV_GET_MONITOR_NAME))
#define HM_GET_DEV_LIST_CMD                         \
		(HOST_MSG_CMD_MSG(HM_GET_DEV_LIST))
#define HM_LOCAL_DEV_SET_MAC_ADDR_CMD               \
		(HOST_MSG_CMD_MSG(HM_LOCAL_DEV_SET_MAC_ADDR))
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD               \
		(HOST_MSG_CMD_MSG(HM_LOCAL_DEV_GET_MAC_ADDR))
#define HM_LOCAL_DEV_SET_NAME_CMD                   \
		(HOST_MSG_CMD_MSG(HM_LOCAL_DEV_SET_NAME))
#define HM_LOCAL_DEV_GET_NAME_CMD                   \
		(HOST_MSG_CMD_MSG(HM_LOCAL_DEV_GET_NAME))
#define HM_LOCAL_DEV_SET_REGULATORY_ID_CMD          \
		(HOST_MSG_CMD_MSG(HM_LOCAL_DEV_SET_REGULATORY_ID))
#define HM_LOCAL_DEV_GET_REGULATORY_ID_CMD          \
		(HOST_MSG_CMD_MSG(HM_LOCAL_DEV_GET_REGULATORY_ID))
#define HM_WVAN_GET_CONNECTION_STATUS_CMD           \
		(HOST_MSG_CMD_MSG(HM_WVAN_GET_CONNECTION_STATUS))
#define HM_WVAN_JOIN_CMD                            \
		(HOST_MSG_CMD_MSG(HM_WVAN_JOIN))
#define HM_WVAN_LEAVE_CMD                           \
		(HOST_MSG_CMD_MSG(HM_WVAN_LEAVE))
#define HM_WVAN_SCAN_CMD                            \
		(HOST_MSG_CMD_MSG(HM_WVAN_SCAN))
#define HM_CEC_MSG_SEND_CMD                         \
		(HOST_MSG_CMD_MSG(HM_CEC_MSG_SEND))
#define HM_HDCP_SET_POLICY_CMD                      \
		(HOST_MSG_CMD_MSG(HM_HDCP_SET_POLICY))
#define HM_HDCP_GET_POLICY_CMD                      \
		(HOST_MSG_CMD_MSG(HM_HDCP_GET_POLICY))
#define HM_HDCP_SET_STREAM_TYPE_CMD                 \
		(HOST_MSG_CMD_MSG(HM_HDCP_SET_STREAM_TYPE))
#define HM_HDCP_GET_STREAM_TYPE_CMD                 \
		(HOST_MSG_CMD_MSG(HM_HDCP_GET_STREAM_TYPE))
#define HM_SET_VENDOR_MSG_FILTER_CMD                \
		(HOST_MSG_CMD_MSG(HM_SET_VENDOR_MSG_FILTER))
#define HM_GET_VENDOR_MSG_FILTER_CMD                \
		(HOST_MSG_CMD_MSG(HM_GET_VENDOR_MSG_FILTER))
#define HM_VENDOR_MSG_SEND_CMD                      \
		(HOST_MSG_CMD_MSG(HM_VENDOR_MSG_SEND))

/* AP->Bow general diagnostic and debug commands - shared by WIHD and MHL */
#define HM_DIAG_COMMAND_CMD                         \
		(HOST_MSG_CMD_MSG(HM_DIAG_COMMAND))
#define HM_DEBUG_OUTPUT_SET_PATH_CMD                \
		(HOST_MSG_CMD_MSG(HM_DEBUG_OUTPUT_SET_PATH))

/* AP->Bow WIHD remote firmware update commands */
#define HM_RMT_FW_UPDATE_START_REQ_CMD              \
		(HOST_MSG_CMD_MSG(HM_RMT_FW_UPDATE_START_REQ))
#define HM_RMT_FW_UPDATE_INFO_CMD                   \
		(HOST_MSG_CMD_MSG(HM_RMT_FW_UPDATE_INFO))
#define HM_RMT_FW_UPDATE_DATA_SEND_START_CMD        \
		(HOST_MSG_CMD_MSG(HM_RMT_FW_UPDATE_DATA_SEND_START))
#define HM_RMT_FW_UPDATE_DATA_SEND_CMD              \
		(HOST_MSG_CMD_MSG(HM_RMT_FW_UPDATE_DATA_SEND))
#define HM_RMT_FW_UPDATE_DATA_SEND_END_CMD          \
		(HOST_MSG_CMD_MSG(HM_RMT_FW_UPDATE_DATA_SEND_END))
#define HM_RMT_FW_UPDATE_END_CMD                    \
		(HOST_MSG_CMD_MSG(HM_RMT_FW_UPDATE_END))

/* AP->Bow MHL commands */
#define HM_MHL_GET_CONNECTION_STATE_CMD             \
		(HOST_MSG_CMD_MSG(HM_MHL_GET_CONNECTION_STATE))
#define HM_MHL_MEASURE_ID_IMPEDANCE_CMD             \
		(HOST_MSG_CMD_MSG(HM_MHL_MEASURE_ID_IMPEDANCE))
#define HM_MHL_SET_CLOCK_SWING_CMD                  \
		(HOST_MSG_CMD_MSG(HM_MHL_SET_CLOCK_SWING))
#define HM_MHL_GET_CLOCK_SWING_CMD                  \
		(HOST_MSG_CMD_MSG(HM_MHL_GET_CLOCK_SWING))
#define HM_MHL_READ_LOCAL_SPAD_CMD                  \
		(HOST_MSG_CMD_MSG(HM_MHL_READ_LOCAL_SPAD))
#define HM_MHL_WRITE_REMOTE_SPAD_CMD                \
		(HOST_MSG_CMD_MSG(HM_MHL_WRITE_REMOTE_SPAD))
#define HM_MHL_SEND_UCP_CHAR_CMD                    \
		(HOST_MSG_CMD_MSG(HM_MHL_SEND_UCP_CHAR))
#define HM_MHL_SEND_UCP_ACK_CMD                     \
		(HOST_MSG_CMD_MSG(HM_MHL_SEND_UCP_ACK))
#define HM_MHL_SEND_RCP_KEY_CMD                     \
		(HOST_MSG_CMD_MSG(HM_MHL_SEND_RCP_KEY))
#define HM_MHL_SEND_RCP_ACK_CMD                     \
		(HOST_MSG_CMD_MSG(HM_MHL_SEND_RCP_ACK))
#define HM_MHL_SEND_RAP_ACTION_CMD                  \
		(HOST_MSG_CMD_MSG(HM_MHL_SEND_RAP_ACTION))
#define HM_MHL_SEND_RAP_ACK_CMD                     \
		(HOST_MSG_CMD_MSG(HM_MHL_SEND_RAP_ACK))
#define HM_MHL_DEVCAP_LOCAL_READ_CMD                \
		(HOST_MSG_CMD_MSG(HM_MHL_DEVCAP_LOCAL_READ))
#define HM_MHL_DEVCAP_LOCAL_WRITE_CMD               \
		(HOST_MSG_CMD_MSG(HM_MHL_DEVCAP_LOCAL_WRITE))
#define HM_MHL_DEVCAP_REMOTE_READ_CMD               \
		(HOST_MSG_CMD_MSG(HM_MHL_DEVCAP_REMOTE_READ))
#define HM_MHL_DEVCAP_REMOTE_WRITE_CMD              \
		(HOST_MSG_CMD_MSG(HM_MHL_DEVCAP_REMOTE_WRITE))

/* Responses to AP->Bow WIHD commands */
#define HM_GET_CHIP_VERSION_CMD_RSP                 \
		(HOST_MSG_CMD_RSP_MSG(HM_GET_CHIP_VERSION))
#define HM_GET_FW_VERSION_CMD_RSP                   \
		(HOST_MSG_CMD_RSP_MSG(HM_GET_FW_VERSION))
#define HM_SET_OUTPUT_MODE_CMD_RSP                  \
		(HOST_MSG_CMD_RSP_MSG(HM_SET_OUTPUT_MODE))
#define HM_GET_OUTPUT_MODE_CMD_RSP                  \
		(HOST_MSG_CMD_RSP_MSG(HM_GET_OUTPUT_MODE))
#define HM_REMOTE_DEV_CONNECT_CMD_RSP               \
		(HOST_MSG_CMD_RSP_MSG(HM_REMOTE_DEV_CONNECT))
#define HM_REMOTE_DEV_DISCONNECT_CMD_RSP            \
		(HOST_MSG_CMD_RSP_MSG(HM_REMOTE_DEV_DISCONNECT))
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP              \
		(HOST_MSG_CMD_RSP_MSG(HM_REMOTE_DEV_GET_INFO))
#define HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD_RSP   \
		(HOST_MSG_CMD_RSP_MSG(HM_REMOTE_DEV_GET_SIGNAL_STRENGTH))
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP      \
		(HOST_MSG_CMD_RSP_MSG(HM_REMOTE_DEV_GET_MONITOR_NAME))
#define HM_GET_DEV_LIST_CMD_RSP                     \
		(HOST_MSG_CMD_RSP_MSG(HM_GET_DEV_LIST))
#define HM_LOCAL_DEV_SET_MAC_ADDR_CMD_RSP           \
		(HOST_MSG_CMD_RSP_MSG(HM_LOCAL_DEV_SET_MAC_ADDR))
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP           \
		(HOST_MSG_CMD_RSP_MSG(HM_LOCAL_DEV_GET_MAC_ADDR))
#define HM_LOCAL_DEV_SET_NAME_CMD_RSP               \
		(HOST_MSG_CMD_RSP_MSG(HM_LOCAL_DEV_SET_NAME))
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP               \
		(HOST_MSG_CMD_RSP_MSG(HM_LOCAL_DEV_GET_NAME))
#define HM_LOCAL_DEV_SET_REGULATORY_ID_CMD_RSP      \
		(HOST_MSG_CMD_RSP_MSG(HM_LOCAL_DEV_SET_REGULATORY_ID))
#define HM_LOCAL_DEV_GET_REGULATORY_ID_CMD_RSP      \
		(HOST_MSG_CMD_RSP_MSG(HM_LOCAL_DEV_GET_REGULATORY_ID))
#define HM_WVAN_JOIN_CMD_RSP                        \
		(HOST_MSG_CMD_RSP_MSG(HM_WVAN_JOIN))
#define HM_WVAN_LEAVE_CMD_RSP                       \
		(HOST_MSG_CMD_RSP_MSG(HM_WVAN_LEAVE))
#define HM_WVAN_SCAN_CMD_RSP                        \
		(HOST_MSG_CMD_RSP_MSG(HM_WVAN_SCAN))
#define HM_CEC_MSG_SEND_CMD_RSP                     \
		(HOST_MSG_CMD_RSP_MSG(HM_CEC_MSG_SEND))
#define HM_HDCP_SET_POLICY_CMD_RSP                  \
		(HOST_MSG_CMD_RSP_MSG(HM_HDCP_SET_POLICY))
#define HM_HDCP_GET_POLICY_CMD_RSP                  \
		(HOST_MSG_CMD_RSP_MSG(HM_HDCP_GET_POLICY))
#define HM_HDCP_SET_STREAM_TYPE_CMD_RSP             \
		(HOST_MSG_CMD_RSP_MSG(HM_HDCP_SET_STREAM_TYPE))
#define HM_HDCP_GET_STREAM_TYPE_CMD_RSP             \
		(HOST_MSG_CMD_RSP_MSG(HM_HDCP_GET_STREAM_TYPE))
#define HM_WVAN_GET_CONNECTION_STATUS_CMD_RSP       \
		(HOST_MSG_CMD_RSP_MSG(HM_WVAN_GET_CONNECTION_STATUS))
#define HM_SET_VENDOR_MSG_FILTER_CMD_RSP            \
		(HOST_MSG_CMD_RSP_MSG(HM_SET_VENDOR_MSG_FILTER))
#define HM_GET_VENDOR_MSG_FILTER_CMD_RSP            \
		(HOST_MSG_CMD_RSP_MSG(HM_GET_VENDOR_MSG_FILTER))
#define HM_VENDOR_MSG_SEND_CMD_RSP                  \
		(HOST_MSG_CMD_RSP_MSG(HM_VENDOR_MSG_SEND))

/* Responses to AP->Bow general diagnostic and debug commands
 * shared by WIHD and MHL */
#define HM_DIAG_COMMAND_CMD_RSP                     \
		(HOST_MSG_CMD_RSP_MSG(HM_DIAG_COMMAND))
#define HM_DEBUG_OUTPUT_SET_PATH_CMD_RSP            \
		(HOST_MSG_CMD_RSP_MSG(HM_DEBUG_OUTPUT_SET_PATH))

/* Responses to AP->Bow WIHD remote firmware update commands */
#define HM_RMT_FW_UPDATE_START_REQ_CMD_RSP          \
		(HOST_MSG_CMD_RSP_MSG(HM_RMT_FW_UPDATE_START_REQ))
#define HM_RMT_FW_UPDATE_INFO_CMD_RSP               \
		(HOST_MSG_CMD_RSP_MSG(HM_RMT_FW_UPDATE_INFO))
#define HM_RMT_FW_UPDATE_DATA_SEND_START_CMD_RSP    \
		(HOST_MSG_CMD_RSP_MSG(HM_RMT_FW_UPDATE_DATA_SEND_START))
#define HM_RMT_FW_UPDATE_DATA_SEND_CMD_RSP          \
		(HOST_MSG_CMD_RSP_MSG(HM_RMT_FW_UPDATE_DATA_SEND))
#define HM_RMT_FW_UPDATE_DATA_SEND_END_CMD_RSP      \
		(HOST_MSG_CMD_RSP_MSG(HM_RMT_FW_UPDATE_DATA_SEND_END))
#define HM_RMT_FW_UPDATE_END_CMD_RSP                \
		(HOST_MSG_CMD_RSP_MSG(HM_RMT_FW_UPDATE_END))

/* Responses to AP->Bow MHL commands */
#define HM_MHL_GET_CONNECTION_STATE_CMD_RSP         \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_GET_CONNECTION_STATE))
#define HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP         \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_MEASURE_ID_IMPEDANCE))
#define HM_MHL_SET_CLOCK_SWING_CMD_RSP              \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_SET_CLOCK_SWING))
#define HM_MHL_GET_CLOCK_SWING_CMD_RSP              \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_GET_CLOCK_SWING))
#define HM_MHL_SET_CBUS_VOLTAGE_CMD_RSP          \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_SET_CBUS_VOLTAGE))

#define HM_MHL_READ_LOCAL_SPAD_CMD_RSP              \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_READ_LOCAL_SPAD))
#define HM_MHL_WRITE_REMOTE_SPAD_CMD_RSP            \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_WRITE_REMOTE_SPAD))
#define HM_MHL_SEND_UCP_CHAR_CMD_RSP                \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_SEND_UCP_CHAR))
#define HM_MHL_SEND_UCP_ACK_CMD_RSP                 \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_SEND_UCP_ACK))
#define HM_MHL_SEND_RCP_KEY_CMD_RSP                 \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_SEND_RCP_KEY))
#define HM_MHL_SEND_RCP_ACK_CMD_RSP                 \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_SEND_RCP_ACK))
#define HM_MHL_SEND_RAP_ACTION_CMD_RSP              \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_SEND_RAP_ACTION))
#define HM_MHL_SEND_RAP_ACK_CMD_RSP                 \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_SEND_RAP_ACK))
#define HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP            \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_DEVCAP_LOCAL_READ))
#define HM_MHL_DEVCAP_LOCAL_WRITE_CMD_RSP           \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_DEVCAP_LOCAL_WRITE))
#define HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP           \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_DEVCAP_REMOTE_READ))
#define HM_MHL_DEVCAP_REMOTE_WRITE_CMD_RSP          \
		(HOST_MSG_CMD_RSP_MSG(HM_MHL_DEVCAP_REMOTE_WRITE))

/* Bow->AP commands */
#define HM_NV_STORAGE_WRITE_CMD                     \
		(HOST_MSG_CMD_TO_AP_MSG(HM_NV_STORAGE_WRITE))
#define HM_NV_STORAGE_READ_CMD                      \
		(HOST_MSG_CMD_TO_AP_MSG(HM_NV_STORAGE_READ))

/* Responses to Bow->AP commands */
#define HM_NV_STORAGE_WRITE_CMD_RSP                 \
		(HOST_MSG_CMD_TO_AP_RSP_MSG(HM_NV_STORAGE_WRITE))
#define HM_NV_STORAGE_READ_CMD_RSP                  \
		(HOST_MSG_CMD_TO_AP_RSP_MSG(HM_NV_STORAGE_READ))

/* Bow->AP notifications */
#define HM_WVAN_ASSOCIATION_RESULT_NOTIFY           \
		(HOST_MSG_NOTIFY_MSG(HM_WVAN_ASSOCIATION_RESULT))
#define HM_WVAN_DISASSOCIATED_NOTIFY                \
		(HOST_MSG_NOTIFY_MSG(HM_WVAN_DISASSOCIATED))
#define HM_REMOTE_DEV_AV_ENABLED_NOTIFY             \
		(HOST_MSG_NOTIFY_MSG(HM_REMOTE_DEV_AV_ENABLED))
#define HM_REMOTE_DEV_CONNECTION_READY_NOTIFY       \
		(HOST_MSG_NOTIFY_MSG(HM_REMOTE_DEV_CONNECTION_READY))
#define HM_REMOTE_DEV_DISCONNECTED_NOTIFY           \
		(HOST_MSG_NOTIFY_MSG(HM_REMOTE_DEV_DISCONNECTED))
#define HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY            \
		(HOST_MSG_NOTIFY_MSG(HM_CEC_USER_CTRL_MSG_RCVD))
#define HM_DEV_LIST_CHANGED_NOTIFY                  \
		(HOST_MSG_NOTIFY_MSG(HM_DEV_LIST_CHANGED))
#define HM_VENDOR_MSG_RCVD_NOTIFY                   \
		(HOST_MSG_NOTIFY_MSG(HM_VENDOR_MSG_RCVD))

#define HM_DIAG_CMD_DONE_NOTIFY                     \
		(HOST_MSG_NOTIFY_MSG(HM_DIAG_CMD_DONE))
#define HM_DIAG_CMD_OUTPUT_NOTIFY                   \
		(HOST_MSG_NOTIFY_MSG(HM_DIAG_CMD_OUTPUT))
#define HM_DEBUG_OUTPUT_NOTIFY                      \
		(HOST_MSG_NOTIFY_MSG(HM_DEBUG_OUTPUT))

#define HM_MHL_SINK_CONNECTED_NOTIFY                \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_SINK_CONNECTED))
#define HM_MHL_SINK_DISCONNECTED_NOTIFY             \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_SINK_DISCONNECTED))
#define HM_MHL_UCP_CHAR_RCV_NOTIFY                  \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_UCP_CHAR_RCV))
#define HM_MHL_UCP_ACK_RCV_NOTIFY                   \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_UCP_ACK_RCV))
#define HM_MHL_RCP_KEY_RCV_NOTIFY                   \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_RCP_KEY_RCV))
#define HM_MHL_RCP_ACK_RCV_NOTIFY                   \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_RCP_ACK_RCV))
#define HM_MHL_RAP_ACTION_RCV_NOTIFY                \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_RAP_ACTION_RCV))
#define HM_MHL_RAP_ACK_RCV_NOTIFY                   \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_RAP_ACK_RCV))
#define HM_MHL_SPAD_DATA_CHG_NOTIFY                 \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_SPAD_DATA_CHG))
#define HM_MHL_VBUS_POWER_REQUEST_NOTIFY            \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_VBUS_POWER_REQUEST))
#define HM_MHL_DCAP_CHG_NOTIFY                      \
		(HOST_MSG_NOTIFY_MSG(HM_MHL_DCAP_CHG))

/* Message bitfield defines - note that not all messages contain payloads */

/* Commands - AP->Bow then Bow->AP */

/* no payload in HM_GET_CHIP_VERSION_CMD */
#define HM_GET_CHIP_VERSION_CMD_MLEN                0

/* no payload in HM_GET_FW_VERSION_CMD */
#define HM_GET_FW_VERSION_CMD_MLEN                  0

/* HM_SET_OUTPUT_MODE_CMD has one 8 bit field - msg length is 1 */
#define HM_SET_OUTPUT_MODE_CMD_MLEN                 1
#define HM_SET_OUTPUT_MODE_CMD_MODE                 \
		FIELD_DEFINE(8, 0, 0) /* b7:0 */

/* no payload in HM_GET_OUTPUT_MODE_CMD */
#define HM_GET_OUTPUT_MODE_CMD_MLEN                 0

/* HM_REMOTE_DEV_CONNECT_CMD has 6 bytes of MAC address.
 * Can be copied with memcpy() or accessed as bytes 0-5.  Rev A added 1 byte
 * "timeout field. */
/* use FIELD_UA_BYTE_ADDRESS to get the address of MAC_ADDR0. */
#define HM_REMOTE_DEV_CONNECT_CMD_MLEN              7
#define HM_REMOTE_DEV_CONNECT_CMD_MAC_ADDR_LEN      6
#define HM_REMOTE_DEV_CONNECT_CMD_MAC_ADDR0         \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_REMOTE_DEV_CONNECT_CMD_MAC_ADDR1         \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_REMOTE_DEV_CONNECT_CMD_MAC_ADDR2         \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */
#define HM_REMOTE_DEV_CONNECT_CMD_MAC_ADDR3         \
		FIELD_DEFINE(8, 24, 0) /* b31:24 of word 0 */
#define HM_REMOTE_DEV_CONNECT_CMD_MAC_ADDR4         \
		FIELD_DEFINE(8, 0, 1)  /* b7:0 of word 1 */
#define HM_REMOTE_DEV_CONNECT_CMD_MAC_ADDR5         \
		FIELD_DEFINE(8, 8, 1)  /* b15:8 of word 1 */
#define HM_REMOTE_DEV_CONNECT_CMD_TIMEOUT           \
		FIELD_DEFINE(8, 16, 1) /* b23:16 of word 1 */

/* no payload in HM_REMOTE_DEV_DISCONNECT_CMD */
#define HM_REMOTE_DEV_DISCONNECT_CMD_MLEN           0

/* no payload in HM_REMOTE_DEV_GET_INFO_CMD */
#define HM_REMOTE_DEV_GET_INFO_CMD_MLEN             0

/* no payload in HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD */
#define HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD_MLEN  0

/* no payload in HM_REMOTE_DEV_GET_MONITOR_NAME_CMD */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_MLEN     0

/* no payload in HM_GET_DEV_LIST_CMD */
#define HM_GET_DEV_LIST_CMD_MLEN                    0

/* no payload in HM_LOCAL_DEV_GET_MAC_ADDR_CMD */
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD_MLEN          0

/* HM_LOCAL_DEV_SET_NAME_CMD has 1-16 bytes of name data
 * Defined here for byte access but memcpy() is probably better access method */
/* use FIELD_UA_BYTE_ADDRESS to get the address of NAME0 */
#define HM_LOCAL_DEV_SET_NAME_CMD_MLEN_MIN          \
		1   /* may be 1-16 bytes extra data */
#define HM_LOCAL_DEV_SET_NAME_CMD_MLEN_MAX          \
		16  /* may be 1-16 bytes extra data */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME0             \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME1             \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME2             \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME3             \
		FIELD_DEFINE(8, 24, 0) /* b31:24 of word 0 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME4             \
		FIELD_DEFINE(8, 0, 1)  /* b7:0 of word 1 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME5             \
		FIELD_DEFINE(8, 8, 1)  /* b15:8 of word 1 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME6             \
		FIELD_DEFINE(8, 16, 1) /* b23:16 of word 1 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME7             \
		FIELD_DEFINE(8, 24, 1) /* b31:24 of word 1 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME8             \
		FIELD_DEFINE(8, 0, 2)  /* b7:0 of word 2 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME9             \
		FIELD_DEFINE(8, 8, 2)  /* b15:8 of word 2 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME10            \
		FIELD_DEFINE(8, 16, 2) /* b23:16 of word 2 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME11            \
		FIELD_DEFINE(8, 24, 2) /* b31:24 of word 2 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME12            \
		FIELD_DEFINE(8, 0, 3)  /* b7:0 of word 3 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME13            \
		FIELD_DEFINE(8, 8, 3)  /* b15:8 of word 3 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME14            \
		FIELD_DEFINE(8, 16, 3) /* b23:16 of word 3 */
#define HM_LOCAL_DEV_SET_NAME_CMD_NAME15            \
		FIELD_DEFINE(8, 24, 3) /* b31:24 of word 3 */

/* no payload in HM_LOCAL_DEV_GET_NAME_CMD */
#define HM_LOCAL_DEV_GET_NAME_CMD_MLEN              0

/* no payload in HM_WVAN_GET_CONNECTION_STATUS_CMD */
#define HM_WVAN_GET_CONNECTION_STATUS_CMD_MLEN      0

/* HM_WVAN_JOIN_CMD has 3 8-bit fields: wvan id, hr_channel, lr_channel */
#define HM_WVAN_JOIN_CMD_MLEN                       3
#define HM_WVAN_JOIN_CMD_WVAN_ID                    \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_WVAN_JOIN_CMD_HR_CHANNEL                 \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_WVAN_JOIN_CMD_LR_CHANNEL                 \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* no payload in HM_WVAN_LEAVE_CMD */
#define HM_WVAN_LEAVE_CMD_MLEN                      0

/* no payload in HM_WVAN_SCAN_CMD */
#define HM_WVAN_SCAN_CMD_MLEN                       0

/* HM_CEC_MSG_SEND_CMD is variable length with 0-16 bytes of message.
 * Need to check if 0 length messages are allowable (makes no sense). */
#define HM_CEC_MSG_SEND_CMD_CEC_HDRLEN              2
#define HM_CEC_MSG_SEND_CMD_CEC_DATALEN_MIN         2
#define HM_CEC_MSG_SEND_CMD_CEC_DATALEN_MAX         16
#define HM_CEC_MSG_SEND_CMD_MLEN_MIN                \
		(HM_CEC_MSG_SEND_CMD_CEC_HDRLEN + \
		 HM_CEC_MSG_SEND_CMD_CEC_DATALEN_MIN)
#define HM_CEC_MSG_SEND_CMD_MLEN_MAX                \
		(HM_CEC_MSG_SEND_CMD_CEC_HDRLEN + \
		 HM_CEC_MSG_SEND_CMD_CEC_DATALEN_MAX)
#define HM_CEC_MSG_SEND_CMD_CEC_DATALEN             \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_CEC_MSG_SEND_CMD_RSVD                    \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_CEC_MSG_SEND_CMD_CEC_DATA0               \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_HDCP_SET_POLICY_CMD has a 1 byte policy field */
#define HM_HDCP_SET_POLICY_CMD_MLEN                 1
#define HM_HDCP_SET_POLICY_CMD_POLICY               \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_HDCP_GET_POLICY_CMD has no payload */
#define HM_HDCP_GET_POLICY_CMD_MLEN                 0

/* HM_HDCP_SET_STREAM_TYPE_CMD has a 1 byte stream_type field */
#define HM_HDCP_SET_STREAM_TYPE_CMD_MLEN            1
#define HM_HDCP_SET_STREAM_TYPE_CMD_STREAM_TYPE     \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_HDCP_GET_STREAM_TYPE_CMD has no payload */
#define HM_HDCP_GET_STREAM_TYPE_CMD_MLEN            0

/* HM_RMT_FW_UPDATE_START_REQ_CMD has a 4 byte customerkey field */
#define HM_RMT_FW_UPDATE_START_REQ_CMD_MLEN         4
#define HM_RMT_FW_UPDATE_START_REQ_CMD_CUSTOMERKEY  \
		FIELD_DEFINE(32, 0, 0) /* b31:0 of word 0 */

/* HM_RMT_FW_UPDATE_INFO_CMD has fields for fw_size, entrypoint, mic */
#define HM_RMT_FW_UPDATE_INFO_CMD_MLEN              16
#define HM_RMT_FW_UPDATE_INFO_CMD_MIC_LEN           8
#define HM_RMT_FW_UPDATE_INFO_CMD_FW_SIZE           \
		FIELD_DEFINE(32, 0, 0) /* b31:0 of word 0 */
#define HM_RMT_FW_UPDATE_INFO_CMD_ENTRYPOINT        \
		FIELD_DEFINE(32, 0, 1) /* b31:0 of word 1 */
#define HM_RMT_FW_UPDATE_INFO_CMD_MIC0              \
		FIELD_DEFINE(8, 0, 2)  /* b7:0 of word 2 */

/* HM_RMT_FW_UPDATE_DATA_SEND_START_CMD has a 4 byte start_offset field */
#define HM_RMT_FW_UPDATE_DATA_SEND_START_CMD_MLEN         4
#define HM_RMT_FW_UPDATE_DATA_SEND_START_CMD_START_OFFSET \
		FIELD_DEFINE(32, 0, 0) /* b31:0 of word 0 */

/* HM_RMT_FW_UPDATE_DATA_SEND_CMD has 0-512 of cmd data */
#define HM_RMT_FW_UPDATE_DATA_SEND_CMD_MLEN_MIN     0
#define HM_RMT_FW_UPDATE_DATA_SEND_CMD_MLEN_MAX     512
#define HM_RMT_FW_UPDATE_DATA_SEND_CMD_DATA0        \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_RMT_FW_UPDATE_DATA_SEND_END_CMD has a 4 byte crc32 field */
#define HM_RMT_FW_UPDATE_DATA_SEND_END_MLEN         4
#define HM_RMT_FW_UPDATE_DATA_SEND_END_CRC32        \
		FIELD_DEFINE(32, 0, 0) /* b31:0 of word 0 */

/* HM_RMT_FW_UPDATE_END_CMD has no payload */
#define HM_RMT_FW_UPDATE_END_MLEN                   0

/* HM_SET_VENDOR_MSG_FILTER_CMD has a 3 byte vendor ID field */
#define HM_SET_VENDOR_MSG_FILTER_CMD_MLEN           3
#define HM_SET_VENDOR_MSG_FILTER_CMD_VID0           \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word0 */
#define HM_SET_VENDOR_MSG_FILTER_CMD_VID1           \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word0 */
#define HM_SET_VENDOR_MSG_FILTER_CMD_VID2           \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word0 */

/* HM_GET_VENDOR_MSG_FILTER has no payload */
#define HM_GET_VENDOR_MSG_FILTER_CMD_MLEN           0

/* HM_VENDOR_MSG_SEND_CMD has a 3 byte vendor ID,
 * 6 byte MAC address, 1-16 bytes of msg data */
#define HM_VENDOR_MSG_HDR_LEN                       9
#define HM_VENDOR_MSG_DATA_LEN_MIN                  1
#define HM_VENDOR_MSG_DATA_LEN_MAX                  16
#define HM_VENDOR_MSG_SEND_CMD_VID0                 \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word0 */
#define HM_VENDOR_MSG_SEND_CMD_VID1                 \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word0 */
#define HM_VENDOR_MSG_SEND_CMD_VID2                 \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word0 */
#define HM_VENDOR_MSG_SEND_CMD_MAC_ADDR0            \
		FIELD_DEFINE(8, 24, 0) /* b31:24 of word0 */
#define HM_VENDOR_MSG_SEND_CMD_MAC_ADDR1            \
		FIELD_DEFINE(8, 0, 1)  /* b7:0 of word1 */
#define HM_VENDOR_MSG_SEND_CMD_MAC_ADDR2            \
		FIELD_DEFINE(8, 8, 1)  /* b15:8 of word1 */
#define HM_VENDOR_MSG_SEND_CMD_MAC_ADDR3            \
		FIELD_DEFINE(8, 16, 1) /* b23:16 of word1 */
#define HM_VENDOR_MSG_SEND_CMD_MAC_ADDR4            \
		FIELD_DEFINE(8, 24, 1) /* b31:24 of word1 */
#define HM_VENDOR_MSG_SEND_CMD_MAC_ADDR5            \
		FIELD_DEFINE(8, 0, 2)  /* b7:0 of word2 */
#define HM_VENDOR_MSG_SEND_CMD_MSG_DATA0            \
		FIELD_DEFINE(8, 8, 2)  /* b15:8 of word2 */

/* AP->Bow general diagnostic and debug commands - shared by WIHD and MHL */

/* HM_DIAG_COMMAND_CMD has 1-1023 bytes of command data */
#define HM_DIAG_COMMAND_CMD_MLEN_MIN                1
#define HM_DIAG_COMMAND_CMD_MLEN_MAX                0x3FF
#define HM_DIAG_COMMAND_CMD_DATA0                   \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_DEBUG_OUTPUT_SET_PATH_CMD has 1 byte for the "path" value */
#define HM_DEBUG_OUTPUT_SET_PATH_CMD_MLEN           1
#define HM_DEBUG_OUTPUT_SET_PATH_CMD_PATH           \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* all currently defined MHL commands are AP->Bow */

/* no payload in HM_MHL_GET_CONNECTION_STATE_CMD */
#define HM_MHL_GET_CONNECTION_STATE_CMD_MLEN        0

/* no payload in HM_MHL_MEASURE_ID_IMPEDANCE_CMD */
#define HM_MHL_MEASURE_ID_IMPEDANCE_CMD_MLEN        0

/* HM_MHL_READ_LOCAL_SPAD_CMD has 1 byte offset and 1 byte length */
#define HM_MHL_READ_LOCAL_SPAD_CMD_MLEN             2
#define HM_MHL_READ_LOCAL_SPAD_CMD_DATALEN_MAX      64
#define HM_MHL_READ_LOCAL_SPAD_CMD_OFFSET           \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */
#define HM_MHL_READ_LOCAL_SPAD_CMD_LENGTH           \
		FIELD_DEFINE(8, 8, 0) /* b15:8 of word 0 */

/* HM_MHL_WRITE_REMOTE_SPAD_CMD is variable length.
 * 1 byte each for offset and length, 0-255 bytes of data. */
#define HM_MHL_WRITE_REMOTE_SPAD_CMD_MLEN_MIN       2
#define HM_MHL_WRITE_REMOTE_SPAD_CMD_DATALEN_MAX    64
#define HM_MHL_WRITE_REMOTE_SPAD_CMD_MLEN_MAX       \
		(HM_MHL_WRITE_REMOTE_SPAD_CMD_MLEN_MIN + \
		 HM_MHL_WRITE_REMOTE_SPAD_CMD_DATALEN_MAX)

#define HM_MHL_WRITE_REMOTE_SPAD_CMD_OFFSET         \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_MHL_WRITE_REMOTE_SPAD_CMD_LENGTH         \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_MHL_WRITE_REMOTE_SPAD_CMD_DATA_0         \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_MHL_SEND_UCP_CHAR_CMD has 1 byte payload: The UCP key */
#define HM_MHL_SEND_UCP_CHAR_CMD_MLEN               1
#define HM_MHL_SEND_UCP_CHAR_CMD_CHAR               \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_MHL_SEND_UCP_ACK_CMD has 2 byte payload:
 * the UCP ack/nack, UCP error code (only set if nack) */
#define HM_MHL_SEND_UCP_ACK_CMD_MLEN                2
#define HM_MHL_SEND_UCP_ACK_CMD_ACK                 \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */
#define HM_MHL_SEND_UCP_ACK_CMD_ERR                 \
		FIELD_DEFINE(8, 8, 0) /* b15:8 of word 0 */

/* HM_MHL_SEND_RCP_KEY_CMD has 1 byte payload: the RCP key */
#define HM_MHL_SEND_RCP_KEY_CMD_MLEN                1
#define HM_MHL_SEND_RCP_KEY_CMD_KEY                 \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_MHL_SEND_RCP_ACK_CMD has 2 byte payload:
 * the RCP ack/nack, RCP error code (only set if nack) */
#define HM_MHL_SEND_RCP_ACK_CMD_MLEN                2
#define HM_MHL_SEND_RCP_ACK_CMD_ACK                 \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */
#define HM_MHL_SEND_RCP_ACK_CMD_ERR                 \
		FIELD_DEFINE(8, 8, 0) /* b15:8 of word 0 */

/* HM_MHL_SEND_RAP_ACTION_CMD has 1 byte payload: the RAP Action */
#define HM_MHL_SEND_RAP_ACTION_CMD_MLEN             1
#define HM_MHL_SEND_RAP_ACTION_CMD_ACTION           \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_MHL_SEND_RAP_ACK_CMD has 1 byte payload: the RAPK error code */
#define HM_MHL_SEND_RAP_ACK_CMD_MLEN                1
#define HM_MHL_SEND_RAP_ACK_CMD_ERR                 \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_MHL_DEVCAP_LOCAL_READ_CMD */
#define HM_MHL_DEVCAP_LOCAL_READ_CMD_MLEN           2
#define HM_MHL_DEVCAP_LOCAL_READ_CMD_OFFSET         \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */
#define HM_MHL_DEVCAP_LOCAL_READ_CMD_DATALEN        \
		FIELD_DEFINE(8, 8, 0) /* b15:8 of word 0 */

/* HM_MHL_DEVCAP_LOCAL_WRITE_CMD */
#define HM_MHL_DEVCAP_LOCAL_WRITE_CMD_MLEN_MIN      2
#define HM_MHL_DEVCAP_LOCAL_WRITE_CMD_DATALEN_MAX   \
		HM_MHL_DEVCAP_MAX_CAPABILITY_SIZE
#define HM_MHL_DEVCAP_LOCAL_WRITE_CMD_MLEN_MAX      \
		(HM_MHL_DEVCAP_LOCAL_WRITE_CMD_MLEN_MIN + \
		 HM_MHL_DEVCAP_LOCAL_WRITE_CMD_DATALEN_MAX)
#define HM_MHL_DEVCAP_LOCAL_WRITE_CMD_OFFSET        \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_MHL_DEVCAP_LOCAL_WRITE_CMD_RSVD          \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_MHL_DEVCAP_LOCAL_WRITE_CMD_DATA0         \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_MHL_DEVCAP_REMOTE_READ_CMD */
#define HM_MHL_DEVCAP_REMOTE_READ_CMD_MLEN          2
#define HM_MHL_DEVCAP_REMOTE_READ_CMD_OFFSET        \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */
#define HM_MHL_DEVCAP_REMOTE_READ_CMD_DATALEN       \
		FIELD_DEFINE(8, 8, 0) /* b15:8 of word 0 */

/* HM_MHL_DEVCAP_REMOTE_WRITE_CMD */
#define HM_MHL_DEVCAP_REMOTE_WRITE_CMD_MLEN_MIN     2
#define HM_MHL_DEVCAP_REMOTE_WRITE_CMD_DATALEN_MAX  \
		HM_MHL_DEVCAP_MAX_CAPABILITY_SIZE
#define HM_MHL_DEVCAP_REMOTE_WRITE_CMD_MLEN_MAX     \
		(HM_MHL_DEVCAP_REMOTE_WRITE_CMD_MLEN_MIN + \
		 HM_MHL_DEVCAP_REMOTE_WRITE_CMD_DATALEN_MAX)
#define HM_MHL_DEVCAP_REMOTE_WRITE_CMD_OFFSET       \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_MHL_DEVCAP_REMOTE_WRITE_CMD_RSVD         \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_MHL_DEVCAP_REMOTE_WRITE_CMD_DATA0        \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_NV_STORAGE_WRITE_CMD is a special case.
 * Can have 0 - 512 bytes of data at an offset address following data length,
 * a reserved field for alignment, and a 32 bite offset.
 * Should only be accessed using the field_ua_xxx() calls.
 * The payload is best written by allocating the buffer then using
 * FIELD_UA_BYTE_ADDRESS to get the address of the DATA0 byte. */
#define HM_NV_STORAGE_WRITE_CMD_MLEN_MIN            6
#define HM_NV_STORAGE_WRITE_CMD_MLEN_MAX            518
#define HM_NV_STORAGE_WRITE_CMD_DATA_LENGTH         \
		FIELD_DEFINE(10, 0, 0)  /* b9:0 of word 0 */
#define HM_NV_STORAGE_WRITE_CMD_RSVD                \
		FIELD_DEFINE(6, 10, 0)  /* b15:10 of word 0 */
/* wraps from b15 of word 1 to b16 of word 0 - UA access only! */
#define HM_NV_STORAGE_WRITE_CMD_SRC_OFFSET          \
		FIELD_DEFINE(32, 16, 0)
/* use with FIELD_UA_BYTE_ADDRESS */
#define HM_NV_STORAGE_WRITE_CMD_DATA0               \
		FIELD_DEFINE(8, 16, 1)  /* bits 23:16 of word 1 */

/* HM_NV_STORAGE_READ_CMD has same length and offset fields as
 * STORAGE_WRITE, but no other payload. */
#define HM_NV_STORAGE_READ_CMD_MLEN                 6
#define HM_NV_STORAGE_READ_CMD_DATA_LENGTH          \
		FIELD_DEFINE(10, 0, 0)  /* b9:0 of word 0 */
#define HM_NV_STORAGE_READ_CMD_RSVD                 \
		FIELD_DEFINE(6, 10, 0)  /* b15:10 of word 0 */
/* wraps from b15 of word 1 to b16 of word 0 - UA access only! */
#define HM_NV_STORAGE_READ_CMD_SRC_OFFSET           \
		FIELD_DEFINE(32, 16, 0)

/* Responses - first Bow->AP then AP->Bow.
 * Note that ALL responses have the 8 bit "result code" field in common. */

/* HM_COMMON_CMD_RSP_MLEN is used for generic error command responses
 * for unknown commands. */
#define HM_COMMON_CMD_RSP_MLEN                      1
#define HM_COMMON_CMD_RSP_RESULT_CODE               \
		FIELD_DEFINE(8, 0, 0)   /* b7:0 of word 0 */

/* HM_GET_CHIP_VERSION_CMD_RSP returns:
 * result code, 1 reserved byte, 0-32 bytes of version number string. */
#define HM_GET_CHIP_VERSION_CMD_RSP_MLEN_MIN        2
#define HM_GET_CHIP_VERSION_CMD_RSP_MLEN_MAX        34
#define HM_GET_CHIP_VERSION_CMD_RSP_RSVD            \
		FIELD_DEFINE(8, 8, 0)   /* b15:8  of word 0 */
#define HM_GET_CHIP_VERSION_CMD_RSP_STRING0         \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 */

/* HW_GET_FW_VERSION_CMD_RSP returns:
 * result code, 1 reserved byte, 0-32 bytes of version string. */
#define HM_GET_FW_VERSION_CMD_RSP_MLEN_MIN          2
#define HM_GET_FW_VERSION_CMD_RSP_MLEN_MAX          34
#define HM_GET_FW_VERSION_CMD_RSP_RSVD              \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
#define HM_GET_FW_VERSION_STRING_0                  \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 */
#define HM_GET_FW_VERSION_STRING_1                  \
		FIELD_DEFINE(8, 24, 0)  /* b31:24 of word 0 */
/* no more defined. The data is a string, and the best strategy is to use
 * FIELD_UA_BYTE_ADDRESS() for HM_GET_FW_VERSION_STRING_0 and move the data
 * with memcpy() for the appropriate length. */

/* HM_SET_OUTPUT_MODE_CMD_RSP returns only the result code. */
#define HM_SET_OUTPUT_MODE_CMD_RSP_MLEN             1

/* HW_GET_OUTPUT_MODE_CMD_RSP returns:
 * result code, 1 byte mode. */
#define HM_GET_OUTPUT_MODE_CMD_RSP_MLEN             2
#define HM_GET_OUTPUT_MODE_CMD_RSP_MODE             \
		FIELD_DEFINE(8, 8, 0)   /* bit 15:8 of word 0 */

/* HM_REMOTE_DEV_CONNECT_CMD_RSP returns only the result code. */
#define HM_REMOTE_DEV_CONNECT_CMD_RSP_MLEN          1

/* HM_REMOTE_DEV_DISCONNECT_CMD_RSP returns only the result code. */
#define HM_REMOTE_DEV_DISCONNECT_CMD_RSP_MLEN       1

/* HM_REMOTE_DEV_GET_INFO_CMD_RSP is either 1 or 28 bytes in size.
 * If no remote device is currently connected, just returns "success" code
 * with no data.
 * If connected, must return all info below: 8 bits reserved, 64 bits of MAC
 * address, 8 bits each category and AV type, and 16 bytes of device name. */
/* NOTE: the data structure from MAC_ADDR0 on is identical to the
 * HM_GET_DEV_LIST_CMD_RSP_DEV_X_ defines below, and they can be used to
 * access this structure by setting the base address to:
 * "(void *) FIELD_UA_BYTE_ADDRESS(msgPtr,
 * HM_REMOTE_DEV_GET_INFO_CMD_RSP_MAC_ADDR0)" */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_MLEN         28
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_RSVD         \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_MAC_ADDR0    \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_MAC_ADDR1    \
		FIELD_DEFINE(8, 24, 0)  /* b31:24 of word 0 */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_MAC_ADDR2    \
		FIELD_DEFINE(8, 0, 1)   /* b7:0 of word 1 */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_MAC_ADDR3    \
		FIELD_DEFINE(8, 8, 1)   /* b15:8 of word 1 */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_MAC_ADDR4    \
		FIELD_DEFINE(8, 16, 1)  /* b23:16 of word 1 */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_MAC_ADDR5    \
		FIELD_DEFINE(8, 24, 1)  /* b31:24 of word 1 */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_CATEGORY     \
		FIELD_DEFINE(8, 0, 2)   /* b7:8 of word 2 */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_AV_TYPE      \
		FIELD_DEFINE(8, 8, 2)   /* b15:8 of word 2 */

/* if read/written as 16 bits byte order is unchanged */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_MFG_ALL      \
		FIELD_DEFINE(16, 16, 2)  /* b15:0 of MFG ID */
/* if read/written in BIG ENDIAN byte order */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_MFG_MSB      \
		FIELD_DEFINE(8, 16, 2)   /* b15:8 of MFG ID */
/* if read/written in BIG ENDIAN byte order */
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_MFG_LSB      \
		FIELD_DEFINE(8, 24, 2)   /* b7:0 of MFG ID */

#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_0       FIELD_DEFINE(8, 0, 3)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_1       FIELD_DEFINE(8, 8, 3)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_2       FIELD_DEFINE(8, 16, 3)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_3       FIELD_DEFINE(8, 24, 3)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_4       FIELD_DEFINE(8, 0, 4)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_5       FIELD_DEFINE(8, 8, 4)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_6       FIELD_DEFINE(8, 16, 4)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_7       FIELD_DEFINE(8, 24, 4)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_8       FIELD_DEFINE(8, 0, 5)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_9       FIELD_DEFINE(8, 8, 5)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_10      FIELD_DEFINE(8, 16, 5)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_11      FIELD_DEFINE(8, 24, 5)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_12      FIELD_DEFINE(8, 0, 6)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_13      FIELD_DEFINE(8, 8, 6)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_14      FIELD_DEFINE(8, 16, 6)
#define HM_REMOTE_DEV_GET_INFO_CMD_RSP_NAME_15      FIELD_DEFINE(8, 24, 6)

/* HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD_RSP returns:
 * result code, 1 byte strength. */
#define HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD_RSP_MLEN     2
#define HM_REMOTE_DEV_GET_SIGNAL_STRENGTH_CMD_RSP_STRENGTH \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */

/* HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP returns:
 * result code, 1 reserved byte, 13 bytes of monitor name beyond status. */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_MLEN      15
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_MLEN 13
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_RSVD      \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_0    \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_1    \
		FIELD_DEFINE(8, 24, 0)  /* b31:24 of word 0 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_2    \
		FIELD_DEFINE(8, 0, 1)   /* b7:0 of word 1 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_3    \
		FIELD_DEFINE(8, 8, 1)   /* b15:8 of word 1 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_4    \
		FIELD_DEFINE(8, 16, 1)  /* b23:16 of word 1 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_5    \
		FIELD_DEFINE(8, 24, 1)  /* b31:24 of word 1 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_6    \
		FIELD_DEFINE(8, 0, 2)   /* b7:8 of word 2 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_7    \
		FIELD_DEFINE(8, 8, 2)   /* b15:8 of word 2 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_8    \
		FIELD_DEFINE(8, 16, 2)  /* b23:16 of word 2 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_9    \
		FIELD_DEFINE(8, 24, 2)  /* b31:24 of word 2 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_10   \
		FIELD_DEFINE(8, 0, 3)   /* b7:0 of word 3 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_11   \
		FIELD_DEFINE(8, 8, 3)   /* b15:8 of word 3 */
#define HM_REMOTE_DEV_GET_MONITOR_NAME_CMD_RSP_NAME_12   \
		FIELD_DEFINE(8, 16, 3)  /* b23:16 of word 3 */

/* HM_GET_DEV_LIST_CMD_RSP is a complex message.
 * Besides the 2 byte fixed length section (1 byte each for result code and
 * reserved field), it can contain 0-16 26 byte device descriptors. The message
 * definition will only define the fixed section and the address of the first
 * byte of the first descriptor. The device descriptor definitions may be used
 * to construct and copy the device information as needed at the correct offset
 * in the message. */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_LEN             26
#define HM_GET_DEV_LIST_CMD_RSP_MAX_DEVS            16

#define HM_GET_DEV_LIST_CMD_RSP_MLEN_MIN            2
#define HM_GET_DEV_LIST_CMD_RSP_MLEN_MAX            \
		(HM_GET_DEV_LIST_CMD_RSP_MLEN_MIN + \
		 (HM_GET_DEV_LIST_CMD_RSP_DEV_LEN * \
		  HM_GET_DEV_LIST_CMD_RSP_MAX_DEVS))

#define HM_GET_DEV_LIST_CMD_RSP_RSVD                \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
/* this is only used to get the base address for devices */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_BASE_0          \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 */

/* access macros for device descriptors */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MAC_ADDR0     \
		FIELD_DEFINE(8, 0, 0)   /* b7:0 of word 0 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MAC_ADDR1     \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MAC_ADDR2     \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MAC_ADDR3     \
		FIELD_DEFINE(8, 24, 0)  /* b31:24 of word 0 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MAC_ADDR4     \
		FIELD_DEFINE(8, 0, 1)   /* b7:0 of word 1 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MAC_ADDR5     \
		FIELD_DEFINE(8, 8, 1)   /* b15:8 of word 1 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_CATEGORY      \
		FIELD_DEFINE(8, 16, 1)  /* b23:16 of word 1 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_AV_TYPE       \
		FIELD_DEFINE(8, 24, 1)  /* b31:24 of word 1 of device */

/* if read/written as 16 bits byte order is unchanged */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG_ALL       \
		FIELD_DEFINE(16, 0, 2)  /* b15:0 of MFG ID */
/* ms bit of 16 bit big-endian MFG ID */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG_RSVD      \
		FIELD_DEFINE(1, 0, 2)   /* b0:0 of MFG ID */
/* MS char of 16 bit big-endian MFG ID */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG0          \
		FIELD_DEFINE(5, 1, 2)   /* b5:1 of word 2 of device */
/* middle char of 16 bit big-endian MFG ID */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG1          \
		FIELD_DEFINE(5, 6, 2)   /* b10:6 of word 2 of device */
/* LS char of 16 bit big-endian MFG ID */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_MFG2          \
		FIELD_DEFINE(5, 11, 2)  /* b15:11 of word 2 of device */

#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME0         \
		FIELD_DEFINE(8, 16, 2)  /* b23:16 of word 2 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME1         \
		FIELD_DEFINE(8, 24, 2)  /* b31:24 of word 2 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME2         \
		FIELD_DEFINE(8, 0, 3)   /* b7:0 of word 3 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME3         \
		FIELD_DEFINE(8, 8, 3)   /* b15:8 of word 3 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME4         \
		FIELD_DEFINE(8, 16, 3)  /* b23:16 of word 3 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME5         \
		FIELD_DEFINE(8, 24, 3)  /* b31:24 of word 3 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME6         \
		FIELD_DEFINE(8, 0, 4)   /* b7:0 of word 4 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME7         \
		FIELD_DEFINE(8, 8, 4)   /* b15:8 of word 4 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME8         \
		FIELD_DEFINE(8, 16, 4)  /* b23:16 of word 4 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME9         \
		FIELD_DEFINE(8, 24, 4)  /* b31:24 of word 4 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME10        \
		FIELD_DEFINE(8, 0, 5)   /* b7:0 of word 5 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME11        \
		FIELD_DEFINE(8, 8, 5)   /* b15:8 of word 5 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME12        \
		FIELD_DEFINE(8, 16, 5)  /* b23:16 of word 5 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME13        \
		FIELD_DEFINE(8, 24, 5)  /* b31:24 of word 5 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME14        \
		FIELD_DEFINE(8, 0, 6)   /* b7:0 of word 6 of device */
#define HM_GET_DEV_LIST_CMD_RSP_DEV_X_NAME15        \
		FIELD_DEFINE(8, 8, 6)   /* b15:8 of word 6 of device */

/* HM_LOCAL_DEV_SET_MAC_ADDR_CMD_RSP returns only the result code. */
#define HM_LOCAL_DEV_SET_MAC_ADDR_CMD_RSP_MLEN      1

/* HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP returns:
 * result code, 1 reserved byte, 6 bytes of MAC address. */
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP_MLEN      8
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP_RSVD      \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP_MAC_ADDR0 \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 */
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP_MAC_ADDR1 \
		FIELD_DEFINE(8, 24, 0)  /* b31:24 of word 0 */
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP_MAC_ADDR2 \
		FIELD_DEFINE(8, 0, 1)   /* b7:0 of word 1 */
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP_MAC_ADDR3 \
		FIELD_DEFINE(8, 8, 1)   /* b15:8 of word 1 */
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP_MAC_ADDR4 \
		FIELD_DEFINE(8, 16, 1)  /* b23:16 of word 1 */
#define HM_LOCAL_DEV_GET_MAC_ADDR_CMD_RSP_MAC_ADDR5 \
		FIELD_DEFINE(8, 24, 1)  /* b31:24 of word 1 */

/* HM_LOCAL_DEV_SET_NAME_CMD_RSP returns only the result code. */
#define HM_LOCAL_DEV_SET_NAME_CMD_RSP_MLEN          1

/* HM_LOCAL_DEV_GET_NAME_CMD_RSP returns:
 * result code, 1 reserved byte, 16 byte name (null terminated if less than
 * maximum length). */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_MLEN          18
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_LEN      16
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_RSVD          \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_0        \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_1        \
		FIELD_DEFINE(8, 24, 0)  /* b31:24 of word 0 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_2        \
		FIELD_DEFINE(8, 0, 1)   /* b7:0 of word 1 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_3        \
		FIELD_DEFINE(8, 8, 1)   /* b15:8 of word 1 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_4        \
		FIELD_DEFINE(8, 16, 1)  /* b23:16 of word 1 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_5        \
		FIELD_DEFINE(8, 24, 1)  /* b31:24 of word 1 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_6        \
		FIELD_DEFINE(8, 0, 2)   /* b7:8 of word 2 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_7        \
		FIELD_DEFINE(8, 8, 2)   /* b15:8 of word 2 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_8        \
		FIELD_DEFINE(8, 16, 2)  /* b23:16 of word 2 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_9        \
		FIELD_DEFINE(8, 24, 2)  /* b31:24 of word 2 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_10       \
		FIELD_DEFINE(8, 0, 3)   /* b7:0 of word 3 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_11       \
		FIELD_DEFINE(8, 8, 3)   /* b15:8 of word 3 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_12       \
		FIELD_DEFINE(8, 16, 3)  /* b23:16 of word 3 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_13       \
		FIELD_DEFINE(8, 24, 3)  /* b31:24 of word 3 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_14       \
		FIELD_DEFINE(8, 0, 4)   /* b7:0 of word 4 */
#define HM_LOCAL_DEV_GET_NAME_CMD_RSP_NAME_15       \
		FIELD_DEFINE(8, 8, 4)   /* b15:8 of word 4 */

/* HM_LOCAL_DEV_SET_REGULATORY_ID_CMD_RSP returns only the result code. */
#define HM_LOCAL_DEV_SET_REGULATORY_ID_CMD_RSP_MLEN 1

/* HM_LOCAL_DEV_GET_REGULATORY_ID_CMD_RSP returns:
 * result code, 1 byte regulatory ID. */
#define HM_LOCAL_DEV_GET_REGULATORY_ID_CMD_RSP_MLEN 2
#define HM_LOCAL_DEV_GET_REGULATORY_ID_CMD_RSP_REG_ID \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */

/* HM_WVAN_GET_CONNECTION_STATUS_CMD returns the result code and status */
#define HM_WVAN_GET_CONNECTION_STATUS_CMD_RSP_MLEN  2
#define HM_WVAN_GET_CONNECTION_STATUS_CMD_RSP_CONNECTION_STATUS \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */

/* HM_SET_VENDOR_MSG_FILTER_CMD_RSP returns only the result code. */
#define HM_SET_VENDOR_MSG_FILTER_CMD_RSP_MLEN       1

/* HM_GET_VENDOR_MSG_FILTER_CMD_RSP returns result code and
 * 3 bytes vendor ID. */
#define HM_GET_VENDOR_MSG_FILTER_CMD_RSP_MLEN       4
#define HM_GET_VENDOR_MSG_FILTER_CMD_RSP_VID0       \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_GET_VENDOR_MSG_FILTER_CMD_RSP_VID1       \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */
#define HM_GET_VENDOR_MSG_FILTER_CMD_RSP_VID2       \
		FIELD_DEFINE(8, 24, 0) /* b31:24 of word 0 */

/* HM_REMOTE_DEV_VENDOR_MSG_SEND_CMD_RSP returns only result code. */
#define HM_REMOTE_DEV_VENDOR_MSG_SEND_CMD_RSP_MLEN  1

/* HM_WVAN_JOIN_CMD_RSP returns only the result code. */
#define HM_WVAN_JOIN_CMD_RSP_MLEN                   1

/* HM_WVAN_LEAVE_CMD_RSP returns only the result code. */
#define HM_WVAN_LEAVE_CMD_RSP_MLEN                  1

/* HM_WVAN_SCAN_CMD_RSP is a complex message.
 * Besides the 2 byte fixed length SECTION_ALL_ACCESS (1 byte each for result
 * code and reserved field), it can contain 0-10 36 byte WVAN descriptors. The
 * message definition will only define the fixed section and the address of the
 * first byte of the first descriptor. The WVAN descriptor definitions may be
 * used to construct and copy the WVAN data as needed at the correct offset in
 * the message. */
#define HM_WVAN_SCAN_CMD_RSP_DEV_LEN                36
#define HM_WVAN_SCAN_CMD_RSP_MAX_DEVS               10

#define HM_WVAN_SCAN_CMD_RSP_MLEN_MIN               2
#define HM_WVAN_SCAN_CMD_RSP_MLEN_MAX               \
		(HM_WVAN_SCAN_CMD_RSP_MLEN_MIN + \
		 (HM_WVAN_SCAN_CMD_RSP_DEV_LEN * \
		  HM_WVAN_SCAN_CMD_RSP_MAX_DEVS))

#define HM_WVAN_SCAN_CMD_RSP_RSVD                   \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
/* this is only used to get the base address for devices */
#define HM_WVAN_SCAN_CMD_RSP_WVAN_BASE_0            \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 */

/* access macros for wvan descriptors */
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME_MLEN  32
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_ID         \
		FIELD_DEFINE(8, 0, 0)   /* b7:0 of word 0 of wvan */
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_HR_CHANNEL      \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 of wvan */
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_LR_CHANNEL      \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 of wvan */
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_BEACON_STR      \
		FIELD_DEFINE(8, 24, 0)  /* b31:24 of word 0 of wvan */
/* name is 32 bytes */
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME0      \
		FIELD_DEFINE(8, 0, 1)   /* b7:0 of word 1 of wvan */
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME1      FIELD_DEFINE(8, 8, 1)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME2      FIELD_DEFINE(8, 16, 1)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME3      FIELD_DEFINE(8, 24, 1)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME4      FIELD_DEFINE(8, 0, 2)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME5      FIELD_DEFINE(8, 8, 2)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME6      FIELD_DEFINE(8, 16, 2)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME7      FIELD_DEFINE(8, 24, 2)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME8      FIELD_DEFINE(8, 0, 3)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME9      FIELD_DEFINE(8, 8, 3)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME10     FIELD_DEFINE(8, 16, 3)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME11     FIELD_DEFINE(8, 24, 3)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME12     FIELD_DEFINE(8, 0, 4)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME13     FIELD_DEFINE(8, 8, 4)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME14     FIELD_DEFINE(8, 16, 4)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME15     FIELD_DEFINE(8, 24, 4)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME16     FIELD_DEFINE(8, 0, 5)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME17     FIELD_DEFINE(8, 8, 5)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME18     FIELD_DEFINE(8, 16, 5)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME19     FIELD_DEFINE(8, 24, 5)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME20     FIELD_DEFINE(8, 0, 6)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME21     FIELD_DEFINE(8, 8, 6)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME22     FIELD_DEFINE(8, 16, 6)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME23     FIELD_DEFINE(8, 24, 6)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME24     FIELD_DEFINE(8, 0, 7)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME25     FIELD_DEFINE(8, 8, 7)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME26     FIELD_DEFINE(8, 16, 7)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME27     FIELD_DEFINE(8, 24, 7)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME28     FIELD_DEFINE(8, 0, 8)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME29     FIELD_DEFINE(8, 8, 8)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME30     FIELD_DEFINE(8, 16, 8)
#define HM_WVAN_SCAN_CMD_RSP_WVAN_X_WVAN_NAME31     FIELD_DEFINE(8, 24, 8)

/* HM_CEC_MSG_SEND_CMD_RSP returns only the result code. */
#define HM_CEC_MSG_SEND_CMD_RSP_MLEN                1

/* HM_HDCP_SET_POLICY_CMD_RSP returns only the result code. */
#define HM_HDCP_SET_POLICY_CMD_RSP_MLEN             1

/* HM_HDCP_GET_POLICY_CMD_RSP returns:
 * result code, 1 byte policy. */
#define HM_HDCP_GET_POLICY_CMD_RSP_MLEN             2
#define HM_HDCP_GET_POLICY_CMD_RSP_POLICY           \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */

/* HM_HDCP_SET_STREAM_TYPE_CMD_RSP returns only the result code. */
#define HM_HDCP_SET_STREAM_TYPE_CMD_RSP_MLEN        1

/* HM_HDCP_GET_STREAM_TYPE_CMD_RSP returns:
 * result code, 1 byte stream_type. */
#define HM_HDCP_GET_STREAM_TYPE_CMD_RSP_MLEN        2
#define HM_HDCP_GET_STREAM_TYPE_CMD_RSP_STREAM_TYPE \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */

/* Bow->AP general diagnostic and debug command responses.
 * (Shared by WIHD and MHL) */

/* HM_DIAG_COMMAND_CMD_RSP returns:
 * result code, 1 reserved byte, 2 byte command ID used by AP to map
 * notification messages to the correct command.
 * Command ID is in lttle endian order. */
#define HM_DIAG_COMMAND_CMD_RSP_MLEN                4
#define HM_DIAG_COMMAND_CMD_RSP_RSVD                \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
#define HM_DIAG_COMMAND_CMD_RSP_CMD_ID_LSB          \
		FIELD_DEFINE(8, 16, 0)  /* b23:16 of word 0 */
#define HM_DIAG_COMMAND_CMD_RSP_CMD_ID_MSB          \
		FIELD_DEFINE(8, 24, 0)  /* b31:24 of word 0 */
/* OK for little endian CPU to shortcut */
#define HM_DIAG_COMMAND_CMD_RSP_CMD_ID              \
		FIELD_DEFINE(16, 16, 0) /* b31:16 of word 0 */

/* HM_DEBUG_OUTPUT_SET_PATH_CMD_RSP returns only the result code. */
#define HM_DEBUG_OUTPUT_SET_PATH_CMD_RSP_MLEN       1

/* all currently defined MHL command responses are Bow->AP. */

/* HM_MHL_GET_CONNECTION_STATE_CMD_RSP returns:
 * result code, 1 byte connection state. */
#define HM_MHL_GET_CONNECTION_STATE_CMD_RSP_MLEN    2
#define HM_MHL_GET_CONNECTION_STATE_CMD_RSP_CONNECT_STATE \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */

/* HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP returns:
 * result code, 1 reserved byte, 4 bytes impedance (little endian byte order) */
#define HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP_MLEN    6
#define HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP_RSVD    \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
/* wraps from b16 of word 0 to b15 of word 1 - UA access only! */
#define HM_MHL_MEASURE_ID_IMPEDANCE_CMD_RSP_IMPEDANCE FIELD_DEFINE(32, 16, 0)

/* HM_MHL_SET_CLOCK_SWING_CMD has 1 byte payload: the Clock Swing value */
#define HM_MHL_SET_CLOCK_SWING_CMD_MLEN             1
#define HM_MHL_SET_CLOCK_SWING_CMD_VALUE            \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_MHL_SET_CBUS_VOLTAGE_CMD */
#define HM_MHL_SET_CBUS_VOLTAGE_CMD_MLEN      3
#define HM_MHL_SET_CBUS_VOLTAGE_CMD_VALUE           \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */
#define HM_MHL_SET_CBUS_VOLTAGE_CMD_PERIOD_LO      \
		FIELD_DEFINE(8, 8, 0) /* b15:8 of word 0 */
#define HM_MHL_SET_CBUS_VOLTAGE_CMD_PERIOD_HI       \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_MHL_READ_LOCAL_SPAD_CMD_RSP returns:
 * result code, 1 reserved byte, 0-255 bytes data (no offset returned). */
#define HM_MHL_READ_LOCAL_SPAD_CMD_RSP_MLEN_MIN     2
#define HM_MHL_READ_LOCAL_SPAD_CMD_RSP_DATALEN_MAX  64
#define HM_MHL_READ_LOCAL_SPAD_CMD_RSP_MLEN_MAX     \
		(HM_MHL_READ_LOCAL_SPAD_CMD_RSP_MLEN_MIN + \
		 HM_MHL_READ_LOCAL_SPAD_CMD_RSP_DATALEN_MAX)

#define HM_MHL_READ_LOCAL_SPAD_CMD_RSP_RSVD         \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_MHL_READ_LOCAL_SPAD_CMD_RSP_DATA_0       \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_MHL_WRITE_REMOTE_SPAD_CMD_RSP returns only the result code. */
#define HM_MHL_WRITE_REMOTE_SPAD_CMD_RSP_MLEN       1

/* HM_MHL_SEND_UCP_CHAR_CMD_RSP returns only the result code. */
#define HM_MHL_SEND_UCP_CHAR_CMD_RSP_MLEN           1

/* HM_MHL_SEND_UCP_ACK_CMD_RSP returns only the result code. */
#define HM_MHL_SEND_UCP_ACK_CMD_RSP_MLEN            1

/* HM_MHL_SEND_RCP_KEY_CMD_RSP returns only the result code. */
#define HM_MHL_SEND_RCP_KEY_CMD_RSP_MLEN            1

/* HM_MHL_SEND_RCP_ACK_CMD_RSP returns only the result code. */
#define HM_MHL_SEND_RCP_ACK_CMD_RSP_MLEN            1

/* HM_MHL_SEND_RAP_ACTION_CMD_RSP returns only the result code. */
#define HM_MHL_SEND_RAP_ACTION_CMD_RSP_MLEN         1

/* HM_MHL_SEND_RAP_ACK_CMD_RSP returns only the result code. */
#define HM_MHL_SEND_RAP_ACK_CMD_RSP_MLEN            1

/* HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP (2 + DATALEN_MAX bytes) returns:
 * result code, 1 reserved byte, 0-N bytes of data. */
#define HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_MLEN_MIN   2
#define HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_DATALEN_MAX \
		HM_MHL_DEVCAP_MAX_CAPABILITY_SIZE
#define HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_MLEN_MAX   \
		(HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_MLEN_MIN + \
		 HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_DATALEN_MAX)
#define HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_RSVD       \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_DATA0      \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_MHL_DEVCAP_LOCAL_WRITE_CMD_RSP returns only the result code. */
#define HM_MHL_DEVCAP_LOCAL_WRITE_CMD_RSP_MLEN      1

/* HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP (2 + DATALEN_MAX bytes) returns:
 * result code, 1 reserved byte, 0-N bytes of data. */
#define HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP_MLEN_MIN  2
#define HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP_DATALEN_MAX \
		HM_MHL_DEVCAP_MAX_CAPABILITY_SIZE
#define HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP_MLEN_MAX  \
		(HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_MLEN_MIN + \
		 HM_MHL_DEVCAP_LOCAL_READ_CMD_RSP_DATALEN_MAX)
#define HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP_RSVD      \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_MHL_DEVCAP_REMOTE_READ_CMD_RSP_DATA0     \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_MHL_DEVCAP_REMOTE_WRITE_CMD_RSP returns only the result code. */
#define HM_MHL_DEVCAP_REMOTE_WRITE_CMD_RSP_MLEN     1

/* command responses for nonvolatile storage. */

/* HM_NV_STORAGE_WRITE_CMD_RSP returns:
 * result code, 1 reserved byte, 16 bit data_length (bytes written). */
#define HM_NV_STORAGE_WRITE_CMD_RSP_MLEN                4
#define HM_NV_STORAGE_WRITE_CMD_RSP_RSVD                \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
#define HM_NV_STORAGE_WRITE_CMD_RSP_DATA_LENGTH         \
		FIELD_DEFINE(16, 16, 0) /* b31:16 of word 0 */

/* HM_NV_STORAGE_READ_CMD_RSP returns:
 * result code, reserved byte, data length (16 bits), 0-512 bytes of data. */
#define HM_NV_STORAGE_READ_CMD_RSP_MLEN_MIN             4
#define HM_NV_STORAGE_READ_CMD_RSP_MLEN_MAX             516
#define HM_NV_STORAGE_READ_CMD_RSP_RSVD                 \
		FIELD_DEFINE(8, 8, 0)   /* b15:8 of word 0 */
#define HM_NV_STORAGE_READ_CMD_RSP_DATA_LENGTH          \
		FIELD_DEFINE(16, 16, 0) /* b31:16 of word 0 */
#define HM_NV_STORAGE_READ_CMD_RSP_DATA0                \
		FIELD_DEFINE(8, 0, 1)   /* b7:0 of word 1 */

/* Bow->AP Notification messages
 * These have been redefined to include (possibly multi-byte) reason fields.
 * Notification-specific reasons TBD. */

/* Bow->AP notifications */
/* updated to A.01 version of spec (March 28, 2013) */

/* HM_WVAN_ASSOCIATION_RESULT_NOTIFY currently has no extra data. */
#define HM_WVAN_ASSOCIATION_RESULT_NOTIFY_MLEN      0

/* HM_WVAN_DISASSOCIATED_NOTIFY has a 1 byte reason code. */
#define HM_WVAN_DISASSOCIATED_NOTIFY_MLEN           1
#define HM_WVAN_DISASSOCIATED_NOTIFY_REASON         \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_REMOTE_DEV_AV_ENABLED_NOTIFY currently has no extra data. */
#define HM_REMOTE_DEV_AV_ENABLED_NOTIFY_MLEN        0

/* HM_REMOTE_DEV_CONNECTION_READY_NOTIFY currently has no extra data. */
#define HM_REMOTE_DEV_CONNECTION_READY_NOTIFY_MLEN  0

/* HM_REMOTE_DEV_DISCONNECTED_NOTIFY currently has 1 byte reason code.
 * any enum SiI_Notification_Reason value is legal, though it may not be
 * possible to easily determine and return all possible reason values. */
#define HM_REMOTE_DEV_DISCONNECTED_NOTIFY_MLEN      1
#define HM_REMOTE_DEV_DISCONNECTED_NOTIFY_REASON    \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY has 3 bytes of data */
#define HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY_MLEN       3
#define HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY_CEC_MSG0   \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY_CEC_MSG1   \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_CEC_USER_CTRL_MSG_RCVD_NOTIFY_CEC_MSG2   \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_DEV_LIST_CHANGED is defined in section 6.2.1.7 of
 * SiL-PR-1084-B.01-2013-08-06. It is a complex message, identical to
 * HM_GET_DEV_LIST_CMD_RSP except that both fields of the 2 byte fixed length
 * section are reserved. It can contain 0-16 26 byte device descriptors. The
 * message definition will only define the fixed section and the address of the
 * first byte of the first descriptor. The device descriptor definitions may be
 * used to construct and copy the device information as needed at the correct
 * offset in the message. */
#define HM_DEV_LIST_CHANGED_DEV_LEN                 26
#define HM_DEV_LIST_CHANGED_MAX_DEVS                16

#define HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MIN         2
#define HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MAX         \
		(HM_DEV_LIST_CHANGED_NOTIFY_MLEN_MIN + \
		 (HM_DEV_LIST_CHANGED_DEV_LEN * \
		  HM_DEV_LIST_CHANGED_MAX_DEVS))

#define HM_DEV_LIST_CHANGED_NOTIFY_RSVD             \
		FIELD_DEFINE(16, 0, 0) /* b15:0 of word 0 */
/* this is only used to get the base address for devices */
#define HM_DEV_LIST_CHANGED_NOTIFY_DEV_BASE_0       \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* access macros for device descriptors
 * These are identical to HM_GET_DEV_LIST_CMD_RSP_DEV_xxxx macros. */

/* HM_VENDOR_MSG_RCVD_NOTIFY has a 3 byte vendor ID,
 * a 6 byte transmitter MAC addr, and 1-16 bytes vendor msg data. */
#define HM_VENDOR_MSG_RCVD_NOTIFY_MLEN_MIN          10
#define HM_VENDOR_MSG_RCVD_NOTIFY_MLEN_MAX          25
#define HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_VID0       \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_VID1       \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_VID2       \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */
#define HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR0  \
		FIELD_DEFINE(8, 24, 0) /* b31:24 of word 0 */
#define HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR1  \
		FIELD_DEFINE(8, 0, 1)  /* b7:0 of word 1 */
#define HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR2  \
		FIELD_DEFINE(8, 8, 1)  /* b15:8 of word 1 */
#define HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR3  \
		FIELD_DEFINE(8, 16, 1) /* b23:16 of word 1 */
#define HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR4  \
		FIELD_DEFINE(8, 24, 1) /* b31:24 of word 1 */
#define HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MAC_ADDR5  \
		FIELD_DEFINE(8, 0, 2)  /* b7:0 of word 2 */
#define HM_VENDOR_MSG_RCVD_NOTIFY_VENDOR_MSG_DATA0  \
		FIELD_DEFINE(8, 8, 2)  /* b15:8 of word 2 */

/* HM_DIAG_CMD_DONE_NOTIFY has a 2 byte cmd_id field (little endian order).
 * This is used by AP to confirm that the diag command with the given ID has
 * finished execution. */
#define HM_DIAG_CMD_DONE_NOTIFY_MLEN                2
#define HM_DIAG_CMD_DONE_NOTIFY_CMD_ID              \
		FIELD_DEFINE(16, 0, 0) /* b15:0 of word 0 */
#define HM_DIAG_CMD_DONE_NOTIFY_CMD_ID_LSB          \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_DIAG_CMD_DONE_NOTIFY_CMD_ID_MSB          \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */

/* HM_DIAG_CMD_OUTPUT_NOTIFY has 0-1023 bytes of payload (0 case is legal
 * but useless).
 * This is used for text output by diagnostic commands. */
#define HM_DIAG_CMD_OUTPUT_NOTIFY_MLEN_MIN          0
#define HM_DIAG_CMD_OUTPUT_NOTIFY_MLEN_MAX          0x3FF
#define HM_DIAG_CMD_OUTPUT_NOTIFY_DATA0             \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_DEBUG_OUTPUT_NOTIFY has 0-1023 bytes of payload (0 case is legal
 * but useless).
 * This is used for log output. */
#define HM_DEBUG_OUTPUT_NOTIFY_MLEN_MIN             0
#define HM_DEBUG_OUTPUT_NOTIFY_MLEN_MAX             0x3FF
#define HM_DEBUG_OUTPUT_NOTIFY_NOTIFY_DATA0         \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_MHL_SINK_CONNECTED_NOTIFY currently has no extra data. */
#define HM_MHL_SINK_CONNECTED_NOTIFY_MLEN           0

/* HM_MHL_SINK_DISCONNECTED_NOTIFY currently has no extra data. */
#define HM_MHL_SINK_DISCONNECTED_NOTIFY_MLEN        0

/* HM_MHL_UCP_CHAR_RCV_NOTIFY has a 1 byte key code. */
#define HM_MHL_UCP_CHAR_RCV_NOTIFY_MLEN             1
#define HM_MHL_UCP_CHAR_RCV_NOTIFY_CHAR             \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_MHL_UCP_ACK_RCV_NOTIFY has 3 bytes of data:
 * 1 byte key code, 1 byte ack value, 1 byte error code (may be ignored
 * for ACK). */
#define HM_MHL_UCP_ACK_RCV_NOTIFY_MLEN              3
#define HM_MHL_UCP_ACK_RCV_NOTIFY_CHAR              \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_MHL_UCP_ACK_RCV_NOTIFY_ACK               \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_MHL_UCP_ACK_RCV_NOTIFY_ERROR             \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_MHL_RCP_KEY_RCV_NOTIFY has a 1 byte key code. */
#define HM_MHL_RCP_KEY_RCV_NOTIFY_MLEN              1
#define HM_MHL_RCP_KEY_RCV_NOTIFY_KEY               \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_MHL_RCP_ACK_RCV_NOTIFY has 3 bytes of data:
 * 1 byte key code, 1 byte ack value, 1 byte error code (may be ignored
 * for ACK). */
#define HM_MHL_RCP_ACK_RCV_NOTIFY_MLEN              3
#define HM_MHL_RCP_ACK_RCV_NOTIFY_KEY               \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_MHL_RCP_ACK_RCV_NOTIFY_ACK               \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_MHL_RCP_ACK_RCV_NOTIFY_ERROR             \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_MHL_RAP_ACTION_RCV_NOTIFY has a 1 byte action code. */
#define HM_MHL_RAP_ACTION_RCV_NOTIFY_MLEN           1
#define HM_MHL_RAP_ACTION_RCV_NOTIFY_ACTION         \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_MHL_RAP_ACK_RCV_NOTIFY has a 1 byte error code. */
#define HM_MHL_RAP_ACK_RCV_NOTIFY_MLEN              1
#define HM_MHL_RAP_ACK_RCV_NOTIFY_ERROR             \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

/* HM_MHL_SPAD_DATA_CHG_NOTIFY is 2+DATALEN bytes (datalength 0-64 bytes).
 * A 0-length change makes no sense, but it would be a legal message. */
#define HM_MHL_SPAD_DATA_CHG_NOTIFY_MLEN_MIN        2
#define HM_MHL_SPAD_DATA_CHG_NOTIFY_DATALEN_MAX     64
#define HM_MHL_SPAD_DATA_CHG_NOTIFY_MLEN_MAX        \
		(HM_MHL_SPAD_DATA_CHG_NOTIFY_MLEN_MIN + \
		 HM_MHL_SPAD_DATA_CHG_NOTIFY_DATALEN_MAX)

#define HM_MHL_SPAD_DATA_CHG_NOTIFY_OFFSET          \
		FIELD_DEFINE(8, 0, 0)  /* b7:0 of word 0 */
#define HM_MHL_SPAD_DATA_CHG_NOTIFY_RSVD            \
		FIELD_DEFINE(8, 8, 0)  /* b15:8 of word 0 */
#define HM_MHL_SPAD_DATA_CHG_NOTIFY_DATA_0    \
		FIELD_DEFINE(8, 16, 0) /* b23:16 of word 0 */

/* HM_MHL_VBUS_POWER_REQUEST_NOTIFY has a 1 byte PWR_REQ field. */
#define HM_MHL_VBUS_POWER_REQUEST_NOTIFY_MLEN       1

#define HM_MHL_VBUS_POWER_REQUEST_NOTIFY_PWR_REQ    \
		FIELD_DEFINE(8, 0, 0) /* b7:0 of word 0 */

#define HM_MHL_DCAP_CHG_NOTIFY_MLEN 0
/******************************************************************************
 * AP Non Volatile storage file offsets and sizes
 ******************************************************************************/

/* WiHD Configuration: 0x000-0xbff (3K available) */
#define HM_NONVOLATILE_WIHD_CONFIG_BASE             0x0
#define HM_NONVOLATILE_WIHD_CONFIG_MAX_SIZE         0xc00

/* WiHD Customer:      0xc00-0xfff (1K available) */
#define HM_NONVOLATILE_WIHD_CUSTOMER_BASE           0xc00
#define HM_NONVOLATILE_WIHD_CUSTOMER_MAX_SIZE       0x400

/* WiHD Security:      0x1000-0x17ff (2K available) */
#define HM_NONVOLATILE_WIHD_SECURITY_BASE           0x1000
#define HM_NONVOLATILE_WIHD_SECURITY_MAX_SIZE       0x1000

#define HM_NONVOLATILE_WIHD_FILE_SIZE_MAX           0x2000

/******************************************************************************
 * Top Level HostMsg calls
 ******************************************************************************/

/*
 * Set up for host messaging:
 *      create and allocate all resources
 *      initialize the AP<->BB SPI interrupts and mailboxes
 *      start HostMsg receive task
 */
int HostMsgInit(void);

/*
 * Stop host message:
 *      stop HostMsg receive task
 *      disable the AP<->BB SPI interrupts and mailboxes
 *      free all easily released allocated resources
 */
int HostMsgTerm(void);

/* Calls to send all externally visible message types. */

/* Notification messages only - no response, generates new sequence number. */
enum sii_os_status HostMsgSendNotification(uint16_t msgCode,
					uint16_t msgLength, void *pMsgData);

/*
 * Send command with no wait for response (expects responses to use state
 * machine queue to pass events and data to state machine).
 */
enum sii_os_status HostMsgSendCommandAsync(uint16_t msgCode,
					uint16_t msgLength, void *pMsgData);

/*
 * Send command with wait of up to maxTimeout mS for response.
 * Returns data length to *pRetLength, and pointer to malloc'd data buffer to
 * *ppRetData.  Caller is responsible for freeing buffer after use if
 * *pRetLength is != 0.
 */
enum sii_os_status HostMsgSendCommand(uint16_t msgCode, uint16_t msgLength,
				void *pMsgData, uint32_t maxTimeout,
				uint16_t *pRetLength, void **ppRetData);

/*
 * Host Messaging event handlers
 * Functions are declared in host_msg.h.
 * 3 events currently defined: Packet Ready, ACK, and NAK.
 */
void HostMsgRxPacketHandler(void);
void HostMsgAckHandler(void);
void HostMsgNakHandler(void);

#if 1
/* test functions */
void SendHMConnectMsg(void);
void SendHMDisconnectMsg(void);
void SendHMTestMsg(uint16_t dataLength);
int CheckMboxRegs(uint8_t *pSource, uint8_t *pRead, uint8_t regCount);
void DoMailboxTest(uint8_t data);
#endif

#endif /* !__HOST_MSG_H__ */

