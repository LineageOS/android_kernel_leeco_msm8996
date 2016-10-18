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
#ifndef _MHL_SM_H
#define _MHL_SM_H

#include "sii6400.h"

/* These values are in msec */
#define DELAY_BETWEEN_RCPE_AND_RCPK	20
#define DELAY_BETWEEN_UCPE_AND_UCPK	20

#define MHL_LOCAL_DEVCAP_OFFSET_DEFAULT		0
#define MHL_REMOTE_DEVCAP_OFFSET_DEFAULT	0

#define MAX_MHL_CHAR_REPORT_DATA_STRING_SIZE	20
#define MAX_MHL_KEY_REPORT_DATA_STRING_SIZE	20
#define MAX_MHL_ACTION_REPORT_DATA_STRING_SIZE	20
#define MAX_MHL_SPAD_REPORT_DATA_STRING_SIZE	400

/* No error (Not allowed in UCPE messages) */
#define	UCPE_NO_ERROR			0x00
/* Unsupported/unrecognized charcode */
#define	UCPE_INEFFECTIVE_KEYCODE	0x01

/* No error (Not allowed in RAPK messages) */
#define	RAPK_NO_ERROR			0x00
/* Unrecognized action code */
#define	RAPK_UNRECOGNIZED_ACTION_CODE	0x01
/* Unsupported action code */
#define	RAPK_UNSUPPORTED_ACTION_CODE	0x02
/* Responder busy */
#define	RAPK_BUSY			0x03

/* RAP Action codes */
#define RAP_POLL	0x00
#define RAP_CONTENT_ON	0x10
#define RAP_CONTENT_OFF	0x11

struct sii6400_mhl_sm_info {
	uint8_t local_devcap[MAX_MHL_DEVCAP_SIZE];
	uint8_t local_devcap_offset;
	uint8_t remote_devcap_offset;
	uint8_t ucp_sent;
	uint8_t ucp_sent_status;
	uint8_t ucp_received;
	uint8_t rcp_sent;
	uint8_t rcp_sent_status;
	uint8_t rcp_received;
	uint8_t rap_sent;
	uint8_t rap_sent_status;
	uint8_t rap_received;
	uint8_t swing_set;
};

extern struct SiiOsQueue *get_mhl_chip_version_resp_q;
extern struct SiiOsQueue *get_mhl_firmware_version_resp_q;
extern struct SiiOsQueue *get_mhl_connection_state_resp_q;
extern struct SiiOsQueue *get_mhl_id_impedance_resp_q;
extern struct SiiOsQueue *set_mhl_clock_swing_resp_q;
extern struct SiiOsQueue *get_mhl_clock_swing_resp_q;
extern struct SiiOsQueue *set_mhl_cbus_voltage_resp_q;
extern struct SiiOsQueue *set_mhl_local_devcap_resp_q;
extern struct SiiOsQueue *get_mhl_local_devcap_resp_q;
extern struct SiiOsQueue *set_mhl_local_devcap_offset_resp_q;
extern struct SiiOsQueue *get_mhl_local_devcap_offset_resp_q;
extern struct SiiOsQueue *get_mhl_remote_devcap_resp_q;
extern struct SiiOsQueue *set_mhl_remote_devcap_offset_resp_q;
extern struct SiiOsQueue *get_mhl_remote_devcap_offset_resp_q;
extern struct SiiOsQueue *send_mhl_ucp_charcode_resp_q;
extern struct SiiOsQueue *get_mhl_ucp_rcvd_charcode_resp_q;
extern struct SiiOsQueue *get_mhl_ucp_sent_charcode_resp_q;
extern struct SiiOsQueue *send_mhl_ucp_ack_resp_q;
extern struct SiiOsQueue *get_mhl_ucp_ack_resp_q;
extern struct SiiOsQueue *send_mhl_rcp_keycode_resp_q;
extern struct SiiOsQueue *get_mhl_rcp_rcvd_keycode_resp_q;
extern struct SiiOsQueue *get_mhl_rcp_sent_keycode_resp_q;
extern struct SiiOsQueue *send_mhl_rcp_ack_resp_q;
extern struct SiiOsQueue *get_mhl_rcp_ack_resp_q;
extern struct SiiOsQueue *send_mhl_rap_actioncode_resp_q;
extern struct SiiOsQueue *get_mhl_rap_rcvd_actioncode_resp_q;
extern struct SiiOsQueue *get_mhl_rap_sent_actioncode_resp_q;
extern struct SiiOsQueue *send_mhl_rap_ack_resp_q;
extern struct SiiOsQueue *get_mhl_rap_ack_resp_q;

bool mhl_is_connected(enum sii6400_mhl_state state);

enum sii_os_status get_mhl_chip_version(struct SiiOsQueue *resp_q);
enum sii_os_status get_mhl_firmware_version(struct SiiOsQueue *resp_q);
enum sii_os_status get_mhl_dev_type(struct SiiOsQueue *resp_q);
enum sii_os_status get_mhl_connection_state(struct SiiOsQueue *resp_q);
enum sii_os_status get_mhl_id_impedance(struct SiiOsQueue *resp_q);
enum sii_os_status set_mhl_clock_swing(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_mhl_clock_swing(struct SiiOsQueue *resp_q);
enum sii_os_status set_mhl_cbus_voltage(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status set_mhl_local_devcap(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_mhl_local_devcap(struct SiiOsQueue *resp_q);
enum sii_os_status set_mhl_local_devcap_offset(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_mhl_local_devcap_offset(struct SiiOsQueue *resp_q);
enum sii_os_status get_mhl_remote_devcap(struct SiiOsQueue *resp_q);
enum sii_os_status set_mhl_remote_devcap_offset(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_mhl_remote_devcap_offset(struct SiiOsQueue *resp_q);
enum sii_os_status send_mhl_ucp_charcode(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_mhl_ucp_rcvd_charcode(struct SiiOsQueue *resp_q);
enum sii_os_status get_mhl_ucp_sent_charcode(struct SiiOsQueue *resp_q);
enum sii_os_status send_mhl_ucp_ack(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_mhl_ucp_ack(struct SiiOsQueue *resp_q);
enum sii_os_status send_mhl_rcp_keycode(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_mhl_rcp_rcvd_keycode(struct SiiOsQueue *resp_q);
enum sii_os_status get_mhl_rcp_sent_keycode(struct SiiOsQueue *resp_q);
enum sii_os_status send_mhl_rcp_ack(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_mhl_rcp_ack(struct SiiOsQueue *resp_q);
enum sii_os_status send_mhl_rap_actioncode(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_mhl_rap_rcvd_actioncode(struct SiiOsQueue *resp_q);
enum sii_os_status get_mhl_rap_sent_actioncode(struct SiiOsQueue *resp_q);
enum sii_os_status send_mhl_rap_ack(struct SiiOsQueue *resp_q,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_mhl_rap_ack(struct SiiOsQueue *resp_q);
enum sii_os_status report_mhl_connected(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_mhl_disconnected(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_mhl_spad_msg_rcv(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_mhl_ucp_char_rcv(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_mhl_ucp_ack_rcv(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_mhl_rcp_key_rcv(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_mhl_rcp_ack_rcv(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_mhl_rap_action_rcv(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_mhl_rap_ack_rcv(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_mhl_vbus_power_request_rcv(void *req_data,
						uint32_t req_data_size);
enum sii_os_status report_dcap_chg(void *req_data, uint32_t req_data_size);
enum sii_os_status sii6400_mhl_init(struct sii6400_device_info *devinfo);
bool sii6400_mhl_sm_is_enabled(void);
void sii6400_mhl_sm_enable(void);
void sii6400_mhl_sm_disable(void);
int sii6400_mhl_sm_init(void);
void sii6400_mhl_sm_exit(void);

#endif /* !_MHL_SM_H */

