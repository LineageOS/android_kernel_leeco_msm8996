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

#ifndef _WIHD_SM_H
#define _WIHD_SM_H

#include "sii6400.h"

/* These values are in msec */
#define MAX_WAIT_FOR_SCAN_SEMAPHORE	10000
#define MAX_WAIT_FOR_SCAN_RESPONSE	10000

#define MAX_WIHD_KEY_REPORT_DATA_STRING_SIZE	20

#define CEC_MESSAGE_HEADER		0x00
#define CEC_REMOTE_CONTROL_PRESSED	0x44
#define CEC_REMOTE_CONTROL_RELEASED	0x45

struct sii6400_wihd_sm_info {
	bool wvan_scan_enabled;
	bool disable_scanning_pending;
	bool restart_scanning_pending;
	unsigned long int wvan_scan_duration;
	unsigned int wvan_scan_interval;
	struct SiiOsSemaphore *scan_sem;

	bool associate_pending;
	struct sii6400_wvan_info next_wvan_info;
	struct sii6400_wvan_info wvan_info;

	struct sii6400_remote_device remote_device;

	uint8_t rc_sent;
	uint8_t rc_received;
};

extern struct SiiOsQueue *get_wihd_chip_version_resp_q;
extern struct SiiOsQueue *get_wihd_firmware_version_resp_q;
extern struct SiiOsQueue *get_wihd_id_impedance_resp_q;
extern struct SiiOsQueue *set_output_mode_resp_queue;
extern struct SiiOsQueue *get_output_mode_resp_queue;
extern struct SiiOsQueue *get_remote_mac_addr_resp_q;
extern struct SiiOsQueue *get_remote_category_resp_q;
extern struct SiiOsQueue *get_remote_manufacturer_resp_q;
extern struct SiiOsQueue *get_remote_monitor_name_resp_q;
extern struct SiiOsQueue *get_remote_name_resp_q;
extern struct SiiOsQueue *get_remote_type_resp_q;
extern struct SiiOsQueue *get_signal_strength_resp_q;
extern struct SiiOsQueue *get_mac_addr_resp_q;
extern struct SiiOsQueue *set_name_resp_q;
extern struct SiiOsQueue *get_name_resp_q;
extern struct SiiOsQueue *set_wvan_scan_duration_resp_q;
extern struct SiiOsQueue *get_wvan_scan_duration_resp_q;
extern struct SiiOsQueue *set_wvan_scan_interval_resp_q;
extern struct SiiOsQueue *get_wvan_scan_interval_resp_q;
extern struct SiiOsQueue *scan_for_wvans_resp_q;
extern struct SiiOsQueue *get_wvan_scan_status_resp_q;
extern struct SiiOsQueue *join_wvan_resp_q;
extern struct SiiOsQueue *leave_wvan_resp_q;
extern struct SiiOsQueue *get_wvan_info_resp_q;
extern struct SiiOsQueue *get_wvan_connection_status_resp_q;
extern struct SiiOsQueue *get_device_list_resp_q;
extern struct SiiOsQueue *wihd_connect_resp_q;
extern struct SiiOsQueue *wihd_disconnect_resp_q;
extern struct SiiOsQueue *cec_send_message_resp_q;
extern struct SiiOsQueue *get_wihd_rc_rcvd_ctrlcode_resp_q;
extern struct SiiOsQueue *get_wihd_rc_sent_ctrlcode_resp_q;
extern struct SiiOsQueue *send_wihd_rc_ctrlcode_resp_q;
extern struct SiiOsQueue *set_hdcp_policy_resp_q;
extern struct SiiOsQueue *get_hdcp_policy_resp_q;
extern struct SiiOsQueue *set_hdcp_stream_type_resp_q;
extern struct SiiOsQueue *get_hdcp_stream_type_resp_q;
extern struct SiiOsQueue *set_remote_fw_update_resp_q;
extern struct SiiOsQueue *get_vendor_msg_recv_filter_resp_q;
extern struct SiiOsQueue *set_vendor_msg_recv_filter_resp_q;
extern struct SiiOsQueue *set_vendor_msg_vendor_id_resp_q;
extern struct SiiOsQueue *set_vendor_msg_mac_addr_resp_q;
extern struct SiiOsQueue *send_vendor_msg_resp_q;

bool wihd_is_idle(enum sii6400_wihd_state state);
bool wihd_is_scanning(enum sii6400_wihd_state state);
bool wihd_is_associating(enum sii6400_wihd_state state);
bool wihd_is_associated(enum sii6400_wihd_state state);
bool wihd_is_connecting(enum sii6400_wihd_state state);
bool wihd_is_connected(enum sii6400_wihd_state state);

enum sii_os_status get_wihd_chip_version(struct SiiOsQueue *resp_queue);
enum sii_os_status get_wihd_firmware_version(struct SiiOsQueue *resp_queue);
enum sii_os_status get_wihd_id_impedance(struct SiiOsQueue *resp_queue);
enum sii_os_status set_output_mode(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_output_mode(struct SiiOsQueue *resp_queue);
enum sii_os_status get_remote_mac_addr(struct SiiOsQueue *resp_queue);
enum sii_os_status get_remote_category(struct SiiOsQueue *resp_queue);
enum sii_os_status get_remote_manufacturer(struct SiiOsQueue *resp_queue);
enum sii_os_status get_remote_monitor_name(struct SiiOsQueue *resp_queue);
enum sii_os_status get_remote_name(struct SiiOsQueue *resp_queue);
enum sii_os_status get_remote_type(struct SiiOsQueue *resp_queue);
enum sii_os_status get_signal_strength(struct SiiOsQueue *resp_queue);
enum sii_os_status get_mac_addr(struct SiiOsQueue *resp_queue);
enum sii_os_status set_name(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_name(struct SiiOsQueue *resp_queue);
enum sii_os_status set_wvan_scan_duration(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_wvan_scan_duration(struct SiiOsQueue *resp_queue);
enum sii_os_status set_wvan_scan_interval(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_wvan_scan_interval(struct SiiOsQueue *resp_queue);
enum sii_os_status scan_for_wvans(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_wvan_scan_status(struct SiiOsQueue *resp_queue);
enum sii_os_status join_wvan(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status leave_wvan(struct SiiOsQueue *resp_queue);
enum sii_os_status get_wvan_info(struct SiiOsQueue *resp_queue);
enum sii_os_status get_connection_status(struct SiiOsQueue *resp_queue);
enum sii_os_status get_device_list(struct SiiOsQueue *resp_queue);
enum sii_os_status wihd_connect(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status wihd_disconnect(struct SiiOsQueue *resp_queue);
enum sii_os_status cec_send_message(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_wihd_rc_rcvd_ctrlcode(struct SiiOsQueue *resp_queue);
enum sii_os_status get_wihd_rc_sent_ctrlcode(struct SiiOsQueue *resp_queue);
enum sii_os_status send_wihd_rc_ctrlcode(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status set_hdcp_policy(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_hdcp_policy(struct SiiOsQueue *resp_queue);
enum sii_os_status set_hdcp_stream_type(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_hdcp_stream_type(struct SiiOsQueue *resp_queue);
enum sii_os_status report_scan_results(void *req_data, uint32_t req_data_size);
enum sii_os_status report_associated(void *req_data, uint32_t req_data_size);
enum sii_os_status report_disassociated(void *req_data, uint32_t req_data_size);
enum sii_os_status report_connected(void *req_data, uint32_t req_data_size);
enum sii_os_status report_disconnected(void *req_data, uint32_t req_data_size);
enum sii_os_status report_cec_msg_received(void *req_data,
						uint32_t req_data_size);
enum sii_os_status report_vendor_msg_received(void *req_data,
						uint32_t req_data_size);
enum sii_os_status set_remote_fw_update(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_vendor_msg_recv_filter(struct SiiOsQueue *resp_queue);
enum sii_os_status set_vendor_msg_recv_filter(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status set_vendor_msg_vendor_id(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status set_vendor_msg_mac_addr(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status send_vendor_msg(struct SiiOsQueue *resp_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status sii6400_wihd_init(struct sii6400_device_info *devinfo);
bool sii6400_wihd_sm_is_enabled(void);
void sii6400_wihd_sm_enable(void);
void sii6400_wihd_sm_disable(void);
int sii6400_wihd_sm_init(void);
void sii6400_wihd_sm_exit(void);
enum sii_os_status report_dev_list_changed(void *req_data,
						uint32_t req_data_size);

#endif /* !_WIHD_SM_H */

