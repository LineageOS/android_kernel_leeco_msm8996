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

#ifndef _DIAG_SM_H
#define _DIAG_SM_H

#include "sii6400.h"

/* These values are in msec */
#define MAX_WAIT_FOR_DIAG_CMD_RESPONSE		30000
#define WAIT_TIME_FOR_DIAG_CMD_QUEUE		2000
#define WAIT_TIME_FOR_DIAG_CMD_QUEUE_FAILURE	1000
#define WAIT_TIME_FOR_DIAG_CMD_BUSY		50

#define DIAG_CMD_QUEUE_SIZE			10
#define DIAG_CMD_OUTPUT_QUEUE_SIZE		20

#define DEBUG_OUTPUT_MAGIC_NUMBER		0x0112358d

struct debug_output {
	uint32_t magic_number;
	uint32_t time_stamp;
	uint8_t debug_msg_data[];
};

struct sii6400_diag_sm_info {
	bool diag_cmd_can_be_sent;
	struct sii6400_diag_cmd latest_async_queued;
	struct sii6400_diag_cmd latest_sync_queued;
	struct sii6400_diag_cmd latest_sent;
	struct sii6400_diag_cmd_output cur_being_rcvd;
};

struct diag_cmd_work {
	struct work_struct diagnostic_cmd_work;
};

extern struct SiiOsQueue *get_diag_cmd_resp_q;
extern struct SiiOsQueue *send_diag_cmd_resp_q;
extern struct SiiOsQueue *get_diag_cmd_sync_resp_q;
extern struct SiiOsQueue *send_diag_cmd_sync_resp_q;
extern struct SiiOsQueue *get_diag_cmd_output_resp_q;
extern struct SiiOsQueue *set_debug_output_path_resp_q;

enum sii_os_status get_diag_command(struct SiiOsQueue *response_queue);
enum sii_os_status send_diag_command(struct SiiOsQueue *response_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_diag_cmd_sync(struct SiiOsQueue *response_queue);
enum sii_os_status send_diag_cmd_sync(struct SiiOsQueue *response_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status get_diag_cmd_output(struct SiiOsQueue *response_queue);
enum sii_os_status set_debug_output_path(struct SiiOsQueue *response_queue,
				void *req_data, uint32_t req_data_size);
enum sii_os_status report_diag_cmd_done(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_diag_cmd_output(void *req_data,
					uint32_t req_data_size);
enum sii_os_status report_debug_output(void *req_data,
					uint32_t req_data_size);

enum sii_os_status sii6400_diag_init(struct sii6400_device_info *devinfo);
bool sii6400_diag_sm_is_enabled(void);
void sii6400_diag_sm_enable(void);
void sii6400_diag_sm_disable(void);
int sii6400_diag_sm_init(void);
void sii6400_diag_sm_exit(void);

#endif /* !_DIAG_SM_H */

