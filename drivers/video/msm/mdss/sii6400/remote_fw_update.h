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

#ifndef _REMOTE_FW_UPDATE_H
#define _REMOTE_FW_UPDATE_H

#include "osal.h"

enum sii6400_remote_fw_update_state {
	REMOTE_FW_UPDATE_STATE_OFF = 0,
	REMOTE_FW_UPDATE_STATE_ON,
};

int sii6400_remote_fw_update_init(void);
void sii6400_remote_fw_update_exit(void);
enum sii_os_status sii6400_remote_fw_update_start(void);
enum sii_os_status sii6400_remote_fw_update_abort(void);

#endif /* !_REMOTE_FW_UPDATE_H */

