/*
 * linux/drivers/power/mhl_uevent.h
 *
 * Copyright 2004 LeTV <www.letv.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file COPYING in the main directory of this
 * archive for more details.
 */
#ifndef _POWER_MHL_UEVENT_H
#define _POWER_MHL_UEVENT_H

int mhl_kobj_init(void);
int mhl_kobj_exit(void);
bool mhl_state_uevent(u16 mhl_adc, u8 mhl_disconnected);

#endif /* _POWER_MHL_UEVENT_H */
