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

#ifndef _RCP_INPUTDEV_H
#define _RCP_INPUTDEV_H

struct rcp_keymap {
	unsigned multicode:1;
	unsigned press_and_hold_key:1;
	unsigned reserved:6;
	uint16_t map[2];
	uint8_t rcp_support;
};

#define	DEV_LD_DISPLAY		(0x01 << 0)
#define	DEV_LD_VIDEO		(0x01 << 1)
#define	DEV_LD_AUDIO		(0x01 << 2)
#define	DEV_LD_MEDIA		(0x01 << 3)
#define	DEV_LD_TUNER		(0x01 << 4)
#define	DEV_LD_RECORD		(0x01 << 5)
#define	DEV_LD_SPEAKER		(0x01 << 6)
#define	DEV_LD_GUI		(0x01 << 7)

#define	LOGICAL_DEVICE_MAP	\
	(DEV_LD_AUDIO | DEV_LD_VIDEO | DEV_LD_MEDIA | DEV_LD_GUI)

#define	NUM_RCP_KEY_CODES	0x80	/* inclusive */

#define RCP_KEY_RELEASED_MASK	0x80
#define RCP_KEY_ID_MASK		0x7F

/* No error (Not allowed in RCPE messages) */
#define	RCPE_NO_ERROR			0x00
/* Unsupported/unrecognized keycode */
#define	RCPE_INEFFECTIVE_KEYCODE	0x01
/* Responder busy - initiator may retry message */
#define	RCPE_BUSY			0x02

extern struct rcp_keymap rcp_support_table[NUM_RCP_KEY_CODES];

int generate_rcp_input_event(uint8_t rcp_keycode);
int init_rcp_input_dev(void);
void destroy_rcp_input_dev(void);
void rcp_input_dev_one_time_init(void);

#endif /* !_RCP_INPUTDEV_H */

