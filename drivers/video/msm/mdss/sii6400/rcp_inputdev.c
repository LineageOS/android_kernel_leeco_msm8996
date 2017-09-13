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

#include <linux/input.h>
#include <linux/device.h>

#include "osal.h"
#include "sii6400.h"
#include "rcp_inputdev.h"

enum rcp_state_e {
	PH0_IDLE = 0,
	PH3_PRESS_AND_HOLD_KEY,
	PH8_HOLD_MODE,

	NUM_RCP_STATES
};

#if 0
static char *state_strings[NUM_RCP_STATES] = {
	"idle",
	"press_and_hold_key",
	"hold_mode"
};
#endif

enum rcp_event_e {
	NORMAL_KEY_PRESS,
	NORMAL_KEY_PRESS_SAME,
	NORMAL_KEY_RELEASE,
	NORMAL_KEY_RELEASE_SAME,
	PRESS_AND_HOLD_KEY_PRESS,
	PRESS_AND_HOLD_KEY_PRESS_SAME,
	PRESS_AND_HOLD_KEY_RELEASE,
	PRESS_AND_HOLD_KEY_RELEASE_SAME,
	RCP_T_HOLD_MAINTAIN_EXPIRED,
	RCP_T_PRESS_MODE_EXPIRED,

	NUM_RCP_EVENTS
};

#if 0
static char *event_strings[NUM_RCP_EVENTS] = {
	"normal_key_press",
	"normal_key_press_same",
	"normal_key_release",
	"normal_key_release_same",
	"press_and_hold_key_press",
	"press_and_hold_key_press_same",
	"press_and_hold_key_release",
	"press_and_hold_key_release_same",
	"rcp_T_hold_maintain_expired",
	"rcp_T_press_mode_expired",
};

static enum rcp_state_e current_rcp_state = PH0_IDLE;

static uint8_t rcp_previous_key;
static uint8_t rcp_current_key;
#endif

struct rcp_keymap rcp_support_table[NUM_RCP_KEY_CODES] = {
	/* 0x00 = Select */
	{0, 0, 0, {KEY_SELECT,       0}, (DEV_LD_GUI)},
	/* 0x01 = Up */
	{0, 1, 0, {KEY_UP,           0}, (DEV_LD_GUI)},
	/* 0x02 = Down */
	{0, 1, 0, {KEY_DOWN,         0}, (DEV_LD_GUI)},
	/* 0x03 = Left */
	{0, 1, 0, {KEY_LEFT,         0}, (DEV_LD_GUI)},
	/* 0x04 = Right */
	{0, 1, 0, {KEY_RIGHT,        0}, (DEV_LD_GUI)},
	/* 0x05 = right-up */
	{1, 1, 0, {KEY_RIGHT,   KEY_UP}, (DEV_LD_GUI)},
	/* 0x06 = right-down */
	{1, 1, 0, {KEY_LEFT,  KEY_DOWN}, (DEV_LD_GUI)},
	/* 0x07 = left-up */
	{1, 1, 0, {KEY_LEFT,    KEY_UP}, (DEV_LD_GUI)},
	/* 0x08 = left-down */
	{1, 1, 0, {KEY_LEFT,  KEY_DOWN}, (DEV_LD_GUI)},
	/* 0x09 = RootMenu */
	{0, 0, 0, {KEY_MENU,         0}, (DEV_LD_GUI)},
	/* 0x0A Reserved */
	{0, 0, 0, {KEY_UNKNOWN,      0}, 0},
	/* 0x0B Reserved */
	{0, 0, 0, {KEY_UNKNOWN,      0}, 0},
	/* 0x0C Reserved */
	{0, 0, 0, {KEY_BOOKMARKS,    0}, 0},
	/* 0x0D = Select */
	{0, 0, 0, {KEY_EXIT,         0}, (DEV_LD_GUI)},
	/* 0x0E Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x0F Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},

	/* 0x10 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x11 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x12 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x13 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x14 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x15 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x16 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x17 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x18 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x19 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x1A Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x1B Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x1C Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x1D Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x1E Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x1F Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},

	/* 0x20 = Numeric key */
	{0, 0, 0, {KEY_NUMERIC_0,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x21 = Numeric key */
	{0, 0, 0, {KEY_NUMERIC_0,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x22 = Numeric key */
	{0, 0, 0, {KEY_NUMERIC_2,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x23 = Numeric key */
	{0, 0, 0, {KEY_NUMERIC_3,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x24 = Numeric key */
	{0, 0, 0, {KEY_NUMERIC_4,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x25 = Numeric key */
	{0, 0, 0, {KEY_NUMERIC_5,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x26 = Numeric key */
	{0, 0, 0, {KEY_NUMERIC_6,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x27 = Numeric key */
	{0, 0, 0, {KEY_NUMERIC_7,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x28 = Numeric key */
	{0, 0, 0, {KEY_NUMERIC_8,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x29 = Numeric key */
	{0, 0, 0, {KEY_NUMERIC_9,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x2A = Dot */
	{0, 0, 0, {KEY_DOT,          0}, 0},
	/* 0x2B = Enter key */
	{0, 0, 0, {KEY_ENTER,        0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 0x2C = Clear key */
	{0, 0, 0, {KEY_CLEAR,        0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA | DEV_LD_TUNER)},
	/* 2D Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 2E Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 2F Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},

	/* 0x30 = Channel Up */
	{0, 1, 0, {KEY_CHANNELUP,    0}, (DEV_LD_TUNER)},
	/* 0x31 = Channel Dn */
	{0, 1, 0, {KEY_CHANNELDOWN,  0}, (DEV_LD_TUNER)},
	/* 0x32 = Previous Channel */
	{0, 0, 0, {KEY_UNKNOWN,      0}, (DEV_LD_TUNER)},
	/* 0x33 = Sound Select */
	{0, 0, 0, {KEY_SOUND,        0}, (DEV_LD_AUDIO)},
	/* 0x34 = Input Select */
	{0, 0, 0, {KEY_UNKNOWN,      0}, 0},
	/* 0x35 = Show Information */
	{0, 0, 0, {KEY_PROGRAM,      0}, 0},
	/* 0x36 = Help */
	{0, 0, 0, {KEY_UNKNOWN,      0}, 0},
	/* 0x37 = Page Up */
	{0, 1, 0, {KEY_PAGEUP,       0}, 0},
	/* 0x38 = Page Down */
	{0, 1, 0, {KEY_PAGEDOWN,     0}, 0},
	/* 0x39 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x3A Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x3B Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x3C Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x3D Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x3E Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x3F Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},

	/* 0x40 Undefined */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x41 = Volume Up */
	{0, 1, 0, {KEY_VOLUMEUP,     0}, (DEV_LD_SPEAKER)},
	/* 0x42 = Volume Down */
	{0, 1, 0, {KEY_VOLUMEDOWN,   0}, (DEV_LD_SPEAKER)},
	/* 0x43 = Mute */
	{0, 0, 0, {KEY_MUTE,         0}, (DEV_LD_SPEAKER)},
	/* 0x44 = Play */
	{0, 0, 0, {KEY_PLAY,         0}, (DEV_LD_VIDEO | DEV_LD_AUDIO)},
	/* 0x45 = Stop */
	{0, 0, 0, {KEY_STOP,         0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_RECORD)},
	/* 0x46 = Pause */
	{0, 0, 0, {KEY_PLAYPAUSE,    0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_RECORD)},
	/* 0x47 = Record */
	{0, 0, 0, {KEY_RECORD,       0}, (DEV_LD_RECORD)},
	/* 0x48 = Rewind */
	{0, 1, 0, {KEY_REWIND,       0}, (DEV_LD_VIDEO | DEV_LD_AUDIO)},
	/* 0x49 = Fast Forward */
	{0, 1, 0, {KEY_FASTFORWARD,  0}, (DEV_LD_VIDEO | DEV_LD_AUDIO)},
	/* 0x4A = Eject */
	{0, 0, 0, {KEY_EJECTCD,      0}, (DEV_LD_MEDIA)},
	/* 0x4B = Forward */
	{0, 1, 0, {KEY_NEXTSONG,     0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA)},
	/* 0x4C = Backward */
	{0, 1, 0, {KEY_PREVIOUSSONG, 0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_MEDIA)},
	/* 0x4D Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x4E Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x4F Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},

	/* 0x50 = Angle */
	{0, 0, 0, {KEY_UNKNOWN,      0}, 0},
	/* 0x51 = Subpicture */
	{0, 0, 0, {KEY_UNKNOWN,      0}, 0},
	/* 0x52 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x53 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x54 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x55 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x56 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x57 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x58 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x59 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x5A Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x5B Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x5C Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x5D Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x5E Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x5F Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},

	/* 0x60 = Play */
	{0, 0, 0, {KEY_PLAYPAUSE,    0}, (DEV_LD_VIDEO | DEV_LD_AUDIO)},
	/* 0x61 = Pause the Play */
	{0, 0, 0, {KEY_PLAYPAUSE,    0}, (DEV_LD_VIDEO | DEV_LD_AUDIO)},
	/* 0x62 = Record */
	{0, 0, 0, {KEY_RECORD,       0}, (DEV_LD_RECORD)},
	/* 0x63 = Pause the Record */
	{0, 0, 0, {KEY_PAUSE,        0}, (DEV_LD_RECORD)},
	/* 0x64 = Stop */
	{0, 0, 0, {KEY_STOP,         0},
		(DEV_LD_VIDEO | DEV_LD_AUDIO | DEV_LD_RECORD)},
	/* 0x65 = Mute */
	{0, 0, 0, {KEY_MUTE,         0}, (DEV_LD_SPEAKER)},
	/* 0x66 = Restore Mute */
	{0, 0, 0, {KEY_MUTE,         0}, (DEV_LD_SPEAKER)},
	/* 0x67 Undefined */
	{0, 0, 0, {KEY_UNKNOWN,      0}, 0},
	/* 0x68 Undefined */
	{0, 0, 0, {KEY_UNKNOWN,      0}, 0},
	/* 0x69 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x6A Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x6B Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x6C Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x6D Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x6E Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x6F Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},

	/* 0x70 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x71 Reserved */
	{0, 0, 0, {KEY_F1,           0}, 0},
	/* 0x72 Reserved */
	{0, 0, 0, {KEY_F2,           0}, 0},
	/* 0x73 Reserved */
	{0, 0, 0, {KEY_F3,           0}, 0},
	/* 0x74 Reserved */
	{0, 0, 0, {KEY_F4,           0}, 0},
	/* 0x75 Reserved */
	{0, 0, 0, {KEY_F5,           0}, 0},
	/* 0x76 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x77 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x78 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x79 Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x7A Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x7B Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x7C Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x7D Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
	/* 0x7E Undefined */
	{0, 0, 0, {KEY_VENDOR,       0}, 0},
	/* 0x7F Reserved */
	{0, 0, 0, {KEY_RESERVED,     0}, 0},
};

static u16 rcp_def_keymap[NUM_RCP_KEY_CODES]
#if 0 //def OLD_KEYMAP_TABLE //zxu1 20150228(
= {
	KEY_ENTER,
	KEY_UP,
	KEY_DOWN,
	KEY_LEFT,
	KEY_RIGHT,
	KEY_UNKNOWN,	/* right-up */
	KEY_UNKNOWN,	/* right-down */
	KEY_UNKNOWN,	/* left-up */
	KEY_UNKNOWN,	/* left-down */
	KEY_MENU,
	KEY_UNKNOWN,	/* setup */
	KEY_UNKNOWN,	/* contents */
	KEY_UNKNOWN,	/* favorite */
	KEY_BACK,
	KEY_RESERVED,	/* 0x0e */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x1F */
	KEY_0,
	KEY_1,
	KEY_2,
	KEY_3,
	KEY_4,
	KEY_5,
	KEY_6,
	KEY_7,
	KEY_8,
	KEY_9,
	KEY_DOT,
	KEY_ENTER,
	KEY_CLEAR,
	KEY_RESERVED,	/* 0x2D */
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x2F */
	KEY_UNKNOWN,	/* channel up */
	KEY_UNKNOWN,	/* channel down */
	KEY_UNKNOWN,	/* previous channel */
	KEY_UNKNOWN,	/* sound select */
	KEY_UNKNOWN,	/* input select */
	KEY_UNKNOWN,	/* show information */
	KEY_UNKNOWN,	/* help */
	KEY_UNKNOWN,	/* page up */
	KEY_UNKNOWN,	/* page down */
	KEY_RESERVED,	/* 0x39 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x3F */
	KEY_RESERVED,	/* 0x40 */
	KEY_UNKNOWN,	/* volume up */
	KEY_UNKNOWN,	/* volume down */
	KEY_UNKNOWN,	/* mute */
	KEY_PLAY,
	KEY_STOP,
	KEY_PLAYPAUSE,
	KEY_UNKNOWN,	/* record */
	KEY_REWIND,
	KEY_FASTFORWARD,
	KEY_UNKNOWN,	/* eject */
	KEY_NEXTSONG,
	KEY_PREVIOUSSONG,
	KEY_RESERVED,	/* 0x4D */
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x4F */
	KEY_UNKNOWN,	/* angle */
	KEY_UNKNOWN,	/* subtitle */
	KEY_RESERVED,	/* 0x52 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x5F */
	KEY_PLAY,
	KEY_PAUSE,
	KEY_UNKNOWN,	/* record_function */
	KEY_UNKNOWN,	/* pause_record_function */
	KEY_STOP,
	KEY_UNKNOWN,	/* mute_function */
	KEY_UNKNOWN,	/* restore_volume_function */
	KEY_UNKNOWN,	/* tune_function */
	KEY_UNKNOWN,	/* select_media_function */
	KEY_RESERVED,	/* 0x69 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x70 */
	KEY_UNKNOWN,	/* F1 */
	KEY_UNKNOWN,	/* F2 */
	KEY_UNKNOWN,	/* F3 */
	KEY_UNKNOWN,	/* F4 */
	KEY_UNKNOWN,	/* F5 */
	KEY_RESERVED,	/* 0x76 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x7D */
	KEY_VENDOR,
	KEY_RESERVED,	/* 0x7F */
}
#endif //)
;

static struct input_dev *rcp_input_dev;

/* Timer settings in msec */
#define T_PRESS_MODE		300
#define T_HOLD_MAINTAIN		2000

static struct SiiOsTimer *T_press_mode_timer;
static struct SiiOsTimer *T_hold_maintain_timer;

#if 0
static void T_press_mode_timer_function(void *data);
static void T_press_mode_work_function(struct work_struct *work);
static void T_hold_maintain_timer_function(void *data);
static void T_hold_maintain_work_function(struct work_struct *work);

static DECLARE_WORK(T_press_mode_work, T_press_mode_work_function);
static DECLARE_WORK(T_hold_maintain_work, T_hold_maintain_work_function);

static int rcp_trigger_key_action(uint8_t index, bool press_release);
static int handle_rcp_event(uint8_t current_key, uint8_t prev_key,
				enum rcp_event_e event);
static enum sii_os_status start_T_press_mode_timer(void);
static enum sii_os_status start_T_hold_maintain_timer(void);
static enum sii_os_status stop_T_press_mode_timer(void);
static enum sii_os_status stop_T_hold_maintain_timer(void);
#endif


#if 0
static int rcp_trigger_key_action(uint8_t index, bool press_release)
{
	int status = -EINVAL;

	/*dbg("");*/

	if (NULL != rcp_input_dev) {
		/*input_report_key(rcp_input_dev, index, press_release); zxu1 20150212*/
		/*input_report_key(rcp_input_dev, rcp_support_table[index].map[0], press_release); zxu1 20150228*/
		input_report_key(rcp_input_dev, rcp_def_keymap[index], press_release);
		input_sync(rcp_input_dev);
		status = 0;
	}
	return status;
}
#endif

#if 0
static int handle_rcp_event(uint8_t current_key, uint8_t prev_key,
				enum rcp_event_e event)
{
	int status = 0;
	uint8_t current_index = current_key & RCP_KEY_ID_MASK;
	uint8_t prev_index = prev_key & RCP_KEY_ID_MASK;

	dbg("received 0x%02x: %s(%d) in state: %s(%d)\n",
	    current_key,
	    event_strings[event], event,
	    state_strings[current_rcp_state], current_rcp_state);

	/* now process the event according to the current state */
	switch (current_rcp_state) {
	case PH0_IDLE:
		switch (event) {
		case NORMAL_KEY_PRESS:
		case NORMAL_KEY_PRESS_SAME:
			status = rcp_trigger_key_action(current_index, 1);
			/* no update for current_rcp_state */
			break;

		case NORMAL_KEY_RELEASE:
		case NORMAL_KEY_RELEASE_SAME:
			status = rcp_trigger_key_action(current_index, 0);
			/* no update for current_rcp_state */
			break;

		case PRESS_AND_HOLD_KEY_PRESS:
		case PRESS_AND_HOLD_KEY_PRESS_SAME:
			/*(void)start_T_press_mode_timer();
			current_rcp_state = PH3_PRESS_AND_HOLD_KEY;*/
			/*Disable long press checking and directly send the RCP key*/
			/*zxu1 20150304*/
			status = rcp_trigger_key_action(current_index, 1);
			break;

		case PRESS_AND_HOLD_KEY_RELEASE:
		case PRESS_AND_HOLD_KEY_RELEASE_SAME:
			err("unexpected %s(%d) in state: %s(%d)\n",
			    event_strings[event], event,
			    state_strings[current_rcp_state],
			    current_rcp_state);
			break;

		default:
			err("unexpected event: %d in state: %d\n",
			    event, current_rcp_state);
			/* no update for current_rcp_state */
			status = -EINVAL;
			break;
		}
		break;

	case PH3_PRESS_AND_HOLD_KEY:
		switch (event) {
		case NORMAL_KEY_PRESS:
		case NORMAL_KEY_PRESS_SAME:
			(void)stop_T_press_mode_timer();
			(void)rcp_trigger_key_action(prev_index, 0);
			/* OK to overwrite status */
			status = rcp_trigger_key_action(current_index, 1);
			current_rcp_state = PH0_IDLE;
			break;

		case NORMAL_KEY_RELEASE:
		case NORMAL_KEY_RELEASE_SAME:
			(void)stop_T_press_mode_timer();
			(void)rcp_trigger_key_action(prev_index, 0);
			(void)rcp_trigger_key_action(current_index, 1);
			status = rcp_trigger_key_action(current_index, 0);
			current_rcp_state = PH0_IDLE;
			break;

		case PRESS_AND_HOLD_KEY_PRESS:
			(void)start_T_press_mode_timer();
			status = rcp_trigger_key_action(prev_index, 1);
			/* no update for current_rcp_state */
			break;

		case PRESS_AND_HOLD_KEY_PRESS_SAME:
			(void)stop_T_press_mode_timer();
			(void)start_T_hold_maintain_timer();
			status = rcp_trigger_key_action(prev_index, 1);
			current_rcp_state = PH8_HOLD_MODE;
			break;

		case PRESS_AND_HOLD_KEY_RELEASE:
		case PRESS_AND_HOLD_KEY_RELEASE_SAME:
			(void)stop_T_press_mode_timer();
			status = rcp_trigger_key_action(prev_index, 0);
			current_rcp_state = PH0_IDLE;
			break;

		case RCP_T_PRESS_MODE_EXPIRED:
			(void)start_T_hold_maintain_timer();
			status = rcp_trigger_key_action(prev_index, 0);
			current_rcp_state = PH8_HOLD_MODE;
			break;

		default:
			err("unexpected event: %d in state: %d\n",
			    event, current_rcp_state);
			/* no update for current_rcp_state */
			status = -EINVAL;
			break;
		}
		break;

	case PH8_HOLD_MODE:
		switch (event) {
		case NORMAL_KEY_PRESS:
		case NORMAL_KEY_PRESS_SAME:
			(void)stop_T_hold_maintain_timer();
			(void)rcp_trigger_key_action(prev_index, 0);
			status = rcp_trigger_key_action(current_index, 1);
			current_rcp_state = PH0_IDLE;
			break;

		case NORMAL_KEY_RELEASE:
		case NORMAL_KEY_RELEASE_SAME:
			(void)stop_T_hold_maintain_timer();
			(void)rcp_trigger_key_action(prev_index, 0);
			(void)rcp_trigger_key_action(current_index, 1);
			status = rcp_trigger_key_action(current_index, 0);
			current_rcp_state = PH0_IDLE;
			break;

		case PRESS_AND_HOLD_KEY_PRESS:
			(void)stop_T_hold_maintain_timer();
			(void)start_T_press_mode_timer();
			status = rcp_trigger_key_action(prev_index, 1);
			current_rcp_state = PH3_PRESS_AND_HOLD_KEY;
			break;

		case PRESS_AND_HOLD_KEY_PRESS_SAME:
			(void)start_T_hold_maintain_timer();
			status = rcp_trigger_key_action(prev_index, 1);
			/* no update for current_rcp_state */
			break;

		case PRESS_AND_HOLD_KEY_RELEASE:
			(void)stop_T_hold_maintain_timer();
			(void)rcp_trigger_key_action(prev_index, 0);
			(void)rcp_trigger_key_action(current_index, 1);
			status = rcp_trigger_key_action(current_index, 0);
			current_rcp_state = PH0_IDLE;
			break;

		case PRESS_AND_HOLD_KEY_RELEASE_SAME:
			(void)stop_T_hold_maintain_timer();
			status = rcp_trigger_key_action(prev_index, 0);
			current_rcp_state = PH0_IDLE;
			break;

		case RCP_T_HOLD_MAINTAIN_EXPIRED:
			status = rcp_trigger_key_action(prev_index, 0);
			current_rcp_state = PH0_IDLE;
			break;

		default:
			err("unexpected event: %d in state: %d\n",
			    event, current_rcp_state);
			/* no update for current_rcp_state */
			status = -EINVAL;
		}
		break;

	default:
		err("irrational state value:%d\n", current_rcp_state);
		break;
	}
	return status;
}
#endif

#if 0
static void T_press_mode_timer_function(void *data)
{
	schedule_work(&T_hold_maintain_work);
}

static void T_press_mode_work_function(struct work_struct *work)
{
	(void)handle_rcp_event(rcp_current_key, rcp_previous_key,
				RCP_T_PRESS_MODE_EXPIRED);
}

static void T_hold_maintain_timer_function(void *data)
{
	schedule_work(&T_press_mode_work);
}

static void T_hold_maintain_work_function(struct work_struct *work)
{
	(void)handle_rcp_event(rcp_current_key, rcp_previous_key,
				RCP_T_HOLD_MAINTAIN_EXPIRED);
}

static enum sii_os_status start_T_press_mode_timer(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == T_press_mode_timer) {
		err("T_press_mode timer does not exist\n");
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto done;
	}

	retval = SiiOsTimerSchedule(T_press_mode_timer, T_PRESS_MODE);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("T_press_mode timer not started: %x\n", retval);
		goto done;
	}

done:
	return retval;
}

static enum sii_os_status start_T_hold_maintain_timer(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == T_hold_maintain_timer) {
		err("T_hold_maintain timer does not exist\n");
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto done;
	}

	retval = SiiOsTimerSchedule(T_hold_maintain_timer, T_HOLD_MAINTAIN);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("T_hold_maintain timer not started: %x\n", retval);
		goto done;
	}

done:
	return retval;
}

static enum sii_os_status stop_T_press_mode_timer(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == T_press_mode_timer) {
		err("T_press_mode timer does not exist\n");
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto done;
	}

	/* Delete the T_press_mode timer */
	retval = SiiOsTimerDelete(T_press_mode_timer);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not stop T_press_mode timer: %x\n", retval);
		goto done;
	}
	T_press_mode_timer = NULL;

	/* Recreate the T_press_mode timer */
	retval = SiiOsTimerCreate("T_press_mode_timer",
					T_press_mode_timer_function,
					NULL, false, 0, false,
					&T_press_mode_timer);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Unable to create T_press_mode timer: %x\n", retval);
		T_press_mode_timer = NULL;
		goto done;
	}

done:
	return retval;
}

static enum sii_os_status stop_T_hold_maintain_timer(void)
{
	enum sii_os_status retval = SII_OS_STATUS_SUCCESS;

	dbg("");

	if (NULL == T_hold_maintain_timer) {
		err("T_hold_maintain timer does not exist\n");
		retval = SII_OS_STATUS_ERR_NOT_AVAIL;
		goto done;
	}

	/* Delete the T_hold_maintain timer */
	retval = SiiOsTimerDelete(T_hold_maintain_timer);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Could not stop T_hold_maintain timer: %x\n", retval);
		goto done;
	}
	T_hold_maintain_timer = NULL;

	/* Recreate the T_hold_maintain timer */
	retval = SiiOsTimerCreate("T_hold_maintain_timer",
					T_hold_maintain_timer_function,
					NULL, false, 0, false,
					&T_hold_maintain_timer);
	if (SII_OS_STATUS_SUCCESS != retval) {
		err("Unable to create T_hold_maintain timer: %x\n", retval);
		T_hold_maintain_timer = NULL;
		goto done;
	}

done:
	return retval;
}

int generate_rcp_input_event(uint8_t rcp_keycode)
{
	/*
	 * Since in MHL, bit 7 == 1 indicates key release and
	 *       in Linux, zero means key release,
	 * use XOR (^) to invert the sense.
	 */
	int status = -EINVAL;
	int index = rcp_keycode & RCP_KEY_ID_MASK;

	/*dbg("");*/

	if ((KEY_UNKNOWN != rcp_def_keymap[index]) &&
	    (KEY_RESERVED != rcp_def_keymap[index])) {
		enum rcp_event_e event;
		bool key_press = (rcp_keycode & RCP_KEY_RELEASED_MASK) ?
						false : true;

		if (key_press) {
			if (rcp_support_table[index].press_and_hold_key) {
				if (index == rcp_previous_key)
					event = PRESS_AND_HOLD_KEY_PRESS_SAME;
				else
					event = PRESS_AND_HOLD_KEY_PRESS;
			} else {
				if (index == rcp_previous_key)
					event = NORMAL_KEY_PRESS_SAME;
				else
					event = NORMAL_KEY_PRESS;
			}
		} else {
			if (rcp_support_table[index].press_and_hold_key) {
				if (index == rcp_previous_key)
					event = PRESS_AND_HOLD_KEY_RELEASE_SAME;
				else
					event = PRESS_AND_HOLD_KEY_RELEASE;
			} else {
				if (index == rcp_previous_key)
					event = NORMAL_KEY_RELEASE_SAME;
				else
					event = NORMAL_KEY_RELEASE;
			}
		}
		status = handle_rcp_event(rcp_keycode, rcp_current_key, event);
	}

	rcp_previous_key = rcp_current_key;
	rcp_current_key = rcp_keycode;

	return status;
}
#endif

#ifdef OLD_KEYMAP_TABLE
int generate_rcp_input_event(uint8_t rcp_keycode)
{
	int status = -EINVAL;

	dbg("");

	if (NULL != rcp_input_dev) {
		if (rcp_keycode < ARRAY_SIZE(rcp_def_keymap) &&
			rcp_def_keymap[rcp_keycode] != KEY_UNKNOWN &&
			rcp_def_keymap[rcp_keycode] != KEY_RESERVED) {

			input_report_key(rcp_input_dev, rcp_def_keymap[rcp_keycode], 1);
			input_report_key(rcp_input_dev, rcp_def_keymap[rcp_keycode], 0);
			input_sync(rcp_input_dev);

			status = 0;
		}
	}
	return status;
}
#endif

int init_rcp_input_dev(void)
{
	int retval = 0;
#ifndef OLD_KEYMAP_TABLE
	enum sii_os_status rv = SII_OS_STATUS_SUCCESS;
#endif
	unsigned int i;

	dbg("");

	if (NULL != rcp_input_dev) {
		warn("RCP input device already exists!\n");
		goto done;
	}

	rcp_input_dev = input_allocate_device();
	if (NULL == rcp_input_dev) {
		err("Failed to allocate RCP input device\n");
		retval = -EFAULT;
		goto done;
	}

	set_bit(EV_KEY, rcp_input_dev->evbit);

/*	rcp_input_dev->phys = "mdt_kbd/input0"; */
	rcp_input_dev->name = "Remote Control";
	rcp_input_dev->keycode = rcp_def_keymap;
	rcp_input_dev->keycodesize = sizeof(u16);
	rcp_input_dev->keycodemax = ARRAY_SIZE(rcp_def_keymap);

	for (i = 1; i < ARRAY_SIZE(rcp_def_keymap); i++) {
		u16 keycode = rcp_def_keymap[i];
		if ((KEY_UNKNOWN != keycode) && (KEY_RESERVED != keycode))
			set_bit(keycode, rcp_input_dev->keybit);
	}

	rcp_input_dev->id.bustype = BUS_VIRTUAL;
/*	rcp_input_dev->id.vendor = 0x1095; */
/*	rcp_input_dev->id.product = 0x8240; */

	retval = input_register_device(rcp_input_dev);
	if (retval) {
		err("Failed to register device\n");
		goto done;
	}

#ifndef OLD_KEYMAP_TABLE
	rv = SiiOsTimerCreate("T_press_mode_timer",
				T_press_mode_timer_function,
				NULL, false, 0, false, &T_press_mode_timer);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("Unable to create T_press_mode timer\n");
		retval = -EFAULT;
		goto done;
	}

	rv = SiiOsTimerCreate("T_hold_maintain_timer",
				T_hold_maintain_timer_function,
				NULL, false, 0, false, &T_hold_maintain_timer);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("Unable to create T_hold_maintain timer\n");
		retval = -EFAULT;
		goto done;
	}
#endif

	dbg("device created\n");

done:
	if (retval)
		destroy_rcp_input_dev();
	return retval;
}

void destroy_rcp_input_dev(void)
{
	dbg("");

	if (NULL != T_press_mode_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(T_press_mode_timer)) {
			err("Failed to delete T_press_mode timer\n");
		}
		T_press_mode_timer = NULL;
	}
	if (NULL != T_hold_maintain_timer) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsTimerDelete(T_hold_maintain_timer)) {
			err("Failed to delete T_hold_maintain timer\n");
		}
		T_hold_maintain_timer = NULL;
	}
	if (NULL != rcp_input_dev) {
		input_unregister_device(rcp_input_dev);
		rcp_input_dev = NULL;
	}
}

void rcp_input_dev_one_time_init(void)
{
	int i;

	dbg("");

	for (i = 0; i < NUM_RCP_KEY_CODES; i++)
		rcp_def_keymap[i] = rcp_support_table[i].map[0];
}

