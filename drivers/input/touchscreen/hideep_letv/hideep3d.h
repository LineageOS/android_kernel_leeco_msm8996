/*************************************************************************
 * Copyright (C) 2012 Hideep, Inc.
 * kim.liao@hideep.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *************************************************************************/

#ifndef _LINUX_HIDEEP3D_H
#define _LINUX_HIDEEP3D_H

/*************************************************************************
 * this is include special HEAD file.
 *************************************************************************/
#ifdef CONFIG_FB
	#include <linux/fb.h>
	#include <linux/notifier.h>
#endif

/*************************************************************************
 * this is include normal HEAD file.
 *************************************************************************/
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>
#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/input/mt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/gfp.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/dma-mapping.h>
#include <linux/fcntl.h>
#include <linux/unistd.h>

/*************************************************************************
 * definition part.
 * define is (open, set, enable) if not, is (close, clear, disable)
 * some special switch of functions.
 *************************************************************************/
/* HIDEEP_DEBUG_DEVICE is for input output function for debugging. */
#define HIDEEP_DEBUG_DEVICE

/* HIDEEP_AUTO_UPDATE is for FW automatically update function. */
#define HIDEEP_AUTO_UPDATE
#ifdef HIDEEP_AUTO_UPDATE
/* UNIT ms */
#define HIDEEP_UPDATE_FW_THREAD_DELAY 5000
#endif

/* HIDEEP_PROTOCOL_2_0 is for protocol 2.0. */
#define HIDEEP_PROTOCOL_2_0

/* HIDEEP_DWZ_VERSION_CHECK if define, it will check dwz version. */
#define HIDEEP_DWZ_VERSION_CHECK

/* HIDEEP_SELFTEST_MODE if define, it will use self test. */
#define HIDEEP_SELFTEST_MODE

#ifdef HIDEEP_SELFTEST_MODE
#define SELF_TEST_DATA_ADDR 0x1008
#define TEST_MODE_COMMAND_ADDR 0x0804
#endif

#define HIDEEP3D_DD_VERSION_MAJOR	0
#define HIDEEP3D_DD_VERSION_MINOR 0
#define HIDEEP3D_DD_DESCRIPTION	"HiDeep 3D reference Device Driver"


/*************************************************************************
 * Firmware name.
 *************************************************************************/
#define HIDEEP3D_AUTO_FW						"hideep3d_auto.bin"
#define HIDEEP3D_MAN_FW						"hideep3d_man.bin"

/*************************************************************************
 * board porting config
 *************************************************************************/
#define HIDEEP3D_DEV_NAME						"HiDeep3D:"
#define HIDEEP3D_DEBUG_DEVICE_NAME			"hideep3d_debug"
#define HIDEEP3D_I2C_NAME						"hideep_3d"
#define HIDEEP3D_I2C_ADDR						0x6C

/*************************************************************************
 * register addr, touch & key event config
 *************************************************************************/
#define HIDEEP3D_MT_MAX						10
#define HIDEEP3D_KEY_MAX      				3
#define HIDEEP3D_EVENT_COUNT_ADDR				0x0240
#define HIDEEP3D_TOUCH_DATA_ADDR				(HIDEEP_EVENT_COUNT_ADDR + 2)
#define HIDEEP_KEY_DATA_ADDR				(HIDEEP_TOUCH_DATA_ADDR + (sizeof(struct hideep_mt_t)) * HIDEEP_MT_MAX)
#define HIDEEP_VR_IMAGE_ADDR				0x1000
#define HIDEEP_DWZ_ADDR						0x02C0
#define NVM_DWZ_LEN							(0x0400 - 0x280)	//why 0x280?

/*************************************************************************
 * command list
 *************************************************************************/
#define HIDEEP_EMPTY			(0x0000)
#define HIDEEP_RESET			(0x0802)
#define HIDEEP_TEST_MODE		(0x0804)
#define HIDEEP_SLEEP_MODE		(0x080F)
#define HIDEEP_SELF_TEST_MODE	(0x0810)
#define HIDEEP_SELF_MODE1	(0x0803)
#define HIDEEP_SELF_MODE2	(0x0804)
#define HIDEEP_CHECK_COMMAND	(0x000c)


/*************************************************************************
 * HIDEEP REGULATOR NAME
 *************************************************************************/
/*disable hideep regulator,regulator powered by touchscreen*/
/*#define HIDEEP_USE_REGULATOR*/
#ifdef HIDEEP_USE_REGULATOR
#define  HIDEEP_REGULATOR_VDD "vdd_ana"
#define  HIDEEP_REGULATOR_VID "vdd_io"
#else
#define  HIDEEP_REGULATOR_VDD NULL
#define  HIDEEP_REGULATOR_VID NULL
#endif


/*************************************************************************
 * HIDEEP PROTOCOL
 *************************************************************************/
#ifdef  HIDEEP_PROTOCOL_2_0
struct hideep3d_mt_t {
	u16 x;
	u16 y;
	u16 z;
	u8 w;
	u8 flag;
	u8 type;
	u8 index;
};
#else
struct hideep3d_mt_t {
	u8 flag;
	u8 index;
	u16 x;
	u16 y;
	u8 z;
	u8 w;
};
#endif

enum e_dev_state {
	power_init = 1,
	power_normal,
	power_sleep,
	power_updating,
	power_debugging,
};

/*************************************************************************
 * panel info & firmware info
 *************************************************************************/
struct dwz_3dinfo_t {
	u32 c_begin;	// code start address
	u16 c_crc[6];	// code crc

	u32 d_begin;	// custom code
	u16 d_len;		// custom code length
	u16 rsv0;

	u32 v_begin;	// vreg code
	u16 v_len;		// vreg code length
	u16 rsv1;

	u32 f_begin;	// vreg code
	u16 f_len;		// vreg code length
	u16 rsv2;

	u16 ver_b;		// version information
	u16 ver_c;
	u16 ver_d;
	u16 ver_v;

	u8 factory_id;
	u8 panel_type;
	u8 model_name[6];	// model name
	u16 product_code;	// product code
	u16 extra_option;	// extra option

	u16 ver_ft_major;
	u16 ver_ft_minor;
};

/*************************************************************************
 * driver information for hideep_t
 *************************************************************************/
struct hideep_platform_data_t {
	u32 version;

	int reset_gpio;

	const char *regulator_vdd;	//vdd regulator name
	const char *regulator_vid;	//vid regulator name

	struct regulator *vcc_vdd;  // main voltage
	struct regulator *vcc_vid;  // I/O voltage

	struct pinctrl *pinctrl;
	struct pinctrl_state *reset_up;
	struct pinctrl_state *reset_down;

	const char *project_name;
};

struct hideep_debug_dev_t {
	u8 *p_data;
	struct cdev cdev;
	u32 ready;
	u8 *vr_buff;
	u8 *im_buff;
	u16 im_size;
	u16 vr_size;
	bool release_flag;
	bool enable;
	int tx_num;
	int rx_num;
#ifdef HIDEEP_SELFTEST_MODE
	u8 *self_buff;
	u16 frame_size;
	u8 *rawdata;
#endif
	struct hideep3d_t *h3d;
};

struct hideep_debug_cfg_t {
	u16 im_size;
	u16 vr_size;
};

struct hideep3d_t {
	struct i2c_client *client;
	struct hideep_platform_data_t *p_data;

	struct mutex dev_mutex;
	struct mutex i2c_mutex;

#ifdef HIDEEP_DEBUG_DEVICE
	u32 debug_dev_no;
	struct hideep_debug_dev_t debug_dev;
	struct class *debug_class;
#endif

	u32 vr_addr;
	u32 vr_data;
	u32 vr_size;

	bool suspended;
	enum e_dev_state dev_state;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

#ifdef CONFIG_FB
	struct notifier_block fb_notif;
#endif

	long tch_bit;
	u32 tch_count;

	struct hideep3d_mt_t touch_evt[HIDEEP3D_MT_MAX];

//#ifdef CONFIG_AULU_Z
#if 1
	u16 z_buffer;
	bool z_status;
	u32 z_index;

	u32 z_calib_start;
	u32 z_calib_end;
	bool z_flag_calib2;
	bool z_ready_flag;
	u16 z_data[4096];
	bool z_flag_ready;
	short z_value_buf;
#endif

	u8 i2c_buf[256];

	struct dwz_3dinfo_t *dwz_info;

	bool manually_update;
	struct workqueue_struct *p_workqueue_fw;
	struct delayed_work work_fwu;

	u16 addr;
	struct work_struct work;
	int h3d_fw_update_state;
};

extern int loglevel3d;
extern struct hideep3d_t *g_h3d;

/*************************************************************************
 * function define
 *************************************************************************/
#ifdef HIDEEP_SELFTEST_MODE
int hideep3d_init_proc(struct hideep3d_t *h3d);
void hideep3d_uninit_proc(void);
#endif
int hideep3d_sysfs_init(struct hideep3d_t *h3d);
int hideep3d_sysfs_exit(struct hideep3d_t *h3d);
int hideep3d_load_ucode(struct device *dev, const char *fn);
unsigned int hideep3d_chipid(struct hideep3d_t *h3d);
int hideep3d_load_dwz(struct hideep3d_t *h3d);
int hideep3d_i2c_read(struct hideep3d_t *h3d, u16 addr, u16 len, u8 *buf);
int hideep3d_i2c_write(struct hideep3d_t *h3d, u16 addr, u16 len, u8 *buf);
#ifdef HIDEEP_DEBUG_DEVICE
int hideep3d_debug_init(struct hideep3d_t *h3d);
void hideep3d_debug_uninit(struct hideep3d_t *h3d);
#endif
int hideep3d_fuse_ucode(struct i2c_client *client, u8 *code, size_t len, int offset);
void hideep3d_reset_ic(struct hideep3d_t *h3d);
int hideep3d_enter_pgm(struct i2c_client *client);
void hideep3d_power(struct hideep3d_t *h3d, int on);
unsigned short hideep3d_get_value(unsigned short x, unsigned short y);
void hideep3d_release_flag(void);
int hideep3d_dev_state_init_updating(void);

/*************************************************************************
 * debug message
 *************************************************************************/
#define HIDEEP3D_LOG_LEVEL_DEBUG	2
#define HIDEEP3D_LOG_LEVEL_XY		3
#define HIDEEP3D_LOG_LEVEL_I2C	4
#define HIDEEP3D_INFO(a, arg...) printk(KERN_INFO HIDEEP3D_DEV_NAME "info %s:%d " a "\n", __func__, __LINE__,##arg)
#define HIDEEP3D_WARN(a, arg...) printk(KERN_WARNING HIDEEP3D_DEV_NAME "warning %s:%d " a "\n", __func__, __LINE__, ##arg)
#define HIDEEP3D_ERR(a, arg...) printk(KERN_ERR HIDEEP3D_DEV_NAME "error %s:%d " a "\n", __func__, __LINE__, ##arg)
#define HIDEEP3D_DBG(a, arg...)\
do {\
	if (loglevel3d > HIDEEP3D_LOG_LEVEL_DEBUG)\
	printk(KERN_ERR HIDEEP3D_DEV_NAME "debug %s:%d "a"\n", __func__, __LINE__, ##arg);\
} while(0)

#define HIDEEP3D_XY(a, arg...)\
do {\
	if (loglevel3d > HIDEEP3D_LOG_LEVEL_XY)\
		printk(KERN_ERR HIDEEP3D_DEV_NAME " %s:%d " a "\n", __func__, __LINE__, ##arg);\
} while(0)

#define HIDEEP3D_I2C(a, arg...)\
do {\
	if( loglevel3d > HIDEEP3D_LOG_LEVEL_I2C)\
		printk(KERN_INFO HIDEEP3D_DEV_NAME " %s:%d " a "\n", __func__, __LINE__, ##arg);\
} while(0)

#endif /* _LINUX_HIDEEP3D_TS_H */

