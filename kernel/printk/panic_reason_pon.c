/*
 *  linux/kernel/printk.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 * LETV---wupeng
 */
#define pr_fmt(fmt) "last: " fmt

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/nmi.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/security.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <linux/ratelimit.h>
#include <linux/cpu.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <trace/events/printk.h>
#include <linux/utsname.h>
#include <linux/rtc.h>
#include <linux/crc16.h>

#include "printk_i.h"
#include "last_kmsg.h"
#include "panic_reason_pon.h"

extern int qpnp_pon_spare_reg_masked_write(u8 off, u8 mask, u8 reg_val);
extern int qpnp_pon_spare_reg_masked_read(u8 off, u8 mask);
int last_panic_reason_init(void);

extern int last_kmsg_init_status;
int last_panic_trig_reason = -1;
u8 last_pon_reason;
u8 panic_trig_reason;
u32 panic_trig_reason_with_crc;
u32 xbl_copied_flg = KERN_MAGIC_FOR_XBL;
int read_fmt_flg = 1;

struct panic_desc{
	char *str;
	u8 val;
}panic_desc_t;

struct panic_desc reason_arr[] =
{
	{"Normal", 0},//
	{"SUB_SYSTEM_RESTART", TRIG_SUB_SYSTEM_RESET}, //
	{"WDOG_BAK", TRIG_WDOG_BARK}, //
	{"WDOG_BITE", TRIG_INIT_STATE},//
	{"SYSRQ", TRIG_SYSRQ_CRASH},//
	{"OVER_TEMP", TRIG_OVER_TEMPERATURE},//
	{"KERN_BUG_MARCRO", TRIG_KERNEL_BUG_MACRO},
	{"SLUB_BUG", TRIG_SLUB_BUG},
	{"KERN_MEM_FAULT", TRIG_KERN_MEM_FAULT},
	{"LONG_PRESS_PWR_KEY", TRIG_LONG_PRESS_PWR_KEY},//
	{"KERN_BAD_MODE", TRIG_BAD_MODE},
	{"GPU_HANG", TRIG_GPU_HANG},//
	{"S3_RESET", TRIG_S3_RESET},//
	{"UNKNOWN", 0xff},//
};

static u8 fetch_rsn(void)
{
	int ret1, ret2;
	u8 reg = 0;

	ret1 = qpnp_pon_spare_reg_masked_read(PANIC_PON_REG_OFFSET, PANIC_PON_MASK);
	ret2 = qpnp_pon_spare_reg_masked_read(ALIVE_REG_OFFSET, ALIVE_MASK);
	ret2 = (ret2 >> ALIVE_SHIFT);
	reg = ret1 | (ret2 << 7);

	pr_info("fetch_rsn=0x%x.\n", reg);
	return reg;
}

static bool store_rsn(u8 val)
{
	u8 reg;
	int ret;

	reg = (val & 0x7f) << PANIC_PON_SHIFT;
	ret = qpnp_pon_spare_reg_masked_write(PANIC_PON_REG_OFFSET, PANIC_PON_MASK, reg);
	if (ret < 0)
		return false;

	if ((val == TRIG_INIT_STATE) || (val & KERNEL_ALIVE)) {
		reg = ((val & KERNEL_ALIVE) >> 7) << ALIVE_SHIFT;
		ret = qpnp_pon_spare_reg_masked_write(ALIVE_REG_OFFSET, ALIVE_MASK, reg);
		if (ret < 0)
			return false;
	}

	return true;
}

void set_panic_trig_rsn(u8 val)
{
	/* A finite state machine is here.
	*  From 0 --> TRIG_INIT_STATE
	*  From TRIG_INIT_STATE  --> to any other CRASH_STATE, both low priority and high one
	*  the overide by the same priority is wrong, but high one could overide low ones.
	*  KERNEL_ALIVE is a addon flag, it could add to any others.
	*/
	u8 old_rsn = panic_trig_reason;
	int flag = 0;

	do{
		if ((old_rsn == 0) && (val == TRIG_INIT_STATE))
			break;
		else if (val == KERNEL_ALIVE)
			break;
		else if ((old_rsn == TRIG_INIT_STATE) && (val != TRIG_INIT_STATE))
			break;
		else if ((old_rsn & PANIC_LOW_PRIO) && (val & PANIC_HIGH_PRIO))
			break;

		flag = 1;
	}while(0);

	if (flag == 1) {
		pr_err("Error: panic_trig_reason from 0x%x to 0x%x.\n", old_rsn, val);
		return;
	}

	if (val == KERNEL_ALIVE)
		val = old_rsn | KERNEL_ALIVE;

	panic_trig_reason = val;
	if (val != TRIG_INIT_STATE)
		pr_info("set_panic_trig_rsn: from 0x%x to 0x%x\n", old_rsn, panic_trig_reason);


	store_rsn(val);
	return;
}

static ssize_t panic_rsn_read(struct file *file, char __user *buf, size_t len, loff_t * offset)
{
	int ret;

	/* last_kmsg is later than panic_reason. so return NOT_READY to
	*	ask app read again.
	*/

	/*TODO: if -EAGAIN is better? */
	if  (last_kmsg_init_status == -1) {
		 return simple_read_from_buffer(buf, len, offset, "NOT_READY", strlen("NOT_READY"));
	}

	if (last_panic_trig_reason == -1)
		last_panic_reason_init();

	if (read_fmt_flg == 1) {
		unsigned int arr_sz = ARRAY_SIZE(reason_arr);
		char *pstr = NULL;
		int i;

		for(i=0; i<arr_sz ; i++) {
			if (reason_arr[i].val == (last_panic_trig_reason & TRIG_REASON_MASK)) {
				pstr = reason_arr[i].str;
				break;
			}
		}
		if (pstr == NULL) {
			pstr = "UNKNOWN";
		}

		ret = simple_read_from_buffer(buf, len, offset, pstr, strlen(pstr));
	} else {
		char tmp_buf[8];
		ret = snprintf(tmp_buf, 8, "0x%x\n", last_panic_trig_reason);
		ret = simple_read_from_buffer(buf, len, offset, tmp_buf, ret);
	}

	return ret;
}

static ssize_t panic_rsn_write (struct file *file, const char __user *buf,
			size_t len, loff_t *offset)
{
	pr_info("input fmt=%s", buf);

	if(strcmp("string", buf ) == 0)
		read_fmt_flg = 1;
	else
		read_fmt_flg = 0;
	return len;
}

static const struct file_operations panic_rsn_file_ops = {
	.owner = THIS_MODULE,
	.read = panic_rsn_read,
	.write = panic_rsn_write,
};

int last_panic_reason_init(void)
{
	int tmp_rsn = last_pon_reason & 0x7f;

	if ((boot_reason == 1) && (tmp_rsn == TRIG_CMD_REBOOT)) //normal reboot
		last_panic_trig_reason = 0;
	else if ((boot_reason > 2) && (tmp_rsn == TRIG_CMD_POWEROFF)) //normal poweron
		last_panic_trig_reason = 0;
	else if (last_kmsg_init_status == 1) //new kenrel version
		last_panic_trig_reason = 0;

	if (last_panic_trig_reason != -1)
		goto exit;

	if ((tmp_rsn == TRIG_CMD_REBOOT) || (tmp_rsn == TRIG_CMD_POWEROFF))
		tmp_rsn = 0xff;

	if (last_kmsg_init_status == 4)
		last_panic_trig_reason = TRIG_S3_RESET; //s3 reset
	else
		last_panic_trig_reason = tmp_rsn;

exit:
	pr_info("last_panic_trig_reason=0x%x.\n", last_panic_trig_reason);
	return 0;
}

void panic_reason_hook(void *p)
{
	panic_reason_addr_info_t *p_tmp = (panic_reason_addr_info_t *)p;
	p_tmp->last_panic_reason_addr = (u64)virt_to_phys(&panic_trig_reason_with_crc);
	p_tmp->xbl_copied_flg_addr = (u64)virt_to_phys(&xbl_copied_flg);
}

int panic_reason_init(void)
{
	//read panic_reason
	last_pon_reason = fetch_rsn();
	proc_create("lst_pnc_rsn", S_IRUGO|S_IWUGO, NULL,&panic_rsn_file_ops);

	set_panic_trig_rsn(TRIG_INIT_STATE);
	return 0;
}

module_init(panic_reason_init);
