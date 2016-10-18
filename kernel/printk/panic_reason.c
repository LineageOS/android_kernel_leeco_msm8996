/*
 *  linux/kernel/printk.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 * LETV---wupeng
 */

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
#include <linux/panic_reason.h>
#include <trace/events/printk.h>
#include <linux/utsname.h>
#include <linux/rtc.h>
#include <linux/crc16.h>

#include "printk_i.h"
#include "last_kmsg.h"

u8 last_panic_trig_reason;
u8 panic_trig_reason;
u32 panic_trig_reason_with_crc;

void set_panic_trig_rsn(u8 val)
{
	/* A state machine is here.
	*  From 0 --> TRIG_INIT_STATE
	*  From TRIG_INIT_STATE  --> to any other CRASH_STATE,such as TRIG_MODEM_RESET
	*  From one crash state to another one is illegal
	*  KERNEL_ALIVE is a addon flag, it could add to any others.
	*/
	u8 tmp = panic_trig_reason;
	u16 crc16_val;

	do{
		if ((tmp == 0) && (val == TRIG_INIT_STATE)){
			break;
		}else if (val == KERNEL_ALIVE){
			break;
		}else if ((tmp == TRIG_INIT_STATE) && (val != TRIG_INIT_STATE))
			break;

		pr_err("Error: panic_trig_reason from 0x%x to 0x%x.\n", tmp, val);
	}while(0);

	if (val == KERNEL_ALIVE)
		val = tmp | KERNEL_ALIVE;

	panic_trig_reason = val;

	if (val != TRIG_INIT_STATE)
		pr_info("set_panic_trig_rsn: from 0x%x to 0x%x\n", tmp, panic_trig_reason);

	crc16_val = crc16(CRC16_START_VAL, (u8 *)(&val), 1);
	panic_trig_reason_with_crc = (crc16_val <<16) | val;

	return;
}

static u8 get_panic_trig_rsn(void *p)
{
	last_dbg_info_t *p_dbg = (last_dbg_info_t *)(p + LAST_DBG_HEAD_OFFSET);
	u32 raw_val = p_dbg->panic_info.panic_reason;
	u16 crc16_old, crc16_new;
	u8 reason;

	reason = raw_val & 0xff;
	crc16_old = raw_val >> 16;

	crc16_new = crc16(CRC16_START_VAL, (u8 *)(&reason), 1);

	if (crc16_old == crc16_new) {
		last_panic_trig_reason = reason;
	} else {
		pr_warn("panic_trig_reason broken raw_val=0x%x.\n", raw_val);
		last_panic_trig_reason = PANIC_REASON_BROKEN;
	}

	pr_info("%s: last_panic_reason raw_val=0x%x\n",
				__FUNCTION__,raw_val);
	return last_panic_trig_reason;
}

void panic_reason_hook(void *p)
{
	panic_reason_addr_info_t *p_tmp = (panic_reason_addr_info_t *)p;
	p_tmp->last_panic_reason_addr = (u64)virt_to_phys(&panic_trig_reason_with_crc);
}

static ssize_t panic_rsn_read(struct file *file, char __user *buf,
		      size_t len, loff_t * offset)
{
	char tmp_buf[8];
	int ret;

	ret = snprintf(tmp_buf, 8, "0x%x\n", last_panic_trig_reason);
	return simple_read_from_buffer(buf, len, offset,tmp_buf,ret);
}

static const struct file_operations panic_rsn_file_ops = {
	.owner = THIS_MODULE,
	.read = panic_rsn_read,
};

int last_panic_reason_init(void)
{
	void *p = get_mem_last_info();
	u8 ret = 0;

	if (p == NULL)
		return 0;

	ret = get_panic_trig_rsn(p);
	if (PANIC_REASON_BROKEN == ret) {
		pr_info("panic_reason is broken");
	}

	proc_create("lst_pnc_rsn", S_IRUGO, NULL,&panic_rsn_file_ops);

	return 0;
}

int panic_reason_init(void)
{
	set_panic_trig_rsn(TRIG_INIT_STATE);
	return 0;
}

module_init(panic_reason_init);
