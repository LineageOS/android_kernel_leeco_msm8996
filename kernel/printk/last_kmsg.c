/*
 *  linux/kernel/printk.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 * Modified to make sys_syslog() more flexible: added commands to
 * return the last 4k of kernel messages, regardless of whether
 * they've been read or not.  Added option to suppress kernel printk's
 * to the console.  Added hook for sending the console messages
 * elsewhere, in preparation for a serial line console (someday).
 * Ted Ts'o, 2/11/93.
 * Modified for sysctl support, 1/8/97, Chris Horn.
 * Fixed SMP synchronization, 08/08/99, Manfred Spraul
 *     manfred@colorfullife.com
 * Rewrote bits to get rid of console_lock
 *	01Mar01 Andrew Morton
 */

#define pr_fmt(fmt) "last: " fmt

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/nmi.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>			/* For in_interrupt() */
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/security.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <linux/aio.h>
#include <linux/syscalls.h>
#include <linux/kexec.h>
#include <linux/kdb.h>
#include <linux/ratelimit.h>
#include <linux/kmsg_dump.h>
#include <linux/syslog.h>
#include <linux/cpu.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/panic_reason.h>
#include <trace/events/printk.h>
#include <linux/utsname.h>
#include <linux/rtc.h>
#include <linux/crc16.h>
#include <linux/panic_reason.h>

#include "printk_i.h"
#include "last_kmsg.h"

#ifndef CONFIG_LOG_BUF_MAGIC
	#error "last_kmsg need CONFIG_LOG_BUF_MAGIC"
#endif

int last_kmsg_init_status = -1;

/*======================================
* debug  for last_kmsg
*=======================================
*/
static int last_debug_mask;
module_param_named(
	debug_mask, last_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

/*
* ********************NOTICE***********************
*
* Most of function of this file is a clone of the functions in
* printk.c.
* If the function in printk.c change, this file must be changed
* accordingly.
*
*/
static char *log_buf_la;
static u32 log_buf_len_la;

static u64 log_first_seq_la;
static u32 log_first_idx_la;

static u64 log_next_seq_la;
static u32 log_next_idx_la;

/*======================================
* definetion for last_kmsg
*=======================================
*/
#define FIRST_RUN_DELAY_IN_MS (8*1000)
#define TRY_DELAY_IN_MS (2*1000)
#define TRY_MAX_TIMES (30)
#define LAST_KMSG_LOG_BROKEN		(0x9999)
#define MAX_READ_LEN ((__LOG_BUF_LEN) + ((__LOG_BUF_LEN) >> 2))

extern int qpnp_pon_spare_reg_masked_write(u8 off, u8 mask, u8 reg_val);
extern int qpnp_pon_spare_reg_masked_read(u8 off, u8 mask);
extern void last_kmsg_info_hook(last_kmsg_addr_info_t *p);
extern int last_panic_reason_init(void);
extern const char *pon_to_str(u8 pon);
extern const char *poff_to_str(u16 poff);
extern const char *wam_reset_to_str(u16 rst_rsn);
static int last_kmsg_open(struct inode *inode, struct file *file);
static int last_kmsg_release(struct inode *inode, struct file *file);
static ssize_t last_kmsg_bin_read(struct file *file, char __user *buf,
										size_t len, loff_t * offset);
static void collect_last_debug_addr_info(void);
static void collect_addr_func(struct work_struct *work);
static void last_kmsg_handler(struct work_struct *work);

void *memory_addr_base = NULL;
int memory_size;
struct mutex last_kmsg_lock;
static struct file *filp_parti = NULL;

struct last_kmsg_user {
	u64 seq;
	u32 idx;
	int add_tail;
};

#define DEFAULT_KERNELLOG_FILENAME "/dev/block/bootdevice/by-name/kernellog"
char *kernlog_file = DEFAULT_KERNELLOG_FILENAME;

static int flg;
static DECLARE_DELAYED_WORK(addr_wr, collect_addr_func);
static DECLARE_DELAYED_WORK(lk_wr, last_kmsg_handler);
struct workqueue_struct *p_lk_wq = NULL;

/*
* ********************NOTICE***********************
*
* log_next()
* log_from_idx()

* The 2 functions are redefine here, because the functions
* in printk.c access to const array log_buf[]
* although we also could use the log_buf as name for
* last_kmsg buffer, but it could make ram_parser not work.
* ram_parser is a tool from Qualcomm to parse ramdump to
* get kmsg in dump.
*/

static struct printk_log *log_from_idx(u32 idx, char *buf)
{
	struct printk_log *msg;

	msg = (struct printk_log *)(buf + idx);

	if (!msg->len)
		return (struct printk_log *)buf;
	return msg;
}

static u32 log_next(u32 idx, char *buf)
{
	struct printk_log *msg;

	msg = (struct printk_log *)(buf + idx);

	if (!msg->len) {
		msg = (struct printk_log *)buf;
		return msg->len;
	}
	return idx + msg->len;
}

static ssize_t last_kmsg_pkg_scan(void)
{
	char *last_kmsg_buf = log_buf_la;
	u64 next_seq = log_next_seq_la;
	u64 seq = log_first_seq_la;
	u32 idx = log_first_idx_la;

	enum log_flags prev = 0;
	int len = 0;

	while (len >= 0 && seq < next_seq) {
		struct printk_log *msg;
		int textlen;

		if (idx > (__LOG_BUF_LEN - sizeof(struct printk_log))) {
			pr_err("Error: idx error idx=%u.\n", idx);
			len = -LAST_KMSG_LOG_BROKEN;
			break;
		}

		msg = log_from_idx(idx, last_kmsg_buf);
		if (msg->magic != LOG_MAGIC_VAL) {
			pr_err("Error: magic error idx=%u magic=0x%x.\n",
					idx, msg->magic);
			len = -LAST_KMSG_LOG_BROKEN;
			break;
		}

		textlen = msg_print_text(msg, prev, true, NULL, 0);

		if (textlen < 0) {
			pr_err("Error: textlen error idx=%u textlen=%d.\n",
					idx, textlen);
			len = -LAST_KMSG_LOG_BROKEN;
			break;
		}

		idx = log_next(idx, last_kmsg_buf);
		seq++;
		prev = msg->flags;

		len += textlen;
	}

	return len;
}

static int last_kmsg_print(struct file *file,
							void __user *buf, loff_t pos, size_t count)
{
	struct last_kmsg_user *user = file->private_data;
	char *text = NULL;
	char *last_kmsg_buf = NULL;
	int len = 0;
	u64 next_seq;
	u64 seq;
	u32 idx;
	enum log_flags prev;

	if (buf == NULL)
		return 0;

	text = kmalloc(LOG_LINE_MAX + PREFIX_MAX, GFP_KERNEL);
	if (!text)
		return -ENOMEM;

	mutex_lock(&last_kmsg_lock);

	last_kmsg_buf = log_buf_la;
	/* last message fitting into this dump */
	next_seq = log_next_seq_la;
	len = 0;
	prev = 0;
	idx = user->idx;
	seq = user->seq;

	while (len >= 0 && seq < next_seq) {
		struct printk_log *msg;
		int textlen;

		if (idx > (__LOG_BUF_LEN - sizeof(struct printk_log))) {
			pr_err("Error: idx error idx=%u.\n", idx);
			break;
		}

		if (last_debug_mask & 0x02)
			pr_info("idx= %u seq= %llu\n", idx, seq);

		msg = log_from_idx(idx, last_kmsg_buf);
		textlen = msg_print_text(msg, prev, true, text, LOG_LINE_MAX + PREFIX_MAX);

		if (textlen < 0) {
			/* error report*/
			pr_err("Error: textlen error idx=%u textlen=%d.\n",
					idx, textlen);
			len = textlen;
			break;
		}

		if ((len + textlen) > count) {
			/* buf is nearly full.*/
			break;
		}

		idx = log_next(idx, last_kmsg_buf);
		seq++;
		prev = msg->flags;

		if (copy_to_user(buf + len, text, textlen)) {
			pr_err("copy_to_user fail in %s.\n", __func__);
			len = -EFAULT;
			break;
        }

		len += textlen;
	}

	user->idx = idx;
	user->seq = seq;

	mutex_unlock(&last_kmsg_lock);
	kfree(text);

	return len;
}

static ssize_t last_kmsg_print_tail(struct file *file, char __user *buf, size_t len)
{
	struct last_kmsg_user *user = file->private_data;
	last_dbg_info_t *p_info =
						(last_dbg_info_t *)(memory_addr_base + LAST_DBG_HEAD_OFFSET);
	pon_poff_info_t *p = &p_info->pon_poff;
	uint32 val = 0;
	char *buff = NULL;
	size_t buff_len = MAX(len, LOG_LINE_MAX + PREFIX_MAX);

	if ((user->add_tail==0) || (memory_addr_base==NULL)) {
		return 0;
	}

	buff = kmalloc(buff_len, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	user->add_tail = 0;

	do {
		/* raw value */
		len = snprintf(buff, buff_len,
				"PON SUMMARY: PM0=%llx PM1=%llx gcc_reset_status=0x%x\n",
				p->pm0, p->pm1, p->reset_status_reg);
		if (buff_len <= len)
			break;

#if 0
		/* boot type */
		len += snprintf(buff+len, buff_len-len,
				"%s\n", cold_boot ? "cold boot" : "warm_boot");
		if (buff_len <= len)
			break;
#endif
		/* power on status */
		len += snprintf(buff+len, buff_len - len,
				"Power ON REASON: %s\n", pon_to_str(p->pm0 & 0xff));
		if (buff_len <= len)
			break;

		/* warm reset on status */
		val = (p->pm0 &0xffff0000) >> 16;
		if (val) {
			len += snprintf(buff+len, buff_len - len,
				"WARM RESET: %s\n", wam_reset_to_str(val));
		}
		if (buff_len <= len)
			break;

		/* power off status */
		val = (p->pm0 &0xffff00000000) >> 32;
		if (val) {
			len += snprintf(buff+len, buff_len - len,
				"Power OFF REASON: %s\n", poff_to_str(val));
		}
		if (buff_len <= len)
			break;
	} while(0);

	if (copy_to_user(buf, buff, len)) {
		pr_err("copy_to_user fail in %s.\n", __func__);
		len = -EFAULT;
	}
	if (buff)
		kfree(buff);

	return len;
}
static ssize_t last_kmsg_print_all(struct file *file, char __user *buf,
		      size_t len, loff_t * offset)
{
	loff_t pos = *offset;
	int ret = 0;

	/* the data in kmsg buffer is stored in packet. when it expand to string
	* sometimes, it'll greater than buffer size. so set MAX_READ_LEN larger
	* than __LOG_BUF_LEN
	*/
	if ((pos < 0) || (pos > MAX_READ_LEN))
		return -EINVAL;

	ret = last_kmsg_print(file, buf, pos, len);

	if (ret > 0) {
		*offset = pos + ret;
	} else if(ret == 0) {
		ret = last_kmsg_print_tail(file, buf, len);
	}
	return ret;
}

static ssize_t last_kmsg_set(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	if (count) {
		char c;

		if (get_user(c, buf))
			return -EFAULT;
		if (c != '0')
			flg = c - '0';
	}
	return count;
}

static void last_kmsg_set_first(struct last_kmsg_user *user)
{
	user->idx = log_first_idx_la;
	user->seq = log_first_seq_la;
	user->add_tail = 1;
}

static int last_kmsg_open(struct inode *inode, struct file *file)
{
	struct last_kmsg_user *user;
	/* write-only does not need any file context */
	if ((file->f_flags & O_ACCMODE) == O_WRONLY)
		return 0;

#ifdef FEATURE_NOT_AVAILABLE_BY_LETV
	{
	int err;
	err = check_syslog_permissions(SYSLOG_ACTION_READ_ALL,
				       SYSLOG_FROM_READER);
	if (err)
		return err;
	}
#endif

	user = kmalloc(sizeof(struct last_kmsg_user), GFP_KERNEL);
	if (!user)
		return -ENOMEM;

	last_kmsg_set_first(user);
	file->private_data = user;

	return 0;
}

static int last_kmsg_release(struct inode *inode, struct file *file)
{
	struct last_kmsg_user *user = file->private_data;

	if (user != NULL)
		kfree(user);

	return 0;
}

static const struct file_operations last_kmsg_file_ops = {
	.owner = THIS_MODULE,
	.open = last_kmsg_open,
	.read = last_kmsg_print_all,
	.write = last_kmsg_set,
	.release = last_kmsg_release,
};

static ssize_t last_kmsg_bin_read
(
struct file *file, char __user *buf,size_t len, loff_t * offset
)
{
	return simple_read_from_buffer(buf, len, offset, log_buf_la, __LOG_BUF_LEN);
}

static const struct file_operations last_kmsg_bin_file_ops = {
	.owner = THIS_MODULE,
	.open = last_kmsg_open,
	.read = last_kmsg_bin_read,
	.release = last_kmsg_release,
};

/* return 1 if match, 0 for mismatch */
static inline int check_crc16(void *buf, size_t size, u16 crc)
{
	u16 crc_new = crc16(CRC16_START_VAL, buf, size);
	if (crc_new != crc) 
		pr_info("crc =0x%hx crc_new=0x%hx", crc, crc_new);
	return (crc_new == crc) ? 1 : 0;
}

/*
* For the 4 critical global variable, each one has a crc16 value.
* if crc16 match, means the variable is right.
* return value:
*	-1 ----last_kmsg is no valid
*	0  ---- partly broken
*	1  ----valid
*/
static int check_last_kmsg_crc16(last_kmsg_info_t *p)
{
	int crc_valid_cnt = 0;
	int ret = 0;

	/* if all idx and seq are 0, last_kmsg is invalid. */
	if ((p->log_first_idx == 0) &&
		(p->log_first_seq == 0) &&
		(p->log_next_idx == 0) &&
		(p->log_next_seq == 0)) 
	{
		return -1;
	}

	/*log_first_idx*/
	ret = check_crc16((void *)(&p->log_first_idx),
						sizeof(p->log_first_idx),
						p->log_first_idx_crc16);
	crc_valid_cnt += ret;
	if (last_debug_mask)
		pr_info("crc16 check, right cnt=%d\n", crc_valid_cnt);

	/*log_first_seq*/
	ret = check_crc16((void *)(&p->log_first_seq),
						sizeof(p->log_first_seq),
						p->log_first_seq_crc16);
	crc_valid_cnt += ret;
	if (last_debug_mask)
		pr_info("crc16 check, right cnt=%d\n", crc_valid_cnt);

	/*log_next_idx*/
	ret = check_crc16((void *)(&p->log_next_idx),
					sizeof(p->log_next_idx),
					p->log_next_idx_crc16);
	crc_valid_cnt += ret;
	if (last_debug_mask)
		pr_info("crc16 check, right cnt=%d\n", crc_valid_cnt);

	/*log_next_seq*/
	ret = check_crc16((void *)(&p->log_next_seq),
						sizeof(p->log_next_seq),
						p->log_next_seq_crc16);
	crc_valid_cnt += ret;
	if (last_debug_mask)
		pr_info("crc16 check, right cnt=%d\n", crc_valid_cnt);

	pr_info("crc16 check, right cnt=%d\n", crc_valid_cnt);
	if (crc_valid_cnt == 0)
		ret = -1;
	else if (crc_valid_cnt == 4)
		ret = 1;
	else
		ret = 0;

	return ret;
}

void* get_mem_last_info(void)
{
	void *p = NULL;
	int order =0;

	if (memory_addr_base != NULL)
		return memory_addr_base;

	order = get_order(LAST_DBG_MEM_LEN);
	p = (void *)__get_free_pages(GFP_KERNEL, order);

	if (IS_ERR_OR_NULL(p)) {
		WARN(1, "Unable to ioremap reserved memory.\n");
		return NULL;
	}

	memory_addr_base = p;
	memory_size = LAST_DBG_MEM_LEN;
	return p;
}

void free_mem_last_info(void)
{
	if (memory_addr_base != NULL) {
		free_pages((unsigned long)memory_addr_base,
					get_order(LAST_DBG_MEM_LEN));
		memory_addr_base = NULL;
	}
}

static bool is_version_match(char *ver)
{
	bool ret = false;
	char buf[KERN_VER_LEN];
	u32 len = strlen(linux_banner);

	if (strlen(ver) == 0)
		return false;

	if (len + 1 > KERN_VER_LEN)
		len = KERN_VER_LEN - 1;

	memcpy(buf, ver, len + 1);
	buf[len] = 0;
	if (0 == strncmp(buf, linux_banner, len))
		ret = true;

	return ret;
}

static bool is_valid_last(last_dbg_info_t *p)
{
	return p->status == sizeof(last_dbg_info_t);
}

static bool collect_kern_ver(char *buf, u32 size)
{
	int len = snprintf(buf, size, "%s", linux_banner);
	return (len == strlen(linux_banner)) ? true : false;
}

struct file * open_storage_device(void)
{
	struct file * filp = filp_parti;

	if (filp == NULL) {
		filp = filp_open(kernlog_file, O_RDWR, 0);
		if (IS_ERR_OR_NULL(filp)) {
			filp = NULL;
		} else if ((NULL == filp->f_op) || (NULL == filp->f_op->write) || (NULL == filp->f_op->read)) {
			filp_close(filp, NULL);
		} else {
			filp_parti = filp;
		}
	}

	return filp_parti;
}

void close_storage_device(void)
{
	if(! IS_ERR_OR_NULL(filp_parti)) {
		filp_close(filp_parti, NULL);
		filp_parti = NULL;
	}

	return;
}

static void collect_addr_func(struct work_struct *work)
{
	last_dbg_addr_info_t *p_addr = NULL;
	last_dbg_addr_info_t addr_tmp;
	struct file *filp = NULL;
	ssize_t len = 0;
	bool result = true;

	if (last_debug_mask)
		pr_info("write parti begin.");

	filp = filp_parti;
	collect_last_debug_addr_info();

	p_addr = (last_dbg_addr_info_t *)(memory_addr_base + LAST_DBG_HEAD_ADDR_OFFSET);

	/* write */
	filp->f_pos = LAST_DBG_HEAD_ADDR_OFFSET;
	len = filp->f_op->write(filp, (void*)p_addr,
							sizeof(last_dbg_addr_info_t), &(filp->f_pos));

	if (len < sizeof(last_dbg_addr_info_t)) {
		result = false;
		goto exit;
	}
	vfs_fsync(filp, false);

	/* read & verify */
	filp->f_pos = LAST_DBG_HEAD_ADDR_OFFSET;
	len = filp->f_op->read(filp, (void*)(&addr_tmp),
						sizeof(last_dbg_addr_info_t), &(filp->f_pos));

	if (len < sizeof(last_dbg_addr_info_t)) {
		result = false;
		goto exit;
	}

	if (0 != memcmp(p_addr, &addr_tmp, sizeof(last_dbg_addr_info_t))) {
		result = false;
	}

exit:
	if (result == false) {
		/* fail */
		WARN(1, "update last_addr_info fail.\n");
	} else {
		/* succ*/
		pr_info("update last_addr_info succ.\n");
	}

	/* in this case, last_kmsg is not available and the resource is useless. 
	* so close file and free memory
	*/
	close_storage_device();
	free_mem_last_info();
}

static void collect_last_debug_addr_info(void)
{
	last_dbg_addr_info_t *p =(last_dbg_addr_info_t *)(memory_addr_base + LAST_DBG_HEAD_ADDR_OFFSET);
	memset(p, 0, sizeof(last_dbg_addr_info_t));

	last_kmsg_info_hook(&p->last_kmsg_addr);
	collect_kern_ver((char *)(&p->kern_ver), KERN_VER_LEN);
	panic_reason_hook(&p->panic_reason_addr);

	p->crc = calc_crc32((uint8*)p, sizeof(last_dbg_addr_info_t) - sizeof(uint32));
}

static void last_kmsg_handler(struct work_struct *work)
{
	static int lk_retry_cnt = 0;

	last_dbg_addr_info_t *p_addr = NULL;
	last_dbg_info_t *p_dbg = NULL;
	struct file *filp = NULL;
	bool need_update_addr_info = false;
	int proc_file_node_flg = 0;
	bool retry_flg = false;
	int crc_result = 0;
	ssize_t len = 0;

	void *p = NULL;
	int ret = 0;

	if (last_debug_mask)
		pr_info("enter last_kmsg_handler.\n");

	filp = open_storage_device();
	if (filp == NULL) {
		retry_flg = true;
		if (last_debug_mask)
			pr_info("open storage %s fail\n", kernlog_file);
		goto exit;
	}

	p = get_mem_last_info();
	if (p == NULL) {
		retry_flg = true;
		if (last_debug_mask)
			pr_info("get memory_base fail\n");
		goto exit;
	}

	/* get addr info */
	p_addr = (last_dbg_addr_info_t *)(p + LAST_DBG_HEAD_ADDR_OFFSET);
	filp->f_pos = LAST_DBG_HEAD_ADDR_OFFSET;
	len = filp->f_op->read(filp, (void*)p_addr,
						sizeof(last_dbg_addr_info_t), &(filp->f_pos));
	if (len < sizeof(last_dbg_addr_info_t)) {
		pr_err("read last_dbg_addr_info_t fail\n");
		goto exit;
	}

	/*  check  version*/
	if (!is_version_match((char *)(&p_addr->kern_ver))){
		need_update_addr_info = true;
		pr_info("version doesn't match.\n");
		goto exit;
	}

	p_dbg = (last_dbg_info_t *)(p + LAST_DBG_HEAD_OFFSET);
	filp->f_pos = LAST_DBG_HEAD_OFFSET;
	len = filp->f_op->read(filp, (void*)p_dbg,
						sizeof(last_dbg_info_t), &(filp->f_pos));
	if (len < sizeof(last_dbg_info_t)) {
		pr_err("read last_dbg_info_t fail\n");
		goto exit;
	}

	/*  check  last if it's valid or destoried one...*/
	if (!is_valid_last(p_dbg)){
		pr_info("the last info isn't valid.\n");
		goto exit;
	}

	/* destory last debug info. */
	p_dbg->status = 0;
	filp->f_pos = LAST_DBG_HEAD_OFFSET;
	len = filp->f_op->write(filp, (void*)p_dbg,
						sizeof(last_dbg_info_t), &(filp->f_pos));
	if (len < sizeof(last_dbg_info_t)) {
		pr_err("update last_dbg_info_t fail\n");
		goto exit;
	}

	/*check crc*/
	crc_result = check_last_kmsg_crc16(&p_dbg->last_kmsg);
	if (crc_result < 0) {
		pr_info("crc16 check fail crc_result=%d.\n", crc_result);
		goto exit;
	}

	/*read last_debug_info*/
	filp->f_pos = LAST_DBG_INFO_OFFSET;
	len = filp->f_op->read(filp, (void*)p,
						p_addr->last_kmsg_addr.log_buf_len, &(filp->f_pos));
	if (len < p_addr->last_kmsg_addr.log_buf_len) {
		pr_info("read last_log_buf fail.\n");
		goto exit;
	}

	/* save log_buf of last_kmsg */
	log_buf_la = p;
	log_buf_len_la = p_addr->last_kmsg_addr.log_buf_len;

	/* scan last_kmsg and just if it's readable*/
	if (crc_result == 0) {
		proc_file_node_flg = 0x2;
	} else if (crc_result == 1) {
		log_first_seq_la = p_dbg->last_kmsg.log_first_seq;
		log_first_idx_la = p_dbg->last_kmsg.log_first_idx;
		log_next_seq_la = p_dbg->last_kmsg.log_next_seq;
		log_next_idx_la = p_dbg->last_kmsg.log_next_idx;
		ret = last_kmsg_pkg_scan();
		if (ret <= 0) {
			proc_file_node_flg = 0x3;
		} else {
			proc_file_node_flg = 0x1;
		}
	}

	if (last_debug_mask)
		pr_info("proc_file_node_flg = %d\n", proc_file_node_flg);

	/* create file node */
	if (proc_file_node_flg & 0x01) {
		proc_create("last_kmsg", S_IALLUGO, NULL, &last_kmsg_file_ops);
	}
	if (proc_file_node_flg & 0x02) {
		proc_create("last_kmsg_bin", S_IRUGO, NULL, &last_kmsg_bin_file_ops);
	}

	/*handle panic reason*/
	last_kmsg_init_status = 0;

	/* success, close file */
	close_storage_device();

	pr_info("OK at %d.", lk_retry_cnt);
	return;

exit:
	if (need_update_addr_info) {
		last_kmsg_init_status = 1;	//version not match
		queue_delayed_work(p_lk_wq, &addr_wr, 0);
	} else if (! retry_flg) {
		last_kmsg_init_status = 2;	// last_kmsg error
		close_storage_device();
		free_mem_last_info();
		pr_err("last_kmsg is not available");
	} else if (lk_retry_cnt < TRY_MAX_TIMES){
		queue_delayed_work(p_lk_wq, &lk_wr, msecs_to_jiffies(TRY_DELAY_IN_MS));
		lk_retry_cnt ++;
	} else {
		last_kmsg_init_status = 3;	//no persist storage
		close_storage_device();
		free_mem_last_info();
		pr_err("last_kmsg storage is not available");
	}
}

extern unsigned int poff_reason;
static bool is_ign_last(void)
{
	/* stage3 reset */
	if (poff_reason == 16) {
		pr_info("power off_reason=%s", poff_to_str(poff_reason-1));
		return true;
	}

#if 0
	/* besides hard reset, and SMPL */
	if (boot_reason > 2) {
		pr_info("boot_reason=%s", pon_to_str(boot_reason-1));
		return true;
	}
#endif

	return false;
}

int last_kmsg_init(void)
{
	p_lk_wq = create_singlethread_workqueue("lk_wq");
	BUG_ON(p_lk_wq == NULL);

	mutex_init(&last_kmsg_lock);

	if (!is_ign_last())
		queue_delayed_work(p_lk_wq, &lk_wr, msecs_to_jiffies(TRY_DELAY_IN_MS));
	else
		last_kmsg_init_status = 4;

	if (last_debug_mask)
		pr_info("last_kmsg_init done.\n");
	return 0;
}

/*
* NOTIFICATION:
* This MUST run after qpnp_pon_init
*/
module_init(last_kmsg_init);
