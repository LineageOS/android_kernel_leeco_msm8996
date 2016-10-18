/*******************************************************************************
 * Copyright (C) 2014 HiDeep, Inc.
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
 *******************************************************************************/
#include "hideep3d.h"
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#ifdef HIDEEP_SELFTEST_MODE
struct proc_dir_entry *hideep_proc = NULL;
static int proc_en = 0;

static int hideep_seq_show(struct seq_file *s, void *v)
{
	int ret, i, time_out = 0;
	int tx = g_h3d->debug_dev.tx_num;
	int rx = g_h3d->debug_dev.rx_num;
	int frm_size = tx * rx * 2;
	int data_size = tx * rx;
	unsigned char state;
	unsigned char frm_data[frm_size];
	unsigned short data[data_size];

	mutex_lock(&g_h3d->dev_mutex);

	do {
		mdelay(50);
		HIDEEP3D_INFO("time_out = %d", ++time_out);
		hideep3d_i2c_read(g_h3d, HIDEEP_CHECK_COMMAND, 1, &state);
		if (time_out > 50)
			goto self_data_err;
	} while (!state);

	mdelay(200);
	ret = hideep3d_i2c_read(g_h3d, SELF_TEST_DATA_ADDR, frm_size, frm_data);

	if (ret < 0)
		goto self_data_err;

	memcpy(data, frm_data, frm_size);

	for (i = 0; i < data_size; i++) {
		seq_printf(s, "%d,", data[i]);
	}
	seq_printf(s, "\n");
	hideep3d_reset_ic(g_h3d);
	mutex_unlock(&g_h3d->dev_mutex);

	return 0;

self_data_err:
	hideep3d_reset_ic(g_h3d);
	HIDEEP3D_ERR("self data read error");
	return -1;
}

static int hideep_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, hideep_seq_show, NULL);
}

static const struct file_operations hideep_rawdata_proc_fops = {
	.owner = THIS_MODULE,
	.open = hideep_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int hideep3d_init_proc(struct hideep3d_t *h3d)
{
	int ret = 0;
	struct proc_dir_entry *rawdata = NULL;

	hideep_proc = proc_mkdir("hideep", NULL);
	if (hideep_proc == NULL) {
		HIDEEP3D_ERR("Can't not create proc entry!!!!");
		ret = -ENOMEM;
		goto hideep_init_proc_exit;
	}

	rawdata = proc_create_data("selftest", 0666, hideep_proc, &hideep_rawdata_proc_fops, h3d);
	if (rawdata == NULL) {
		ret = -ENOMEM;
		goto hideep_init_proc_exit;
	}

	proc_en = 1;
	return 0;
hideep_init_proc_exit:
	return ret;
}

void hideep3d_uninit_proc(void)
{
	HIDEEP3D_INFO("");
	remove_proc_entry("selftest", hideep_proc);
	remove_proc_entry("hideep", NULL);
	proc_en = 0;
	HIDEEP3D_INFO("");
}
#endif