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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/termios.h>

#include "osal.h"
#include "hal.h"
#include "device.h"
#include "sii6400.h"
#include "wihd_sm.h"
#include "mhl_sm.h"
#include "diag_sm.h"
#include "host_msg.h"

static uint32_t nv_data_offset;
static uint16_t nv_data_size;

/* static function prototypes */
static ssize_t debug_test_cmd(const char *buf, size_t count);
static ssize_t reboot_sii6400(void);
static ssize_t load_sii6400(const char *buf, size_t count);
static int reboot_validation_board(void);

static ssize_t sys_test_cmd(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sys_reboot_sii6400(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sys_load_sii6400(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sys_uevent_test_sii6400(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sys_read_nv_sii6400(struct device *dev,
				struct device_attribute *attr, char *buf);
static ssize_t sys_write_nv_sii6400(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sys_set_nv_data_offset_sii6400(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);
static ssize_t sys_set_nv_data_size_sii6400(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

/*
 * Send a command to the sii6400 and wait for a response.
 * Write is followed by a read to get the response.
 * The response is sent to the kernel debug log.
 */
static ssize_t debug_test_cmd(const char *buf, size_t count)
{
	int retval = 0;
	int reset = 0;
	uint32_t cmd_data = 0;
	int mb_num = 0;
	uint32_t resp_data = 0xffffffff;
	int rv = 0;
	enum sii_status sii_rv = SII_STATUS_SUCCESS;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	if (1 == sscanf(buf, "ECHO %x\n", &cmd_data)) {
		uint32_t spiCommand[2];
		uint32_t rxData[2];

		/* register left-shifted by two bits to give address */
		spiCommand[0] = ECHO_CMD_WORD;
		/* upper 3 bytes of commandWord are value to be written */
		spiCommand[0] |= (cmd_data << 8);
		/* the second command is a NOP to get the read value back */
		spiCommand[1] = NOP_CMD_WORD;

		sii_rv = SiiHalSpiOp(spiCommand, rxData, 2);
		if (SII_STATUS_SUCCESS != sii_rv) {
			err("ECHO 0x%06x command error: 0x%x\n",
			    cmd_data, sii_rv);
			retval = -EFAULT;
		}

		dbg("ECHO 0x%06x returned with 0x%08x\n", cmd_data, rxData[1]);
	} else if (1 == sscanf(buf, "WRITE_TEST %u\n", &cmd_data)) {
		struct SiiOsTime start_time, end_time, timediff;
		int64_t timediff_ns_avg, timediff_ns_min, timediff_ns_max;
		uint32_t i;

		timediff_ns_min = 0x7fffffffffffffffLL;
		timediff_ns_max = 0;
		SiiOsGetTimeCurrent(&start_time);
		for (i = 0; i < cmd_data; i++) {
			struct SiiOsTime start_one_time, end_one_time;
			int64_t timediff_ns;

			resp_data = 0x123456;

			SiiOsGetTimeCurrent(&start_one_time);

			retval = SiiDeviceRegWrite(SII_REG_MBOX_10, resp_data);
			if (retval < 0) {
				err("WMB10 0x123456 command error: 0x%x\n",
				    retval);
			}

			SiiOsGetTimeCurrent(&end_one_time);
			timediff.time = timespec_sub(end_one_time.time,
						start_one_time.time);
			timediff_ns = ((int64_t)timediff.time.tv_sec *
						NSEC_PER_SEC) +
						timediff.time.tv_nsec;
			timediff_ns_min = (timediff_ns_min < timediff_ns) ?
						timediff_ns_min : timediff_ns;
			timediff_ns_max = (timediff_ns < timediff_ns_max) ?
						timediff_ns_max : timediff_ns;
		}
		SiiOsGetTimeCurrent(&end_time);
		timediff.time = timespec_sub(end_time.time, start_time.time);
		timediff_ns_avg = ((int64_t)timediff.time.tv_sec *
						NSEC_PER_SEC) +
						timediff.time.tv_nsec;
		do_div(timediff_ns_avg, cmd_data);
		dbg("WMB10 %u times results:\n", cmd_data);
		dbg("%lld ns min, %lld ns max, %lld ns avg\n",
		    timediff_ns_min, timediff_ns_max, timediff_ns_avg);
	} else if (2 == sscanf(buf, "WMB%d %x\n", &mb_num, &cmd_data)) {
		resp_data = cmd_data & 0xffffff;
		retval = SiiDeviceRegWrite(
			(enum sii_spi_reg_num)((int)SII_REG_MBOX_00 + mb_num),
					resp_data);
		if (retval < 0) {
			err("WMB%d 0x%06x command error: 0x%x\n",
			    mb_num, cmd_data, retval);
		}
		dbg("WMB%d 0x%06x returned with 0x%06x\n",
		    mb_num, cmd_data, resp_data);
	} else if (1 == sscanf(buf, "RMB%d\n", &mb_num)) {
		retval = SiiDeviceRegRead(
			(enum sii_spi_reg_num)((int)SII_REG_MBOX_00 + mb_num),
					&resp_data);
		if (retval < 0)
			err("RMB%d command error: 0x%x\n", mb_num, retval);
		dbg("RMB%d returned with 0x%06x\n", mb_num, resp_data);
	} else if (1 == sscanf(buf, "RST%d\n", &reset)) {
		if (0 == reset) {
			/* exit state machines */
			sii6400_wihd_sm_disable();
			sii6400_mhl_sm_disable();
			sii6400_diag_sm_disable();
		}
		sii_rv = SiiHalReset((0 == reset) ? true : false);
		if (SII_STATUS_SUCCESS != sii_rv) {
			err("RST%d command error: 0x%x\n", reset, sii_rv);
			retval = -EFAULT;
		}
		dbg("RESET%d sent OK\n", reset);
	} else if (!strncmp(buf, "BULK\n", count)) {
		uint32_t bulk_data[] = {
			0x04030201, 0x08070605, 0x0c0b0a09, 0x100f0e0d,
			0x14131211, 0x18171615, 0x1c1b1a19, 0x201f1e1d,
			0x24232221, 0x28272625, 0x2c2b2a29, 0x302f2e2d,
			0x34333231, 0x38373635, 0x3c3b3a39, 0x403f3e3d,
			0x44434241, 0x48474645, 0x4c4b4a49, 0x504f4e4d,
			0x54535251, 0x58575655, 0x5c5b5a59, 0x605f5e5d,
			0x64636261, 0x68676665, 0x6c6b6a69, 0x706f6e6d,
			0x74737271, 0x78777675, 0x7c7b7a79, 0x807f7e7d,
			0x84838281, 0x88878685, 0x8c8b8a89, 0x908f8e8d,
			0x94939291, 0x98979695, 0x9c9b9a99, 0xa09f9e9d,
			0xa4a3a2a1, 0xa8a7a6a5, 0xacabaaa9, 0xb0afaead,
			0xb4b3b2b1, 0xb8b7b6b5, 0xbcbbbab9, 0xc0bfbebd,
			0xc4c3c2c1, 0xc8c7c6c5, 0xcccbcac9, 0xd0cfcecd,
			0xd4d3d2d1, 0xd8d7d6d5, 0xdcdbdad9, 0xe0dfdedd,
			0xe4e3e2e1, 0xe8e7e6e5, 0xecebeae9, 0xf0efeeed,
			0xf4f3f2f1, 0xf8f7f6f5, 0xfcfbfaf9, 0x00fffefd,
			0,
		};

		sii_rv = SiiHalBulkTxfr(bulk_data, (uint32_t)sizeof(bulk_data));
		if (SII_STATUS_SUCCESS != sii_rv) {
			err("Bulk Data Xfer message error: 0x%x\n", sii_rv);
			retval = -EFAULT;
		} else {
			dbg("BULK sent OK\n");
		}
	} else if (!strncmp(buf, "CHECK_INTS\n", count)) {
		uint32_t current_ints = 0;
		uint32_t current_int_mask = 0;

		rv = SiiDeviceRegRead(SII_REG_INT_N_STATUS, &current_ints);
		if (rv < 0)
			err("Error reading INT_N_STATUS: 0x%x\n", rv);
		else
			dbg("current_ints = 0x%06x\n", current_ints);

		rv = SiiDeviceRegRead(SII_REG_INT_N_ENABLE, &current_int_mask);
		if (rv < 0)
			err("Error reading INT_N_ENABLE: 0x%x\n", rv);
		else
			dbg("current_int_mask = 0x%06x\n", current_int_mask);

		if (!(current_int_mask & current_ints))
			dbg("No interrupts to be handled\n");
	} else if (!strncmp(buf, "REBOOT\n", count)) {
		retval = reboot_sii6400();
	} else if (!strncmp(buf, "LOAD\n", count)) {
#ifdef SII6400_COMBINED_FIRMWARE
		retval = load_sii6400(wihd_mhl_firmware,
					strlen(wihd_mhl_firmware));
#else
		retval = load_sii6400(wihd_firmware, strlen(wihd_firmware));
#endif
	} else if (!strncmp(buf, "WMSGTEST\n", count)) {
		SendHMTestMsg(32);
	} else if (!strncmp(buf, "WMSGDISCON\n", count)) {
		SendHMDisconnectMsg();
	} else if (!strncmp(buf, "WMSGCON\n", count)) {
		SendHMConnectMsg();
	} else if (1 == sscanf(buf, "WMBOX %x\n", &cmd_data)) {
		DoMailboxTest(cmd_data & 0xff);
	} else {
		warn("Cannot send unknown command %s", buf);
	}

done:
	return retval;
}

/*
 * Reboot the sii6400.
 */
static ssize_t reboot_sii6400(void)
{
	int retval = 0;

	dbg("");

	retval = HostMsgTerm();
	if (retval < 0) {
		err("HostMsgTerm failed\n");
		goto done;
	}

	retval = reboot_validation_board();
	if (retval < 0) {
		err("Reboot failed\n");
		goto done;
	}
	dbg("Reboot succeeded\n");

	retval = HostMsgInit();
	if (retval < 0) {
		err("HostMsgInit failed\n");
		goto done;
	}

done:
	return retval;
}

/*
 * Load firmware to the sii6400.
 */
static ssize_t load_sii6400(const char *buf, size_t count)
{
	int retval = 0;
#ifdef SII6400_COMBINED_FIRMWARE
	const char *firmware_file = wihd_mhl_firmware;
#else
	const char *firmware_file = wihd_firmware;
#endif
	char *filepath = NULL;

	dbg("");

	filepath = SiiOsCalloc("FWFilePath", FILEPATH_NAME_LEN + 1, 0);
	if (NULL == filepath) {
		err("Out of memory\n");
		retval = -ENOMEM;
		goto done;
	}

	/* If invalid file path, use default file path */
	if ((NULL != buf) && (0 < count) && ('/' == *buf)) {
		if (1 == sscanf(buf, "%" __stringify(FILEPATH_NAME_LEN) "s",
								filepath)) {
			firmware_file = filepath;
		}
	}
	dbg("firmware file input = %s", buf);
	dbg("firmware file to use = %s\n", firmware_file);

	retval = SiiDeviceFirmwareBoot(firmware_file);
	if (retval < 0)
		retval = -EFAULT;

done:
	SiiOsFree(filepath);
	return retval;
}

/*
 * Reboot the board.
 * Returns: 0 if successful
 *          an error (<0) if the command did not get sent or
 *		an invalid response was received
 */
static int reboot_validation_board(void)
{
	int retval = -1;
	int i;

	dbg("");

	for (i = 0; i < MAX_REBOOT_RETRY; i++) {
		int error = 0;
		enum sii_status sii_rv = SII_STATUS_SUCCESS;

		if (sii6400_module_is_exiting())
			goto done;

		dbg("Reboot board: attempt %d\n", i + 1);

		sii6400_boot_successful = false;
		sii6400_boot_failed = false;

		/* Set reset high */
		sii_rv = SiiHalReset(true);
		if (SII_STATUS_SUCCESS != sii_rv) {
			err("RESET0 command error: 0x%x\n", sii_rv);
			/*goto done;*/
		}

		msleep(RESET0_INTERVAL);

		/* Set reset low */
		sii_rv = SiiHalReset(false);
		if (SII_STATUS_SUCCESS != sii_rv) {
			err("RESET1 command error: 0x%x\n", sii_rv);
			/*goto done;*/
		}

		/* Wait for the BOOT_SUCCESS interrupt */
		error = wait_event_interruptible_timeout(sii6400_interrupt_wait,
			(sii6400_boot_successful),
			msecs_to_jiffies(WAIT_TIME_FOR_BOOT_SUCCESS_INTERRUPT));
		if (0 == error) {
			continue; /* timeout */
		} else if (error < 0) {
			retval = -1;
			goto done;
		}

		if (sii6400_boot_successful)
			break;
	}
	sii6400_boot_successful = false;
	sii6400_boot_failed = false;
	if (MAX_REBOOT_RETRY == i)
		retval = -1;

done:
	return retval;
}

int sii6400_device_dbg_open(struct inode *inode, struct file *filp)
{
	dbg("");
	return 0;
}

int sii6400_device_dbg_close(struct inode *inode, struct file *filp)
{
	dbg("");
	return 0;
}

/*
 * Send a command to the sii6400 and wait for a response.
 */
static ssize_t sys_test_cmd(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int retval = 0;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	retval = debug_test_cmd(buf, count);

done:
	if (0 <= retval)
		retval = count;
	return retval;
}

/*
 * Reboot the sii6400.
 */
static ssize_t sys_reboot_sii6400(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval = 0;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	retval = reboot_sii6400();

done:
	if (0 <= retval)
		retval = count;
	return retval;
}

/*
 * Load firmware onto the sii6400.
 */
static ssize_t sys_load_sii6400(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int retval = 0;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	retval = load_sii6400(buf, count);

done:
	if (0 <= retval)
		retval = count;
	return retval;
}

/*
 * Send a test UEvent with the provided payload.
 */
#define DATA_FILE_NAME_LEN 128
static ssize_t sys_uevent_test_sii6400(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval = 0;
	char *payload = NULL;
	struct file *filp = NULL;
	char data_file[DATA_FILE_NAME_LEN + 1] = {0};
	unsigned int data_size = 0;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	if (2 != sscanf(buf, "%" __stringify(DATA_FILE_NAME_LEN) "s %u",
						data_file, &data_size)) {
		warn("Invalid UEvent Data File name or size: %s", buf);
		retval = -EINVAL;
		goto done;
	}

	if (PAGE_SIZE <= data_size) {
		warn("Unusable data size: %u\n", data_size);
		retval = -EINVAL;
		goto done;
	}

	payload = SiiOsCalloc("UEventTest", PAGE_SIZE, 0);
	if (NULL == payload) {
		err("Out of memory\n");
		retval = -ENOMEM;
		goto done;
	}

	filp = filp_open(data_file, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		err("Could not open UEvent Data File (%s): 0x%x\n",
		    data_file, (unsigned int)PTR_ERR(filp));
		retval = -EINVAL;
		goto done;
	}

	if ((NULL == filp) ||
	    (NULL == filp->f_op) ||
	    (NULL == filp->f_op->read)) {
		warn("Cannot read UEvent Data file\n");
	} else {
		mm_segment_t oldfs = get_fs();

		set_fs(KERNEL_DS);

		filp->f_pos = 0;
		retval = filp->f_op->read(filp,
					(unsigned char __user __force *)payload,
					data_size, &filp->f_pos);
		if (retval < 0) {
			err("UEvent Data file read failed: 0x%x\n", retval);
			goto file_done;
		}

file_done:
		set_fs(oldfs);
	}

	filp_close(filp, NULL);

	if (retval < 0)
		goto done;

	if (NULL != dev) {
		char *envp[] = {payload, NULL};

		(void)kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
	}

done:
	SiiOsFree(payload);
	if (0 <= retval)
		retval = count;
	return retval;
}

static ssize_t sys_read_nv_sii6400(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int retval = 0;
	unsigned int copied = 0;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	copied = read_from_nv_storage(buf, nv_data_size, nv_data_offset);
	buf[nv_data_size + 1] = 0;
	dbg("Read data %s (size %u, offset %u) from NV Storage file\n",
	    buf, nv_data_size, nv_data_offset);

done:
	if (0 <= retval)
		retval = copied;
	return retval;
}

static ssize_t sys_write_nv_sii6400(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval = 0;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	dbg("Write data %s (size %u, offset %u) to NV Storage file\n",
	    buf, (uint32_t)count, nv_data_offset);
	retval = write_to_nv_storage(buf, count, nv_data_offset);

done:
	if (0 <= retval)
		retval = count;
	return retval;
}

static ssize_t sys_set_nv_data_offset_sii6400(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval = 0;
	int rv = 0;
	unsigned long data_offset = 0;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &data_offset);
	if (rv) {
		warn("Invalid Sii6400 NV Data Offset %s", buf);
		retval = rv;
		goto done;
	}

	if (NV_STORAGE_SIZE <= data_offset) {
		warn("Supplied data offset %lu too large\n", data_offset);
		retval = -EINVAL;
		goto done;
	}
	nv_data_offset = (uint32_t)data_offset;
	dbg("NV Data Offset = %u\n", nv_data_offset);

done:
	if (0 <= retval)
		retval = count;
	return retval;
}

static ssize_t sys_set_nv_data_size_sii6400(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval = 0;
	int rv = 0;
	unsigned long data_size = 0;

	dbg("");

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == buf) {
		err("Invalid buf pointer\n");
		retval = -EINVAL;
		goto done;
	}

	rv = strict_strtoul(buf, 0, &data_size);
	if (rv) {
		warn("Invalid Sii6400 NV Data Size %s", buf);
		retval = rv;
		goto done;
	}

	if ((NV_STORAGE_SIZE <= data_size) || (PAGE_SIZE <= data_size)) {
		warn("Supplied data size %lu too large\n", data_size);
		retval = -EINVAL;
		goto done;
	}
	nv_data_size = (uint16_t)data_size;
	dbg("NV Data Size = %u\n", nv_data_size);

done:
	if (0 <= retval)
		retval = count;
	return retval;
}

/*
 * Declare the sysfs entries for Sii6400 Debug Attributes.
 * These macros create instances of:
 *   dev_attr_test_cmd
 *   dev_attr_reboot
 *   dev_attr_load
 *   dev_attr_uevent_test
 *   dev_attr_nv_in
 *   dev_attr_nv_out
 *   dev_attr_nv_offset
 *   dev_attr_nv_size
 */
static DEVICE_ATTR(test_cmd, (S_IWUSR|S_IWGRP), NULL, sys_test_cmd);
static DEVICE_ATTR(reboot, (S_IWUSR|S_IWGRP), NULL, sys_reboot_sii6400);
static DEVICE_ATTR(load, (S_IWUSR|S_IWGRP), NULL, sys_load_sii6400);
static DEVICE_ATTR(uevent_test, (S_IWUSR|S_IWGRP), NULL, sys_uevent_test_sii6400);
static DEVICE_ATTR(nv_in, (S_IRUGO), sys_read_nv_sii6400, NULL);
static DEVICE_ATTR(nv_out, (S_IWUSR|S_IWGRP), NULL, sys_write_nv_sii6400);
static DEVICE_ATTR(nv_offset, (S_IWUSR|S_IWGRP), NULL, sys_set_nv_data_offset_sii6400);
static DEVICE_ATTR(nv_size, (S_IWUSR|S_IWGRP), NULL, sys_set_nv_data_size_sii6400);

static struct attribute *sii6400_dbg_attrs[] = {
	&dev_attr_test_cmd.attr,
	&dev_attr_reboot.attr,
	&dev_attr_load.attr,
	&dev_attr_uevent_test.attr,
	&dev_attr_nv_in.attr,
	&dev_attr_nv_out.attr,
	&dev_attr_nv_offset.attr,
	&dev_attr_nv_size.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group sii6400_dbg_attr_group = {
	.name = __stringify(debug),
	.attrs = sii6400_dbg_attrs,
};

/* Debug device initialization and release */
int sii6400_dbg_init(struct sii6400_device_info *devinfo)
{
	int retval = 0;

	dbg("");

	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -EFAULT;
		goto failed_nullptr;
	}

	retval = sysfs_create_group(&devinfo->device->kobj,
					&sii6400_dbg_attr_group);
	if (retval < 0)
		warn("failed to create Debug attribute group\n");

	return 0;

failed_nullptr:
	return retval;
}

void sii6400_dbg_exit(struct sii6400_device_info *devinfo)
{
	dbg("");

	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		return;
	}

	sysfs_remove_group(&devinfo->device->kobj, &sii6400_dbg_attr_group);
}

