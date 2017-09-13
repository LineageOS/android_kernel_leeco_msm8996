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
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/crc32.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>

#include "osal.h"
#include "hal.h"
#include "sii6400.h"
#include "device.h"
#include "host_msg.h"
#include "osal_linux_kernel.h"

DECLARE_WAIT_QUEUE_HEAD(sii6400_interrupt_wait);

/* Semaphore to protect access to the INT_N_ENABLE and
 * AP_CTRL_INT_STATUS registers */
#define MAX_WAIT_FOR_INT_N_ENABLE_SEMAPHORE		3000
#define MAX_WAIT_FOR_AP_CTRL_INT_STATUS_SEMAPHORE	3000
static struct SiiOsSemaphore *sii6400_int_n_enable_sem;
static struct SiiOsSemaphore *sii6400_ap_ctrl_int_status_sem;

/* firmware header version */
enum fw_hdr_version_type {
	FW_HDR_VERSION_01 = 1,
	FW_HDR_VERSION_02 = 2,
};

/* firmware header property IDs */
enum fw_hdr_property_type {
	FW_HDR_PRPTY_SIG = 0,
	FW_HDR_PRPTY_HDR_VERSION,
	FW_HDR_PRPTY_HW_ID,
	FW_HDR_PRPTY_HDR_LENGTH,
	FW_HDR_PRPTY_FW_SIZE,
	FW_HDR_PRPTY_FW_ENTRY,
	FW_HDR_PRPTY_MIC,
	FW_HDR_PRPTY_FW_VERSION,
	FW_HDR_PRPTY_CUST_ID,
	FW_HDR_PRPTY_CRC,
};

static int SiiDeviceFirmwareLoad(const char *firmware_file);
static int verify_firmware_file(
			struct sii6400_firmware_file_hdr_common *firmware_hdr,
			uint32_t *firmware_buf[], uint32_t num_firmware_bufs,
			uint32_t firmware_crc);
static uint32_t get_max_fw_hdr_size(void);
static int get_fw_hdr_property(void *firmware_hdr,
			enum fw_hdr_version_type fw_hdr_version,
			enum fw_hdr_property_type property_id,
			void *fw_hdr_property,
			uint32_t fw_hdr_property_size);


/*
 * Initialize the device
 * Returns: SII_STATUS_SUCCESS if successful
 *          SII_STATUS_ERR_FAILED if initialization failed
 */
enum sii_status SiiDeviceInit(void)
{
	enum sii_status retVal = SII_STATUS_SUCCESS;
	enum sii_os_status rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	init_waitqueue_head(&sii6400_interrupt_wait);

	rv = SiiOsSemaphoreCreate("int_enable_semaphore", 1, 1,
					&sii6400_int_n_enable_sem);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("Unable to create interrupt enable semaphore\n");
		retVal = SII_STATUS_ERR_FAILED;
		goto init_failed;
	}

	rv = SiiOsSemaphoreCreate("int_status_semaphore", 1, 1,
					&sii6400_ap_ctrl_int_status_sem);
	if (SII_OS_STATUS_SUCCESS != rv) {
		err("Unable to create interrupt status semaphore\n");
		retVal = SII_STATUS_ERR_FAILED;
		goto init_failed;
	}

	return SII_STATUS_SUCCESS;

init_failed:
	SiiDeviceTerm();
	return retVal;
}

/*
 * Terminate the device
 */
void SiiDeviceTerm(void)
{
	dbg("");

	wake_up_interruptible_sync(&sii6400_interrupt_wait);

	if (NULL != sii6400_int_n_enable_sem) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsSemaphoreDelete(sii6400_int_n_enable_sem)) {
			err("Failed to delete interrupt enable semaphore\n");
		}
		sii6400_int_n_enable_sem = NULL;
	}
	if (NULL != sii6400_ap_ctrl_int_status_sem) {
		if (SII_OS_STATUS_SUCCESS !=
			SiiOsSemaphoreDelete(sii6400_ap_ctrl_int_status_sem)) {
			err("Failed to delete interrupt status semaphore\n");
		}
		sii6400_ap_ctrl_int_status_sem = NULL;
	}
}

/*
 * Function to write 3 bytes of data to SPI register 0-63 (0-0x3f).
 * Must always read/write 3 bytes of data to SPI registers.
 * NOTE: implementation assumes little-endian byte order.
 */
int SiiDeviceRegWrite(enum sii_spi_reg_num myRegister, uint32_t myValue)
{
	uint32_t rxData = 0;
	uint32_t spiCommand = 0;
	enum sii_status retVal = SII_STATUS_ERR_INVALID_PARAM;

	/*dbg("");*/

	if ((uint32_t)myRegister >= (uint32_t)SII_REG_MAXRANGE) {
		err("register %u out of range\n", myRegister);
		goto done;
	}

	/* only low 3 bytes can be set */
	if (0xff000000U & myValue) {
		err("value 0x%x out of range\n", myValue);
		goto done;
	}

	/* register left-shifted by two bits to give address */
	spiCommand = (((uint32_t)myRegister) << 2);
	/* upper 3 bytes of commandWord are value to be written */
	spiCommand |= (myValue << 8);

	/* NOTE: bits 1:0 are set to 00 - this is the SPI write command */
	retVal = SiiHalSpiOp(&spiCommand, &rxData, 1);
	if (SII_STATUS_SUCCESS != retVal)
		err("SPI Write command failed\n");

done:
	return (int)retVal;
}

/*
 * Function to read 3 bytes of data from SPI register 0-63 (0-0x3f).
 * Must always read/write 3 bytes of data to SPI registers.
 * NOTE: implementation assumes little-endian byte order.
 */
int SiiDeviceRegRead(enum sii_spi_reg_num myRegister, uint32_t *pRetData)
{
	uint32_t spiCommand[2] = {0, 0};
	uint32_t rxData[2] = {0, 0};
	enum sii_status retVal = SII_STATUS_ERR_INVALID_PARAM;

	/*dbg("");*/

	if ((uint32_t)myRegister >= (uint32_t)SII_REG_MAXRANGE) {
		err("register %u out of range\n", myRegister);
		goto done;
	}

	if (NULL == pRetData) {
		err("pRetData was NULL\n");
		goto done;
	}

	/* register left-shifted by two bits to give address */
	spiCommand[0] = (((uint32_t)myRegister) << 2);
	/* set the "read command" bits */
	spiCommand[0] |= 2;
	/* the second command is a NOP to get the read value back */
	spiCommand[1] = NOP_CMD_WORD;

	retVal = SiiHalSpiOp(spiCommand, rxData, 2);
	if (SII_STATUS_SUCCESS != retVal) {
		err("SPI Read command failed\n");
		goto done;
	}

	/* mask to 3 bytes before returning */
	rxData[1] = rxData[1] & 0x00ffffff;
	*pRetData = rxData[1];

done:
	return (int)retVal;
}

/*
 * Atomically read/modify/write the SII_REG_INT_N_ENABLE register
 * Clears all bits specified by the clearMask, then sets all bits specified
 * by the setMask.
 * Returns the value of the register before any modifications were made in
 * prevVal (if not NULL).
 */
int SiiDeviceIntEnableReadWrite(uint32_t clearMask, uint32_t setMask,
				uint32_t *prevVal)
{
	int retVal = 0;
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;
	uint32_t oldVal = 0;
	uint32_t newVal = 0;

	sii_os_rv = SiiOsSemaphoreTake(sii6400_int_n_enable_sem,
					MAX_WAIT_FOR_INT_N_ENABLE_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not take Interrupt N Enable semaphore: %x\n",
		    sii_os_rv);
		retVal = (int)sii_os_rv;
		goto done;
	}

	retVal = SiiDeviceRegRead(SII_REG_INT_N_ENABLE, &oldVal);
	if (retVal < 0) {
		err("Error reading INT_N_ENABLE: 0x%x\n", retVal);
		goto done_give_sem;
	}
	/*dbg("Read INT_N_ENABLE 0x%06x\n", oldVal);*/

	newVal = (oldVal & ~clearMask) | setMask;

	retVal = SiiDeviceRegWrite(SII_REG_INT_N_ENABLE, newVal);
	if (retVal < 0) {
		err("Error writing INT_N_ENABLE: 0x%x\n", retVal);
		goto done_give_sem;
	}
	/*dbg("Write INT_N_ENABLE 0x%06x\n", newVal);*/

done_give_sem:
	sii_os_rv = SiiOsSemaphoreGive(sii6400_int_n_enable_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not release Interrupt N Enable semaphore: %x\n",
		    sii_os_rv);
	}
done:
	if (NULL != prevVal)
		*prevVal = oldVal;

	return retVal;
}

/*
 * Atomically read/modify/write the SII_REG_AP_CTRL_INT_STATUS register
 * Clears all bits specified by the clearMask, then sets all bits specified
 * by the setMask.
 * Returns the value of the register before any modifications were made in
 * prevVal (if not NULL).
 */
int SiiDeviceApIntStatusReadWrite(uint32_t clearMask, uint32_t setMask,
					uint32_t *prevVal)
{
	int retVal = 0;
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;
	uint32_t oldVal = 0;
	uint32_t newVal = 0;

	sii_os_rv = SiiOsSemaphoreTake(sii6400_ap_ctrl_int_status_sem,
				MAX_WAIT_FOR_AP_CTRL_INT_STATUS_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not take AP Ctrl Interrupt Status semaphore: %x\n",
		    sii_os_rv);
		retVal = (int)sii_os_rv;
		goto done;
	}

	retVal = SiiDeviceRegRead(SII_REG_AP_CTRL_INT_STATUS, &oldVal);
	if (retVal < 0) {
		err("Error reading AP_CTRL_INT_STATUS: 0x%x\n", retVal);
		goto done_give_sem;
	}
	/*dbg("Read AP_CTRL_INT_STATUS 0x%06x\n", oldVal);*/

	newVal = (oldVal & ~clearMask) | setMask;

	retVal = SiiDeviceRegWrite(SII_REG_AP_CTRL_INT_STATUS, newVal);
	if (retVal < 0) {
		err("Error writing AP_CTRL_INT_STATUS: 0x%x\n", retVal);
		goto done_give_sem;
	}
	/*dbg("Write AP_CTRL_INT_STATUS 0x%06x\n", newVal);*/

done_give_sem:
	sii_os_rv = SiiOsSemaphoreGive(sii6400_ap_ctrl_int_status_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not release AP Ctrl INTR Status semaphore: %x\n",
		    sii_os_rv);
	}
done:
	if (NULL != prevVal)
		*prevVal = oldVal;

	return retVal;
}

/*
 * Load firmware image to board.
 * Returns: 0 if successful
 *          an error (<0) if the command did not get sent or
 *                           an invalid response was received.
 */
static int SiiDeviceFirmwareLoad(const char *firmware_file)
{
	int retval = -1;
	int prev_retval = 0;
	enum sii_status sii_rv = SII_STATUS_SUCCESS;
	uint32_t resp_data = 0;
	void *fw_header = NULL;
	uint32_t *fw_data = NULL;
	uint32_t *fw_crc = NULL;
	struct sii6400_firmware_file_hdr_common *fw_hdr_common;
	const struct firmware *fw_entry = NULL;
	struct sii6400_device_info *devinfo = NULL;
	uint32_t firmware_hdr_version = 0;
	uint32_t firmware_hdr_size = 0;
	uint32_t firmware_hardware_id = 0;
	uint32_t firmware_size = 0;
	uint32_t firmware_entry = 0;
	uint8_t firmware_hdr_mic[8];
	char firmware_version[128];
	uint32_t firmware_customer_id = 0;
	uint32_t firmware_hdr_crc32 = 0;
	uint32_t **firmware_buf = NULL;
	uint32_t num_firmware_bufs = 0;
#ifdef FIRMWARE_DATA_MODE_FIRST
	uint32_t num_firmware_bufs_firmware = 0;
	struct file *filp = NULL;
	unsigned char *cfg_buf = NULL;
	int result = -1;
	uint32_t cfg_data_bgn = 0;
	uint32_t cfg_data_end = 0;
	uint32_t cfg_data_cnt = 0;
	uint32_t data_frag = 0;
	mm_segment_t oldfs = 0;
	bool configfile_exist = true;
#endif
	uint32_t padded_firmware_size = 0;
	uint32_t firmware_pad_size = 0;
	uint32_t firmware_start_low = 0;
	uint32_t firmware_start_high = 0;
	uint32_t auth_code_0 = 0;
	uint32_t auth_code_1 = 0;
	uint32_t auth_code_2 = 0;
	uint32_t auth_code_3 = 0;
	uint32_t i;
	int error = 0;
	struct sii6400_firmware_memory *firmware_memory;

	dbg("");

	sii6400_boot_successful = false;
	sii6400_boot_failed = false;

	if (NULL == firmware_file) {
		err("Invalid parameter\n");
		retval = -1;
		goto done;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Invalid devinfo pointer\n");
		retval = -1;
		goto done;
	}

	firmware_memory = get_firmware_memory();
	if (firmware_memory == NULL)
		err("it's can't get firmware_memory\n");

	/* Open firmware file and load header and data into buffers */
	firmware_hdr_size = get_max_fw_hdr_size();

	error = request_firmware(&fw_entry, firmware_file,
					devinfo->device);
	if ((0 != error) || (NULL == fw_entry)) {
		err("Failed to load Bow&Arrow firmware\n");
		retval = -1;
		goto done;
	}
	/* After this, need to release the firmware... */

	dbg("Read %u bytes of %s in\n",
		firmware_hdr_size, firmware_file);

	fw_header = (void *)(fw_entry->data);
	fw_data = (uint32_t *)(fw_entry->data + firmware_hdr_size);

	fw_hdr_common =
		(struct sii6400_firmware_file_hdr_common *)fw_header;
	dbg("sig = %c%c%c%c%c%c%c%c\n",
		fw_hdr_common->sig[0], fw_hdr_common->sig[1],
		fw_hdr_common->sig[2], fw_hdr_common->sig[3],
		fw_hdr_common->sig[4], fw_hdr_common->sig[5],
		fw_hdr_common->sig[6], fw_hdr_common->sig[7]);

	firmware_hdr_version = fw_hdr_common->header_version;
	dbg("header version = %u\n", firmware_hdr_version);

	retval = get_fw_hdr_property(fw_header,
			(enum fw_hdr_version_type)firmware_hdr_version,
			FW_HDR_PRPTY_HW_ID,
			(void *)&firmware_hardware_id,
			sizeof(firmware_hardware_id));
	if (0 == retval)
		dbg("hardware ID = %u\n", firmware_hardware_id);

	retval = get_fw_hdr_property(fw_header,
			(enum fw_hdr_version_type)firmware_hdr_version,
			FW_HDR_PRPTY_HDR_LENGTH,
			(void *)&firmware_hdr_size,
			sizeof(firmware_hdr_size));
	if (0 == retval) {
		dbg("header length = 0x%08x bytes\n",
		    firmware_hdr_size);
	}

	retval = get_fw_hdr_property(fw_header,
			(enum fw_hdr_version_type)firmware_hdr_version,
			FW_HDR_PRPTY_FW_SIZE,
			(void *)&firmware_size,
			sizeof(firmware_size));
	if (0 == retval)
		dbg("fsize = 0x%08x bytes\n", firmware_size);

	retval = get_fw_hdr_property(fw_header,
			(enum fw_hdr_version_type)firmware_hdr_version,
			FW_HDR_PRPTY_FW_ENTRY,
			(void *)&firmware_entry,
			sizeof(firmware_entry));
	if (0 == retval)
		dbg("fentry = 0x%08x\n", firmware_entry);

	memset(firmware_hdr_mic, 0, sizeof(firmware_hdr_mic));
	retval = get_fw_hdr_property(fw_header,
			(enum fw_hdr_version_type)firmware_hdr_version,
			FW_HDR_PRPTY_MIC,
			(void *)firmware_hdr_mic,
			sizeof(firmware_hdr_mic));
	if (0 == retval) {
		dbg("mic = 0x%02x 0x%02x 0x%02x 0x%02x\n",
		    firmware_hdr_mic[0], firmware_hdr_mic[1],
		    firmware_hdr_mic[2], firmware_hdr_mic[3]);
		dbg("      0x%02x 0x%02x 0x%02x 0x%02x\n",
		    firmware_hdr_mic[4], firmware_hdr_mic[5],
		    firmware_hdr_mic[6], firmware_hdr_mic[7]);
	}

	memset(firmware_version, 0, sizeof(firmware_version));
	retval = get_fw_hdr_property(fw_header,
			(enum fw_hdr_version_type)firmware_hdr_version,
			FW_HDR_PRPTY_FW_VERSION,
			(void *)firmware_version,
			sizeof(firmware_version));
	if (0 == retval)
		dbg("fversion = %s\n", firmware_version);

	retval = get_fw_hdr_property(fw_header,
			(enum fw_hdr_version_type)firmware_hdr_version,
			FW_HDR_PRPTY_CUST_ID,
			(void *)&firmware_customer_id,
			sizeof(firmware_customer_id));
	if (0 == retval)
		dbg("customer id = 0x%08x\n", firmware_customer_id);

	retval = get_fw_hdr_property(fw_header,
			(enum fw_hdr_version_type)firmware_hdr_version,
			FW_HDR_PRPTY_CRC,
			(void *)&firmware_hdr_crc32,
			sizeof(firmware_hdr_crc32));
	if (0 == retval)
		dbg("header CRC = 0x%08x\n", firmware_hdr_crc32);

	firmware_pad_size = firmware_size % 12 ?
				12 - firmware_size % 12 : 0;
	padded_firmware_size = firmware_size + firmware_pad_size;
	dbg("fpad_size = %u\n", firmware_pad_size);

	firmware_start_low = firmware_entry & 0x0000ffff;
	firmware_start_high = (firmware_entry & 0xffff0000) >> 16;
	dbg("firmware_start_low = 0x%08x\n", firmware_start_low);
	dbg("firmware_start_high = 0x%08x\n", firmware_start_high);

	auth_code_0 = (firmware_hdr_mic[1] << 8) | firmware_hdr_mic[0];
	auth_code_1 = (firmware_hdr_mic[3] << 8) | firmware_hdr_mic[2];
	auth_code_2 = (firmware_hdr_mic[5] << 8) | firmware_hdr_mic[4];
	auth_code_3 = (firmware_hdr_mic[7] << 8) | firmware_hdr_mic[6];
	dbg("auth_code_0 = 0x%08x\n", auth_code_0);
	dbg("auth_code_1 = 0x%08x\n", auth_code_1);
	dbg("auth_code_2 = 0x%08x\n", auth_code_2);
	dbg("auth_code_3 = 0x%08x\n", auth_code_3);

#ifdef FIRMWARE_DATA_MODE_FIRST
	filp = filp_open(configdata_file, O_RDONLY, 0);
	if (IS_ERR(filp) || (NULL == filp)) {
		configfile_exist = false;
		if (-ENOENT != PTR_ERR(filp)) {
			err("Could not open existing NVS file %s\n",
			    configdata_file);
			retval = (int)SII_OS_STATUS_ERR_FAILED;
			goto release;
		}
		warn("No such file exist\n");
		goto load_firmware;
	}

	if ((firmware_memory != NULL) &&
		firmware_memory->cfg_buf != NULL) {
		cfg_buf = firmware_memory->cfg_buf;
		memset(cfg_buf, 0, NV_STORAGE_SIZE * sizeof(unsigned char *));
	} else {
		warn("alloc memory set mode:cfg_buf\n");
		cfg_buf = SiiOsCalloc("configbuff",
			NV_STORAGE_SIZE * sizeof(unsigned char *), 0);
		if (NULL == cfg_buf) {
			err("Out of memory\n");
			retval = -1;
			goto file_done;
		}

		if (firmware_memory != NULL)
			firmware_memory->cfg_buf = cfg_buf;
	}

	if ((NULL == filp) ||
	    (NULL == filp->f_op) ||
	    (NULL == filp->f_op->read)) {
		warn("Cannot read config Data file\n");
	} else {
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		filp->f_pos = 0;
		result = filp->f_op->read(filp,
					(unsigned char __user __force *)cfg_buf,
					NV_STORAGE_SIZE, &filp->f_pos);
		set_fs(oldfs);
		if (result < 0) {
			err("Config file read failed: 0x%x\n", result);
			retval = (int)SII_OS_STATUS_ERR_FAILED;
			goto file_done;
		}
	}

load_firmware:

	num_firmware_bufs_firmware = padded_firmware_size / FIRMWARE_BUF_SIZE;
	if (padded_firmware_size % FIRMWARE_BUF_SIZE)
		num_firmware_bufs_firmware++;

	num_firmware_bufs = MAX_DEVICE_MEMORY / FIRMWARE_BUF_SIZE;
	if (MAX_DEVICE_MEMORY % FIRMWARE_BUF_SIZE)
		num_firmware_bufs++;

	if (num_firmware_bufs < num_firmware_bufs_firmware) {
		err("Out of flash\n");
		retval = -1;
		goto file_done;
	}

	if ((firmware_memory != NULL) &&
		(firmware_memory->firmware_buf != NULL)) {
		firmware_buf = firmware_memory->firmware_buf;
	} else {
		warn("alloc memory set mode:firmware_buf\n");
		firmware_buf = SiiOsCalloc("FWBufArray",
				num_firmware_bufs * sizeof(uint32_t *), 0);
		if (NULL == firmware_buf) {
			err("Out of memory\n");
			retval = -1;
			goto file_done;
		}

		if (firmware_memory != NULL)
			firmware_memory->firmware_buf = firmware_buf;
	}

	for (i = 0; i < num_firmware_bufs; i++) {
		if ((firmware_memory != NULL) &&
			(firmware_memory->firmware_buf != NULL) &&
			(firmware_memory->firmware_buf[i] != NULL)) {
			firmware_buf[i] = firmware_memory->firmware_buf[i];
			memset(firmware_buf[i], 0, FIRMWARE_BUF_SIZE);
		} else {
			warn("alloc memory set mode:firmware_buf[]\n");
			firmware_buf[i] = SiiOsCalloc("", FIRMWARE_BUF_SIZE, 0);
			if (NULL == firmware_buf[i]) {
				err("Out of memory\n");
				retval = -1;
				goto file_done;
			}
			if ((firmware_memory != NULL) &&
				(firmware_memory->firmware_buf != NULL)) {
				firmware_memory->firmware_buf[i] =
							firmware_buf[i];
			}
		}
	}

	for (i = 0; i < num_firmware_bufs_firmware - 1; i++) {
		memcpy(firmware_buf[i], &fw_data[i*FIRMWARE_BUF_SIZE/4],
						FIRMWARE_BUF_SIZE);
	}
	memcpy(firmware_buf[i], &fw_data[i*FIRMWARE_BUF_SIZE/4],
					firmware_size % FIRMWARE_BUF_SIZE);

	if (configfile_exist) {
		cfg_data_bgn = DATA_FILE_START_ADDR / FIRMWARE_BUF_SIZE;
		data_frag = DATA_FILE_START_ADDR % FIRMWARE_BUF_SIZE;
		if (0 != data_frag) {
			memcpy((void *)(&firmware_buf[cfg_data_bgn][data_frag/4]),
					(void *)&cfg_buf[cfg_data_cnt],
					FIRMWARE_BUF_SIZE - data_frag);
			cfg_data_bgn++;
			cfg_data_cnt += FIRMWARE_BUF_SIZE - data_frag;
		}

		cfg_data_end = MAX_DEVICE_MEMORY / FIRMWARE_BUF_SIZE;
		for (i = cfg_data_bgn; i < cfg_data_end; i++) {
			memcpy(firmware_buf[i], (void *)&cfg_buf[cfg_data_cnt],
							FIRMWARE_BUF_SIZE);
			cfg_data_cnt += FIRMWARE_BUF_SIZE;
		}
		memcpy(firmware_buf[i], (void *)&cfg_buf[cfg_data_cnt],
			MAX_DEVICE_MEMORY % FIRMWARE_BUF_SIZE);
	}
#else
	num_firmware_bufs = padded_firmware_size / FIRMWARE_BUF_SIZE;
	if (padded_firmware_size % FIRMWARE_BUF_SIZE)
		num_firmware_bufs++;

	if ((firmware_memory != NULL) &&
		(firmware_memory->firmware_buf != NULL)) {
		firmware_buf = firmware_memory->firmware_buf;
	} else {
		warn("alloc memory set mode: firmware_buf\n");
		firmware_buf = SiiOsCalloc("FWBufArray",
				num_firmware_bufs * sizeof(uint32_t *), 0);
		if (NULL == firmware_buf) {
			err("Out of memory\n");
			retval = -1;
			goto file_done;
		}
		if (firmware_memory != NULL)
			firmware_memory->firmware_buf = firmware_buf;
	}

	for (i = 0; i < num_firmware_bufs; i++) {
		if ((firmware_memory != NULL) &&
			(firmware_memory->firmware_buf != NULL) &&
			(firmware_memory->firmware_buf[i] != NULL)) {
			firmware_buf[i] = firmware_memory->firmware_buf[i];
			memset(firmware_buf[i], 0, FIRMWARE_BUF_SIZE);
		} else {
			warn("alloc memory set mode: firmware_buf[]\n");
			firmware_buf[i] = SiiOsCalloc("", FIRMWARE_BUF_SIZE, 0);
			if (NULL == firmware_buf[i]) {
				err("Out of memory\n");
				retval = -1;
				goto file_done;
			}
			if ((firmware_memory != NULL) &&
				(firmware_memory->firmware_buf != NULL)) {
				firmware_memory->firmware_buf[i] =
							firmware_buf[i];
			}
		}
	}

	for (i = 0; i < num_firmware_bufs - 1; i++) {
		memcpy(firmware_buf[i], &fw_data[i*FIRMWARE_BUF_SIZE/4],
						FIRMWARE_BUF_SIZE);
	}
	memcpy(firmware_buf[i], &fw_data[i*FIRMWARE_BUF_SIZE/4],
					firmware_size % FIRMWARE_BUF_SIZE);
#endif

	fw_crc = (uint32_t *)(fw_entry->data + firmware_hdr_size +
				firmware_size);

file_done:

	if (retval < 0)
		goto release;

	if ((NULL == firmware_buf) || (NULL == fw_crc)) {
		retval = -1;
		goto release;
	}

#ifdef FIRMWARE_DATA_MODE_FIRST
	retval = verify_firmware_file(fw_header, firmware_buf,
					num_firmware_bufs_firmware, *fw_crc);
#else
	retval = verify_firmware_file(fw_header, firmware_buf,
					num_firmware_bufs, *fw_crc);
#endif
	if (retval < 0) {
		err("Firmware file not valid, will not be loaded!\n");
		goto release;
	}

	/* Reset board and wait for the BOOT_READY interrupt */
	for (i = 0; i < MAX_BOOT_READY_RETRY; i++) {
		if (sii6400_module_is_exiting())
			goto release;

		dbg("Reset board: attempt %d\n", i + 1);

		sii6400_boot_ready = false;

		/* Set reset high */
		sii_rv = SiiHalReset(true);
		if (SII_STATUS_SUCCESS != sii_rv) {
			err("RESET0 command error: 0x%x\n", sii_rv);
			/*goto release;*/
		}

		msleep(RESET0_INTERVAL);

		/* Set reset low */
		sii_rv = SiiHalReset(false);
		if (SII_STATUS_SUCCESS != sii_rv) {
			err("RESET1 command error: 0x%x\n", sii_rv);
			/*goto release;*/
		}

		/* Wait for the BOOT_READY interrupt */
		error = wait_event_interruptible_timeout(sii6400_interrupt_wait,
			(sii6400_boot_ready),
			msecs_to_jiffies(WAIT_TIME_FOR_BOOT_READY_INTERRUPT));
		if (0 == error)
			continue; /* timeout */

		if (error < 0) {
			retval = (int)SII_STATUS_ERR_FAILED;
			goto release;
		}

		if (sii6400_boot_ready)
			break;
	}
	sii6400_boot_ready = false;
	if (MAX_BOOT_READY_RETRY == i) {
		err("BOOT READY interrupt never seen\n");
		retval = -1;
		goto release;
	}

	/* Set the SRAM_SEL bit */
	retval = SiiDeviceApIntStatusReadWrite(0, SRAM_SEL_BIT, &resp_data);
	if (retval < 0) {
		err("Read/Write AP_CTRL_INT_STATUS command error: 0x%x\n",
		    retval);
		goto done_clear_sram_bit;
	}
	dbg("Read/Write AP_CTRL_INT_STATUS command returned 0x%06x\n",
		resp_data);

	/* Set the PG_WR_START_ADDR to the RAM starting address */
	resp_data = FIRMWARE_ADDR_L;
	retval = SiiDeviceRegWrite(SII_REG_PG_WR_START_ADDR_L, resp_data);
	if (retval < 0) {
		err("Write PG_WR_START_ADDR_L command error: 0x%x\n", retval);
		goto done_clear_sram_bit;
	}

	resp_data = 0;
	retval = SiiDeviceRegRead(SII_REG_PG_WR_START_ADDR_L, &resp_data);
	if (retval < 0) {
		err("Read PG_WR_START_ADDR_L command error: 0x%x\n", retval);
		goto done_clear_sram_bit;
	}
	dbg("Write/Read PG_WR_START_ADDR_L command returned 0x%06x\n",
	    resp_data);

	resp_data = FIRMWARE_ADDR_H;
	retval = SiiDeviceRegWrite(SII_REG_PG_WR_START_ADDR_H, resp_data);
	if (retval < 0) {
		err("Write PG_WR_START_ADDR_H command error: 0x%x\n", retval);
		goto done_clear_sram_bit;
	}

	resp_data = 0;
	retval = SiiDeviceRegRead(SII_REG_PG_WR_START_ADDR_H, &resp_data);
	if (retval < 0) {
		err("Read PG_WR_START_ADDR_H command error: 0x%x\n", retval);
		goto done_clear_sram_bit;
	}
	dbg("Write/Read PG_WR_START_ADDR_H command returned 0x%06x\n",
	    resp_data);

	/* Do the bulk data load */
	for (i = 0; i < num_firmware_bufs; i++) {
		sii_rv = SiiHalBulkTxfr(firmware_buf[i], FIRMWARE_BUF_SIZE);
		if (SII_STATUS_SUCCESS != sii_rv)
			err("Bulk transfer command error: 0x%x\n", sii_rv);
		else
			dbg("Bulk transfer command completed OK\n");
	}

done_clear_sram_bit:
	prev_retval = retval;

	/* Clear the SRAM_SEL bit */
	retval = SiiDeviceApIntStatusReadWrite(SRAM_SEL_BIT, 0, &resp_data);
	if (retval < 0) {
		err("Read/Write AP_CTRL_INT_STATUS command error: 0x%x\n",
		    retval);
		goto release;
	}
	dbg("Read/Write AP_CTRL_INT_STATUS command returned 0x%06x\n",
	    resp_data);

	if (prev_retval < 0)
		goto release;

	/* Set the size of the firmware */
	resp_data = firmware_size;
	retval = SiiDeviceRegWrite(SII_REG_MBOX_00, resp_data);
	if (retval < 0) {
		err("Write MBOX_00 command error: 0x%x\n", retval);
		goto release;
	}

	resp_data = 0;
	retval = SiiDeviceRegRead(SII_REG_MBOX_00, &resp_data);
	if (retval < 0) {
		err("Read MBOX_00 command error: 0x%x\n", retval);
		goto release;
	}
	dbg("Write/Read MB0 command returned 0x%06x\n", resp_data);

	/* Set the firmware starting address */
	resp_data = firmware_start_high;
	retval = SiiDeviceRegWrite(SII_REG_MBOX_01, resp_data);
	if (retval < 0) {
		err("Write MBOX_01 command error: 0x%x\n", retval);
		goto release;
	}

	resp_data = 0;
	retval = SiiDeviceRegRead(SII_REG_MBOX_01, &resp_data);
	if (retval < 0) {
		err("Read MBOX_01 command error: 0x%x\n", retval);
		goto release;
	}
	dbg("Write/Read MB1 command returned 0x%06x\n", resp_data);

	resp_data = firmware_start_low;
	retval = SiiDeviceRegWrite(SII_REG_MBOX_02, resp_data);
	if (retval < 0) {
		err("Write MBOX_02 command error: 0x%x\n", retval);
		goto release;
	}

	resp_data = 0;
	retval = SiiDeviceRegRead(SII_REG_MBOX_02, &resp_data);
	if (retval < 0) {
		err("Read MBOX_02 command error: 0x%x\n", retval);
		goto release;
	}
	dbg("Write/Read MB2 command returned 0x%06x\n", resp_data);

	/* Set the authentication code */
	resp_data = auth_code_3;
	retval = SiiDeviceRegWrite(SII_REG_MBOX_03, resp_data);
	if (retval < 0) {
		err("Write MBOX_03 command error: 0x%x\n", retval);
		goto release;
	}

	resp_data = 0;
	retval = SiiDeviceRegRead(SII_REG_MBOX_03, &resp_data);
	if (retval < 0) {
		err("Read MBOX_03 command error: 0x%x\n", retval);
		goto release;
	}
	dbg("Write/Read MB3 command returned 0x%06x\n", resp_data);

	resp_data = auth_code_2;
	retval = SiiDeviceRegWrite(SII_REG_MBOX_04, resp_data);
	if (retval < 0) {
		err("Write MBOX_04 command error: 0x%x\n", retval);
		goto release;
	}

	resp_data = 0;
	retval = SiiDeviceRegRead(SII_REG_MBOX_04, &resp_data);
	if (retval < 0) {
		err("Read MBOX_04 command error: 0x%x\n", retval);
		goto release;
	}
	dbg("Write/Read MB4 command returned 0x%06x\n", resp_data);

	resp_data = auth_code_1;
	retval = SiiDeviceRegWrite(SII_REG_MBOX_05, resp_data);
	if (retval < 0) {
		err("Write MBOX_05 command error: 0x%x\n", retval);
		goto release;
	}

	resp_data = 0;
	retval = SiiDeviceRegRead(SII_REG_MBOX_05, &resp_data);
	if (retval < 0) {
		err("Read MBOX_05 command error: 0x%x\n", retval);
		goto release;
	}
	dbg("Write/Read MB5 command returned 0x%06x\n", resp_data);

	resp_data = auth_code_0;
	retval = SiiDeviceRegWrite(SII_REG_MBOX_06, resp_data);
	if (retval < 0) {
		err("Write MBOX_06 command error: 0x%x\n", retval);
		goto release;
	}

	resp_data = 0;
	retval = SiiDeviceRegRead(SII_REG_MBOX_06, &resp_data);
	if (retval < 0) {
		err("Read MBOX_06 command error: 0x%x\n", retval);
		goto release;
	}
	dbg("Write/Read MB6 command returned 0x%06x\n", resp_data);

	/* done with load - the caller needs to call HostMsgInit() and
	 * wait for the BOOT_SUCCESS interrupt
	 */
	retval = 0;

release:
	/* release the firmware data */
	release_firmware(fw_entry);

done:
#ifdef FIRMWARE_DATA_MODE_FIRST
	if ((NULL != filp) && !IS_ERR(filp) && (filp_close(filp, NULL)))
		err("Error closing NVS file %s\n", configdata_file);
#endif

	return retval;
}

static int verify_firmware_file(
			struct sii6400_firmware_file_hdr_common *firmware_hdr,
			uint32_t *firmware_buf[], uint32_t num_firmware_bufs,
			uint32_t firmware_crc)
{
	int retval = 0;
	uint32_t calc_header_crc = ~0;
	uint32_t calc_firmware_crc = ~0;
	uint32_t fm_size = 0;
	uint32_t buf_size = 0;
	uint32_t hdr_size = 0;
	uint32_t hdr_crc32 = 0;
	uint32_t i;

	dbg("");

	if ((NULL == firmware_hdr) ||
	    (NULL == firmware_buf)) {
		err("Invalid parameters\n");
		retval = -1;
		goto done;
	}

	retval = get_fw_hdr_property(firmware_hdr,
			(enum fw_hdr_version_type)firmware_hdr->header_version,
			FW_HDR_PRPTY_HDR_LENGTH, (void *)&hdr_size,
			sizeof(hdr_size));
	if (retval < 0) {
		err("Get firmware file header size fail\n");
		goto done;
	}

	retval = get_fw_hdr_property(firmware_hdr,
			(enum fw_hdr_version_type)firmware_hdr->header_version,
			FW_HDR_PRPTY_CRC, (void *)&hdr_crc32,
			sizeof(hdr_crc32));
	if (retval < 0) {
		err("Get firmware file header CRC fail\n");
		goto done;
	}

	calc_header_crc = ~crc32(calc_header_crc, (uint8_t *)firmware_hdr,
			hdr_size - sizeof(uint32_t));

	retval = get_fw_hdr_property(firmware_hdr,
			(enum fw_hdr_version_type)firmware_hdr->header_version,
			FW_HDR_PRPTY_FW_SIZE, (void *)&fm_size,
			sizeof(fm_size));
	if (retval < 0) {
		err("Get firmware size fail\n");
		goto done;
	}

	for (i = 0; i < num_firmware_bufs; i++) {
		buf_size = (FIRMWARE_BUF_SIZE < fm_size) ?
				FIRMWARE_BUF_SIZE : fm_size;
		calc_firmware_crc = crc32(calc_firmware_crc,
					(uint8_t *)firmware_buf[i], buf_size);
		fm_size -= FIRMWARE_BUF_SIZE;
	}
	calc_firmware_crc = ~calc_firmware_crc;

	if (calc_header_crc != hdr_crc32) {
		err("FW header CRC mismatch: expected = 0x%08x\n",
		    hdr_crc32);
		err("FW header CRC mismatch: got = 0x%08x\n",
		    calc_header_crc);
		retval = -1;
	} else {
		dbg("Firmware header CRC matches\n");
	}

	if (calc_firmware_crc != firmware_crc) {
		err("FW body CRC mismatch: expected = 0x%08x, got = 0x%08x\n",
		    firmware_crc, calc_firmware_crc);
		retval = -1;
	} else {
		dbg("Firmware body CRC matches\n");
	}

done:
	return retval;
}

int SiiDeviceFirmwareBoot(const char *firmware_file)
{
	int retval = 0;
	int error = 0;
	uint32_t resp_data = 0;

	dbg("");

	if (NULL == firmware_file) {
		err("Invalid parameter\n");
		retval = -1;
		goto done;
	}

	retval = HostMsgTerm();
	if (retval < 0) {
		err("HostMsg failed to terminate: 0x%x\n", retval);
		goto done;
	}

	retval = SiiDeviceFirmwareLoad(firmware_file);
	if (retval < 0) {
		err("Firmware load failed\n");
		goto done;
	}
	dbg("Firmware load succeeded\n");

	/* Turn off BOOT interrupts */
	retval = SiiDeviceIntEnableReadWrite(ALL_BOOT_INTERRUPT_BITS, 0,
						&resp_data);
	if (retval < 0) {
		err("Error modifying INT_N_ENABLE (off): 0x%x\n", retval);
		goto done;
	}

	/* Set the AP_LD_DONE bit */
	retval = SiiDeviceApIntStatusReadWrite(0, AP_LD_DONE_BIT, &resp_data);
	if (retval < 0) {
		err("Read/Write AP_CTRL_INT_STATUS command error: 0x%x\n",
		    retval);
		goto done;
	}
	dbg("Read/Write AP_CTRL_INT_STATUS command returned 0x%06x\n",
	    resp_data);

	retval = HostMsgInit();
	if (retval < 0) {
		err("HostMsg failed to initialize: 0x%x\n", retval);
		goto done;
	}

	/* Turn on BOOT interrupts */
	retval = SiiDeviceIntEnableReadWrite(0, ALL_BOOT_INTERRUPT_BITS,
						&resp_data);
	if (retval < 0) {
		err("Error modifying INT_N_ENABLE (on): 0x%x\n", retval);
		goto done;
	}

	/* Wait for the BOOT_SUCCESS or BOOT_FAIL interrupt */
	error = wait_event_interruptible_timeout(sii6400_interrupt_wait,
		(sii6400_boot_successful || sii6400_boot_failed),
		msecs_to_jiffies(WAIT_TIME_FOR_BOOT_SUCCESS_FAIL_INTERRUPT));
	if (0 == error) {
		retval = (int)SII_OS_STATUS_TIMEOUT;
		err("Timed out waiting for boot success or failure\n");
		goto done;
	}
	if (error < 0) {
		retval = (int)SII_STATUS_ERR_FAILED;
		goto done;
	}

	if (sii6400_boot_successful)
		retval = 0;
	else
		retval = (int)SII_STATUS_ERR_FAILED;

done:
	sii6400_boot_successful = false;
	sii6400_boot_failed = false;
	return retval;
}

static uint32_t get_max_fw_hdr_size(void)
{
	uint32_t size = 0;

	size = max(sizeof(struct sii6400_firmware_file_hdr_v01),
		   sizeof(struct sii6400_firmware_file_hdr_v02));

	return size;
}

static int get_fw_hdr_property(void *firmware_hdr,
				enum fw_hdr_version_type fw_hdr_version,
				enum fw_hdr_property_type property_id,
				void *fw_hdr_property,
				uint32_t fw_hdr_property_size)
{
	int retval = 0;

	if ((NULL == firmware_hdr) ||
	    (NULL == fw_hdr_property)) {
		err("Invalid parameters\n");
		retval = -1;
		goto done;
	}

	switch (fw_hdr_version) {
	case FW_HDR_VERSION_01:
	{
		struct sii6400_firmware_file_hdr_v01 *fw_hdr_v01;

		fw_hdr_v01 =
			(struct sii6400_firmware_file_hdr_v01 *)firmware_hdr;

		switch (property_id) {
		case FW_HDR_PRPTY_SIG:
			if (fw_hdr_property_size != sizeof(fw_hdr_v01->sig)) {
				err("FW_HDR_PRPTY_SIG: Bad size\n");
				retval = -1;
				goto done;
			}
			memcpy(fw_hdr_property, fw_hdr_v01->sig,
				fw_hdr_property_size);
			break;
		case FW_HDR_PRPTY_HDR_VERSION:
		{
			uint32_t *hdr_version = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v01->header_version)) {
				err("FW_HDR_PRPTY_HDR_VERSION: Bad size\n");
				retval = -1;
				goto done;
			}
			*hdr_version = fw_hdr_v01->header_version;
		}
			break;
		case FW_HDR_PRPTY_HW_ID:
		{
			uint32_t *hw_id = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v01->hardware_id)) {
				err("FW_HDR_PRPTY_HW_ID: Bad size\n");
				retval = -1;
				goto done;
			}
			*hw_id = fw_hdr_v01->hardware_id;
		}
			break;
		case FW_HDR_PRPTY_HDR_LENGTH:
		{
			uint32_t *hdr_length = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v01->header_length)) {
				err("FW_HDR_PRPTY_HDR_LENGTH: Bad size\n");
				retval = -1;
				goto done;
			}
			*hdr_length = fw_hdr_v01->header_length;
		}
			break;
		case FW_HDR_PRPTY_FW_SIZE:
		{
			uint32_t *fw_size = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v01->firmware_size)) {
				err("FW_HDR_PRPTY_FW_SIZE: Bad size\n");
				retval = -1;
				goto done;
			}
			*fw_size = fw_hdr_v01->firmware_size;
		}
			break;
		case FW_HDR_PRPTY_FW_ENTRY:
		{
			uint32_t *fw_entry = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v01->firmware_entry)) {
				err("FW_HDR_PRPTY_FW_ENTRY: Bad size\n");
				retval = -1;
				goto done;
			}
			*fw_entry = fw_hdr_v01->firmware_entry;
		}
			break;
		case FW_HDR_PRPTY_MIC:
		{
			if (fw_hdr_property_size != sizeof(fw_hdr_v01->mic)) {
				err("FW_HDR_PRPTY_MIC: Bad size\n");
				retval = -1;
				goto done;
			}
			memcpy(fw_hdr_property, fw_hdr_v01->mic,
				fw_hdr_property_size);
		}
			break;
		case FW_HDR_PRPTY_FW_VERSION:
		{
			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v01->firmware_version)) {
				err("FW_HDR_PRPTY_FW_VERSION: Bad size\n");
				retval = -1;
				goto done;
			}
			memcpy(fw_hdr_property, fw_hdr_v01->firmware_version,
					fw_hdr_property_size);
		}
			break;
		case FW_HDR_PRPTY_CRC:
		{
			uint32_t *hdr_crc32 = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v01->header_crc32)) {
				err("FW_HDR_PRPTY_CRC: Bad size\n");
				retval = -1;
				goto done;
			}
			*hdr_crc32 = fw_hdr_v01->header_crc32;
		}
			break;
		default:
			err("Invalid property ID\n");
			retval = -1;
			break;
		}
	}
		break;
	case FW_HDR_VERSION_02:
	{
		struct sii6400_firmware_file_hdr_v02 *fw_hdr_v02;

		fw_hdr_v02 =
			(struct sii6400_firmware_file_hdr_v02 *)firmware_hdr;

		switch (property_id) {
		case FW_HDR_PRPTY_SIG:
			if (fw_hdr_property_size != sizeof(fw_hdr_v02->sig)) {
				err("FW_HDR_PRPTY_SIG: Bad size\n");
				retval = -1;
				goto done;
			}
			memcpy(fw_hdr_property, fw_hdr_v02->sig,
				fw_hdr_property_size);
			break;
		case FW_HDR_PRPTY_HDR_VERSION:
		{
			uint32_t *hdr_version = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v02->header_version)) {
				err("FW_HDR_PRPTY_HDR_VERSION: Bad size\n");
				retval = -1;
				goto done;
			}
			*hdr_version = fw_hdr_v02->header_version;
		}
			break;
		case FW_HDR_PRPTY_HW_ID:
		{
			uint32_t *hw_id = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v02->hardware_id)) {
				err("FW_HDR_PRPTY_HW_ID: Bad size\n");
				retval = -1;
				goto done;
			}
			*hw_id = fw_hdr_v02->hardware_id;
		}
			break;
		case FW_HDR_PRPTY_HDR_LENGTH:
		{
			uint32_t *hdr_length = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v02->header_length)) {
				err("FW_HDR_PRPTY_HDR_LENGTH: Bad size\n");
				retval = -1;
				goto done;
			}
			*hdr_length = fw_hdr_v02->header_length;
		}
			break;
		case FW_HDR_PRPTY_FW_SIZE:
		{
			uint32_t *fw_size = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v02->firmware_size)) {
				err("FW_HDR_PRPTY_FW_SIZE: Bad size\n");
				retval = -1;
				goto done;
			}
			*fw_size = fw_hdr_v02->firmware_size;
		}
			break;
		case FW_HDR_PRPTY_FW_ENTRY:
		{
			uint32_t *fw_entry = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v02->firmware_entry)) {
				err("FW_HDR_PRPTY_FW_ENTRY: Bad size\n");
				retval = -1;
				goto done;
			}
			*fw_entry = fw_hdr_v02->firmware_entry;
		}
			break;
		case FW_HDR_PRPTY_MIC:
		{
			if (fw_hdr_property_size != sizeof(fw_hdr_v02->mic)) {
				err("FW_HDR_PRPTY_MIC: Bad size\n");
				retval = -1;
				goto done;
			}
			memcpy(fw_hdr_property, fw_hdr_v02->mic,
					fw_hdr_property_size);
		}
			break;
		case FW_HDR_PRPTY_FW_VERSION:
		{
			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v02->firmware_version)) {
				err("FW_HDR_PRPTY_FW_VERSION: Bad size\n");
				retval = -1;
				goto done;
			}
			memcpy(fw_hdr_property, fw_hdr_v02->firmware_version,
					fw_hdr_property_size);
		}
			break;
		case FW_HDR_PRPTY_CUST_ID:
		{
			uint32_t *cust_id = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v02->customer_id)) {
				err("FW_HDR_PRPTY_CUST_ID: Bad size\n");
				retval = -1;
				goto done;
			}
			*cust_id = fw_hdr_v02->customer_id;
		}
			break;
		case FW_HDR_PRPTY_CRC:
		{
			uint32_t *hdr_crc32 = (uint32_t *)fw_hdr_property;

			if (fw_hdr_property_size !=
					sizeof(fw_hdr_v02->header_crc32)) {
				err("FW_HDR_PRPTY_CRC: Bad size\n");
				retval = -1;
				goto done;
			}
			*hdr_crc32 = fw_hdr_v02->header_crc32;
		}
			break;
		default:
			err("Invalid property ID\n");
			retval = -1;
			break;
		}
	}
		break;
	default:
		err("Invalid firmware header version\n");
		retval = -1;
		break;
	}

done:
	return retval;
}

