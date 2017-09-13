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

#ifndef _DEVICE_H
#define _DEVICE_H

#include "sii_common.h"

#define AP_LD_DONE_BIT			0x000004
#define SRAM_SEL_BIT			0x000001
#define AP_HOST_SUPPORT_BIT		0x000010
#define AP_HOST_READY_BIT		0x000020

#define BOOT_FAIL_INTERRUPT_BIT		0x000040
#define BOOT_SUCCESS_INTERRUPT_BIT	0x000020
#define BOOT_READY_INTERRUPT_BIT	0x000010
#define FATAL_ERROR_INTERRUPT_BIT	0x000008
#define ALL_BOOT_INTERRUPT_BITS		(BOOT_FAIL_INTERRUPT_BIT | \
					 BOOT_SUCCESS_INTERRUPT_BIT | \
					 BOOT_READY_INTERRUPT_BIT | \
					 FATAL_ERROR_INTERRUPT_BIT)

#define ALL_INTERRUPT_BITS		0x00FFFFFF

#define FIRMWARE_ADDR_L			0x000000
#define FIRMWARE_ADDR_H			0x012090

#define NOP_CMD_WORD			0x00000003
#define ECHO_CMD_WORD			0x00000007
#define PAGE_WRITE_CMD_WORD		0x0000000B
#define WR_AP_CTRL_INT_STATUS_CMD_WORD	0x00000004
#define RD_AP_CTRL_INT_STATUS_CMD_WORD	0x00000006
#define WR_INT_N_ENABLE_CMD_WORD	0x00000008
#define RD_INT_N_ENABLE_CMD_WORD	0x0000000A
#define WR_INT_N_STATUS_CMD_WORD	0x0000000C
#define RD_INT_N_STATUS_CMD_WORD	0x0000000E
#define WR_PG_WR_START_ADDR_L_CMD_WORD	0x00000010
#define RD_PG_WR_START_ADDR_L_CMD_WORD	0x00000012
#define WR_PG_WR_START_ADDR_H_CMD_WORD	0x00000014
#define RD_PG_WR_START_ADDR_H_CMD_WORD	0x00000016
#define WR_MB0_CMD_WORD			0x00000028
#define RD_MB0_CMD_WORD			0x0000002A

#define WR_MB_CMD_WORD(mb_num)		(WR_MB0_CMD_WORD + (mb_num << 2))
#define RD_MB_CMD_WORD(mb_num)		(RD_MB0_CMD_WORD + (mb_num << 2))

/* These values are in msec */
#define RESET0_INTERVAL					100
#define WAIT_TIME_FOR_BOOT_READY_INTERRUPT		1500
#define WAIT_TIME_FOR_BOOT_SUCCESS_INTERRUPT		15000
#define WAIT_TIME_FOR_BOOT_SUCCESS_FAIL_INTERRUPT	150000

#define MAX_BOOT_READY_RETRY		100
#define MAX_REBOOT_RETRY		100

/* Optimal if this is divisible by 12 */
#define FIRMWARE_BUF_SIZE		(PAGE_SIZE * 3)

#define	FIRMWARE_DATA_MODE_FIRST
#ifdef	FIRMWARE_DATA_MODE_FIRST
/* Max device memory is 704KB */
#define MAX_DEVICE_MEMORY	0xAF400//0x0B0000
/* Max firmware size is 704KB-8KB data = 696KB */
#define DATA_FILE_START_ADDR	0xAD400//0x0AE000

#endif

struct sii6400_firmware_file_hdr_common {
	char sig[8];			/* Magic identifier */
	uint32_t header_version;	/* Version of this struct */
};

struct sii6400_firmware_file_hdr_v01 {
	char sig[8];			/* Magic identifier */
	uint32_t header_version;	/* Version of this struct */
	uint32_t hardware_id;		/* 3 = Gen3, 4 = B&A */
	uint32_t header_length;		/* Size of header in bytes */
	uint32_t firmware_size;		/* Size of fw in bytes (no header) */
	uint32_t firmware_entry;	/* Firmware starting address */
	uint8_t mic[8];			/* Authentication code */
	char firmware_version[128];	/* Firmware version */
	uint32_t header_crc32;		/* Header CRC */
};

struct sii6400_firmware_file_hdr_v02 {
	char sig[8];			/* Magic identifier */
	uint32_t header_version;	/* Version of this struct */
	uint32_t hardware_id;		/* 3 = Gen3, 4 = B&A */
	uint32_t header_length;		/* Size of header in bytes */
	uint32_t firmware_size;		/* Size of fw in bytes (no header) */
	uint32_t firmware_entry;	/* Firmware starting address */
	uint8_t mic[8];			/* Authentication code */
	char firmware_version[128];	/* Firmware version */
	uint32_t customer_id;		/* Customer ID */
	uint32_t header_crc32;		/* Header CRC */
};

/*
 * SII6400 SPI registers.
 * Mailbox register addresses are all offset from mailbox 0.
 */
enum sii_spi_reg_num {
	SII_REG_AP_CTRL_INT_EN	= 0,
	SII_REG_AP_CTRL_INT_STATUS,
	SII_REG_INT_N_ENABLE,
	SII_REG_INT_N_STATUS,
	SII_REG_PG_WR_START_ADDR_L,
	SII_REG_PG_WR_START_ADDR_H,
	SII_REG_APB_ADDR_L,
	SII_REG_APB_ADDR_H,
	SII_REG_APB_WDATA_L,
	SII_REG_APB_WDATA_H,
	SII_REG_MBOX_00,
	SII_REG_MBOX_01,
	SII_REG_MBOX_02,
	SII_REG_MBOX_03,
	SII_REG_MBOX_04,
	SII_REG_MBOX_05,
	SII_REG_MBOX_06,
	SII_REG_MBOX_07,
	SII_REG_MBOX_08,
	SII_REG_MBOX_09,
	SII_REG_MBOX_10,
	SII_REG_MBOX_11,
	SII_REG_MBOX_12,
	SII_REG_MBOX_13,
	SII_REG_MBOX_14,
	SII_REG_MBOX_15,
	SII_REG_MBOX_16,
	SII_REG_MBOX_17,
	SII_REG_MBOX_18,
	SII_REG_MBOX_19,
	SII_REG_MBOX_20,
	SII_REG_MBOX_21,
	SII_REG_MBOX_22,
	SII_REG_MBOX_23,
	SII_REG_MBOX_24,
	SII_REG_MBOX_25,
	SII_REG_MBOX_26,
	SII_REG_MBOX_27,
	SII_REG_MBOX_28,
	SII_REG_MBOX_29,
	SII_REG_MBOX_30,
	SII_REG_MBOX_31,
	SII_REG_MBOX_32,
	SII_REG_MBOX_33,
	SII_REG_MBOX_34,
	SII_REG_MBOX_35,
	SII_REG_MBOX_36,
	SII_REG_MBOX_37,
	SII_REG_MBOX_38,
	SII_REG_MBOX_39,
	SII_REG_MBOX_40,
	SII_REG_MBOX_41,
	SII_REG_MBOX_42,
	SII_REG_MBOX_43,
	SII_REG_MBOX_44,
	SII_REG_MBOX_45,
	SII_REG_MBOX_46,
	SII_REG_MBOX_47,
	SII_REG_MBOX_48,
	SII_REG_MBOX_49,
	SII_REG_MBOX_50,
	SII_REG_MBOX_51,
	SII_REG_MBOX_52,
	SII_REG_MBOX_53,
	SII_REG_MAXRANGE		/* guard value */
};

extern wait_queue_head_t sii6400_interrupt_wait;

extern bool sii6400_boot_ready;
extern bool sii6400_boot_failed;
extern bool sii6400_boot_successful;


enum sii_status SiiDeviceInit(void);
void SiiDeviceTerm(void);
int SiiDeviceRegWrite(enum sii_spi_reg_num myRegister, uint32_t myValue);
int SiiDeviceRegRead(enum sii_spi_reg_num myRegister, uint32_t *pRetData);
int SiiDeviceIntEnableReadWrite(uint32_t clearMask, uint32_t setMask,
					uint32_t *prevVal);
int SiiDeviceApIntStatusReadWrite(uint32_t clearMask, uint32_t setMask,
					uint32_t *prevVal);
int SiiDeviceFirmwareBoot(const char *firmware_file);

#endif /* _DEVICE_H */

