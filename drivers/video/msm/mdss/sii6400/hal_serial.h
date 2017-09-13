/*
  SiI6400 Linux Driver

  Hardware Abstraction Layer (HAL) implementation for PC using serial connection

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

#ifndef _HAL_SERIAL_H
#define _HAL_SERIAL_H

#define USB_SERIAL_AP_MSG_FIFO_SIZE	4096
#define USB_SERIAL_AP_MSG_FIFO_MAX	128
#define AP_SERIAL_OUT_IN_BUFFER_SIZE	128

#define ASCII_TEXT_LINE_BUFFER_SIZE	140

#define MAX_TTY_READ_RETRY_COUNT	5

/* These values are in msec */
#define VCOM_SPI_DELAY_BEFORE_SERIAL_READ	1
#define VCOM_SPI_DELAY_AFTER_EAGAIN		100

/* These values are in tenths of msec */
#define POLL_TIME_FOR_INTERRUPT_TASK		10
#define POLL_TIME_FOR_AP_SERIAL_READ_REG_RD_WR	25000
#define POLL_TIME_FOR_AP_SERIAL_READ_BULK_XFER	100000

/* VCOM SPI command protocol tokens */
#define SPI_BULK_TXFR_START_ASCII	0x02
#define SPI_BULK_TXFR_START_BIN		0x01
#define SPI_CMD_START_ASCII		0x09
#define SPI_CMD_START_BIN		0x0b
#define SPI_OPERATION_END		0x0d
#define SPI_ESCAPE			0x1b
#define SPI_RESET0			0x13
#define SPI_RESET1			0x11

#define IS_VCOM_AP_SPECIAL(x)	((SPI_BULK_TXFR_START_ASCII == (x)) || \
				 (SPI_BULK_TXFR_START_BIN == (x)) || \
				 (SPI_CMD_START_ASCII == (x)) || \
				 (SPI_CMD_START_BIN == (x)) || \
				 (SPI_OPERATION_END == (x)) || \
				 (SPI_ESCAPE == (x)) || \
				 (SPI_RESET0 == (x)) || \
				 (SPI_RESET1 == (x)))

#define SPI_RESULT_START		0x0e
#define SPI_RESULT_END			0x0f
#define SPI_RESET_ACK			0x06
#define SPI_NOTIFICATION_START		0x05
#define SPI_NOTIFICATION_END		0x04

#define MAX_RESPONSE_LOOP_CNT		3

#define BULK_DATA_BUF_SIZE		(PAGE_SIZE)

#endif /* !_HAL_SERIAL_H */

