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

/*#define LOG_SPI_OP_TIMES*/
/*#define LOG_POLL_TIMES*/
/*#define DEBUG_MUTEXES*/

/*#define USE_INTERRUPT_DISABLE*/
#define USE_DELAYED_WORK_FOR_INTERRUPT
#define REARM_INTERRUPT_ON_DELAY

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/termios.h>

#include "osal.h"
#include "hal.h"
#include "hal_serial.h"
#include "device.h"
#include "sii6400.h"
#include "host_msg_private.h"

#define APP_SERIAL_PORT "/dev/ttyACM0"

/* A mutex to protect the tty_write and tty_read operations on the AP port */
static DEFINE_MUTEX(sii6400_ap_tty_mutex);

/* AP serial tty file */
static struct file *f_tty_ap;

static bool ap_port_exists;

#define AP_CHECK_FOR_INTERRUPT_DELAY	(HZ/10)

struct ap_work {
	struct delayed_work ap_check_for_intr_work;
};

static struct workqueue_struct *ap_check_for_intr_wq;
static struct ap_work *my_ap_work;

static void ap_check_for_interrupt(struct work_struct *work);

static struct workqueue_struct *isr_handler_wq;
#ifdef USE_DELAYED_WORK_FOR_INTERRUPT
#define POLL_INTERRUPTS_DELAY	(2 * HZ)
static struct delayed_work *isr_handler_work;
#else /* USE_DELAYED_WORK_FOR_INTERRUPT */
static struct work_struct *isr_handler_work;
#endif /* USE_DELAYED_WORK_FOR_INTERRUPT */

/* Used to prevent flooding the system log with serial port error messages */
static bool ap_serial_port_error_reported;

static char *ap_serial_port = APP_SERIAL_PORT;	/* App Port name */
module_param(ap_serial_port, charp, S_IRUGO);
MODULE_PARM_DESC(ap_serial_port, "AP serial port name (default: /dev/ttyACM0)");

static unsigned char error_resp[] = {
	SPI_RESULT_START, 'E', 'R', 'R', 'O', 'R', SPI_RESULT_END
};
static unsigned char interrupt_resp[] = {
	SPI_NOTIFICATION_START, 'I', 'N', 'T', 'N', SPI_NOTIFICATION_END
};
static unsigned char bulk_data_resp[] = {
	SPI_RESULT_START, 'O', 'K', 'A', 'Y', SPI_RESULT_END
};

static bool sii6400_in_irq;
#ifdef USE_INTERRUPT_DISABLE
static bool interrupts_enabled = true;
static bool unhandled_interrupts;
#endif /* USE_INTERRUPT_DISABLE */
static bool in_reset = true;
#ifdef REARM_INTERRUPT_ON_DELAY
static bool interrupt_poll_enabled;
#endif /* REARM_INTERRUPT_ON_DELAY */

/* Semaphore to protect access to SPI communications */
#define MAX_WAIT_FOR_SPI_SEMAPHORE	500
static struct SiiOsSemaphore *sii6400_spi_sem;

/* static function prototypes */
static int tty_read(unsigned char *buf, size_t buf_size, struct file *filp,
			int timeout);
static int tty_write(const char *buf, size_t msg_out_len, struct file *filp);

static int send_vcom_cmd(unsigned char *cmd_resp_buf, size_t cmd_buf_size,
				size_t cmd_len);
static int set_vcom_cmd(uint32_t cmd_data, unsigned char *cmd_resp_buf,
				size_t cmd_buf_size, size_t *cmd_len);
static int get_vcom_resp_data(const unsigned char *cmd_resp_buf,
				size_t cmd_buf_size, uint32_t *resp_data);
static int send_vcom_bulk_data_xfer_msg(unsigned char *bulk_data_xfer_msg_buf,
			size_t bulk_data_xfer_msg_buf_size, size_t msg_len);
static int set_vcom_bulk_data_xfer_msg(uint32_t *bulk_data_buf,
					size_t bulk_data_buf_size,
					unsigned char *bulk_data_xfer_msg_buf,
					size_t bulk_data_xfer_msg_buf_size,
					size_t *msg_len);
static int extract_interrupt_from_vcom_resp(unsigned char *resp_buf,
						size_t resp_buf_size);
static int check_for_interrupt_on_ap_port(int timeout);
static void isr_handler(void);
static void isr_work_handler(struct work_struct *work);
static int enable_interrupts_serial(void);
static int disable_interrupts_serial(void);


/*
 * Read a buffer from the VCOM (poll for timeout tenths of msec).
 * Return the number of bytes read (>= 0) or an error (< 0).
 */
static int tty_read(unsigned char *buf, size_t buf_size, struct file *filp,
			int timeout)
{
	int result = -1;

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == filp) {
		/*warn("failed due to unopen port\n");*/
	} else if ((NULL == filp->f_op) || (NULL == filp->f_op->read)) {
		warn("Cannot read from VCOM\n");
	} else {
		mm_segment_t oldfs;

		oldfs = get_fs();
		set_fs(KERNEL_DS);

		if ((NULL != filp->f_op->poll) && (0 != timeout)) {
			struct poll_wqueues table;
			struct timeval start, now;

			do_gettimeofday(&start);
			poll_initwait(&table);
			while (1) {
				long elapsed;
				int mask;

				mask = filp->f_op->poll(filp, &table.pt);
				if (mask & (POLLRDNORM | POLLRDBAND | POLLIN |
						POLLHUP | POLLERR))
					break;
				do_gettimeofday(&now);
				elapsed = (USEC_PER_SEC *
						(now.tv_sec - start.tv_sec) +
						now.tv_usec - start.tv_usec);
				if (elapsed > timeout)
					break;
				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(((timeout - elapsed) *
							HZ) / 10000);
			}
			poll_freewait(&table);

#ifdef LOG_POLL_TIMES
			{
				long elapsed;

				do_gettimeofday(&now);
				elapsed = (USEC_PER_SEC *
						(now.tv_sec - start.tv_sec) +
						now.tv_usec - start.tv_usec);
				dbg("polled for %lu.%06lu sec",
				    elapsed / USEC_PER_SEC,
				    elapsed % USEC_PER_SEC);
			}
#endif /* LOG_POLL_TIMES */
		} else if (NULL == filp->f_op->poll) {
			warn("polling not available!\n");
		}

		filp->f_pos = 0;
		result = filp->f_op->read(filp,
					(unsigned char __user __force *)buf,
					buf_size, &filp->f_pos);
		if (result < 0) {
			if (-EAGAIN != result)
				err("read failed: 0x%x\n", result);
		}

		set_fs(oldfs);
	}

done:
	return result;
}

/*
 * Write a buffer to the VCOM.
 * Return the number of bytes sent (>= 0) or an error (< 0).
 */
static int tty_write(const char *buf, size_t msg_out_len, struct file *filp)
{
	int result = -1;

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == filp) {
		warn("failed due to unopen port\n");
	} else if ((NULL == filp->f_op) || (NULL == filp->f_op->write)) {
		warn("Cannot write to VCOM\n");
	} else {
		mm_segment_t oldfs;

		oldfs = get_fs();
		set_fs(KERNEL_DS);

		if (0 < msg_out_len) {
			filp->f_pos = 0;
			result = filp->f_op->write(filp,
				(const unsigned char __user __force *)buf,
				msg_out_len, &filp->f_pos);
			if (result < 0)
				err("write failed\n");
		}
		set_fs(oldfs);
	}

done:
	return result;
}

/*
 * Write a command message to the AP Serial Port.
 * Read the response from the AP Serial Port and store it in the
 *  command/response buffer.
 * Returns: the number of bytes in the response.
 */
static int send_vcom_cmd(unsigned char *cmd_resp_buf, size_t cmd_buf_size,
				size_t cmd_len)
{
	int rv = 0;
	int bytes_sent = 0;
	int bytes_recvd = 0;
	int tty_read_retry_count = 0;

	/*dbg("");*/

	if (sii6400_module_is_exiting())
		return 0;

	rv = mutex_lock_interruptible(&sii6400_ap_tty_mutex);
	if (rv) {
		warn("lock ap_tty_mutex: signal received: 0x%x\n", rv);
		bytes_recvd = rv;
		goto failed_ap_tty_mutex;
	}

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == cmd_resp_buf) {
		err("NULL pointer sent for required buffer\n");
		goto done;
	}

	/* Send the command */
	bytes_sent = tty_write(cmd_resp_buf, cmd_len, f_tty_ap);
	if (bytes_sent < 0)
		err("tty_write failed: 0x%x\n", bytes_sent);
	else if (bytes_sent < (int)cmd_len)
		warn("tty_write failed to send the complete command\n");

	/* Need some delay here to get it to work */
	/*udelay(VCOM_SPI_DELAY_BEFORE_SERIAL_READ * 1000);*/
	msleep(VCOM_SPI_DELAY_BEFORE_SERIAL_READ);

	tty_read_retry_count = 0;
retry_tty_read:
	bytes_recvd = tty_read(cmd_resp_buf, cmd_buf_size,
			f_tty_ap, POLL_TIME_FOR_AP_SERIAL_READ_REG_RD_WR);
	if (bytes_recvd < 0) {
		if (-EAGAIN == bytes_recvd) {
			if (MAX_TTY_READ_RETRY_COUNT < tty_read_retry_count++) {
				err("tty_read failed: tried %d times\n",
				    tty_read_retry_count);
				goto done;
			}
			dbg("tty_read retry %d\n", tty_read_retry_count);
			msleep(VCOM_SPI_DELAY_AFTER_EAGAIN);
			goto retry_tty_read;
		}
		if (!ap_serial_port_error_reported) {
			ap_serial_port_error_reported = true;
			err("tty_read failed: 0x%x\n", bytes_recvd);
		}
		goto done;
	} else if (0 == bytes_recvd) {
		if (!ap_serial_port_error_reported) {
			ap_serial_port_error_reported = true;
			warn("tty_read failed to read any bytes\n");
		}
		goto done;
	}
	ap_serial_port_error_reported = false;

	if (bytes_recvd < (int)cmd_buf_size)
		*(cmd_resp_buf + bytes_recvd) = 0;

done:
	mutex_unlock(&sii6400_ap_tty_mutex);
failed_ap_tty_mutex:
	return bytes_recvd;
}

/*
 * Form the VCOM command message from the uint32 command and
 *  put it in 'cmd_resp_buf'.
 * Store the length of the command string in 'cmd_len'.
 * Returns: 0 if successful
 *          an error if command string could not be created
 */
static int set_vcom_cmd(uint32_t cmd_data, unsigned char *cmd_resp_buf,
				size_t cmd_buf_size, size_t *cmd_len)
{
	int retval = 0;
	uint8_t i;
	uint8_t *cmd_bytes = (uint8_t *)&cmd_data;

	/*dbg("");*/

	if ((NULL == cmd_resp_buf) || (NULL == cmd_len)) {
		err("NULL pointer sent for required buffer\n");
		retval = -1;
		goto done;
	}

	if (cmd_buf_size < 11) {
		err("VCOM command buffer size error\n");
		retval = -1;
		goto done;
	}

	cmd_resp_buf[0] = SPI_CMD_START_ASCII;
	for (i = 0; i < 4; i++) {
		unsigned char nibble_low = *cmd_bytes & 0x0f;
		unsigned char nibble_high = (*cmd_bytes >> 4) & 0x0f;

		cmd_resp_buf[i * 2 + 1] = (nibble_high < 10) ?
						'0' + nibble_high :
						'A' + nibble_high - 10;
		cmd_resp_buf[i * 2 + 2] = (nibble_low < 10) ?
						'0' + nibble_low :
						'A' + nibble_low - 10;
		cmd_bytes++;
	}
	cmd_resp_buf[9] = SPI_OPERATION_END;
	cmd_resp_buf[10] = '\0';
	*cmd_len = 10;

done:
	return retval;
}

/*
 * Derive the uint32 response from the VCOM response string.
 * Returns: 0 if successful
 *          an error if the response could not be derived
 */
static int get_vcom_resp_data(const unsigned char *cmd_resp_buf,
				size_t cmd_buf_size, uint32_t *resp_data)
{
	int retval = 0;
	uint8_t i;
	uint8_t *resp_bytes = NULL;

	/*dbg("");*/

	if ((NULL == cmd_resp_buf) || (NULL == resp_data)) {
		err("NULL pointer sent for required buffer\n");
		retval = -1;
		goto done;
	}

	if (cmd_buf_size < 11) {
		err("VCOM command buffer size error\n");
		retval = -1;
		goto done;
	}

	/* Check for error response */
	if (0 == memcmp(cmd_resp_buf, error_resp, sizeof(error_resp))) {
		err("Command not sent, ERROR returned\n");
		retval = -1;
		goto done;
	}

	if (SPI_RESULT_START != cmd_resp_buf[0]) {
		err("Expected VCOM response start, but got 0x%0x\n",
		    cmd_resp_buf[0]);
		retval = -1;
		goto done;
	}

	if (SPI_RESULT_END != cmd_resp_buf[9]) {
		err("Expected VCOM response end, but got 0x%0x\n",
		    cmd_resp_buf[9]);
		retval = -1;
		goto done;
	}

	*resp_data = 0;
	resp_bytes = (uint8_t *)resp_data;
	for (i = 0; i < 4; i++) {
		unsigned char nibble_low;
		unsigned char nibble_high;
		char cur_char;

		cur_char = cmd_resp_buf[i * 2 + 1];
		if (('0' <= cur_char) && (cur_char <= '9')) {
			nibble_high = cur_char - '0';
		} else if (('a' <= cur_char) && (cur_char <= 'f')) {
			nibble_high = cur_char - 'a' + 10;
		} else if (('A' <= cur_char) && (cur_char <= 'F')) {
			nibble_high = cur_char - 'A' + 10;
		} else if (0 == cur_char) {
			/* this should not be necessary, but it is! */
			nibble_high = 0;
		} else {
			err("Expected VCOM ASCII hex char, but got 0x%x\n",
			    cur_char);
			retval = -1;
			goto done;
		}

		cur_char = cmd_resp_buf[i * 2 + 2];
		if (('0' <= cur_char) && (cur_char <= '9')) {
			nibble_low = cur_char - '0';
		} else if (('a' <= cur_char) && (cur_char <= 'f')) {
			nibble_low = cur_char - 'a' + 10;
		} else if (('A' <= cur_char) && (cur_char <= 'F')) {
			nibble_low = cur_char - 'A' + 10;
		} else if (0 == cur_char) {
			/* this should not be necessary, but it is! */
			nibble_low = 0;
		} else {
			err("Expected VCOM ASCII hex char, but got 0x%x\n",
			    cur_char);
			retval = -1;
			goto done;
		}

		*resp_bytes = (nibble_high << 4) + nibble_low;
		resp_bytes++;
	}

done:
	return retval;
}

/*
 * Write a bulk data transfer message to the AP Serial Port.
 * Read the response from the AP Serial Port and
 *  store it in the bulk data transfer message buffer.
 * Returns: the number of bytes in the response
 */
static int send_vcom_bulk_data_xfer_msg(unsigned char *bulk_data_xfer_msg_buf,
			size_t bulk_data_xfer_msg_buf_size, size_t msg_len)
{
	int rv = 0;
	int bytes_sent = 0;
	int bytes_recvd = 0;
	int tty_read_retry_count = 0;

	/*dbg("");*/

	if (sii6400_module_is_exiting())
		return 0;

	rv = mutex_lock_interruptible(&sii6400_ap_tty_mutex);
	if (rv) {
		warn("lock ap_tty_mutex: signal received: 0x%x\n", rv);
		bytes_recvd = rv;
		goto failed_ap_tty_mutex;
	}

	if (sii6400_module_is_exiting())
		goto done;

	if (NULL == bulk_data_xfer_msg_buf) {
		err("NULL pointer sent for required buffer\n");
		goto done;
	}

	/* Send the bulk transfer message */
	bytes_sent = tty_write(bulk_data_xfer_msg_buf, msg_len, f_tty_ap);
	if (bytes_sent < 0)
		err("tty_write failed: 0x%x\n", bytes_sent);
	else if (bytes_sent < (int)msg_len)
		warn("tty_write failed to send the complete command\n");

	/* Need some delay here to get it to work */
	/*udelay(VCOM_SPI_DELAY_BEFORE_SERIAL_READ * 1000);*/
	msleep(VCOM_SPI_DELAY_BEFORE_SERIAL_READ);

	tty_read_retry_count = 0;
retry_tty_read:
	bytes_recvd = tty_read(bulk_data_xfer_msg_buf,
				bulk_data_xfer_msg_buf_size,
				f_tty_ap,
				POLL_TIME_FOR_AP_SERIAL_READ_BULK_XFER);
	if (bytes_recvd < 0) {
		if (-EAGAIN == bytes_recvd) {
			if (MAX_TTY_READ_RETRY_COUNT < tty_read_retry_count++) {
				err("tty_read failed: tried %d times\n",
				    tty_read_retry_count);
				goto done;
			}
			dbg("tty_read retry %d\n", tty_read_retry_count);
			msleep(VCOM_SPI_DELAY_AFTER_EAGAIN);
			goto retry_tty_read;
		}
		if (!ap_serial_port_error_reported) {
			ap_serial_port_error_reported = true;
			err("tty_read failed: 0x%x\n", bytes_recvd);
		}
		goto done;
	} else if (0 == bytes_recvd) {
		if (!ap_serial_port_error_reported) {
			ap_serial_port_error_reported = true;
			warn("tty_read failed to read any bytes\n");
		}
		goto done;
	}
	ap_serial_port_error_reported = false;

	if (bytes_recvd < (int)bulk_data_xfer_msg_buf_size)
		*(bulk_data_xfer_msg_buf + bytes_recvd) = 0;

done:
	mutex_unlock(&sii6400_ap_tty_mutex);
failed_ap_tty_mutex:
	return bytes_recvd;
}

/*
 * Form the VCOM bulk data transfer message and put it into the
 *  bulk_data_xfer_msg_buf.
 * Store the length of the message in 'msg_len'.
 * Returns: 0 if successful
 *          an error if the bulk data transfer message could not be created
 */
static int set_vcom_bulk_data_xfer_msg(uint32_t *bulk_data_buf,
					size_t bulk_data_buf_size,
					unsigned char *bulk_data_xfer_msg_buf,
					size_t bulk_data_xfer_msg_buf_size,
					size_t *msg_len)
{
	int retval = 0;
	unsigned int i, j;
	uint8_t *bulk_data_bytes = NULL;
	uint8_t *current_out_ptr = NULL;
	size_t num_bulk_data_longs = bulk_data_buf_size / 3;
	/* size in bytes is always divisible by 12 */

	/*dbg("");*/

	if ((NULL == bulk_data_buf) ||
	    (NULL == bulk_data_xfer_msg_buf) ||
	    (NULL == msg_len)) {
		err("NULL pointer sent for required buffer\n");
		retval = -1;
		goto done;
	}
	bulk_data_bytes = (uint8_t *)bulk_data_buf;
	current_out_ptr = bulk_data_xfer_msg_buf;

	if (bulk_data_xfer_msg_buf_size < bulk_data_buf_size * 2 + 3) {
		err("VCOM bulk data buffer size error\n");
		retval = -1;
		goto done;
	}

	*current_out_ptr++ = SPI_BULK_TXFR_START_BIN;

	for (j = 0; j < num_bulk_data_longs; j++) {
		*current_out_ptr++ = PAGE_WRITE_CMD_WORD;
		for (i = 0; i < 3; i++) {
			if (IS_VCOM_AP_SPECIAL(*bulk_data_bytes))
				*current_out_ptr++ = SPI_ESCAPE;
			*current_out_ptr++ = *bulk_data_bytes++;
		}
	}

	*current_out_ptr++ = SPI_OPERATION_END;
	*msg_len = (size_t)(current_out_ptr - bulk_data_xfer_msg_buf);
	*current_out_ptr++ = 0;

done:
	return retval;
}

/*
 * Look for an interrupt before or after expected response data.
 * If an interrupt is found, call the interrupt handler and remove the interrupt
 *  message from the response buffer.
 * Make sure the resp_buf contents are NULL terminated before returning.
 * Returns: 0 if successful
 *          an error (<0) if the parameters are incorrect.
 */
static int extract_interrupt_from_vcom_resp(unsigned char *resp_buf,
						size_t resp_buf_size)
{
	int retval = 0;
	unsigned char *interrupt_start = NULL;
	unsigned char *interrupt_end = NULL;
	unsigned int i;

	if (NULL == resp_buf) {
		err("Invalid parameter\n");
		retval = -1;
		goto done;
	}

	for (i = 0; i < resp_buf_size; i++) {
		if (!resp_buf[i])
			break;

		switch (resp_buf[i]) {
		case SPI_NOTIFICATION_START:
			if (NULL != interrupt_start)
				err("Too many interrupts: %s\n", resp_buf);
			else
				interrupt_start = &resp_buf[i];
			break;

		case SPI_NOTIFICATION_END:
			if (NULL != interrupt_end)
				err("Too many interrupt ends: %s\n", resp_buf);
			else
				interrupt_end = &resp_buf[i];
			break;

		default:
			break;
		}
	}

	if (NULL != interrupt_start) {
		/* If interrupt is found, extract it
		 * and call interrupt handler. */
		if (0 == memcmp(interrupt_start, interrupt_resp,
						sizeof(interrupt_resp))) {
			/* Interrupt arrived, call ISR and continue */
			isr_handler();
		} else {
			err("Badly formed interrupt: %s\n", resp_buf);
		}

		/* If there is more than the interrupt response in the resp buf,
		 * remove interrupt and return the rest. */
		if ((NULL != interrupt_end) &&
		    (interrupt_start < interrupt_end)) {
			unsigned char *put_chars_here = interrupt_start;
			unsigned char *take_chars_from_here = interrupt_end + 1;
			unsigned char *buf_end = resp_buf + resp_buf_size;
			while (*take_chars_from_here &&
			    (take_chars_from_here < buf_end))
				*put_chars_here++ = *take_chars_from_here++;
			*put_chars_here = 0;
		}
	}

	*(resp_buf + resp_buf_size - 1) = 0;

done:
	return retval;
}

/*
 * Poll the UART for 'timeout' tenths of msec. If a message arrives, make sure
 *  it is an "interrupt occurred" message.
 * Returns: 0 if successful
 *          an error (<0) if the command did not get sent or
 *          an interrupt did not arrive
 */
static int check_for_interrupt_on_ap_port(int timeout)
{
	int retval = -1;
	int bytes_recvd = 0;
	unsigned char *cmd_resp_buf = NULL;

	/*dbg("");*/

	if (sii6400_module_is_exiting())
		goto done;

	cmd_resp_buf = SiiOsCalloc("APIntCheck",
					AP_SERIAL_OUT_IN_BUFFER_SIZE, 0);
	if (NULL == cmd_resp_buf) {
		err("Out of memory\n");
		retval = -ENOMEM;
		goto done;
	}

	if (!mutex_lock_interruptible(&sii6400_ap_tty_mutex)) {
		if (!sii6400_module_is_exiting()) {
			bytes_recvd = tty_read(cmd_resp_buf,
					AP_SERIAL_OUT_IN_BUFFER_SIZE,
					f_tty_ap, timeout);
		}
		mutex_unlock(&sii6400_ap_tty_mutex);
	}
	if (bytes_recvd < 0) {
		if ((-EAGAIN != bytes_recvd) &&
		    (!ap_serial_port_error_reported)) {
			ap_serial_port_error_reported = true;
			err("tty_read failed: 0x%x\n", bytes_recvd);
		}
		retval = bytes_recvd;
		goto done;
	} else if (0 == bytes_recvd) {
		if (!ap_serial_port_error_reported) {
			ap_serial_port_error_reported = true;
			warn("tty_read failed to read any bytes\n");
		}
		retval = -1;
		goto done;
	}
	ap_serial_port_error_reported = false;

	/* look for interrupt and call isr_handler if found */
	(void)extract_interrupt_from_vcom_resp(cmd_resp_buf,
						AP_SERIAL_OUT_IN_BUFFER_SIZE);
	if (*cmd_resp_buf)
		warn("Unexpected data on serial port: %s\n", cmd_resp_buf);

done:
	SiiOsFree(cmd_resp_buf);
	return retval;
}

/*
 * Task that polls the AP Serial Port for an interrupt message. When an
 * interrupt message is sent, the ISR is called.
 * This task rearms itself as long as the workqueue still exists.
 */
static void ap_check_for_interrupt(struct work_struct *work)
{
#if 0
	struct ap_work *ap_work = container_of(work, struct ap_work,
				ap_check_for_interrupt_work.work);
#endif

	if (sii6400_module_is_exiting())
		return;

	/* Check for INTN */
	(void)check_for_interrupt_on_ap_port(POLL_TIME_FOR_INTERRUPT_TASK);

	if (!sii6400_module_is_exiting()) {
		if ((NULL != ap_check_for_intr_wq) &&
		    (NULL != my_ap_work)) {
			/* Queue check for interrupt work */
			if (!queue_delayed_work(ap_check_for_intr_wq,
					&my_ap_work->ap_check_for_intr_work,
					AP_CHECK_FOR_INTERRUPT_DELAY)) {
				warn("AP check for INTR task already queued\n");
			}
		} else {
			warn("ap_check_for_intr workqueue no longer exists\n");
		}
	}
}

/*
 * Callback when an interrupt occurs.
 */
static void isr_handler(void)
{
	dbg("");

	if (!in_reset && !sii6400_in_irq) {
		sii6400_in_irq = true;

		if (NULL != isr_handler_work) {
#ifdef USE_DELAYED_WORK_FOR_INTERRUPT
#ifdef REARM_INTERRUPT_ON_DELAY
			cancel_delayed_work_sync(isr_handler_work);
#endif /* REARM_INTERRUPT_ON_DELAY */
			sii6400_in_irq = false;
			queue_delayed_work(isr_handler_wq, isr_handler_work, 0);
#else /* USE_DELAYED_WORK_FOR_INTERRUPT */
			sii6400_in_irq = false;
			queue_work(isr_handler_wq, isr_handler_work);
#endif /* USE_DELAYED_WORK_FOR_INTERRUPT */
		} else {
			sii6400_in_irq = false;
		}
	}
}

static void isr_work_handler(struct work_struct *work)
{
	if (!in_reset && !sii6400_in_irq) {
		sii6400_in_irq = true;
#ifdef USE_INTERRUPT_DISABLE
		if (interrupts_enabled) {
#endif /* USE_INTERRUPT_DISABLE */
			SiiHalIsr();

#ifdef USE_DELAYED_WORK_FOR_INTERRUPT
#ifdef REARM_INTERRUPT_ON_DELAY
			if (!sii6400_module_is_exiting() &&
			    !in_reset &&
			    interrupt_poll_enabled &&
			    (NULL != isr_handler_work)) {
				if (!queue_delayed_work(isr_handler_wq,
						isr_handler_work,
						POLL_INTERRUPTS_DELAY)) {
					warn("Interrupt task already queued\n");
				}
			}
#endif /* REARM_INTERRUPT_ON_DELAY */
#endif /* USE_DELAYED_WORK_FOR_INTERRUPT */

#ifdef USE_INTERRUPT_DISABLE
			unhandled_interrupts = false;
		} else {
			unhandled_interrupts = true;
			dbg("unhandled!");
		}
#endif /* USE_INTERRUPT_DISABLE */
		sii6400_in_irq = false;
	}
}

/*
 * Enable Sii6400 Interrupts
 * Returns: 0 if successful
 *          -1 if interrupts could not be enabled
 */
static int enable_interrupts_serial(void)
{
#ifdef USE_INTERRUPT_DISABLE
	int retval = -1;

	/*dbg("");*/

	if (!sii6400_in_irq) {
		interrupts_enabled = true;
		/* Handle any interrupts that arrived while interrupts
		 * were disabled */
		if (unhandled_interrupts)
			isr_handler();

		retval = 0;
	}

	return retval;
#else /* USE_INTERRUPT_DISABLE */
	return 0;
#endif /* USE_INTERRUPT_DISABLE */
}

/*
 * Disable Sii6400 Interrupts
 * Returns: 0 if successful
 *          -1 if interrupts could not be disabled
 */
static int disable_interrupts_serial(void)
{
#ifdef USE_INTERRUPT_DISABLE
	int retval = -1;

	/*dbg("");*/

	if (!sii6400_in_irq) {
		interrupts_enabled = false;

		retval = 0;
	}

	return retval;
#else /* USE_INTERRUPT_DISABLE */
	return 0;
#endif /* USE_INTERRUPT_DISABLE */
}

/*
 * Initialize the HAL
 * Returns: SII_STATUS_SUCCESS if successful
 *          SII_STATUS_ERR_FAILED if initialization failed
 */
enum sii_status SiiHalInit(void)
{
	enum sii_status retval = SII_STATUS_SUCCESS;
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;

	dbg("");

	sii_os_rv = SiiOsSemaphoreCreate("spi_semaphore", 1, 1,
						&sii6400_spi_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Unable to create SPI semaphore\n");
		retval = SII_STATUS_ERR_FAILED;
		goto init_failed;
	}

	f_tty_ap = NULL;
	ap_port_exists = false;
	f_tty_ap = filp_open(ap_serial_port,
			O_RDWR | O_NOCTTY | O_NDELAY, 0666);
	if (IS_ERR(f_tty_ap)) {
		warn("Could not open AP Serial Port: 0x%x\n",
		     (unsigned int)PTR_ERR(f_tty_ap));
		f_tty_ap = NULL;
	} else {
		ap_port_exists = true;
	}

	if (ap_port_exists) {
		ap_check_for_intr_wq = alloc_workqueue("ap_check_for_intr_wq",
				WQ_HIGHPRI | WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
		if (NULL == ap_check_for_intr_wq) {
			err("ap_check_for_intr_wq create failed\n");
			retval = SII_STATUS_ERR_FAILED;
			goto init_failed;
		}

		my_ap_work = SiiOsCalloc("AP_Work", sizeof(struct ap_work), 0);
		if (NULL == my_ap_work) {
			err("Out of memory\n");
			retval = SII_STATUS_ERR_FAILED;
			goto init_failed;
		}

		INIT_DELAYED_WORK(&my_ap_work->ap_check_for_intr_work,
						ap_check_for_interrupt);

		/* Queue check for interrupt work */
		if (!queue_delayed_work(ap_check_for_intr_wq,
					&my_ap_work->ap_check_for_intr_work,
					AP_CHECK_FOR_INTERRUPT_DELAY)) {
			warn("AP check for interrupt task already queued\n");
		}
	}

	isr_handler_wq = alloc_workqueue("isr_handler_wq",
						WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (NULL == isr_handler_wq) {
		err("isr_handler_wq create failed\n");
		retval = SII_STATUS_ERR_FAILED;
		goto init_failed;
	}

#ifdef USE_DELAYED_WORK_FOR_INTERRUPT
	isr_handler_work = SiiOsCalloc("ISR_Work",
					sizeof(struct delayed_work), 0);
	if (NULL == isr_handler_work) {
		err("Out of memory\n");
		retval = SII_STATUS_ERR_FAILED;
		goto init_failed;
	}

	INIT_DELAYED_WORK(isr_handler_work, isr_work_handler);
#else /* USE_DELAYED_WORK_FOR_INTERRUPT */
	isr_handler_work = SiiOsCalloc("ISR_Work",
					sizeof(struct work_struct), 0);
	if (NULL == isr_handler_work) {
		err("Out of memory\n");
		retval = SII_STATUS_ERR_FAILED;
		goto init_failed;
	}

	INIT_WORK(isr_handler_work, isr_work_handler);
#endif /* USE_DELAYED_WORK_FOR_INTERRUPT */

	return SII_STATUS_SUCCESS;

init_failed:
	SiiHalTerm();
	return retval;
}

/*
 * Terminate the HAL
 */
void SiiHalTerm(void)
{
	dbg("");

	if (!mutex_lock_interruptible(&sii6400_ap_tty_mutex)) {
		if (NULL != f_tty_ap) {
			if (filp_close(f_tty_ap, NULL))
				err("Error closing %s\n", ap_serial_port);
			f_tty_ap = NULL;
		}
		mutex_unlock(&sii6400_ap_tty_mutex);
	}

	if (NULL != my_ap_work) {
		/* ap_check_for_interrupt rearms itself, so explicitly stop it
		 * before destroying the workqueue. */
		cancel_delayed_work_sync(&my_ap_work->ap_check_for_intr_work);
		SiiOsFree(my_ap_work);
		my_ap_work = NULL;
	}
	if (NULL != ap_check_for_intr_wq) {
		destroy_workqueue(ap_check_for_intr_wq);
		ap_check_for_intr_wq = NULL;
	}

	if (NULL != isr_handler_work) {
#ifdef USE_DELAYED_WORK_FOR_INTERRUPT
		cancel_delayed_work_sync(isr_handler_work);
#else /* USE_DELAYED_WORK_FOR_INTERRUPT */
		cancel_work_sync(isr_handler_work);
#endif /* USE_DELAYED_WORK_FOR_INTERRUPT */
		SiiOsFree(isr_handler_work);
		isr_handler_work = NULL;
	}
	if (NULL != isr_handler_wq) {
		destroy_workqueue(isr_handler_wq);
		isr_handler_wq = NULL;
	}

	if (NULL != sii6400_spi_sem) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsSemaphoreDelete(sii6400_spi_sem)) {
			err("Failed to delete SPI semaphore\n");
		}
		sii6400_spi_sem = NULL;
	}
}

/*
 * Issue a SPI operation to the device.
 * The uint32 command (in 'data') will not be checked for correct data.
 * The uint32 response will be set in 'pRetData' (if not NULL).
 * Returns: SII_STATUS_SUCCESS if successful
 *          SII_STATUS_ERR_FAILED an error occurred
 */
enum sii_status SiiHalSpiOp(uint32_t *pTxData, uint32_t *pRxData,
				uint32_t size)
{
	enum sii_status retStatus = SII_STATUS_SUCCESS;
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;
	int rv = 0;
	uint32_t retData = 0;
	unsigned char *cmd_resp_buf = NULL;
	size_t cmd_len = 0;
	uint32_t i;
#ifdef LOG_SPI_OP_TIMES
	struct SiiOsTime start_time, end_time, timediff;
#endif /* LOG_SPI_OP_TIMES */

	/*dbg("");*/

	if (NULL == pTxData || size == 0) {
		err("no input SPI data\n");
		retStatus = SII_STATUS_ERR_FAILED;
		goto done;
	}

	if (in_reset) {
		dbg("SPI Op does not work in_reset\n");
		retStatus = SII_STATUS_ERR_FAILED;
		goto done;
	}

	cmd_resp_buf = SiiOsCalloc("SPIOp", AP_SERIAL_OUT_IN_BUFFER_SIZE, 0);
	if (NULL == cmd_resp_buf) {
		err("Out of memory\n");
		retStatus = SII_STATUS_ERR_FAILED;
		goto done;
	}

	(void)disable_interrupts_serial();

	sii_os_rv = SiiOsSemaphoreTake(sii6400_spi_sem,
					MAX_WAIT_FOR_SPI_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not take SPI semaphore: %x\n", sii_os_rv);
		retStatus = SII_STATUS_ERR_FAILED;
		goto done_enable_ints;
	}

#ifdef LOG_SPI_OP_TIMES
	SiiOsGetTimeCurrent(&start_time);
#endif /* LOG_SPI_OP_TIMES */

	for (i = 0; i < size; i++) {
		/* Convert command data uint32_t into VCOM SPI command buffer */
		rv = set_vcom_cmd(pTxData[i], cmd_resp_buf,
					AP_SERIAL_OUT_IN_BUFFER_SIZE, &cmd_len);
		if (rv < 0) {
			err("Could not get VCOM command buffer\n");
			retStatus = SII_STATUS_ERR_FAILED;
			goto done_give_sem;
		}

		/* Send command and receive response */
		rv = send_vcom_cmd(cmd_resp_buf,
					AP_SERIAL_OUT_IN_BUFFER_SIZE, cmd_len);
		if (rv < 0) {
			err("Could not send VCOM command\n");
			retStatus = SII_STATUS_ERR_FAILED;
			goto done_give_sem;
		}

		/* look for interrupt before or after expected response data. */
		(void)extract_interrupt_from_vcom_resp(cmd_resp_buf,
						AP_SERIAL_OUT_IN_BUFFER_SIZE);

		/* Convert VCOM SPI response buffer into return data uint32 */
		rv = get_vcom_resp_data(cmd_resp_buf,
					AP_SERIAL_OUT_IN_BUFFER_SIZE, &retData);
		if (rv < 0) {
			err("Could not get VCOM command response\n");
			retStatus = SII_STATUS_ERR_FAILED;
			goto done_give_sem;
		}

		if (NULL != pRxData)
			*(pRxData + i) = retData;
	}

#ifdef LOG_SPI_OP_TIMES
	SiiOsGetTimeCurrent(&end_time);
	timediff.time = timespec_sub(end_time.time, start_time.time);
	dbg("write/read time: %lld nsec\n",
	    ((int64_t)timediff.time.tv_sec * NSEC_PER_SEC) +
					timediff.time.tv_nsec);
#endif /* LOG_SPI_OP_TIMES */

done_give_sem:
	sii_os_rv = SiiOsSemaphoreGive(sii6400_spi_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv)
		err("Could not release SPI semaphore: %x\n", sii_os_rv);
done_enable_ints:
	(void)enable_interrupts_serial();
done:
	SiiOsFree(cmd_resp_buf);
	return retStatus;
}

/*
 * Send a reset command.
 * Input: true = put chip in reset (RESET0); false = release reset (RESET1)
 * Returns: SII_STATUS_SUCCESS if successful
 *          SII_STATUS_ERR_FAILED an error occurred
 */
enum sii_status SiiHalReset(bool reset)
{
	enum sii_status retStatus = SII_STATUS_ERR_FAILED;
	int rv = 0;
	unsigned char *cmd_resp_buf = NULL;
	size_t cmd_len = 0;

	dbg("%s reset\n", reset ? "in" : "out of");

	cmd_resp_buf = SiiOsCalloc("SPIReset", AP_SERIAL_OUT_IN_BUFFER_SIZE, 0);
	if (NULL == cmd_resp_buf) {
		err("Out of memory\n");
		goto done;
	}

	/* Put reset(0 or 1) command into buffer */
	if (reset)
		cmd_resp_buf[0] = SPI_RESET0;
	else
		cmd_resp_buf[0] = SPI_RESET1;
	cmd_len = 1;

	/* Send command and receive response */
	rv = send_vcom_cmd(cmd_resp_buf, AP_SERIAL_OUT_IN_BUFFER_SIZE, cmd_len);
	if (rv < 0) {
		err("Could not send VCOM command\n");
		goto done;
	}

	/* Look for RESET_ACK response */
	if (SPI_RESET_ACK != cmd_resp_buf[0]) {
		err("Expected VCOM response RESET_ACK, but got 0x%0x\n",
		    cmd_resp_buf[0]);
		goto done;
	}
	retStatus = SII_STATUS_SUCCESS;
	in_reset = reset;
#ifdef REARM_INTERRUPT_ON_DELAY
	if (in_reset)
		interrupt_poll_enabled = false;
#endif /* REARM_INTERRUPT_ON_DELAY */

done:
	SiiOsFree(cmd_resp_buf);
	return retStatus;
}

/*
 * Send a Bulk transfer.
 * Returns: SII_STATUS_SUCCESS if successful
 *          SII_STATUS_ERR_INVALID_PARAM if pBuffer is NULL or
 *					    size is not a multiple of 12
 *          SII_STATUS_ERR_FAILED an error occurred
 */
enum sii_status SiiHalBulkTxfr(uint32_t *pBuffer, uint32_t size)
{
	enum sii_status retStatus = SII_STATUS_SUCCESS;
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;
	int rv = 0;
	unsigned char *bulk_data_xfer_msg_buf = NULL;
	size_t bulk_data_xfer_msg_buf_size = 0;
	size_t msg_len = 0;
	uint32_t *next = pBuffer;
	uint32_t size_left = 0;
	/* The max_bytes_sent_per_page calculation is nasty. Take 3 bytes from
	 * the maximum size, then expect to send 8 ascii bytes per 3 bytes of
	 * data. Need to include a 1-byte PAGE_WRITE_CMD_WORD value for every
	 * 3 bytes of data, then convert each byte to two ascii characters. */
	uint32_t max_bytes_sent_per_page =
			((((BULK_DATA_BUF_SIZE - 3) * 3) / 8) / 12) * 12;
	struct SiiOsTime start_time, end_time;

	dbg("");

	if (NULL == pBuffer) {
		err("NULL bulk data buffer\n");
		retStatus = SII_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (0 != size % 12) {
		err("Bulk data size %d not a multiple of 12\n", size);
		retStatus = SII_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (in_reset) {
		dbg("Bulk Tx does not work in_reset\n");
		retStatus = SII_STATUS_ERR_FAILED;
		goto done;
	}

	bulk_data_xfer_msg_buf_size = BULK_DATA_BUF_SIZE;
	bulk_data_xfer_msg_buf = SiiOsCalloc("SPIBulk", BULK_DATA_BUF_SIZE, 0);
	if (NULL == bulk_data_xfer_msg_buf) {
		err("Out of memory\n");
		retStatus = SII_STATUS_ERR_FAILED;
		goto done;
	}

	dbg("total number bytes to be sent = %u\n", size);
	dbg("max_bytes_sent_per_page = %u\n", max_bytes_sent_per_page);
	dbg("bulk_data_xfer_msg_buf_size = %u\n",
		(uint32_t)bulk_data_xfer_msg_buf_size);

	(void)disable_interrupts_serial();

	sii_os_rv = SiiOsSemaphoreTake(sii6400_spi_sem,
					MAX_WAIT_FOR_SPI_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not take SPI semaphore: %x\n", sii_os_rv);
		retStatus = SII_STATUS_ERR_FAILED;
		goto done_enable_ints;
	}

	SiiOsGetTimeCurrent(&start_time);

	next = pBuffer;
	size_left = size;
	while (0 != size_left) {
		uint32_t bytes_sent = 0;
		bytes_sent = (size_left < max_bytes_sent_per_page) ?
				size_left : max_bytes_sent_per_page;

		/* Convert bulk data into VCOM SPI bulk data transfer message */
		rv = set_vcom_bulk_data_xfer_msg(next, bytes_sent,
						bulk_data_xfer_msg_buf,
						bulk_data_xfer_msg_buf_size,
						&msg_len);
		if (rv < 0) {
			err("Could not set VCOM bulk data buffer\n");
			retStatus = SII_STATUS_ERR_FAILED;
			goto done_give_sem;
		}

		/* Send bulk data transfer message and receive response */
		rv = send_vcom_bulk_data_xfer_msg(bulk_data_xfer_msg_buf,
						bulk_data_xfer_msg_buf_size,
						msg_len);
		if (rv < 0) {
			err("Could not send VCOM bulk data buffer\n");
			retStatus = SII_STATUS_ERR_FAILED;
			goto done_give_sem;
		}

		/* Verify that the response is correct */
		if (0 != memcmp(bulk_data_xfer_msg_buf, bulk_data_resp,
					sizeof(bulk_data_resp))) {
			err("Bulk Transfer failed to be sent\n");
			retStatus = SII_STATUS_ERR_FAILED;
			goto done_give_sem;
		}

		next += (bytes_sent / 4);
		if (bytes_sent < size_left)
			size_left -= bytes_sent;
		else
			size_left = 0;
		dbg("bytes_sent = %u, msg_len = %u, size_left = %u\n",
		    bytes_sent, (uint32_t)msg_len, size_left);
	}

	SiiOsGetTimeCurrent(&end_time);

	dbg("xfer time: %lld msec\n",
	    SiiOsGetTimeDifferenceMs(&end_time, &start_time));

#ifdef REARM_INTERRUPT_ON_DELAY
	interrupt_poll_enabled = true;
#endif /* REARM_INTERRUPT_ON_DELAY */

done_give_sem:
	sii_os_rv = SiiOsSemaphoreGive(sii6400_spi_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv)
		err("Could not release SPI semaphore: %x\n", sii_os_rv);
done_enable_ints:
	(void)enable_interrupts_serial();
done:
	SiiOsFree(bulk_data_xfer_msg_buf);
	return retStatus;
}

/*
 * Called when all interrupts have been serviced.
 */
void SiiHalIsrDone(void)
{
	/*dbg("");*/
}

