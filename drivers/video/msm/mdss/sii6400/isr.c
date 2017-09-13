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

#include "osal.h"
#include "device.h"
#include "host_msg_private.h"
#include "wihd_sm.h"
#include "mhl_sm.h"
#include "diag_sm.h"

#define MAX_REENABLE_INTERRUPTS_RETRIES		3

bool sii6400_boot_ready;
bool sii6400_boot_failed;
bool sii6400_boot_successful;


static void FatalErrorIsrHandler(void);
static void BootReadyIsrHandler(void);
static void BootSuccessIsrHandler(void);
static void BootFailIsrHandler(void);


/*
 * WatchDog Timer has detected that a Fatal error has occurred.
 */
static void FatalErrorIsrHandler(void)
{
	enum sii_status sii_rv = SII_STATUS_SUCCESS;

	err("FATAL_ERROR interrupt received\n");

	/* exit state machines */
	sii6400_wihd_sm_disable();
	sii6400_mhl_sm_disable();
	sii6400_diag_sm_disable();

	/* Put the device in "reset" */
	sii_rv = SiiHalReset(true);
	if (SII_STATUS_SUCCESS != sii_rv) {
		err("Reset failed\n");
	} else {
		struct sii6400_device_info *devinfo = get_sii6400_devinfo();
		if (NULL == devinfo) {
			err("Invalid devinfo pointer\n");
		} else {
			char event_data[MAX_DRIVER_MODE_LEN + 4] = {0};
			int rv = 0;

			/* Change the mode to "off" */
			devinfo->mode = SII6400_MODE_OFF;
			dbg("Mode changed to off\n");

			rv = scnprintf(event_data, MAX_DRIVER_MODE_LEN + 4,
					"\"off\"");
			if (0 < rv) {
				(void)send_sii6400_uevent(devinfo->device,
						DEVICE_EVENT, MODE_CHANGE_EVENT,
						event_data);
			}
		}
	}
}

/*
 * The Board is now initialized and ready to run the firmware.
 */
static void BootReadyIsrHandler(void)
{
	dbg("");
	sii6400_boot_ready = true;
}

/*
 * The firmware has been loaded and is running.
 */
static void BootSuccessIsrHandler(void)
{
	dbg("");
	sii6400_boot_successful = true;
}

/*
 * Either the firmware failed to load or failed to run.
 */
static void BootFailIsrHandler(void)
{
	dbg("");
	sii6400_boot_failed = true;
}

/*
 * Read the current interrupt status and mask level and call the appropriate
 * handlers.
 */
void SiiHalIsr(void)
{
	int retStatus = 0;
	uint32_t currentIsr = 0;
	uint32_t currentMask = 0;
	uint32_t pendingInts = 0;
	uint32_t hostBits = 0;
	int i;

	/*dbg("");*/

	/*
	 * Find out which interrupt sources are currently enabled and
	 * disable all interrupt sources while in the ISR.
	 */
	retStatus = SiiDeviceIntEnableReadWrite(ALL_INTERRUPT_BITS, 0,
						&currentMask);
	if (retStatus < 0) {
		err("Error modifying INT_N_ENABLE (off): 0x%x\n", retStatus);
		goto done_no_reenable;
	}

	/* Read the current interrupt status and mask value. */
	retStatus = SiiDeviceRegRead(SII_REG_INT_N_STATUS, &currentIsr);
	if (retStatus < 0) {
		err("Error reading INT_N_STATUS: 0x%x\n", retStatus);
		goto done;
	}

	/* Valid interrupts == active, unmasked, and known to handler. */
	pendingInts = currentMask & currentIsr;

	/* Get the Host protocol interrupt bits (subset of pending). */
	hostBits = FIELD_EXTRACT(pendingInts,
				REG_GLUE_AP_CTRL_INT_STATUS_ADDR_HOST_MSG);

	/* Clear all unhandled interrupts. */
	retStatus = SiiDeviceRegWrite(SII_REG_INT_N_STATUS,
					(currentIsr & ~currentMask));
	if (retStatus < 0) {
		err("Error writing INT_N_STATUS: 0x%x\n", retStatus);
		goto done;
	}

	/* Remain in loop until all pending interrupts have been handled. */
	while (pendingInts & currentMask) {
		/* Clear pending interrupts. */
		retStatus = SiiDeviceRegWrite(SII_REG_INT_N_STATUS,
						pendingInts);
		if (retStatus < 0) {
			err("Error writing INT_N_STATUS: 0x%x\n", retStatus);
			goto done;
		}

		/* Handle pending interrupts... */

		if (pendingInts & FATAL_ERROR_INTERRUPT_BIT)
			FatalErrorIsrHandler();

		if (pendingInts & BOOT_READY_INTERRUPT_BIT)
			BootReadyIsrHandler();

		if (pendingInts & BOOT_SUCCESS_INTERRUPT_BIT)
			BootSuccessIsrHandler();

		if (pendingInts & BOOT_FAIL_INTERRUPT_BIT)
			BootFailIsrHandler();

		if (hostBits & HOSTMSG_SW_INT_ACK)
			HostMsgAckHandler();

		if (hostBits & HOSTMSG_SW_INT_NAK)
			HostMsgNakHandler();

		if (hostBits & HOSTMSG_SW_INT_PKT_RDY)
			HostMsgRxPacketHandler();

		/* Update the current interrupt status. */
		retStatus = SiiDeviceRegRead(SII_REG_INT_N_STATUS, &currentIsr);
		if (retStatus < 0) {
			err("Error reading INT_N_STATUS: 0x%x\n", retStatus);
			goto done;
		}

		/* Update the pending interrupts. */
		pendingInts = currentMask & currentIsr;

		/* Get the Host protocol interrupt bits (subset of pending). */
		hostBits = FIELD_EXTRACT(pendingInts,
				REG_GLUE_AP_CTRL_INT_STATUS_ADDR_HOST_MSG);
	}

done:
	for (i = 0; i < MAX_REENABLE_INTERRUPTS_RETRIES; i++) {
		/* Re-enable previous interrupt sources. */
		retStatus = SiiDeviceIntEnableReadWrite(0, currentMask, NULL);
		if (retStatus < 0) {
			err("Error modify INT_N_ENABLE(on)(attempt %d): 0x%x\n",
			    i + 1, retStatus);
		} else {
			break;
		}
	}

done_no_reenable:
	if (sii6400_boot_ready ||
	    sii6400_boot_successful ||
	    sii6400_boot_failed) {
		wake_up_interruptible(&sii6400_interrupt_wait);
	}

	SiiHalIsrDone();
}

