/*************************************************************************
 * Copyright (C) 2015 Hideep, Inc.
 * anthony.kim@hideep.com
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
 *************************************************************************/

#ifndef _LINUX_HIDEEP_DBG_H
#define _LINUX_HIDEEP_DBG_H

/*************************************************************************
 * VR OP Mode register
 *************************************************************************/
#define HIDEEP_OPM_TOUCH_A					(0x00)
#define HIDEEP_OPM_RAW						(0x8F)

/*************************************************************************
 * VR info
 *************************************************************************/
#define ADDR_VR     (0x0000)
#define ADDR_VR_END (0xFFFF)
#define ADDR_IMG    (0x1000)
#define ADDR_UC     (0x10000000)
#define MAX_VR_BUFF  1024

#define HIDEEP_Z_VALUE		0x246

#define HIDEEP_Z_CALIB2			0x1004
#define HIDEEP_Z_CALIB2_READ	0x70007000
#define HIDEEP_VERSION_INFO		0x10000001

#define HIDEEP_RELEASE_FLAG		0x8000

/* IOCTL command */
#define HIDEEP_IOC_MAGIC  'k'
#define HIDEEP_CFG     _IOW(HIDEEP_IOC_MAGIC,  0x01, struct hideep_debug_cfg_t)
#define HIDEEP_DEBUG_MODE	_IOW(HIDEEP_IOC_MAGIC, 0x02, int)
#define HIDEEP_GET_DATA		_IOR(HIDEEP_IOC_MAGIC, 0x03, unsigned char*)
#define HIDEEP_IOC_MAXNR 0xff

#endif /* _LINUX_HIDEEP_DBG_H */
