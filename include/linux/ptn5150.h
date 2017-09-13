/*
 * Copyright (C) 2015 World Peace Industrial Group.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.
 *
 */


/*For OTG function support*/
//#define PTN_OTG_SUPPORT

/*Open just for Debugging*/
#define PTN5150_DEBUG


/*----------------- REGISTER DEFINATION -----------------*/

#define PTN5150_7BIT_I2C_ADDR_LOW	0x1d
#define PTN5150_7BIT_I2C_ADDR_HIGH	0x3d

#define	PTN5150_VERSION_REG			0x01

#define	VERSION_ID_MASK				0xf8
#define	VENDOR_ID_MASK				0x07

#define	PTN5150_CONTROL_REG			0x02

#define	RP_SELECT_DEFULAT			0x00
#define	RP_SELECT_MEDIUM			0x08
#define	RP_SELECT_HIGH				0x18

#define	PORT_SET_UFP				(0x0 << 1)
#define	PORT_SET_DFP				(0x1 << 1)
#define	PORT_SET_DRP				(0x2 << 1)

#define	ATTACH_INT_MASK_EN			0x00
#define	ATTACH_INT_MASK_DIS			0x01

#define	PTN5150_VID_REG				0x01
#define	PTN5150_INT_STATUS_REG		0x03
#define	PTN5150_INT_MASK_REG		0x18
#define	PTN5150_INT_STATUS_REL_REG	0x19
#define	PTN5150_SPECIAL_43H_REG		0x43
#define	PTN5150_SPECIAL_49H_REG		0x49
#define	PTN5150_SPECIAL_4AH_REG		0x4A
#define	PTN5150_SPECIAL_4CH_REG		0x4C

#define	CABLE_DISCONNECT			0x00
#define	DFP_ATTACHED				0x01
#define	UFP_ATTACHED				0x02
#define	ANLOG_AUDIO_MODE			0x03

#define	PTN5150_CC_STATUS_REG		0x04

#define	VBUS_DETECTION				0x80

#define	RP_DET_AS_NONE				0x00
#define	RP_DET_AS_DEFAULT			0x01
#define	RP_DET_AS_MEDIUM			0x02
#define	RP_DET_AS_HIGH				0x03

#define	ATTACHED_IS_NONE			0x00
#define	ATTACHED_IS_DFP				0x01
#define	ATTACHED_IS_UFP				0x02
#define	ATTACHED_IS_AUDIO			0x03
#define	ATTACHED_IS_DEBUG			0x04

#define	CC_POLARITY_NONE			0x00
#define	CC_POLARITY_CC1				0x01
#define	CC_POLARITY_CC2				0x02

#define	PTN5150_CONDET_REG			0x09

#define	CON_DET_EN					0x00
#define	CON_DET_DIS					0x01

#define	PTN5150_VCONN_REG			0x0a

#define	VCONN_STANDBY				0x00
#define	VCONN_CC1					0x01
#define	VCONN_CC2					0x02

//#define	PTN5150_VCONN_ACCESS_REG	0x43

#define	VCONN_ACCESS_CODE			0xe0

/*----------------- REGISTER DEFINATION -----------------*/


/*----------------- TRY SINK STATUS -----------------*/
#if 0
#define  Try_Sink_Idle_DRP      				0
#define  Try_Sink_Attached_Wait_Src    			1
#define  Try_Sink_Source_To_Sink    			2
#define  Try_Sink_Attached_Wait_Src_Detached   	3
#define  Try_Sink_Attached_As_UFP    			4
#define  Try_Sink_tDRPTry_Expire    			5
#define  Try_Sink_Try_Wait_Src     				6
#define  Try_Sink_Attached_As_DFP    			7
#define  Try_Sink_Attached_As_Audio    			8
#define  Try_Sink_Attached_As_Debug    			9
#endif //v0.1 state

#define Try_Sink_Idle_DRP					0
#define Try_Sink_Attached_Wait_Src			1
#define Try_Sink_Source_To_Sink				2
#define Try_Sink_Attached_Wait_Src_Detached	3
#define Try_Sink_Attached_As_UFP			4
#define Try_Sink_tDRPTry_Expire				5
#define Try_Sink_Try_Wait_Src				6
#define Try_Sink_Attached_As_DFP			7
#define Try_Sink_Try_Wait_Src_Expire		8

/*----------------- TRY SINK STATUS -----------------*/

