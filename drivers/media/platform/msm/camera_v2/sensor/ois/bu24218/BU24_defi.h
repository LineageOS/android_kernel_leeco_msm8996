/////////////////////////////////////////////////////////////////////////////
// File Name	: BU24_defi.h
// Function		: Header file for BU24 series OIS controller
// Rule         : Use TAB 4
//
// Copyright(c)	Rohm Co.,Ltd. All rights reserved 
// 
/***** ROHM Confidential ***************************************************/
#ifndef BU24_DEFI_H
#define BU24_DEFI_H

#include <soc/qcom/camera2.h>
#include "msm_camera_i2c.h"

/******* Registers *******/
#define	DWNLD_DATA1_START	0x0000
#define DWNLD_DATA2_START	0x1C00
#define DWNLD_CALIB_START	0x1DC0
#define DWNLD_REVISION		0x6010
#define OIS_CNTL			0x6020
#define OIS_MODE			0x6021
#define GYRO_CNTL			0x6023
#define OIS_STS				0x6024
#define GYRO_SET1			0x602C
#define GYRO_SET2			0x602D
#define DWNLD_COMPLETE		0xF006
#define DWNLD_CHECKSUM		0xF008
#define DWNLD_START			0xF010

/******* Values *******/
#define STILL_MODE			0x03
#define MOVIE_MODE_1		0x61
#define MOVIE_MODE_2		0x63
#define VIEWFINDER_MODE		0x79
#define ZSL_MODE			0x7B
#define SERVO_OFF			0x00
#define SERVO_ON			0x01
#define OIS_ON				0x02



/******* OIS Controll Interface *******/
int BU24_OIS_Gyro_On(void);
int BU24_OIS_Download(void);
int BU24_OIS_Change_Mode(int mode);
int BU24_OIS_Servo_Set(unsigned char enable);
int Ois_rohm_SetI2cClient(struct msm_camera_i2c_client *client);

#endif
