/*
 *  stmvl53l0_module.c - Linux kernel modules for STM VL53L0 FlightSense TOF
 *						 sensor
 *
 *  Copyright (C) 2015 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
/*
 * API includes
 */
#include "vl53l0_api.h"
#include "vl53l010_api.h"
/*
#include "vl53l0_def.h"
#include "vl53l0_platform.h"
#include "stmvl53l0-i2c.h"
#include "stmvl53l0-cci.h"
#include "stmvl53l0.h"
*/

//#define USE_INT
#define IRQ_NUM	   59
//#define DEBUG_TIME_LOG
#ifdef DEBUG_TIME_LOG
struct timeval start_tv, stop_tv;
#endif

/*
 * Global data
 */

#ifdef CAMERA_CCI
static struct stmvl53l0_module_fn_t stmvl53l0_module_func_tbl = {
	.init = stmvl53l0_init_cci,
	.deinit = stmvl53l0_exit_cci,
	.power_up = stmvl53l0_power_up_cci,
	.power_down = stmvl53l0_power_down_cci,
};
#else
static struct stmvl53l0_module_fn_t stmvl53l0_module_func_tbl = {
	.init = stmvl53l0_init_i2c,
	.deinit = stmvl53l0_exit_i2c,
	.power_up = stmvl53l0_power_up_i2c,
	.power_down = stmvl53l0_power_down_i2c,
};
#endif
struct stmvl53l0_module_fn_t *pmodule_func_tbl;

struct stmvl53l0_api_fn_t {
	VL53L0_Error (*GetVersion)(VL53L0_Version_t *pVersion);
	VL53L0_Error (*GetPalSpecVersion)(VL53L0_Version_t *pPalSpecVersion);
	
	VL53L0_Error (*GetProductRevision)(VL53L0_DEV Dev,
					uint8_t* pProductRevisionMajor,
					uint8_t* pProductRevisionMinor);
	VL53L0_Error (*GetDeviceInfo)(VL53L0_DEV Dev,
					VL53L0_DeviceInfo_t *pVL53L0_DeviceInfo);
	VL53L0_Error (*GetDeviceErrorStatus)(VL53L0_DEV Dev,
					VL53L0_DeviceError * pDeviceErrorStatus);
	VL53L0_Error (*GetDeviceErrorString)(VL53L0_DeviceError ErrorCode,
					char *pDeviceErrorString);
	VL53L0_Error (*GetPalErrorString)(VL53L0_Error PalErrorCode,
					char *pPalErrorString);
	VL53L0_Error (*GetPalState)(VL53L0_DEV Dev,	VL53L0_State * pPalState);
	VL53L0_Error (*SetPowerMode)(VL53L0_DEV Dev,
					VL53L0_PowerModes PowerMode);
	VL53L0_Error (*GetPowerMode)(VL53L0_DEV Dev,
					VL53L0_PowerModes * pPowerMode);
	VL53L0_Error (*SetOffsetCalibrationDataMicroMeter)(VL53L0_DEV Dev,
					int32_t OffsetCalibrationDataMicroMeter);
	VL53L0_Error (*GetOffsetCalibrationDataMicroMeter)(VL53L0_DEV Dev,
					int32_t *pOffsetCalibrationDataMicroMeter);
	VL53L0_Error (*SetGroupParamHold)(VL53L0_DEV Dev,
					uint8_t GroupParamHold);
	VL53L0_Error (*GetUpperLimitMilliMeter)(VL53L0_DEV Dev,
					uint16_t *pUpperLimitMilliMeter);
	VL53L0_Error (*SetDeviceAddress)(VL53L0_DEV Dev,
					uint8_t DeviceAddress);
	VL53L0_Error (*DataInit)(VL53L0_DEV Dev);
	VL53L0_Error (*SetTuningSettingBuffer)(VL53L0_DEV Dev,
              		uint8_t* pTuningSettingBuffer,
              		uint8_t UseInternalTuningSettings);
	VL53L0_Error (*GetTuningSettingBuffer)(VL53L0_DEV Dev,
              		uint8_t* pTuningSettingBuffer,
              		uint8_t* pUseInternalTuningSettings);
	VL53L0_Error (*StaticInit)(VL53L0_DEV Dev);
	VL53L0_Error (*WaitDeviceBooted)(VL53L0_DEV Dev);
	VL53L0_Error (*ResetDevice)(VL53L0_DEV Dev);
	VL53L0_Error (*SetDeviceParameters)(VL53L0_DEV Dev,
					const VL53L0_DeviceParameters_t *pDeviceParameters);
	VL53L0_Error (*GetDeviceParameters)(VL53L0_DEV Dev,
					VL53L0_DeviceParameters_t *pDeviceParameters);
	VL53L0_Error (*SetDeviceMode)(VL53L0_DEV Dev,
					VL53L0_DeviceModes DeviceMode);
	VL53L0_Error (*GetDeviceMode)(VL53L0_DEV Dev,
					VL53L0_DeviceModes * pDeviceMode);
	VL53L0_Error (*SetHistogramMode)(VL53L0_DEV Dev,
					VL53L0_HistogramModes HistogramMode);
	VL53L0_Error (*GetHistogramMode)(VL53L0_DEV Dev,
					VL53L0_HistogramModes * pHistogramMode);
	VL53L0_Error (*SetMeasurementTimingBudgetMicroSeconds)(VL53L0_DEV Dev,
                    uint32_t  MeasurementTimingBudgetMicroSeconds);
	VL53L0_Error (*GetMeasurementTimingBudgetMicroSeconds)(
					VL53L0_DEV Dev,
					uint32_t *pMeasurementTimingBudgetMicroSeconds);
	VL53L0_Error (*GetVcselPulsePeriod)(VL53L0_DEV Dev,
                    VL53L0_VcselPeriod VcselPeriodType,
					uint8_t	*pVCSELPulsePeriod);
	VL53L0_Error (*SetVcselPulsePeriod)(VL53L0_DEV Dev,
                    VL53L0_VcselPeriod VcselPeriodType,
					uint8_t VCSELPulsePeriod);
	VL53L0_Error (*SetSequenceStepEnable)(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId,
					uint8_t SequenceStepEnabled);
	VL53L0_Error (*GetSequenceStepEnable)(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId,
					uint8_t* pSequenceStepEnabled);
	VL53L0_Error (*GetSequenceStepEnables)(VL53L0_DEV Dev,
                    VL53L0_SchedulerSequenceSteps_t *pSchedulerSequenceSteps);
	VL53L0_Error (*SetSequenceStepTimeout)(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId,
					FixPoint1616_t TimeOutMilliSecs);
	VL53L0_Error (*GetSequenceStepTimeout)(VL53L0_DEV Dev,
                    VL53L0_SequenceStepId SequenceStepId,
					FixPoint1616_t *pTimeOutMilliSecs);
	VL53L0_Error (*GetNumberOfSequenceSteps)(VL53L0_DEV Dev,
                    uint8_t* pNumberOfSequenceSteps);
	VL53L0_Error (*GetSequenceStepsInfo)(
                    VL53L0_SequenceStepId SequenceStepId,
                    char* pSequenceStepsString);
	VL53L0_Error (*SetInterMeasurementPeriodMilliSeconds)(
					VL53L0_DEV Dev,
					uint32_t InterMeasurementPeriodMilliSeconds);
	VL53L0_Error (*GetInterMeasurementPeriodMilliSeconds)(
					VL53L0_DEV Dev,
					uint32_t *pInterMeasurementPeriodMilliSeconds);
	VL53L0_Error (*SetXTalkCompensationEnable)(VL53L0_DEV Dev,
					uint8_t XTalkCompensationEnable);
	VL53L0_Error (*GetXTalkCompensationEnable)(VL53L0_DEV Dev,
					uint8_t* pXTalkCompensationEnable);
	VL53L0_Error (*SetXTalkCompensationRateMegaCps)(
					VL53L0_DEV Dev,
					FixPoint1616_t XTalkCompensationRateMegaCps);
	VL53L0_Error (*GetXTalkCompensationRateMegaCps)(
					VL53L0_DEV Dev,
					FixPoint1616_t *pXTalkCompensationRateMegaCps);
	VL53L0_Error (*GetNumberOfLimitCheck)(
					uint16_t *pNumberOfLimitCheck);
	VL53L0_Error (*GetLimitCheckInfo)(VL53L0_DEV Dev,
					uint16_t LimitCheckId, char *pLimitCheckString);
	VL53L0_Error (*SetLimitCheckEnable)(VL53L0_DEV Dev,
					uint16_t LimitCheckId, 
					uint8_t LimitCheckEnable);
	VL53L0_Error (*GetLimitCheckEnable)(VL53L0_DEV Dev,
					uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);
	VL53L0_Error (*SetLimitCheckValue)(VL53L0_DEV Dev,
					uint16_t LimitCheckId, 
					FixPoint1616_t LimitCheckValue);
	VL53L0_Error (*GetLimitCheckValue)(VL53L0_DEV Dev,
					uint16_t LimitCheckId,
					FixPoint1616_t *pLimitCheckValue);
	VL53L0_Error (*GetLimitCheckCurrent)(VL53L0_DEV Dev,
					uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent);
	VL53L0_Error (*SetWrapAroundCheckEnable)(VL53L0_DEV Dev,
					uint8_t WrapAroundCheckEnable);
	VL53L0_Error (*GetWrapAroundCheckEnable)(VL53L0_DEV Dev,
					uint8_t *pWrapAroundCheckEnable);
	VL53L0_Error (*PerformSingleMeasurement)(VL53L0_DEV Dev);
	VL53L0_Error (*PerformRefCalibration)(VL53L0_DEV Dev);
	VL53L0_Error (*PerformXTalkCalibration)(VL53L0_DEV Dev,
            		FixPoint1616_t XTalkCalDistance,
					FixPoint1616_t *pXTalkCompensationRateMegaCps);
	VL53L0_Error (*PerformOffsetCalibration)(VL53L0_DEV Dev,
            		FixPoint1616_t CalDistanceMilliMeter,
					int32_t* pOffsetMicroMeter);
	VL53L0_Error (*StartMeasurement)(VL53L0_DEV Dev);
	VL53L0_Error (*StopMeasurement)(VL53L0_DEV Dev);
	VL53L0_Error (*GetMeasurementDataReady)(VL53L0_DEV Dev,
            		uint8_t *pMeasurementDataReady);
	VL53L0_Error (*WaitDeviceReadyForNewMeasurement)(VL53L0_DEV Dev,
					uint32_t MaxLoop);
	VL53L0_Error (*GetRangingMeasurementData)(VL53L0_DEV Dev,
					VL53L0_RangingMeasurementData_t *pRangingMeasurementData);
	VL53L0_Error (*GetHistogramMeasurementData)(VL53L0_DEV Dev,
					VL53L0_HistogramMeasurementData_t *pHistogramMeasurementData);
	VL53L0_Error (*PerformSingleRangingMeasurement)(VL53L0_DEV Dev,
					VL53L0_RangingMeasurementData_t *pRangingMeasurementData);
	VL53L0_Error (*PerformSingleHistogramMeasurement)(VL53L0_DEV Dev,
					VL53L0_HistogramMeasurementData_t *pHistogramMeasurementData);
	VL53L0_Error (*SetNumberOfROIZones)(VL53L0_DEV Dev,
					uint8_t NumberOfROIZones);
	VL53L0_Error (*GetNumberOfROIZones)(VL53L0_DEV Dev,
					uint8_t *pNumberOfROIZones);
	VL53L0_Error (*GetMaxNumberOfROIZones)(VL53L0_DEV Dev,
					uint8_t *pMaxNumberOfROIZones);
	VL53L0_Error (*SetGpioConfig)(VL53L0_DEV Dev, 
					uint8_t Pin,
					VL53L0_DeviceModes DeviceMode,
					VL53L0_GpioFunctionality Functionality,
					VL53L0_InterruptPolarity Polarity);
	VL53L0_Error (*GetGpioConfig)(VL53L0_DEV Dev,
            		uint8_t Pin,
            		VL53L0_DeviceModes* pDeviceMode,
            		VL53L0_GpioFunctionality* pFunctionality,
            		VL53L0_InterruptPolarity* pPolarity);
	VL53L0_Error (*SetInterruptThresholds)(VL53L0_DEV Dev,
					VL53L0_DeviceModes DeviceMode,
					FixPoint1616_t ThresholdLow,
					FixPoint1616_t ThresholdHigh);
	VL53L0_Error (*GetInterruptThresholds)(VL53L0_DEV Dev,
		            VL53L0_DeviceModes DeviceMode,
        		    FixPoint1616_t* pThresholdLow,
        		    FixPoint1616_t* pThresholdHigh);
	VL53L0_Error (*ClearInterruptMask)(VL53L0_DEV Dev,
					uint32_t InterruptMask);
	VL53L0_Error (*GetInterruptMaskStatus)(VL53L0_DEV Dev,
            		uint32_t *pInterruptMaskStatus);
	VL53L0_Error (*EnableInterruptMask)(VL53L0_DEV Dev, uint32_t InterruptMask);
	VL53L0_Error (*SetSpadAmbientDamperThreshold)(VL53L0_DEV Dev,
            		uint16_t SpadAmbientDamperThreshold);
	VL53L0_Error (*GetSpadAmbientDamperThreshold)(VL53L0_DEV Dev,
            		uint16_t* pSpadAmbientDamperThreshold);
	VL53L0_Error (*SetSpadAmbientDamperFactor)(VL53L0_DEV Dev,
            		uint16_t SpadAmbientDamperFactor);
	VL53L0_Error (*GetSpadAmbientDamperFactor)(VL53L0_DEV Dev,
                    uint16_t* pSpadAmbientDamperFactor);
	VL53L0_Error (*PerformRefSpadManagement)(VL53L0_DEV Dev,uint32_t *refSpadCount, uint8_t *isApertureSpads);


};

static struct stmvl53l0_api_fn_t stmvl53l0_api_func_tbl= {
	.GetVersion = VL53L0_GetVersion,
	.GetPalSpecVersion = VL53L0_GetPalSpecVersion,
	.GetProductRevision = VL53L0_GetProductRevision,
	.GetDeviceInfo = VL53L0_GetDeviceInfo,
	.GetDeviceErrorStatus = VL53L0_GetDeviceErrorStatus,
	.GetDeviceErrorString = VL53L0_GetDeviceErrorString,
	.GetPalErrorString = VL53L0_GetPalErrorString,
	.GetPalState = VL53L0_GetPalState,
	.SetPowerMode = VL53L0_SetPowerMode,
	.GetPowerMode = VL53L0_GetPowerMode,
	.SetOffsetCalibrationDataMicroMeter = VL53L0_SetOffsetCalibrationDataMicroMeter,
	.GetOffsetCalibrationDataMicroMeter = VL53L0_GetOffsetCalibrationDataMicroMeter,
	.SetGroupParamHold = VL53L0_SetGroupParamHold,
	.GetUpperLimitMilliMeter = VL53L0_GetUpperLimitMilliMeter,
	.SetDeviceAddress = VL53L0_SetDeviceAddress,
	.DataInit = VL53L0_DataInit,
	.SetTuningSettingBuffer = VL53L0_SetTuningSettingBuffer,
	.GetTuningSettingBuffer = VL53L0_GetTuningSettingBuffer,
	.StaticInit = VL53L0_StaticInit,
	.WaitDeviceBooted = VL53L0_WaitDeviceBooted,
	.ResetDevice = VL53L0_ResetDevice,
	.SetDeviceParameters = VL53L0_SetDeviceParameters,
	.SetDeviceMode = VL53L0_SetDeviceMode,
	.GetDeviceMode = VL53L0_GetDeviceMode,
	.SetHistogramMode = VL53L0_SetHistogramMode,
	.GetHistogramMode = VL53L0_GetHistogramMode,
	.SetMeasurementTimingBudgetMicroSeconds = VL53L0_SetMeasurementTimingBudgetMicroSeconds,
	.GetMeasurementTimingBudgetMicroSeconds = VL53L0_GetMeasurementTimingBudgetMicroSeconds,
	.GetVcselPulsePeriod = VL53L0_GetVcselPulsePeriod,
	.SetVcselPulsePeriod = VL53L0_SetVcselPulsePeriod,
	.SetSequenceStepEnable = VL53L0_SetSequenceStepEnable,
	.GetSequenceStepEnable = VL53L0_GetSequenceStepEnable,
	.GetSequenceStepEnables = VL53L0_GetSequenceStepEnables,
	.SetSequenceStepTimeout = VL53L0_SetSequenceStepTimeout,
	.GetSequenceStepTimeout = VL53L0_GetSequenceStepTimeout,
	.GetNumberOfSequenceSteps = VL53L0_GetNumberOfSequenceSteps,
	.GetSequenceStepsInfo = VL53L0_GetSequenceStepsInfo,
	.SetInterMeasurementPeriodMilliSeconds = VL53L0_SetInterMeasurementPeriodMilliSeconds,
	.GetInterMeasurementPeriodMilliSeconds = VL53L0_GetInterMeasurementPeriodMilliSeconds,
	.SetXTalkCompensationEnable = VL53L0_SetXTalkCompensationEnable,
	.GetXTalkCompensationEnable = VL53L0_GetXTalkCompensationEnable,
	.SetXTalkCompensationRateMegaCps = VL53L0_SetXTalkCompensationRateMegaCps,
	.GetXTalkCompensationRateMegaCps = VL53L0_GetXTalkCompensationRateMegaCps,
	.GetNumberOfLimitCheck = VL53L0_GetNumberOfLimitCheck,
	.GetLimitCheckInfo = VL53L0_GetLimitCheckInfo,
	.SetLimitCheckEnable = VL53L0_SetLimitCheckEnable,
	.GetLimitCheckEnable = VL53L0_GetLimitCheckEnable,
	.SetLimitCheckValue = VL53L0_SetLimitCheckValue,
	.GetLimitCheckValue = VL53L0_GetLimitCheckValue,
	.GetLimitCheckCurrent = VL53L0_GetLimitCheckCurrent,
	.SetWrapAroundCheckEnable = VL53L0_SetWrapAroundCheckEnable,
	.GetWrapAroundCheckEnable = VL53L0_GetWrapAroundCheckEnable,
	.PerformSingleMeasurement = VL53L0_PerformSingleMeasurement,
	.PerformRefCalibration = VL53L0_PerformRefCalibration,
	.PerformXTalkCalibration = VL53L0_PerformXTalkCalibration,
	.PerformOffsetCalibration = VL53L0_PerformOffsetCalibration,
	.StartMeasurement = VL53L0_StartMeasurement,
	.StopMeasurement = VL53L0_StopMeasurement,
	.GetMeasurementDataReady = VL53L0_GetMeasurementDataReady,
	.WaitDeviceReadyForNewMeasurement = VL53L0_WaitDeviceReadyForNewMeasurement,
	.GetRangingMeasurementData = VL53L0_GetRangingMeasurementData,
	.GetHistogramMeasurementData = VL53L0_GetHistogramMeasurementData,
	.PerformSingleRangingMeasurement = VL53L0_PerformSingleRangingMeasurement,
	.PerformSingleHistogramMeasurement = VL53L0_PerformSingleHistogramMeasurement,
	.SetNumberOfROIZones = VL53L0_SetNumberOfROIZones,
	.GetNumberOfROIZones = VL53L0_GetNumberOfROIZones,
	.GetMaxNumberOfROIZones = VL53L0_GetMaxNumberOfROIZones,
	.SetGpioConfig = VL53L0_SetGpioConfig,
	.GetGpioConfig = VL53L0_GetGpioConfig,
	.SetInterruptThresholds = VL53L0_SetInterruptThresholds,
	.GetInterruptThresholds = VL53L0_GetInterruptThresholds,
	.ClearInterruptMask = VL53L0_ClearInterruptMask,
	.GetInterruptMaskStatus = VL53L0_GetInterruptMaskStatus,
	.EnableInterruptMask = VL53L0_EnableInterruptMask,
	.SetSpadAmbientDamperThreshold = VL53L0_SetSpadAmbientDamperThreshold,
	.GetSpadAmbientDamperThreshold = VL53L0_GetSpadAmbientDamperThreshold,
	.SetSpadAmbientDamperFactor = VL53L0_SetSpadAmbientDamperFactor,
	.GetSpadAmbientDamperFactor = VL53L0_GetSpadAmbientDamperFactor,
	.PerformRefSpadManagement = VL53L0_PerformRefSpadManagement,

};
struct stmvl53l0_api_fn_t *papi_func_tbl;

/*
 * IOCTL definitions
 */
#define VL53L0_IOCTL_INIT			_IO('p', 0x01)
#define VL53L0_IOCTL_XTALKCALB		_IOW('p', 0x02, unsigned int)
#define VL53L0_IOCTL_OFFCALB		_IOW('p', 0x03, unsigned int)
#define VL53L0_IOCTL_STOP			_IO('p', 0x05)
#define VL53L0_IOCTL_SETXTALK		_IOW('p', 0x06, unsigned int)
#define VL53L0_IOCTL_SETOFFSET		_IOW('p', 0x07, int8_t)
#define VL53L0_IOCTL_GETDATAS \
			_IOR('p', 0x0b, VL53L0_RangingMeasurementData_t)
#define VL53L0_IOCTL_REGISTER \
			_IOWR('p', 0x0c, struct stmvl53l0_register)
#define VL53L0_IOCTL_PARAMETER \
			_IOWR('p', 0x0d, struct stmvl53l0_parameter)

#define CALIBRATION_FILE 1
#ifdef CALIBRATION_FILE
int8_t offset_calib;
int16_t xtalk_calib;
#endif

static long stmvl53l0_ioctl(struct file *file,
							unsigned int cmd,
							unsigned long arg);
/*static int stmvl53l0_flush(struct file *file, fl_owner_t id);*/
static int stmvl53l0_open(struct inode *inode, struct file *file);
static int stmvl53l0_init_client(struct stmvl53l0_data *data);
static int stmvl53l0_start(struct stmvl53l0_data *data, uint8_t scaling,
			init_mode_e mode);
static int stmvl53l0_stop(struct stmvl53l0_data *data);

#ifdef CALIBRATION_FILE
#if 0
static void stmvl53l0_read_calibration_file(struct stmvl53l0_data *data)
{
	struct file *f;
	char buf[8];
	mm_segment_t fs;
	int i, is_sign = 0;

	f = filp_open("/data/calibration/offset", O_RDONLY, 0);
	if (f != NULL && !IS_ERR(f) && f->f_dentry != NULL) {
		fs = get_fs();
		set_fs(get_ds());
		/* init the buffer with 0 */
		for (i = 0; i < 8; i++)
			buf[i] = 0;
		f->f_op->read(f, buf, 8, &f->f_pos);
		set_fs(fs);
		vl53l0_dbgmsg("offset as:%s, buf[0]:%c\n", buf, buf[0]);
		offset_calib = 0;
		for (i = 0; i < 8; i++) {
			if (i == 0 && buf[0] == '-')
				is_sign = 1;
			else if (buf[i] >= '0' && buf[i] <= '9')
				offset_calib = offset_calib * 10 +
					(buf[i] - '0');
			else
				break;
		}
		if (is_sign == 1)
			offset_calib = -offset_calib;
		vl53l0_dbgmsg("offset_calib as %d\n", offset_calib);
/*later
		VL6180x_SetOffsetCalibrationData(vl53l0_dev, offset_calib);
*/
		filp_close(f, NULL);
	} else {
		vl53l0_errmsg("no offset calibration file exist!\n");
	}

	is_sign = 0;
	f = filp_open("/data/calibration/xtalk", O_RDONLY, 0);
	if (f != NULL && !IS_ERR(f) && f->f_dentry != NULL) {
		fs = get_fs();
		set_fs(get_ds());
		/* init the buffer with 0 */
		for (i = 0; i < 8; i++)
			buf[i] = 0;
		f->f_op->read(f, buf, 8, &f->f_pos);
		set_fs(fs);
		vl53l0_dbgmsg("xtalk as:%s, buf[0]:%c\n", buf, buf[0]);
		xtalk_calib = 0;
		for (i = 0; i < 8; i++) {
			if (i == 0 && buf[0] == '-')
				is_sign = 1;
			else if (buf[i] >= '0' && buf[i] <= '9')
				xtalk_calib = xtalk_calib * 10 + (buf[i] - '0');
			else
				break;
		}
		if (is_sign == 1)
			xtalk_calib = -xtalk_calib;
		vl53l0_dbgmsg("xtalk_calib as %d\n", xtalk_calib);
/* later
		VL6180x_SetXTalkCompensationRate(vl53l0_dev, xtalk_calib);
*/
		filp_close(f, NULL);
	} else {
		vl53l0_errmsg("no xtalk calibration file exist!\n");
	}

}
#endif

static void stmvl53l0_write_offset_calibration_file(void)
{
	struct file *f;
	char buf[8];
	mm_segment_t fs;

	f = filp_open("/data/calibration/offset", O_WRONLY|O_CREAT, 0644);
	if (f != NULL) {
		fs = get_fs();
		set_fs(get_ds());
		snprintf(buf, sizeof(buf), "%d", offset_calib);
		vl53l0_dbgmsg("write offset as:%s, buf[0]:%c\n", buf, buf[0]);
		f->f_op->write(f, buf, 8, &f->f_pos);
		set_fs(fs);
	}
	filp_close(f, NULL);

}

static void stmvl53l0_write_xtalk_calibration_file(void)
{
	struct file *f;
	char buf[8];
	mm_segment_t fs;

	f = filp_open("/data/calibration/xtalk", O_WRONLY|O_CREAT, 0644);
	if (f != NULL) {
		fs = get_fs();
		set_fs(get_ds());
		snprintf(buf, sizeof(buf), "%d", xtalk_calib);
		vl53l0_dbgmsg("write xtalk as:%s, buf[0]:%c\n", buf, buf[0]);
		f->f_op->write(f, buf, 8, &f->f_pos);
		set_fs(fs);
	}
	filp_close(f, NULL);

}
#endif
#ifdef DEBUG_TIME_LOG
static void stmvl53l0_DebugTimeGet(struct timeval *ptv)
{
	do_gettimeofday(ptv);
}

static void stmvl53l0_DebugTimeDuration(struct timeval *pstart_tv,
			struct timeval *pstop_tv)
{
	long total_sec, total_msec;
	total_sec = pstop_tv->tv_sec - pstart_tv->tv_sec;
	total_msec = (pstop_tv->tv_usec - pstart_tv->tv_usec)/1000;
	total_msec += total_sec * 1000;
	pr_err("elapsedTime:%ld\n", total_msec);
}
#endif

static void stmvl53l0_setupAPIFunctions(struct stmvl53l0_data *data)
{
	uint8_t revision = 0;
	VL53L0_DEV vl53l0_dev = data;

	/* Read Revision ID */
	VL53L0_RdByte(vl53l0_dev, VL53L0_REG_IDENTIFICATION_REVISION_ID, &revision);
	vl53l0_errmsg("read REVISION_ID: 0x%x\n", revision);
	revision = ( revision & 0xF0) >> 4;
	if (revision == 1) {
		/*cut 1.1*/ 
		vl53l0_errmsg("to setup API cut 1.1\n");
		papi_func_tbl->GetVersion = VL53L0_GetVersion;
		papi_func_tbl->GetPalSpecVersion = VL53L0_GetPalSpecVersion;
		papi_func_tbl->GetProductRevision = VL53L0_GetProductRevision;
		papi_func_tbl->GetDeviceInfo = VL53L0_GetDeviceInfo;
		papi_func_tbl->GetDeviceErrorStatus = VL53L0_GetDeviceErrorStatus;
		papi_func_tbl->GetDeviceErrorString = VL53L0_GetDeviceErrorString;
		papi_func_tbl->GetPalErrorString = VL53L0_GetPalErrorString;
		papi_func_tbl->GetPalState = VL53L0_GetPalState;
		papi_func_tbl->SetPowerMode = VL53L0_SetPowerMode;
		papi_func_tbl->GetPowerMode = VL53L0_GetPowerMode;
		papi_func_tbl->SetOffsetCalibrationDataMicroMeter = VL53L0_SetOffsetCalibrationDataMicroMeter;
		papi_func_tbl->GetOffsetCalibrationDataMicroMeter = VL53L0_GetOffsetCalibrationDataMicroMeter;
		papi_func_tbl->SetGroupParamHold = VL53L0_SetGroupParamHold;
		papi_func_tbl->GetUpperLimitMilliMeter = VL53L0_GetUpperLimitMilliMeter;
		papi_func_tbl->SetDeviceAddress = VL53L0_SetDeviceAddress;
		papi_func_tbl->DataInit = VL53L0_DataInit;
		papi_func_tbl->SetTuningSettingBuffer = VL53L0_SetTuningSettingBuffer;
		papi_func_tbl->GetTuningSettingBuffer = VL53L0_GetTuningSettingBuffer;
		papi_func_tbl->StaticInit = VL53L0_StaticInit;
		papi_func_tbl->WaitDeviceBooted = VL53L0_WaitDeviceBooted;
		papi_func_tbl->ResetDevice = VL53L0_ResetDevice;
		papi_func_tbl->SetDeviceParameters = VL53L0_SetDeviceParameters;
		papi_func_tbl->SetDeviceMode = VL53L0_SetDeviceMode;
		papi_func_tbl->GetDeviceMode = VL53L0_GetDeviceMode;
		papi_func_tbl->SetHistogramMode = VL53L0_SetHistogramMode;
		papi_func_tbl->GetHistogramMode = VL53L0_GetHistogramMode;
		papi_func_tbl->SetMeasurementTimingBudgetMicroSeconds = VL53L0_SetMeasurementTimingBudgetMicroSeconds;
		papi_func_tbl->GetMeasurementTimingBudgetMicroSeconds = VL53L0_GetMeasurementTimingBudgetMicroSeconds;
		papi_func_tbl->GetVcselPulsePeriod = VL53L0_GetVcselPulsePeriod;
		papi_func_tbl->SetVcselPulsePeriod = VL53L0_SetVcselPulsePeriod;
		papi_func_tbl->SetSequenceStepEnable = VL53L0_SetSequenceStepEnable;
		papi_func_tbl->GetSequenceStepEnable = VL53L0_GetSequenceStepEnable;
		papi_func_tbl->GetSequenceStepEnables = VL53L0_GetSequenceStepEnables;
		papi_func_tbl->SetSequenceStepTimeout = VL53L0_SetSequenceStepTimeout;
		papi_func_tbl->GetSequenceStepTimeout = VL53L0_GetSequenceStepTimeout;
		papi_func_tbl->GetNumberOfSequenceSteps = VL53L0_GetNumberOfSequenceSteps;
		papi_func_tbl->GetSequenceStepsInfo = VL53L0_GetSequenceStepsInfo;
		papi_func_tbl->SetInterMeasurementPeriodMilliSeconds = VL53L0_SetInterMeasurementPeriodMilliSeconds;
		papi_func_tbl->GetInterMeasurementPeriodMilliSeconds = VL53L0_GetInterMeasurementPeriodMilliSeconds;
		papi_func_tbl->SetXTalkCompensationEnable = VL53L0_SetXTalkCompensationEnable;
		papi_func_tbl->GetXTalkCompensationEnable = VL53L0_GetXTalkCompensationEnable;
		papi_func_tbl->SetXTalkCompensationRateMegaCps = VL53L0_SetXTalkCompensationRateMegaCps;
		papi_func_tbl->GetXTalkCompensationRateMegaCps = VL53L0_GetXTalkCompensationRateMegaCps;
		papi_func_tbl->GetNumberOfLimitCheck = VL53L0_GetNumberOfLimitCheck;
		papi_func_tbl->GetLimitCheckInfo = VL53L0_GetLimitCheckInfo;
		papi_func_tbl->SetLimitCheckEnable = VL53L0_SetLimitCheckEnable;
		papi_func_tbl->GetLimitCheckEnable = VL53L0_GetLimitCheckEnable;
		papi_func_tbl->SetLimitCheckValue = VL53L0_SetLimitCheckValue;
		papi_func_tbl->GetLimitCheckValue = VL53L0_GetLimitCheckValue;
		papi_func_tbl->GetLimitCheckCurrent = VL53L0_GetLimitCheckCurrent;
		papi_func_tbl->SetWrapAroundCheckEnable = VL53L0_SetWrapAroundCheckEnable;
		papi_func_tbl->GetWrapAroundCheckEnable = VL53L0_GetWrapAroundCheckEnable;
		papi_func_tbl->PerformSingleMeasurement = VL53L0_PerformSingleMeasurement;
		papi_func_tbl->PerformRefCalibration = VL53L0_PerformRefCalibration;
		papi_func_tbl->PerformXTalkCalibration = VL53L0_PerformXTalkCalibration;
		papi_func_tbl->PerformOffsetCalibration = VL53L0_PerformOffsetCalibration;
		papi_func_tbl->StartMeasurement = VL53L0_StartMeasurement;
		papi_func_tbl->StopMeasurement = VL53L0_StopMeasurement;
		papi_func_tbl->GetMeasurementDataReady = VL53L0_GetMeasurementDataReady;
		papi_func_tbl->WaitDeviceReadyForNewMeasurement = VL53L0_WaitDeviceReadyForNewMeasurement;
		papi_func_tbl->GetRangingMeasurementData = VL53L0_GetRangingMeasurementData;
		papi_func_tbl->GetHistogramMeasurementData = VL53L0_GetHistogramMeasurementData;
		papi_func_tbl->PerformSingleRangingMeasurement = VL53L0_PerformSingleRangingMeasurement;
		papi_func_tbl->PerformSingleHistogramMeasurement = VL53L0_PerformSingleHistogramMeasurement;
		papi_func_tbl->SetNumberOfROIZones = VL53L0_SetNumberOfROIZones;
		papi_func_tbl->GetNumberOfROIZones = VL53L0_GetNumberOfROIZones;
		papi_func_tbl->GetMaxNumberOfROIZones = VL53L0_GetMaxNumberOfROIZones;
		papi_func_tbl->SetGpioConfig = VL53L0_SetGpioConfig;
		papi_func_tbl->GetGpioConfig = VL53L0_GetGpioConfig;
		papi_func_tbl->SetInterruptThresholds = VL53L0_SetInterruptThresholds;
		papi_func_tbl->GetInterruptThresholds = VL53L0_GetInterruptThresholds;
		papi_func_tbl->ClearInterruptMask = VL53L0_ClearInterruptMask;
		papi_func_tbl->GetInterruptMaskStatus = VL53L0_GetInterruptMaskStatus;
		papi_func_tbl->EnableInterruptMask = VL53L0_EnableInterruptMask;
		papi_func_tbl->SetSpadAmbientDamperThreshold = VL53L0_SetSpadAmbientDamperThreshold;
		papi_func_tbl->GetSpadAmbientDamperThreshold = VL53L0_GetSpadAmbientDamperThreshold;
		papi_func_tbl->SetSpadAmbientDamperFactor = VL53L0_SetSpadAmbientDamperFactor;
		papi_func_tbl->GetSpadAmbientDamperFactor = VL53L0_GetSpadAmbientDamperFactor;
		papi_func_tbl->PerformRefSpadManagement = VL53L0_PerformRefSpadManagement;


			
	} else if (revision ==0) {
		/*cut 1.0*/
		vl53l0_errmsg("to setup API cut 1.0\n");
		papi_func_tbl->GetVersion = VL53L010_GetVersion;
		papi_func_tbl->GetPalSpecVersion = VL53L010_GetPalSpecVersion;
		//papi_func_tbl->GetProductRevision = NULL;
		papi_func_tbl->GetDeviceInfo = VL53L010_GetDeviceInfo;
		papi_func_tbl->GetDeviceErrorStatus = VL53L010_GetDeviceErrorStatus;
		papi_func_tbl->GetDeviceErrorString = VL53L010_GetDeviceErrorString;
		papi_func_tbl->GetPalErrorString = VL53L010_GetPalErrorString;
		papi_func_tbl->GetPalState = VL53L010_GetPalState;
		papi_func_tbl->SetPowerMode = VL53L010_SetPowerMode;
		papi_func_tbl->GetPowerMode = VL53L010_GetPowerMode;
		papi_func_tbl->SetOffsetCalibrationDataMicroMeter = VL53L010_SetOffsetCalibrationDataMicroMeter;
		papi_func_tbl->GetOffsetCalibrationDataMicroMeter = VL53L010_GetOffsetCalibrationDataMicroMeter;
		papi_func_tbl->SetGroupParamHold = VL53L010_SetGroupParamHold;
		papi_func_tbl->GetUpperLimitMilliMeter = VL53L010_GetUpperLimitMilliMeter;
		papi_func_tbl->SetDeviceAddress = VL53L010_SetDeviceAddress;
		papi_func_tbl->DataInit = VL53L010_DataInit;
		/*
		papi_func_tbl->SetTuningSettingBuffer = NULL;
		papi_func_tbl->GetTuningSettingBuffer = NULL;
		*/
		papi_func_tbl->StaticInit = VL53L010_StaticInit;
		papi_func_tbl->WaitDeviceBooted = VL53L010_WaitDeviceBooted;
		papi_func_tbl->ResetDevice = VL53L010_ResetDevice;
		papi_func_tbl->SetDeviceParameters = VL53L010_SetDeviceParameters;
		papi_func_tbl->SetDeviceMode = VL53L010_SetDeviceMode;
		papi_func_tbl->GetDeviceMode = VL53L010_GetDeviceMode;
		papi_func_tbl->SetHistogramMode = VL53L010_SetHistogramMode;
		papi_func_tbl->GetHistogramMode = VL53L010_GetHistogramMode;
		papi_func_tbl->SetMeasurementTimingBudgetMicroSeconds = VL53L010_SetMeasurementTimingBudgetMicroSeconds;
		papi_func_tbl->GetMeasurementTimingBudgetMicroSeconds = VL53L010_GetMeasurementTimingBudgetMicroSeconds;
		/*
		papi_func_tbl->GetVcselPulsePeriod = NULL;
		papi_func_tbl->SetVcselPulsePeriod = NULL;
		papi_func_tbl->SetSequenceStepEnable = NULL;
		papi_func_tbl->GetSequenceStepEnable = NULL;
		papi_func_tbl->GetSequenceStepEnables = NULL;
		papi_func_tbl->SetSequenceStepTimeout = NULL;
		papi_func_tbl->GetSequenceStepTimeout = NULL;
		papi_func_tbl->GetNumberOfSequenceSteps =NULL;
		papi_func_tbl->GetSequenceStepsInfo = NULL;
		*/
		papi_func_tbl->SetInterMeasurementPeriodMilliSeconds = VL53L010_SetInterMeasurementPeriodMilliSeconds;
		papi_func_tbl->GetInterMeasurementPeriodMilliSeconds = VL53L010_GetInterMeasurementPeriodMilliSeconds;
		papi_func_tbl->SetXTalkCompensationEnable = VL53L010_SetXTalkCompensationEnable;
		papi_func_tbl->GetXTalkCompensationEnable = VL53L010_GetXTalkCompensationEnable;
		papi_func_tbl->SetXTalkCompensationRateMegaCps = VL53L010_SetXTalkCompensationRateMegaCps;
		papi_func_tbl->GetXTalkCompensationRateMegaCps = VL53L010_GetXTalkCompensationRateMegaCps;
		papi_func_tbl->GetNumberOfLimitCheck = VL53L010_GetNumberOfLimitCheck;
		papi_func_tbl->GetLimitCheckInfo = VL53L010_GetLimitCheckInfo;
		papi_func_tbl->SetLimitCheckEnable = VL53L010_SetLimitCheckEnable;
		papi_func_tbl->GetLimitCheckEnable = VL53L010_GetLimitCheckEnable;
		papi_func_tbl->SetLimitCheckValue = VL53L010_SetLimitCheckValue;
		papi_func_tbl->GetLimitCheckValue = VL53L010_GetLimitCheckValue;
		papi_func_tbl->GetLimitCheckCurrent = VL53L010_GetLimitCheckCurrent;
		papi_func_tbl->SetWrapAroundCheckEnable = VL53L010_SetWrapAroundCheckEnable;
		papi_func_tbl->GetWrapAroundCheckEnable = VL53L010_GetWrapAroundCheckEnable;
		papi_func_tbl->PerformSingleMeasurement = VL53L010_PerformSingleMeasurement;
		papi_func_tbl->PerformRefCalibration = VL53L010_PerformRefCalibration;
		papi_func_tbl->PerformXTalkCalibration = VL53L010_PerformXTalkCalibration;
		papi_func_tbl->PerformOffsetCalibration = VL53L010_PerformOffsetCalibration;
		papi_func_tbl->StartMeasurement = VL53L010_StartMeasurement;
		papi_func_tbl->StopMeasurement = VL53L010_StopMeasurement;
		papi_func_tbl->GetMeasurementDataReady = VL53L010_GetMeasurementDataReady;
		papi_func_tbl->WaitDeviceReadyForNewMeasurement = VL53L010_WaitDeviceReadyForNewMeasurement;
		papi_func_tbl->GetRangingMeasurementData = VL53L010_GetRangingMeasurementData;
		papi_func_tbl->GetHistogramMeasurementData = VL53L010_GetHistogramMeasurementData;
		papi_func_tbl->PerformSingleRangingMeasurement = VL53L010_PerformSingleRangingMeasurement;
		papi_func_tbl->PerformSingleHistogramMeasurement = VL53L010_PerformSingleHistogramMeasurement;
		papi_func_tbl->SetNumberOfROIZones = VL53L010_SetNumberOfROIZones;
		papi_func_tbl->GetNumberOfROIZones = VL53L010_GetNumberOfROIZones;
		papi_func_tbl->GetMaxNumberOfROIZones = VL53L010_GetMaxNumberOfROIZones;
		papi_func_tbl->SetGpioConfig = VL53L010_SetGpioConfig;
		papi_func_tbl->GetGpioConfig = VL53L010_GetGpioConfig;
		papi_func_tbl->SetInterruptThresholds = VL53L010_SetInterruptThresholds;
		papi_func_tbl->GetInterruptThresholds = VL53L010_GetInterruptThresholds;
		papi_func_tbl->ClearInterruptMask = VL53L010_ClearInterruptMask;
		papi_func_tbl->GetInterruptMaskStatus = VL53L010_GetInterruptMaskStatus;
		papi_func_tbl->EnableInterruptMask = VL53L010_EnableInterruptMask;
		papi_func_tbl->SetSpadAmbientDamperThreshold = VL53L010_SetSpadAmbientDamperThreshold;
		papi_func_tbl->GetSpadAmbientDamperThreshold = VL53L010_GetSpadAmbientDamperThreshold;
		papi_func_tbl->SetSpadAmbientDamperFactor = VL53L010_SetSpadAmbientDamperFactor;
		papi_func_tbl->GetSpadAmbientDamperFactor = VL53L010_GetSpadAmbientDamperFactor;
		//papi_func_tbl->PerformRefSpadManagement =NULL;

	}

}

static void stmvl53l0_ps_read_measurement(struct stmvl53l0_data *data)
{
	struct timeval tv;

	do_gettimeofday(&tv);

	data->ps_data = data->rangeData.RangeMilliMeter;
	input_report_abs(data->input_dev_ps, ABS_DISTANCE,
		(int)(data->ps_data + 5) / 10);
	input_report_abs(data->input_dev_ps, ABS_HAT0X, tv.tv_sec);
	input_report_abs(data->input_dev_ps, ABS_HAT0Y, tv.tv_usec);
	input_report_abs(data->input_dev_ps, ABS_HAT1X,
		data->rangeData.RangeMilliMeter);
	input_report_abs(data->input_dev_ps, ABS_HAT1Y,
		data->rangeData.RangeStatus);
	input_report_abs(data->input_dev_ps, ABS_HAT2X,
		data->rangeData.SignalRateRtnMegaCps);
	input_report_abs(data->input_dev_ps, ABS_HAT2Y,
		data->rangeData.AmbientRateRtnMegaCps);
	input_report_abs(data->input_dev_ps, ABS_HAT3X,
		data->rangeData.MeasurementTimeUsec);
	input_report_abs(data->input_dev_ps, ABS_HAT3Y,
		data->rangeData.RangeDMaxMilliMeter);
	input_sync(data->input_dev_ps);

	if (data->enableDebug)
		vl53l0_errmsg(
"range:%d, signalRateRtnMegaCps:%d,	error:0x%x,rtnambrate:%d,measuretime:%d\n",
			data->rangeData.RangeMilliMeter,
			data->rangeData.SignalRateRtnMegaCps,
			data->rangeData.RangeStatus,
			data->rangeData.AmbientRateRtnMegaCps,
			data->rangeData.MeasurementTimeUsec);


}

static void stmvl53l0_cancel_handler(struct stmvl53l0_data *data)
{
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&data->update_lock.wait_lock, flags);
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	ret = cancel_delayed_work(&data->dwork);
	if (ret == 0)
		vl53l0_errmsg("cancel_delayed_work return FALSE\n");

	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);

}

static void stmvl53l0_schedule_handler(struct stmvl53l0_data *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->update_lock.wait_lock, flags);
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&data->dwork);
	schedule_delayed_work(&data->dwork, msecs_to_jiffies(data->delay_ms));
	spin_unlock_irqrestore(&data->update_lock.wait_lock, flags);

}


#ifdef USE_INT
static irqreturn_t stmvl53l0_interrupt_handler(int vec, void *info)
{

	struct stmvl53l0_data *data = (struct stmvl53l0_data *)info;

	if (data->irq == vec) {
		data->interrupt_received = 1;
		schedule_delayed_work(&data->dwork, 0);
	}
	return IRQ_HANDLED;
}
#endif

/* work handler */
static void stmvl53l0_work_handler(struct work_struct *work)
{
	struct stmvl53l0_data *data = container_of(work, struct stmvl53l0_data,
				dwork.work);
	VL53L0_DEV vl53l0_dev = data;
	//uint8_t reg_val;
	uint32_t interruptStatus=0;
	VL53L0_Error Status = VL53L0_ERROR_NONE;


	mutex_lock(&data->work_mutex);

	if (data->enable_ps_sensor == 1) {
		/* vl53l0_dbgmsg("Enter\n"); */
#ifdef DEBUG_TIME_LOG
		stmvl53l0_DebugTimeGet(&stop_tv);
		stmvl53l0_DebugTimeDuration(&start_tv, &stop_tv);
#endif
		/*
		Status = papi_func_tbl->PerformSingleRangingMeasurement(vl53l0_dev,
			&(data->rangeData));
		*/
		Status = VL53L0_GetInterruptMaskStatus(vl53l0_dev, &interruptStatus);
		if (data->enableDebug)
			pr_err("interruptStatus:0x%x, interrupt_received:%d\n",
				interruptStatus, data->interrupt_received);
		data->interrupt_received = 0;
		if (Status == VL53L0_ERROR_NONE && 
			interruptStatus == data->gpio_function) {
				Status = papi_func_tbl->ClearInterruptMask(vl53l0_dev, 0);
				Status = papi_func_tbl->GetRangingMeasurementData(
						vl53l0_dev, &(data->rangeData));
				/* to push the measurement */
				if (Status == VL53L0_ERROR_NONE)
					stmvl53l0_ps_read_measurement(data);

				if (data->enableDebug)
					pr_err("Measured range:%d\n",
					data->rangeData.RangeMilliMeter);
		}
#ifdef DEBUG_TIME_LOG
		stmvl53l0_DebugTimeGet(&start_tv);
#endif

		if (data->deviceMode == VL53L0_DEVICEMODE_SINGLE_RANGING) {
		    Status = papi_func_tbl->StartMeasurement(vl53l0_dev);
		} 

		/* enable work handler */
		/* if interrupt is trigger, the work-handler will kick out immediately*/
		stmvl53l0_schedule_handler(data);
	}

	mutex_unlock(&data->work_mutex);

}


/*
 * SysFS support
 */
static ssize_t stmvl53l0_show_modelid(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t modelid = 0;
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	VL53L0_DEV vl53l0_dev = data;

	if (msm_sensor_power_onoff(1,0)) {
		pr_err("%s: Fail to power up\n", __func__);
		return sprintf(buf, "%d\n", modelid);
	}

	VL53L0_RdByte(vl53l0_dev, 0xc0, &modelid);
	vl53l0_errmsg("read MODEL_ID: 0x%x\n", modelid);
	msm_sensor_power_onoff(0,0);

	return snprintf(buf, 6, "0x%02x\n", modelid);
}

static DEVICE_ATTR(modelid, 0664/*S_IWUGO | S_IRUGO*/,
				   stmvl53l0_show_modelid, NULL);

static ssize_t stmvl53l0_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);

	return snprintf(buf, 4, "%d\n", data->enable_ps_sensor);
}

static ssize_t stmvl53l0_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);

	unsigned long val = simple_strtoul(buf, NULL, 10);

	if ((val != 0) && (val != 1)) {
		vl53l0_errmsg("store unvalid value=%ld\n", val);
		return count;
	}
	mutex_lock(&data->work_mutex);
	vl53l0_dbgmsg("Enter, enable_ps_sensor flag:%d\n",
		data->enable_ps_sensor);
	vl53l0_dbgmsg("enable ps senosr ( %ld)\n", val);

	if (val == 1) {
		/* turn on tof sensor */
		if (data->enable_ps_sensor == 0) {
			/* to start */
			stmvl53l0_start(data, 3, NORMAL_MODE);
		} else {
			vl53l0_errmsg("Already enabled. Skip !");
		}
	} else {
		/* turn off tof sensor */
		if (data->enable_ps_sensor == 1) {
			data->enable_ps_sensor = 0;
			/* to stop */
			stmvl53l0_stop(data);
		}
	}
	vl53l0_dbgmsg("End\n");
	mutex_unlock(&data->work_mutex);

	return count;
}

static DEVICE_ATTR(enable_ps_sensor, 0664/*S_IWUGO | S_IRUGO*/,
				   stmvl53l0_show_enable_ps_sensor,
					stmvl53l0_store_enable_ps_sensor);

static ssize_t stmvl53l0_show_enable_debug(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);

	//return snprintf(buf, sizeof(buf), "%d\n", data->enableDebug);
	
	return snprintf(buf, 8, "%d\n", data->enableDebug);
}

/* for debug */
static ssize_t stmvl53l0_store_enable_debug(struct device *dev,
					struct device_attribute *attr, const
					char *buf, size_t count)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	long on = simple_strtoul(buf, NULL, 10);

	if ((on != 0) &&  (on != 1)) {
		vl53l0_errmsg("set debug=%ld\n", on);
		return count;
	}
	data->enableDebug = on;

	return count;
}

/* DEVICE_ATTR(name,mode,show,store) */
static DEVICE_ATTR(enable_debug, 0660/*S_IWUSR | S_IRUGO*/,
				   stmvl53l0_show_enable_debug,
					stmvl53l0_store_enable_debug);

static ssize_t stmvl53l0_show_set_delay_ms(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
//	return snprintf(buf, sizeof(buf), "%d\n", data->delay_ms);

	return snprintf(buf, 16, "%d\n", data->delay_ms);
}

/* for work handler scheduler time */
static ssize_t stmvl53l0_store_set_delay_ms(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct stmvl53l0_data *data = dev_get_drvdata(dev);
	long delay_ms = simple_strtoul(buf, NULL, 10);

	if (delay_ms == 0) {
		vl53l0_errmsg("set delay_ms=%ld\n", delay_ms);
		return count;
	}
	mutex_lock(&data->work_mutex);
	data->delay_ms = delay_ms;
	mutex_unlock(&data->work_mutex);

	return count;
}

/* DEVICE_ATTR(name,mode,show,store) */
static DEVICE_ATTR(set_delay_ms, 0660/*S_IWUGO | S_IRUGO*/,
				   stmvl53l0_show_set_delay_ms,
					stmvl53l0_store_set_delay_ms);

static struct attribute *stmvl53l0_attributes[] = {
	&dev_attr_modelid.attr,
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_debug.attr,
	&dev_attr_set_delay_ms.attr ,
	NULL
};


static const struct attribute_group stmvl53l0_attr_group = {
	.attrs = stmvl53l0_attributes,
};

/*
 * misc device file operation functions
 */
static int stmvl53l0_ioctl_handler(struct file *file,
			unsigned int cmd, unsigned long arg,
			void __user *p)
{
	int rc = 0;
	unsigned int xtalkint = 0;
	unsigned int targetDistance = 0;
	int8_t offsetint = 0;
	struct stmvl53l0_data *data =
			container_of(file->private_data,
				struct stmvl53l0_data, miscdev);
	struct stmvl53l0_register reg;
	struct stmvl53l0_parameter parameter;
	VL53L0_DEV vl53l0_dev = data;
	uint8_t page_num = 0;

	if (!data)
		return -EINVAL;

	vl53l0_dbgmsg("Enter enable_ps_sensor:%d\n", data->enable_ps_sensor);
	switch (cmd) {
	/* enable */
	case VL53L0_IOCTL_INIT:
		vl53l0_dbgmsg("VL53L0_IOCTL_INIT\n");
		/* turn on tof sensor only if it's not enabled by other
		client */
		if (data->enable_ps_sensor == 0) {
			/* to start */
			rc = stmvl53l0_start(data, 3, NORMAL_MODE);
		} else
			rc = -EINVAL;
		break;
	/* crosstalk calibration */
	case VL53L0_IOCTL_XTALKCALB:
		vl53l0_dbgmsg("VL53L0_IOCTL_XTALKCALB\n");
		data->xtalkCalDistance = 100;
		if (copy_from_user(&targetDistance, (unsigned int *)p,
			sizeof(unsigned int))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		data->xtalkCalDistance = targetDistance;

		/* turn on tof sensor only if it's not enabled by other
		client */
		if (data->enable_ps_sensor == 0) {
			/* to start */
			stmvl53l0_start(data, 3, XTALKCALIB_MODE);
		} else
			rc = -EINVAL;
		break;
	/* set up Xtalk value */
	case VL53L0_IOCTL_SETXTALK:
		vl53l0_dbgmsg("VL53L0_IOCTL_SETXTALK\n");
		if (copy_from_user(&xtalkint, (unsigned int *)p,
			sizeof(unsigned int))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		vl53l0_dbgmsg("SETXTALK as 0x%x\n", xtalkint);
#ifdef CALIBRATION_FILE
		xtalk_calib = xtalkint;
		stmvl53l0_write_xtalk_calibration_file();
#endif
/* later
		VL6180x_SetXTalkCompensationRate(vl53l0_dev, xtalkint);
*/
		break;
	/* offset calibration */
	case VL53L0_IOCTL_OFFCALB:
		vl53l0_dbgmsg("VL53L0_IOCTL_OFFCALB\n");
		data->offsetCalDistance = 50;
		if (copy_from_user(&targetDistance, (unsigned int *)p,
			sizeof(unsigned int))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		data->offsetCalDistance = targetDistance;
		if (data->enable_ps_sensor == 0) {
			/* to start */
			stmvl53l0_start(data, 3, OFFSETCALIB_MODE);
		} else
			rc = -EINVAL;
		break;
	/* set up offset value */
	case VL53L0_IOCTL_SETOFFSET:
		vl53l0_dbgmsg("VL53L0_IOCTL_SETOFFSET\n");
		if (copy_from_user(&offsetint, (int8_t *)p, sizeof(int8_t))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		vl53l0_dbgmsg("SETOFFSET as %d\n", offsetint);
#ifdef CALIBRATION_FILE
		offset_calib = offsetint;
		stmvl53l0_write_offset_calibration_file();
#endif
/* later
		VL6180x_SetOffsetCalibrationData(vl53l0_dev, offsetint);
*/
		break;
	/* disable */
	case VL53L0_IOCTL_STOP:
		vl53l0_dbgmsg("VL53L0_IOCTL_STOP\n");
		/* turn off tof sensor only if it's enabled by other client */
		if (data->enable_ps_sensor == 1) {
			data->enable_ps_sensor = 0;
			/* to stop */
			stmvl53l0_stop(data);
		}
		break;
	/* Get all range data */
	case VL53L0_IOCTL_GETDATAS:
		vl53l0_dbgmsg("VL53L0_IOCTL_GETDATAS\n");
		if (copy_to_user((VL53L0_RangingMeasurementData_t *)p,
			&(data->rangeData),
			sizeof(VL53L0_RangingMeasurementData_t))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		break;
	/* Register tool */
	case VL53L0_IOCTL_REGISTER:
		vl53l0_dbgmsg("VL53L0_IOCTL_REGISTER\n");
		if (copy_from_user(&reg, (struct stmvl53l0_register *)p,
			sizeof(struct stmvl53l0_register))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		reg.status = 0;
		page_num = (uint8_t)((reg.reg_index & 0x0000ff00) >> 8);
		vl53l0_dbgmsg(
"VL53L0_IOCTL_REGISTER,	page number:%d\n", page_num);
		if (page_num != 0)
			reg.status = VL53L0_WrByte(vl53l0_dev, 0xFF, page_num);

		switch (reg.reg_bytes) {
		case(4):
			if (reg.is_read)
				reg.status = VL53L0_RdDWord(vl53l0_dev,
					(uint8_t)reg.reg_index,
					&reg.reg_data);
			else
				reg.status = VL53L0_WrDWord(vl53l0_dev,
					(uint8_t)reg.reg_index,
					reg.reg_data);
			break;
		case(2):
			if (reg.is_read)
				reg.status = VL53L0_RdWord(vl53l0_dev,
					(uint8_t)reg.reg_index,
					(uint16_t *)&reg.reg_data);
			else
				reg.status = VL53L0_WrWord(vl53l0_dev,
					(uint8_t)reg.reg_index,
					(uint16_t)reg.reg_data);
			break;
		case(1):
			if (reg.is_read)
				reg.status = VL53L0_RdByte(vl53l0_dev,
					(uint8_t)reg.reg_index,
					(uint8_t *)&reg.reg_data);
			else
				reg.status = VL53L0_WrByte(vl53l0_dev,
					(uint8_t)reg.reg_index,
					(uint8_t)reg.reg_data);
			break;
		default:
			reg.status = -1;

		}
		if (page_num != 0)
			reg.status = VL53L0_WrByte(vl53l0_dev, 0xFF, 0);


		if (copy_to_user((struct stmvl53l0_register *)p, &reg,
				sizeof(struct stmvl53l0_register))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		break;
	/* parameter access */
	case VL53L0_IOCTL_PARAMETER:
		vl53l0_dbgmsg("VL53L0_IOCTL_PARAMETER\n");
		if (copy_from_user(&parameter, (struct stmvl53l0_parameter *)p,
				sizeof(struct stmvl53l0_parameter))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		parameter.status = 0;
		switch (parameter.name) {
		case (OFFSET_PAR):
			if (parameter.is_read)
				parameter.status =
					papi_func_tbl->GetOffsetCalibrationDataMicroMeter(
						vl53l0_dev, &parameter.value);
			else
				parameter.status =
					papi_func_tbl->SetOffsetCalibrationDataMicroMeter(
						vl53l0_dev, parameter.value);
			vl53l0_dbgmsg("get parameter value as %d\n",parameter.value);
			break;
		case (XTALKRATE_PAR):
			if (parameter.is_read)
				parameter.status =
					papi_func_tbl->GetXTalkCompensationRateMegaCps(
						vl53l0_dev, (FixPoint1616_t *)
						&parameter.value);
			else
				parameter.status =
					papi_func_tbl->SetXTalkCompensationRateMegaCps(
						vl53l0_dev,
						(FixPoint1616_t)
							parameter.value);

			break;
		case (XTALKENABLE_PAR):
			if (parameter.is_read)
				parameter.status =
					papi_func_tbl->GetXTalkCompensationEnable(
						vl53l0_dev,
						(uint8_t *) &parameter.value);
			else
				parameter.status =
					papi_func_tbl->SetXTalkCompensationEnable(
						vl53l0_dev,
						(uint8_t) parameter.value);
			break;
		case (GPIOFUNC_PAR):
			if (parameter.is_read) {
				parameter.status =
					papi_func_tbl->GetGpioConfig(vl53l0_dev, 0, 0,
						&data->gpio_function,
						&data->gpio_polarity);
				parameter.value = data->gpio_function;
			} else {
				data->gpio_function = parameter.value;
				parameter.status =
					papi_func_tbl->SetGpioConfig(vl53l0_dev, 0, 0,
						data->gpio_function,
						data->gpio_polarity);
			}
			break;
		case (LOWTHRESH_PAR):
			if (parameter.is_read) {
				parameter.status =
					papi_func_tbl->GetInterruptThresholds(vl53l0_dev, 0, 
					&(data->low_threshold), &(data->high_threshold));
				parameter.value = data->low_threshold >> 16;
			} else {
				data->low_threshold = parameter.value << 16;
				parameter.status =
					papi_func_tbl->SetInterruptThresholds(vl53l0_dev, 0, 
					data->low_threshold, data->high_threshold);
			}
			break;
		case (HIGHTHRESH_PAR):
			if (parameter.is_read) {
				parameter.status =
					papi_func_tbl->GetInterruptThresholds(vl53l0_dev, 0, 
					&(data->low_threshold), &(data->high_threshold));
				parameter.value = data->high_threshold >> 16;
			} else {
				data->high_threshold = parameter.value << 16;
				parameter.status =
					papi_func_tbl->SetInterruptThresholds(vl53l0_dev, 0, 
					data->low_threshold, data->high_threshold);
			}
			break;
		case (DEVICEMODE_PAR):
			if (parameter.is_read) {
				parameter.status = 
					papi_func_tbl->GetDeviceMode(vl53l0_dev, 
						(VL53L0_DeviceModes*)&(parameter.value));
			} else {
				parameter.status =
					papi_func_tbl->SetDeviceMode(vl53l0_dev,
						(VL53L0_DeviceModes)(parameter.value));
				data->deviceMode = (VL53L0_DeviceModes)(parameter.value);
			}
			break;


		
		case (INTERMEASUREMENT_PAR):
			if (parameter.is_read) {
				parameter.status = 
					papi_func_tbl->GetInterMeasurementPeriodMilliSeconds(vl53l0_dev,
						(uint32_t*)&(parameter.value));
			} else {
				parameter.status =
					papi_func_tbl->SetInterMeasurementPeriodMilliSeconds(vl53l0_dev,
						(uint32_t)(parameter.value));
				data->interMeasurems = parameter.value;
			}
			break;


		}
		
		if (copy_to_user((struct stmvl53l0_parameter *)p, &parameter,
				sizeof(struct stmvl53l0_register))) {
			vl53l0_errmsg("%d, fail\n", __LINE__);
			return -EFAULT;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int stmvl53l0_open(struct inode *inode, struct file *file)
{
	return 0;
}

#if 0
static int stmvl53l0_flush(struct file *file, fl_owner_t id)
{
	struct stmvl53l0_data *data = container_of(file->private_data,
					struct stmvl53l0_data, miscdev);
	(void) file;
	(void) id;

	if (data) {
		if (data->enable_ps_sensor == 1) {
			/* turn off tof sensor if it's enabled */
			data->enable_ps_sensor = 0;
			/* to stop */
			stmvl53l0_stop(data);
		}
	}
	return 0;
}
#endif
static long stmvl53l0_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int ret;
	struct stmvl53l0_data *data =
			container_of(file->private_data,
					struct stmvl53l0_data, miscdev);
	mutex_lock(&data->work_mutex);
	ret = stmvl53l0_ioctl_handler(file, cmd, arg, (void __user *)arg);
	mutex_unlock(&data->work_mutex);

	return ret;
}

/*
 * Initialization function
 */
static int stmvl53l0_init_client(struct stmvl53l0_data *data)
{
	uint8_t id = 0, type = 0;
	uint8_t revision = 0, module_id = 0;
	VL53L0_Error Status = VL53L0_ERROR_NONE;
	VL53L0_DeviceInfo_t DeviceInfo;
	VL53L0_DEV vl53l0_dev = data;
	FixPoint1616_t	SigmaLimitValue;

	vl53l0_dbgmsg("Enter\n");

	/* Read Model ID */
	VL53L0_RdByte(vl53l0_dev, VL53L0_REG_IDENTIFICATION_MODEL_ID, &id);
	vl53l0_errmsg("read MODEL_ID: 0x%x\n", id);
	if (id == 0xee) {
		vl53l0_errmsg("STM VL53L0 Found\n");
	} else if (id == 0) {
		vl53l0_errmsg("Not found STM VL53L0\n");
		return -EIO;
	}
	VL53L0_RdByte(vl53l0_dev, 0xc1, &id);
	vl53l0_errmsg("read 0xc1: 0x%x\n", id);
	VL53L0_RdByte(vl53l0_dev, 0xc2, &id);
	vl53l0_errmsg("read 0xc2: 0x%x\n", id);
	VL53L0_RdByte(vl53l0_dev, 0xc3, &id);
	vl53l0_errmsg("read 0xc3: 0x%x\n", id);

	/* Read Model Version */
	VL53L0_RdByte(vl53l0_dev, 0xC0, &type);
	VL53L0_RdByte(vl53l0_dev, VL53L0_REG_IDENTIFICATION_REVISION_ID,
		&revision);
	VL53L0_RdByte(vl53l0_dev, 0xc3,
		&module_id);
	vl53l0_errmsg("STM VL53L0 Model type : %d. rev:%d. module:%d\n", type,
		revision, module_id);

	vl53l0_dev->I2cDevAddr      = 0x52;
	vl53l0_dev->comms_type      =  1;
	vl53l0_dev->comms_speed_khz =  400;

	/* Setup API functions based on revision */
	stmvl53l0_setupAPIFunctions(data);

	if (Status == VL53L0_ERROR_NONE && data->reset) {
		pr_err("Call of VL53L0_DataInit\n");
		Status = papi_func_tbl->DataInit(vl53l0_dev); /* Data initialization */
		data->reset = 0;
	}

	if (Status == VL53L0_ERROR_NONE) {
		pr_err("VL53L0_GetDeviceInfo:\n");
		Status = papi_func_tbl->GetDeviceInfo(vl53l0_dev, &DeviceInfo);
		if (Status == VL53L0_ERROR_NONE) {
			pr_err("Device Name : %s\n", DeviceInfo.Name);
			pr_err("Device Type : %s\n", DeviceInfo.Type);
			pr_err("Device ID : %s\n", DeviceInfo.ProductId);
			pr_err("Product type: %d\n", DeviceInfo.ProductType);
			pr_err("ProductRevisionMajor : %d\n",
				DeviceInfo.ProductRevisionMajor);
			pr_err("ProductRevisionMinor : %d\n",
				DeviceInfo.ProductRevisionMinor);
		}
	}

	if (Status == VL53L0_ERROR_NONE) {
		pr_err("Call of VL53L0_StaticInit\n");
		Status = papi_func_tbl->StaticInit(vl53l0_dev);
		/* Device Initialization */
	}

	if (Status == VL53L0_ERROR_NONE) {

		pr_err("Call of VL53L0_SetDeviceMode\n");
		Status = papi_func_tbl->SetDeviceMode(vl53l0_dev,
					VL53L0_DEVICEMODE_SINGLE_RANGING);
		/* Setup in	single ranging mode */
	}

	if (Status == VL53L0_ERROR_NONE) {
		pr_err("set LimitCheckValue SIGMA_FINAL_RANGE\n");
		SigmaLimitValue = 32 << 16;
		Status = papi_func_tbl->SetLimitCheckValue(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE,
					SigmaLimitValue);
	}

	if (Status == VL53L0_ERROR_NONE) {
		pr_err("set LimitCheckValue SIGNAL_RATE_FINAL_RANGE\n");
		SigmaLimitValue = 16384; /* 0.25 * 65536 */
		Status = papi_func_tbl->SetLimitCheckValue(vl53l0_dev,
					VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
					SigmaLimitValue);
	}
		
	/*  Enable/Disable Sigma and Signal check */
	if (Status == VL53L0_ERROR_NONE)
		Status = papi_func_tbl->SetLimitCheckEnable(vl53l0_dev, 
					VL53L0_CHECKENABLE_SIGMA_FINAL_RANGE, 1);

	if (Status == VL53L0_ERROR_NONE)
		Status = papi_func_tbl->SetLimitCheckEnable(vl53l0_dev, 
					VL53L0_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);

	if (Status == VL53L0_ERROR_NONE)
		Status = papi_func_tbl->SetWrapAroundCheckEnable(vl53l0_dev, 1);

#ifdef CALIBRATION_FILE
	/*stmvl53l0_read_calibration_file(data);*/
#endif

	vl53l0_dbgmsg("End\n");

	return 0;
}

static int stmvl53l0_start(struct stmvl53l0_data *data, uint8_t scaling,
	init_mode_e mode)
{
	int rc = 0;
	VL53L0_DEV vl53l0_dev = data;

	vl53l0_dbgmsg("Enter\n");

	/* Power up */
	rc = pmodule_func_tbl->power_up(data->client_object, &data->reset);
	if (rc) {
		vl53l0_errmsg("%d,error rc %d\n", __LINE__, rc);
		return rc;
	}
	/* init */
	rc = stmvl53l0_init_client(data);
	if (rc) {
		vl53l0_errmsg("%d, error rc %d\n", __LINE__, rc);
		pmodule_func_tbl->power_down(data->client_object);
		return -EINVAL;
	}

	/* check mode */
	if (mode != NORMAL_MODE)
		papi_func_tbl->SetXTalkCompensationEnable(vl53l0_dev, 0);

	if (mode == OFFSETCALIB_MODE) {
		/*VL53L0_SetOffsetCalibrationDataMicroMeter(vl53l0_dev, 0);*/
		FixPoint1616_t OffsetMicroMeter;
		papi_func_tbl->PerformOffsetCalibration(vl53l0_dev,
			(data->offsetCalDistance<<16),
			&OffsetMicroMeter);
		pr_err("Offset calibration:%u\n", OffsetMicroMeter);
		return rc;
	}
	else if (mode == XTALKCALIB_MODE) {
		FixPoint1616_t XTalkCompensationRateMegaCps;
		/*caltarget distance : 100mm and convert to
		* fixed point 16 16 format
		*/
		papi_func_tbl->PerformXTalkCalibration(vl53l0_dev,
			(data->xtalkCalDistance<<16),
			&XTalkCompensationRateMegaCps);
		pr_err("Xtalk calibration:%u\n", XTalkCompensationRateMegaCps);
		return rc;
	}
	/* set up device parameters */
	data->gpio_polarity = VL53L0_INTERRUPTPOLARITY_LOW;	

	papi_func_tbl->SetGpioConfig(vl53l0_dev, 0, 0,
		data->gpio_function,
		VL53L0_INTERRUPTPOLARITY_LOW);

	papi_func_tbl->SetInterruptThresholds(vl53l0_dev, 0,
		data->low_threshold, data->high_threshold);

	papi_func_tbl->SetInterMeasurementPeriodMilliSeconds(vl53l0_dev,
			data->interMeasurems); 

	pr_err("DeviceMode:0x%x, interMeasurems:%d==\n", data->deviceMode,
			data->interMeasurems);
	papi_func_tbl->SetDeviceMode(vl53l0_dev,
			data->deviceMode);
	papi_func_tbl->ClearInterruptMask(vl53l0_dev,
							0);
	/* start the ranging */
	papi_func_tbl->StartMeasurement(vl53l0_dev);
	data->enable_ps_sensor = 1;

	/* enable work handler */
	stmvl53l0_schedule_handler(data);
	vl53l0_dbgmsg("End\n");

	return rc;
}

static int stmvl53l0_stop(struct stmvl53l0_data *data)
{
	int rc = 0;
	VL53L0_DEV vl53l0_dev = data;

	vl53l0_dbgmsg("Enter\n");

	/* stop - if continuous mode */
	if (data->deviceMode == VL53L0_DEVICEMODE_CONTINUOUS_RANGING ||
		data->deviceMode == VL53L0_DEVICEMODE_CONTINUOUS_TIMED_RANGING)
		papi_func_tbl->StopMeasurement(vl53l0_dev);

	/* clean interrupt */
	papi_func_tbl->ClearInterruptMask(vl53l0_dev, 0);

	/* cancel work handler */
	stmvl53l0_cancel_handler(data);
	/* power down */
	rc = pmodule_func_tbl->power_down(data->client_object);
	if (rc) {
		vl53l0_errmsg("%d, error rc %d\n", __LINE__, rc);
		return rc;
	}
	vl53l0_dbgmsg("End\n");

	return rc;
}

/*
 * I2C init/probing/exit functions
 */
static const struct file_operations stmvl53l0_ranging_fops = {
	.owner =			THIS_MODULE,
	.unlocked_ioctl =	stmvl53l0_ioctl,
	.open =				stmvl53l0_open,
	//.flush =			stmvl53l0_flush,
};

/*
static struct miscdevice stmvl53l0_ranging_dev = {
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		"stmvl53l0_ranging",
	.fops =		&stmvl53l0_ranging_fops
};
*/

int stmvl53l0_setup(struct stmvl53l0_data *data)
{
	int rc = 0;

#ifdef USE_INT
	int irq = 0;
#endif

	vl53l0_dbgmsg("Enter\n");

	/* init mutex */
	mutex_init(&data->update_lock);
	mutex_init(&data->work_mutex);

#ifdef USE_INT
	/* init interrupt */
	gpio_request(IRQ_NUM, "vl53l0_gpio_int");
	gpio_direction_input(IRQ_NUM);
	irq = gpio_to_irq(IRQ_NUM);
	if (irq < 0) {
		vl53l0_errmsg("filed to map GPIO: %d to interrupt:%d\n",
			IRQ_NUM, irq);
	} else {
		vl53l0_dbgmsg("register_irq:%d\n", irq);
		/* IRQF_TRIGGER_FALLING- poliarity:0 IRQF_TRIGGER_RISNG -
		poliarty:1 */
		rc = request_threaded_irq(irq, NULL,
				stmvl53l0_interrupt_handler,
				IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
				"vl53l0_interrupt",
				(void *)data);
		if (rc) {
			vl53l0_errmsg(
"%d, Could not allocate STMVL53L0_INT ! result:%d\n",  __LINE__, rc);
#ifdef USE_INT
			free_irq(irq, data);
#endif
			goto exit_free_irq;
		}
	}
	data->irq = irq;
	vl53l0_errmsg("interrupt is hooked\n");
#endif

	/* init work handler */
	INIT_DELAYED_WORK(&data->dwork, stmvl53l0_work_handler);

	/* Register to Input Device */
	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		rc = -ENOMEM;
		vl53l0_errmsg("%d error:%d\n", __LINE__, rc);
/*
#ifdef USE_INT
		free_irq(irq, data);
#endif
*/
		goto exit_free_irq;
	}
	set_bit(EV_ABS, data->input_dev_ps->evbit);
	/* range in cm*/
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 76, 0, 0);
	/* tv_sec */
	input_set_abs_params(data->input_dev_ps, ABS_HAT0X, 0, 0xffffffff,
		0, 0);
	/* tv_usec */
	input_set_abs_params(data->input_dev_ps, ABS_HAT0Y, 0, 0xffffffff,
		0, 0);
	/* range in_mm */
	input_set_abs_params(data->input_dev_ps, ABS_HAT1X, 0, 765, 0, 0);
	/* error code change maximum to 0xff for more flexibility */
	input_set_abs_params(data->input_dev_ps, ABS_HAT1Y, 0, 0xff, 0, 0);
	/* rtnRate */
	input_set_abs_params(data->input_dev_ps, ABS_HAT2X, 0, 0xffffffff,
		0, 0);
	/* rtn_amb_rate */
	input_set_abs_params(data->input_dev_ps, ABS_HAT2Y, 0, 0xffffffff,
		0, 0);
	/* rtn_conv_time */
	input_set_abs_params(data->input_dev_ps, ABS_HAT3X, 0, 0xffffffff,
		0, 0);
	/* dmax */
	input_set_abs_params(data->input_dev_ps, ABS_HAT3Y, 0, 0xffffffff,
		0, 0);
	data->input_dev_ps->name = "STM VL53L0 proximity sensor";

	rc = input_register_device(data->input_dev_ps);
	if (rc) {
		rc = -ENOMEM;
		vl53l0_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_free_dev_ps;
	}
	/* setup drv data */
	input_set_drvdata(data->input_dev_ps, data);

	/* Register sysfs hooks */
	data->range_kobj = kobject_create_and_add("range", kernel_kobj);
	if (!data->range_kobj) {
		rc = -ENOMEM;
		vl53l0_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_unregister_dev_ps;
	}
	//data->range_kobj = &data->input_dev_ps->dev.kobj;
	rc = sysfs_create_group(&data->input_dev_ps->dev.kobj,
			&stmvl53l0_attr_group);
	if (rc) {
		rc = -ENOMEM;
		vl53l0_errmsg("%d error:%d\n", __LINE__, rc);
		goto exit_unregister_dev_ps_1;
	}

	/* to register as a misc device */
	data->miscdev.minor = MISC_DYNAMIC_MINOR;
	data->miscdev.name = "stmvl53l0_ranging";
	data->miscdev.fops = &stmvl53l0_ranging_fops;
	vl53l0_errmsg("Misc device registration name:%s\n", data->dev_name);
	if (misc_register(&data->miscdev) != 0)
		vl53l0_errmsg(
"Could not register misc. dev for stmvl53l0	ranging\n");

	/* init default device parameter value */
	data->enable_ps_sensor = 0;
	data->reset = 1;
	data->delay_ms = 30;	/* delay time to 30ms */
	data->enableDebug = 0;
	data->gpio_polarity = VL53L0_INTERRUPTPOLARITY_LOW;
	data->gpio_function = VL53L0_GPIOFUNCTIONALITY_NEW_MEASURE_READY;
	data->low_threshold = 60;
	data->high_threshold = 200;
	data->deviceMode = VL53L0_DEVICEMODE_SINGLE_RANGING;
	data->interMeasurems = 30;

	vl53l0_dbgmsg("support ver. %s enabled\n", DRIVER_VERSION);
	vl53l0_dbgmsg("End");

	return 0;
exit_unregister_dev_ps_1:
	kobject_put(data->range_kobj);
exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
exit_free_dev_ps:
	input_free_device(data->input_dev_ps);
exit_free_irq:
#ifdef USE_INT
	free_irq(irq, data);
#endif
	kfree(data);
	return rc;
}

static int __init stmvl53l0_init(void)
{
	int ret = -1;

	vl53l0_dbgmsg("Enter\n");

	/* assign function table */
	pmodule_func_tbl = &stmvl53l0_module_func_tbl;
	papi_func_tbl = &stmvl53l0_api_func_tbl;

	/* client specific init function */
	ret = pmodule_func_tbl->init();

	if (ret)
		vl53l0_errmsg("%d failed with %d\n", __LINE__, ret);

	vl53l0_dbgmsg("End\n");

	return ret;
}

static void __exit stmvl53l0_exit(void)
{
	vl53l0_dbgmsg("Enter\n");

	vl53l0_dbgmsg("End\n");
}


MODULE_AUTHOR("STMicroelectronics Imaging Division");
MODULE_DESCRIPTION("ST FlightSense Time-of-Flight sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(stmvl53l0_init);
module_exit(stmvl53l0_exit);

