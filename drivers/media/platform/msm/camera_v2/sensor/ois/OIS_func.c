/////////////////////////////////////////////////////////////////////////////
// File Name	: OIS_func.c
// Function		: Various function for OIS control
// Rule         : Use TAB 4
//
// Copyright(c)	Rohm Co.,Ltd. All rights reserved
//
/***** ROHM Confidential ***************************************************/
/*
#define	_USE_MATH_DEFINES							// RHM_HT 2013.03.24	Add for using "M_PI" in math.h (VS2008)
#include <math.h>
#include <conio.h>
#include <ctype.h>
*/

#include "OIS_head.h"
#include "OIS_prog.h"
#include "OIS_coef.h"
#include "OIS_defi.h"
#include "Ois.h"
//#include "usb_func.h"			//Darcy mask/20140620

static struct msm_camera_i2c_client *g_i2c_ctrl;

extern	OIS_UWORD	OIS_REQUEST;			// OIS control register.
// ==> RHM_HT 2013.03.04	Change type (OIS_UWORD -> double)
extern	double		OIS_PIXEL[2];			// Just Only use for factory adjustment.
// <== RHM_HT 2013.03.04

// ==> RHM_HT 2013.03.13	add for HALL_SENSE_ADJUST
extern	OIS_WORD	CROP_X;								// x start position for cropping
extern	OIS_WORD	CROP_Y;								// y start position for cropping
extern	OIS_WORD	CROP_WIDTH;							// cropping width
extern	OIS_WORD 	CROP_HEIGHT;						// cropping height
extern	OIS_UBYTE	SLICE_LEVE;							// slice level of bitmap binalization

extern	double		DISTANCE_BETWEEN_CIRCLE;			// distance between center of each circle (vertical and horizontal) [mm]
extern	double		DISTANCE_TO_CIRCLE;					// distance to the circle [mm]
extern	double		D_CF;								// Correction Factor for distance to the circle
// ==> RHM_HT 2013/07/10	Added new user definition variables for DC gain check
extern	OIS_UWORD 	ACT_DRV;							// [mV]: Full Scale of OUTPUT DAC.
extern	OIS_UWORD 	FOCAL_LENGTH;						// [um]: Focal Length 3.83mm
extern	double 		MAX_OIS_SENSE;						// [um/mA]: per actuator difinition (change to absolute value)
extern	double		MIN_OIS_SENSE;						// [um/mA]: per actuator difinition (change to absolute value)
extern	OIS_UWORD 	MAX_COIL_R;							// [ohm]: Max value of coil resistance
extern	OIS_UWORD 	MIN_COIL_R;							// [ohm]: Min value of coil resistance
// <== RHM_HT 2013/07/10	Added new user definition variables


// ==> RHM_HT 2013/11/25	Modified
OIS_UWORD 		u16_ofs_tbl[] = {						// RHM_HT 2013.03.13	[Improvement of Loop Gain Adjust] Change to global variable
// 					0x0FBC,								// 1	For MITSUMI
					0x0DFC,								// 2
// 					0x0C3D,								// 3
					0x0A7D,								// 4
// 					0x08BD,								// 5
					0x06FE,								// 6
// 					0x053E,								// 7
					0x037F,								// 8
// 					0x01BF,								// 9
					0x0000,								// 10
// 					0xFE40,								// 11
					0xFC80,								// 12
// 					0xFAC1,								// 13
					0xF901,								// 14
// 					0xF742,								// 15
					0xF582,								// 16
// 					0xF3C2,								// 17
					0xF203,								// 18
// 					0xF043								// 19
};
// <== RHM_HT 2013/11/25	Modified

// <== RHM_HT 2013.03.13

/* 2015-12-12 add-s */
int  Ois_rohm_func_SetI2cClient(struct msm_camera_i2c_client *client)
{
	if (!client) {
		pr_err("%s: i2c client is NULL !!!\n", __func__);
		return -1;
	} else {
		g_i2c_ctrl = client;
	}
	return 0;
}
/* 2015-12-12 add-e */

//  *****************************************************
//  **** Program Download Function
//  *****************************************************
ADJ_STS		func_PROGRAM_DOWNLOAD( void ){	// RHM_HT 2013/04/15	Change "typedef" of return value

	OIS_UWORD	sts;						// RHM_HT 2013/04/15	Change "typedef".

	download( 0, 0 );						// Program Download
	sts = I2C_OIS_mem__read( _M_OIS_STS );	// Check Status

	if ( ( sts & 0x0004 ) == 0x0004 ){
		// ==> RHM_HT 2013/07/10	Added
		OIS_UWORD u16_dat;

		u16_dat = I2C_OIS_mem__read( _M_FIRMVER );
//		DEBUG_printf(("Firm Ver :      %4d\n\n", u16_dat ));				//Darcy mask/20140620
		// <== RHM_HT 2013/07/10	Added

		return ADJ_OK;						// Success				RHM_HT 2013/04/15	Change return value.
	}
	else{
		return PROG_DL_ERR;					// FAIL					RHM_HT 2013/04/15	Change return value.
	}
}


// ==> RHM_HT 2013/11/26	Reverted
//  *****************************************************
//  **** COEF Download function
//  *****************************************************
void	func_COEF_DOWNLOAD( OIS_UWORD u16_coef_type ){
	OIS_UWORD	sts;						// RHM_HT 2013/04/15	Change "typedef".

//	OIS_UWORD u16_i, u16_dat;

	download( 1, u16_coef_type );			// COEF Download

	sts = I2C_OIS_mem__read( _M_CEFTYP);	// Check Status
	pr_err("[%s:%d] sts = %d \n", __func__, __LINE__, sts);
/*	//Darcy Mask
	for( u16_i = 1; u16_i <= 256; u16_i++){
		u16_dat = I2C_OIS_mem__read( u16_i-1 );
		//  memory dump around start area and end area
		if( ( u16_i < 3 ) || ( u16_i > 253 ) ){
			DEBUG_printf(("M[%.2x] %.4X\n",u16_i-1, u16_dat ));
		}
		//
		if( u16_i == 128 ){
			DEBUG_printf(("  ... \n" ));
		}
		// Coef type
		if( (u16_i-1) == _M_CEFTYP ){
			DEBUG_printf(("COEF     M[%.2x] %.4X\n",u16_i-1, u16_dat ));
		}
	}
*/														
}

extern uint8_t module_id;
extern uint8_t otp_date_buf[3];
//#define SUNNY_MODULE 0x02
//#define LG_MODULE  0x03
// <== RHM_HT 2013/11/26	Reverted


//  *****************************************************
//  **** Download the data
//  *****************************************************
void	download( OIS_UWORD u16_type, OIS_UWORD u16_coef_type ){

	// Data Transfer Size per one I2C access
	#define		DWNLD_TRNS_SIZE		(6)

	OIS_UBYTE	temp[DWNLD_TRNS_SIZE+1];
	OIS_UWORD	block_cnt;
	OIS_UWORD	total_cnt;
	OIS_UWORD	lp;
	OIS_UWORD	n;
	OIS_UWORD	u16_i;
        pr_err("%s,module_id:0x%x year:%d,month;%d,day:%d",__func__,module_id,otp_date_buf[0],otp_date_buf[1],otp_date_buf[2]);
	if (module_id == LG_MODULE) {
		if ( u16_type == 0 ){
		if( otp_date_buf[0] == 16 && (otp_date_buf[1] < 2 || (otp_date_buf[1] == 2 && otp_date_buf[2] < 28)))
				n = DOWNLOAD_BIN_LG_LEN_1;
			else
				n = DOWNLOAD_BIN_LG_LEN_2;

		}
		else{
			//if( otp_date_buf[0] == 16 && otp_date_buf[1] < 3)
			if( otp_date_buf[0] == 16 && (otp_date_buf[1] < 2 || (otp_date_buf[1] == 2 && otp_date_buf[2] < 28)))
				n = DOWNLOAD_COEF_LG_LEN_1;
			else
				n = DOWNLOAD_COEF_LG_LEN_2;
		}
	}else if (module_id == SUNNY_MODULE)
	{

		if ( u16_type == 0 ){
			if( otp_date_buf[0] == 16 && otp_date_buf[1] < 3)
			n = DOWNLOAD_BIN_SUNNY_LEN_1;
			else
			n = DOWNLOAD_BIN_SUNNY_LEN_2;
		}
		else{
			if( otp_date_buf[0] == 16 && otp_date_buf[1] < 3)
			n = DOWNLOAD_COEF_SUNNY_LEN_1;
			else
			n = DOWNLOAD_COEF_SUNNY_LEN_2;
		}
	}
	block_cnt	= n / DWNLD_TRNS_SIZE + 1;
	total_cnt	= block_cnt;

	while( 1 ){
		// Residual Number Check
		if( block_cnt == 1 ){
			lp = n % DWNLD_TRNS_SIZE;
		}
		else{
			lp = DWNLD_TRNS_SIZE;
		}

		// Transfer Data set
		if( lp != 0 ){
			if( u16_type == 0 ){
				temp[0] = _OP_FIRM_DWNLD;
				if ( module_id == LG_MODULE ){
					//pr_err("xxxxdownload lg prog");
					//if( otp_date_buf[0] == 16 && otp_date_buf[1] < 3){
					if( otp_date_buf[0] == 16 && (otp_date_buf[1] < 2 || (otp_date_buf[1] == 2 && otp_date_buf[2] < 28))){
						for( u16_i = 1; u16_i <= lp; u16_i += 1 ){
							temp[ u16_i ] = DOWNLOAD_BIN_LG_1[ ( total_cnt - block_cnt ) * DWNLD_TRNS_SIZE + u16_i - 1 ];
						}
					}else{

						for( u16_i = 1; u16_i <= lp; u16_i += 1 ){
							temp[ u16_i ] = DOWNLOAD_BIN_LG_2[ ( total_cnt - block_cnt ) * DWNLD_TRNS_SIZE + u16_i - 1 ];
						}
					}
				}else if ( module_id == SUNNY_MODULE ){
						//pr_err("xxxxdownload sunny prog");
					if( otp_date_buf[0] == 16 && otp_date_buf[1] < 3){
						for( u16_i = 1; u16_i <= lp; u16_i += 1 ){
							temp[ u16_i ] = DOWNLOAD_BIN_SUNNY_1[ ( total_cnt - block_cnt ) * DWNLD_TRNS_SIZE + u16_i - 1 ];
						}
					}else{
						for( u16_i = 1; u16_i <= lp; u16_i += 1 ){
							temp[ u16_i ] = DOWNLOAD_BIN_SUNNY_2[ ( total_cnt - block_cnt ) * DWNLD_TRNS_SIZE + u16_i - 1 ];
						}

					}
				}
			}
			else{
				temp[0] = _OP_COEF_DWNLD;
				if (module_id == LG_MODULE){
					//pr_err("xxxxdownload lg coef");
					//if( otp_date_buf[0] == 16 && otp_date_buf[1] < 3){
					if( otp_date_buf[0] == 16 && (otp_date_buf[1] < 2 || (otp_date_buf[1] == 2 && otp_date_buf[2] < 28))){
						for( u16_i = 1; u16_i <= lp; u16_i += 1 ){
							temp[u16_i] = DOWNLOAD_COEF_LG_1[(total_cnt - block_cnt) * DWNLD_TRNS_SIZE + u16_i -1];	// RHM_HT 2013/07/10	Modified
						}
					}else{
						for( u16_i = 1; u16_i <= lp; u16_i += 1 ){
							temp[u16_i] = DOWNLOAD_COEF_LG_2[(total_cnt - block_cnt) * DWNLD_TRNS_SIZE + u16_i -1];	// RHM_HT 2013/07/10	Modified
						}
					}
				}else if (module_id == SUNNY_MODULE){
					//pr_err("xxxxdownload sunny coef");
					if( otp_date_buf[0] == 16 && otp_date_buf[1] < 3){
						for( u16_i = 1; u16_i <= lp; u16_i += 1 ){
							temp[u16_i] = DOWNLOAD_COEF_SUNNY_1[(total_cnt - block_cnt) * DWNLD_TRNS_SIZE + u16_i -1];	// RHM_HT 2013/07/10	Modified
						}
					}else{
						for( u16_i = 1; u16_i <= lp; u16_i += 1 ){
							temp[u16_i] = DOWNLOAD_COEF_SUNNY_2[(total_cnt - block_cnt) * DWNLD_TRNS_SIZE + u16_i -1];	// RHM_HT 2013/07/10	Modified
						}
					}
				}
			}
			// Data Transfer
			// 			WR_I2C( _SLV_OIS_, lp + 1, temp );
			//pr_err("DL_I2C block_cnt = %d \n", block_cnt);
			g_i2c_ctrl->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
			g_i2c_ctrl->i2c_func_tbl->i2c_write_seq(
					g_i2c_ctrl, temp[0], &(temp[1]), lp);

	}

		// Block Counter Decrement
		block_cnt = block_cnt - 1;
		if( block_cnt == 0 ){
			break;
		}
	}

	pr_err("DL_I2C block_cnt = %d \n", block_cnt);
}

// ==> RHM_HT 2015/01/08	Added
OIS_UWORD	INTG__INPUT;			// Integral Input value	szx_2014/12/24_2
OIS_UWORD	KGNTG_VALUE;			// KgxTG / KgyTG		szx_2014/12/24_2
// <== RHM_HT 2015/01/08	Added

void SET_FADJ_PARAM( const _FACT_ADJ *param )
{
	//*********************
	// HALL ADJUST
	//*********************
	// Set Hall Current DAC   value that is FACTORY ADJUSTED
	I2C_OIS_per_write( _P_30_ADC_CH0, param->gl_CURDAT );
	// Set Hall     PreAmp Offset   that is FACTORY ADJUSTED
	I2C_OIS_per_write( _P_31_ADC_CH1, param->gl_HALOFS_X );
	I2C_OIS_per_write( _P_32_ADC_CH2, param->gl_HALOFS_Y );
	// Set Hall-X/Y PostAmp Offset  that is FACTORY ADJUSTED
	I2C_OIS_mem_write( _M_X_H_ofs, param->gl_HX_OFS );
	I2C_OIS_mem_write( _M_Y_H_ofs, param->gl_HY_OFS );
	// Set Residual Offset          that is FACTORY ADJUSTED
	I2C_OIS_per_write( _P_39_Ch3_VAL_1, param->gl_PSTXOF );
	I2C_OIS_per_write( _P_3B_Ch3_VAL_3, param->gl_PSTYOF );

	//*********************
	// DIGITAL GYRO OFFSET
	//*********************
	I2C_OIS_mem_write( _M_Kgx00, param->gl_GX_OFS );
	I2C_OIS_mem_write( _M_Kgy00, param->gl_GY_OFS );
	I2C_OIS_mem_write( _M_TMP_X_, param->gl_TMP_X_ );
	I2C_OIS_mem_write( _M_TMP_Y_, param->gl_TMP_Y_ );

	//*********************
	// HALL SENSE
	//*********************
	// Set Hall Gain   value that is FACTORY ADJUSTED
	I2C_OIS_mem_write( _M_KgxHG, param->gl_KgxHG );
	I2C_OIS_mem_write( _M_KgyHG, param->gl_KgyHG );
	// Set Cross Talk Canceller
	I2C_OIS_mem_write( _M_KgxH0, param->gl_KgxH0 );
	I2C_OIS_mem_write( _M_KgyH0, param->gl_KgyH0 );

	//*********************
	// LOOPGAIN
	//*********************
	I2C_OIS_mem_write( _M_KgxG, param->gl_KGXG );
	I2C_OIS_mem_write( _M_KgyG, param->gl_KGYG );

// ==> RHM_HT 2015/01/08	Added
	// Get default Integ_input and KgnTG value
	INTG__INPUT = I2C_OIS_mem__read( 0x38 );
	KGNTG_VALUE = I2C_OIS_mem__read( _M_KgxTG );
// <== RHM_HT 2015/01/08	Added

	// Position Servo ON ( OIS OFF )
	I2C_OIS_mem_write( _M_EQCTL, 0x0C0C );
}

// ==> RHM_HT 2016/01/12	Added for CLAF
void SET_FADJ_PARAM_CLAF( const _FACT_ADJ_AF *param )
{
	I2C_OIS_per_write( _P_37_ADC_CH7,	param->gl_CURDAZ    );	// Hall Bias
	I2C_OIS_per_write( _P_36_ADC_CH6,	param->gl_HALOFS_Z  );	// Pre-amp offset
	I2C_OIS_per_write( _P_38_Ch3_VAL_0,	param->gl_PSTZOF    );	// Post-amp offset
	I2C_OIS_per_write( _P_M_HZOFS, 		param->gl_P_M_HZOFS );	// Digital offst
	I2C_OIS_per_write( _P_M_KzHG,  		param->gl_P_M_KzHG  );	// Hall Normalized gain
}
// <== RHM_HT 2016/01/12	Added for CLAF
// ==> RHM_HT 2016/01/18	Added for CLAF issue

void SET_FADJ_PARAM_CLAF_2( const _FACT_ADJ_AF *param )
{
        //OIS_UWORD temp;
	I2C_OIS_per_write( _P_30_ADC_CH0,	param->gl_CURDAZ    );	// Hall Bias
	I2C_OIS_per_write( _P_36_ADC_CH6,	param->gl_HALOFS_Z  );	// Pre-amp offset
	I2C_OIS_per_write( _P_38_Ch3_VAL_0,	param->gl_PSTZOF    );	// Post-amp offset
	I2C_OIS_per_write( _P_M_HZOFS, 		param->gl_P_M_HZOFS );	// Digital offst
	I2C_OIS_per_write( _P_M_KzHG,  		param->gl_P_M_KzHG  );	// Hall Normalized gain
/*
        temp = I2C_OIS_per__read(_P_30_ADC_CH0);
        pr_err("xxxx _P_30_ADC_CH0 = 0x%x",temp);
        temp = I2C_OIS_per__read(_P_36_ADC_CH6);
        pr_err("xxxx _P_30_ADC_CH6 = 0x%x",temp);
        temp = I2C_OIS_per__read(_P_38_Ch3_VAL_0);
        pr_err("xxxx _P_38_Ch3_VAL_0 = 0x%x",temp);
        temp = I2C_OIS_per__read(_P_M_HZOFS);
        pr_err("xxxx _P_M_HZOFS = 0x%x",temp);
        temp = I2C_OIS_per__read(_P_M_KzHG);
        pr_err("xxxx _P_M_KzHG = 0x%x",temp);
*/


}

void SET_FADJ_PARAM_2( const _FACT_ADJ *param )
{
	//*********************
	// HALL ADJUST
	//*********************
	// Set Hall Current DAC   value that is FACTORY ADJUSTED
	I2C_OIS_per_write( _P_37_ADC_CH7, param->gl_CURDAT );
	// Set Hall     PreAmp Offset   that is FACTORY ADJUSTED
	I2C_OIS_per_write( _P_31_ADC_CH1, param->gl_HALOFS_X );
	I2C_OIS_per_write( _P_32_ADC_CH2, param->gl_HALOFS_Y );
	// Set Hall-X/Y PostAmp Offset  that is FACTORY ADJUSTED
	I2C_OIS_mem_write( _M_X_H_ofs, param->gl_HX_OFS );
	I2C_OIS_mem_write( _M_Y_H_ofs, param->gl_HY_OFS );
	// Set Residual Offset          that is FACTORY ADJUSTED
	I2C_OIS_per_write( _P_39_Ch3_VAL_1, param->gl_PSTXOF );
	I2C_OIS_per_write( _P_3B_Ch3_VAL_3, param->gl_PSTYOF );

	//*********************
	// DIGITAL GYRO OFFSET
	//*********************
	I2C_OIS_mem_write( _M_Kgx00, param->gl_GX_OFS );
	I2C_OIS_mem_write( _M_Kgy00, param->gl_GY_OFS );
	I2C_OIS_mem_write( _M_TMP_X_, param->gl_TMP_X_ );
	I2C_OIS_mem_write( _M_TMP_Y_, param->gl_TMP_Y_ );
	//*********************
	// HALL SENSE
	//*********************
	// Set Hall Gain   value that is FACTORY ADJUSTED
	I2C_OIS_mem_write( _M_KgxHG, param->gl_KgxHG );
	I2C_OIS_mem_write( _M_KgyHG, param->gl_KgyHG );
	// Set Cross Talk Canceller
	I2C_OIS_mem_write( _M_KgxH0, param->gl_KgxH0 );
	I2C_OIS_mem_write( _M_KgyH0, param->gl_KgyH0 );
	//*********************
	// LOOPGAIN
	//*********************
	I2C_OIS_mem_write( _M_KgxG, param->gl_KGXG );
	I2C_OIS_mem_write( _M_KgyG, param->gl_KGYG );

	// Get default Integ_input and KgnTG value
	INTG__INPUT = I2C_OIS_mem__read( 0x38 );
	KGNTG_VALUE = I2C_OIS_mem__read( _M_KgxTG );
	// Position Servo ON ( OIS OFF )
	I2C_OIS_mem_write( _M_EQCTL, 0x0C0C );

}

// <== RHM_HT 2016/01/18	Added for CLAF issue


//  *****************************************************
//  **** Scence parameter
//  *****************************************************
//#define	ANGLE_LIMIT	(0x3020)       // (0x2BC0 * 1.1)		// GYRSNS * limit[deg]
#define	ANGLE_LIMIT	(0x2406)       // (0x2BC0 * 1.1)		// GYRSNS * limit[deg]
// ==> RHM_HT 2015/01/08	Changed to global variables
// #define	INTG__INPUT	(0x1404)			// Integral Input value	szx_2014/12/24_2
// #define	KGNTG_VALUE	(0x3879)			// KgxTG / KgyTG		szx_2014/12/24_2
// <== RHM_HT 2015/01/08	Changed to global variables
#define	G_SENSE		131				// [LSB/dps] for ICG20660
ADJ_STS	func_SET_SCENE_PARAM(OIS_UBYTE u16_scene, OIS_UBYTE u16_mode, OIS_UBYTE filter, OIS_UBYTE range, const _FACT_ADJ *param)	// RHM_HT 2013/04/15	Change "typedef" of return value
{
	OIS_UWORD u16_i;
	OIS_UWORD u16_dat;

// ==> RHM_HT 2013/11/25	Modified
	OIS_UBYTE u16_adr_target[3]        = { _M_Kgxdr, _M_X_LMT, _M_X_TGT,  };

	OIS_UWORD u16_dat_SCENE_NIGHT_1[3] = { 0x7FFE,   ANGLE_LIMIT,   G_SENSE * 16,    };	// 16dps
	OIS_UWORD u16_dat_SCENE_NIGHT_2[3] = { 0x7FFC,   ANGLE_LIMIT,   G_SENSE * 16,    };	// 16dps
	OIS_UWORD u16_dat_SCENE_NIGHT_3[3] = { 0x7FFA,   ANGLE_LIMIT,   G_SENSE * 16,    };	// 16dps

	OIS_UWORD u16_dat_SCENE_D_A_Y_1[3] = { 0x7FFE,   ANGLE_LIMIT,   G_SENSE * 40,    };	// 40dps
	OIS_UWORD u16_dat_SCENE_D_A_Y_2[3] = { 0x7FFA,   ANGLE_LIMIT,   G_SENSE * 40,    };	// 40dps
	OIS_UWORD u16_dat_SCENE_D_A_Y_3[3] = { 0x7FF0,   ANGLE_LIMIT,   G_SENSE * 40,    };	// 40dps

	OIS_UWORD u16_dat_SCENE_SPORT_1[3] = { 0x7FFE,   ANGLE_LIMIT,   G_SENSE * 60,    };	// 60dps
	OIS_UWORD u16_dat_SCENE_SPORT_2[3] = { 0x7FF0,   ANGLE_LIMIT,   G_SENSE * 60,    };	// 60dps
	OIS_UWORD u16_dat_SCENE_SPORT_3[3] = { 0x7FE0,   ANGLE_LIMIT,   G_SENSE * 60,    };	// 60dps

	OIS_UWORD u16_dat_SCENE_TEST___[3] = { 0x7FF0,   0x7FFF,   0x7FFF,    };	// Limmiter OFF
// <== RHM_HT 2013/11/25	Modified

	OIS_UWORD *u16_dat_SCENE_;

	OIS_UBYTE	size_SCENE_tbl = sizeof( u16_dat_SCENE_NIGHT_1 ) / sizeof(OIS_UWORD);

	// Disable OIS ( position Servo is not disable )
	u16_dat = I2C_OIS_mem__read( _M_EQCTL );
	u16_dat = ( u16_dat &  0xFEFE );
	I2C_OIS_mem_write( _M_EQCTL, u16_dat );

	// Scene parameter select
	switch( u16_scene ){
		case _SCENE_NIGHT_1 : u16_dat_SCENE_ = u16_dat_SCENE_NIGHT_1;	DEBUG_printf(("+▲▲▲▲▲▲▲▲▲▲+\n+---_SCENE_NIGHT_1---+\n+▲▲▲▲▲▲▲▲▲▲+\n"));		break;
		case _SCENE_NIGHT_2 : u16_dat_SCENE_ = u16_dat_SCENE_NIGHT_2;	DEBUG_printf(("+▲▲▲▲▲▲▲▲▲▲+\n+---_SCENE_NIGHT_2---+\n+▲▲▲▲▲▲▲▲▲▲+\n"));		break;
		case _SCENE_NIGHT_3 : u16_dat_SCENE_ = u16_dat_SCENE_NIGHT_3;	DEBUG_printf(("+▲▲▲▲▲▲▲▲▲▲+\n+---_SCENE_NIGHT_3---+\n+▲▲▲▲▲▲▲▲▲▲+\n"));		break;
		case _SCENE_D_A_Y_1 : u16_dat_SCENE_ = u16_dat_SCENE_D_A_Y_1;	DEBUG_printf(("+??????????+\n+---_SCENE_D_A_Y_1---+\n+??????????+\n"));		break;
		case _SCENE_D_A_Y_2 : u16_dat_SCENE_ = u16_dat_SCENE_D_A_Y_2;	DEBUG_printf(("+??????????+\n+---_SCENE_D_A_Y_2---+\n+??????????+\n"));		break;
		case _SCENE_D_A_Y_3 : u16_dat_SCENE_ = u16_dat_SCENE_D_A_Y_3;	DEBUG_printf(("+??????????+\n+---_SCENE_D_A_Y_3---+\n+??????????+\n"));		break;
		case _SCENE_SPORT_1 : u16_dat_SCENE_ = u16_dat_SCENE_SPORT_1;	DEBUG_printf(("+??????????+\n+---_SCENE_SPORT_1---+\n+??????????+\n"));		break;
		case _SCENE_SPORT_2 : u16_dat_SCENE_ = u16_dat_SCENE_SPORT_2;	DEBUG_printf(("+??????????+\n+---_SCENE_SPORT_2---+\n+??????????+\n"));		break;
		case _SCENE_SPORT_3 : u16_dat_SCENE_ = u16_dat_SCENE_SPORT_3;	DEBUG_printf(("+??????????+\n+---_SCENE_SPORT_3---+\n+??????????+\n"));		break;
		case _SCENE_TEST___ : u16_dat_SCENE_ = u16_dat_SCENE_TEST___;	DEBUG_printf(("+********************+\n+---dat_SCENE_TEST___+\n+********************+\n"));		break;
		default             : u16_dat_SCENE_ = u16_dat_SCENE_TEST___;	DEBUG_printf(("+********************+\n+---dat_SCENE_TEST___+\n+********************+\n"));		break;
	}

	// Set parameter to the OIS controller
	for( u16_i = 0; u16_i < size_SCENE_tbl; u16_i += 1 ){
		I2C_OIS_mem_write( u16_adr_target[u16_i],          	u16_dat_SCENE_[u16_i]   );
	}
	for( u16_i = 0; u16_i < size_SCENE_tbl; u16_i += 1 ){
		I2C_OIS_mem_write( u16_adr_target[u16_i] + 0x80,	u16_dat_SCENE_[u16_i] );
	}

	// Set/Reset Notch filter
	if ( filter == 1 ) {					// Disable Filter
		u16_dat = I2C_OIS_mem__read( _M_EQCTL );
		u16_dat |= 0x4000;
		I2C_OIS_mem_write( _M_EQCTL, u16_dat );
	}
	else{									// Enable Filter
		u16_dat = I2C_OIS_mem__read( _M_EQCTL );
		u16_dat &= 0xBFFF;
		I2C_OIS_mem_write( _M_EQCTL, u16_dat );
	}

	// Clear the register of the OIS controller
	I2C_OIS_mem_write( _M_wDgx02, 0x0000 );
	I2C_OIS_mem_write( _M_wDgx03, 0x0000 );
	I2C_OIS_mem_write( _M_wDgx06, 0x7FFF );
	I2C_OIS_mem_write( _M_Kgx15,  0x0000 );

	I2C_OIS_mem_write( _M_wDgy02, 0x0000 );
	I2C_OIS_mem_write( _M_wDgy03, 0x0000 );
	I2C_OIS_mem_write( _M_wDgy06, 0x7FFF );
	I2C_OIS_mem_write( _M_Kgy15,  0x0000 );

	// Set the pre-Amp offset value (X and Y)
// ==> RHM_HT 2013/11/25	Modified
	if	( range == 1 ) {
		I2C_OIS_per_write( _P_31_ADC_CH1, param->gl_SFTHAL_X );
		I2C_OIS_per_write( _P_32_ADC_CH2, param->gl_SFTHAL_Y );
	}
	else{
		I2C_OIS_per_write( _P_31_ADC_CH1, param->gl_HALOFS_X );
		I2C_OIS_per_write( _P_32_ADC_CH2, param->gl_HALOFS_Y );
	}
// <== RHM_HT 2013/11/25	Modified

	// Enable OIS (if u16_mode = 1)
	if(	( u16_mode == 1 ) ){
		u16_dat = I2C_OIS_mem__read( _M_EQCTL );
		u16_dat = ( u16_dat |  0x0101 );
		I2C_OIS_mem_write( _M_EQCTL, u16_dat );
		DEBUG_printf(("SET : EQCTL:%.4x\n", u16_dat ));
	}
	else{														// ==> RHM_HT 2013.03.23	Add for OIS controll
		u16_dat = I2C_OIS_mem__read( _M_EQCTL );
		u16_dat = ( u16_dat &  0xFEFE );
		I2C_OIS_mem_write( _M_EQCTL, u16_dat );
		DEBUG_printf(("SET : EQCTL:%.4x\n", u16_dat ));
	}															// <== RHM_HT 2013.03.23	Add for OIS controll

	return ADJ_OK;												// RHM_HT 2013/04/15	Change return value
}


ADJ_STS	func_SET_SCENE_PARAM_for_NewGYRO_Fil(OIS_UBYTE u16_scene, OIS_UBYTE u16_mode, OIS_UBYTE filter, OIS_UBYTE range, const _FACT_ADJ *param)	// RHM_HT 2013/04/15	Change "typedef" of return value
{
	OIS_UWORD u16_i;
	OIS_UWORD u16_dat;

// szx_2014/09/19 ---> Modified
// ==> RHM_HT 2013/11/25	Modified
	OIS_UBYTE u16_adr_target[4]        = { _M_Kgxdr, _M_X_LMT, _M_X_TGT, 0x1B,    };

	OIS_UWORD u16_dat_SCENE_NIGHT_1[4] = { 0x7FE0,   ANGLE_LIMIT,   G_SENSE * 16,   0x0300,  };	// 16dps
	OIS_UWORD u16_dat_SCENE_NIGHT_2[4] = { 0x7FFF,   ANGLE_LIMIT,   G_SENSE * 16,   0x0080,  };	// 16dps	RHM_HT 2014/11/27	Changed Kgxdr at Xiaomi
	OIS_UWORD u16_dat_SCENE_NIGHT_3[4] = { 0x7FF0,   ANGLE_LIMIT,   G_SENSE * 16,   0x0300,  };	// 16dps	RHM_HT 2014/11/27	Changed Kgxdr at Xiaomi

	OIS_UWORD u16_dat_SCENE_D_A_Y_1[4] = { 0x7FE0,   ANGLE_LIMIT,   G_SENSE * 40,   0x0300,  };	// 40dps	RHM_HT 2014/11/27	Changed Kgxdr at Xiaomi
	OIS_UWORD u16_dat_SCENE_D_A_Y_2[4] = { 0x7F80,   ANGLE_LIMIT,   G_SENSE * 40,   0x0140,  };	// 40dps	RHM_HT 2014/11/27	Changed Kgxdr at Xiaomi
	OIS_UWORD u16_dat_SCENE_D_A_Y_3[4] = { 0x7F00,   ANGLE_LIMIT,   G_SENSE * 40,   0x0300,  };	// 40dps	RHM_HT 2014/11/27	Changed Kgxdr at Xiaomi

	OIS_UWORD u16_dat_SCENE_SPORT_1[4] = { 0x7FE0,   ANGLE_LIMIT,   G_SENSE * 60,   0x0300,  };	// 60dps	RHM_HT 2014/11/27	Changed Kgxdr at Xiaomi
// szx_2014/12/24 ===>
	// OIS_UWORD u16_dat_SCENE_SPORT_2[4] = { 0x7FFF,   ANGLE_LIMIT,   G_SENSE * 60,   0x0080,  };	// 60dps	RHM_HT 2014/11/27	Changed Kgxdr at Xiaomi
	// OIS_UWORD u16_dat_SCENE_SPORT_3[4] = { 0x7FF0,   ANGLE_LIMIT,   G_SENSE * 60,   0x0300,  };	// 60dps	RHM_HT 2014/11/27	Changed Kgxdr at Xiaomi
	OIS_UWORD u16_dat_SCENE_SPORT_2[4] = { 0x7F80,   ANGLE_LIMIT,   G_SENSE * 60,   0x0000,  };	// 60dps	RHM_HT 2014/11/27	Changed Kgxdr at Xiaomi
// szx_2015/01/20 ===>
 	OIS_UWORD u16_dat_SCENE_SPORT_3[4] = { 0x7FFF,   ANGLE_LIMIT,   G_SENSE * 60,   0x0100,  };	// 60dps	RHM_HT 2015/08/13	Changed for Xiaomi A1
//	OIS_UWORD u16_dat_SCENE_SPORT_3[4] = { 0x7FFF,   ANGLE_LIMIT,   G_SENSE * 5,   0x00E0,  };	//  5dps
// szx_2015/01/20 <===
// szx_2014/12/24 <===

	OIS_UWORD u16_dat_SCENE_TEST___[4] = { 0x7FFF,   0x7FFF,   0x7FFF,   0x0080,  };	// Limmiter OFF
// <== RHM_HT 2013/11/25	Modified
// szx_2014/09/19 <---

	OIS_UWORD *u16_dat_SCENE_;

	OIS_UBYTE	size_SCENE_tbl = sizeof( u16_dat_SCENE_NIGHT_1 ) / sizeof(OIS_UWORD);

	// Disable OIS ( position Servo is not disable )
	u16_dat = I2C_OIS_mem__read( _M_EQCTL );
	u16_dat = ( u16_dat &  0xFEFE );
	I2C_OIS_mem_write( _M_EQCTL, u16_dat );

	// Scene parameter select
	switch( u16_scene ){
		case _SCENE_NIGHT_1 : u16_dat_SCENE_ = u16_dat_SCENE_NIGHT_1;	DEBUG_printf(("+▲▲▲▲▲▲▲▲▲▲+\n+---_SCENE_NIGHT_1---+\n+▲▲▲▲▲▲▲▲▲▲+\n"));		break;
		case _SCENE_NIGHT_2 : u16_dat_SCENE_ = u16_dat_SCENE_NIGHT_2;	DEBUG_printf(("+▲▲▲▲▲▲▲▲▲▲+\n+---_SCENE_NIGHT_2---+\n+▲▲▲▲▲▲▲▲▲▲+\n"));		break;
		case _SCENE_NIGHT_3 : u16_dat_SCENE_ = u16_dat_SCENE_NIGHT_3;	DEBUG_printf(("+▲▲▲▲▲▲▲▲▲▲+\n+---_SCENE_NIGHT_3---+\n+▲▲▲▲▲▲▲▲▲▲+\n"));		break;
		case _SCENE_D_A_Y_1 : u16_dat_SCENE_ = u16_dat_SCENE_D_A_Y_1;	DEBUG_printf(("+??????????+\n+---_SCENE_D_A_Y_1---+\n+??????????+\n"));		break;
		case _SCENE_D_A_Y_2 : u16_dat_SCENE_ = u16_dat_SCENE_D_A_Y_2;	DEBUG_printf(("+??????????+\n+---_SCENE_D_A_Y_2---+\n+??????????+\n"));		break;
		case _SCENE_D_A_Y_3 : u16_dat_SCENE_ = u16_dat_SCENE_D_A_Y_3;	DEBUG_printf(("+??????????+\n+---_SCENE_D_A_Y_3---+\n+??????????+\n"));		break;
		case _SCENE_SPORT_1 : u16_dat_SCENE_ = u16_dat_SCENE_SPORT_1;	DEBUG_printf(("+??????????+\n+---_SCENE_SPORT_1---+\n+??????????+\n"));		break;
		case _SCENE_SPORT_2 : u16_dat_SCENE_ = u16_dat_SCENE_SPORT_2;	DEBUG_printf(("+??????????+\n+---_SCENE_SPORT_2---+\n+??????????+\n"));		break;
		case _SCENE_SPORT_3 : u16_dat_SCENE_ = u16_dat_SCENE_SPORT_3;	DEBUG_printf(("+??????????+\n+---_SCENE_SPORT_3---+\n+??????????+\n"));		break;
		case _SCENE_TEST___ : u16_dat_SCENE_ = u16_dat_SCENE_TEST___;	DEBUG_printf(("+********************+\n+---dat_SCENE_TEST___+\n+********************+\n"));		break;
		default             : u16_dat_SCENE_ = u16_dat_SCENE_TEST___;	DEBUG_printf(("+********************+\n+---dat_SCENE_TEST___+\n+********************+\n"));		break;
	}

	// Set parameter to the OIS controller
	for( u16_i = 0; u16_i < size_SCENE_tbl; u16_i += 1 ){
		I2C_OIS_mem_write( u16_adr_target[u16_i],          	u16_dat_SCENE_[u16_i]   );
	}
	for( u16_i = 0; u16_i < size_SCENE_tbl; u16_i += 1 ){
		I2C_OIS_mem_write( u16_adr_target[u16_i] + 0x80,	u16_dat_SCENE_[u16_i] );
	}

	// szx_2014/12/24 ===>
	// ①　リミッタ(X1)を読み出す
	// ②　積分入力(X2)を読み出す
	// ③　X2 * 7FFFh/(X1)/2(後段アンプゲイン) = (X2) * 4000h/(X1)を積分入力として設定
	// ④　Kg*TGを読み出し(X3)、X3=X3/(7FFFh/X1/2)  ⇒ X3=X3/(4000h/X1))  ⇒ X3=X3*X1/4000h
	{
		OIS_ULONG	temp;
// 		temp = I2C_OIS_mem__read( 0x38 );				// X2
// 		temp = temp * 16384;							// X2 * 4000h
// 		u16_dat = I2C_OIS_mem__read( _M_X_LMT );		// X1
// 		u16_dat = temp / u16_dat;						// X2 * 4000h / X1
// 
// 		I2C_OIS_mem_write( 0x38, u16_dat );
// 		I2C_OIS_mem_write( 0xB8, u16_dat );
// 
// 		//----------------------------------------------
// 
// 		temp = I2C_OIS_mem__read( 0x47 );				// X3
// 		temp = temp * I2C_OIS_mem__read( _M_X_LMT );	// X3 * X1
// 		temp = temp / 16384;							// X3 * X1 / 4000h
// 
// 		I2C_OIS_mem_write( 0x47, temp );
// 		I2C_OIS_mem_write( 0xC7, temp );

		temp = ( INTG__INPUT * 16384 );	// X2 * 4000h / X1
		u16_dat = temp / ANGLE_LIMIT;

		I2C_OIS_mem_write( 0x38, u16_dat );
		I2C_OIS_mem_write( 0xB8, u16_dat );

		//----------------------------------------------

		temp = ( KGNTG_VALUE * ANGLE_LIMIT );	// X3 * X1 / 4000h
		u16_dat = temp / 16384;

		I2C_OIS_mem_write( 0x47, u16_dat );
		I2C_OIS_mem_write( 0xC7, u16_dat );

		//----------------------------------------------

		I2C_OIS_mem_write( 0x40, 0x7FF0 );		// 
		I2C_OIS_mem_write( 0xC0, 0x7FF0 );		// 
	}
	// szx_2014/12/24 <===
	// Set/Reset Notch filter
	if ( filter == 1 ) {					// Disable Filter
		u16_dat = I2C_OIS_mem__read( _M_EQCTL );
		u16_dat |= 0x4000;
		I2C_OIS_mem_write( _M_EQCTL, u16_dat );
	}
	else{									// Enable Filter
		u16_dat = I2C_OIS_mem__read( _M_EQCTL );
		u16_dat &= 0xBFFF;
		I2C_OIS_mem_write( _M_EQCTL, u16_dat );
	}

// szx_2014/09/19 --->
// 	// Clear the register of the OIS controller
// 	I2C_OIS_mem_write( _M_wDgx02, 0x0000 );
// 	I2C_OIS_mem_write( _M_wDgx03, 0x0000 );
// 	I2C_OIS_mem_write( _M_wDgx06, 0x7FFF );
// 	I2C_OIS_mem_write( _M_Kgx15,  0x0000 );
// 	
// 	I2C_OIS_mem_write( _M_wDgy02, 0x0000 );
// 	I2C_OIS_mem_write( _M_wDgy03, 0x0000 );
// 	I2C_OIS_mem_write( _M_wDgy06, 0x7FFF );
// 	I2C_OIS_mem_write( _M_Kgy15,  0x0000 );
// szx_2014/09/19 <---
	
	// Set the pre-Amp offset value (X and Y)
// ==> RHM_HT 2013/11/25	Modified
	if	( range == 1 ) {
		I2C_OIS_per_write( _P_31_ADC_CH1, param->gl_SFTHAL_X );
		I2C_OIS_per_write( _P_32_ADC_CH2, param->gl_SFTHAL_Y );
	}
	else{
		I2C_OIS_per_write( _P_31_ADC_CH1, param->gl_HALOFS_X );
		I2C_OIS_per_write( _P_32_ADC_CH2, param->gl_HALOFS_Y );
	}
// <== RHM_HT 2013/11/25	Modified

	// Enable OIS (if u16_mode = 1)
	if(	( u16_mode == 1 ) ){	// OIS ON
		u16_dat = I2C_OIS_mem__read( _M_EQCTL );
		u16_dat = ( u16_dat &  0xEFFF );						// Clear Halfshutter mode
		u16_dat = ( u16_dat |  0x0101 );
		I2C_OIS_mem_write( _M_EQCTL, u16_dat );
		DEBUG_printf(("SET : EQCTL:%.4x\n", u16_dat ));
	}
	else if	( u16_mode == 2 ){	// Half Shutter		// szx_2014/09/19 --->
		u16_dat = I2C_OIS_mem__read( _M_EQCTL );
		u16_dat = ( u16_dat |  0x1101 );
		I2C_OIS_mem_write( _M_EQCTL, u16_dat );
		DEBUG_printf(("SET : EQCTL:%.4x\n", u16_dat ));
	}	// <--- szx_2014/09/19
	else{														// ==> RHM_HT 2013.03.23	Add for OIS controll
		u16_dat = I2C_OIS_mem__read( _M_EQCTL );
		u16_dat = ( u16_dat &  0xFEFE );
		I2C_OIS_mem_write( _M_EQCTL, u16_dat );
		DEBUG_printf(("SET : EQCTL:%.4x\n", u16_dat ));
	}															// <== RHM_HT 2013.03.23	Add for OIS controll

//	I2C_OIS_mem_write( _M_EQCTL, 0x0000);
//	u16_dat = I2C_OIS_mem__read( _M_EQCTL );
//	pr_err("[BU63165] ===Andy=== SET : EQCTL:%.4x\n", u16_dat);
#if 0
	u16_dat = I2C_OIS_mem__read(0x7f);
	pr_err("77777[BU63165] ===Andy=== SET : 847f:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_mem__read(0xf6);
	pr_err("77777[BU63165] ===Andy=== SET : 84f6:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_mem__read(0x76);
	pr_err("777777[BU63165] ===Andy=== SET : 8476:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_per__read(0x39);
	pr_err("777777[BU63165] ===Andy=== SET : 8239:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_per__read(0x3b);
	pr_err("777777[BU63165] ===Andy=== SET : 823b:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_mem__read(0x1e);
	pr_err("777777[BU63165] ===Andy=== SET : 841e:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_mem__read(0x9e);
	pr_err("777777[BU63165] ===Andy=== SET : 849e:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_mem__read(0x46);
	pr_err("777777[BU63165] ===Andy=== SET : 8446:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_mem__read(0xc6);
	pr_err("777777[BU63165] ===Andy=== SET : 84c6:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_mem__read(0x70);
	pr_err("777777[BU63165] ===Andy=== SET : 8470:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_mem__read(0x72);
	pr_err("777777[BU63165] ===Andy=== SET : 8472:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_mem__read(0x0f);
	pr_err("777777[BU63165] ===Andy=== SET : 840f:%.4x\n", u16_dat);
	u16_dat = I2C_OIS_mem__read(0x8f);
	pr_err("777777[BU63165] ===Andy=== SET : 848f:%.4x\n", u16_dat);
#endif
	return ADJ_OK;												// RHM_HT 2013/04/15	Change return value
}
// ==> RHM_HT 2014/11/27	Added
//  *****************************************************
//  **** Enable HalfShutter
//  *****************************************************
void	HalfShutterOn( void )
{
	OIS_UWORD u16_dat = 0;

	u16_dat = I2C_OIS_mem__read( _M_EQCTL );
	u16_dat = ( u16_dat |  0x1101 );
	I2C_OIS_mem_write( _M_EQCTL, u16_dat );
	DEBUG_printf(("SET : EQCTL:%.4x\n", u16_dat ));
}
// <== RHM_HT 2014/11/27	Added
//  *****************************************************
//  **** Write to the Peripheral register < 82h >
//  **** ------------------------------------------------
//  **** OIS_UBYTE	adr	Peripheral Address
//  **** OIS_UWORD	dat	Write data
//  *****************************************************
void	I2C_OIS_per_write( OIS_UBYTE u08_adr, OIS_UWORD u16_dat ){

	OIS_UBYTE	out[4];
	
	out[0] = _OP_Periphe_RW;
	out[1] = u08_adr;
	out[2] = ( u16_dat      ) & 0xFF;
	out[3] = ( u16_dat >> 8 ) & 0xFF;
		
	WR_I2C( _SLV_OIS_, 4, out );
}

//  *****************************************************
//  **** Write to the Memory register < 84h >
//  **** ------------------------------------------------
//  **** OIS_UBYTE	adr	Memory Address
//  **** OIS_UWORD	dat	Write data
//  *****************************************************
void	I2C_OIS_mem_write( OIS_UBYTE u08_adr, OIS_UWORD u16_dat){

	OIS_UBYTE	out[4];
	
	out[0] = _OP_Memory__RW;
	out[1] = u08_adr;
	out[2] = ( u16_dat      ) & 0xFF;
	out[3] = ( u16_dat >> 8 ) & 0xFF;
		
	WR_I2C( _SLV_OIS_, 4, out );
}

//  *****************************************************
//  **** Read from the Peripheral register < 82h >
//  **** ------------------------------------------------
//  **** OIS_UBYTE	adr	Peripheral Address
//  **** OIS_UWORD	dat	Read data
//  *****************************************************
OIS_UWORD	I2C_OIS_per__read( OIS_UBYTE u08_adr ){

	OIS_UBYTE	u08_dat[2];
	
	u08_dat[0] = _OP_Periphe_RW;	// Op-code
	u08_dat[1] = u08_adr;			// target address
		
	return RD_I2C( _SLV_OIS_, 2, u08_dat );
}


//  *****************************************************
//  **** Read from the Memory register < 84h >
//  **** ------------------------------------------------
//  **** OIS_UBYTE	adr	Memory Address
//  **** OIS_UWORD	dat	Read data
//  *****************************************************
OIS_UWORD	I2C_OIS_mem__read( OIS_UBYTE u08_adr){

	OIS_UBYTE	u08_dat[2];
	
	u08_dat[0] = _OP_Memory__RW;	// Op-code
	u08_dat[1] = u08_adr;			// target address
		
	return RD_I2C( _SLV_OIS_, 2, u08_dat );
}


//  *****************************************************
//  **** Special Command 8Ah
// 		_cmd_8C_EI			0	// 0x0001
// 		_cmd_8C_DI			1	// 0x0002
//  *****************************************************
void	I2C_OIS_spcl_cmnd( OIS_UBYTE u08_on, OIS_UBYTE u08_dat ){

	if( ( u08_dat == _cmd_8C_EI ) ||
		( u08_dat == _cmd_8C_DI )    ){

		OIS_UBYTE out[2];
		
		out[0] = _OP_SpecialCMD;
		out[1] = u08_dat;
		
		pr_err("SPCL WR_I2C 0x%x 0x%x", out[0], out[1]);

                g_i2c_ctrl->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
                g_i2c_ctrl->i2c_func_tbl->i2c_write(
                        g_i2c_ctrl, out[0], out[1], MSM_CAMERA_I2C_BYTE_DATA);

	}
}


//  *****************************************************
//  **** F0-F3h Command NonAssertClockStretch Function
//  *****************************************************
void	I2C_OIS_F0123_wr_( OIS_UBYTE u08_dat0, OIS_UBYTE u08_dat1, OIS_UWORD u16_dat2 ){

	OIS_UBYTE out[5];
		
	out[0] = 0xF0;
	out[1] = u08_dat0;
	out[2] = u08_dat1;
	out[3] = u16_dat2 / 256;
	out[4] = u16_dat2 % 256;
		
	pr_err("SPCL WR_I2C 0x%x 0x%x dat2:%d", out[0], out[1], u16_dat2);

	g_i2c_ctrl->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	g_i2c_ctrl->i2c_func_tbl->i2c_write_seq(
		g_i2c_ctrl, out[0], &(out[1]), 4);


}

OIS_UWORD	I2C_OIS_F0123__rd( void ){

	OIS_UBYTE	u08_dat;
	
	//u08_dat = 0xF0;				// Op-code value you write to 0x90
	//u08_dat = 0xF2;				// Op-code high 8bit
	u08_dat = 0xF3;				// Op-code low 8bit
		
	return RD_I2C( _SLV_OIS_, 1, &u08_dat );
}


// -----------------------------------------------------------
// -----------------------------------------------------------
// -----------------------------------------------------------

