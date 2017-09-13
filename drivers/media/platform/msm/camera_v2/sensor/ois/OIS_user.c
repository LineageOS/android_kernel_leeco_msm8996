/////////////////////////////////////////////////////////////////////////////
// File Name	: OIS_user.c
// Function		: User defined function.
// 				  These functions depend on user's circumstance.
// 				  
// Rule         : Use TAB 4
// 
// Copyright(c)	Rohm Co.,Ltd. All rights reserved 
// 
/***** ROHM Confidential ***************************************************/
#ifndef OIS_USER_C
#define OIS_USER_C
#endif

#include "OIS_head.h"
//#include "usb_func.h"
//#include "winbase.h"


static struct msm_camera_i2c_client *g_i2c_ctrl;
// Following Variables that depend on user's environment			RHM_HT 2013.03.13	add
OIS_UWORD			FOCUS_VAL	= 0x0122;				// Focus Value

/* 2015-12-12 add-s */
int  Ois_rohm_user_SetI2cClient(struct msm_camera_i2c_client *client)
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


// /////////////////////////////////////////////////////////
// VCOSET function
// ---------------------------------------------------------
// <Function>
//		To use external clock at CLK/PS, it need to set PLL.
//		After enabling PLL, more than 30ms wait time is required to change clock source.
//		So the below sequence has to be used:
// 		Input CLK/PS --> Call VCOSET0 --> Download Program/Coed --> Call VCOSET1
//
// <Input>
//		none
//
// <Output>
//		none
//
// =========================================================
void	VCOSET0( void )
{

    OIS_UWORD 	CLK_PS = 24000;            					// Input Frequency [kHz] of CLK/PS terminal (Depend on your system)
   // OIS_UWORD 	FVCO_1 = 27000;                				// Target Frequency [kHz]
    OIS_UWORD 	FVCO_1 = 36000;                				// Target Frequency [kHz]
    OIS_UWORD 	FREF   = 25;             						// Reference Clock Frequency [kHz]
 
    OIS_UWORD	DIV_N  = CLK_PS / FREF - 1;         			// calc DIV_N
    OIS_UWORD	DIV_M  = FVCO_1 / FREF - 1;         			// calc DIV_M
 
    I2C_OIS_per_write( 0x62, DIV_N  ); 							// Divider for internal reference clock
    I2C_OIS_per_write( 0x63, DIV_M  ); 							// Divider for internal PLL clock
    I2C_OIS_per_write( 0x64, 0x4060 ); 							// Loop Filter

    I2C_OIS_per_write( 0x60, 0x3011 ); 							// PLL
    I2C_OIS_per_write( 0x65, 0x0080 ); 							// 
    I2C_OIS_per_write( 0x61, 0x8002 ); 							// VCOON 
    I2C_OIS_per_write( 0x61, 0x8003 ); 							// Circuit ON 
    I2C_OIS_per_write( 0x61, 0x8809 ); 							// PLL ON
}


void	VCOSET1( void )
{
// 
//     OIS_UWORD 	CLK_PS = 23880;            						// Input Frequency [kHz] of CLK/PS terminal (Depend on your system)	RHM_HT 2013.05.09	Change 12M -> 6.75M
//     OIS_UWORD 	FVCO_1 = 27000;                					// Target Frequency [kHz]
//     OIS_UWORD 	FREF   = 25;             						// Reference Clock Frequency [kHz]
//  
//     OIS_UWORD	DIV_N  = CLK_PS / FREF - 1;         			// calc DIV_N
//     OIS_UWORD	DIV_M  = FVCO_1 / FREF - 1;         			// calc DIV_M
//  
//     I2C_OIS_per_write( 0x62, DIV_N  ); 							// Divider for internal reference clock
//     I2C_OIS_per_write( 0x63, DIV_M  ); 							// Divider for internal PLL clock
//     I2C_OIS_per_write( 0x64, 0x4060 ); 							// Loop Filter
// 
//     I2C_OIS_per_write( 0x60, 0x3011 ); 							// PLL
//     I2C_OIS_per_write( 0x65, 0x0080 ); 							// 
//     I2C_OIS_per_write( 0x61, 0x8002 ); 							// VCOON 
//     I2C_OIS_per_write( 0x61, 0x8003 ); 							// Circuit ON 
//     I2C_OIS_per_write( 0x61, 0x8809 ); 							// PLL ON
// 
//     Wait( 30 );                  								// Wait for PLL lock

    I2C_OIS_per_write( 0x05, 0x000C ); 							// Prepare for PLL clock as master clock
    I2C_OIS_per_write( 0x05, 0x000D ); 							// Change to PLL clock
}

void	WR_I2C( OIS_UBYTE slvadr, OIS_UBYTE size, OIS_UBYTE *dat )
{
	OIS_UWORD       addr = dat[0] << 8 | dat[1];
	OIS_UBYTE	*data_wr   = dat + 2;

	g_i2c_ctrl->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
        g_i2c_ctrl->i2c_func_tbl->i2c_write_seq(
                g_i2c_ctrl, addr, data_wr, size - 2);

//	pr_err("WR_I2C addr:0x%04x data[0]:0x%x data[1]:0x%x \n", addr, data_wr[0], data_wr[1]);
}

OIS_UWORD	RD_I2C( OIS_UBYTE slvadr, OIS_UBYTE size, OIS_UBYTE *dat )
{
	OIS_UWORD	read_data = 0;
	OIS_UBYTE	data_rd[2];
	OIS_UWORD       addr = dat[0] << 8 | dat[1];


	g_i2c_ctrl->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
#if 0
	g_i2c_ctrl->i2c_func_tbl->i2c_read(
		g_i2c_ctrl, addr, &read_data, MSM_CAMERA_I2C_WORD_DATA);
#endif
	g_i2c_ctrl->i2c_func_tbl->i2c_read_seq(
		g_i2c_ctrl, addr, data_rd, size);
	read_data = data_rd[0] << 8 | data_rd[1];
//	pr_err("RD_I2C addr:0x%04x data:0x%04x \n", addr, read_data);
	
	return read_data;
}

// *********************************************************
// Write Factory Adjusted data to the non-volatile memory
// ---------------------------------------------------------
// <Function>
//		Factory adjusted data are sotred somewhere
//		non-volatile memory.
//
// <Input>
//		_FACT_ADJ	Factory Adjusted data
//
// <Output>
//		none
//
// <Description>
//		You have to port your own system.
//
// *********************************************************
void	store_FADJ_MEM_to_non_volatile_memory( _FACT_ADJ param )
{
	/* 	Write to the non-vollatile memory such as EEPROM or internal of the CMOS sensor... */	
}

// FACTORY Adjusted data
// These data are stored at the non-vollatile
// memory inside of the CMOS sensor.
// The Host ( ISP or I2C master ) read these
// data from above memory and write to the OIS
// controller.
// ---------------------------------------------
_FACT_ADJ	FADJ_MEM
= {
#if 0
                        0x01bf,
                        0x01e3,
                        0x020a,
                        0x0120,
                        0x02ec,
                        0x0080,
                        0x007f,
                        0xffa0,
                        0xffc0,
                        0xdb3f,
                        0xda4a, 
                        0x2d36,
                        0x268c,
                        0x01e3,
                        0x020a,
                        0x0000,
                        0x0000,
                        0xffce,
                        0xfffc,
#endif
0x01bf,
0x01e3,
0x020a,
0x0120,
0x02ec,
0x0080,
0x007f,
0xfd40,
0x0460,
0xdb3f,
0xda4a,
0x2d36,
0x268c,

0x01e3,
0x020a,
0x0000,
0x0000,
0xffce,
0xfffc,


/*
	0x0201,	// gl_CURDAT;
	0x0200,	// gl_HALOFS_X;
	0x0200,	// gl_HALOFS_Y;
	0x0000,	// gl_HX_OFS;
	0x0000,	// gl_HY_OFS;
	0x0080,	// gl_PSTXOF;		RHM_HT 2013.03.21	Change order to adjust EEP ROM map
	0x0080,	// gl_PSTYOF;		RHM_HT 2013.03.21	Change order to adjust EEP ROM map
	0x0000,	// gl_GX_OFS;
	0x0000,	// gl_GY_OFS;

	0x2000,	// gl_KgxHG ;		RHM_HT 2013/11/25	Modified
	0x2000,	// gl_KgyHG ;		RHM_HT 2013/11/25	Modified
	0x2000,	// gl_KGXG  ;		RHM_HT 2013/11/25	Modified
	0x2000,	// gl_KGYG  ;		RHM_HT 2013/11/25	Modified
	0x0200,	// gl_SFTHAL_X;		RHM_HT 2013/11/25	Added
	0x0200,	// gl_SFTHAL_Y;		RHM_HT 2013/11/25	Added
	0x0000,	// gl_TMP_X_;		RHM_HT 2013/11/25	Added
	0x0000,	// gl_TMP_Y_;		RHM_HT 2013/11/25	Added
	0x0000,	// gl_KgxH0;		RHM_HT 2013/11/25	Added
	0x0000,	// gl_KgyH0;		RHM_HT 2013/11/25	Added
*/
};


// *********************************************************
// Read Factory Adjusted data from the non-volatile memory
// ---------------------------------------------------------
// <Function>
//		Factory adjusted data are sotred somewhere
//		non-volatile memory.  I2C master has to read these
//		data and store the data to the OIS controller.
//
// <Input>
//		none
//
// <Output>
//		_FACT_ADJ	Factory Adjusted data
//
// <Description>
//		You have to port your own system.
//
// *********************************************************
_FACT_ADJ	get_FADJ_MEM_from_non_volatile_memory( void )
{
	/* 	Read from the non-vollatile memory such as EEPROM or internal of the CMOS sensor... */	
	
	return FADJ_MEM;		// Note: This return data is for DEBUG.
}

// ==> RHM_HT 2013/04/15	Add for DEBUG
// *********************************************************
// Printf for DEBUG
// ---------------------------------------------------------
// <Function>
//
// <Input>
//		const char *format, ...	
// 				Same as printf
//
// <Output>
//		none
//
// <Description>
//
// *********************************************************
int debug_print(const char *format, ...)
{
	return 0;
		//Darcy deleted/20140620
}


