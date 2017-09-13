//********************************************************************************
//
//		<< LC898122 Evaluation Soft>>
//		Program Name	: Ois.h
// 		Explanation		: LC898122 Global Declaration & ProtType Declaration
//		Design			: Y.Yamada
//		History			: First edition						2009.07.30 Y.Tashita
//********************************************************************************
#ifndef _OIS_H_
#define _OIS_H_
#define	FW_VER			0x0017
#ifdef	OISINI
	#define	OISINI__
#else
	#define	OISINI__		extern
#endif







#ifdef	OISCMD
	#define	OISCMD__
#else
	#define	OISCMD__		extern
#endif


// Define According To Usage

/****************************** Define説明 ******************************/
/*	USE_3WIRE_DGYRO		Digital Gyro I/F 3線Mode使用					*/
/*	USE_PANASONIC		Pansonic Digital Gyro使用						*/
/*	USE_INVENSENSE		Invensense Digital Gyro使用						*/
/*		USE_IDG2020		Inv IDG-2020使用								*/
/*	USE_STMICRO_L3G4IS	ST-Micro Digital Gyro L3G4IS使用				*/
/*	STANDBY_MODE		Standby制御使用(未確認)							*/
/*	GAIN_CONT			:Gain control機能使用							*/
/*		(disable)		DSC			:三脚Mode使用						*/
/*	HALLADJ_HW			Hall Calibration LSI機能使用					*/
/*	NEUTRAL_CENTER		Servo Centerを上向き姿勢の電流0mAとする			*/
/*	INIT_PWMMODE		PWMモード選択 PWM or CVL-PWM					*/
/*							PWMMOD_PWM ->     PWM MODE使用				*/
/*							PWMMOD_CVL -> CVL-PWM MODE使用				*/
/*	ACTREG_6P5OHM		6.5Ωアクチュエータ使用							*/
/*	DEF_SET				LSI初期設定デフォルト値使用						*/
/*	SXQ_INI, SYQ_INI	Hall Filter極性設定								*/
/*	GXHY_GYHX			Gyro Filter -> Hall Filter接続設定				*/
/************************************************************************/

#define	XY_SIMU_SET
//#define	INIT_FAST

/* Select Gyro Sensor */
//#define		USE_3WIRE_DGYRO		//for D-Gyro 3-Wire SPI interface

//#define		USE_PANASONIC		// Panasonic or Other
//#define		USE_STMICRO_L3G4IS	// ST-Micro-L3G4IS
#define		USE_INVENSENSE		// INVENSENSE

#ifdef USE_INVENSENSE
//	#define		FS_SEL		0		/* ±262LSB/°/s  */
//	#define		FS_SEL		1		/* ±131LSB/°/s  */
//	#define		FS_SEL		2		/* ±65.5LSB/°/s  */
	#define		FS_SEL		3		/* ±32.8LSB/°/s  */
	
//	#define		GYROSTBY			/* Sleep+STBY */
	#define		GYROX_INI		0x45	// Gyro X axis select
	#define		GYROY_INI		0x43	// Gyro Y axis select
#endif

#ifdef	USE_STMICRO_L3G4IS
	#define		GYROX_INI		0x43	// Gyro X axis select
	#define		GYROY_INI		0x45	// Gyro Y axis select
#endif

#ifdef USE_PANASONIC
	#define		GYROX_INI		0x7C	// Gyro X axis select
	#define		GYROY_INI		0x78	// Gyro Y axis select
#endif


/* Select Mode */
#define		STANDBY_MODE			// STANDBY Mode
#define		GAIN_CONT				// Gain Control Mode

#define		ACTREG_6P5OHM			// Use 6.5ohm Actuator
//#define		ACTREG_9P20HM
//#define		ACTREG_15P0OHM

#define		PWM_BREAK				// PWM mode select (disable zero cross)
//#define		AF_PWMMODE			// AF Driver PWM mode
//#define		AF_MID_MOUNT			// AF MIDDLE MOUNT

#define		DEF_SET				// default value re-setting
//#define		USE_EXTCLK_ALL		// USE Ext clk for ALL
//#define		USE_EXTCLK_PWM		// USE Ext clk for PWM
//#define		USE_VH_SYNC			// USE V/H Sync for PWM
//#define		PWM_CAREER_TEST		// PWM_CAREER_TEST
#define		MONITOR_OFF			// default Monitor output

//#define		HALLADJ_HW			// H/W Hall adjustment 
#define		NEUTRAL_CENTER			// Upper Position Current 0mA Measurement
//#define		MODULE_CALIBRATION		// for module maker   use float

#ifdef	MODULE_CALIBRATION
// #define	OSC_I2CCK				// Adj by I2C Clk
// #define	OSC_EXCLK				// Adj by Ex-Clk
#endif

#define		H1COEF_CHANGER			/* H1 coef lvl chage */

//#define		NEW_PTST				// method of Pan/Tilt

/* OIS Calibration Parameter */
 #define		DAHLXO_INI		0x0000		// Hall X Offset
 #define		DAHLXB_INI		0xC000		// Hall X Bias
 #define		DAHLYO_INI		0x0000		// Hall Y Offset
 #define		DAHLYB_INI		0xC000		// Hall X Bias
 #define		HXOFF0Z_INI		0x0000		// Hall X AD Offset
 #define		HYOFF1Z_INI		0x0000		// Hall Y AD Offset
// #define		OPTCEN_X		0x0000		// Hall X Optical Offset
// #define		OPTCEN_Y		0x0000		// Hall Y Optical Offset
 #define		SXGAIN_INI		0x2000		// Hall X Loop Gain
 #define		SYGAIN_INI		0x2000		// Hall Y Loop Gain
 #define		DGYRO_OFST_XH	0x00		// Digital Gyro X AD Offset H
 #define		DGYRO_OFST_XL	0x00		// Digital Gyro X AD Offset L
 #define		DGYRO_OFST_YH	0x00		// Digital Gyro Y AD Offset H
 #define		DGYRO_OFST_YL	0x00		// Digital Gyro Y AD Offset L
 #define		GXGAIN_INI		0x3F333333	// Gyro X Zoom Value
 #define		GYGAIN_INI		0xBF333333	// Gyro Y Zoom Value
 #define		OSC_INI			0x2C		/* OSC Init, VDD=2.8V */

/* Actuator Select */
/* Hall parameter */
//20M
#define		BIAS_CUR_OIS_20M	0x44		// 3.0mA/3.0mA
#define		AMP_GAIN_X_20M		0x03		// x75
#define		AMP_GAIN_Y_20M		0x03		// x75
//13M
#define		BIAS_CUR_OIS_13M	0x33		// 2.0mA/2.0mA
#define		AMP_GAIN_X_13M		0x05		// x150
#define		AMP_GAIN_Y_13M		0x05		// x150

#define BIAS_CUR_OIS_13MO 0x44 /* 2.0mA/2.0mA */
#define		AMP_GAIN_X_13MO		0x03		// x150
#define		AMP_GAIN_Y_13MO		0x03		// x150

/* AF Open parameter */
//20M
#define		RWEXD1_L_AF_20M		0x7FFF		//0x7FFF
#define		RWEXD2_L_AF_20M		0x113E		//0x4a02
#define		RWEXD3_L_AF_20M		0x7211		//0x7d62
#define		FSTCTIME_AF_20M		0xA9		// (19.04 x 1000)/64 -128 = 169.5(A9h)
#define		FSTMODE_AF_20M		0x00		//
//13M
#define		RWEXD1_L_AF_13M		0x7FFF		//
#define		RWEXD2_L_AF_13M		0x4a02		//
#define		RWEXD3_L_AF_13M		0x7d62		//
#define		FSTCTIME_AF_13M		0xF9		//
#define		FSTMODE_AF_13M		0x02		//

#define     RWEXD1_L_AF_13MO    0x7FFF      //
#define     RWEXD2_L_AF_13MO    0x146C      // abe 2014.11.12
#define     RWEXD3_L_AF_13MO    0x6F50      // abe 2014.11.12
#define     FSTCTIME_AF_13MO    0xD1        // abe 2014.11.12
#define     FSTMODE_AF_13MO     0x00        // abe 2014.06.03

#ifdef	ACTREG_6P5OHM
/* (0.3750114X^3+0.5937681X)*(0.3750114X^3+0.5937681X) 6.5ohm*/
#define         A3_IEXP3        0x3EC0017F
#define         A1_IEXP1        0x3F180130
#endif	//ACTREG_6P5OHM

#ifdef	ACTREG_9P20HM
/* (0.3750114X^3+0.55X)*(0.3750114X^3+0.55X) 9.2ohm*/
#define         A3_IEXP3        0x3EC0017F
#define         A1_IEXP1        0x3F0CCCCD
#endif	//ACTREG_9P20HM

#ifdef	ACTREG_15P0OHM
/* (0.4531388X^3+0.4531388X)*(0.4531388X^3+0.4531388X) 15ohm*/
#define         A3_IEXP3        0x3EE801CF
#define         A1_IEXP1        0x3EE801CF
#endif	//ACTREG_15P0OHM

/* AF adjust parameter */
#define		DAHLZB_INI		0x9000
#define		DAHLZO_INI		0x0000
#define		BIAS_CUR_AF		0x00			//0.25mA
#define		AMP_GAIN_AF		0x00			//x6

//#ifdef	AF_MID_MOUNT
#define		AFDROF_INI		0x10			// 
//#endif	//AF_MID_MOUNT

/*** Hall, Gyro Parameter Setting ***/
/* Hall Parameter */
 #define		SXGAIN_LOP		0x3000		// X Loop Gain Adjust Sin Wave Amplitude
 #define		SYGAIN_LOP		0x3000		// Y Loop Gain Adjust Sin Wave Amplitude
 
 #define		SXQ_INI			0x3F800000	/* Hall Filter Connection Setting(sxq, syq) */
 #define		SYQ_INI			0xBF800000		// 0x3F800000 -> Positive
												// 0xBF800000 -> Negative
/* Gyro Parameter */
#define		GXHY_GYHX		0				/* Gyro Filter Connection Setting */
												// 0 : GyroX -> HallX, GyroY -> HallY 
												// 1 : GyroX -> HallY, GyroY -> HallX 

#define		TCODEH_ADJ		0x0000

//20M
#define		GYRLMT1H_20M		0x3DCCCCC0	//0.1F
#define		GYRLMT3_S1_20M		0x3EE66666	//0.45F
#define		GYRLMT3_S2_20M		0x3EE66666	//0.45F

#define		GYRLMT4_S1_20M		0x40400000	//3.0F
#define		GYRLMT4_S2_20M		0x40400000	//3.0F

#define		GYRA12_HGH_20M		0x40000000	/* 2.00F */
#define		GYRA12_MID_20M		0x3F800000	/* 1.0F */
#define		GYRA34_HGH_20M		0x3F000000	/* 0.5F */
#define		GYRA34_MID_20M		0x3DCCCCCD	/* 0.1F */

#define		GYRB12_HGH_20M		0x3E4CCCCD	/* 0.20F */
#define		GYRB12_MID_20M		0x3CA3D70A	/* 0.02F */
#define		GYRB34_HGH_20M		0x3CA3D70A	/* 0.02F */
#define		GYRB34_MID_20M		0x3C23D70A	/* 0.001F */

//13M
#define		GYRLMT1H_13M		0x3DCCCCC0	//0.1F
#define		GYRLMT3_S1_13M		0x3F0F5C29	//0.56F
#define		GYRLMT3_S2_13M		0x3F0F5C29	//0.56F
//#define		GYRLMT3_S1_13M		0x3F000000	//0.5F
//#define		GYRLMT3_S2_13M		0x3F000000	//0.5F

#define		GYRLMT4_S1_13M		0x40333333	//2.8F
#define		GYRLMT4_S2_13M		0x40333333	//2.8F

#define		GYRA12_HGH_13M		0x401CCCCD	/* 2.45F */
#define		GYRA12_MID_13M		0x3FB33333	/* 1.4F */
#define		GYRA34_HGH_13M		0x3F000000	/* 0.5F */
#define		GYRA34_MID_13M		0x3DCCCCCD	/* 0.1F */

#define		GYRB12_HGH_13M		0x3E4CCCCD	/* 0.20F */
#define		GYRB12_MID_13M		0x3CA3D70A	/* 0.02F */
#define		GYRB34_HGH_13M		0x3CA3D70A	/* 0.02F */
#define		GYRB34_MID_13M		0x3C23D70A	/* 0.01F */

// Command Status
#define		EXE_END		0x02		// Execute End (Adjust OK)
#define		EXE_HXADJ	0x06		// Adjust NG : X Hall NG (Gain or Offset)
#define		EXE_HYADJ	0x0A		// Adjust NG : Y Hall NG (Gain or Offset)
#define		EXE_LXADJ	0x12		// Adjust NG : X Loop NG (Gain)
#define		EXE_LYADJ	0x22		// Adjust NG : Y Loop NG (Gain)
#define		EXE_GXADJ	0x42		// Adjust NG : X Gyro NG (offset)
#define		EXE_GYADJ	0x82		// Adjust NG : Y Gyro NG (offset)
#define		EXE_OCADJ	0x402		// Adjust NG : OSC Clock NG
#define		EXE_AFOFF	0x802		// Adjust NG : AF Offset
#define		EXE_ERR		0x99		// Execute Error End

// Common Define
#define	SUCCESS			0x00		// Success
#define	FAILURE			0x01		// Failure

#ifndef ON
 #define	ON			0x01		// ON
 #define	OFF			0x00		// OFF
#endif
 #define	SPC			0x02		// Special Mode

#define	X_DIR			0x00		// X Direction
#define	Y_DIR			0x01		// Y Direction
#define	X2_DIR			0x10		// X Direction
#define	Y2_DIR			0x11		// Y Direction

#define	NOP_TIME		0.00004166F

#ifdef STANDBY_MODE
 // Standby mode
 #define		STB1_ON		0x00		// Standby1 ON
 #define		STB1_OFF	0x01		// Standby1 OFF
 #define		STB2_ON		0x02		// Standby2 ON
 #define		STB2_OFF	0x03		// Standby2 OFF
 #define		STB3_ON		0x04		// Standby3 ON
 #define		STB3_OFF	0x05		// Standby3 OFF
 #define		STB4_ON		0x06		// Standby4 ON			/* for Digital Gyro Read */
 #define		STB4_OFF	0x07		// Standby4 OFF
 #define		STB2_OISON	0x08		// Standby2 ON (only OIS)
 #define		STB2_OISOFF	0x09		// Standby2 OFF(only OIS)
 #define		STB2_AFON	0x0A		// Standby2 ON (only AF)
 #define		STB2_AFOFF	0x0B		// Standby2 OFF(only AF)
#endif

/* Optical Center & Gyro Gain for Mode */
 #define	VAL_SET				0x00		// Setting mode
 #define	VAL_FIX				0x01		// Fix Set value
 #define	VAL_SPC				0x02		// Special mode


struct STFILREG {
	unsigned short	UsRegAdd ;
	unsigned char	UcRegDat ;
} ;													// Register Data Table

struct STFILRAM {
	unsigned short	UsRamAdd ;
	unsigned long	UlRamDat ;
} ;													// Filter Coefficient Table

struct STCMDTBL
{
	unsigned short Cmd ;
	unsigned int UiCmdStf ;
	void ( *UcCmdPtr )( void ) ;
} ;

/*** caution [little-endian] ***/

// Word Data Union
union	WRDVAL{
	unsigned short	UsWrdVal ;
	unsigned char	UcWrkVal[ 2 ] ;
	struct {
		unsigned char	UcLowVal ;
		unsigned char	UcHigVal ;
	} StWrdVal ;
} ;

typedef union WRDVAL	UnWrdVal ;

union	DWDVAL {
	unsigned long	UlDwdVal ;
	unsigned short	UsDwdVal[ 2 ] ;
	struct {
		unsigned short	UsLowVal ;
		unsigned short	UsHigVal ;
	} StDwdVal ;
	struct {
		unsigned char	UcRamVa0 ;
		unsigned char	UcRamVa1 ;
		unsigned char	UcRamVa2 ;
		unsigned char	UcRamVa3 ;
	} StCdwVal ;
} ;

typedef union DWDVAL	UnDwdVal;

// Float Data Union
union	FLTVAL {
	float			SfFltVal ;
	unsigned long	UlLngVal ;
	unsigned short	UsDwdVal[ 2 ] ;
	struct {
		unsigned short	UsLowVal ;
		unsigned short	UsHigVal ;
	} StFltVal ;
} ;

typedef union FLTVAL	UnFltVal ;


typedef struct STADJPAR {
	struct {
		unsigned char	UcAdjPhs ;				// Hall Adjust Phase

		unsigned short	UsHlxCna ;				// Hall Center Value after Hall Adjust
		unsigned short	UsHlxMax ;				// Hall Max Value
		unsigned short	UsHlxMxa ;				// Hall Max Value after Hall Adjust
		unsigned short	UsHlxMin ;				// Hall Min Value
		unsigned short	UsHlxMna ;				// Hall Min Value after Hall Adjust
		unsigned short	UsHlxGan ;				// Hall Gain Value
		unsigned short	UsHlxOff ;				// Hall Offset Value
		unsigned short	UsAdxOff ;				// Hall A/D Offset Value
		unsigned short	UsHlxCen ;				// Hall Center Value

		unsigned short	UsHlyCna ;				// Hall Center Value after Hall Adjust
		unsigned short	UsHlyMax ;				// Hall Max Value
		unsigned short	UsHlyMxa ;				// Hall Max Value after Hall Adjust
		unsigned short	UsHlyMin ;				// Hall Min Value
		unsigned short	UsHlyMna ;				// Hall Min Value after Hall Adjust
		unsigned short	UsHlyGan ;				// Hall Gain Value
		unsigned short	UsHlyOff ;				// Hall Offset Value
		unsigned short	UsAdyOff ;				// Hall A/D Offset Value
		unsigned short	UsHlyCen ;				// Hall Center Value
	} StHalAdj ;

	struct {
		unsigned short	UsLxgVal ;				// Loop Gain X
		unsigned short	UsLygVal ;				// Loop Gain Y
		unsigned short	UsLxgSts ;				// Loop Gain X Status
		unsigned short	UsLygSts ;				// Loop Gain Y Status
	} StLopGan ;

	struct {
		unsigned short	UsGxoVal ;				// Gyro A/D Offset X
		unsigned short	UsGyoVal ;				// Gyro A/D Offset Y
		unsigned short	UsGxoSts ;				// Gyro Offset X Status
		unsigned short	UsGyoSts ;				// Gyro Offset Y Status
	} StGvcOff ;
	
//#ifdef	AF_MID_MOUNT
	struct {
		unsigned char	UcAfOff ;				// AF Offset
	} StAfOff ;
//#endif	//AF_MID_MOUNT
	
	unsigned char		UcOscVal ;				// OSC value

} stAdjPar ;

OISCMD__	stAdjPar	StAdjPar ;				// Execute Command Parameter

OISCMD__	unsigned char	UcOscAdjFlg ;		// For Measure trigger
  #define	MEASSTR		0x01
  #define	MEASCNT		0x08
  #define	MEASFIX		0x80

OISINI__	unsigned short	UsCntXof ;			/* OPTICAL Center Xvalue */
OISINI__	unsigned short	UsCntYof ;			/* OPTICAL Center Yvalue */

OISINI__	unsigned char	UcPwmMod ;			/* PWM MODE */
#define		PWMMOD_CVL	0x00						// CVL PWM MODE
#define		PWMMOD_PWM	0x01						// PWM MODE

#define		INIT_PWMMODE	PWMMOD_CVL			/* PWM/CVL MODE select */
													// PWMMOD_PWM ->     PWM MODE
													// PWMMOD_CVL -> CVL-PWM MODE


OISINI__	unsigned char	UcCvrCod ;			/* CverCode */
 #define	CVER122		0x93					 // LC898122

OISINI__	unsigned char	UcModule ;			/* Module */
	#define	MODULE_13M	0x01					//13M
	#define MODULE_13M_OFLM	0x03					//13M
	#define	MODULE_20M	0x02					//20M

OISINI__	unsigned char	UcAfType ;			/* AF Type */
	#define	UNI_DIR		0x01
	#define	BI_DIR		0x02

// Prottype Declation
OISINI__ void	SelectModule( unsigned char ) ;										// Select Module
OISINI__ void	IniSet( void ) ;													// Initial Top Function
OISINI__ void	IniSetAf( void ) ;													// Initial Top Function

OISINI__ void	ClrGyr( unsigned short, unsigned char ); 							   // Clear Gyro RAM
	#define CLR_FRAM0		 	0x01
	#define CLR_FRAM1 			0x02
	#define CLR_ALL_RAM 		0x03
OISINI__ void	BsyWit( unsigned short, unsigned char ) ;				// Busy Wait Function
OISINI__ void	WitTim( unsigned short ) ;								// Wait
	 #define	WTLE 			10										// Waiting Time For Loop Escape
OISINI__ void	MemClr( unsigned char *, unsigned short ) ;				// Memory Clear Function
OISINI__ void	GyOutSignal( void ) ;									// Slect Gyro Output signal Function
OISINI__ void	GyOutSignalCont( void ) ;								// Slect Gyro Output Continuos Function
#ifdef STANDBY_MODE
OISINI__ void	AccWit( unsigned char ) ;								// Acc Wait Function
OISINI__ void	SelectGySleep( unsigned char ) ;						// Select Gyro Mode Function
#endif
#ifdef	GAIN_CONT
OISINI__ void	AutoGainControlSw( unsigned char ) ;					// Auto Gain Control Sw
#endif
OISINI__ void	DrvSw( unsigned char UcDrvSw ) ;						// Driver Mode setting function
OISINI__ void	AfDrvSw( unsigned char UcDrvSw ) ;						// AF Driver Mode setting function
OISINI__ void	RamAccFixMod( unsigned char ) ;							// Ram Access Fix Mode setting function
OISINI__ void	IniPtMovMod( unsigned char ) ;							// Pan/Tilt parameter setting by mode function
OISINI__ void	ChkCvr( void ) ;										// Check Function
	
OISCMD__ void			SrvCon( unsigned char, unsigned char ) ;					// Servo ON/OFF
OISCMD__ unsigned short	TneRun( void ) ;											// Hall System Auto Adjustment Function
OISCMD__ unsigned char	RtnCen( unsigned char ) ;									// Return to Center Function
OISCMD__ void			OisEna( void ) ;											// OIS Enable Function
OISCMD__ void			OisEnaLin( void ) ;											// OIS Enable Function for Line adjustment
OISCMD__ void			TimPro( void ) ;											// Timer Interrupt Process Function
OISCMD__ void			S2cPro( unsigned char ) ;									// S2 Command Process Function
	//20M
	#define		DIFIL_S2_20M		0x3F7FFD00
	//13M
	#define		DIFIL_S2_13M		0x3F7FFE00
OISCMD__ void			SetSinWavePara( unsigned char , unsigned char ) ;			// Sin wave Test Function
	#define		SINEWAVE	0
	#define		XHALWAVE	1
	#define		YHALWAVE	2
	#define		CIRCWAVE	255
OISCMD__ unsigned char	TneGvc( void ) ;											// Gyro VC Offset Adjust

OISCMD__ void			SetZsp( unsigned char ) ;									// Set Zoom Step parameter Function
OISCMD__ void			OptCen( unsigned char, unsigned short, unsigned short ) ;	// Set Optical Center adjusted value Function
OISCMD__ void			StbOnnN( unsigned char , unsigned char ) ;					// Stabilizer For Servo On Function
#ifdef	MODULE_CALIBRATION
OISCMD__ unsigned char	LopGan( unsigned char ) ;									// Loop Gain Adjust
#endif
#ifdef STANDBY_MODE
 OISCMD__ void			SetStandby( unsigned char ) ;								/* Standby control	*/
#endif
#ifdef	MODULE_CALIBRATION
OISCMD__ unsigned short	OscAdj( void ) ;											/* OSC clock adjustment */
OISCMD__ unsigned short	OscAdjA( unsigned short ) ;									/* OSC clock adjustment Semi Auto */
#endif

#ifdef	HALLADJ_HW
 #ifdef	MODULE_CALIBRATION
 OISCMD__ unsigned char	LoopGainAdj(   unsigned char );
 #endif
 OISCMD__ unsigned char	BiasOffsetAdj( unsigned char , unsigned char );
#endif
OISCMD__ void			GyrGan( unsigned char , unsigned long , unsigned long ) ;	/* Set Gyro Gain Function */
OISCMD__ void			SetPanTiltMode( unsigned char ) ;							/* Pan_Tilt control Function */
#ifndef	HALLADJ_HW
 OISCMD__ unsigned long	TnePtp( unsigned char, unsigned char ) ;					// Get Hall Peak to Peak Values
 #define		HALL_H_VAL	0x3F800000												/* 1.0 */

 OISCMD__ unsigned char	TneCen( unsigned char, UnDwdVal ) ;							// Tuning Hall Center
 #define		PTP_BEFORE		0
 #define		PTP_AFTER		1
#endif
#ifdef GAIN_CONT
OISCMD__ unsigned char	TriSts( void ) ;											// Read Status of Tripod mode Function
#endif
OISCMD__ unsigned char	DrvPwmSw( unsigned char ) ;									// Select Driver mode Function
	#define		Mlnp		0					// Linear PWM
	#define		Mpwm		1					// PWM
 #ifdef	NEUTRAL_CENTER																// Gyro VC Offset Adjust
 OISCMD__ unsigned char	TneHvc( void ) ;											// Hall VC Offset Adjust
 #endif	//NEUTRAL_CENTER
OISCMD__ void			SetGcf( unsigned char ) ;									// Set DI filter coefficient Function
OISCMD__	unsigned long	UlH1Coefval ;		// H1 coefficient value
#ifdef H1COEF_CHANGER
 OISCMD__	unsigned char	UcH1LvlMod ;		// H1 level coef mode
 OISCMD__	void			SetH1cMod( unsigned char ) ;							// Set H1C coefficient Level chang Function
 #define		S2MODE		0x40
 #define		ACTMODE		0x80
 #define		MOVMODE		0xFF
#endif
#define SUNNY_MODULE 0x02
#define LG_MODULE  0x03
OISCMD__	unsigned short	RdFwVr( void ) ;										// Read Fw Version Function

//#ifdef	AF_MID_MOUNT
OISCMD__	void			SetTregAf( unsigned short );							// 11bit
OISCMD__	unsigned short	AfMidOffAdj( void );									// 
OISCMD__	unsigned char	GetDOFSTDAF( void );									// 
OISCMD__	void			SetDOFSTDAF( unsigned char ucSetDat );					// 
OISCMD__	void			SetDOFSTDAF_WT( unsigned char ucSetDat );				// 
OISCMD__	unsigned long	MesMSABS1AV( void );									// 
//#else	//
//OISCMD__	void			SetTregAf( unsigned short );							// 10bit
//#endif	//AF_MID_MOUNT
void RemOff( unsigned char UcMod );
void Ois_Init( void );
void Ois_Write_OTP(void);

void RegWriteA(unsigned short RegAddr,unsigned char RegData);
void RegReadA(unsigned short RegAddr,unsigned char *RegData);


void RamWriteA(unsigned short RameAddr,unsigned short RamData);
void RamReadA(unsigned short RameAddr,unsigned short *RamData);


void RamWrite32A(unsigned short RameAddr,unsigned long RamData);
void RamRead32A(unsigned short RameAddr,unsigned long *RamData);
void SmoothSvrOff(void);

OISCMD__ void OisDisable(void); /*OIS Disable Function*/

#endif
