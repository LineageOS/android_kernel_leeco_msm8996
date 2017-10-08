//********************************************************************************
//
//		<< LC898122 Evaluation Soft >>
//		Program Name	: OisIni.c
//		Design			: Y.Yamada
//		History			: LC898122 						2013.01.09 Y.Shigeoka
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#define		OISINI

#include <linux/module.h>
#include <linux/delay.h>

#include	"Ois.h"
#ifdef	INIT_FAST
#include	"OisFil.h"
#else	//INIT_FAST
#include	"OisFil_org.h"
#endif	//INIT_FAST
#include	"OisDef.h"


//**************************
//	Local Function Prottype	
//**************************
void	IniClk( void ) ;		// Clock Setting
void	IniIop( void ) ;		// I/O Port Initial Setting
void	IniMon( void ) ;		// Monitor & Other Initial Setting
void	IniSrv( void ) ;		// Servo Register Initial Setting
void	IniGyr( void ) ;		// Gyro Filter Register Initial Setting
void	IniFil( void ) ;		// Gyro Filter Initial Parameter Setting
void	IniAdj( void ) ;		// Adjust Fix Value Setting
void	IniCmd( void ) ;		// Command Execute Process Initial
void	IniDgy( void ) ;		// Digital Gyro Initial Setting
void	IniAf( void ) ;			// Open AF Initial Setting
void	IniPtAve( void ) ;		// Average setting


//********************************************************************************
// Function Name 	: SelectModule
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: 
// History			: First edition 						2015.01.10 T.Tokoro
//********************************************************************************
void	SelectModule( unsigned char uc_module )
{
	switch( uc_module ){
		case MODULE_13M:
			pr_err("13M  sharp module\n");
			UcAfType = UNI_DIR;
			UcModule = MODULE_13M;
			break;
		case MODULE_20M:
			pr_err("21M  sharp module\n");
			UcAfType = BI_DIR;
			UcModule = MODULE_20M;
			break;
		case MODULE_13M_OFLM:
			pr_err("13M  ofilm module\n");
			UcAfType = UNI_DIR;
			UcModule = MODULE_13M_OFLM;
			break;
		default:
			UcAfType = UNI_DIR;
			UcModule = MODULE_13M;
			break;
	}
}

//********************************************************************************
// Function Name 	: IniSet
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial Setting Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniSet( void )
{
	// Command Execute Process Initial
	IniCmd() ;
	// Clock Setting
	IniClk() ;
	// I/O Port Initial Setting
	IniIop() ;
	// DigitalGyro Initial Setting
	IniDgy() ;
	// Monitor & Other Initial Setting
	IniMon() ;
	// Servo Initial Setting
	IniSrv() ;
	// Gyro Filter Initial Setting
	IniGyr() ;
	// Gyro Filter Initial Setting
	IniFil() ;
	// Adjust Fix Value Setting
	IniAdj() ;
	IniAf() ;

}

//********************************************************************************
// Function Name 	: IniSetAf
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial AF Setting Function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	IniSetAf( void )
{
	// Command Execute Process Initial
	IniCmd() ;
	// Clock Setting
	IniClk() ;
	// AF Initial Setting
	IniAf() ;

}



//********************************************************************************
// Function Name 	: IniClk
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clock Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniClk( void )
{
	ChkCvr() ;									/* Read Cver */
	
	/*OSC Enables*/
	UcOscAdjFlg	= 0 ;							// Osc adj flag 
	
#ifdef	DEF_SET
	/*OSC ENABLE*/
	RegWriteA( OSCSTOP,		0x00 ) ;			// 0x0256
	RegWriteA( OSCSET,		0x90 ) ;			// 0x0257	OSC ini
	RegWriteA( OSCCNTEN,	0x00 ) ;			// 0x0258	OSC Cnt disable
#endif
	/*Clock Enables*/
	RegWriteA( CLKON,		0x1F ) ;			// 0x020B

#ifdef	USE_EXTCLK_ALL
	RegWriteA( CLKSEL,		0x07 ) ;			// 0x020C	All
#else
 #ifdef	USE_EXTCLK_PWM
	RegWriteA( CLKSEL,		0x01 ) ;			// 0x020C	only PWM
 #else
  #ifdef	DEF_SET
	RegWriteA( CLKSEL,		0x00 ) ;			// 0x020C	
  #endif
 #endif
#endif
	
#ifdef	USE_EXTCLK_ALL	// 24MHz
	RegWriteA( PWMDIV,		0x00 ) ;			// 0x0210	24MHz/1
	RegWriteA( SRVDIV,		0x00 ) ;			// 0x0211	24MHz/1
	RegWriteA( GIFDIV,		0x02 ) ;			// 0x0212	24MHz/2 = 12MHz
	RegWriteA( AFPWMDIV,	0x00 ) ;			// 0x0213	24MHz/1 = 24MHz
	RegWriteA( OPAFDIV,		0x02 ) ;			// 0x0214	24MHz/2 = 12MHz
#else
 #ifdef	DEF_SET
	RegWriteA( PWMDIV,		0x00 ) ;			// 0x0210	48MHz/1
	RegWriteA( SRVDIV,		0x00 ) ;			// 0x0211	48MHz/1
	RegWriteA( GIFDIV,		0x03 ) ;			// 0x0212	48MHz/3 = 16MHz
  #ifdef	AF_PWMMODE
	RegWriteA( AFPWMDIV,	0x00 ) ;			// 0x0213	48MHz/1
  #else
	RegWriteA( AFPWMDIV,	0x02 ) ;			// 0x0213	48MHz/2 = 24MHz
  #endif

	if( UcModule == MODULE_20M ){	//20M
		RegWriteA( OPAFDIV,		0x06 ) ;		// 0x0214	48MHz/6 = 8MHz
	}
	else{							//13M
		RegWriteA( OPAFDIV,		0x04 ) ;		// 0x0214	48MHz/4 = 12MHz
	}

 #endif
#endif
}



//********************************************************************************
// Function Name 	: IniIop
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: I/O Port Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniIop( void )
{
#ifdef	DEF_SET
	/*set IOP direction*/
	RegWriteA( P0LEV,		0x00 ) ;	// 0x0220	[ - 	| - 	| WLEV5 | WLEV4 ][ WLEV3 | WLEV2 | WLEV1 | WLEV0 ]
	RegWriteA( P0DIR,		0x00 ) ;	// 0x0221	[ - 	| - 	| DIR5	| DIR4	][ DIR3  | DIR2  | DIR1  | DIR0  ]
	/*set pull up/down*/
	RegWriteA( P0PON,		0x0F ) ;	// 0x0222	[ -    | -	  | PON5 | PON4 ][ PON3  | PON2 | PON1 | PON0 ]
	RegWriteA( P0PUD,		0x0F ) ;	// 0x0223	[ -    | -	  | PUD5 | PUD4 ][ PUD3  | PUD2 | PUD1 | PUD0 ]
#endif
	/*select IOP signal*/
#ifdef	USE_3WIRE_DGYRO
	RegWriteA( IOP1SEL,		0x02 ); 	// 0x0231	IOP1 : IOP1
#else
	RegWriteA( IOP1SEL,		0x00 ); 	// 0x0231	IOP1 : DGDATAIN (ATT:0236h[0]=1)
#endif
#ifdef	DEF_SET
	RegWriteA( IOP0SEL,		0x02 ); 	// 0x0230	IOP0 : IOP0
	RegWriteA( IOP2SEL,		0x02 ); 	// 0x0232	IOP2 : IOP2
	RegWriteA( IOP3SEL,		0x00 ); 	// 0x0233	IOP3 : DGDATAOUT
	RegWriteA( IOP4SEL,		0x00 ); 	// 0x0234	IOP4 : DGSCLK
	RegWriteA( IOP5SEL,		0x00 ); 	// 0x0235	IOP5 : DGSSB
	RegWriteA( DGINSEL,		0x00 ); 	// 0x0236	DGDATAIN 0:IOP1 1:IOP2
	RegWriteA( I2CSEL,		0x00 );		// 0x0248	I2C noise reduction ON
	RegWriteA( DLMODE,		0x00 );		// 0x0249	Download OFF
#endif
	
}

//********************************************************************************
// Function Name 	: IniDgy
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Digital Gyro Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniDgy( void )
{
 #ifdef USE_INVENSENSE
	unsigned char	UcGrini ;
 #endif

	/*************/
	/*For ST gyro*/
	/*************/
	
	/*Set SPI Type*/
 #ifdef USE_3WIRE_DGYRO
	RegWriteA( SPIM 	, 0x00 );					// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
 #else
	RegWriteA( SPIM 	, 0x01 );					// 0x028F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
 #endif												// 		DGSPI4	0: 3-wire SPI, 1: 4-wire SPI
	
	/*Set to Command Mode*/
	RegWriteA( GRSEL	, 0x01 );					// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

	/*Digital Gyro Read settings*/
	RegWriteA( GRINI	, 0x80 );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]

 #ifdef USE_STMICRO_L3G4IS
  	RegWriteA( LSBF	, 0x03 );						// 0x028D
	
	/*Set Digital Gyro Settings*/
	RegWriteA( GRADR0	, 0x0B );					// 0x0283
	RegWriteA( GSETDT	, 0x0F );					// 0x028A	Enable BDU, Enable 3-wire SPI mode
	RegWriteA( GRACC	, 0x10 );					// 0x0282	[ ADRPLUS(1:0) | - | WR1B ][ - | RD4B | RD2B | RD1B ]
	AccWit( 0x10 ) ;								/* Digital Gyro busy wait 				*/
	
	/*Set Digital Gyro Settings*/
	RegWriteA( GRADR0	, 0x0D );					// 0x0283
	RegWriteA( GSETDT	, 0x01 );					// 0x028A	LPF = 300Hz
	RegWriteA( GRACC	, 0x10 );					// 0x0282	[ ADRPLUS(1:0) | - | WR1B ][ - | RD4B | RD2B | RD1B ]
	AccWit( 0x10 ) ;								/* Digital Gyro busy wait 				*/
 #endif // USE_STMICRO_L3G4IS

 #ifdef USE_INVENSENSE
	RegReadA( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	RegWriteA( GRINI, ( UcGrini | SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	
	RegWriteA( GRADR0,	0x6A ) ;					// 0x0283	Set I2C_DIS
	RegWriteA( GSETDT,	0x10 ) ;					// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	AccWit( 0x10 ) ;								/* Digital Gyro busy wait 				*/

	RegWriteA( GRADR0,	0x1B ) ;					// 0x0283	Set GYRO_CONFIG
	RegWriteA( GSETDT,	( FS_SEL << 3) ) ;			// 0x028A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0282	Set Trigger ON				*/
	AccWit( 0x10 ) ;								/* Digital Gyro busy wait 				*/

	RegReadA( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
	RegWriteA( GRINI, ( UcGrini & ~SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
 #endif // USE_INVENSENSE

 #ifdef USE_PANASONIC		/* 1Panasonic 	EWTS9P */
	RegWriteA( PANAM	, 0x09 );					// 0x028E	Panasonic mode, Burst Output
	RegWriteA( REVB7	, 0x03 );					// 0x028C	Reverse bit7
 #endif // USE_PANASONIC
	
	RegWriteA( RDSEL,	0x7C ) ;					// 0x028B	RDSEL(Data1 and 2 for continuos mode)
	
	GyOutSignal() ;
}


//********************************************************************************
// Function Name 	: IniMon
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Monitor & Other Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniMon( void )
{
	RegWriteA( PWMMONA, 0x00 ) ;				// 0x0030	0:off
	
	RegWriteA( MONSELA, 0x5C ) ;				// 0x0270	DLYMON1
	RegWriteA( MONSELB, 0x5D ) ;				// 0x0271	DLYMON2
	RegWriteA( MONSELC, 0x00 ) ;				// 0x0272	
	RegWriteA( MONSELD, 0x00 ) ;				// 0x0273	

	// Monitor Circuit
	RegWriteA( WC_PINMON1,	0x00 ) ;			// 0x01C0	Filter Monitor
	RegWriteA( WC_PINMON2,	0x00 ) ;			// 0x01C1	
	RegWriteA( WC_PINMON3,	0x00 ) ;			// 0x01C2	
	RegWriteA( WC_PINMON4,	0x00 ) ;			// 0x01C3	
	/* Delay Monitor */
	RegWriteA( WC_DLYMON11,	0x04 ) ;			// 0x01C5	DlyMonAdd1[10:8]
	RegWriteA( WC_DLYMON10,	0x40 ) ;			// 0x01C4	DlyMonAdd1[ 7:0]
	RegWriteA( WC_DLYMON21,	0x04 ) ;			// 0x01C7	DlyMonAdd2[10:8]
	RegWriteA( WC_DLYMON20,	0xC0 ) ;			// 0x01C6	DlyMonAdd2[ 7:0]
	RegWriteA( WC_DLYMON31,	0x00 ) ;			// 0x01C9	DlyMonAdd3[10:8]
	RegWriteA( WC_DLYMON30,	0x00 ) ;			// 0x01C8	DlyMonAdd3[ 7:0]
	RegWriteA( WC_DLYMON41,	0x00 ) ;			// 0x01CB	DlyMonAdd4[10:8]
	RegWriteA( WC_DLYMON40,	0x00 ) ;			// 0x01CA	DlyMonAdd4[ 7:0]

/* Monitor */
	RegWriteA( PWMMONA, 0x80 ) ;				// 0x0030	1:on 
//	RegWriteA( IOP0SEL,		0x01 ); 			// 0x0230	IOP0 : MONA
/**/


}

//********************************************************************************
// Function Name 	: IniSrv
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Servo Initial Setting
// History			: First edition 						2013.01.08 Y.Shigeoka
//********************************************************************************
void	IniSrv( void )
{
	unsigned char	UcStbb0 ;

	UcPwmMod = INIT_PWMMODE ;						// Driver output mode

	RegWriteA( WC_EQON,		0x00 ) ;				// 0x0101		Filter Calcu
	RegWriteA( WC_RAMINITON,0x00 ) ;				// 0x0102		
	ClrGyr( 0x0000 , CLR_ALL_RAM );					// All Clear
	
	RegWriteA( WH_EQSWX,	0x02 ) ;				// 0x0170		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	RegWriteA( WH_EQSWY,	0x02 ) ;				// 0x0171		[ - | - | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	
	RamAccFixMod( OFF ) ;							// 32bit Float mode
	
	/* Monitor Gain */
	RamWrite32A( dm1g, 0x3F800000 ) ;				// 0x109A
	RamWrite32A( dm2g, 0x3F800000 ) ;				// 0x109B
	RamWrite32A( dm3g, 0x3F800000 ) ;				// 0x119A
	RamWrite32A( dm4g, 0x3F800000 ) ;				// 0x119B
	
	/* Hall output limitter */
	RamWrite32A( sxlmta1,   0x3F800000 ) ;			// 0x10E6		Hall X output Limit
	RamWrite32A( sylmta1,   0x3F800000 ) ;			// 0x11E6		Hall Y output Limit
	
	/* Emargency Stop */
	RegWriteA( WH_EMGSTPON,	0x00 ) ;				// 0x0178		Emargency Stop OFF
	RegWriteA( WH_EMGSTPTMR,0xFF ) ;				// 0x017A		255*(16/23.4375kHz)=174ms
	
	RamWrite32A( sxemglev,   0x3F800000 ) ;			// 0x10EC		Hall X Emargency threshold
	RamWrite32A( syemglev,   0x3F800000 ) ;			// 0x11EC		Hall Y Emargency threshold
	
	/* Hall Servo smoothing */
	RegWriteA( WH_SMTSRVON,	0x00 ) ;				// 0x017C		Smooth Servo OFF
#ifdef	USE_EXTCLK_ALL	// 24MHz
	RegWriteA( WH_SMTSRVSMP,0x03 ) ;				// 0x017D		2.7ms=2^03/11.718kHz
	RegWriteA( WH_SMTTMR,	0x07 ) ;				// 0x017E		10ms=(0+1)*16/11.718kHz
#else	//USE_EXTCLK_ALL
	RegWriteA( WH_SMTSRVSMP,0x06 ) ;				// 0x017D		2.7ms=2^06/23.4375kHz
	RegWriteA( WH_SMTTMR,	0x0f ) ;				// 0x017E		10ms=(15+1)*16/23.4375kHz
#endif	//USE_EXTCLK_ALL
	
	RamWrite32A( sxsmtav,   0xBC800000 ) ;			// 0x10ED		1/64 X smoothing ave coefficient
	RamWrite32A( sysmtav,   0xBC800000 ) ;			// 0x11ED		1/64 Y smoothing ave coefficient
	RamWrite32A( sxsmtstp,  0x3AE90466 ) ;			// 0x10EE		0.001778 X smoothing offset
	RamWrite32A( sysmtstp,  0x3AE90466 ) ;			// 0x11EE		0.001778 Y smoothing offset
	
	/* High-dimensional correction  */
	RegWriteA( WH_HOFCON,	0x11 ) ;				// 0x0174		OUT 3x3
	
	/* Front */
	RamWrite32A( sxiexp3,   A3_IEXP3 ) ;			// 0x10BA		
	RamWrite32A( sxiexp2,   0x00000000 ) ;			// 0x10BB		
	RamWrite32A( sxiexp1,   A1_IEXP1 ) ;			// 0x10BC		
	RamWrite32A( sxiexp0,   0x00000000 ) ;			// 0x10BD		
	RamWrite32A( sxiexp,    0x3F800000 ) ;			// 0x10BE		

	RamWrite32A( syiexp3,   A3_IEXP3 ) ;			// 0x11BA		
	RamWrite32A( syiexp2,   0x00000000 ) ;			// 0x11BB		
	RamWrite32A( syiexp1,   A1_IEXP1 ) ;			// 0x11BC		
	RamWrite32A( syiexp0,   0x00000000 ) ;			// 0x11BD		
	RamWrite32A( syiexp,    0x3F800000 ) ;			// 0x11BE		

	/* Back */
	RamWrite32A( sxoexp3,   A3_IEXP3 ) ;			// 0x10FA		
	RamWrite32A( sxoexp2,   0x00000000 ) ;			// 0x10FB		
	RamWrite32A( sxoexp1,   A1_IEXP1 ) ;			// 0x10FC		
	RamWrite32A( sxoexp0,   0x00000000 ) ;			// 0x10FD		
	RamWrite32A( sxoexp,    0x3F800000 ) ;			// 0x10FE		

	RamWrite32A( syoexp3,   A3_IEXP3 ) ;			// 0x11FA		
	RamWrite32A( syoexp2,   0x00000000 ) ;			// 0x11FB		
	RamWrite32A( syoexp1,   A1_IEXP1 ) ;			// 0x11FC		
	RamWrite32A( syoexp0,   0x00000000 ) ;			// 0x11FD		
	RamWrite32A( syoexp,    0x3F800000 ) ;			// 0x11FE		
	
	/* Sine wave */
#ifdef	DEF_SET
	RegWriteA( WC_SINON,	0x00 ) ;				// 0x0180		Sin Wave off
	RegWriteA( WC_SINFRQ0,	0x00 ) ;				// 0x0181		
	RegWriteA( WC_SINFRQ1,	0x60 ) ;				// 0x0182		
	RegWriteA( WC_SINPHSX,	0x00 ) ;				// 0x0183		
	RegWriteA( WC_SINPHSY,	0x20 ) ;				// 0x0184		
	
	/* AD over sampling */
	RegWriteA( WC_ADMODE,	0x06 ) ;				// 0x0188		AD Over Sampling
	
	/* Measure mode */
	RegWriteA( WC_MESMODE,		0x00 ) ;			// 0x0190		Measurement Mode
	RegWriteA( WC_MESSINMODE,	0x00 ) ;			// 0x0191		
	RegWriteA( WC_MESLOOP0,		0x08 ) ;			// 0x0192		
	RegWriteA( WC_MESLOOP1,		0x02 ) ;			// 0x0193		
	RegWriteA( WC_MES1ADD0,		0x00 ) ;			// 0x0194		
	RegWriteA( WC_MES1ADD1,		0x00 ) ;			// 0x0195		
	RegWriteA( WC_MES2ADD0,		0x00 ) ;			// 0x0196		
	RegWriteA( WC_MES2ADD1,		0x00 ) ;			// 0x0197		
	RegWriteA( WC_MESABS,		0x00 ) ;			// 0x0198		
	RegWriteA( WC_MESWAIT,		0x00 ) ;			// 0x0199		
	
	/* auto measure */
	RegWriteA( WC_AMJMODE,		0x00 ) ;			// 0x01A0		Automatic measurement mode
	
	RegWriteA( WC_AMJLOOP0,		0x08 ) ;			// 0x01A2		Self-Aadjustment
	RegWriteA( WC_AMJLOOP1,		0x02 ) ;			// 0x01A3		
	RegWriteA( WC_AMJIDL0,		0x02 ) ;			// 0x01A4		
	RegWriteA( WC_AMJIDL1,		0x00 ) ;			// 0x01A5		
	RegWriteA( WC_AMJ1ADD0,		0x00 ) ;			// 0x01A6		
	RegWriteA( WC_AMJ1ADD1,		0x00 ) ;			// 0x01A7		
	RegWriteA( WC_AMJ2ADD0,		0x00 ) ;			// 0x01A8		
	RegWriteA( WC_AMJ2ADD1,		0x00 ) ;			// 0x01A9		
	
	/* Data Pass */
	RegWriteA( WC_DPI1ADD0,		0x00 ) ;			// 0x01B0		Data Pass
	RegWriteA( WC_DPI1ADD1,		0x00 ) ;			// 0x01B1		
	RegWriteA( WC_DPI2ADD0,		0x00 ) ;			// 0x01B2		
	RegWriteA( WC_DPI2ADD1,		0x00 ) ;			// 0x01B3		
	RegWriteA( WC_DPI3ADD0,		0x00 ) ;			// 0x01B4		
	RegWriteA( WC_DPI3ADD1,		0x00 ) ;			// 0x01B5		
	RegWriteA( WC_DPI4ADD0,		0x00 ) ;			// 0x01B6		
	RegWriteA( WC_DPI4ADD1,		0x00 ) ;			// 0x01B7		
	RegWriteA( WC_DPO1ADD0,		0x00 ) ;			// 0x01B8		Data Pass
	RegWriteA( WC_DPO1ADD1,		0x00 ) ;			// 0x01B9		
	RegWriteA( WC_DPO2ADD0,		0x00 ) ;			// 0x01BA		
	RegWriteA( WC_DPO2ADD1,		0x00 ) ;			// 0x01BB		
	RegWriteA( WC_DPO3ADD0,		0x00 ) ;			// 0x01BC		
	RegWriteA( WC_DPO3ADD1,		0x00 ) ;			// 0x01BD		
	RegWriteA( WC_DPO4ADD0,		0x00 ) ;			// 0x01BE		
	RegWriteA( WC_DPO4ADD1,		0x00 ) ;			// 0x01BF		
	RegWriteA( WC_DPON,			0x00 ) ;			// 0x0105		Data pass OFF
	
	/* Interrupt Flag */
	RegWriteA( WC_INTMSK,	0xFF ) ;				// 0x01CE		All Mask
	
#endif
	
	/* Ram Access */
	RamAccFixMod( OFF ) ;							// 32bit float mode

	// PWM Signal Generate
	DrvSw( OFF ) ;									/* 0x0070	Drvier Block Ena=0 */
	RegWriteA( DRVFC2	, 0x90 );					// 0x0002	Slope 3, Dead Time = 30 ns
	RegWriteA( DRVSELX	, 0xFF );					// 0x0003	PWM X drv max current  DRVSELX[7:0]
	RegWriteA( DRVSELY	, 0xFF );					// 0x0004	PWM Y drv max current  DRVSELY[7:0]

#ifdef	PWM_BREAK
 #ifdef	PWM_CAREER_TEST
	RegWriteA( PWMFC,		0x7C ) ;				// 0x0011	VREF, PWMFRQ=7:PWMCLK(EXCLK)/PWMPERIODX[5:2]=18MHz/4=4.5MHz, MODE0B, 11-bit Accuracy
 #else		//PWM_CAREER_TEST
	if( UcCvrCod == CVER122 ) {
		RegWriteA( PWMFC,   0x2D ) ;				// 0x0011	VREF, PWMCLK/256, MODE0B, 12Bit Accuracy
	}
	else {
		RegWriteA( PWMFC,   0x3D ) ;				// 0x0011	VREF, PWMCLK/128, MODE0B, 12Bit Accuracy
	}
 #endif	//PWM_CAREER_TEST
#else
	RegWriteA( PWMFC,   0x21 ) ;					// 0x0011	VREF, PWMCLK/256, MODE1, 12Bit Accuracy
#endif

#ifdef	USE_VH_SYNC
	RegWriteA( STROBEFC,	0x80 ) ;				// 0x001C	�O������Strobe�M���̗L��
	RegWriteA( STROBEDLYX,	0x00 ) ;				// 0x001D	Delay
	RegWriteA( STROBEDLYY,	0x00 ) ;				// 0x001E	Delay
#endif	//USE_VH_SYNC

	RegWriteA( PWMA,    0x00 ) ;					// 0x0010	PWM X/Y standby
	RegWriteA( PWMDLYX,  0x04 ) ;					// 0x0012	X Phase Delay Setting
	RegWriteA( PWMDLYY,  0x04 ) ;					// 0x0013	Y Phase Delay Setting
	
#ifdef	DEF_SET
	RegWriteA( DRVCH1SEL,	0x00 ) ;				// 0x0005	OUT1/OUT2	X axis
	RegWriteA( DRVCH2SEL,	0x00 ) ;				// 0x0006	OUT3/OUT4	Y axis
	
	RegWriteA( PWMDLYTIMX,	0x00 ) ;				// 0x0014		PWM Timing
	RegWriteA( PWMDLYTIMY,	0x00 ) ;				// 0x0015		PWM Timing
#endif
	
	if( UcCvrCod == CVER122 ) {
#ifdef	PWM_CAREER_TEST
		RegWriteA( PWMPERIODY,	0xD0 ) ;			// 0x001A	11010000h --> PWMPERIODX[5:2] = 0100h = 4
		RegWriteA( PWMPERIODY2,	0xD0 ) ;			// 0x001B	11010000h --> PWMPERIODY[5:2] = 0100h = 4
#else		//PWM_CAREER_TEST
		RegWriteA( PWMPERIODY,	0x00 ) ;			// 0x001A		PWM Carrier Freq
		RegWriteA( PWMPERIODY2,	0x00 ) ;			// 0x001B		PWM Carrier Freq
#endif
	}
	else {
#ifdef	PWM_CAREER_TEST
		RegWriteA( PWMPERIODX,	0xF2 ) ;			// 0x0018		PWM Carrier Freq
		RegWriteA( PWMPERIODX2,	0x00 ) ;			// 0x0019		PWM Carrier Freq
		RegWriteA( PWMPERIODY,	0xF2 ) ;			// 0x001A		PWM Carrier Freq
		RegWriteA( PWMPERIODY2,	0x00 ) ;			// 0x001B		PWM Carrier Freq
#else		//PWM_CAREER_TEST
	#ifdef	USE_EXTCLK_PWM
		RegWriteA( PWMPERIODX,	0x84 ) ;			// 0x0018		PWM Carrier Freq
		RegWriteA( PWMPERIODX2,	0x00 ) ;			// 0x0019		PWM Carrier Freq
		RegWriteA( PWMPERIODY,	0x84 ) ;			// 0x001A		PWM Carrier Freq
		RegWriteA( PWMPERIODY2,	0x00 ) ;			// 0x001B		PWM Carrier Freq
	#else	//USE_EXTCLK_PWM
		RegWriteA( PWMPERIODX,	0x00 ) ;			// 0x0018		PWM Carrier Freq
		RegWriteA( PWMPERIODX2,	0x00 ) ;			// 0x0019		PWM Carrier Freq
		RegWriteA( PWMPERIODY,	0x00 ) ;			// 0x001A		PWM Carrier Freq
		RegWriteA( PWMPERIODY2,	0x00 ) ;			// 0x001B		PWM Carrier Freq
	#endif	//USE_EXTCLK_PWM
#endif
	}
	
	/* Linear PWM circuit setting */
	RegWriteA( CVA		, 0xC0 );					// 0x0020	Linear PWM mode enable

	if( UcCvrCod == CVER122 ) {
		RegWriteA( CVFC 	, 0x22 );				// 0x0021	
	}

	RegWriteA( CVFC2 	, 0x80 );					// 0x0022
	if( UcCvrCod == CVER122 ) {
		RegWriteA( CVSMTHX	, 0x00 );				// 0x0023	smooth off
		RegWriteA( CVSMTHY	, 0x00 );				// 0x0024	smooth off
	}

	RegReadA( STBB0 	, &UcStbb0 );				// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0 &= 0x80 ;
	RegWriteA( STBB0, UcStbb0 ) ;					// 0x0250	OIS standby
	
}



//********************************************************************************
// Function Name 	: IniGyr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Setting Initialize Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
#ifdef GAIN_CONT
  #define	TRI_LEVEL		0x3A83126F		/* 0.0010 */
  #define	TIMELOW			0x50			/* */
  #define	TIMEHGH			0x05			/* */
 #ifdef	USE_EXTCLK_ALL	// 24MHz
  #define	TIMEBSE			0x2F			/* 4.0ms */
 #else
  #define	TIMEBSE			0x5D			/* 3.96ms */
 #endif
  #define	MONADR			GXXFZ
  #define	GANADR			gxadj
  #define	XMINGAIN		0x00000000
  #define	XMAXGAIN		0x3F800000
  #define	YMINGAIN		0x00000000
  #define	YMAXGAIN		0x3F800000
  #define	XSTEPUP			0x38D1B717		/* 0.0001	 */
  #define	XSTEPDN			0xBD4CCCCD		/* -0.05 	 */
  #define	YSTEPUP			0x38D1B717		/* 0.0001	 */
  #define	YSTEPDN			0xBD4CCCCD		/* -0.05 	 */
#endif


void	IniGyr( void )
{
#ifdef	NEW_PTST
	UnFltVal		UnGyrLmt ;
#endif	//NEW_PTST
	
	unsigned long	GYRLMT1H;
	unsigned long	GYRLMT3_S1;
	unsigned long	GYRLMT3_S2;
	unsigned long	GYRLMT4_S1;
	unsigned long	GYRLMT4_S2;
	unsigned long	GYRA12_HGH;
	unsigned long	GYRA12_MID;
	unsigned long	GYRA34_HGH;
	unsigned long	GYRA34_MID;
	unsigned long	GYRB12_HGH;
	unsigned long	GYRB12_MID;
	unsigned long	GYRB34_HGH;
	unsigned long	GYRB34_MID;
	
	if( UcModule == MODULE_20M ){
		GYRLMT1H	= GYRLMT1H_20M;
		GYRLMT3_S1	= GYRLMT3_S1_20M;
		GYRLMT3_S2	= GYRLMT3_S2_20M;
		GYRLMT4_S1	= GYRLMT4_S1_20M;
		GYRLMT4_S2	= GYRLMT4_S2_20M;
		GYRA12_HGH	= GYRA12_HGH_20M;
		GYRA12_MID	= GYRA12_MID_20M;
		GYRA34_HGH	= GYRA34_HGH_20M;
		GYRA34_MID	= GYRA34_MID_20M;
		GYRB12_HGH	= GYRB12_HGH_20M;
		GYRB12_MID	= GYRB12_MID_20M;
		GYRB34_HGH	= GYRB34_HGH_20M;
		GYRB34_MID	= GYRB34_MID_20M;
	}
	else{
		GYRLMT1H	= GYRLMT1H_13M;
		GYRLMT3_S1	= GYRLMT3_S1_13M;
		GYRLMT3_S2	= GYRLMT3_S2_13M;
		GYRLMT4_S1	= GYRLMT4_S1_13M;
		GYRLMT4_S2	= GYRLMT4_S2_13M;
		GYRA12_HGH	= GYRA12_HGH_13M;
		GYRA12_MID	= GYRA12_MID_13M;
		GYRA34_HGH	= GYRA34_HGH_13M;
		GYRA34_MID	= GYRA34_MID_13M;
		GYRB12_HGH	= GYRB12_HGH_13M;
		GYRB12_MID	= GYRB12_MID_13M;
		GYRB34_HGH	= GYRB34_HGH_13M;
		GYRB34_MID	= GYRB34_MID_13M;
	}
	
	/*Gyro Filter Setting*/
	RegWriteA( WG_EQSW	, 0x03 );						// 0x0110		[ - | Sw6 | Sw5 | Sw4 ][ Sw3 | Sw2 | Sw1 | Sw0 ]
	
	/*Gyro Filter Down Sampling*/
	
	RegWriteA( WG_SHTON	, 0x10 );						// 0x0107		[ - | - | - | CmSht2PanOff ][ - | - | CmShtOpe(1:0) ]
														//				CmShtOpe[1:0] 00: �V���b�^�[OFF, 01: �V���b�^�[ON, 1x:�O������
										
#ifdef	DEF_SET
	RegWriteA( WG_SHTDLYTMR , 0x00 );					// 0x0117	 	Shutter Delay
	RegWriteA( WG_GADSMP, 	  0x00 );					// 0x011C		Sampling timing
	RegWriteA( WG_HCHR, 	  0x00 );					// 0x011B		H-filter limitter control not USE
	RegWriteA( WG_LMT3MOD , 0x00 );						// 0x0118 	[ - | - | - | - ][ - | - | - | CmLmt3Mod ]
														//				CmLmt3Mod	0: �ʏ탊�~�b�^�[����, 1: �~�̔��a���~�b�^�[����
	RegWriteA( WG_VREFADD , 0x12 );						// 0x0119	 	�Z���^�[�߂����s���x��RAM�̃A�h���X����6�r�b�g�@(default 0x12 = GXH1Z2/GYH1Z2)
#endif
	RegWriteA( WG_SHTMOD , 0x06 );						// 0x0116	 	Shutter Hold mode

	// Limiter
	RamWrite32A( gxlmt1H, GYRLMT1H ) ;					// 0x1028
	RamWrite32A( gylmt1H, GYRLMT1H ) ;					// 0x1128

	RamWrite32A( gxlmt3HS0, GYRLMT3_S1 ) ;				// 0x1029
	RamWrite32A( gylmt3HS0, GYRLMT3_S1 ) ;				// 0x1129
	
	RamWrite32A( gxlmt3HS1, GYRLMT3_S2 ) ;				// 0x102A
	RamWrite32A( gylmt3HS1, GYRLMT3_S2 ) ;				// 0x112A

	RamWrite32A( gylmt4HS0, GYRLMT4_S1 ) ;				// 0x112B	Y��Limiter4 High臒l0
	RamWrite32A( gxlmt4HS0, GYRLMT4_S1 ) ;				// 0x102B	X��Limiter4 High臒l0
	
	RamWrite32A( gxlmt4HS1, GYRLMT4_S2 ) ;				// 0x102C	X��Limiter4 High臒l1
	RamWrite32A( gylmt4HS1, GYRLMT4_S2 ) ;				// 0x112C	Y��Limiter4 High臒l1

	
	/* Pan/Tilt parameter */
	RegWriteA( WG_PANADDA, 		0x12 );					// 0x0130	GXH1Z2/GYH1Z2 Select
	RegWriteA( WG_PANADDB, 		0x09 );					// 0x0131	GXIZ/GYIZ Select
	
	 //Threshold
	RamWrite32A( SttxHis, 	0x00000000 );				// 0x1226
	RamWrite32A( SttxaL, 	0x00000000 );				// 0x109D
	RamWrite32A( SttxbL, 	0x00000000 );				// 0x109E
	RamWrite32A( Sttx12aM, 	GYRA12_MID );				// 0x104F
	RamWrite32A( Sttx12aH, 	GYRA12_HGH );				// 0x105F
	RamWrite32A( Sttx12bM, 	GYRB12_MID );				// 0x106F
	RamWrite32A( Sttx12bH, 	GYRB12_HGH );				// 0x107F
	RamWrite32A( Sttx34aM, 	GYRA34_MID );				// 0x108F
	RamWrite32A( Sttx34aH, 	GYRA34_HGH );				// 0x109F
	RamWrite32A( Sttx34bM, 	GYRB34_MID );				// 0x10AF
	RamWrite32A( Sttx34bH, 	GYRB34_HGH );				// 0x10BF
	RamWrite32A( SttyaL, 	0x00000000 );				// 0x119D
	RamWrite32A( SttybL, 	0x00000000 );				// 0x119E
	RamWrite32A( Stty12aM, 	GYRA12_MID );				// 0x114F
	RamWrite32A( Stty12aH, 	GYRA12_HGH );				// 0x115F
	RamWrite32A( Stty12bM, 	GYRB12_MID );				// 0x116F
	RamWrite32A( Stty12bH, 	GYRB12_HGH );				// 0x117F
	RamWrite32A( Stty34aM, 	GYRA34_MID );				// 0x118F
	RamWrite32A( Stty34aH, 	GYRA34_HGH );				// 0x119F
	RamWrite32A( Stty34bM, 	GYRB34_MID );				// 0x11AF
	RamWrite32A( Stty34bH, 	GYRB34_HGH );				// 0x11BF
	
	// Pan level
	RegWriteA( WG_PANLEVABS, 		0x00 );				// 0x0133
	
	// Average parameter are set IniAdj

	// Phase Transition Setting
	// State 2 -> 1
	RegWriteA( WG_PANSTT21JUG0, 	0x00 );		// 0x0140
	RegWriteA( WG_PANSTT21JUG1, 	0x00 );		// 0x0141
	// State 3 -> 1
	RegWriteA( WG_PANSTT31JUG0, 	0x00 );				// 0x0142
	RegWriteA( WG_PANSTT31JUG1, 	0x00 );				// 0x0143
	// State 4 -> 1
	RegWriteA( WG_PANSTT41JUG0, 	0x01 );				// 0x0144
	RegWriteA( WG_PANSTT41JUG1, 	0x00 );				// 0x0145
	// State 1 -> 2
	RegWriteA( WG_PANSTT12JUG0, 	0x00 );				// 0x0146
	RegWriteA( WG_PANSTT12JUG1, 	0x07 );				// 0x0147
	// State 1 -> 3
	RegWriteA( WG_PANSTT13JUG0, 	0x00 );				// 0x0148
	RegWriteA( WG_PANSTT13JUG1, 	0x00 );				// 0x0149
	// State 2 -> 3
	RegWriteA( WG_PANSTT23JUG0, 	0x11 );				// 0x014A
	RegWriteA( WG_PANSTT23JUG1, 	0x00 );				// 0x014B
	// State 4 -> 3
	RegWriteA( WG_PANSTT43JUG0, 	0x00 );				// 0x014C
	RegWriteA( WG_PANSTT43JUG1, 	0x00 );				// 0x014D
	// State 3 -> 4
	RegWriteA( WG_PANSTT34JUG0, 	0x01 );				// 0x014E
	RegWriteA( WG_PANSTT34JUG1, 	0x00 );				// 0x014F
	// State 2 -> 4
	RegWriteA( WG_PANSTT24JUG0, 	0x00 );				// 0x0150
	RegWriteA( WG_PANSTT24JUG1, 	0x00 );				// 0x0151
	// State 4 -> 2
	RegWriteA( WG_PANSTT42JUG0, 	0x44 );				// 0x0152
	RegWriteA( WG_PANSTT42JUG1, 	0x04 );				// 0x0153

	// State Timer
	RegWriteA( WG_PANSTT1LEVTMR, 	0x00 );				// 0x015B
	RegWriteA( WG_PANSTT2LEVTMR, 	0x00 );				// 0x015C
	RegWriteA( WG_PANSTT3LEVTMR, 	0x00 );				// 0x015D
	RegWriteA( WG_PANSTT4LEVTMR, 	0x03 );				// 0x015E
	
	// Control filter
	RegWriteA( WG_PANTRSON0, 		0x11 );				// 0x0132	USE I12/iSTP/Gain-Filter
	
	// State Setting
	IniPtMovMod( OFF ) ;								// Pan/Tilt setting (Still)
	
	// Hold
	RegWriteA( WG_PANSTTSETILHLD,	0x00 );				// 0x015F
	
	
	// State2,4 Step Time Setting
	RegWriteA( WG_PANSTT2TMR0,	0x01 );					// 0x013C
	RegWriteA( WG_PANSTT2TMR1,	0x00 );					// 0x013D	
	RegWriteA( WG_PANSTT4TMR0,	0x02 );					// 0x013E
	RegWriteA( WG_PANSTT4TMR1,	0x07 );					// 0x013F	
	
	RegWriteA( WG_PANSTTXXXTH,	0x00 );					// 0x015A

#ifdef	NEW_PTST
	UnGyrLmt.SfFltVal	= 0.003F ;						// St4 Limiter�@1/S��
	RamWrite32A( npxlev8, UnGyrLmt.UlLngVal ) ;			// 0x109B
	RamWrite32A( npylev8, UnGyrLmt.UlLngVal ) ;			// 0x119B


	// Fast
//	UnGyrLmt.SfFltVal	= 0.0076F ;						// St1
	UnGyrLmt.SfFltVal	= 0.02F ;						// St1
	RamWrite32A( npxlev1, UnGyrLmt.UlLngVal ) ;			// 0x100F
	RamWrite32A( npylev1, UnGyrLmt.UlLngVal ) ;			// 0x110F
	RamWrite32A( npxlev1_i, UnGyrLmt.UlLngVal ) ;		// 0x10CF
	RamWrite32A( npylev1_i, UnGyrLmt.UlLngVal ) ;		// 0x11CF
	UnGyrLmt.SfFltVal	= 0.0005F ;						// St2 Limiter
	RamWrite32A( npxlev2, UnGyrLmt.UlLngVal ) ;			// 0x101F
	RamWrite32A( npylev2, UnGyrLmt.UlLngVal ) ;			// 0x111F
	RamWrite32A( npxlev2_i, UnGyrLmt.UlLngVal ) ;		// 0x10DF
	RamWrite32A( npylev2_i, UnGyrLmt.UlLngVal ) ;		// 0x11DF

	//Slow
	UnGyrLmt.SfFltVal	= 0.0005F ;  					// St3��
//	UnGyrLmt.SfFltVal	= 0.0050F ;  					// St3��
	RamWrite32A( npxlev3, UnGyrLmt.UlLngVal ) ;			// 0x102F
	RamWrite32A( npylev3, UnGyrLmt.UlLngVal ) ;			// 0x112F
//	UnGyrLmt.SfFltVal	= 0.001F ;  					// St3
	UnGyrLmt.SfFltVal	= 0.0025F ;  					// St3��
	RamWrite32A( npxlev3_i, UnGyrLmt.UlLngVal ) ;		// 0x10EF
	RamWrite32A( npylev3_i, UnGyrLmt.UlLngVal ) ;		// 0x11EF
//	UnGyrLmt.SfFltVal	= 0.0009F ;						// St4 Limiter
	UnGyrLmt.SfFltVal	= 0.0005f ;						// St4 Limiter
	RamWrite32A( npxlev4, UnGyrLmt.UlLngVal ) ;			// 0x103F
	RamWrite32A( npylev4, UnGyrLmt.UlLngVal ) ;			// 0x113F
//	UnGyrLmt.SfFltVal	= 0.0009F ;						// St4 Limiter(initial)
	UnGyrLmt.SfFltVal	= 0.0005F ;						// St4 Limiter
	RamWrite32A( npxlev4_i, UnGyrLmt.UlLngVal ) ;		// 0x10FF
	RamWrite32A( npylev4_i, UnGyrLmt.UlLngVal ) ;		// 0x11FF


//	RegWriteA( WG_VREFADD , 0x12 );						// 0x0119

	// Pan/Tilt NEW�@PanTilt�@Setteing
	RegWriteA( WG_NPANST12BTMR, 0x0F ) ;				// 0x0167 
	RegWriteA( WG_NPANST3RTMR, 0x0A ) ;					// 0x0166 
	RegWriteA( WG_NPANST12TMRX, 0x00 ) ;				// 0x0168 682u:682��
	RegWriteA( WG_NPANST12TMRY, 0x00 ) ;				// 0x0169 682u:682��
	RegWriteA( WG_NPANST3TMRX, 0x08 ) ;					// 0x016A 98ms
	RegWriteA( WG_NPANST3TMRY, 0x08 ) ;					// 0x016B 98ms
	RegWriteA( WG_NPANST4TMRX, 0x02 ) ;					// 0x016C 21ms
	RegWriteA( WG_NPANST4TMRY, 0x02 ) ;					// 0x016D 21ms

	RegWriteA( WG_NPANSTFRC, 0x00 ) ;					// 0x010B State posituon ON
	RegWriteA( WG_NPANFUN, 0x01 ) ;						// 0x016E Gain Cut,decrease
	RegWriteA( WG_NPANINITMR, 0x02 ) ;					// 0x016F 21ms '�]�����Ɍ������݂͓����l
	RegWriteA( WG_NPANSTOFF, 0xA0 ) ;					// 0x010E State5,7 OFF

	RegWriteA( WG_NPANTST0 ,0x08 );						// 0x0164 Option Setting

	UnGyrLmt.UlLngVal = 0x3951B717 ;					// St1 		decrease
	RamWrite32A( gxistp_1u, UnGyrLmt.UlLngVal ) ;		// 0x1085
	RamWrite32A( gyistp_1u, UnGyrLmt.UlLngVal ) ;		// 0x1185
	UnGyrLmt.UlLngVal = 0x3F7FFE80 ;
	RamWrite32A( gxistp_2d, UnGyrLmt.UlLngVal );		// 0x1087	ST1,ST3�@Ccof Val
	RamWrite32A( gyistp_2d, UnGyrLmt.UlLngVal );		// 0x1187	ST1,ST3�@Ccof Val
	UnGyrLmt.UlLngVal = 0x3F7FFE80 ;
	RamWrite32A( gyistp_2u, UnGyrLmt.UlLngVal );		// 0x1188	ST2,ST4�@Ccof Val
	RamWrite32A( gxistp_2u, UnGyrLmt.UlLngVal );		// 0x1088	ST2,ST4�@Ccof Val
	
#endif	//NEW_PTST

#ifdef GAIN_CONT
	RamWrite32A( gxlevlow, TRI_LEVEL );					// 0x10AE	Low Th
	RamWrite32A( gylevlow, TRI_LEVEL );					// 0x11AE	Low Th
	RamWrite32A( gxadjmin, XMINGAIN );					// 0x1094	Low gain
	RamWrite32A( gxadjmax, XMAXGAIN );					// 0x1095	Hgh gain
	RamWrite32A( gxadjdn, XSTEPDN );					// 0x1096	-step
	RamWrite32A( gxadjup, XSTEPUP );					// 0x1097	+step
	RamWrite32A( gyadjmin, YMINGAIN );					// 0x1194	Low gain
	RamWrite32A( gyadjmax, YMAXGAIN );					// 0x1195	Hgh gain
	RamWrite32A( gyadjdn, YSTEPDN );					// 0x1196	-step
	RamWrite32A( gyadjup, YSTEPUP );					// 0x1197	+step
	
	RegWriteA( WG_LEVADD, (unsigned char)MONADR );		// 0x0120	Input signal
	RegWriteA( WG_LEVTMR, 		TIMEBSE );				// 0x0123	Base Time
	RegWriteA( WG_LEVTMRLOW, 	TIMELOW );				// 0x0121	X Low Time
	RegWriteA( WG_LEVTMRHGH, 	TIMEHGH );				// 0x0122	X Hgh Time
	RegWriteA( WG_ADJGANADD, (unsigned char)GANADR );	// 0x0128	control address
	RegWriteA( WG_ADJGANGO, 		0x00 );				// 0x0108	manual off

	/* exe function */
	AutoGainControlSw( OFF ) ;							/* Auto Gain Control Mode OFF */
	//AutoGainControlSw( ON ) ;							/* Auto Gain Control Mode ON  */
#endif
	
}


//********************************************************************************
// Function Name 	: IniFil
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Initial Parameter Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniFil( void )
{
#ifdef	INIT_FAST
	unsigned char	UcAryId ;
	unsigned short	UsDatId, UsDatNum ;

	unsigned char	*pFilRegDat;
	unsigned char	*pFilReg;
	unsigned char	*pFilRamDat;
	unsigned char	*pFilRam;
	if( UcModule == MODULE_20M ){
		pFilRegDat	= (unsigned char *)CsFilRegDat_20M;
		pFilReg		= (unsigned char *)CsFilReg_20M;
		pFilRamDat	= (unsigned char *)CsFilRamDat_20M;
		pFilRam		= (unsigned char *)CsFilRam_20M;
	}
	else{
		pFilRegDat	= (unsigned char *)CsFilRegDat_13M;
		pFilReg		= (unsigned char *)CsFilReg_13M;
		pFilRamDat	= (unsigned char *)CsFilRamDat_13M;
		pFilRam		= (unsigned char *)CsFilRam_13M;
	}

	RegWriteA( WC_RAMACCXY, 	0x01 ) ;		// 0x018D	Simultaneously Setting On
	
	// Filter Registor Parameter Setting
	UcAryId	= 0 ;
	UsDatNum = 0 ;
	UsDatId	= 0 ;
	while( pFilReg[ UcAryId ] != 0xFF )
	{
		UsDatNum	= pFilReg[ UcAryId ];
//		SeqWriteA( ( unsigned char * )&pFilRegDat[ UsDatId ], UsDatNum ) ;
		UcAryId++ ;
		UsDatId	+= UsDatNum ;
	}

	// Filter X-axis Ram Parameter Setting	
	UcAryId	= 0 ;
	UsDatNum = 0 ;
	UsDatId	= 0 ;
	while( pFilRam[ UcAryId ] != 0xFF )
	{
		UsDatNum	= pFilRam[ UcAryId ];
//		SeqWriteA( ( unsigned char * )&pFilRamDat[ UsDatId ], UsDatNum ) ;
		UsDatId	+= UsDatNum ;
		UcAryId++ ;
	}
	
	RegWriteA( WC_RAMACCXY,		0x00 ) ;		// 0x018D	Simultaneously Setting Off
#else	//INIT_FAST
	unsigned short	UsAryId ;
	struct STFILREG		*pFilReg;
	struct STFILRAM		*pFilRam;

	if( UcModule == MODULE_20M ){
		pFilReg = (struct STFILREG *)CsFilReg_20M;
		pFilRam = (struct STFILRAM *)CsFilRam_20M;
	}
	else{
		pFilReg = (struct STFILREG *)CsFilReg_13M;
		pFilRam = (struct STFILRAM *)CsFilRam_13M;
	}

	// Filter Registor Parameter Setting
	UsAryId	= 0 ;
	while( pFilReg[ UsAryId ].UsRegAdd != 0xFFFF )
	{
		RegWriteA( pFilReg[ UsAryId ].UsRegAdd, pFilReg[ UsAryId ].UcRegDat ) ;
		UsAryId++ ;
	}
	
	#ifdef	XY_SIMU_SET
	RegWriteA( WC_RAMACCXY, 	0x01 ) ;		// 0x018D	Simultaneously Setting On
	#endif	//XY_SIMU_SET
	
	// Filter Ram Parameter Setting
	UsAryId	= 0 ;
	while( pFilRam[ UsAryId ].UsRamAdd != 0xFFFF )
	{
		RamWrite32A( pFilRam[ UsAryId ].UsRamAdd, pFilRam[ UsAryId ].UlRamDat ) ;
		UsAryId++ ;
	}
	
	#ifdef	XY_SIMU_SET
	RegWriteA( WC_RAMACCXY,		0x00 ) ;		// 0x018D	Simultaneously Setting Off
	#endif	//XY_SIMU_SET
#endif	//INIT_FAST
}



//********************************************************************************
// Function Name 	: IniAdj
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Adjust Value Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniAdj( void )
{
	unsigned char	BIAS_CUR_OIS;
	unsigned char	AMP_GAIN_X;
	unsigned char	AMP_GAIN_Y;
	
	if( UcModule == MODULE_20M ){
		BIAS_CUR_OIS = BIAS_CUR_OIS_20M;
		AMP_GAIN_X	 = AMP_GAIN_X_20M;
		AMP_GAIN_Y	 = AMP_GAIN_Y_20M;
    } else if (UcModule == MODULE_13M) {
		BIAS_CUR_OIS = BIAS_CUR_OIS_13M;
		AMP_GAIN_X	 = AMP_GAIN_X_13M;
		AMP_GAIN_Y	 = AMP_GAIN_Y_13M;
	} else {
		BIAS_CUR_OIS = BIAS_CUR_OIS_13MO;
		AMP_GAIN_X = AMP_GAIN_X_13MO;
		AMP_GAIN_Y = AMP_GAIN_Y_13MO;
	}
	RegWriteA( WC_RAMACCXY, 0x00 ) ;			// 0x018D	Filter copy off

	IniPtAve( ) ;								// Average setting
	
	/* OIS */
	RegWriteA( CMSDAC0, BIAS_CUR_OIS ) ;		// 0x0251	Hall Dac�d��
	RegWriteA( OPGSEL0, AMP_GAIN_X ) ;			// 0x0253	Hall amp Gain X
	RegWriteA( OPGSEL1, AMP_GAIN_Y ) ;			// 0x0254	Hall amp Gain Y
	/* AF */
	RegWriteA( CMSDAC1, BIAS_CUR_AF ) ;			// 0x0252	Hall Dac�d��
	RegWriteA( OPGSEL2, AMP_GAIN_AF ) ;			// 0x0255	Hall amp Gain AF

	RegWriteA( OSCSET, OSC_INI ) ;				// 0x0257	OSC ini
	
	/* adjusted value */
	RegWriteA( IZAH,	DGYRO_OFST_XH ) ;		// 0x02A0		Set Offset High byte
	RegWriteA( IZAL,	DGYRO_OFST_XL ) ;		// 0x02A1		Set Offset Low byte
	RegWriteA( IZBH,	DGYRO_OFST_YH ) ;		// 0x02A2		Set Offset High byte
	RegWriteA( IZBL,	DGYRO_OFST_YL ) ;		// 0x02A3		Set Offset Low byte
	
	/* Ram Access */
	RamAccFixMod( ON ) ;						// 16bit Fix mode
	
	/* OIS adjusted parameter */
	RamWriteA( DAXHLO,		DAHLXO_INI ) ;		// 0x1479
	RamWriteA( DAXHLB,		DAHLXB_INI ) ;		// 0x147A
	RamWriteA( DAYHLO,		DAHLYO_INI ) ;		// 0x14F9
	RamWriteA( DAYHLB,		DAHLYB_INI ) ;		// 0x14FA
	RamWriteA( OFF0Z,		HXOFF0Z_INI ) ;		// 0x1450
	RamWriteA( OFF1Z,		HYOFF1Z_INI ) ;		// 0x14D0
	RamWriteA( sxg,			SXGAIN_INI ) ;		// 0x10D3
	RamWriteA( syg,			SYGAIN_INI ) ;		// 0x11D3
//	UsCntXof = OPTCEN_X ;						/* Clear Optical center X value */
//	UsCntYof = OPTCEN_Y ;						/* Clear Optical center Y value */
//	RamWriteA( SXOFFZ1,		UsCntXof ) ;		// 0x1461
//	RamWriteA( SYOFFZ1,		UsCntYof ) ;		// 0x14E1

	/* AF adjusted parameter */
	RamWriteA( DAZHLO,		DAHLZO_INI ) ;		// 0x1529
	RamWriteA( DAZHLB,		DAHLZB_INI ) ;		// 0x152A

	/* Ram Access */
	RamAccFixMod( OFF ) ;						// 32bit Float mode
//#ifdef	AF_MID_MOUNT
	if( UcAfType == BI_DIR ){
		SetDOFSTDAF( AFDROF_INI );					// 0x0084
	}
//#endif	//AF_MID_MOUNT
	RamWrite32A( gxzoom, GXGAIN_INI ) ;			// 0x1020 Gyro X axis Gain adjusted value
	RamWrite32A( gyzoom, GYGAIN_INI ) ;			// 0x1120 Gyro Y axis Gain adjusted value

	RamWrite32A( sxq, SXQ_INI ) ;				// 0x10E5	X axis output direction initial value
	RamWrite32A( syq, SYQ_INI ) ;				// 0x11E5	Y axis output direction initial value
	
	if( GXHY_GYHX ){			/* GX -> HY , GY -> HX */
		RamWrite32A( sxgx, 0x00000000 ) ;		// 0x10B8
		RamWrite32A( sxgy, 0x3F800000 ) ;		// 0x10B9
		
		RamWrite32A( sygy, 0x00000000 ) ;		// 0x11B8
		RamWrite32A( sygx, 0x3F800000 ) ;		// 0x11B9
	}
	
	SetZsp(0) ;									// Zoom coefficient Initial Setting
	
	RegWriteA( PWMA 	, 0xC0 );				// 0x0010		PWM enable

	RegWriteA( STBB0 	, 0xDF );				// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	RegWriteA( WC_EQSW	, 0x02 ) ;				// 0x01E0
	RegWriteA( WC_MESLOOP1	, 0x02 ) ;			// 0x0193
	RegWriteA( WC_MESLOOP0	, 0x00 ) ;			// 0x0192
	RegWriteA( WC_AMJLOOP1	, 0x02 ) ;			// 0x01A3
	RegWriteA( WC_AMJLOOP0	, 0x00 ) ;			// 0x01A2
	
	
	SetPanTiltMode( OFF ) ;					/* Pan/Tilt OFF */
	
	SetGcf( 0 ) ;							/* DI initial value */
#ifdef	H1COEF_CHANGER
	SetH1cMod( ACTMODE ) ;					/* Lvl Change Active mode */
#endif	//H1COEF_CHANGER
	
	DrvSw( ON ) ;							/* 0x0001		Driver Mode setting */
	
	RegWriteA( WC_EQON, 0x01 ) ;				// 0x0101	Filter ON
}



//********************************************************************************
// Function Name 	: IniCmd
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Command Execute Process Initial
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniCmd( void )
{

	MemClr( ( unsigned char * )&StAdjPar, sizeof( stAdjPar ) ) ;	// Adjust Parameter Clear
	
}


//********************************************************************************
// Function Name 	: BsyWit
// Retun Value		: NON
// Argment Value	: Trigger Register Address, Trigger Register Data
// Explanation		: Busy Wait Function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	BsyWit( unsigned short	UsTrgAdr, unsigned char	UcTrgDat )
{
	unsigned char	UcFlgVal ;
	unsigned char	UcCnt ;

	RegWriteA( UsTrgAdr, UcTrgDat ) ;	// Trigger Register Setting

/* 20141113 Miyake */
	for (UcCnt = 0; UcCnt < 60; UcCnt++) {
		RegReadA( UsTrgAdr, &UcFlgVal ) ;
		UcFlgVal &= ( UcTrgDat & 0x0F ) ;
		if (UcFlgVal == 0) break;
		WitTim( WTLE );
	}
}


//********************************************************************************
// Function Name 	: MemClr
// Retun Value		: void
// Argment Value	: Clear Target Pointer, Clear Byte Number
// Explanation		: Memory Clear Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	MemClr( unsigned char	*NcTgtPtr, unsigned short	UsClrSiz )
{
	unsigned short	UsClrIdx ;

	for ( UsClrIdx = 0 ; UsClrIdx < UsClrSiz ; UsClrIdx++ )
	{
		*NcTgtPtr	= 0 ;
		NcTgtPtr++ ;
	}
}



//********************************************************************************
// Function Name 	: WitTim
// Retun Value		: NON
// Argment Value	: Wait Time(ms)
// Explanation		: Timer Wait Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	WitTim( unsigned short	UsWitTim )
{
	unsigned short w_time;
	w_time = UsWitTim;
	mdelay(w_time);
#if 0
	unsigned long	UlLopIdx, UlWitCyc ;

	UlWitCyc	= ( unsigned long )( ( float )UsWitTim / NOP_TIME / ( float )12 ) ;

	for( UlLopIdx = 0 ; UlLopIdx < UlWitCyc ; UlLopIdx++ )
	{
		;
	}
#endif
}

//********************************************************************************
// Function Name 	: GyOutSignal
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	GyOutSignal( void )
{

	RegWriteA( GRADR0,	GYROX_INI ) ;			// 0x0283	Set Gyro XOUT H~L
	RegWriteA( GRADR1,	GYROY_INI ) ;			// 0x0284	Set Gyro YOUT H~L
	
	/*Start OIS Reading*/
	RegWriteA( GRSEL	, 0x02 );				// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

//********************************************************************************
// Function Name 	: GyOutSignalCont
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Continuosl Function
// History			: First edition 						2013.06.06 Y.Shigeoka
//********************************************************************************
void	GyOutSignalCont( void )
{

	/*Start OIS Reading*/
	RegWriteA( GRSEL	, 0x04 );				// 0x0280	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

#ifdef STANDBY_MODE
//********************************************************************************
// Function Name 	: AccWit
// Retun Value		: NON
// Argment Value	: Trigger Register Data
// Explanation		: Acc Wait Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	AccWit( unsigned char UcTrgDat )
{
	unsigned char	UcFlgVal ;
	unsigned char	UcCnt ;
	
/* 20141113 Miyake */
	for (UcCnt = 0; UcCnt < 60; UcCnt++) {
		RegReadA(GRACC, &UcFlgVal);			// 0x0282
		UcFlgVal &= UcTrgDat;
		if (UcFlgVal == 0) break;
		WitTim(WTLE);
	}
}

//********************************************************************************
// Function Name 	: SelectGySleep
// Retun Value		: NON
// Argment Value	: mode	
// Explanation		: Select Gyro mode Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	SelectGySleep( unsigned char UcSelMode )
{
 #ifdef USE_INVENSENSE
	unsigned char	UcRamIni ;
	unsigned char	UcGrini ;

	if(UcSelMode == ON)
	{
		RegWriteA( WC_EQON, 0x00 ) ;		// 0x0101	Equalizer OFF
		RegWriteA( GRSEL,	0x01 ) ;		/* 0x0280	Set Command Mode			*/

		RegReadA( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
		RegWriteA( GRINI, ( UcGrini | SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ - | SLOWMODE | - | - ]
		
		RegWriteA( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GRACC,	0x01 ) ;		/* 0x0282	Set Read Trigger ON				*/
		AccWit( 0x01 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA( GRDAT0H, &UcRamIni ) ;	/* 0x0290 */
		
		UcRamIni |= 0x40 ;					/* Set Sleep bit */
  #ifdef GYROSTBY
		UcRamIni &= ~0x01 ;					/* Clear PLL bit(internal oscillator */
  #endif
		
		RegWriteA( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GSETDT,	UcRamIni ) ;	/* 0x028A	Set Write Data(Sleep ON)	*/
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/

  #ifdef GYROSTBY
		RegWriteA( GRADR0,	0x6C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GSETDT,	0x07 ) ;		/* 0x028A	Set Write Data(STBY ON)	*/
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
  #endif
	}
	else
	{
  #ifdef GYROSTBY
		RegWriteA( GRADR0,	0x6C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GSETDT,	0x00 ) ;		/* 0x028A	Set Write Data(STBY OFF)	*/
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
  #endif
		RegWriteA( GRADR0,	0x6B ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GRACC,	0x01 ) ;		/* 0x0282	Set Read Trigger ON				*/
		AccWit( 0x01 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA( GRDAT0H, &UcRamIni ) ;	/* 0x0290 */
		
		UcRamIni &= ~0x40 ;					/* Clear Sleep bit */
  #ifdef GYROSTBY
		UcRamIni |=  0x01 ;					/* Set PLL bit */
  #endif
		
		RegWriteA( GSETDT,	UcRamIni ) ;	// 0x028A	Set Write Data(Sleep OFF)
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		
		RegReadA( GRINI	, &UcGrini );					// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		RegWriteA( GRINI, ( UcGrini & ~SLOWMODE) );		// 0x0281	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		
		GyOutSignal( ) ;					/* Select Gyro output signal 			*/
		
		WitTim( 50 ) ;						// 50ms wait
		
		RegWriteA( WC_EQON, 0x01 ) ;		// 0x0101	GYRO Equalizer ON

		ClrGyr( 0x007F , CLR_FRAM1 );		// Gyro Delay RAM Clear
	}
 #else									/* Panasonic */
	
//	unsigned char	UcRamIni ;


	if(UcSelMode == ON)
	{
		RegWriteA( WC_EQON, 0x00 ) ;		// 0x0101	GYRO Equalizer OFF
		RegWriteA( GRSEL,	0x01 ) ;		/* 0x0280	Set Command Mode			*/
		RegWriteA( GRADR0,	0x4C ) ;		/* 0x0283	Set Write Command			*/
		RegWriteA( GSETDT,	0x02 ) ;		/* 0x028A	Set Write Data(Sleep ON)	*/
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
	}
	else
	{
		RegWriteA( GRADR0,	0x4C ) ;		// 0x0283	Set Write Command
		RegWriteA( GSETDT,	0x00 ) ;		// 0x028A	Set Write Data(Sleep OFF)
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0282	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		GyOutSignal( ) ;					/* Select Gyro output signal 			*/
		
		WitTim( 50 ) ;						// 50ms wait
		
		RegWriteA( WC_EQON, 0x01 ) ;		// 0x0101	GYRO Equalizer ON
		ClrGyr( 0x007F , CLR_FRAM1 );		// Gyro Delay RAM Clear
	}
 #endif
}
#endif

#ifdef	GAIN_CONT
//********************************************************************************
// Function Name 	: AutoGainControlSw
// Retun Value		: NON
// Argment Value	: 0 :OFF  1:ON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.11.30 Y.Shigeoka
//********************************************************************************
void	AutoGainControlSw( unsigned char UcModeSw )
{

	if( UcModeSw == OFF )
	{
		RegWriteA( WG_ADJGANGXATO, 	0xA0 );					// 0x0129	X exe off
		RegWriteA( WG_ADJGANGYATO, 	0xA0 );					// 0x012A	Y exe off
		RamWrite32A( GANADR			 , XMAXGAIN ) ;			// Gain Through
		RamWrite32A( GANADR | 0x0100 , YMAXGAIN ) ;			// Gain Through
	}
	else
	{
		RegWriteA( WG_ADJGANGXATO, 	0xA3 );					// 0x0129	X exe on
		RegWriteA( WG_ADJGANGYATO, 	0xA3 );					// 0x012A	Y exe on
	}

}
#endif


//********************************************************************************
// Function Name 	: ClrGyr
// Retun Value		: NON
// Argment Value	: UsClrFil - Select filter to clear.  If 0x0000, clears entire filter
//					  UcClrMod - 0x01: FRAM0 Clear, 0x02: FRAM1, 0x03: All RAM Clear
// Explanation		: Gyro RAM clear function
// History			: First edition 						2013.01.09 Y.Shigeoka
//********************************************************************************
void	ClrGyr( unsigned short UsClrFil , unsigned char UcClrMod )
{
	unsigned char	UcRamClr;
	unsigned char	UcCnt ;

	/*Select Filter to clear*/
	RegWriteA( WC_RAMDLYMOD1,	(unsigned char)(UsClrFil >> 8) ) ;		// 0x018F		FRAM Initialize Hbyte
	RegWriteA( WC_RAMDLYMOD0,	(unsigned char)UsClrFil ) ;				// 0x018E		FRAM Initialize Lbyte

	/*Enable Clear*/
	RegWriteA( WC_RAMINITON	, UcClrMod ) ;	// 0x0102	[ - | - | - | - ][ - | - | �x��Clr | �W��Clr ]
	
	/*Check RAM Clear complete*/

/* 20141113 Miyake*/
	for (UcCnt = 0; UcCnt < 60; UcCnt++) {
		RegReadA(WC_RAMINITON, &UcRamClr);
		UcRamClr &= UcClrMod;
		if (UcRamClr == 0) break;
		WitTim(WTLE);
	}
}


//********************************************************************************
// Function Name 	: DrvSw
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: Driver Mode setting function
// History			: First edition 						2012.04.25 Y.Shigeoka
//********************************************************************************
void	DrvSw( unsigned char UcDrvSw )
{
	if( UcDrvSw == ON )
	{
		if( UcPwmMod == PWMMOD_CVL ) {
			RegWriteA( DRVFC	, 0xF0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE2,LCEN
		}
		else {
#ifdef	PWM_BREAK
			RegWriteA( DRVFC	, 0x00 );			// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
#else
			RegWriteA( DRVFC	, 0xC0 );			// 0x0001	Drv.MODE=1,Drv.BLK=1,MODE1
#endif
		}
	}
	else
	{
		if( UcPwmMod == PWMMOD_CVL ) {
			RegWriteA( DRVFC	, 0x30 );				// 0x0001	Drvier Block Ena=0
		}
		else {
#ifdef	PWM_BREAK
			RegWriteA( DRVFC	, 0x00 );				// 0x0001	Drv.MODE=0,Drv.BLK=0,MODE0B
#else
			RegWriteA( DRVFC	, 0x00 );				// 0x0001	Drvier Block Ena=0
#endif
		}
	}
}

//********************************************************************************
// Function Name 	: AfDrvSw
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: AF Driver Mode setting function
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	AfDrvSw( unsigned char UcDrvSw )
{
	if( UcDrvSw == ON )
	{
//	#ifdef	AF_MID_MOUNT
		if( UcAfType == BI_DIR ){
			RegWriteA( DRVFCAF,		0x10 ) ;		// DRVFCAF(0x0081)
		}
//	#else	//AF_MID_MOUNT
		else{
		  #ifdef	AF_PWMMODE
			RegWriteA( DRVFCAF	, 0x00 );			// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
		  #else
			RegWriteA( DRVFCAF	, 0x20 );			// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
		  #endif
		}
//	#endif	//AF_MID_MOUNT
		RegWriteA( CCAAF,   0x80 ) ;				// 0x00A0	[7]=0:OFF 1:ON
	}
	else
	{
		RegWriteA( CCAAF,   0x00 ) ;				// 0x00A0	[7]=0:OFF 1:ON
	}
}

//********************************************************************************
// Function Name 	: RamAccFixMod
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: Ram Access Fix Mode setting function
// History			: First edition 						2013.05.21 Y.Shigeoka
//********************************************************************************
void	RamAccFixMod( unsigned char UcAccMod )
{
	switch ( UcAccMod ) {
		case OFF :
			RegWriteA( WC_RAMACCMOD,	0x00 ) ;		// 0x018C		GRAM Access Float32bit
			break ;
		case ON :
			RegWriteA( WC_RAMACCMOD,	0x31 ) ;		// 0x018C		GRAM Access Fix32bit
			break ;
	}
}
	

//********************************************************************************
// Function Name 	: IniAf
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Open AF Initial Setting
// History			: First edition 						2013.09.12 Y.Shigeoka
//********************************************************************************
void	IniAf( void )
{
	unsigned char	UcStbb0 ;
	
	unsigned short	RWEXD1_L_AF;
	unsigned short	RWEXD2_L_AF;
	unsigned short	RWEXD3_L_AF;
	unsigned char	FSTCTIME_AF;
	unsigned char	FSTMODE_AF;
	
	if( UcModule == MODULE_20M ){
		RWEXD1_L_AF	= RWEXD1_L_AF_20M;
		RWEXD2_L_AF	= RWEXD2_L_AF_20M;
		RWEXD3_L_AF	= RWEXD3_L_AF_20M;
		FSTCTIME_AF	= FSTCTIME_AF_20M;
		FSTMODE_AF	= FSTMODE_AF_20M;
	}
	else{
		RWEXD1_L_AF	= RWEXD1_L_AF_13M;
		RWEXD2_L_AF	= RWEXD2_L_AF_13M;
		RWEXD3_L_AF	= RWEXD3_L_AF_13M;
		FSTCTIME_AF	= FSTCTIME_AF_13M;
		FSTMODE_AF	= FSTMODE_AF_13M;
	}
	
	AfDrvSw( OFF ) ;								/* AF Drvier Block Ena=0 */

//#ifdef	AF_MID_MOUNT
	if( UcAfType == BI_DIR ){
		SetTregAf( 0x0400 ) ;						// 0 code = Half Code
		RegWriteA( DRVFCAF,		0x10 ) ;			// DRVFCAF(0x0081)
		RegWriteA( DRVFC3AF,	0x40 ) ;			// DRVFC3AF(0x0083)
		RegWriteA( DRVFC4AF,	0x80 ) ;			// DRVFC4AF(0x0084)
		RegWriteA( AFFC,		0x90 ) ;			// AFFC(0x0088)
		//RegWriteA( CCAAF,		0x80 ) ;			// CCAAF(0x00A0)
	}
//#else	//AF_MID_MOUNT
	else{
	  #ifdef	AF_PWMMODE
		RegWriteA( DRVFCAF	, 0x00 );					// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-0
	  #else		//AF_PWMMODE
		RegWriteA( DRVFCAF	, 0x20 );					// 0x0081	Drv.MODEAF=0,Drv.ENAAF=0,MODE-2
	  #endif	//AF_PWMMODE
		RegWriteA( DRVFC3AF	, 0x00 );					// 0x0083	DGAINDAF	Gain 0
		RegWriteA( DRVFC4AF	, 0x80 );					// 0x0084	DOFSTDAF
		RegWriteA( PWMAAF,    0x00 ) ;					// 0x0090	AF PWM standby
		RegWriteA( AFFC,   0x80 ) ;						// 0x0088	OpenAF/-/-
	}
//#endif	//AF_MID_MOUNT

#ifdef	AF_PWMMODE
	RegWriteA( DRVFC2AF,    0x82 ) ;				// 0x0082	AF slope3
	RegWriteA( DRVCH3SEL,   0x02 ) ;				// 0x0085	AF only IN1 control
	RegWriteA( PWMFCAF,     0x89 ) ;				// 0x0091	AF GND , Carrier , MODE1 
	RegWriteA( PWMPERIODAF, 0xA0 ) ;				// 0x0099	AF none-synchronism
#else
	RegWriteA( DRVFC2AF,    0x00 ) ;				// 0x0082	AF slope0
	RegWriteA( DRVCH3SEL,   0x00 ) ;				// 0x0085	AF H bridge control
	RegWriteA( PWMFCAF,     0x01 ) ;				// 0x0091	AF VREF , Carrier , MODE1
	RegWriteA( PWMPERIODAF, 0x20 ) ;				// 0x0099	AF none-synchronism
#endif
	
//#ifdef	AF_MID_MOUNT
	if( UcAfType == BI_DIR ){
		RegWriteA( CCFCAF,	 0x08 ) ;					// CCFCAF(0x00A1)
	}
//#else	//AF_MID_MOUNT
	else{
		RegWriteA( CCFCAF,   0x40 ) ;					// 0x00A1	GND/-
	}
//#endif	//AF_MID_MOUNT
	
	RegReadA( STBB0 	, &UcStbb0 );		// 0x0250 	[ STBAFDRV | STBOISDRV | STBOPAAF | STBOPAY ][ STBOPAX | STBDACI | STBDACV | STBADC ]
	UcStbb0 &= 0x7F ;
	RegWriteA( STBB0, UcStbb0 ) ;			// 0x0250	OIS standby
	RegWriteA( STBB1, 0x00 ) ;				// 0x0264	All standby
	
	/* AF Initial setting */
	RegWriteA( FSTMODE,		FSTMODE_AF ) ;		// 0x0302
	RamWriteA( RWEXD1_L,	RWEXD1_L_AF ) ;		// 0x0396 - 0x0397 (Register continuos write)
	RamWriteA( RWEXD2_L,	RWEXD2_L_AF ) ;		// 0x0398 - 0x0399 (Register continuos write)
	RamWriteA( RWEXD3_L,	RWEXD3_L_AF ) ;		// 0x039A - 0x039B (Register continuos write)
	RegWriteA( FSTCTIME,	FSTCTIME_AF ) ;		// 0x0303 	
	RegWriteA( TCODEH,		0x04 ) ;			// Fast Mode
	
#ifdef	AF_PWMMODE
	RegWriteA( PWMAAF,    0x80 ) ;			// 0x0090	AF PWM enable
#endif

	UcStbb0 |= 0x80 ;
	RegWriteA( STBB0, UcStbb0 ) ;			// 0x0250	
	RegWriteA( STBB1	, 0x05 ) ;			// 0x0264	[ - | - | - | - ][ - | STBAFOP1 | - | STBAFDAC ]

	AfDrvSw( ON ) ;								/* AF Drvier Block Ena=1 */
}

//********************************************************************************
// Function Name 	: SetTregAf
// Retun Value		: 
// Argment Value	: Min:000h Max:7FFh (11bit)
// Argment Value	: Min:000h Max:3FFh (10bit)
// Explanation		: 
// History			: First edition 						2014.06.19 T.Tokoro
//********************************************************************************
void	SetTregAf( unsigned short UsTregAf )
{
//#ifdef	AF_MID_MOUNT
	if( UcAfType == BI_DIR ){
		RamWriteA( TREG_H,	(UsTregAf << 5) ) ;		// TREG_H(0x0380) - TREG_L(0x0381)
													// TREG[15:5] 11bit
													// AF_D[10:0]=TREG[15:5](0380h/0381h)�B
													// Min:000h Max:7FFh
	}
//#else	//AF_MID_MOUNT
	else{
		RamWriteA( TREG_H,	(UsTregAf << 6) ) ;		// TREG_H(0x0380) - TREG_L(0x0381)
													// TREG[15:4] 10bit
													// AF_D[9:0]=TREG[15:4](0380h/0381h)�B
													// Min:000h Max:3FFh
	}
//#endif	//AF_MID_MOUNT
}

//********************************************************************************
// Function Name 	: IniPtAve
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan/Tilt Average parameter setting function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
void	IniPtAve( void )
{
	RegWriteA( WG_PANSTT1DWNSMP0, 0x00 );		// 0x0134
	RegWriteA( WG_PANSTT1DWNSMP1, 0x00 );		// 0x0135
	RegWriteA( WG_PANSTT2DWNSMP0, 0x90 );		// 0x0136 400
	RegWriteA( WG_PANSTT2DWNSMP1, 0x01 );		// 0x0137
	RegWriteA( WG_PANSTT3DWNSMP0, 0x64 );		// 0x0138 100
	RegWriteA( WG_PANSTT3DWNSMP1, 0x00 );		// 0x0139
	RegWriteA( WG_PANSTT4DWNSMP0, 0x00 );		// 0x013A
	RegWriteA( WG_PANSTT4DWNSMP1, 0x00 );		// 0x013B

	RamWrite32A( st1mean, 0x3f800000 );		// 0x1235
	RamWrite32A( st2mean, 0x3B23D700 );		// 0x1236	1/400
	RamWrite32A( st3mean, 0x3C23D700 );		// 0x1237	1/100
	RamWrite32A( st4mean, 0x3f800000 );		// 0x1238
			
}
	
//********************************************************************************
// Function Name 	: IniPtMovMod
// Retun Value		: NON
// Argment Value	: OFF:Still  ON:Movie
// Explanation		: Pan/Tilt parameter setting by mode function
// History			: First edition 						2013.09.26 Y.Shigeoka
//********************************************************************************
void	IniPtMovMod( unsigned char UcPtMod )
{
	switch ( UcPtMod ) {
		case OFF :
			RegWriteA( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA( WG_PANSTTSETGAIN, 	0x54 );		// 0x0155
			RegWriteA( WG_PANSTTSETISTP, 	0x14 );		// 0x0156
			RegWriteA( WG_PANSTTSETIFTR,	0x94 );		// 0x0157
			RegWriteA( WG_PANSTTSETLFTR,	0x00 );		// 0x0158

			break ;
		case ON :
			RegWriteA( WG_PANSTTSETGYRO, 	0x00 );		// 0x0154
			RegWriteA( WG_PANSTTSETGAIN, 	0x00 );		// 0x0155
			RegWriteA( WG_PANSTTSETISTP, 	0x14 );		// 0x0156
			RegWriteA( WG_PANSTTSETIFTR,	0x94 );		// 0x0157
			RegWriteA( WG_PANSTTSETLFTR,	0x00 );		// 0x0158
			break ;
	}
}

//********************************************************************************
// Function Name 	: ChkCvr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Check Cver function
// History			: First edition 						2013.10.03 Y.Shigeoka
//********************************************************************************
void	ChkCvr( void )
{
	RegReadA( CVER ,	&UcCvrCod );		// 0x027E
	RegWriteA( VRREG ,	(unsigned char)FW_VER );		// 0x2D0	Version
}

//********************************************************************************
// Function Name 	: RemOff
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Remove Offset
// History			: First edition 						2015.01.14 T.Tokoro
//********************************************************************************
void	RemOff( unsigned char UcMod )
{
	switch ( UcMod ) {
		case OFF :
			RegWriteA( WG_PANSTT6, 0x11 ) ;			// 0x010A	Pan/Tilt State0 setting
			RegWriteA( WC_RAMACCXY, 	0x01 ) ;	// 0x018D	Simultaneously Setting On
			RamWrite32A( gxia_1, 0x39D2BD40 ) ;		// 0x1043	3Hz
			RamWrite32A( gxib_1, 0x39D2BD40 ) ;		// 0x1044
			RamWrite32A( gxic_1, 0x3F7FCB40 ) ;		// 0x1045
			RegWriteA( WC_RAMACCXY,		0x00 ) ;	// 0x018D	Simultaneously Setting Off
			RegWriteA( WG_PANSTT6, 0x00 ) ;			// 0x010A	Pan/Tilt Normal operation
			break ;
		case ON :
			ClrGyr( 0x007F, CLR_FRAM1 ) ;			// 			Clear Gyro Filter
			RegWriteA( WC_RAMACCXY, 	0x01 ) ;	// 0x018D	Simultaneously Setting On
			RamWrite32A( gxia_1, 0x3aaf73c0 ) ;		// 0x1043	10Hz
			RamWrite32A( gxib_1, 0x3aaf73c0 ) ;		// 0x1044
			RamWrite32A( gxic_1, 0x3F7F5080 ) ;		// 0x1045
			RegWriteA( WC_RAMACCXY,		0x00 ) ;	// 0x018D	Simultaneously Setting Off
			SetPanTiltMode( ON ) ; 					// Pan/Tilt control enable
			RegWriteA( WG_PANSTT6, 0x44 ) ;			// 0x010A	Pan/Tilt State3 setting
			break ;
	}
}


