#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <media/msm_cam_sensor.h>
#include "OIS_head.h"
#include "ois_interface.h"
#include "OisDef.h"
#include "Ois.h"
#include "OIS_defi.h"
#include "bu24218/BU24_defi.h"
//#include "msm_sensor.h
//#define OIS_DBG
#ifdef OIS_DBG
#define OISDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define OISDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#define X5
#define LC898122_CHECK_RIGISTER 0x027F
#define BU63165_CHECK_RIGISTER 0x00
#define LC898122_CHECK_DATA 0xAC
#define BU63165_CHECK_DATA 0x0735
#define INIT_FAILED -1
#define INIT_SUCCESS 1
static int ois_flag;

static struct msm_camera_i2c_client *g_i2c_clinet;

static int ois_dw_status = 0;

int Get_Ois_DW_Status(void)
{
	return ois_dw_status;
}

EXPORT_SYMBOL(Get_Ois_DW_Status);

static int get_ois_flag(void)
{
	return ois_flag;
}

static void set_ois_flag(void)
{
	ois_flag = msm_get_ois_flag();
}

/*  write interface for AF  OIS*/
void RegWriteA(unsigned short RegAddr, unsigned char RegData)
{
	int32_t  rc = 0;
	uint16_t w_data;
	uint32_t w_addr;
	w_addr = (RegAddr & 0xFFFF);
	w_data = (RegData & 0x00FF);
	rc = msm_camera_cci_i2c_write(g_i2c_clinet,
			w_addr, w_data, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_err("%s: %x: RegWriteA    failed\n", __func__, RegAddr);
}

void RegReadA(unsigned short RegAddr, unsigned char *RegData)
{
	int32_t rc = 0;
	uint16_t r_data;
	uint32_t w_addr;
	w_addr = (RegAddr & 0xFFFF);
	rc = msm_camera_cci_i2c_read(g_i2c_clinet,
			w_addr, &r_data, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_err("%s: %x: RegReadA read  failed\n", __func__, RegAddr);
	*RegData = (r_data & 0x00FF);
}

void RamWriteA(unsigned short RameAddr, unsigned short RamData)
{
	int32_t rc = 0;
	uint16_t w_data;
	uint32_t w_addr;
	w_addr = (RameAddr & 0xFFFF);
	w_data = (RamData & 0xFFFF);
	rc = msm_camera_cci_i2c_write(g_i2c_clinet,
			w_addr, w_data,
			MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0)
		pr_err("%s: %x: RamWriteA  write failed\n", __func__, RameAddr);
}

void RamReadA(unsigned short RameAddr, unsigned short *RamData)
{
	int32_t rc = 0;
	uint16_t r_data;
	uint32_t w_addr;
	w_addr = (RameAddr & 0xFFFF);
	rc = msm_camera_cci_i2c_read(g_i2c_clinet,
			w_addr, &r_data,
			MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0)
		pr_err("%s: %x: RamReadA read  failed\n", __func__, RameAddr);
	*RamData = (r_data & 0xFFFF);
}

void RamWrite32A(unsigned short RameAddr, unsigned long RamData)
{
	int32_t  rc = 0;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	uint32_t w_data;
	uint32_t w_addr;
	w_addr = (RameAddr & 0xFFFF);
	w_data = (RamData & 0xFFFFFFFF);
	reg_setting.reg_addr = w_addr;
	reg_setting.reg_data[0] = (uint8_t)((w_data & 0xFF000000) >> 24);
	reg_setting.reg_data[1] = (uint8_t)((w_data & 0x00FF0000) >> 16);
	reg_setting.reg_data[2] = (uint8_t)((w_data & 0x0000FF00) >> 8);
	reg_setting.reg_data[3] = (uint8_t)(w_data & 0x000000FF);
	reg_setting.reg_data_size = 4;
	rc = msm_camera_cci_i2c_write_seq(g_i2c_clinet,
		reg_setting.reg_addr,
		reg_setting.reg_data,
		reg_setting.reg_data_size);

	if (rc < 0)
		pr_err("%s: %x: RamWrite32A write failed\n",
			__func__, RameAddr);
}

void RamRead32A(unsigned short RameAddr, unsigned long *RamData)
{

	int32_t rc = 0;
	uint32_t t_data;
	uint32_t w_addr;
	uint8_t r_data[5];
	w_addr = (RameAddr & 0xFFFF);
	rc = msm_camera_cci_i2c_read_seq(g_i2c_clinet,
		w_addr, r_data, 4);
	if (rc < 0)
		pr_err(" %s: %x: RamRead32A RameAddr read failed\n",
			__func__, RameAddr);
	t_data = (r_data[0]<<24)|(r_data[1]<<16)|(r_data[2]<<8)|r_data[3];
	*RamData = (t_data & 0xFFFFFFFF);
}

int Ois_Write_OTP_sharp_imx230(void)
{
	int rc = 0;
	uint8_t ois_otp_buf[MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE];
	uint8_t dac_data = 0;
	OISDBG("[%s:%d] Enter \n", __func__, __LINE__);
	rc = msm_get_otp_data(ois_otp_buf,
			MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE, OTP_REAR_CAMERA_OIS);
	if (rc < 0) {
		pr_err("failed: get rear camera ois : get otp data err %d", rc);
		return rc;
	}

	RamAccFixMod(ON);
	/* OIS adjusted parameter */
	RamWriteA(DAXHLO, (ois_otp_buf[2]<<8|ois_otp_buf[1]));
	RamWriteA(DAYHLO, (ois_otp_buf[4]<<8|ois_otp_buf[3]));
	RamWriteA(DAXHLB, (ois_otp_buf[6]<<8|ois_otp_buf[5]));
	RamWriteA(DAYHLB, (ois_otp_buf[8]<<8|ois_otp_buf[7]));
	RamWriteA(OFF0Z, (ois_otp_buf[10]<<8|ois_otp_buf[9]));
	RamWriteA(OFF1Z, (ois_otp_buf[12]<<8|ois_otp_buf[11]));
	RamWriteA(sxg, (ois_otp_buf[14]<<8|ois_otp_buf[13]));
	RamWriteA(syg, (ois_otp_buf[16]<<8|ois_otp_buf[15]));
	RamAccFixMod(OFF);

	/* AF adjusted parameter */
	RegWriteA(IZAL, ois_otp_buf[21]);
	RegWriteA(IZAH, ois_otp_buf[22]);
	RegWriteA(IZBL, ois_otp_buf[23]);
	RegWriteA(IZBH, ois_otp_buf[24]);

	/* Ram Access */
	RamWrite32A(gxzoom,
		(ois_otp_buf[29]<<24 | ois_otp_buf[28]<<16
			|ois_otp_buf[27]<<8 | ois_otp_buf[26]));
	RamWrite32A(gyzoom,
		(ois_otp_buf[33]<<24
			| ois_otp_buf[32]<<16 | ois_otp_buf[31]<<8
			| ois_otp_buf[30]));
	RegWriteA(OSCSET, ois_otp_buf[25]);

	dac_data = ois_otp_buf[35];
	SetDOFSTDAF(dac_data);

	OISDBG("[%s:%d] Exit \n", __func__, __LINE__);
	return rc;
}

static  _FACT_ADJ fadj =
{
	.gl_CURDAT = 0x01F7,
	.gl_HALOFS_X = 0x0249,
	.gl_HALOFS_Y = 0x022D,
	.gl_HX_OFS = 0x0060,
	.gl_HY_OFS = 0x0055,
	.gl_PSTXOF = 0x007E,
	.gl_PSTYOF = 0x007E,
	.gl_GX_OFS = 0x0161,
	.gl_GY_OFS = 0x0053,
	.gl_KgxHG = 0x21A9,
	.gl_KgyHG = 0x20A5,
	.gl_KGXG = 0x2364,
	.gl_KGYG = 0x20F9,
	.gl_SFTHAL_X = 0x0249,
	.gl_SFTHAL_Y = 0x022D,
	.gl_TMP_X_ = 0x0000,
	.gl_TMP_Y_ = 0x0000,
	.gl_KgxH0 = 0x007D,
	.gl_KgyH0 = 0x004E,
};
static  _FACT_ADJ_AF fadj_af;
uint8_t otp_date_buf[3];
#ifndef TEST_ROHM_BU63165_AF
static int Ois_Write_OTP_sunny_imx230(void)
{
	int rc = 0;
        uint8_t otp_ois_data[41];
        uint8_t otp_af_data[10];
	OISDBG("[%s:%d] Enter \n", __func__, __LINE__);
        memset(otp_ois_data,0,41);
        memset(otp_af_data,0,10);
        memset(otp_date_buf,0,3);
	msm_get_otp_data(otp_date_buf,
			MSM_OTP_REAR_CAMERA_DATE_BUFF_SIZE,
			OTP_REAR_CAMERA_DATE);
        pr_err("date year:%d,month:%d,day:%d",otp_date_buf[0],otp_date_buf[1],otp_date_buf[2]);
	rc = msm_get_otp_data((uint8_t*)&otp_ois_data,
			41, OTP_REAR_CAMERA_OIS);
	if (rc < 0) {
		pr_err("failed: get rear camera ois : get otp data err %d", rc);
		return rc;
	}
        fadj.gl_CURDAT = (otp_ois_data[4] << 8) | otp_ois_data[3];
        fadj.gl_HALOFS_X = (otp_ois_data[6] << 8) | otp_ois_data[5];
        fadj.gl_HALOFS_Y = (otp_ois_data[8] << 8) | otp_ois_data[7];
        fadj.gl_HX_OFS = (otp_ois_data[10] << 8) | otp_ois_data[9];
        fadj.gl_HY_OFS = (otp_ois_data[12] << 8) | otp_ois_data[11];
        fadj.gl_PSTXOF = (otp_ois_data[14] << 8) | otp_ois_data[13];
        fadj.gl_PSTYOF = (otp_ois_data[16] << 8) | otp_ois_data[15];
        fadj.gl_GX_OFS = (otp_ois_data[18] << 8) | otp_ois_data[17];
        fadj.gl_GY_OFS = (otp_ois_data[20] << 8) | otp_ois_data[19];
        fadj.gl_KgxHG = (otp_ois_data[22] << 8) | otp_ois_data[21];
        fadj.gl_KgyHG = (otp_ois_data[24] << 8) | otp_ois_data[23];
        fadj.gl_KGXG = (otp_ois_data[26] << 8) | otp_ois_data[25];
        fadj.gl_KGYG = (otp_ois_data[28] << 8) | otp_ois_data[27];
        fadj.gl_SFTHAL_X = (otp_ois_data[30] << 8) | otp_ois_data[29];
        fadj.gl_SFTHAL_Y = (otp_ois_data[32] << 8) | otp_ois_data[31];
        fadj.gl_TMP_X_ = (otp_ois_data[34] << 8) | otp_ois_data[33];
        fadj.gl_TMP_Y_ = (otp_ois_data[36] << 8) | otp_ois_data[35];
        fadj.gl_KgxH0 = (otp_ois_data[38] << 8) | otp_ois_data[37];
        fadj.gl_KgyH0 = (otp_ois_data[40] << 8) | otp_ois_data[39];
	SET_FADJ_PARAM((const _FACT_ADJ *)&fadj);
	rc = msm_get_otp_data((uint8_t*)&otp_af_data,
			MSM_OTP_REAR_CAMERA_CLAF_BUFF_SIZE, OTP_REAR_CAMERA_CLAF);
	if (rc < 0) {
		pr_err("failed: get rear camera ois : get otp data err %d", rc);
		return rc;
	}
	fadj_af.gl_CURDAZ = (otp_af_data[0] << 8) | otp_af_data[1];
	fadj_af.gl_HALOFS_Z = (otp_af_data[2] << 8) | otp_af_data[3];
	fadj_af.gl_PSTZOF = (otp_af_data[4] << 8) | otp_af_data[5];
	fadj_af.gl_P_M_HZOFS = (otp_af_data[6] << 8) | otp_af_data[7];
	fadj_af.gl_P_M_KzHG = (otp_af_data[8] << 8) | otp_af_data[9];
     //   pr_err("fadj_af.gl_CURDAZ = 0x%x,fadj_af.gl_HALOFS_Z=0x%x,fadj_af.gl_PSTZOF=0x%x,fadj_af.gl_P_M_HZOFS=0x%x,fadj_af.gl_P_M_KzHG=0x%x",fadj_af.gl_CURDAZ,fadj_af.gl_HALOFS_Z,fadj_af.gl_PSTZOF,fadj_af.gl_P_M_HZOFS,fadj_af.gl_P_M_KzHG);
        SET_FADJ_PARAM_CLAF((const _FACT_ADJ_AF *)&fadj_af);

	OISDBG("[%s:%d] Exit \n", __func__, __LINE__);
	return rc;
}
#endif



#ifndef TEST_ROHM_BU63165_AF
static int Ois_Write_OTP_lg_imx230(void)
{
	int rc = 0;
        uint8_t otp_ois_data[41];
        uint8_t otp_af_data[10];

	OISDBG("[%s:%d] Enter \n", __func__, __LINE__);
        memset(otp_ois_data,0,41);
        memset(otp_af_data,0,10);
        memset(otp_date_buf,0,3);

	msm_get_otp_data(otp_date_buf,
			MSM_OTP_REAR_CAMERA_DATE_BUFF_SIZE,
			OTP_REAR_CAMERA_DATE);
        pr_err("date year:%d,month:%d,day:%d",otp_date_buf[0],otp_date_buf[1],otp_date_buf[2]);
	if ((otp_date_buf[1] == 2)&&(otp_date_buf[2] == 25))
		otp_date_buf[1] = 3;
	rc = msm_get_otp_data((uint8_t*)&otp_af_data,
			MSM_OTP_REAR_CAMERA_CLAF_BUFF_SIZE, OTP_REAR_CAMERA_CLAF);

	if (rc < 0) {
		pr_err("failed: get rear camera ois : get otp data err %d", rc);
		return rc;
	}

	if ((otp_date_buf[0] == 16) && ((otp_date_buf[1] < 2) || ((otp_date_buf[1] == 2) && (otp_date_buf[2] < 3))))
	{
		fadj_af.gl_CURDAZ = (otp_af_data[1] << 8) | otp_af_data[0];
		fadj_af.gl_HALOFS_Z = (otp_af_data[3] << 8) | otp_af_data[2];
		fadj_af.gl_PSTZOF = (otp_af_data[5] << 8) | otp_af_data[4];
		fadj_af.gl_P_M_HZOFS = (otp_af_data[7] << 8) | otp_af_data[6];
		fadj_af.gl_P_M_KzHG = (otp_af_data[9] << 8) | otp_af_data[8];


	}else{
		fadj_af.gl_CURDAZ = (otp_af_data[0] << 8) | otp_af_data[1];
		fadj_af.gl_HALOFS_Z = (otp_af_data[2] << 8) | otp_af_data[3];
		fadj_af.gl_PSTZOF = (otp_af_data[4] << 8) | otp_af_data[5];
		fadj_af.gl_P_M_HZOFS = (otp_af_data[6] << 8) | otp_af_data[7];
		fadj_af.gl_P_M_KzHG = (otp_af_data[8] << 8) | otp_af_data[9];
	}

//        pr_err("fadj_af.gl_CURDAZ = 0x%x,fadj_af.gl_HALOFS_Z=0x%x,fadj_af.gl_PSTZOF=0x%x,fadj_af.gl_P_M_HZOFS=0x%x,fadj_af.gl_P_M_KzHG=0x%x",fadj_af.gl_CURDAZ,fadj_af.gl_HALOFS_Z,fadj_af.gl_PSTZOF,fadj_af.gl_P_M_HZOFS,fadj_af.gl_P_M_KzHG);
        SET_FADJ_PARAM_CLAF_2((const _FACT_ADJ_AF *)&fadj_af);
	rc = msm_get_otp_data((uint8_t*)&otp_ois_data,
		    41, OTP_REAR_CAMERA_OIS);
	if (rc < 0) {
		pr_err("failed: get rear camera ois : get otp data err %d", rc);
		return rc;
	}
        fadj.gl_CURDAT = (otp_ois_data[4] << 8) | otp_ois_data[3];
        fadj.gl_HALOFS_X = (otp_ois_data[6] << 8) | otp_ois_data[5];
        fadj.gl_HALOFS_Y = (otp_ois_data[8] << 8) | otp_ois_data[7];
        fadj.gl_HX_OFS = (otp_ois_data[10] << 8) | otp_ois_data[9];
        fadj.gl_HY_OFS = (otp_ois_data[12] << 8) | otp_ois_data[11];
        fadj.gl_PSTXOF = (otp_ois_data[14] << 8) | otp_ois_data[13];
        fadj.gl_PSTYOF = (otp_ois_data[16] << 8) | otp_ois_data[15];
        fadj.gl_GX_OFS = (otp_ois_data[18] << 8) | otp_ois_data[17];
        fadj.gl_GY_OFS = (otp_ois_data[20] << 8) | otp_ois_data[19];
        fadj.gl_KgxHG = (otp_ois_data[22] << 8) | otp_ois_data[21];
        fadj.gl_KgyHG = (otp_ois_data[24] << 8) | otp_ois_data[23];
        fadj.gl_KGXG = (otp_ois_data[26] << 8) | otp_ois_data[25];
        fadj.gl_KGYG = (otp_ois_data[28] << 8) | otp_ois_data[27];
        fadj.gl_SFTHAL_X = (otp_ois_data[30] << 8) | otp_ois_data[29];
        fadj.gl_SFTHAL_Y = (otp_ois_data[32] << 8) | otp_ois_data[31];
        fadj.gl_TMP_X_ = (otp_ois_data[34] << 8) | otp_ois_data[33];
        fadj.gl_TMP_Y_ = (otp_ois_data[36] << 8) | otp_ois_data[35];
        fadj.gl_KgxH0 = (otp_ois_data[38] << 8) | otp_ois_data[37];
        fadj.gl_KgyH0 = (otp_ois_data[40] << 8) | otp_ois_data[39];
#if 0
        pr_err("fadj.gl_CURDAT=0x%x,fadj.gl_HALOFS_X=0x%x,fadj.gl_HALOFS_Y=0x%x,fadj.gl_HX_OFS=0x%x,fadj.gl_HY_OFS=0x%x,fadj.gl_PSTXOF=0x%x,"
                  "fadj.gl_PSTYOF=0x%x,fadj.gl_GX_OFS=0x%x,fadj.gl_GY_OFS=0x%x,fadj.gl_KgxHG=0x%x,fadj.gl_KgyHG=0x%x,fadj.gl_KGXG=0x%x,"
                "fadj.gl_KGYG=0x%x,fadj.gl_SFTHAL_X=0x%x,fadj.gl_SFTHAL_Y=0x%x,fadj.gl_TMP_X_=0x%x,fadj.gl_TMP_Y_=0x%x,fadj.gl_KgxH0=0x%x,fadj.gl_KgyH0=0x%x",fadj.gl_CURDAT,
                fadj.gl_HALOFS_X,fadj.gl_HALOFS_Y,fadj.gl_HX_OFS,fadj.gl_HY_OFS,fadj.gl_PSTXOF,fadj.gl_PSTYOF,fadj.gl_GX_OFS,fadj.gl_GY_OFS,fadj.gl_KgxHG,fadj.gl_KgyHG,
                fadj.gl_KGXG,fadj.gl_KGYG,fadj.gl_SFTHAL_X,fadj.gl_SFTHAL_Y,fadj.gl_TMP_X_,fadj.gl_TMP_Y_,fadj.gl_KgxH0,fadj.gl_KgyH0);
#endif
	SET_FADJ_PARAM_2((const _FACT_ADJ *)&fadj);

	OISDBG("[%s:%d] Exit \n", __func__, __LINE__);
	return rc;
}
#endif

#ifdef X5
unsigned int DOWNLOAD_CALIB_OTP[10];
unsigned int CALIB_SIZE_OTP = sizeof(DOWNLOAD_CALIB_OTP);
static int Ois_Write_OTP_SEMCO_imx318(void)
{
	int rc = 0;
	uint8_t ois_otp_buf[MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE + 2];
        int i = 0;
	OISDBG("Enter");

        memset(DOWNLOAD_CALIB_OTP,0,CALIB_SIZE_OTP);
	rc = msm_get_otp_data(ois_otp_buf,
			MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE + 2, OTP_REAR_CAMERA_OIS);
	if (rc < 0) {
		pr_err("failed: get rear camera ois : get otp data err %d", rc);
		return rc;
	}

	/* OIS adjusted parameter */
        for (i = 0;i < CALIB_SIZE_OTP / 4; i++)
        {
          DOWNLOAD_CALIB_OTP[i] = (ois_otp_buf[i+3]<<24) | (ois_otp_buf[i+2]<<16) | (ois_otp_buf[i+1]<<8) | ois_otp_buf[i];
        }
        DOWNLOAD_CALIB_OTP[9] = ((ois_otp_buf[4*i+1]<<24) | (ois_otp_buf[4*i]<<16)) && 0xFFFF0000  ;
	OISDBG("Exit\n");
	return rc;
}
#endif
int  Ois_SetI2cClient(struct msm_camera_i2c_client *client)
{
	if (!client) {
		pr_err("%s: i2c client is NULL !!!\n", __func__);
		return INIT_FAILED;
	} else {
		g_i2c_clinet = client;
		g_i2c_clinet->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	}
	return INIT_SUCCESS;
}
static int lc898122_check_i2c(void)
{
	unsigned char check_data;

	RegReadA(LC898122_CHECK_RIGISTER, &check_data);
	if (LC898122_CHECK_DATA == check_data) {
		return INIT_SUCCESS;
	} else {
		pr_err("ois lc898122_check_i2c error : 0x%x\n", check_data);
		return INIT_FAILED;
	}
}

static int bu63165_check_i2c(void)
{
	OIS_UWORD check_data;
	//unsigned short check_data1;
	int ret = 0;

	OISDBG("[%s:%d] E.\n", __func__, __LINE__);
	check_data = I2C_OIS_per__read(BU63165_CHECK_RIGISTER);
	pr_err("[%s:%d] check data = 0x%x\n", __func__, __LINE__, check_data);
	if (BU63165_CHECK_DATA == check_data) {
		//return INIT_SUCCESS;
	} else {
		pr_err("[%s:%d] check data read error : 0x%x\n", __func__, __LINE__, check_data);
		return INIT_FAILED;
	}

	I2C_OIS_per_write(0x20,0xAA55);//the write value can be anyone
	check_data = I2C_OIS_per__read(0x20);
	pr_err("[%s:%d] reg:0x20, w = 0xAA55, r = 0x%x\n", __func__, __LINE__, check_data);

	if (check_data == 0xAA55) {// if it's 0x55AA, checke i2c format.
		OISDBG("[%s:%d] Check write data Success.\n", __func__, __LINE__);
		ret = INIT_SUCCESS;
	} else {
		pr_err("[%s:%d] w/r error : 0x%x\n", __func__, __LINE__, check_data);
		ret = INIT_FAILED;
	}
	OISDBG("[%s:%d] X.\n", __func__, __LINE__);
	return ret;
}

static int debug_ois_en = 1;
static int lc898122_max_plus_init(int32_t type)
{
	int32_t rc = 0;
	rc = lc898122_check_i2c();
	if (rc < 0)
		return INIT_FAILED;
	SelectModule(MODULE_20M);
	OISDBG("  max_plus type:%d\n", type);
	switch (type) {
	case OIS_INIT_S:
		OISDBG(" Init set");
		set_ois_flag();
		IniSetAf();
if(debug_ois_en)
{
		IniSet();
		if (get_ois_flag() == 0)
			break;
		rc = Ois_Write_OTP_sharp_imx230();
		RemOff(ON);
		//SetTregAf(0x0100);
		RtnCen(0x00);
		msleep(150);
		RemOff(OFF);

//		SetPanTiltMode(ON);
		OISDBG("return center");
}
		break;
	case OIS_CENTERING_ON_S:

if(debug_ois_en)
{
		if (get_ois_flag() == 0)
			break;
		/*RemOff(ON);
		SetTregAf(0x0100);
		RtnCen(0x00);
		msleep(150);
		RemOff(OFF);
		OISDBG(" return center");
		*/
}
		break;
	case OIS_CENTERING_OFF_S:
		break;
	case OIS_PANTILT_ON_S:

if(debug_ois_en)
{
		if (get_ois_flag() == 0)
			break;
		SetPanTiltMode(ON);
		OISDBG(" SetPanTilode over\n");
}
		break;
	case OIS_ENABLE_S:

if(debug_ois_en)
{
		if (get_ois_flag() == 0)
			break;
		OisEna();
		OISDBG(" OisEna over\n");
}
		break;
	case OIS_DISABLE_S:
		/* OisDisable(); */
		break;
	case OIS_STILL_MODE_S:
if(debug_ois_en)
{
		if (get_ois_flag() == 0)
			break;
		SetH1cMod(0);
		OISDBG(" SetSTILLMode over\n");
}
		break;
	case OIS_MOVIE_MODE_S:
if(debug_ois_en)
{
		if (get_ois_flag() == 0)
			break;
		SetH1cMod(MOVMODE);
		OISDBG(" SetmoveMODE over\n");
}
		break;
	case OIS_CALIBRATION_S:
		break;
	case OIS_POWERDOWN:
		SmoothSvrOff();
		break;
	default:
		pr_err("Not have thise case type :%d\n", type);
		break;
	}
	return rc;
}

uint8_t module_id;
static  uint8_t get_module_id(void)
{
	uint8_t moduleid_otp_buf[MSM_OTP_REAR_CAMERA_MODULE_ID_BUFF_SIZE];
	memset(moduleid_otp_buf, 0,
			MSM_OTP_REAR_CAMERA_MODULE_ID_BUFF_SIZE);
	msm_get_otp_data(moduleid_otp_buf,
			MSM_OTP_REAR_CAMERA_MODULE_ID_BUFF_SIZE,
			OTP_REAR_CAMERA_MODULE_ID);
	pr_err("%s:OTP module id:0x%x \n", __func__,
			 moduleid_otp_buf[0]);
        return  moduleid_otp_buf[0];
}
/* 2015-12-02 add-s x2 sunny rohm bu63165*/
static int bu63165_x2_dl_fw(void)
{
	int32_t rc = 0;
	//ADJ_STS rt = 0;
	OISDBG("[%s:%d] E.\n", __func__, __LINE__);
	rc = bu63165_check_i2c();
	if (rc < 0)
	{
		pr_err("[%s:%d] check i2c err.\n", __func__, __LINE__);
		goto exit;
	}

	VCOSET0();
	OISDBG("[%s:%d] VCOSET0 Succeed.\n", __func__, __LINE__);

	rc = func_PROGRAM_DOWNLOAD();
	if (0 != rc)
	{
		pr_err("[%s:%d] download program err, rt = %d\n", __func__, __LINE__, rc);
		goto exit;
	}
	OISDBG("[%s:%d] func_PROGRAM_DOWNLOAD Succeed.\n", __func__, __LINE__);

	func_COEF_DOWNLOAD(0);
	OISDBG("[%s:%d] func_COEF_DOWNLOAD Succeed.\n", __func__, __LINE__);

	VCOSET1();
	OISDBG("[%s:%d] VCOSET1 Succeed.\n", __func__, __LINE__);
	pr_err(" module_id =0x%x",module_id);
        if (module_id == LG_MODULE){
		rc = Ois_Write_OTP_lg_imx230();
		if (0 != rc)
		{
			pr_err("[%s:%d] wr otp err, rc = %d\n", __func__, __LINE__, rc);
			goto exit;
		}
		pr_err("[%s:%d] lg module Write OTP Succeed.\n", __func__, __LINE__);
	}else if (module_id == SUNNY_MODULE){
		rc = Ois_Write_OTP_sunny_imx230();
		if (0 != rc)
		{
			pr_err("[%s:%d] wr otp err, rc = %d\n", __func__, __LINE__, rc);
			goto exit;
		}
		pr_err("[%s:%d] sunny vWrite OTP Succeed.\n", __func__, __LINE__);
	}
	I2C_OIS_spcl_cmnd( 1, _cmd_8C_EI );
exit:
	pr_err("[%s:%d] X.\n", __func__, __LINE__);
	return rc;
}

static int bu63165_x2_init(int32_t type)
{
	int32_t rc = 0;
	OISDBG("[%s:%d] type:%d\n", __func__, __LINE__, type);

	switch (type) {
		case OIS_INIT_S:
			set_ois_flag();

			pr_err("get_ois_flag() :%d\n",get_ois_flag());
			if (get_ois_flag() == 0)
			        break;
			module_id = get_module_id();
			ois_dw_status = 0;
		        bu63165_x2_dl_fw();

			func_SET_SCENE_PARAM_for_NewGYRO_Fil(_SCENE_SPORT_3, 0, 0, 1, (const _FACT_ADJ *)&fadj);
			ois_dw_status = 1;
			pr_err("======================>>>> after  ois_init\n");
			break;
		case OIS_ENABLE_S:
		        ois_dw_status = 0;
                        OISDBG("ois enable");
			if (get_ois_flag() == 0)
				break;
		/* Change SCENE parameter OIS-ON */
			func_SET_SCENE_PARAM_for_NewGYRO_Fil(_SCENE_SPORT_3, 1, 0, 1, (const _FACT_ADJ *)&fadj);
			OISDBG("[%s:%d] ois enable.\n", __func__, __LINE__);
		        ois_dw_status = 1;
			break;
		case OIS_DISABLE_S:
		        ois_dw_status = 0;
			OISDBG("[%s:%d] ois diable.\n", __func__, __LINE__);
			/* Change SCENE parameter OIS-OFF */
			func_SET_SCENE_PARAM_for_NewGYRO_Fil(_SCENE_SPORT_3, 0, 0, 1, (const _FACT_ADJ *)&fadj);
		        ois_dw_status = 1;
		case OIS_MOVIE_MODE_S:
		//	func_SET_SCENE_PARAM_for_NewGYRO_Fil(_SCENE_SPORT_3, 1, 0, 1, (const _FACT_ADJ *)&fadj);
		case OIS_STILL_MODE_S:
		//	func_SET_SCENE_PARAM_for_NewGYRO_Fil(_SCENE_SPORT_3, 0, 0, 1, (const _FACT_ADJ *)&fadj);
			break;
		case OIS_POWERDOWN:
			ois_dw_status = 0;
			break;
		default:
			pr_err("[%s:%d] Not have thise case type :%d\n", __func__, __LINE__,  type);
			break;
	}

	return rc;
}

#ifdef X5
/*2016-01-08 add x5 bu24218 ois of rohm */
static int bu24218_x5_init(int32_t type)
{
	int32_t rc = 0;

	OISDBG("[%s:%d] type:%d\n", __func__, __LINE__, type);

	switch (type) {
	case OIS_INIT_S:
		OISDBG("[%s:%d] init\n", __func__, __LINE__);
		set_ois_flag();
		if (get_ois_flag() == 0)
			break;
                Ois_Write_OTP_SEMCO_imx318();
		BU24_OIS_Download();
                BU24_OIS_Servo_Set(SERVO_ON);
		OISDBG(" return center");
		break;
	case OIS_ENABLE_S:
		if (get_ois_flag() == 0)
			break;
		/* Change SCENE parameter OIS-ON */
                //BU24_OIS_Servo_Set(SERVO_ON);
                BU24_OIS_Gyro_On();
                BU24_OIS_Change_Mode(ZSL_MODE);
		OISDBG("[%s:%d] ois enable.\n", __func__, __LINE__);
		break;
	case OIS_DISABLE_S:
                BU24_OIS_Servo_Set(SERVO_ON);
		OISDBG("[%s:%d] ois diable.\n", __func__, __LINE__);
		break;
	default:
		pr_err("[%s:%d] Not have thise case type :%d\n", __func__, __LINE__,  type);
		break;
	}

	return rc;
}
/*2016-01-08 add x5 bu24218 ois of rohm */
#endif
/* 2015-12-02 add-s x2 sunny rohm bu63165*/
int oiscontrol_interface(struct msm_camera_i2c_client *client,
		const char *module_name, int32_t type)
{
	int32_t rc = 0;

	pr_err("\n =====[%s:%d] E, slave addr = 0x%x, module_name = %s.\n", __func__, __LINE__, client->cci_client->sid, module_name);
	if (strcmp(module_name, "maxplus") == 0)
	{
		Ois_SetI2cClient(client);
		rc = lc898122_max_plus_init(type);
	}
	else if (strcmp(module_name, "x2") == 0)
	{
		Ois_SetI2cClient(client);//add for debug i2c
		Ois_rohm_user_SetI2cClient(client);
		Ois_rohm_func_SetI2cClient(client);
		rc = bu63165_x2_init(type);
	}
#ifdef  X5
	else if (strcmp(module_name, "x5") == 0)
	{
		Ois_rohm_SetI2cClient(client);
		rc = bu24218_x5_init(type);
	}
#endif
	OISDBG("[%s:%d] X.\n", __func__, __LINE__);
	return rc;
}

/*
int oiscontrol_interface2(struct msm_camera_i2c_client *client)
{
	int32_t rc = 0;
	Ois_SetI2cClient(client);
//	client->cci_client->sid =0x24;
//	client->cci_client->retries = 3;
//	client->cci_client->id_map = 0;
//	client->cci_client->cci_i2c_master = 0;
//	client->cci_client->i2c_freq_mode = 1;

	rc = lc898122_check_i2c();
	if (rc < 0)
		return INIT_FAILED;
	SelectModule(MODULE_20M);

	OISDBG(" Init set");
	set_ois_flag();
	IniSetAf();
	IniSet();
	rc = Ois_Write_OTP_sharp_imx230();
	RemOff(ON);
	SetTregAf(0x0400);
	RtnCen(0x00);
	msleep(150);
	RemOff(OFF);
	return INIT_SUCCESS;
}
*/

