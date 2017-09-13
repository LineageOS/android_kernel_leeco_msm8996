
#include "BU24_defi.h"
#include "BU24_dwnld.h"

#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT  pr_err
#else
#define DEBUG_PRINT
#endif
#define RETRY_TIMES	10
#define INIT_FAILED -1
#define INIT_SUCCESS 1
struct msm_camera_i2c_client *g_i2c_ctrl;
int  Ois_rohm_SetI2cClient(struct msm_camera_i2c_client *client)
{
        if (!client) {
                pr_err("%s: i2c client is NULL !!!\n", __func__);
                return INIT_FAILED;
        } else {
                g_i2c_ctrl = client;
        }
        return INIT_SUCCESS;
}

int I2C_OIS_Write_4Byte(unsigned short addr, unsigned long dat )
{
	int retval;
	unsigned char data_wr[4];
	data_wr[0] = (dat >> 24) & 0xFF;
	data_wr[1] = (dat >> 16) & 0xFF;
	data_wr[2] = (dat >> 8) & 0xFF;
	data_wr[3] = dat & 0xFF;
	g_i2c_ctrl->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	retval = g_i2c_ctrl->i2c_func_tbl->i2c_write_seq(
			    g_i2c_ctrl, addr, data_wr, 4);
	return retval;
}

int I2C_OIS_Write_Seq(unsigned short addr, unsigned char* data, unsigned char size)
{
	int retval;
	g_i2c_ctrl->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	retval = g_i2c_ctrl->i2c_func_tbl->i2c_write_seq(
                 g_i2c_ctrl, addr, data, size);
	return retval;
}

int I2C_OIS_Write_Byte(unsigned short addr, unsigned char data)
{
	int retval;
	g_i2c_ctrl->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	retval = g_i2c_ctrl->i2c_func_tbl->i2c_write(
			 g_i2c_ctrl, addr, data, MSM_CAMERA_I2C_BYTE_DATA);
	return retval;
}

unsigned long I2C_OIS_Read_4Byte(unsigned short addr)
{
        int rc = 0;
        unsigned long t_data,RamData;
        unsigned char r_data[4];
        g_i2c_ctrl->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
        rc = msm_camera_cci_i2c_read_seq(g_i2c_ctrl,
                addr, r_data, 4);
        if (rc < 0)
                pr_err("%s: %x: Read addr read failed\n",
                        __func__, addr);
        t_data = (r_data[0]<<24)|(r_data[1]<<16)|(r_data[2]<<8)|r_data[3];
        pr_err("%s,r_data[0]=%x,r_data[1]=%x,r_data[2]=%x,r_data[3]=%x,t_data = 0x%lx",__func__,r_data[0],r_data[1],r_data[2],r_data[3],t_data);
        RamData = (t_data & 0xFFFFFFFF);
        pr_err("%s,Ramdata = 0x%lx",__func__,RamData);
        return RamData;

}

unsigned char I2C_OIS_Read_Byte(unsigned short addr)
{
	unsigned char read_data = 0;
	g_i2c_ctrl->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
        g_i2c_ctrl->i2c_func_tbl->i2c_read_seq(
                g_i2c_ctrl, addr, &read_data, 1);
        pr_err("RD_I2C addr:0x%04x data:0x%04x \n", addr, read_data);
	return read_data;
}

void waitms(short ms)
{
        msleep(ms);
}

int BU24_OIS_Gyro_On(void)
{
	int sts;
	// Need to change condition of IMX318
	// Set 0x01 @ 0x30AC for OIS ( default is 0x00 for EIS )

	sts = I2C_OIS_Write_Byte(GYRO_CNTL, 0x02);
	if(sts < 0){
		DEBUG_PRINT("%s (%d): I2C Write Error: %d\n", __func__, __LINE__, sts);
		return sts;
		}
	sts = I2C_OIS_Write_Byte(GYRO_SET1, 0x6B);	// PWR_MGMT_1 Register of Gyro Sensor
	if(sts < 0){
		DEBUG_PRINT("%s (%d): I2C Write Error: %d.\n", __func__, __LINE__, sts);
		return sts;
		}
	sts = I2C_OIS_Write_Byte(GYRO_SET2, 0x00);	// Power on Gyro Sensor
	if(sts < 0){
		DEBUG_PRINT("%s (%d): I2C Write Error: %d.\n", __func__, __LINE__, sts);
		return sts;
		}
	waitms(100);
	sts = I2C_OIS_Write_Byte(GYRO_SET1, 0x1B);	// - @ FS_SEL = 0, 0x1B @ FS_SEL = 1 / 2 / 3
	if(sts < 0){
		DEBUG_PRINT("%s (%d): I2C Write Error: %d.\n", __func__, __LINE__, sts);
		return sts;
		}
	sts = I2C_OIS_Write_Byte(GYRO_SET2, 0x18);	// - @ FS_SEL = 0, 0x08 @ FS_SEL = 1, 0x10 @ FS_SEL = 2, 0x18 @ FS_SEL = 3
	if(sts < 0){
		DEBUG_PRINT("%s (%d): I2C Write Error: %d.\n", __func__, __LINE__, sts);
		return sts;
		}
	sts = I2C_OIS_Write_Byte(GYRO_CNTL, 0x00);
	if(sts < 0){
		DEBUG_PRINT("%s (%d): I2C Write Error: %d.\n", __func__, __LINE__, sts);
		return sts;
		}
        return sts;
}

//extern unsigned int  CALIB_SIZE_OTP;
//extern unsigned int  DOWNLOAD_CALIB_OTP[];
int BU24_OIS_Download(void)
{
	int sts = 0;
        int i = 0;
	unsigned long checksum, rev_id;

	//unsigned char temp[4];

	DEBUG_PRINT("%s (%d): start  = %d. \n", __func__, __LINE__, sts);
	sts = I2C_OIS_Write_Byte(DWNLD_START, 0x00);
	if(sts < 0){
		DEBUG_PRINT("%s (%d): Write Failed. Error = %d. \n", __func__, __LINE__, sts);
		return sts;
		}
	waitms(1);
	for(i = 0; i < (DATA1_SIZE / 4); i++){
		I2C_OIS_Write_4Byte(DWNLD_DATA1_START + i * 4, DOWNLOAD_DATA1[i]);
		}
	for(i = 0; i < (DATA2_SIZE / 4); i++){
		I2C_OIS_Write_4Byte(DWNLD_DATA2_START + i * 4, DOWNLOAD_DATA2[i]);
		}
	checksum = I2C_OIS_Read_4Byte(DWNLD_CHECKSUM);	// 32-bit value
	if(checksum != BIN_CHECKSUM){
		DEBUG_PRINT("%s: Firmware Download Failed.\n", __func__);
		return -1;
		}
	for(i = 0; i < (CALIB_SIZE / 4); i++){
		I2C_OIS_Write_4Byte(DWNLD_CALIB_START + i * 4 , DOWNLOAD_CALIB[i]);
		}

	I2C_OIS_Write_Byte(DWNLD_COMPLETE, 0x00);

	for(i = 0; i < RETRY_TIMES; i++){
		waitms(1);
		sts = I2C_OIS_Read_Byte(OIS_STS);
		if(sts == 0x01)
			break;
		}
	if(sts == 0x00){
		DEBUG_PRINT("%s: OIS Status Check Not Ready.\n", __func__);
		return -1;
		}

	rev_id = I2C_OIS_Read_4Byte(DWNLD_REVISION);	// 32-bit value
	if(rev_id != BIN_REVISION){
		DEBUG_PRINT("%s: Program Revision Error.\n", __func__);
		return -1;
		}
	return 0;
}

int BU24_OIS_Change_Mode(int mode)
{
	int sts,i;

	if(!((mode == VIEWFINDER_MODE) || (mode == MOVIE_MODE_1) || (mode == MOVIE_MODE_2) 
		|| (mode == ZSL_MODE) || (mode == STILL_MODE))){
		DEBUG_PRINT("%s: Invalid Mode Setting. Mode = %x. \n", __func__, mode);
		return -1;
		}

	sts = I2C_OIS_Write_Byte(OIS_MODE, mode);
	if(sts < 0){
		DEBUG_PRINT("%s: I2C Write Error.\n", __func__);
		return -1;
		}

	for(i = 0; i < RETRY_TIMES; i++){
		waitms(10);
		sts = I2C_OIS_Read_Byte(OIS_STS);
		if(sts == 0x01)
			break;
		}
	if(sts != 0x01){
		DEBUG_PRINT("%s: OIS Status Check Not Ready.\n", __func__);
		return -1;
		}

	sts = I2C_OIS_Write_Byte(OIS_CNTL, OIS_ON);
	if(sts < 0){
		DEBUG_PRINT("%s: I2C Write Error.\n", __func__);
		return -1;
		}

	return 0;
}

int BU24_OIS_Servo_Set(unsigned char enable)
{
	int sts,i;

	if(enable){	//enable = 1
		sts = I2C_OIS_Write_Byte(OIS_CNTL, SERVO_ON);
		if(sts < 0){
			DEBUG_PRINT("%s: I2C Write Error.", __func__);
			return -1;
			}
		for(i = 0; i < RETRY_TIMES; i++){
			waitms(100);
			sts = I2C_OIS_Read_Byte(OIS_STS);
			if(sts == 0x01)
				break;
			}
		if(sts != 0x01){
			DEBUG_PRINT("%s: OIS Status Check Not Ready.\n", __func__);
			return -1;
			}
		}
	else{	//enable = 0
		sts = I2C_OIS_Write_Byte(OIS_CNTL, SERVO_OFF);
		if(sts < 0){
			DEBUG_PRINT("%s: I2C Write Error.", __func__);
			return -1;
			}
		}

	return 0;
}
