#include <linux/i2c.h>
#include <linux/gpio.h>
#include <soc/qcom/camera2.h>
#include <media/v4l2-subdev.h>
#include <media/msmb_camera.h>
#include "msm_cci.h"
#include "msm_camera_i2c.h"
#include "msm_camera_dt_util.h"
#include "msm_camera_io_util.h"
#include "msm_sensor_driver.h"
extern int msm_get_otp_data(uint8_t*, uint32_t, uint16_t);
extern int Get_Ois_DW_Status(void);

int oiscontrol_interface(struct msm_camera_i2c_client *client,
		 const char *module_name, int32_t type);
