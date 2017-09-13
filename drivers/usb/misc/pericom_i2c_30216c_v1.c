/*
 * PERICOM 30216C driver
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>

#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>


#include <linux/pericom_i2c_30216c_v1.h>


#define DRIVER_NAME "pericom_30216c"

/*
   pericom data struct
  */
struct pericom_30216c_data{
	struct i2c_client *i2c_client;
    struct work_struct check_work;
    int irq;
	unsigned int irq_gpio;
	struct mutex i2c_rw_mutex;
    struct pinctrl *pinctrl_int;
	struct pinctrl_state *intr_active;
	struct pinctrl *pinctrl_switch;
	struct pinctrl_state *switch_active;
	struct pinctrl *pinctrl_ccpwr;
	struct pinctrl_state *ccpwr_active;
	unsigned int 		switch_gpio1;
	unsigned int 		switch_gpio2;
	unsigned int 		cc1_pwr_gpio;
	unsigned int 		cc2_pwr_gpio;
};

static bool pericom_analog_headset_plugin = false;
//static bool i2c_op_fail = false;
static struct mutex typec_headset_lock;
extern bool typec_set_cc_state;
extern void cclogic_updata_port_state(int state);
extern void cclogic_updata_port_polarity(int polarity);
extern int msm_usb_vbus_set(void *_mdwc, bool on, bool ext_call);
extern int wcd_mbhc_plug_detect(void);
extern void cclogic_set_audio_mode_register(void (*func)(bool));
extern int cclogic_set_vbus(bool on);
extern int pi5usb_set_msm_usb_host_mode(bool mode);
static struct pericom_30216c_data global_pericom_data;
extern void cclogic_set_typec_headset_with_analog(int val);
extern bool serial_hw_output_enable(void);
static bool always_uart_debug = false;

/**
* pericom_30216c_i2c_read()
*
* Called by various functions in this driver,
* This function reads data of an arbitrary length from the sensor,
* starting from an assigned register address of the sensor, via I2C
* with a retry mechanism.
*/
static int pericom_30216c_i2c_read(struct pericom_30216c_data *pericom_data,
		unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	struct i2c_msg msg[] = {
		{
			.addr = pericom_data->i2c_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	mutex_lock(&(pericom_data->i2c_rw_mutex));

	for (retry = 0; retry < PERICOM_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(pericom_data->i2c_client->adapter, msg, 1) > 0) {
			retval = length;
			break;
		}
		dev_err(&pericom_data->i2c_client->dev,
				"%s: I2C retry %d\n",__func__, retry + 1);
		msleep(20);
	}

	if (retry == PERICOM_I2C_RETRY_TIMES) {
		dev_err(&pericom_data->i2c_client->dev,
				"%s: I2C read over retry limit\n",	__func__);
		retval = -EIO;
	}

	mutex_unlock(&(pericom_data->i2c_rw_mutex));

	return retval;
}

 /**
 * pericom_30216c_i2c_write()
 *
 * Called by various functions in this driver
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int pericom_30216c_i2c_write(struct pericom_30216c_data *pericom_data,
		unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	struct i2c_msg msg[] = {
		{
			.addr = pericom_data->i2c_client->addr,
			.flags = 0,
			.len = length ,
			.buf = data,
		}
	};

	mutex_lock(&(pericom_data->i2c_rw_mutex));

	for (retry = 0; retry < PERICOM_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(pericom_data->i2c_client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(&pericom_data->i2c_client->dev,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);
	}

	if (retry == PERICOM_I2C_RETRY_TIMES) {
		dev_err(&pericom_data->i2c_client->dev,
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

	mutex_unlock(&(pericom_data->i2c_rw_mutex));

	return retval;
}

/*set mode*/
static int pericom_30216c_set_power_mode(struct pericom_30216c_data *pericom_data,enum pericom_power_mode mode)
{
	int ret;
	char buf[2] = {0x20,0};

	//read reg1 value
	pericom_30216c_i2c_read(pericom_data,buf,2);
	//construct reg1 value
	buf[1] =(buf[1] & ~PERICOM_POWER_SAVING_MASK)| ((mode << PERICOM_POWER_SAVING_OFFSET )& PERICOM_POWER_SAVING_MASK);
	buf[1] &= ~PERICOM_INTERRUPT_MASK;

	//write reg1 value
	ret = pericom_30216c_i2c_write(pericom_data, buf, 2);
	return ret;
}

static int pericom_30216c_set_role_mode(struct pericom_30216c_data *pericom_data,enum pericom_role_mode mode)
{
	int ret,i;
	char buf[4] = {0,0,0,0};

	//read reg1 value
	pericom_30216c_i2c_read(pericom_data,buf,2);
	//construct reg1 value
	if(mode==0x00) {
		buf[1] = (buf[1] & ~PERICOM_ROLE_MODE_MASK)
			|((mode << PERICOM_ROLE_OFFSET )& PERICOM_ROLE_MODE_MASK);      //device mode 0x00

		}
	else if(mode==0x01){
		buf[1] = (buf[1] & ~PERICOM_ROLE_MODE_MASK)
			|((mode << PERICOM_ROLE_OFFSET )& PERICOM_ROLE_MODE_MASK);		//Host mode 0x02,0x0a,0x12
	}
	else //default typec mode is trysnk mode
		buf[1] = (buf[1] & ~PERICOM_ROLE_MODE_MASK)
			|(PERICOM_ROLE_MODE_MASK|PERICOM_DRP2_TRY_SNK);   //TrySNK DRP mode 0x46,0x4e,0x56

	//buf[1] = (buf[1] & ~PERICOM_ROLE_MODE_MASK)
	//		|((mode << PERICOM_ROLE_OFFSET )& PERICOM_ROLE_MODE_MASK);

	/* mask interrupt */
	buf[1] |= PERICOM_INTERRUPT_MASK;
	//write reg1 value
	ret = pericom_30216c_i2c_write(pericom_data, buf, 2);

	/* sleep and wait for correct status */
	for (i = 0; i <= 5; i++) {
		msleep(1500);
		pericom_30216c_i2c_read(pericom_data, buf, 4);
		dev_info(&pericom_data->i2c_client->dev,
				"read:%d,%d,%d,%d\n",buf[0],buf[1],buf[2],buf[3]);
		/* mode=0 : device, reg4 must be 0x08
		  * mode=1: host , reg4 must be 0x04
		  * mode=2: drp,  all value is ok
		  */
		if (((mode == DEVICE_MODE) && (buf[3] & 0x08))
			|| ((mode == HOST_MODE) && (buf[3] & 0x04))
			|| (mode == TRYSNK_DRP_MODE))
			break;
	}

	if ( i > 5 ) dev_info(&pericom_data->i2c_client->dev, "try to %d mode fail \n", mode);
	/* unmask interrupt*/
	buf[1] &= ~PERICOM_INTERRUPT_MASK;
	//write reg1 value
	ret = pericom_30216c_i2c_write(pericom_data, buf, 2);

	return ret;
}

#if 0
/*set device mode*/
static int pericom_30216c_set_device_mode(struct pericom_30216c_data *pericom_data)
{
	return pericom_30216c_set_role_mode(pericom_data,DEVICE_MODE);
}

/* set host mode
  * success if return positive value ,or return negative
  */
static int pericom_30216c_set_host_mode(struct pericom_30216c_data *pericom_data)
{
	return pericom_30216c_set_role_mode(pericom_data,HOST_MODE);
}

/* set DRP mode
  * success if return positive value ,or return negative
  */
//static int pericom_30216c_set_drp_mode(struct pericom_30216c_data *pericom_data)
//{
//	return pericom_30216c_set_role_mode(pericom_data,DRP_MODE);
//}
#endif
static int pericom_30216c_set_trysnk_drp_mode(struct pericom_30216c_data *pericom_data)
{
	return pericom_30216c_set_role_mode(pericom_data,TRYSNK_DRP_MODE);
}

//static int pericom_30216c_set_trysrc_drp_mode(struct pericom_30216c_data *pericom_data)
//{
//	return pericom_30216c_set_role_mode(pericom_data,TRYSRC_DRP_MODE);
//}

/* get role mode
  * success if return positive value ,or return negative
  */

#if 0
static int pericom_30216c_get_role_mode(struct pericom_30216c_data *pericom_data)
{
	int ret;
	char buf[2] = {0,0};

	//read reg1 value
	ret = pericom_30216c_i2c_read(pericom_data,buf,2);
	//construct reg1 value
	buf[1] = (buf[1] & PERICOM_ROLE_MODE_MASK) >> PERICOM_ROLE_OFFSET;
	return ret > 0 ? buf[1]:ret;
}
#endif

/* set power saving mode
  * success if return positive value ,or return negative
  */
static int pericom_30216c_set_powersaving_mode(struct pericom_30216c_data *pericom_data)
{
	return pericom_30216c_set_power_mode(pericom_data,POWERSAVING_MODE);
}

/* set active mode
  * success if return positive value ,or return negative
  */
static int pericom_30216c_set_poweractive_mode(struct pericom_30216c_data *pericom_data)
{
	return pericom_30216c_set_power_mode(pericom_data,ACTIVE_MODE);
}

#if 0
static int pericom_30216c_get_id_status(struct typec_dev *dev)
{
	struct pericom_30216c_data *pericom_data =
		  container_of(dev,struct pericom_30216c_data,c_dev);

	return pericom_30216c_get_role_mode(pericom_data);
}

static int pericom_30216c_set_id_status(struct typec_dev *dev,int value)
{
	int rc = -1;
	struct pericom_30216c_data *pericom_data =
		  container_of(dev,struct pericom_30216c_data,c_dev);

	if (value == DEVICE_MODE)
		rc = pericom_30216c_set_device_mode(pericom_data);
	else if (value == HOST_MODE)
		rc = pericom_30216c_set_host_mode(pericom_data);
	//else if (value == DRP_MODE)
	//	rc = pericom_30216c_set_drp_mode(pericom_data);
    else// (value == TRYSNK_DRP_MODE)
		rc = pericom_30216c_set_trysnk_drp_mode(pericom_data);        //trysnk mode
	//else if (value == TRYSRC_DRP_MODE)
	//	rc = pericom_30216c_set_trysrc_drp_mode(pericom_data);		  //trysrc mode
	return rc;
}

/* get cc  orientation
  * success if return positive value ,or return 0
  * return 1 means cc1
  * return 2 means cc2.
  */
static int pericom_30216c_get_cc_orientation(struct pericom_30216c_data *pericom_data)
{
	int ret;
	char buf[4] = {0,0,0,0};

	//read reg4 value
	ret = pericom_30216c_i2c_read(pericom_data,buf,4);
	//construct reg1 value
	buf[3] = buf[3] & PERICOM_CC_ORI_MASK;

	return ret > 0 ? buf[3]:0;
}

static int pericom_30216c_get_cc_pin(struct typec_dev *dev)
{
	struct pericom_30216c_data *pericom_data =
		  container_of(dev,struct pericom_30216c_data,c_dev);

	return pericom_30216c_get_cc_orientation(pericom_data);
}
#endif

static bool ic_is_present(struct pericom_30216c_data *pericom_data)
{
	char buf = 0;

	//read reg0 value
	pericom_30216c_i2c_read(pericom_data,&buf,1);
	pr_info("pericom VID = %x\n", buf);

	return (buf == 0x20)?true:false;    //30216C ChipID is 0x20
}

/*This func is for headset switch AD mode*/
void pericom_cclogic_set_audio_mode(bool mode)
{
	/*true for Analog mode and false for Digit mode*/
	//struct cyccg_platform_data *pdata = cyccg_pdata;
	struct pericom_30216c_data *pericom_data = &global_pericom_data;
	unsigned char reg_val;
    unsigned char buf[4];

	#ifdef HD_SWITCH_DBG
	switch_cnt++;
	#endif
	mutex_lock(&typec_headset_lock);
    #ifdef HD_SWITCH_DBG
    pr_info("pericom switch nyx , c:%d, m:%d\n", switch_cnt, mode);
    #endif

	if(true == always_uart_debug)
	{
		pr_err("Err: Serial HW-Output Enable!\n");
	}

	if (mode) {
		pr_info("pericom switch to audio mode\n");
        msleep(50);
		pi5usb_set_msm_usb_host_mode(false);/*simulate plugout*/

		gpio_set_value(pericom_data->switch_gpio2, 1);/*Headphone*/
		gpio_set_value(pericom_data->switch_gpio1, 0);/*MIC*/
		//ptn5150_read_reg(pericom_data->client, PTN5150_CC_STATUS_REG, &reg_val);
		pericom_30216c_i2c_read(pericom_data, buf, 4);
        reg_val = buf[3];
        if (0x1 == (reg_val&0x3)) {
            gpio_set_value(pericom_data->cc2_pwr_gpio, 1);
		} else if (0x2 == (reg_val&0x3)) {
            gpio_set_value(pericom_data->cc1_pwr_gpio, 1);
		} else {
            pr_err("pericom err:%x\n", reg_val);
		}

		//cyccg_pdata->mode = AUDIO_MODE;
		//if (!letv_typec_plug_state)
			//letv_typec_plug_state = true;
		//notify audio module for headset plug in
		if (!pericom_analog_headset_plugin) {
			pr_info("pericom audio headset plug in!!\n");
			pericom_analog_headset_plugin = true;
			wcd_mbhc_plug_detect();
		}
		cclogic_set_typec_headset_with_analog(1);
		cclogic_updata_port_state(3);/*"cc_state: audio"*/
		//cyccg_usb_audio_insert = true;
		/*The behavior above */
	} else {
        pr_info("pericom switch to digital nyx\n");
        gpio_set_value(pericom_data->switch_gpio2, 0);/*USB*/
		gpio_set_value(pericom_data->switch_gpio1, 0);

        msleep(30);
		pi5usb_set_msm_usb_host_mode(true);
		gpio_set_value(pericom_data->cc1_pwr_gpio, 0);
		gpio_set_value(pericom_data->cc2_pwr_gpio, 0);
		//pr_info("ptn5150 set usb to host mode\n");
		//pdata->mode = DFP_MODE;
		//notify audio module for headset plug out
		if (pericom_analog_headset_plugin) {
			pr_info("audio headset plug out!!\n");
			pericom_analog_headset_plugin = false;
			wcd_mbhc_plug_detect();
		}
		cclogic_set_typec_headset_with_analog(0);
		//cyccg_usb_audio_insert = false;
		//if (letv_typec_plug_state)
			//letv_typec_plug_state = false;
		cclogic_updata_port_state(2);/*"cc_state: dfp"*/
	}
	mutex_unlock(&typec_headset_lock);
	return;
}
EXPORT_SYMBOL(pericom_cclogic_set_audio_mode);

#ifdef CONFIG_PM_SLEEP
static int pericom_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct pericom_30216c_data *pericom_data = &global_pericom_data;

	enable_irq_wake(pericom_data->irq);

	return ret;
}

static int pericom_pm_resume(struct device *dev)
{
	int ret = 0;

	struct pericom_30216c_data *pericom_data = &global_pericom_data;

	disable_irq_wake(pericom_data->irq);
	return ret;
}

static const struct dev_pm_ops pericom_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pericom_pm_suspend, pericom_pm_resume)
};
#endif /* CONFIG_PM_SLEEP */

static irqreturn_t pericom_30216c_irq_handler(int irq, void *data)
{
	struct pericom_30216c_data *pericom_data = (struct pericom_30216c_data *)data;

    printk("pericom handle irq\n");

    schedule_work(&pericom_data->check_work);

	return IRQ_HANDLED;
}

static void pericom_30216c_irq_serice(struct work_struct *work)
{
	//struct pericom_30216c_data *pericom_data = container_of(work, struct pericom_30216c_data, check_work);
	struct pericom_30216c_data *pericom_data = &global_pericom_data;
	char reg[4] = {0,0,0,0};
	char curr_mode;
	static bool  plug_flag = false;

	//dev_info(&pericom_data->i2c_client->dev, "enter pericom interrupt,curr_mode:%d\n",curr_mode);
	// 0.Mask interrupt
	pr_info("pericom irq bottom!\n");
	pericom_30216c_i2c_read(pericom_data,  reg, 2);
	dev_info(&pericom_data->i2c_client->dev, "0.reg=%x,%x,%x,%x\n",reg[0],reg[1],reg[2],reg[3]);
	curr_mode = reg[1] & (PERICOM_ROLE_MODE_MASK|PERICOM_DRP2_TRY_SNK);   //support trysnk mode
	reg[1] = reg[1] | PERICOM_INTERRUPT_MASK;
	pericom_30216c_i2c_write(pericom_data, reg, 2);
	// 1.delay 30ms
	msleep(30);
	// 2.Read reg
	pericom_30216c_i2c_read(pericom_data,  reg, 4);
	dev_info(&pericom_data->i2c_client->dev, "2.reg=%x,%x,%x,%x\n",reg[0],reg[1],reg[2],reg[3]);
	// 3.Processing
	if ((reg[2] == 0x2) || (reg[3]==0x00) || (reg[3] == 0x80) ){ //detached
		if ((reg[2] == 0x2) && (reg[3]==0x00))
			plug_flag = false;
		curr_mode = (PERICOM_ROLE_MODE_MASK|PERICOM_DRP2_TRY_SNK); //enter trysnk drp mode
        /*DRP default switch to USB + UART channel*/
		gpio_set_value(pericom_data->switch_gpio2, 0);/*USB*/
		if(true == always_uart_debug)
		{
			gpio_set_value(pericom_data->switch_gpio1, 1);/*UART*/
		}
		else
		{
			gpio_set_value(pericom_data->switch_gpio1, 0);/*close UART > MIC*/
		}

		cclogic_set_vbus(0);

		gpio_set_value(pericom_data->cc1_pwr_gpio, 0);
		gpio_set_value(pericom_data->cc2_pwr_gpio, 0);
		cclogic_set_typec_headset_with_analog(-1);

		if (pericom_analog_headset_plugin) {
			pericom_analog_headset_plugin = false;
			pr_info("%s, audio headset plug out!!\n", __func__);
			wcd_mbhc_plug_detect();
		}

		pi5usb_set_msm_usb_host_mode(false);/*if not simulate disconnect, open*/
        cclogic_updata_port_state(0);/*"cc_state: none"*/
        cclogic_updata_port_polarity(0); /* no typec usb connected*/
        if (typec_set_cc_state) {
            pr_info("%s: clear typec_set_cc_state!\n", __func__);
            typec_set_cc_state = false;
        }

        pr_info("pericom detect cable removal\n");
	}
	else if (reg[2] == 0x01){ //attached
		if ((reg[3] == 0x05) || (reg[3] == 0x06) || (reg[3] == 0x15) || (reg[3] == 0x16)){ // Attached port status:device
            curr_mode = (PERICOM_ROLE_MODE_MASK|PERICOM_DRP2_TRY_SNK); //enter trysnk drp mode
            //Hadnset is DFP, digtial haedset
            pr_info("pericom UFP is attached!\n");
            gpio_set_value(pericom_data->switch_gpio1, 0);/*MIC*/
            cclogic_set_vbus(1);
            pi5usb_set_msm_usb_host_mode(true);
            cclogic_updata_port_state(2);/*"cc_state: dfp"*/
            //check cc orientation
            pericom_30216c_i2c_read(pericom_data,  reg, 4);
            dev_info(&pericom_data->i2c_client->dev, "2.reg=%x,%x,%x,%x\n",reg[0],reg[1],reg[2],reg[3]);
            reg[3] = reg[3] & PERICOM_CC_ORI_MASK;
            if (0x01== reg[3]) {
                cclogic_updata_port_polarity(1); /* CC1 connected*/
                pr_info("pericom cclogic port polarity:%x\n", reg[3]);
            }
            else if (0x02== reg[3]){
                cclogic_updata_port_polarity(2);/* CC2 connected*/
                pr_info("pericom cclogic port polarity:%x\n", reg[3]);
            } else {
                cclogic_updata_port_polarity(0);
                pr_info("pericom cclogic port polarity:%x\n", reg[3]);
            }
		 cclogic_set_typec_headset_with_analog(0);
		}
		else if ((reg[3] == 0x13) || (reg[3] == 0x93)) {// Debug Adapter Accessory Attached
		//
		curr_mode = (PERICOM_ROLE_MODE_MASK|PERICOM_DRP2_TRY_SNK); //enter trysnk drp mode
		}
		else if ((reg[3] == 0x0f) || (reg[3] == 0x8f)) {// Audio Accessory Attached
			curr_mode = (PERICOM_ROLE_MODE_MASK|PERICOM_DRP2_TRY_SNK); //enter trysnk drp mode
			pr_info("pericom audio Attached to an accessory!\n");
            /*switch GPIO to select audio channel*/
			gpio_set_value(pericom_data->switch_gpio2, 1);/*Headphone*/
			gpio_set_value(pericom_data->switch_gpio1, 0);/*MIC*/
			/*switch USB drive stack to Host mode*/
			cclogic_updata_port_state(3);/*"cc_state: audio"*/
			//notify audio module for headset plug in
			if (!pericom_analog_headset_plugin) {
				pr_info("%s, audio headset plug in!!\n", __func__);
				msleep(150);
				pericom_analog_headset_plugin = true;
				wcd_mbhc_plug_detect();
			}
		}
	else if ((reg[3] == 0xa9) || (reg[3] == 0x0aa) || (reg[3] == 0xc9) || (reg[3] == 0xca) || (reg[3] == 0xe9) || (reg[3] == 0xea)){
            curr_mode = (PERICOM_ROLE_MODE_MASK|PERICOM_DRP2_TRY_SNK); //enter trysnk drp mode
            gpio_set_value(pericom_data->switch_gpio1, 1);/*UART*/
            cclogic_updata_port_state(1);/*"cc_state: ufp"*/
            pr_info("pericom DFP is attached!\n");
		}
	}
	msleep(20);
	// 4. Unmask interrupt
	reg[1] = curr_mode;
	pericom_30216c_i2c_write(pericom_data, reg, 2);
	pr_info("pericom irq bottom end!\n");

	return;
}

static int pericom_30216c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	//struct regulator *i2c_vdd;
	//struct pericom_30216c_data *pericom_data = client->dev.platform_data;
	struct pericom_30216c_data *pericom_data = &global_pericom_data;
    int ret = 0;

    printk("pericom probe!\n");

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data not supported\n",
				__func__);
		return -EIO;
	}

#if 0
	if (client->dev.of_node) {
		pericom_data = devm_kzalloc(&client->dev,
			sizeof(*pericom_data),
			GFP_KERNEL);
		if (!pericom_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
	}
#endif

	if (!pericom_data) {
		dev_err(&client->dev,
				"%s: No platform data found\n",
				__func__);
		return -EINVAL;
	}

	mutex_init(&(pericom_data->i2c_rw_mutex));
	pericom_data->i2c_client = client;
	//check ic present, if not present ,exit, or go on
	if (!ic_is_present(pericom_data)) {
		dev_err(&client->dev, "Pericom30216 is absent\n");
		ret = -ENXIO;
		goto  err_chip_absent; //absent
	}

	if(true == serial_hw_output_enable())
	{
		always_uart_debug = true;
		pr_err("Serial HW-Output Enable!!!!!!\n");
	}

    //Interrupt pin configurate
    pericom_data->pinctrl_int = devm_pinctrl_get(&client->dev);
    if (IS_ERR_OR_NULL(pericom_data->pinctrl_int)) {
        pr_info("pericom %s: Unable to get pinctrl handle\n", __func__);
    }
    pericom_data->intr_active = pinctrl_lookup_state(pericom_data->pinctrl_int,
                    "m0_ccint_active");
    if (IS_ERR(pericom_data->intr_active)) {
        pr_info("pericom %s: could not get intr_active pinstate\n", __func__);
        goto err_irq_gpio;
    }

    ret = pinctrl_select_state(pericom_data->pinctrl_int,
                    pericom_data->intr_active);
    if (ret != 0) {
        pr_info("pericom %s: Disable TLMM pins failed with %d\n",
            __func__, ret);
    }

    pericom_data->irq_gpio =
        of_get_named_gpio_flags(client->dev.of_node,
        "irq-gpio", 0, NULL);
    if (gpio_is_valid(pericom_data->irq_gpio)) {
        pr_info("%s:irq gpio=%d\n", __func__, pericom_data->irq_gpio);
        ret = gpio_request(pericom_data->irq_gpio, "irq-gpio");
        if(ret) {
            pr_info("typec_irq gpio_request failed!\n");
            goto err_irq_gpio;
        }
    } else {
        pr_info("%s : irq_gpio failed!\n" , __func__);
        goto err_irq_gpio;
    }
    gpio_direction_input(pericom_data->irq_gpio);


    //Switch pin configurate
    pericom_data->pinctrl_switch = devm_pinctrl_get(&client->dev);
    if (IS_ERR_OR_NULL(pericom_data->pinctrl_switch)) {
        pr_info("ptn5150 %s: Unable to get pinctrl_switch\n", __func__);
    }
    pericom_data->switch_active = pinctrl_lookup_state(pericom_data->pinctrl_switch,
                    "m0_ccswitch_active");
    if (IS_ERR(pericom_data->switch_active)) {
        pr_info("ptn5150 %s: could not get switch_active pinstate\n", __func__);
        goto err_switch_gpio;
    }

    ret = pinctrl_select_state(pericom_data->pinctrl_switch,
                    pericom_data->switch_active);
    if (ret != 0) {
        pr_info("ptn5150 %s: Disable switch pins failed with %d\n",
            __func__, ret);
    }

    pericom_data->switch_gpio1 =
        of_get_named_gpio_flags(client->dev.of_node,
        "switch_gpio1", 0, NULL);
    if (gpio_is_valid(pericom_data->switch_gpio1)) {
        pr_info("%s:switch_gpio1=%d\n",__func__, pericom_data->switch_gpio1);
        ret = gpio_request(pericom_data->switch_gpio1, "switch_gpio1");
        if(ret) {
            pr_info("switch_gpio1 gpio_request failed!\n");
            goto err_switch_gpio;
        }
    } else {
        pr_info("%s : switch_gpio1 failed!\n" , __func__);
        goto err_switch_gpio;
    }

    pericom_data->switch_gpio2 =
        of_get_named_gpio_flags(client->dev.of_node,
        "switch_gpio2", 0, NULL);
    if (gpio_is_valid(pericom_data->switch_gpio2)) {
        pr_info("%s:switch_gpio2=%d\n",__func__, pericom_data->switch_gpio2);
        ret = gpio_request(pericom_data->switch_gpio2, "switch_gpio2");
        if(ret) {
            pr_info("switch_gpio2 gpio_request failed!\n");
            goto err_switch_gpio;
        }
    } else {
        pr_info("%s : switch_gpio2 failed!\n" , __func__);
        goto err_switch_gpio;
    }

    if (pericom_data->switch_gpio1) {
        ret = gpio_direction_output(pericom_data->switch_gpio1, 0);
        if (ret < 0) {
            pr_err("%s : not able to set switch_gpio1 as output\n",
                 __func__);
            goto err_switch_gpio;
        }
    }

    if (pericom_data->switch_gpio2) {
        ret = gpio_direction_output(pericom_data->switch_gpio2, 0);
        if (ret < 0) {
            pr_err("%s : not able to set switch_gpio2 as output\n",
                 __func__);
            goto err_switch_gpio;
        }
    }

    /* init switch_gpio state */
    gpio_set_value(pericom_data->switch_gpio2, 0);/*USB*/  //dfl:0
    if(true == always_uart_debug)
    {
        gpio_set_value(pericom_data->switch_gpio1, 1);/*UART*/
    }
    else
    {
        gpio_set_value(pericom_data->switch_gpio1, 0);
    }

    //CC Power pin configurate
    pericom_data->pinctrl_ccpwr = devm_pinctrl_get(&client->dev);
    if (IS_ERR_OR_NULL(pericom_data->pinctrl_ccpwr)) {
        pr_info("ptn5150 %s: Unable to get pinctrl_ccpwr\n", __func__);
    }
    pericom_data->ccpwr_active = pinctrl_lookup_state(pericom_data->pinctrl_ccpwr,
                    "m0_ccpwr_active");
    if (IS_ERR(pericom_data->ccpwr_active)) {
        pr_info("ptn5150 %s: could not get ccpwr_active pinstate\n", __func__);
        goto err_cc_pwr_gpio;
    }

    ret = pinctrl_select_state(pericom_data->pinctrl_ccpwr,
                    pericom_data->ccpwr_active);
    if (ret != 0) {
        pr_info("ptn5150 %s: Disable ccpwr pins failed with %d\n",
            __func__, ret);
    }

    pericom_data->cc1_pwr_gpio =
        of_get_named_gpio_flags(client->dev.of_node,
        "cc1_pwr_gpio", 0, NULL);
    if (gpio_is_valid(pericom_data->cc1_pwr_gpio)) {
        pr_info("%s: cc1_pwr_gpio=%d\n",__func__, pericom_data->cc1_pwr_gpio);
        ret = gpio_request(pericom_data->cc1_pwr_gpio, "cc1_pwr_gpio");
        if(ret) {
            pr_info("cc1_pwr_gpio gpio_request failed!\n");
            goto err_cc_pwr_gpio;
        }
    } else {
        pr_info("%s: cc1_pwr_gpio failed!\n" , __func__);
        goto err_cc_pwr_gpio;
    }

    pericom_data->cc2_pwr_gpio =
        of_get_named_gpio_flags(client->dev.of_node,
        "cc2_pwr_gpio", 0, NULL);
    if (gpio_is_valid(pericom_data->cc2_pwr_gpio)) {
        pr_info("%s: cc2_pwr_gpio=%d\n",__func__, pericom_data->cc2_pwr_gpio);
        ret = gpio_request(pericom_data->cc2_pwr_gpio, "cc2_pwr_gpio");
        if(ret) {
            pr_info("cc2_pwr_gpio gpio_request failed!\n");
            goto err_cc_pwr_gpio;
        }
    } else {
        pr_info("%s: cc2_pwr_gpio failed!\n" , __func__);
        goto err_cc_pwr_gpio;
    }

    if (pericom_data->cc1_pwr_gpio) {
        ret = gpio_direction_output(pericom_data->cc1_pwr_gpio, 0);
        if (ret < 0) {
            pr_err("%s : not able to set cc1_pwr_gpio as output\n",
                 __func__);
            goto err_cc_pwr_gpio;
        }
    }

    if (pericom_data->cc2_pwr_gpio) {
        ret = gpio_direction_output(pericom_data->cc2_pwr_gpio, 0);
        if (ret < 0) {
            pr_err("%s : not able to set cc2_pwr_gpio as output\n",
                 __func__);
            goto err_cc_pwr_gpio;
        }
    }

    /* init switch_gpio state */
    gpio_set_value(pericom_data->cc1_pwr_gpio, 0);
    gpio_set_value(pericom_data->cc2_pwr_gpio, 0);

	//mutex_init(&(pericom_data->i2c_rw_mutex));
	//pericom_data->i2c_client = client;
	mutex_init(&typec_headset_lock);
	pericom_data->irq = client->irq;
    INIT_WORK(&pericom_data->check_work, pericom_30216c_irq_serice);
	i2c_set_clientdata(client, pericom_data);

    pericom_30216c_set_powersaving_mode(pericom_data);          //Reset 30216C
    pr_info("pericom irq:%d,%d\n", pericom_data->irq, gpio_to_irq(pericom_data->irq_gpio));
    msleep(30);

	/* default drp mode and active power mode */
	pericom_30216c_set_trysnk_drp_mode(pericom_data);			//initial 30216C and set trysnk drp mode to 30216c
	pericom_30216c_set_poweractive_mode(pericom_data);
	/* interrupt */
	ret = request_threaded_irq(pericom_data->irq, NULL,
		pericom_30216c_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		DRIVER_NAME, pericom_data);

	if (ret < 0) {
		dev_err(&client->dev,
				"%s: Failed to create irq thread\n",
				__func__);
		goto err_absent;
	}

	/*register typec class*/
	if (ret < 0)
		goto err_fs;
	//enable_irq(pericom_data->irq);
	//global_pericom_data = *pericom_data;

    cclogic_set_audio_mode_register(pericom_cclogic_set_audio_mode);
    cclogic_updata_port_state(0);/*"cc_state: none"*/
    cclogic_updata_port_polarity(0); /* no typec usb connected*/

	return ret;

err_fs:
	free_irq(pericom_data->irq,pericom_data);
err_absent:
	devm_kfree(&client->dev,pericom_data);
err_switch_gpio:
    devm_pinctrl_put(pericom_data->pinctrl_switch);

    if (pericom_data->switch_gpio1) {
        gpio_free(pericom_data->switch_gpio1);
    }

    if (pericom_data->switch_gpio2) {
        gpio_free(pericom_data->switch_gpio2);
    }
err_cc_pwr_gpio:
    devm_pinctrl_put(pericom_data->pinctrl_ccpwr);

    if (pericom_data->cc1_pwr_gpio) {
        gpio_free(pericom_data->cc1_pwr_gpio);
    }

    if (pericom_data->cc2_pwr_gpio) {
        gpio_free(pericom_data->cc2_pwr_gpio);
    }
err_irq_gpio:
    devm_pinctrl_put(pericom_data->pinctrl_int);
    if (pericom_data->irq_gpio) {
        gpio_free(pericom_data->irq_gpio);
    }
err_chip_absent:
    //kfree(pericom_data);
    printk("%s %d: failed! -\n", __func__, __LINE__);

	return ret;
}

static int pericom_30216c_remove(struct i2c_client *client)
{
	//enter power saving mode
	struct pericom_30216c_data *pericom_data = i2c_get_clientdata(client);

	pericom_30216c_set_powersaving_mode(pericom_data);
	return 0;
}

static void pericom_30216c_shutdown(struct i2c_client *client)
{
	//enter power saving mode
	struct pericom_30216c_data *pericom_data = i2c_get_clientdata(client);

	pericom_30216c_set_powersaving_mode(pericom_data);
}

static const struct i2c_device_id pericom_30216c_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, pericom_30216c_id_table);

static struct of_device_id pericom_match_table[] = {
	{ .compatible = "cclogic_dev",},
	{ },
};

static struct i2c_driver pericom_30216c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = pericom_match_table,
#ifdef CONFIG_PM_SLEEP
		.pm = &pericom_pm_ops,
#endif
	},
	.probe = pericom_30216c_probe,
	.remove = pericom_30216c_remove,
	.shutdown = pericom_30216c_shutdown,
	.id_table = pericom_30216c_id_table,
};

static int __init pericom_30216c_init(void)
{
	return i2c_add_driver(&pericom_30216c_driver);
}

static void __exit pericom_30216c_exit(void)
{
	i2c_del_driver(&pericom_30216c_driver);
}

module_init(pericom_30216c_init);
module_exit(pericom_30216c_exit);

MODULE_AUTHOR("Pericom, Inc.");
MODULE_DESCRIPTION("Pericom 30216C I2C  Driver");
MODULE_LICENSE("GPL v2");
