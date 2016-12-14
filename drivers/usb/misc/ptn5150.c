/*
 * Copyright (C) 2016 Zeusis.
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc.
 *
 */
#define DEBUG
#define pr_fmt(fmt) "[%s:%d] " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#endif
#include <linux/ptn5150.h>

#ifdef DEBUG
#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) \
            printk(pr_fmt(fmt), ##__VA_ARGS__)
#else
#define pr_debug(fmt, ...) \
            printk(pr_fmt(fmt), ##__VA_ARGS__)
#endif
#endif

//#define HD_SWITCH_DBG (1)
#define PTN5150_VID (0x0B)

static struct ptn5150_dev *g_ptn_dev;
static int Try_Sink_State = Try_Sink_Idle_DRP;

//extern bool letv_typec_plug_state;
//static bool cyccg_usb_audio_insert = false;
#ifdef HD_SWITCH_DBG
static int switch_cnt = 0;
#endif
static bool ptn5150_analog_headset_plugin = false;
static bool i2c_op_fail = false;
static struct mutex typec_headset_lock;
static struct mutex ptn5150_i2c_lock;
extern bool typec_set_cc_state;
extern void cclogic_updata_port_state(int state);
extern void cclogic_updata_port_polarity(int polarity);
extern int msm_usb_vbus_set(void *_mdwc, bool on, bool ext_call);
extern int wcd_mbhc_plug_detect(void);
extern void cclogic_set_audio_mode_register(void (*func)(bool));
extern int cclogic_set_vbus(bool on);
extern void cclogic_set_typec_headset_with_analog(int val);
extern bool serial_hw_output_enable(void);
static bool always_uart_debug = false;

struct ptn5150_dev	{
	struct delayed_work trysink_check_work1;
    struct delayed_work trysink_check_work2;
	struct delayed_work check_work;
	struct i2c_client	*client;
	struct pinctrl *pinctrl_int;
	struct pinctrl_state *intr_active;
	struct pinctrl *pinctrl_switch;
	struct pinctrl_state *switch_active;
	struct pinctrl *pinctrl_ccpwr;
	struct pinctrl_state *ccpwr_active;
	struct miscdevice	ptn5150_device;
	unsigned int		irq_gpio;
	unsigned int 		switch_gpio1;
	unsigned int 		switch_gpio2;
	unsigned int 		cc1_pwr_gpio;
	unsigned int 		cc2_pwr_gpio;
	int irq;
	//bool				irq_enabled;
	//bool				id_irq_enabled;
	//spinlock_t			irq_enabled_lock;
};

extern int pi5usb_set_msm_usb_host_mode(bool mode);

static int ptn5150_i2c_rxdata(struct i2c_client *client, char *rxdata, int length)
{
	int ret = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata + 1,
		},
	};

	if (i2c_transfer(client->adapter, msgs, 2) != 2) {
		ret = -EIO;
        pr_err("%s: i2c read error: %d\n", __func__, ret);
	}

	return ret;
}

static int ptn5150_i2c_txdata(struct i2c_client *client, char *txdata, int length)
{
	int ret = 0;

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	if (i2c_transfer(client->adapter, msg, 1) != 1) {
		ret = -EIO;
        pr_err("%s: i2c write error: %d\n", __func__, ret);
	}

	return ret;
}

static int ptn5150_write_reg(struct i2c_client *client, u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

    mutex_lock(&ptn5150_i2c_lock);

	buf[0] = addr;
	buf[1] = para;
	ret = ptn5150_i2c_txdata(client, buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		i2c_op_fail = true;
		mutex_unlock(&ptn5150_i2c_lock);
		return -1;
	}

    mutex_unlock(&ptn5150_i2c_lock);

	return 0;
}

static int ptn5150_read_reg(struct i2c_client *client, u8 addr, u8 *pdata)
{
	int ret = -1;
	u8 buf[2] = {addr, 0};

    mutex_lock(&ptn5150_i2c_lock);

	ret = ptn5150_i2c_rxdata(client, buf, 1);
	if (ret < 0) {
		pr_err("read reg failed! %#x ret: %d", buf[0], ret);
		i2c_op_fail = true;
		mutex_unlock(&ptn5150_i2c_lock);
		return -1;
	}
	*pdata = buf[1];

    mutex_unlock(&ptn5150_i2c_lock);

	return 0;
}

/*This func is for headset switch AD mode*/
void ptn5150_cclogic_set_audio_mode(bool mode)
{
	/*true for Analog mode and false for Digit mode*/
	//struct cyccg_platform_data *pdata = cyccg_pdata;
	struct ptn5150_dev *ptn5150_dev = g_ptn_dev;
	unsigned char reg_val;

	#ifdef HD_SWITCH_DBG
	switch_cnt++;
	#endif
	mutex_lock(&typec_headset_lock);
    #ifdef HD_SWITCH_DBG
    pr_info("ptn5150 switch, c:%d, m:%d\n", switch_cnt, mode);
    #endif

	if(true == always_uart_debug)
	{
		pr_err("Err: Serial HW-Output Enable!\n");
	}

	if (mode) {
		pr_info("ptn5150 switch to audio mode\n");
        msleep(50);
		pi5usb_set_msm_usb_host_mode(false);/*simulate plugout*/

		gpio_set_value(ptn5150_dev->switch_gpio2, 1);/*Headphone*/
		gpio_set_value(ptn5150_dev->switch_gpio1, 0);/*MIC*/
		ptn5150_read_reg(ptn5150_dev->client, PTN5150_CC_STATUS_REG, &reg_val);
        if (0x1 == (reg_val&0x3)) {
            gpio_set_value(ptn5150_dev->cc2_pwr_gpio, 1);
		} else if (0x2 == (reg_val&0x3)) {
            gpio_set_value(ptn5150_dev->cc1_pwr_gpio, 1);
		} else {
            pr_err("ptn5150 err:%x\n", reg_val);
		}

		//cyccg_pdata->mode = AUDIO_MODE;
		//if (!letv_typec_plug_state)
			//letv_typec_plug_state = true;
		//notify audio module for headset plug in
		if (!ptn5150_analog_headset_plugin) {
			pr_info("audio headset plug in!!\n");
			wcd_mbhc_plug_detect();
			ptn5150_analog_headset_plugin = true;
		}
		cclogic_set_typec_headset_with_analog(1);
		//cyccg_usb_audio_insert = true;
		/*The behavior above */
		cclogic_updata_port_state(3);/*"cc_state: audio"*/
	} else {
        pr_info("ptn5150 switch to digital\n");
        gpio_set_value(ptn5150_dev->switch_gpio2, 0);/*USB*/
		gpio_set_value(ptn5150_dev->switch_gpio1, 0);/*MIC*/

        msleep(30);
		pi5usb_set_msm_usb_host_mode(true);
		gpio_set_value(ptn5150_dev->cc1_pwr_gpio, 0);
		gpio_set_value(ptn5150_dev->cc2_pwr_gpio, 0);
		//pr_info("ptn5150 set usb to host mode\n");
		//pdata->mode = DFP_MODE;
		//notify audio module for headset plug out
		if (ptn5150_analog_headset_plugin) {
			pr_info("audio headset plug out!!\n");
			wcd_mbhc_plug_detect();
			ptn5150_analog_headset_plugin = false;
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
EXPORT_SYMBOL(ptn5150_cclogic_set_audio_mode);

int ptn5150_is_present(struct i2c_client *client)
{
    int ret = 0;
    unsigned char reg_val;

    ret = ptn5150_read_reg(client, PTN5150_VID_REG, &reg_val);
    pr_debug("ptn5150 reg[0x1]=%x, %d\n", reg_val, ret);
    if(0 != ret)
    {
        return -ENODEV;
    }

    if(PTN5150_VID != reg_val)
    {
        return -ENODEV;
    }

    return ret;
}

void ptn5150_hw_preinit(struct i2c_client *client)
{
	unsigned char reg_val0, reg_val1, reg_val2;

	reg_val0=0x40;
	reg_val1=0x34;
	reg_val2=0x11;
	ptn5150_write_reg(client, PTN5150_SPECIAL_43H_REG, reg_val0);
	ptn5150_write_reg(client, PTN5150_SPECIAL_4CH_REG, reg_val1);
	ptn5150_write_reg(client, PTN5150_CONTROL_REG, reg_val2);
}

int ptn5150_hw_init(struct i2c_client *client)
{
	int ret = 0;
    unsigned char reg_val0,reg_val1,reg_val2,reg_val3,reg_val;

	reg_val0=0x24;//aftung3: 0x34
	reg_val1=0xc0;//aftung3: 0x40
	reg_val2=0x15;
	reg_val3=0x00;//aftung3, as suggested, change to enable all interrupt
	//ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x4c, &reg_val0 );
	ptn5150_write_reg(client, PTN5150_SPECIAL_4CH_REG, reg_val0);
	//ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x43, &reg_val1);
	ptn5150_write_reg(client, PTN5150_SPECIAL_43H_REG, reg_val1);
	//ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x02, &reg_val2);
	ptn5150_write_reg(client, PTN5150_CONTROL_REG, reg_val2);
	//ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x18, &reg_val3);
	ptn5150_write_reg(client, PTN5150_INT_MASK_REG, reg_val3);
	//ret=ptn5150_i2c_read(typec_ptn5150->i2c_hd,&reg_val,0x4c);
	ptn5150_read_reg(client, PTN5150_SPECIAL_4CH_REG, &reg_val);
	pr_debug("ptn5150 reg [0x4c]=0x24 =%x\n", reg_val);//aftung3
	//ret=ptn5150_i2c_read(typec_ptn5150->i2c_hd,&reg_val,0x43);
	ptn5150_read_reg(client, PTN5150_SPECIAL_43H_REG, &reg_val);
	pr_debug("ptn5150 reg [0x43]=0xc0 =%x\n", reg_val);//aftung3
	//ret=ptn5150_i2c_read(typec_ptn5150->i2c_hd,&reg_val,0x02);
    ptn5150_read_reg(client, PTN5150_CONTROL_REG, &reg_val);
	pr_debug("ptn5150 reg [0x02]=0x05 =%x\n", reg_val);
	//ret=ptn5150_i2c_read(typec_ptn5150->i2c_hd,&reg_val,0x18);
	ptn5150_read_reg(client, PTN5150_INT_MASK_REG, &reg_val);
	pr_debug("ptn5150 reg [0x18]=0x00 =%x\n", reg_val);
    //ptn5150_read_reg(client, PTN5150_INT_STATUS_REL_REG, &reg_val);
	//pr_debug("ptn5150 reg [0x19]=%x\n", reg_val);

	return ret;
}

static void ptn5150_dump_regs(struct i2c_client *client)
{
	unsigned char i = 0, data = 0;

	for(i = 0; i <= 0xa; i++){
		ptn5150_read_reg(client, i, &data);
		pr_debug("reg[0x%x] = 0x%02x.\n", i, data & 0xff);
	}

	for(i = 0x10; i <= 0x19; i++){
		ptn5150_read_reg(client, i, &data);
		pr_debug("reg[0x%x] = 0x%02x.\n", i, data & 0xff);
	}
}

#if defined(PTN5150_DEBUG)
/* sysfs interface */
static ssize_t ptn5150_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	ptn5150_dump_regs(client);
	return sprintf(buf,"%s\n","OK");
}

/* debug fs, "echo @1 > /sys/bus/i2c/devices/xxx/ptn5150_debug" @1:debug_flag  */
static ssize_t ptn5150_store(struct device *dev,
        struct device_attribute *attr, const char *buf,size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char data = 0x0;

	/*clear interrupt register test*/
	ptn5150_read_reg(client, 0x03, &data);
	pr_debug("After clear interrupt REG[0x03] = 0x%02x.\n", data);

	return count;
}

static struct device_attribute ptn5150_dev_attr =
	__ATTR(ptn5150_debug, S_IRUGO | S_IWUGO, ptn5150_show, ptn5150_store);
#endif

#ifdef CONFIG_PM_SLEEP
static int ptn5150_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct ptn5150_dev *ptn5150_dev = g_ptn_dev;

	enable_irq_wake(ptn5150_dev->irq);

	return ret;
}

static int ptn5150_pm_resume(struct device *dev)
{
	int ret = 0;

	struct ptn5150_dev *ptn5150_dev = g_ptn_dev;

	disable_irq_wake(ptn5150_dev->irq);
	return ret;
}

static const struct dev_pm_ops ptn5150_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ptn5150_pm_suspend, ptn5150_pm_resume)
};
#endif /* CONFIG_PM_SLEEP */


static void ptn5150_trysink_work1(struct work_struct *work)
{
    unsigned char Register_Value;
    static int rp_miss_count = 0;
    unsigned char data = 0x13, reg;

	struct ptn5150_dev *ptn5150_dev = g_ptn_dev;
	if (Try_Sink_State == Try_Sink_Attached_Wait_Src_Detached)	// If timer expires
	{
		// Check if there is any Rp connected externally
		ptn5150_read_reg(ptn5150_dev->client, PTN5150_SPECIAL_49H_REG, &Register_Value);
        pr_debug("%s ptn5150 reg 0x49 value = 0x%x\n", __func__, Register_Value);
		Register_Value &= 0x14;
		pr_debug("%s ptn5150 reg 0x49 value&0x14 = 0x%x\n", __func__, Register_Value);
        if ((Register_Value == 0x04) || (Register_Value == 0x10))
		{
			rp_miss_count = 0;
			schedule_delayed_work(&ptn5150_dev->trysink_check_work1, msecs_to_jiffies(10));
		}
		else if ((Register_Value == 0x14) || (Register_Value == 0x00))
		{
			rp_miss_count++;
			pr_debug("ptn5150 rp_miss_count = %d\n", rp_miss_count);
			if(rp_miss_count >= 3)
			{
				rp_miss_count = 0;
				pr_debug("ptn5150 Try_Sink_State = %d, will do DRP mode\n", Try_Sink_State);
				ptn5150_read_reg(ptn5150_dev->client, PTN5150_SPECIAL_4AH_REG, &reg);
				pr_debug("%s ptn5150 reg 0x4A value = 0x%x\n", __func__, reg);
				Try_Sink_State = Try_Sink_tDRPTry_Expire;
				ptn5150_write_reg(ptn5150_dev->client, PTN5150_CONTROL_REG, data);
				Try_Sink_State = Try_Sink_Try_Wait_Src;
				schedule_delayed_work(&ptn5150_dev->trysink_check_work2, msecs_to_jiffies(200));
			}
			else
			{
				pr_debug("ptn5150 rp_miss_count = %d schedule trysnd work1\n", rp_miss_count);
				schedule_delayed_work(&ptn5150_dev->trysink_check_work1, msecs_to_jiffies(10));
			}
		}
	}
	pr_debug("Try_Sink_State = %d.\n", Try_Sink_State);
}

static void ptn5150_trysink_work2(struct work_struct *work)
{
    unsigned char reg=0x15;
	unsigned char reg_val0=0x24;//aftung3, restore to default enable accessory detect
	unsigned char reg_val1=0xc0;//aftung3, restore to default
	struct ptn5150_dev *ptn5150_dev = g_ptn_dev;

	pr_debug("%s ptn5150 Try_Sink_State =%d\n", __func__, Try_Sink_State);

	if (Try_Sink_State == Try_Sink_Try_Wait_Src)
	{
		Try_Sink_State = Try_Sink_Try_Wait_Src_Expire;
		//ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x4c, &reg_val0 );	//aftung3
		ptn5150_write_reg(ptn5150_dev->client, PTN5150_SPECIAL_4CH_REG, reg_val0);
		//ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x43, &reg_val1);		//aftung3
		ptn5150_write_reg(ptn5150_dev->client, PTN5150_SPECIAL_43H_REG, reg_val1);

		//ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x02, &reg);
        ptn5150_write_reg(ptn5150_dev->client, PTN5150_CONTROL_REG, reg);
		Try_Sink_State = Try_Sink_Idle_DRP;
	}
}

static irqreturn_t ptn5150_irq_handler(int irq, void *data)
{
	struct ptn5150_dev *ptn5150_dev = data;

    pr_debug("ptn5150 handle irq\n");

    schedule_delayed_work(&ptn5150_dev->check_work, 0);

	return IRQ_HANDLED;
}


static void ptn5150_status_check(struct work_struct *work)
{
    unsigned char Register_Value = 0;
    int attachment_status;
    unsigned char reg1 = 0x15, reg2 = 0x11;
    unsigned char reg_val0 = 0x24;//aftung3
    unsigned char reg_val1 = 0xc0;//aftung3
    unsigned char Reg_Data_tmp;
    struct ptn5150_dev *ptn5150_dev = g_ptn_dev;

    pr_debug("ptn5150_status_check->NXP\n");
    //ptn5150_i2c_read(g_exttypec_ptn5150->i2c_hd, &Register_Value, 0x19);
    ptn5150_read_reg(ptn5150_dev->client, PTN5150_INT_STATUS_REL_REG, &Register_Value);
    pr_debug("ptn5150 0x19 register value = 0x%x\n", Register_Value);

    if (Register_Value & 0x8)//role changed interupt
    {
        //ptn5150_i2c_read(g_exttypec_ptn5150->i2c_hd, &Register_Value, 0x04);
        ptn5150_read_reg(ptn5150_dev->client, PTN5150_CC_STATUS_REG, &Register_Value);
        attachment_status = ((Register_Value >> 2) & 0x07);
        pr_debug("ptn5150 0x04 register value = 0x%x, attachment_status=%d, Try_Sink_State=%d\n", Register_Value, attachment_status, Try_Sink_State);
        switch (attachment_status)
        {
        case CABLE_DISCONNECT:
            // If Cable Disconnected
            if (Try_Sink_State == Try_Sink_Source_To_Sink)
            {
                //schedule_delayed_work(&g_exttypec_ptn5150->trysnk_work1, msecs_to_jiffies(100));
                schedule_delayed_work(&ptn5150_dev->trysink_check_work1, msecs_to_jiffies(100));
                Try_Sink_State = Try_Sink_Attached_Wait_Src_Detached;
            }
            // aftung simplify
            //if ((Try_Sink_State == Try_Sink_Attached_As_DFP) || (Try_Sink_State == Try_Sink_Attached_As_UFP)||(Try_Sink_State == No_Trysink_Analog_In))
            else // this will including any kind of disconnect, when disconnect, we go back to DRP, aind state=try sink idle drp
            {
                reg_val0=0x24;//aftung3, restore to default enable accessory detect
                reg_val1=0xc0;//aftung3, restore to default
                //ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x4c, &reg_val0 );    //aftung3
                ptn5150_write_reg(ptn5150_dev->client, PTN5150_SPECIAL_4CH_REG, reg_val0);
                //ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x43, &reg_val1);     //aftung3
                ptn5150_write_reg(ptn5150_dev->client, PTN5150_SPECIAL_43H_REG, reg_val1);

                //ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x02, &reg1);
                ptn5150_write_reg(ptn5150_dev->client, PTN5150_CONTROL_REG, reg1);

                /*DRP default switch to USB + UART channel*/
				gpio_set_value(ptn5150_dev->switch_gpio2, 0);/*USB*/
				if(true == always_uart_debug)
				{
					gpio_set_value(ptn5150_dev->switch_gpio1, 1);/*UART*/
				}
				else
				{
					gpio_set_value(ptn5150_dev->switch_gpio1, 0);/*close UART > MIC*/
				}

                // Force to run in DRP mode
                Try_Sink_State = Try_Sink_Idle_DRP;
                //ptn5150_analog_headset_plugin = false;
				cclogic_set_vbus(0);

				gpio_set_value(ptn5150_dev->cc1_pwr_gpio, 0);
				gpio_set_value(ptn5150_dev->cc2_pwr_gpio, 0);
				cclogic_set_typec_headset_with_analog(-1);
				if (ptn5150_analog_headset_plugin) {
					ptn5150_analog_headset_plugin = false;
					pr_debug("%s, audio headset plug out!!\n", __func__);
					wcd_mbhc_plug_detect();
				}

				pi5usb_set_msm_usb_host_mode(false);/*if not simulate disconnect, open*/
                cclogic_updata_port_state(0);/*"cc_state: none"*/

                if (typec_set_cc_state) {
                    pr_info("%s: clear typec_set_cc_state!\n", __func__);
                    typec_set_cc_state = false;
                }

                pr_debug("ptn5150 detect cable removal\n");
            }
            break;
        case DFP_ATTACHED:
            // if DFP is attached
            if (Try_Sink_State == Try_Sink_Attached_Wait_Src_Detached)
            {
                //cancel_delayed_work(&g_exttypec_ptn5150->trysnk_work1);
                cancel_delayed_work(&ptn5150_dev->trysink_check_work1);
                Try_Sink_State = Try_Sink_Attached_As_UFP;
            }
            else
            {
                Try_Sink_State = Try_Sink_Attached_As_UFP;
            }
            // aftung: I move the following here, since this needs to be done when we are UFP

			gpio_set_value(ptn5150_dev->switch_gpio1, 1);/*UART*/
            cclogic_updata_port_state(1);/*"cc_state: ufp"*/
            cclogic_set_typec_headset_with_analog(0);
            pr_debug("DFP is attached!\n");
            //ptn5150_trigger_driver(g_exttypec_ptn5150, DEVICE_TYPE,ENABLE);
            break;
        case UFP_ATTACHED:
            // if UFP is attached
            if (Try_Sink_State == Try_Sink_Idle_DRP)
            // in this state, you don't want to enable VBus output, instead, you want to first try sink
            {
                Try_Sink_State = Try_Sink_Attached_Wait_Src;
                reg_val0=0x34;//aftung3, try sink setting, disable accessory detect
                reg_val1=0x40;//aftung3: 0x40
                //ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x4c, &reg_val0 );    //aftung3
                ptn5150_write_reg(ptn5150_dev->client, PTN5150_SPECIAL_4CH_REG, reg_val0);
                //ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x43, &reg_val1);     //aftung3
                ptn5150_write_reg(ptn5150_dev->client, PTN5150_SPECIAL_43H_REG, reg_val1);

                //ptn5150_i2c_write(g_exttypec_ptn5150->i2c_hd, 0x02, &reg2);
                ptn5150_write_reg(ptn5150_dev->client, PTN5150_CONTROL_REG, reg2);
                // Force to run in UFP mode
                Try_Sink_State = Try_Sink_Source_To_Sink;
            }
            else if (Try_Sink_State == Try_Sink_Try_Wait_Src)
            {
                //cancel_delayed_work(&g_exttypec_ptn5150->trysnk_work2);
                cancel_delayed_work(&ptn5150_dev->trysink_check_work2);
                Try_Sink_State = Try_Sink_Attached_As_DFP;
                pr_debug("UFP is attached!\n");
                gpio_set_value(ptn5150_dev->switch_gpio1, 0);/*MIC*/
                cclogic_set_vbus(1);
                pi5usb_set_msm_usb_host_mode(true);
                cclogic_updata_port_state(2);/*"cc_state: dfp"*/
                //check cc orientation
                ptn5150_read_reg(ptn5150_dev->client, PTN5150_CC_STATUS_REG, &Register_Value);
                if (0x1 == (Register_Value&0x3)) {
                    cclogic_updata_port_polarity(1);/* CC1 connected*/
                    pr_info("ptn5150 cclogic port polarity:%x\n", (Register_Value&0x3));
                } else if (0x2 == (Register_Value&0x3)) {
                    cclogic_updata_port_polarity(2);/* CC2 connected*/
                    pr_info("ptn5150 cclogic port polarity:%x\n", (Register_Value&0x3));
                } else {
                    pr_err("ptn5150 err:%x\n", Register_Value);
                }
            }
            break;
        case ANLOG_AUDIO_MODE: // aftung: analog audio accessory//
                // in this case, we don't need to look at the try sink states., just go ahead and handle audio
            pr_debug("audio Attached to an accessory!\n");

            /*switch GPIO to select audio channel*/
			gpio_set_value(ptn5150_dev->switch_gpio2, 1);/*Headphone*/
			gpio_set_value(ptn5150_dev->switch_gpio1, 0);/*MIC*/
			/*switch USB drive stack to Host mode*/
			ptn5150_read_reg(ptn5150_dev->client, 0x19, &Reg_Data_tmp);
			pr_debug("reg[0x19] = 0x%02x.\n", Reg_Data_tmp);
			if(Reg_Data_tmp != 0)
				schedule_delayed_work(&ptn5150_dev->check_work, 0);

			//pi5usb_set_msm_usb_host_mode(true);
			//ptn5150_set_mode(ptn5150_dev->client, PORT_SET_DFP);
			cclogic_updata_port_state(3);/*"cc_state: audio"*/
			//notify audio module for headset plug in
			if (!ptn5150_analog_headset_plugin) {
				pr_debug("%s, audio headset plug in!!\n", __func__);
				msleep(150);
				wcd_mbhc_plug_detect();
				ptn5150_analog_headset_plugin = true;
			}
            break;
        default:
            pr_debug("Something plug in, but not processed. Interrupt register 0x19=%x\n", Register_Value);
            break;

        }
    }
    // aftung: remove the following.  In general, we only enable "role change" interrupt, this will take care of the audio accessory also (case 3 above)
}

static ssize_t ptn5150_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	int ret = 0;

	return ret;
}

static ssize_t ptn5150_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	int ret = 0;

	return ret;
}

static int ptn5150_dev_open(struct inode *inode, struct file *filp)
{
	struct ptn5150_dev *ptn5150_dev = container_of(filp->private_data,
						struct ptn5150_dev,
						ptn5150_device);

	filp->private_data = ptn5150_dev;

	pr_debug("%d, %d\n", imajor(inode), iminor(inode));

	return 0;
}

static long  ptn5150_dev_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	return 0;
}

static const struct file_operations ptn5150_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= ptn5150_dev_read,
	.write	= ptn5150_dev_write,
	.open	= ptn5150_dev_open,
	.unlocked_ioctl  = ptn5150_dev_ioctl,
};

static int ptn5150_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct ptn5150_dev *ptn5150_dev;
	int ret = 0;

	pr_debug("ptn5150 probe start!\n");

	ptn5150_dev = kzalloc(sizeof(struct ptn5150_dev), GFP_KERNEL);
	if (ptn5150_dev == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_allocate_mem;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		goto  err_allocate_mem;
	}

    mutex_init(&ptn5150_i2c_lock);

    ret = ptn5150_is_present(client);
    if (0 != ret) {
        goto err_allocate_mem;
	}

	if(true == serial_hw_output_enable())
	{
		always_uart_debug = true;
		pr_err("Serial HW-Output Enable!!!!!!\n");
	}

    //Interrupt pin configurate
	ptn5150_dev->pinctrl_int = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(ptn5150_dev->pinctrl_int)) {
		pr_debug("ptn5150 %s: Unable to get pinctrl handle\n", __func__);
	}
	ptn5150_dev->intr_active = pinctrl_lookup_state(ptn5150_dev->pinctrl_int,
					"m0_ccint_active");
	if (IS_ERR(ptn5150_dev->intr_active)) {
		pr_debug("ptn5150 %s: could not get intr_active pinstate\n", __func__);
        goto err_irq_gpio;
	}

	ret = pinctrl_select_state(ptn5150_dev->pinctrl_int,
					ptn5150_dev->intr_active);
	if (ret != 0) {
		pr_debug("ptn5150 %s: Disable TLMM pins failed with %d\n",
			__func__, ret);
	}

	ptn5150_dev->irq_gpio =
		of_get_named_gpio_flags(client->dev.of_node,
		"irq-gpio", 0, NULL);
	if (gpio_is_valid(ptn5150_dev->irq_gpio)) {
		pr_debug("%s:irq gpio=%d\n", __func__, ptn5150_dev->irq_gpio);
		ret = gpio_request(ptn5150_dev->irq_gpio, "irq-gpio");
		if(ret) {
			pr_debug("typec_irq gpio_request failed!\n");
			goto err_irq_gpio;
		}
	} else {
		pr_debug("%s : irq_gpio failed!\n" , __func__);
		goto err_irq_gpio;
	}
	gpio_direction_input(ptn5150_dev->irq_gpio);


    //Switch pin configurate
	ptn5150_dev->pinctrl_switch = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(ptn5150_dev->pinctrl_switch)) {
		pr_debug("ptn5150 %s: Unable to get pinctrl_switch\n", __func__);
	}
	ptn5150_dev->switch_active = pinctrl_lookup_state(ptn5150_dev->pinctrl_switch,
					"m0_ccswitch_active");
	if (IS_ERR(ptn5150_dev->switch_active)) {
		pr_debug("ptn5150 %s: could not get switch_active pinstate\n", __func__);
        goto err_switch_gpio;
	}

	ret = pinctrl_select_state(ptn5150_dev->pinctrl_switch,
					ptn5150_dev->switch_active);
	if (ret != 0) {
		pr_debug("ptn5150 %s: Disable switch pins failed with %d\n",
			__func__, ret);
	}

	ptn5150_dev->switch_gpio1 =
		of_get_named_gpio_flags(client->dev.of_node,
		"switch_gpio1", 0, NULL);
	if (gpio_is_valid(ptn5150_dev->switch_gpio1)) {
		pr_debug("%s:switch_gpio1=%d\n",__func__, ptn5150_dev->switch_gpio1);
		ret = gpio_request(ptn5150_dev->switch_gpio1, "switch_gpio1");
		if(ret) {
			pr_debug("switch_gpio1 gpio_request failed!\n");
			goto err_switch_gpio;
		}
	} else {
		pr_debug("%s : switch_gpio1 failed!\n" , __func__);
		goto err_switch_gpio;
	}

	ptn5150_dev->switch_gpio2 =
		of_get_named_gpio_flags(client->dev.of_node,
		"switch_gpio2", 0, NULL);
	if (gpio_is_valid(ptn5150_dev->switch_gpio2)) {
		pr_debug("%s:switch_gpio2=%d\n",__func__, ptn5150_dev->switch_gpio2);
		ret = gpio_request(ptn5150_dev->switch_gpio2, "switch_gpio2");
		if(ret) {
			pr_debug("switch_gpio2 gpio_request failed!\n");
			goto err_switch_gpio;
		}
	} else {
		pr_debug("%s : switch_gpio2 failed!\n" , __func__);
		goto err_switch_gpio;
	}

	if (ptn5150_dev->switch_gpio1) {
		ret = gpio_direction_output(ptn5150_dev->switch_gpio1, 0);
		if (ret < 0) {
			pr_err("%s : not able to set switch_gpio1 as output\n",
				 __func__);
			goto err_switch_gpio;
		}
	}

	if (ptn5150_dev->switch_gpio2) {
		ret = gpio_direction_output(ptn5150_dev->switch_gpio2, 0);
		if (ret < 0) {
			pr_err("%s : not able to set switch_gpio2 as output\n",
				 __func__);
			goto err_switch_gpio;
		}
	}

	/* init switch_gpio state */
	gpio_set_value(ptn5150_dev->switch_gpio2, 0);/*USB*/  //dfl:0
	if(true == always_uart_debug)
	{
		gpio_set_value(ptn5150_dev->switch_gpio1, 1);/*UART*/
	}
	else
	{
		gpio_set_value(ptn5150_dev->switch_gpio1, 0);
	}

    //CC Power pin configurate
	ptn5150_dev->pinctrl_ccpwr = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(ptn5150_dev->pinctrl_ccpwr)) {
		pr_debug("ptn5150 %s: Unable to get pinctrl_ccpwr\n", __func__);
	}
	ptn5150_dev->ccpwr_active = pinctrl_lookup_state(ptn5150_dev->pinctrl_ccpwr,
					"m0_ccpwr_active");
	if (IS_ERR(ptn5150_dev->ccpwr_active)) {
		pr_debug("ptn5150 %s: could not get ccpwr_active pinstate\n", __func__);
        goto err_cc_pwr_gpio;
	}

	ret = pinctrl_select_state(ptn5150_dev->pinctrl_ccpwr,
					ptn5150_dev->ccpwr_active);
	if (ret != 0) {
		pr_debug("ptn5150 %s: Disable ccpwr pins failed with %d\n",
			__func__, ret);
	}

	ptn5150_dev->cc1_pwr_gpio =
		of_get_named_gpio_flags(client->dev.of_node,
		"cc1_pwr_gpio", 0, NULL);
	if (gpio_is_valid(ptn5150_dev->cc1_pwr_gpio)) {
		pr_debug("%s: cc1_pwr_gpio=%d\n",__func__, ptn5150_dev->cc1_pwr_gpio);
		ret = gpio_request(ptn5150_dev->cc1_pwr_gpio, "cc1_pwr_gpio");
		if(ret) {
			pr_debug("cc1_pwr_gpio gpio_request failed!\n");
			goto err_cc_pwr_gpio;
		}
	} else {
		pr_debug("%s: cc1_pwr_gpio failed!\n" , __func__);
		goto err_cc_pwr_gpio;
	}

	ptn5150_dev->cc2_pwr_gpio =
		of_get_named_gpio_flags(client->dev.of_node,
		"cc2_pwr_gpio", 0, NULL);
	if (gpio_is_valid(ptn5150_dev->cc2_pwr_gpio)) {
		pr_debug("%s: cc2_pwr_gpio=%d\n",__func__, ptn5150_dev->cc2_pwr_gpio);
		ret = gpio_request(ptn5150_dev->cc2_pwr_gpio, "cc2_pwr_gpio");
		if(ret) {
			pr_debug("cc2_pwr_gpio gpio_request failed!\n");
			goto err_cc_pwr_gpio;
		}
	} else {
		pr_debug("%s: cc2_pwr_gpio failed!\n" , __func__);
		goto err_cc_pwr_gpio;
	}

	if (ptn5150_dev->cc1_pwr_gpio) {
		ret = gpio_direction_output(ptn5150_dev->cc1_pwr_gpio, 0);
		if (ret < 0) {
			pr_err("%s : not able to set cc1_pwr_gpio as output\n",
				 __func__);
			goto err_cc_pwr_gpio;
		}
	}

	if (ptn5150_dev->cc2_pwr_gpio) {
		ret = gpio_direction_output(ptn5150_dev->cc2_pwr_gpio, 0);
		if (ret < 0) {
			pr_err("%s : not able to set cc2_pwr_gpio as output\n",
				 __func__);
			goto err_cc_pwr_gpio;
		}
	}

	/* init switch_gpio state */
	gpio_set_value(ptn5150_dev->cc1_pwr_gpio, 0);
	gpio_set_value(ptn5150_dev->cc2_pwr_gpio, 0);

	ptn5150_dev->client = client;

	INIT_DELAYED_WORK(&ptn5150_dev->check_work, ptn5150_status_check);
	INIT_DELAYED_WORK(&ptn5150_dev->trysink_check_work1, ptn5150_trysink_work1);
	INIT_DELAYED_WORK(&ptn5150_dev->trysink_check_work2, ptn5150_trysink_work2);

	/* init mutex */
	//spin_lock_init(&ptn5150_dev->irq_enabled_lock);
	//letv_typec_headset
	mutex_init(&typec_headset_lock);
	//letv_typec_headset
	//mutex_init(&ptn5150_i2c_lock);


	ptn5150_hw_preinit(client);
	ptn5150_dev->ptn5150_device.minor = MISC_DYNAMIC_MINOR;
	ptn5150_dev->ptn5150_device.name = "ptn5150";
	ptn5150_dev->ptn5150_device.fops = &ptn5150_dev_fops;
	ptn5150_dev->irq = gpio_to_irq(ptn5150_dev->irq_gpio);

	ret = misc_register(&ptn5150_dev->ptn5150_device);
	if (ret) {
		pr_debug("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_debug("requesting IRQ %d, %d, %d\n", client->irq, ptn5150_dev->irq, ptn5150_dev->irq_gpio);

	// reset IC for first accessory detect
	ptn5150_write_reg(client, 0x10, 0x1);
    msleep(50);

	ptn5150_hw_init(client);
//	ptn5150_dump_regs(client);

#if 0
	ret = request_threaded_irq(client->irq, NULL,\
		/*ptn5150_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,*/
		ptn5150_irq_handler, IRQF_TRIGGER_LOW | IRQF_ONESHOT,\
		client->name, ptn5150_dev);
#endif
	ret = request_threaded_irq(client->irq, NULL,\
		/*ptn5150_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,*/
		ptn5150_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,\
		client->name, ptn5150_dev);
	if(ret < 0) {
		dev_err(&client->dev, "Failed to request irq.\n");
		goto err_request_irq_failed;
	}

	//disable_irq(client->irq);

	i2c_set_clientdata(client, ptn5150_dev);

#if defined(PTN5150_DEBUG)
	ret = device_create_file(&client->dev, &ptn5150_dev_attr);
	if (ret) {
		pr_err("%s: sysfs registration failed, error %d \n", __func__, ret);
		goto err_create_dev_file;
	}
#endif

	g_ptn_dev = ptn5150_dev;
	cclogic_set_audio_mode_register(ptn5150_cclogic_set_audio_mode);
    schedule_delayed_work(&ptn5150_dev->check_work,1*HZ);
    cclogic_updata_port_state(0);/*"cc_state: none"*/

	pr_debug("ptn5150 probe ok!\n");

	return 0;

err_create_dev_file:
	free_irq(client->irq, ptn5150_dev);
err_request_irq_failed:
	misc_deregister(&ptn5150_dev->ptn5150_device);
    cancel_delayed_work_sync(&ptn5150_dev->check_work);
err_misc_register:

err_switch_gpio:
    devm_pinctrl_put(ptn5150_dev->pinctrl_switch);

	if (ptn5150_dev->switch_gpio1) {
		gpio_free(ptn5150_dev->switch_gpio1);
	}

	if (ptn5150_dev->switch_gpio2) {
		gpio_free(ptn5150_dev->switch_gpio2);
	}
err_cc_pwr_gpio:
    devm_pinctrl_put(ptn5150_dev->pinctrl_ccpwr);

	if (ptn5150_dev->cc1_pwr_gpio) {
		gpio_free(ptn5150_dev->cc1_pwr_gpio);
	}

	if (ptn5150_dev->cc2_pwr_gpio) {
		gpio_free(ptn5150_dev->cc2_pwr_gpio);
	}
err_irq_gpio:
    devm_pinctrl_put(ptn5150_dev->pinctrl_int);
    if (ptn5150_dev->irq_gpio) {
		gpio_free(ptn5150_dev->irq_gpio);
    }
err_allocate_mem:
    kfree(ptn5150_dev);
	pr_debug("%s %d: failed! -\n", __func__, __LINE__);
	return ret;
}

static int ptn5150_remove(struct i2c_client *client)
{
#if 1
	struct ptn5150_dev *ptn5150_dev;

	ptn5150_dev = i2c_get_clientdata(client);
	free_irq(client->irq, ptn5150_dev);
	misc_deregister(&ptn5150_dev->ptn5150_device);
	gpio_free(ptn5150_dev->irq_gpio);
	mutex_destroy(&typec_headset_lock);
	mutex_destroy(&ptn5150_i2c_lock);
#if defined(PTN5150_DEBUG)
	device_remove_file(&client->dev, &ptn5150_dev_attr);
#endif
	kfree(ptn5150_dev);
#endif

	return 0;
}

static const struct i2c_device_id ptn5150_id[] = {
	{ "ptn5150", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ptn5150_id);

#ifdef CONFIG_OF
static struct of_device_id ptn5150_match_table[] = {
    { .compatible = "cclogic_dev",},
    { },
};
#else
#define ptn5150_match_table NULL
#endif

static struct i2c_driver ptn5150_driver = {
	.id_table	= ptn5150_id,
	.probe		= ptn5150_probe,
	.remove		= ptn5150_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ptn5150",
#ifdef CONFIG_PM_SLEEP
		.pm = &ptn5150_pm_ops,
#endif
		.of_match_table = ptn5150_match_table,
	},
};

/*
 * module load/unload record keeping
 */

static int __init ptn5150_dev_init(void)
{
	pr_debug("Loading ptn5150 driver\n");
	return i2c_add_driver(&ptn5150_driver);
}
module_init(ptn5150_dev_init);

static void __exit ptn5150_dev_exit(void)
{
#if 0
	struct ptn5150_dev *ptn5150_dev = g_ptn_dev;
    struct i2c_client *client = ptn5150_dev->client;

    ptn5150_hw_preinit(client);
	//ptn5150_dev = i2c_get_clientdata(client);
	free_irq(client->irq, ptn5150_dev);
	misc_deregister(&ptn5150_dev->ptn5150_device);
	gpio_free(ptn5150_dev->irq_gpio);
	mutex_destroy(&typec_headset_lock);
	mutex_destroy(&ptn5150_i2c_lock);
#if defined(PTN5150_DEBUG)
	device_remove_file(&client->dev, &ptn5150_dev_attr);
#endif
	kfree(ptn5150_dev);
#endif

	printk("Unloading ptn5150 driver nyx !\n");
	i2c_del_driver(&ptn5150_driver);
}
module_exit(ptn5150_dev_exit);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("NXP Type-C PTN5150 driver");
MODULE_LICENSE("GPL");

