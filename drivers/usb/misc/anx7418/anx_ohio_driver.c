/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "anx_ohio_driver.h"
#include "anx_ohio_private_interface.h"
#include "anx_ohio_public_interface.h"

/* Use device tree structure data when defined "CONFIG_OF"  */
/* #define CONFIG_OF */

static int create_sysfs_interfaces(struct device *dev);

/* to access global platform data */
static struct ohio_platform_data *g_pdata;

#define DONGLE_CABLE_INSERT  1
#define CABLE_DET_PIN_HAS_GLITCH
#define MSEC_TO_JIFFIES(msec)			((msec) * HZ / 1000)

struct i2c_client *ohio_client;

enum {
	DRP_MODE = 0,
	UFP_MODE,
	DFP_MODE,
	DEBUG_MODE,
	AUDIO_MODE,
};

struct ohio_platform_data {
	int mode;
	int gpio_p_on;
	int gpio_reset;
	int gpio_cbl_det;
	int gpio_intr_comm;
	int gpio_uart_sw;
#ifdef OHIO_DEBUG
	int gpio_v33_ctrl;	/* GPIO debug pin */
#endif
	int gpio_uart_sw2;
	int gpio_ad_sel;
	spinlock_t lock;
};

struct ohio_data {
	struct ohio_platform_data *pdata;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	struct wake_lock ohio_lock;
};

struct class *g_ohio_class;
struct device *g_ohio_device;
static bool usb_audio_insert = false;

/* ohio power status, sync with interface and cable detection thread */
extern int pi5usb_set_msm_usb_host_mode(bool mode);
extern int wcd_mbhc_plug_detect(void);
extern void usb_audio_if_letv(bool *letv, int *pid);
extern int cclogic_set_audio_mode_register(void (*func)(bool));
extern int msm_usb_vbus_set(void *_mdwc, bool on, bool ext_call);
extern void cclogic_updata_port_state(int state);

bool ohio_is_connected(void)
{
	struct ohio_platform_data *pdata = NULL;
	bool result = false;

	pr_info("enter ohio_is_connected\n");

	if (!ohio_client)
		return false;

#ifdef CONFIG_OF
	pdata = g_pdata;
#else
	pdata = ohio_client->dev.platform_data;
#endif

	if (!pdata)
		return false;

	pr_info("get cable detect gpio value \n");
	if (gpio_get_value_cansleep(pdata->gpio_cbl_det)) {
		mdelay(10);
		if (gpio_get_value_cansleep(pdata->gpio_cbl_det)) {
			pr_info("%s %s : Slimport Dongle is detected\n",
					LOG_TAG, __func__);
			result = true;
		}
	}

	return result;
}
EXPORT_SYMBOL(ohio_is_connected);

struct i2c_client *get_i2c_addr(void)
{
	if (ohio_client != NULL)
		return ohio_client;

	return NULL;
}

inline unsigned char OhioReadReg(unsigned char RegAddr)
{
	int ret = 0;

	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_read_byte_data(ohio_client, RegAddr);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
	}
	return (uint8_t) ret;

}

inline int OhioReadBlockReg(u8 RegAddr, u8 len, u8 *dat)
{
	int ret = 0;

	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_read_i2c_block_data(ohio_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
		return -EPERM;
	}

	return (int)ret;
}

inline int OhioWriteBlockReg(u8 RegAddr, u8 len, const u8 *dat)
{
	int ret = 0;

	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_write_i2c_block_data(ohio_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
		return -EPERM;
	}

	return (int)ret;
}

inline void OhioWriteReg(unsigned char RegAddr, unsigned char RegVal)
{
	int ret = 0;
	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_write_byte_data(ohio_client, RegAddr, RegVal);
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
	}
}

void ohio_power_standby(void)
{
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif

#ifdef OHIO_DEBUG
	/* simulate first bootup case */
	gpio_set_value(pdata->gpio_v33_ctrl, 0);
	mdelay(20);
	gpio_set_value(pdata->gpio_v33_ctrl, 1);
	mdelay(2);

#endif

	gpio_set_value(pdata->gpio_reset, 0);
	mdelay(1);
	gpio_set_value(pdata->gpio_p_on, 0);
	mdelay(1);

	pr_info("ohio power down\n");
}

void ohio_hardware_poweron(void)
{
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif
	int retry_count, i;

	pr_info("ohio power on\n");

	interface_init();

	/*power on pin enable */
	gpio_set_value(pdata->gpio_p_on, 1);
	mdelay(10);

	/*power reset pin enable */
	gpio_set_value(pdata->gpio_reset, 1);
	mdelay(2);

	pr_info("ohio check ocm loading...\n");
	for (retry_count = 0; retry_count < 3; retry_count++) {
		/* eeprom load delay T3 (3.2s) */
		for (i = 0; i < OHIO_OCM_LOADING_TIME; i++) {
			/*Interface work? */
			if ((OhioReadReg(0x16) & 0x80) == 0x80) {
				pr_info("interface enabled\n");
				atomic_set(&ohio_power_status, 1);
				send_initialized_setting(0);
#ifdef SUP_OHIO_INT_VECTOR
				/* open interrupt vector */
				OhioWriteReg(OHIO_INTERFACE_INTR_MASK, 0);
#endif
				pr_info("interface init end\n");
				goto load_end;
			}
			mdelay(1);
			printk(".");
		}
	}

load_end:
	if (OhioReadReg(0x7F) == 0x0F) {
		if (OhioReadReg(0x66) == 0x1b) {
			pr_info("EEPROM CRC is correct!\n");
			atomic_set(&ohio_power_status, 1);
		} else if (OhioReadReg(0x66) == 0x4b) {
			pr_info("EEROM CRC is incorrect!\n");
		} else {
			pr_info("EEROM CRC is error!\n");
		}
	} else {
		pr_info("opt chip is power on!\n");
	}
}

static void ohio_free_gpio(struct ohio_data *ohio)
{
#ifdef OHIO_DEBUG
	gpio_free(ohio->pdata->gpio_v33_ctrl);
#endif
	gpio_free(ohio->pdata->gpio_cbl_det);
	gpio_free(ohio->pdata->gpio_reset);
	gpio_free(ohio->pdata->gpio_p_on);
	gpio_free(ohio->pdata->gpio_intr_comm);
	gpio_free(ohio->pdata->gpio_uart_sw);
	gpio_free(ohio->pdata->gpio_uart_sw2);
	gpio_free(ohio->pdata->gpio_ad_sel);
}

static int ohio_init_gpio(struct ohio_data *ohio)
{
	int ret = 0;

	pr_info("%s %s: ohio init gpio\n", LOG_TAG, __func__);
	/*  gpio for chip power down  */
	ret = gpio_request(ohio->pdata->gpio_p_on, "ohio_p_on_ctl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_p_on);
		goto err0;
	}
	gpio_direction_output(ohio->pdata->gpio_p_on, 0);
	/*  gpio for chip reset  */
	ret = gpio_request(ohio->pdata->gpio_reset, "ohio_reset_n");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_reset);
		goto err1;
	}
	gpio_direction_output(ohio->pdata->gpio_reset, 0);

#ifdef OHIO_DEBUG
	/*  gpio for chip standby control DVDD33 */
	ret = gpio_request(ohio->pdata->gpio_v33_ctrl, "ohio_v33_ctrl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_v33_ctrl);
		goto err1;
	}
	gpio_direction_output(ohio->pdata->gpio_v33_ctrl, 0);
#endif

	/*  gpio for ohio cable detect  */
	ret = gpio_request(ohio->pdata->gpio_cbl_det, "ohio_cbl_det");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_cbl_det);
		goto err2;
	}
	gpio_direction_input(ohio->pdata->gpio_cbl_det);
	/*  gpio for chip interface communaction */
	ret = gpio_request(ohio->pdata->gpio_intr_comm, "ohio_intr_comm");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_intr_comm);
		goto err3;
	}
	gpio_direction_input(ohio->pdata->gpio_intr_comm);
	/*  gpio for uart switch */
	ret = gpio_request(ohio->pdata->gpio_uart_sw, "ohio_uart_sw");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
				ohio->pdata->gpio_uart_sw);
		goto err4;
	}
	gpio_direction_output(ohio->pdata->gpio_uart_sw, 0);
	/*  gpio for uart switch2 */
	ret = gpio_request(ohio->pdata->gpio_uart_sw2, "ohio_uart_sw2");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_uart_sw2);
		goto err5;
	}
	gpio_direction_output(ohio->pdata->gpio_uart_sw2, 1);
	/*  gpio for audio D+/1 select */
	ret = gpio_request(ohio->pdata->gpio_ad_sel, "ohio_ad_sel");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
				ohio->pdata->gpio_ad_sel);
		goto err6;
	}
	gpio_direction_output(ohio->pdata->gpio_ad_sel, 0);
	goto out;

err6:
    gpio_free(ohio->pdata->gpio_ad_sel);
err5:
    gpio_free(ohio->pdata->gpio_uart_sw2);
err4:
	gpio_free(ohio->pdata->gpio_uart_sw);
err3:
	gpio_free(ohio->pdata->gpio_intr_comm);
err2:
	gpio_free(ohio->pdata->gpio_cbl_det);
err1:
	gpio_free(ohio->pdata->gpio_reset);
err0:
	gpio_free(ohio->pdata->gpio_p_on);

	return 1;
out:
	return 0;
}

void cable_disconnect(void *data)
{
	struct ohio_data *ohio = data;
	cancel_delayed_work_sync(&ohio->work);
	flush_workqueue(ohio->workqueue);
	ohio_power_standby();
	wake_unlock(&ohio->ohio_lock);
	wake_lock_timeout(&ohio->ohio_lock, 1000 * HZ);

}

/*
 * example: udpate power's source capability from AP to ohio
 *
 */
void update_pwr_src_caps(void)
{
	u32 src_caps[] = {
		/*5V, 0.9A, Fixed */
		PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_900MA, PDO_FIXED_FLAGS)
	};
	/* send source capability from AP to ohio */
	send_pd_msg(TYPE_PWR_SRC_CAP, (const char *)src_caps, sizeof(src_caps));
}
/*This func is for headset switch AD mode*/
void anx_cclogic_set_audio_mode(bool mode)
{
/*true for Analog mode and false for Digit mode*/
#ifdef CONFIG_OF
        struct ohio_platform_data *pdata = g_pdata;
#else
        struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif
	if(mode){
		pr_info("ohio switch to audio mode\n");
		pi5usb_set_msm_usb_host_mode(false);
		msm_usb_vbus_set(NULL, 1, true);
		gpio_set_value(pdata->gpio_uart_sw, 1);
		gpio_set_value(pdata->gpio_uart_sw2, 0);
		gpio_set_value(pdata->gpio_ad_sel, 1);
		pdata->mode = AUDIO_MODE;
		//notify audio module for headset plug in
		wcd_mbhc_plug_detect();
		usb_audio_insert = true;
	}else{
		gpio_set_value(pdata->gpio_uart_sw, 1);
		gpio_set_value(pdata->gpio_uart_sw2, 1);
		gpio_set_value(pdata->gpio_ad_sel, 0);
		msm_usb_vbus_set(NULL, 0, true);
		pi5usb_set_msm_usb_host_mode(true);
		pr_info("ohio set usb to host mode\n");
		pdata->mode = DFP_MODE;
		//notify audio module for headset plug out
		wcd_mbhc_plug_detect();
		usb_audio_insert = false;
	}
	return;
}
EXPORT_SYMBOL(anx_cclogic_set_audio_mode);
/*It's for update firmware about analogix start*/
#include "updatefw.c"
static int analogix_updatefw = 0;
static void ohio_update_ocm(void)
{
	u8 version;
	version = OhioReadReg(0x44);
	pr_info("ohio analogix 7418 ver=0x%x\n",version);
	/* if (version == 0x16) */
	anx7418_update();
}
/*It's for update firmware about analogix end*/
void ohio_main_process(int cable_connected)
{
		//TODO: process main task as you wanted.
#ifdef CONFIG_OF
		struct ohio_platform_data *pdata = g_pdata;
#else
		struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif
		u8 i,j;
		if (cable_connected){
			pr_info("%s: cable plug\n", __func__);
			if (atomic_read(&ohio_power_status) == 1) {
				atomic_set(&ohio_power_status, 0);
				mdelay(30); /* debounce unplug */
				ohio_power_standby();
			}
			ohio_hardware_poweron();
			atomic_set(&ohio_power_status, 1);
			if (analogix_updatefw > 0) {
				ohio_update_ocm();
				analogix_updatefw = 0;
			}

			i=OhioReadReg(0x48);
			pr_info("ohio read 0x48:%x\n",i);
			j=OhioReadReg(0x40);
			pr_info("ohio read 0x40:%x\n",j);

			if( (i & 0x0f) == 0x5 ) {
				pr_info("ohio switch uart_sw to uart mode\n");
				gpio_set_value(pdata->gpio_uart_sw, 0);
				gpio_set_value(pdata->gpio_uart_sw2, 1);
				gpio_set_value(pdata->gpio_ad_sel, 0);

				pr_info("ohio set usb to device mode\n");
//				pi5usb_set_msm_usb_host_mode(false);
				pi5usb_set_msm_usb_host_mode(true);
				pdata->mode = DEBUG_MODE;
				cclogic_updata_port_state(2);
			}else if ( (i & 0x0f) == 0xf ) {
				pr_info("ohio switch to audio mode\n");
				gpio_set_value(pdata->gpio_uart_sw, 1);
				gpio_set_value(pdata->gpio_uart_sw2, 0);
				gpio_set_value(pdata->gpio_ad_sel, 1);

				pr_info("ohio set usb to device mode\n");
				pi5usb_set_msm_usb_host_mode(false);
				pdata->mode = AUDIO_MODE;
				//notify audio module for headset plug in
				wcd_mbhc_plug_detect();
				cclogic_updata_port_state(3);

			}else {
				pr_info("ohio switch to SBU1/SBU2\n");
				gpio_set_value(pdata->gpio_uart_sw, 1);
				gpio_set_value(pdata->gpio_uart_sw2, 1);
				gpio_set_value(pdata->gpio_ad_sel, 0);

				if( (j & 0x08) == 0 ) {
					pi5usb_set_msm_usb_host_mode(true);
					pr_info("ohio set usb to host mode\n");
					pdata->mode = DFP_MODE;
					cclogic_updata_port_state(2);
				}else {
					pr_info("ohio set usb to device mode\n");
					pi5usb_set_msm_usb_host_mode(false);
					pdata->mode = UFP_MODE;
					cclogic_updata_port_state(1);
				}
			}

			OhioWriteReg(IRQ_EXT_SOURCE_2, 0xff);
		} else {
			pr_info("%s: cable unplug\n", __func__);
			atomic_set(&ohio_power_status, 0);
			mdelay(30); /* debounce unplug */
			pr_info("ohio excute power standby \n");
			ohio_power_standby();

			pr_info("ohio switch to SBU1/SBU2,when cable disconnect\n");
			gpio_set_value(pdata->gpio_uart_sw, 1);
			gpio_set_value(pdata->gpio_uart_sw2, 1);
			gpio_set_value(pdata->gpio_ad_sel, 0);

			pr_info("ohio set usb to device mode,when cable disconnect\n");
			pi5usb_set_msm_usb_host_mode(false);
			if (pdata->mode == AUDIO_MODE) {
				pr_info("%s: call headset plug out detect\n", __func__);
				//notify audio module for headset plug out
				wcd_mbhc_plug_detect();
				if (usb_audio_insert) {
					msm_usb_vbus_set(NULL, 0, true);
					usb_audio_insert = false;
				}
			}

			pdata->mode = DRP_MODE;
			cclogic_updata_port_state(0);
		}
}

inline void delay_msec(int ms)
{
	int i = 0;
	int len = 255;
	for (i = 0; i < ms; i++) {
		for (len = 0; len < 255; len++)
			;
	}
}

static unsigned char confirmed_cable_det(void *data)
{
	struct ohio_data *anxohio = data;
#ifdef CABLE_DET_PIN_HAS_GLITCH
	unsigned int count = 200;
	unsigned int cable_det_count = 0;
	u8 val = 0;
	do {
		val = gpio_get_value(anxohio->pdata->gpio_cbl_det);
		if (DONGLE_CABLE_INSERT == val) {
			pr_info("ohio cable detect %x,cnt =%d\n", val,
				cable_det_count);
			cable_det_count++;
		}
		if (cable_det_count > 20)
			break;

		delay_msec(1);
	} while (count--);

	return (cable_det_count > 20) ? 1 : 0;
#else
	return gpio_get_value(anxohio->pdata->gpio_cbl_det);
#endif
}

static irqreturn_t ohio_cbl_det_isr(int irq, void *data)
{
	struct ohio_data *ohio = data;

	queue_delayed_work(ohio->workqueue, &ohio->work, 0);

	return IRQ_HANDLED;
#if 0
	if (cable_connected == DONGLE_CABLE_INSERT) {
		if (atomic_read(&ohio_power_status) == 1) {
			atomic_set(&ohio_power_status, 0);
			mdelay(30); /* debounce unplug */
			ohio_power_standby();
		}
		ohio_hardware_poweron();
		atomic_set(&ohio_power_status, 1);
		if (analogix_updatefw > 0) {
			ohio_update_ocm();
			analogix_updatefw = 0;
		}
	} else {
		pr_info("%s: cable unplug\n", __func__);
		atomic_set(&ohio_power_status, 0);
		mdelay(30); /* debounce unplug */
		ohio_power_standby();
	}
	ohio_main_process(cable_connected);

	return IRQ_HANDLED;
#endif
}

static irqreturn_t ohio_intr_comm_isr(int irq, void *data)
{
	if (atomic_read(&ohio_power_status) != 1) {
		printk("%s: power status fail!!\n", __func__);
		return IRQ_NONE;
	}

	if (is_soft_reset_intr()) {
		clear_soft_interrupt();
#ifdef OHIO_DEBUG
		pr_info("I\n");
#endif

#ifdef SUP_OHIO_INT_VECTOR
		handle_intr_vector();
#else
		if (polling_interface_msg(INTERACE_TIMEOUT_MS) == 0)
			return IRQ_HANDLED;
#endif
	}
	return IRQ_HANDLED;
}

static void ohio_work_func(struct work_struct *work)
{
	struct ohio_data *td = container_of(work, struct ohio_data,
					    work.work);
	int cable_connected;
	cable_connected = confirmed_cable_det((void*)td) ;

	mutex_lock(&td->lock);
	/* ohio_main_process(); */
	ohio_main_process(cable_connected);
	mutex_unlock(&td->lock);
	/*queue_delayed_work(td->workqueue, &td->work,
			   msecs_to_jiffies(workqueu_timer));*/
}

#ifdef CONFIG_OF
int ohio_regulator_configure(struct device *dev,
			     struct ohio_platform_data *pdata)
{
	return 0;
}

static int ohio_parse_dt(struct device *dev, struct ohio_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->gpio_p_on =
	    of_get_named_gpio_flags(np, "p-on-gpio", 0, NULL);

	pdata->gpio_reset =
	    of_get_named_gpio_flags(np, "reset-gpio", 0, NULL);

	pdata->gpio_cbl_det =
	    of_get_named_gpio_flags(np, "cbl-det-gpio", 0, NULL);

#ifdef OHIO_DEBUG
	pdata->gpio_v33_ctrl =
	    of_get_named_gpio_flags(np, "analogix,v33-ctrl-gpio", 0, NULL);
#endif

	pdata->gpio_intr_comm =
	    of_get_named_gpio_flags(np, "intr-comm-gpio", 0, NULL);
	pdata->gpio_uart_sw =
		of_get_named_gpio_flags(np, "uart-sw-gpio", 0, NULL);
	pdata->gpio_uart_sw2 =
		of_get_named_gpio_flags(np, "uart-sw2-gpio", 0, NULL);
	pdata->gpio_ad_sel =
		of_get_named_gpio_flags(np, "ad-sel-gpio", 0, NULL);

	pr_info("%s gpio p_on : %d, reset : %d,  gpio_cbl_det %d\n",
		LOG_TAG, pdata->gpio_p_on,
		pdata->gpio_reset, pdata->gpio_cbl_det);

	return 0;
}
#else
static int ohio_parse_dt(struct device *dev, struct ohio_platform_data *pdata)
{
	return -ENODEV;
}
#endif
static int ohio_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{

	struct ohio_data *ohio;
	struct ohio_platform_data *pdata;
	int ret = 0;
	int cbl_det_irq = 0;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s:ohio's i2c bus doesn't support\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	ohio = kzalloc(sizeof(struct ohio_data), GFP_KERNEL);
	if (!ohio) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct ohio_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;

		/* device tree parsing function call */
		ret = ohio_parse_dt(&client->dev, pdata);
		if (ret != 0)	/* if occurs error */
			goto err0;

		ohio->pdata = pdata;
	} else {
		ohio->pdata = client->dev.platform_data;
	}

	/* to access global platform data */
	g_pdata = ohio->pdata;
	ohio_client = client;

	atomic_set(&ohio_power_status, 0);

	mutex_init(&ohio->lock);

	if (!ohio->pdata) {
		ret = -EINVAL;
		goto err0;
	}

	ret = ohio_init_gpio(ohio);
	if (ret) {
		pr_err("%s: failed to initialize gpio\n", __func__);
		goto err0;
	}

	INIT_DELAYED_WORK(&ohio->work, ohio_work_func);

	ohio->workqueue = create_singlethread_workqueue("ohio_work");
	if (ohio->workqueue == NULL) {
		pr_err("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	cbl_det_irq = gpio_to_irq(ohio->pdata->gpio_cbl_det);
	if (cbl_det_irq < 0) {
		pr_err("%s : failed to get gpio irq\n", __func__);
		goto err1;
	}

	wake_lock_init(&ohio->ohio_lock, WAKE_LOCK_SUSPEND, "ohio_wake_lock");

	ret = request_threaded_irq(cbl_det_irq, NULL, ohio_cbl_det_isr,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING
				   | IRQF_ONESHOT, "ohio-cbl-det", ohio);
	if (ret < 0) {
		pr_err("%s : failed to request irq\n", __func__);
		goto err3;
	}

	ret = irq_set_irq_wake(cbl_det_irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for cable detect", __func__);
		pr_err("interrupt wake set fail\n");
		goto err4;
	}

	ret = enable_irq_wake(cbl_det_irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for cable detect", __func__);
		pr_err("interrupt wake enable fail\n");
		goto err4;
	}

	client->irq = gpio_to_irq(ohio->pdata->gpio_intr_comm);
	if (client->irq < 0) {
		pr_err("%s : failed to get ohio gpio comm irq\n", __func__);
		goto err3;
	}

	ret = request_threaded_irq(client->irq, NULL, ohio_intr_comm_isr,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING
				   | IRQF_ONESHOT, "ohio-intr-comm", ohio);
	if (ret < 0) {
		pr_err("%s : failed to request interface irq\n", __func__);
		goto err4;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for interface communaction", __func__);
		goto err4;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for interface communaction", __func__);
		goto err4;
	}

	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("%s : sysfs register failed", __func__);
		goto err4;
	}

	ohio->pdata->mode = DRP_MODE;

	/* uart default. */
	gpio_set_value(ohio->pdata->gpio_uart_sw, 0);
	gpio_set_value(ohio->pdata->gpio_uart_sw2, 1);

	/*when probe ohio device, enter standy mode */
	ohio_power_standby();

	/*enable driver*/
	queue_delayed_work(ohio->workqueue, &ohio->work, MSEC_TO_JIFFIES(5000));

	cclogic_set_audio_mode_register(anx_cclogic_set_audio_mode);

	pr_info("ohio_i2c_probe successfully %s %s end\n", LOG_TAG, __func__);
	goto exit;

err4:
	free_irq(client->irq, ohio);
err3:
	free_irq(cbl_det_irq, ohio);
err1:
	ohio_free_gpio(ohio);
	destroy_workqueue(ohio->workqueue);
err0:
	ohio_client = NULL;
	kfree(ohio);
exit:
	return ret;
}

static int ohio_i2c_remove(struct i2c_client *client)
{
	struct ohio_data *ohio = i2c_get_clientdata(client);
	printk("ohio_i2c_remove\n");
	free_irq(client->irq, ohio);
	ohio_free_gpio(ohio);
	destroy_workqueue(ohio->workqueue);
	wake_lock_destroy(&ohio->ohio_lock);
	kfree(ohio);
	return 0;
}

static int ohio_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int ohio_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ohio_id[] = {
	{"ohio", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ohio_id);

#ifdef CONFIG_OF
static struct of_device_id anx_match_table[] = {
	{.compatible = "analogix,ohio",},
	{},
};
#endif

static struct i2c_driver ohio_driver = {
	.driver = {
		   .name = "ohio",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = anx_match_table,
#endif
		   },
	.probe = ohio_i2c_probe,
	.remove = ohio_i2c_remove,
	.suspend = ohio_i2c_suspend,
	.resume = ohio_i2c_resume,
	.id_table = ohio_id,
};

static void __init ohio_init_async(void *data, async_cookie_t cookie)
{
	int ret = 0;

	ret = i2c_add_driver(&ohio_driver);
	if (ret < 0)
		pr_err("%s: failed to register ohio i2c drivern", __func__);
}

static int __init ohio_init(void)
{
	async_schedule(ohio_init_async, NULL);
	return 0;
}

static void __exit ohio_exit(void)
{
	i2c_del_driver(&ohio_driver);
}

#ifdef OHIO_DEBUG
/******************************
 * detect ohio chip id
 * ohio Vendor ID definition, low and high byte
 */
#define OHIO_SLVAVE_I2C_ADDR 0x50
#define VENDOR_ID_L 0x02
#define VENDOR_ID_H 0x03
#define OHIO_NUMS 7
uint chipid_list[OHIO_NUMS] = {
	0x7418,
	0x7428,
	0x7408,
	0x7409,
	0x7401,
	0x7402,
	0x7403
};

bool ohio_chip_detect(void)
{
	uint c;
	bool big_endian;
	unchar *ptemp;
	int i;
	/*check whether CPU is big endian */
	c = 0x1222;
	ptemp = (unchar *) &c;
	if (*ptemp == 0x11 && *(ptemp + 1) == 0x22)
		big_endian = 1;
	else
		big_endian = 0;

	c = 0;
	/*check chip id */
	if (big_endian) {

		c = OhioReadReg(VENDOR_ID_H);
		c += OhioReadReg(VENDOR_ID_L) * 256;

	} else {
		c = OhioReadReg(VENDOR_ID_L);
		c += OhioReadReg(VENDOR_ID_H) * 256;
	}

	pr_info("%s %s : CHIPID: ANX%x\n", LOG_TAG, __func__, c & 0x0000FFFF);
	for (i = 0; i < OHIO_NUMS; i++) {
		if (c == chipid_list[i])
			return 1;
	}
	return 0;
}

void dump_reg(void)
{
	int i = 0;
	u8 val = 0;

	printk("dump registerad:\n");
	printk("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
	for (i = 0; i < 256; i++) {
		val = OhioReadReg(i);

		if ((i) % 0x10 == 0x00)
			printk("\n[%x]:%02x ", i, val);
		else
			printk("%02x ", val);

	}
	printk("\n");
}

ssize_t anx_ohio_send_pd_cmd(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	switch (cmd) {
	case TYPE_PWR_SRC_CAP:
		update_pwr_src_caps();
		break;

	case TYPE_DP_SNK_IDENDTITY:
		send_pd_msg(TYPE_DP_SNK_IDENDTITY, 0, 0);
		break;

	case TYPE_PSWAP_REQ:
		send_pd_msg(TYPE_PSWAP_REQ, 0, 0);
		break;
	case TYPE_DSWAP_REQ:
		send_pd_msg(TYPE_DSWAP_REQ, 0, 0);
		break;

	case TYPE_GOTO_MIN_REQ:
		send_pd_msg(TYPE_GOTO_MIN_REQ, 0, 0);
		break;

	case TYPE_PWR_OBJ_REQ:
		interface_send_request();
		break;
	case TYPE_ACCEPT:
		interface_send_accept();
		break;
	case TYPE_REJECT:
		interface_send_reject();
		break;
	case TYPE_SOFT_RST:
		send_pd_msg(TYPE_SOFT_RST, 0, 0);
		break;
	case TYPE_HARD_RST:
		send_pd_msg(TYPE_HARD_RST, 0, 0);
		break;

	case 0xFD:
		pr_info("fetch powerrole: %d\n", get_power_role());
		break;
	case 0xFE:
		pr_info("fetch datarole: %d\n", get_data_role());
		break;

	case 0xff:
		dump_reg();
		break;
	}
	return count;
}

ssize_t anx_ohio_get_data_role(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_data_role());
}

ssize_t anx_ohio_get_power_role(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_power_role());
}

ssize_t anx_ohio_rd_reg(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	printk("reg[%x] = %x\n", cmd, OhioReadReg(cmd));

	return count;

}

ssize_t anx_ohio_wr_reg(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int cmd, val;
	int result;

	result = sscanf(buf, "%d  %d", &cmd, &val);
	pr_info("c %x val %x\n", cmd, val);
	OhioWriteReg(cmd, val);
	pr_info("reg[%x] = %x\n", cmd, OhioReadReg(cmd));
	return count;
}

ssize_t anx_ohio_dump_register(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int i = 0;
	for (i = 0; i < 256; i++) {
		printk("%x", OhioReadReg(i));
		if (i % 0x10 == 0)
			pr_info("\n");

		snprintf(&buf[i], sizeof(u8), "%d", OhioReadReg(i));
	}

	printk("\n");

	return i;
}

ssize_t anx_ohio_select_rdo_index(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int cmd;
	cmd = sscanf(buf, "%d", &cmd);
	if (cmd <= 0)
		return 0;

	pr_info("NewRDO idx %d, Old idx %d\n", cmd, sel_voltage_pdo_index);
	sel_voltage_pdo_index = cmd;
	return count;
}

/* for debugging */
static struct device_attribute anx_ohio_device_attrs[] = {
	__ATTR(pdcmd, S_IWUSR, NULL,
	       anx_ohio_send_pd_cmd),
	__ATTR(rdreg, S_IWUSR, NULL,
	       anx_ohio_rd_reg),
	__ATTR(wrreg, S_IWUSR, NULL,
	       anx_ohio_wr_reg),
	__ATTR(rdoidx, S_IWUSR, NULL,
	       anx_ohio_wr_reg),
	__ATTR(dumpreg, S_IRUGO, anx_ohio_dump_register,
	       NULL),
	__ATTR(prole, S_IRUGO, anx_ohio_get_power_role,
	       NULL),
	__ATTR(drole, S_IRUGO, anx_ohio_get_data_role,
	       NULL)
};
#else
static struct device_attribute anx_ohio_device_attrs[] = {  };
#endif

ssize_t anx_ohio_updatefw_cmd(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	printk("%s\n", __func__);
	if (cmd > 0)
		analogix_updatefw = cmd;

	return count;
}

static struct device_attribute anx_ohio_updatefw_attrs[] = {
	__ATTR(updatefw, S_IWUSR, NULL,
	       anx_ohio_updatefw_cmd)
};

static ssize_t anx_ohio_usb_audio_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
	int rv = 0;
	bool if_letv = false;
	int pid;

	usb_audio_if_letv(&if_letv,&pid);
	if (if_letv)
	    rv = scnprintf(buf, PAGE_SIZE, "%d\n", if_letv);
	else
	    rv = scnprintf(buf, PAGE_SIZE, "%d\n", 0);

	return rv;
}

static ssize_t anx_ohio_usb_audio_pid_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
	int rv = 0;
	bool if_letv = false;
	int pid;

	usb_audio_if_letv(&if_letv,&pid);
	if (if_letv)
	    rv = scnprintf(buf, PAGE_SIZE, "0x%x\n", pid);
	else
	    rv = scnprintf(buf, PAGE_SIZE, "%d\n", 0);

	return rv;
}

static ssize_t anx_ohio_dev_id_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
	int rv = 0;
	uint c;
	bool big_endian;
	unchar *ptemp;

#define SLVAVE_I2C_ADDR 0x50
#define DEVICE_ID_L 0x02
#define DEVICE_ID_H 0x03
	/*check whether CPU is big endian*/
	c = 0x1222;
	ptemp = (unchar*)&c;
	if(*ptemp == 0x11 && *(ptemp + 1) == 0x22)
	    big_endian = 1;
	else
	big_endian = 0;
	c = 0;
	/*check chip id*/
	if(big_endian) {
	    ohio_read_reg(SLVAVE_I2C_ADDR, DEVICE_ID_L, (unchar *)(&c) + 1);
	    ohio_read_reg(SLVAVE_I2C_ADDR, DEVICE_ID_H, (unchar *)(&c));
	}else{
	    ohio_read_reg(SLVAVE_I2C_ADDR, DEVICE_ID_L, (unchar *)(&c));
	    ohio_read_reg(SLVAVE_I2C_ADDR, DEVICE_ID_H, (unchar *)(&c) + 1);
	}

	if (0 != (c & 0x0000FFFF))
	    rv = scnprintf(buf, PAGE_SIZE, "ANX%x\n", c & 0x0000FFFF);
	else
	    rv = scnprintf(buf, PAGE_SIZE, "ANX%s\n", "");

	return rv;
}

static struct device_attribute anx_ohio_cclogic_attrs[] = {
	__ATTR(devid, S_IRUGO, anx_ohio_dev_id_show,NULL),
	__ATTR(usb_audio, S_IRUGO, anx_ohio_usb_audio_show,NULL),
	__ATTR(usb_audio_pid, S_IRUGO, anx_ohio_usb_audio_pid_show,NULL)
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i,j;
	pr_info("ohio create system fs interface ...\n");
	g_ohio_class = class_create(THIS_MODULE, "cclogic_class");
	if (IS_ERR(g_ohio_class)) {
		printk("%s:create class fail \n", __func__);
		return -EINVAL;
	}
	g_ohio_device = device_create(g_ohio_class, NULL, dev->devt, NULL, "cclogic_device");

	for (i = 0; i < ARRAY_SIZE(anx_ohio_cclogic_attrs); i++)
		if (device_create_file(g_ohio_device, &anx_ohio_cclogic_attrs[i]))
			goto error;

	for (j = 0; j < ARRAY_SIZE(anx_ohio_device_attrs); j++)
		if (device_create_file(dev, &anx_ohio_device_attrs[j]))
			goto error;

	if (device_create_file(dev, &anx_ohio_updatefw_attrs[0]))
		goto error;

	pr_info("%s: success\n", __func__);

	return 0;
error:

	for (; j >= 0; j--)
		device_remove_file(dev, &anx_ohio_device_attrs[j]);
	for (; i >= 0; i--)
		device_remove_file(dev, &anx_ohio_cclogic_attrs[i]);
	device_destroy(g_ohio_class, dev->devt);
	class_destroy(g_ohio_class);
	pr_err("%s %s: ohio Unable to create interface", LOG_TAG, __func__);
	return -EINVAL;
}

module_init(ohio_init);
module_exit(ohio_exit);

MODULE_DESCRIPTION("USB PD Ohio driver");
MODULE_AUTHOR("Zhentian Tang <ztang@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.5");
