/*******************************************************************************************
 * Copyright (C) 2012 Hideep, Inc.
 * kim.liao@hideep.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *******************************************************************************/

#include "hideep3d.h"

int loglevel3d;
struct hideep3d_t *g_h3d;
int hideep3d_probe_state = 0;

#ifdef CONFIG_FB
static int hideep3d_suspend(struct device *dev);
static int hideep3d_resume(struct device *dev);
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

void hideep3d_release_flag(void)
{
	unsigned char buf;
	buf = 0x00;
	if (g_h3d == NULL) {
		HIDEEP3D_DBG("Not ready 3d I2C");
	} else {
		hideep3d_i2c_write(g_h3d, 0x809, 1, &buf);
	}
}
EXPORT_SYMBOL(hideep3d_release_flag);

unsigned short hideep3d_get_value(unsigned short x, unsigned short y)
{
	int ret;
	unsigned short z_value = 10;
	unsigned char buf[4];
	unsigned char i2c_buff[2];
	unsigned int info_size;

	if(g_h3d == NULL){
		HIDEEP3D_DBG("Not ready 3d I2C");
		goto hideep3d_no_device_exit;
	}

	mutex_lock(&g_h3d->dev_mutex);
	if(power_updating == g_h3d->dev_state){
		HIDEEP3D_INFO("Device state is power updating");
		goto hideep3d_get_value_exit;
	}
	if (g_h3d->debug_dev.enable == false) {
		buf[0] = x & 0xff;
		buf[1] = (x >> 8) & 0xff;
		buf[2] = y & 0xff;
		buf[3] = (y >> 8) & 0xff;

		HIDEEP3D_XY("Before x-axis : %d, y-axis : %d", x, y);

		ret = hideep3d_i2c_write(g_h3d, 0x80a, 4, buf);

		usleep_range(50, 100);

		ret = hideep3d_i2c_read(g_h3d, HIDEEP3D_EVENT_COUNT_ADDR, 2, (u8*)&i2c_buff);

		if (ret < 0)
			HIDEEP3D_XY("Can't not touch count read1!!!");

		g_h3d->tch_count = i2c_buff[0];

		HIDEEP3D_XY("Touch count : %d", g_h3d->tch_count);

		if (g_h3d->tch_count > 0 && g_h3d->tch_count <= HIDEEP3D_MT_MAX) {
			info_size = g_h3d->tch_count * sizeof(struct hideep3d_mt_t);
			ret = hideep3d_i2c_read(g_h3d, 0x242, info_size, (u8*)g_h3d->touch_evt);
			if (ret < 0 || g_h3d->touch_evt[0].z <= 0) {
				HIDEEP3D_ERR("No get Data!!!!");
				z_value = 10;
				goto hideep3d_get_value_exit;
			}
			z_value = g_h3d->touch_evt[0].z;

			HIDEEP3D_XY("After x-axis : %d, y-axis : %d, z-axis : %d\n", g_h3d->touch_evt[0].x, g_h3d->touch_evt[0].y, g_h3d->touch_evt[0].z);

			if ((g_h3d->z_flag_calib2) && (!g_h3d->z_flag_ready)) {
				HIDEEP3D_XY("z = 0x%02x, index = %d", z_value, g_h3d->z_index);
				if(g_h3d->z_index >= g_h3d->z_calib_start) {
					HIDEEP3D_XY("reading");
					g_h3d->z_data[g_h3d->z_index] = z_value;
					if (g_h3d->z_index >= g_h3d->z_calib_end) {
						HIDEEP3D_XY("ready");
						g_h3d->z_flag_ready = true;
					}
				}
				if (!g_h3d->z_flag_ready)
					g_h3d->z_index++;
			}else{
                HIDEEP3D_XY("z_buffer, %d",z_value);
                g_h3d->z_buffer = z_value;
                g_h3d->z_status = true;
            }
		}
	}
hideep3d_get_value_exit:
	mutex_unlock(&g_h3d->dev_mutex);
hideep3d_no_device_exit:
	return z_value;
}
EXPORT_SYMBOL(hideep3d_get_value);

int hideep3d_dev_state_init_updating(void)
{
	int ret=0;
	if(g_h3d == NULL){
		if(hideep3d_probe_state == 1)
			ret = 1;
		return ret;
	}
	ret = g_h3d->h3d_fw_update_state;

	return ret;
}
EXPORT_SYMBOL(hideep3d_dev_state_init_updating);

static int hideep3d_hw_pwron(struct hideep3d_t *h3d)
{
	struct hideep_platform_data_t *pdata;
	int ret = 0;

	pdata = h3d->p_data;

	if ((!IS_ERR(pdata->vcc_vdd)) && (pdata->regulator_vdd != NULL)) {
		HIDEEP3D_DBG("hideep:vcc_vdd is enable");
		ret = regulator_enable(pdata->vcc_vdd);
		if (ret) {
			HIDEEP3D_ERR("Regulator vdd enable failed ret=%d", ret);
		}
	}
	msleep(1);

	if ((!IS_ERR(pdata->vcc_vid)) && (pdata->regulator_vid != NULL)) {
		HIDEEP3D_DBG("hideep:vcc_vid is enable");
		ret = regulator_enable( pdata->vcc_vid );
		if(ret) {
			HIDEEP3D_ERR("Regulator vcc_vid enable failed ret=%d", ret);
		}
	}
	msleep(3);

	if (pdata->reset_gpio > 0) {
		HIDEEP3D_DBG("hideep:enable the reset_gpio");
		ret = pinctrl_select_state(pdata->pinctrl, pdata->reset_up);
		if (ret < 0) {
			HIDEEP3D_ERR("can not reset pin control!!!");
		}
	}

	return ret;
}

static int hideep3d_hw_pwroff(struct hideep3d_t *h3d)
{
	struct hideep_platform_data_t *pdata;
	int ret = 0;

	pdata = h3d->p_data;

	if (pdata->reset_gpio > 0) {
		HIDEEP3D_DBG("hideep:disable the reset_gpio");
		ret = pinctrl_select_state(pdata->pinctrl, pdata->reset_down);
		if (ret < 0) {
			HIDEEP3D_ERR("can not reset pin control!!!");
		}
	}

	if (!IS_ERR(pdata->vcc_vid) && (pdata->regulator_vid!=NULL)) {
		HIDEEP3D_DBG("hideep:vcc_vid is disable");
		ret = regulator_disable(pdata->vcc_vid);
		if (ret) {
			HIDEEP3D_ERR("Regulator vcc_vid enable failed ret=%d", ret);
		}
	}

	if (!IS_ERR(pdata->vcc_vdd) && (pdata->regulator_vdd!=NULL)) {
		HIDEEP3D_DBG("hideep:vcc_vdd is disable");
		ret = regulator_disable(pdata->vcc_vdd);
		if(ret) {
			HIDEEP3D_ERR("Regulator vdd disable failed ret=%d", ret);
		}
	}

	return ret;
}

void hideep3d_power(struct hideep3d_t *h3d, int on)
{
	int ret = 0;

	if (on) {
		HIDEEP3D_DBG("power on");
		ret = hideep3d_hw_pwron(h3d);
	} else {
		HIDEEP3D_DBG("power off");
		ret = hideep3d_hw_pwroff(h3d);
	}
}

int hideep3d_i2c_read(struct hideep3d_t *h3d, u16 addr, u16 len, u8 *buf)
{
	int ret = -1;
	struct i2c_client *client = h3d->client;

	HIDEEP3D_I2C("addr=0x%02x, len=%d", addr, len);
	mutex_lock(&h3d->i2c_mutex);
	ret = i2c_master_send(client, (char *) &addr, 2);

	if (ret < 0) {
		goto i2c_err;
	}

	ret = i2c_master_recv(client, (char *) buf, len);

	if (ret < 0) {
		goto i2c_err;
	}

	mutex_unlock(&h3d->i2c_mutex);
	return  0;

i2c_err:
	mutex_unlock(&h3d->i2c_mutex);
	return -1;
}

int hideep3d_i2c_write(struct hideep3d_t *h3d, u16 addr, u16 len, u8 *buf)
{
	int ret = -1;
	struct i2c_client *client = h3d->client;

	HIDEEP3D_I2C("addr=0x%02x, len=%d", addr, len);
	mutex_lock(&h3d->i2c_mutex);

	// data mangling..
	h3d->i2c_buf[0] = (addr >> 0) & 0xFF;
	h3d->i2c_buf[1] = (addr >> 8) & 0xFF;
	memcpy( &h3d->i2c_buf[2], buf, len);

	ret = i2c_master_send(client, (char *)h3d->i2c_buf, len + 2);

	if (ret < 0){
		goto i2c_err;
	}

	mutex_unlock(&h3d->i2c_mutex);
	return  0;

i2c_err:
	mutex_unlock(&h3d->i2c_mutex);
	return -1;
}


void hideep3d_reset_ic(struct hideep3d_t *h3d)
{
	struct hideep_platform_data_t *pdata;
	int ret;
	u8 cmd = 1;
	pdata = h3d->p_data;

	HIDEEP3D_DBG("start!!");

	if (pdata->reset_gpio > 0) {
		HIDEEP3D_DBG("hideep:enable the reset_gpio");
		// up, down, up
		ret = gpio_direction_output(pdata->reset_gpio, 1);
		if (ret) {
			HIDEEP3D_ERR("can not reset pin control(status:1xx:%d)!!!",ret);
		} else {
			mdelay(1);	//modified
			ret = gpio_direction_output(pdata->reset_gpio, 0);
			if (ret) {
				HIDEEP3D_ERR("can not reset pin control(status:10x:%d)!!!",ret);
			} else {
				mdelay(20);	//modified
				ret = gpio_direction_output(pdata->reset_gpio, 1);
				if (ret) {
					HIDEEP3D_ERR("can not reset pin control!!!(status:101:%d)",ret);
				}
			}
		}
	} else {
		hideep3d_i2c_write(h3d, HIDEEP_RESET, 1, &cmd);
	}
	mdelay(50);
	HIDEEP3D_DBG("end!!");
}

void hideep3d_init_ic(struct hideep3d_t *h3d)
{
	struct hideep_platform_data_t *pdata;
	struct device dev;
	int ret = 0;

	pdata = h3d->p_data;
	dev = h3d->client->dev;

	/* regulator get */
	if (pdata->regulator_vdd != NULL) {
		pdata->vcc_vdd = regulator_get(&dev, pdata->regulator_vdd);   // main valtage
		if (IS_ERR(pdata->vcc_vdd)) {
			ret = PTR_ERR(pdata->vcc_vdd);
			HIDEEP3D_ERR("Regulator get vcc_vdd ret=%d", ret);
		}
		if (regulator_count_voltages(pdata->vcc_vdd) > 0) {
			ret = regulator_set_voltage(pdata->vcc_vdd, 3300000,3300000);
			if (ret) {
				HIDEEP3D_ERR("Regulator set_vdd reg_avdd failed error=%d\n", ret);
			}
		}
	}

	if (pdata->regulator_vid != NULL) {
		pdata->vcc_vid = regulator_get(&dev, pdata->regulator_vid);   // I/O voltage
		if (IS_ERR(pdata->vcc_vid)) {
			ret = PTR_ERR(pdata->vcc_vid);
			HIDEEP3D_ERR("Regulator get vcc_vid ret=%d", ret);
		}
	}

	if (pdata->reset_gpio > 0) {
		if (gpio_is_valid(pdata->reset_gpio)) {
			ret = gpio_request(pdata->reset_gpio, "reset_gpio");
			if (ret) {
				HIDEEP3D_ERR("unable to request gpio [%d]", pdata->reset_gpio);
			}
		} else {
			HIDEEP3D_WARN("reset[%d] is used by other device", pdata->reset_gpio);
		}
	}
	/* power on */
	hideep3d_power(h3d, true);
	h3d->dev_state = power_init;
	mdelay(30);		// modified

	/* ic reset */
	hideep3d_reset_ic(h3d);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hideep3d_early_suspend(struct early_suspend *h)
{
	struct hideep3d_t *h3d = container_of(h, struct hideep_t, early_suspend);
	u8 sleep_cmd = 1;

	HIDEEP3D_DBG("enter");

	mutex_lock(&h3d->dev_mutex);
	if(h3d->dev_state == power_sleep)
		goto hideep_early_suspend_exit;

	HIDEEP3D_DBG("not waiting.");
	h3d->dev_state = power_sleep;

	hideep3d_i2c_write(h3d, HIDEEP_SLEEP_MODE, 1, &sleep_cmd);

hideep_early_suspend_exit:
	mutex_unlock(&h3d->dev_mutex);
	HIDEEP3D_DBG("exit.");
    return;
}

static void hideep3d_late_resume(struct early_suspend *h)
{
	struct hideep3d_t *h3d = container_of(h, struct hideep_t, early_suspend);

	HIDEEP3D_DBG("enter");

	mutex_lock(&h3d->dev_mutex);

	if(h3d->dev_state == power_normal)
		goto hideep_late_resume_exit;

	HIDEEP3D_DBG("not waiting.");
	h3d->dev_state = power_normal;

hideep_late_resume_exit:
	mdelay(10);
	hideep3d_reset_ic(h3d);

	mutex_unlock(&h3d->dev_mutex);
	HIDEEP3D_DBG("exit.");
	return;
}
#endif

static int hideep3d_i2c_suspend(struct device *dev)
{
	struct hideep3d_t *h3d = dev_get_drvdata(dev);
	u8 sleep_cmd = 1;

	HIDEEP3D_DBG("enter");

	mutex_lock(&h3d->dev_mutex);
	if(h3d->dev_state == power_sleep)
		goto hideep_i2c_suspend_exit;

	HIDEEP3D_DBG("not waiting.");
	h3d->dev_state = power_sleep;

	//normal sleep
	hideep3d_i2c_write(h3d, HIDEEP_SLEEP_MODE, 1, &sleep_cmd);

hideep_i2c_suspend_exit:
	mutex_unlock(&h3d->dev_mutex);
	HIDEEP3D_DBG("exit.");
	return 0;
}

static int hideep3d_i2c_resume(struct device *dev)
{
	struct hideep3d_t *h3d = dev_get_drvdata(dev);

	HIDEEP3D_DBG("enter");

	mutex_lock(&h3d->dev_mutex);

	if(h3d->dev_state == power_normal)
		goto hideep_i2c_resume_exit;

	HIDEEP3D_DBG("not waiting.");
	h3d->dev_state = power_normal;

hideep_i2c_resume_exit:
	mdelay(10);
	hideep3d_reset_ic(h3d);

	mutex_unlock(&h3d->dev_mutex);
	HIDEEP3D_DBG("exit.");
	return 0;
}

#ifdef CONFIG_FB
static int hideep3d_suspend(struct device *dev)
{
	struct hideep3d_t *h3d = dev_get_drvdata(dev);
	u8 sleep_cmd = 1;

	HIDEEP3D_DBG("enter");

	mutex_lock(&h3d->dev_mutex);
	if(h3d->dev_state == power_sleep)
		goto hideep_suspend_exit;

	HIDEEP3D_DBG("not waiting.");
	h3d->dev_state = power_sleep;

	//normal sleep
	hideep3d_i2c_write(h3d, HIDEEP_SLEEP_MODE, 1, &sleep_cmd);
	hideep3d_power(h3d, false);

hideep_suspend_exit:
	mutex_unlock(&h3d->dev_mutex);
	HIDEEP3D_DBG("exit.");
	return 0;
}

static int hideep3d_resume(struct device *dev)
{
	struct hideep3d_t *h3d = dev_get_drvdata(dev);
	u8 sleep_cmd = 0;

	HIDEEP3D_DBG("enter");

	mutex_lock(&h3d->dev_mutex);

	if(h3d->dev_state == power_normal)
		goto hideep_resume_exit;

	HIDEEP3D_DBG("not waiting.");
	h3d->dev_state = power_normal;

	hideep3d_power(h3d, true);
	mdelay(10);
	/* send wake up command.. */
	hideep3d_i2c_read(h3d, HIDEEP_SLEEP_MODE, 1, &sleep_cmd);

hideep_resume_exit:
	mdelay(10);
	hideep3d_reset_ic(h3d);

	mutex_unlock(&h3d->dev_mutex);
	HIDEEP3D_DBG("exit.");
	return 0;
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	struct hideep3d_t *h3d = container_of(self, struct hideep3d_t, fb_notif);

	if ((evdata) && (evdata->data) && (event == FB_EVENT_BLANK) && (h3d) && (h3d->client)) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			HIDEEP3D_DBG("resume");
			if (h3d->suspended == 1) {
				hideep3d_resume(&h3d->client->dev);
				h3d->suspended = 0;
			}
		} else if (*blank == FB_BLANK_POWERDOWN) {
			HIDEEP3D_DBG("suspend");
			if (h3d->suspended == 0) {
				h3d->suspended = 1;
				hideep3d_suspend(&h3d->client->dev);
			}
		}
	}

	return 0;
}
#endif

#ifdef HIDEEP_AUTO_UPDATE
static void fwu_startup_fw_update_work(struct work_struct *work)
{
	int ret;
	char fw_name[30];
	struct hideep_platform_data_t *pdata;

	pdata = g_h3d->p_data;
	if(!pdata->project_name){
		HIDEEP3D_ERR("Can't get the project name from DTS");
		return;
	}
	g_h3d->manually_update = false;
	g_h3d->h3d_fw_update_state=1;
	ret = snprintf(fw_name, sizeof(fw_name), "letv_%s_%s", pdata->project_name,HIDEEP3D_AUTO_FW);
	HIDEEP3D_DBG("hideep starting fw[%s] update...", fw_name);
	ret = hideep3d_load_ucode(&g_h3d->client->dev, fw_name);

	if (ret)
		HIDEEP3D_ERR("The firmware update failed(%d)", ret);
	g_h3d->h3d_fw_update_state=0;
	HIDEEP3D_DBG("hideep done fw[%s] update.", fw_name);
	return;
}
#endif

static int hideep3d_parse_dts(struct device *dev, struct hideep_platform_data_t *pdata)
{
	int ret = 0;
	struct device_node *np;

	HIDEEP3D_DBG("enter");
	np = dev->of_node;

	/* device tree infomation get */
	pdata->reset_gpio = of_get_named_gpio(np, "hideep3d,reset-gpio", 0);
	HIDEEP3D_DBG("reset_gpio = %d, is %s specified", pdata->reset_gpio, pdata->reset_gpio < 0 ? "not" : "");

	of_property_read_string(np, "hideep,project_name",&pdata->project_name);

	pdata->pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR(pdata->pinctrl)) {
		HIDEEP3D_ERR("can not get pinctrl!!!");
		return PTR_ERR(pdata->pinctrl);
	}

	pdata->reset_down = pinctrl_lookup_state(pdata->pinctrl, "pmx_ft_suspend");
	if (IS_ERR(pdata->reset_down)) {
		HIDEEP3D_ERR("can not get reset_down state");
	}

	pdata->reset_up = pinctrl_lookup_state(pdata->pinctrl, "pmx_ft_active");
	if (IS_ERR(pdata->reset_up)) {
		HIDEEP3D_ERR("can not get reset_up state");
	}

	pdata->regulator_vdd = HIDEEP_REGULATOR_VDD;
	pdata->regulator_vid = HIDEEP_REGULATOR_VID;
	return ret;
}

static int hideep3d_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret= 0;
	struct hideep_platform_data_t *p_data;
	struct dwz_3dinfo_t *dwz;
	struct hideep3d_t *h3d;

	HIDEEP3D_INFO("enter");
	hideep3d_probe_state = 1;
	/* check i2c bus */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		HIDEEP3D_ERR("check i2c device error");
		ret = -ENODEV;
		return ret;
	}

	/* init platform data */
	if (client->dev.of_node) {
		p_data = devm_kzalloc(&client->dev, sizeof(struct hideep_platform_data_t), GFP_KERNEL);

		if(!p_data) {
			HIDEEP3D_ERR("can't allocate  memory for p_data");
			ret = -ENOMEM;
			goto hideep_probe_pdata_memory_err;
		}
		ret = hideep3d_parse_dts(&client->dev, p_data);
		if (ret)
			return ret;
	} else {
		p_data = client->dev.platform_data;
		if(!p_data)
			return -ENODEV;
	}

	/* init hideep_t */
	h3d = kzalloc(sizeof(struct hideep3d_t), GFP_KERNEL);
	if (!h3d){
		HIDEEP3D_ERR("can't allocate  memory for ts");
		ret = -ENOMEM;
		goto hideep_probe_ts_memory_err;
	}
	dwz = kzalloc(sizeof(struct dwz_3dinfo_t), GFP_KERNEL);

	h3d->client = client;
	h3d->p_data = p_data;
	h3d->dwz_info = dwz;
	i2c_set_clientdata(client, h3d);

	mutex_init(&h3d->i2c_mutex);
	mutex_init(&h3d->dev_mutex);

	hideep3d_init_ic(h3d);

#ifdef HIDEEP_DWZ_VERSION_CHECK
	/* read info */
	ret = hideep3d_load_dwz(h3d);
	if (ret < 0) {
		HIDEEP3D_ERR("fail to load dwz, ret = 0x%x", ret);
		goto hideep_probe_read_dwz_err;
	}
#endif

#ifdef HIDEEP_AUTO_UPDATE
	h3d->p_workqueue_fw = create_singlethread_workqueue("hideep_fw_auto_workqueue");
	INIT_DELAYED_WORK(&h3d->work_fwu, fwu_startup_fw_update_work);
	queue_delayed_work(h3d->p_workqueue_fw, &h3d->work_fwu, msecs_to_jiffies(HIDEEP_UPDATE_FW_THREAD_DELAY));
#endif

#ifdef HIDEEP_DEBUG_DEVICE
	ret = hideep3d_debug_init(h3d);
	if (ret) {
		HIDEEP3D_ERR("fail init debug, ret = 0x%x", ret);
		ret = -1;
		goto hideep_probe_debug_init_err;
	}
#endif

#ifdef HIDEEP_SELFTEST_MODE
	ret = hideep3d_init_proc(h3d);
	if (ret) {
		HIDEEP3D_ERR("fail init procfs, ret = 0x%x", ret);
		ret = -1;
		goto hideep_probe_procfs_init_err;
	}
#endif

	ret = hideep3d_sysfs_init(h3d);
	if (ret) {
		HIDEEP3D_ERR("fail init sys, ret = 0x%x", ret);
		ret = -1;
		goto hideep_probe_sysfs_init_err;
	}

#ifdef CONFIG_FB
	h3d->suspended = 0;
	h3d->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&h3d->fb_notif);
	if (ret) {
		HIDEEP3D_ERR("Unable to register fb_notifier: ret = %d", ret);
		ret = -1;
		goto hideep_probe_register_fb_err;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	h3d->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	h3d->early_suspend.suspend = hideep3d_early_suspend;
	h3d->early_suspend.resume = hideep3d_late_resume;
	register_early_suspend(&h3d->early_suspend);
#endif

	h3d->dev_state = power_normal;
	hideep3d_probe_state = 0;
	g_h3d = h3d;
	HIDEEP3D_INFO("probe is ok!");
	return 0;

hideep_probe_register_fb_err:
	hideep3d_sysfs_exit(h3d);

hideep_probe_sysfs_init_err:
hideep_probe_procfs_init_err:
hideep_probe_debug_init_err:

#ifdef HIDEEP_DWZ_VERSION_CHECK
hideep_probe_read_dwz_err:
#endif
	if (h3d->p_data->regulator_vid != NULL || h3d->p_data->regulator_vdd != NULL)
		hideep3d_power(h3d, false);

	if(h3d)
		kfree(h3d);

hideep_probe_ts_memory_err:
	if(p_data)
		devm_kfree(&client->dev,p_data);

hideep_probe_pdata_memory_err:
	HIDEEP3D_ERR("probe err!");
	return ret;
}

static int hideep3d_remove(struct i2c_client *client)
{
	struct hideep3d_t *h3d = i2c_get_clientdata(client);

#ifdef CONFIG_FB
	if (fb_unregister_client(&h3d->fb_notif))
		HIDEEP3D_ERR("Error occurred while unregistering fb_notifier");
#endif

#ifdef DO_STARTUP_FW_UPDATE
	cancel_delayed_work_sync(&h3d->fwu_work);
	flush_workqueue(h3d->fwu_workqueue);
	destroy_workqueue(h3d->fwu_workqueue);
#endif

	if (h3d->p_data->regulator_vid != NULL || h3d->p_data->regulator_vdd != NULL)
		hideep3d_power(h3d, false);
	hideep3d_reset_ic(h3d);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&h3d->early_suspend);
#endif
	hideep3d_sysfs_exit(h3d);

#ifdef HIDEEP_SELFTEST_MODE
	hideep3d_uninit_proc();
#endif

#ifdef HIDEEP_DEBUG_DEVICE
	hideep3d_debug_uninit(h3d);
#endif
	kfree(h3d);
	return 0;
}

static const struct i2c_device_id hideep3d_dev_idtable[] = {
	{ HIDEEP3D_I2C_NAME, 0 },
	{}
};

static struct of_device_id hideep3d_match_table[] = {
	{.compatible = "hideep3d,hideep_3d"},
	{},
};

#ifdef CONFIG_PM
static const struct dev_pm_ops hideep3d_pm_ops = {
	.suspend = hideep3d_i2c_suspend,
	.resume = hideep3d_i2c_resume,
};
#endif

static struct i2c_driver hideep3d_driver = {
	.probe = hideep3d_probe,
	.remove = hideep3d_remove,
	.id_table = hideep3d_dev_idtable,
	.driver = {
		.name = HIDEEP3D_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hideep3d_match_table),
#ifdef CONFIG_PM
		.pm = &hideep3d_pm_ops,
#endif
	},
};

static int hideep3d_init(void)
{
	int ret;

	HIDEEP3D_DBG("enter");
	ret = i2c_add_driver(&hideep3d_driver);
	if (ret != 0) {
		HIDEEP3D_ERR("unable to add i2c driver.");
	}
	return ret;
}

static void hideep3d_exit(void)
{
	HIDEEP3D_DBG("enter");
	i2c_del_driver(&hideep3d_driver);

	return;
}

module_init(hideep3d_init);
module_exit(hideep3d_exit);

MODULE_DESCRIPTION("Driver for HiDeep 3D Touchscreen Controller");
MODULE_AUTHOR("anthony.kim@hideep.com");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

