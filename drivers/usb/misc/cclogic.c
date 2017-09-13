/*
 * linux/drivers/usb/misc/cclogic.c
 *
 * Copyright 2016 LeTV <www.letv.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file COPYING in the main directory of this
 * archive for more details.
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/usb.h>
#include <linux/cclogic.h>

static struct class *typec_class;
static struct device *typec_dev;
static int cclogic_typec_headset_with_analog = -1;
static struct mutex typec_headset_lock;

#define LETV_USB_AUDIO_VID  0x262A
#define LETV_USB_AUDIO_PID_0  0x1530
#define LETV_USB_AUDIO_PID_1  0x1532
#define LETV_USB_AUDIO_PID_2  0x1534
#define LETV_USB_AUDIO_PID_3  0x1604

static const struct usb_device_id letv_usb_audio_id[] = {
	{ USB_DEVICE(LETV_USB_AUDIO_VID, LETV_USB_AUDIO_PID_0) },
	{ USB_DEVICE(LETV_USB_AUDIO_VID, LETV_USB_AUDIO_PID_1) },
	{ USB_DEVICE(LETV_USB_AUDIO_VID, LETV_USB_AUDIO_PID_2) },
	{ USB_DEVICE(LETV_USB_AUDIO_VID, LETV_USB_AUDIO_PID_3) },
	{ } /* Terminating entry */
};

enum typec_audio_mode {
	TYPEC_AUDIO_USB = 0,
	TYPEC_AUDIO_ANALOG,
};

char *typec_port_state_string[] = {"none", "ufp", "dfp", "audio", "debug"};

typedef enum {
	TYPEC_POLARITY_NONE = 0,
	TYPEC_POLARITY_CC1,
	TYPEC_POLARITY_CC2,
	TYPEC_POLARITY_MAX,
} typec_port_polarity;

char *typec_port_polarity_string[] = {"none", "cc1", "cc2"};

struct usb_cclogic {
	void (*set_audio_mode)(bool mode);
	int usb_audio_mode;
	void (*set_vconn)(bool on);
	struct mutex lock;
	struct delayed_work ad_sel_work;
	int next_state;
	int current_state;
	int wait_smbcharge_init_done;
	typec_port_state port_state;
	typec_port_polarity port_polarity;
	struct regulator *vconn;
	int (*set_vconn_helper)(bool on);
} le_cc;

#define SERIAL_STATE_CMDLINE_MAX 40
static char g_serial_hw_output_state[SERIAL_STATE_CMDLINE_MAX];

static int __init get_bootparam_serial_select_state(char *str)
{
	strlcpy(g_serial_hw_output_state, str, SERIAL_STATE_CMDLINE_MAX);
	return 1;
}
__setup("android.letv.serial_hw_output=", get_bootparam_serial_select_state);

bool serial_hw_output_enable(void)
{
	if(g_serial_hw_output_state != NULL)
	{
		if(!strncmp(g_serial_hw_output_state, "enabled", 7))
		{
			return true;
		}
	}

	return false;
}
EXPORT_SYMBOL(serial_hw_output_enable);

static int check_letv_usb_dev(struct usb_device *udev, void *data)
{
	bool ret = 0;

	if (!udev)
		return 0;

	usb_lock_device(udev);

	//In case of this device has already disconnected
	if (udev->actconfig == NULL)
		goto out;
	if (udev->actconfig->interface[0] == NULL)
		goto out;
	if (udev->actconfig->interface[0]->unregistering)
		goto out;
	if (udev->parent == NULL)
		goto out;

	ret = ((udev->parent->parent == NULL) &&
	       (udev->actconfig->interface[0]) &&
	       (usb_match_id(udev->actconfig->interface[0], letv_usb_audio_id)));

	if (ret) {
		if (data)
			*(u64 *)data = (u64)udev;
	}
out:
	usb_unlock_device(udev);
	return ret;
}

/**
* @data: reserved for future usage
* Iterate over all USB devices. If there is only one Letv usb
* audio device on bus returns 1 otherwise returns 0
*/
int letv_audio_mode_supported(void *data)
{
	return usb_for_each_dev(data, check_letv_usb_dev);
}
EXPORT_SYMBOL(letv_audio_mode_supported);

void cclogic_set_audio_mode_register(void (*func)(bool))
{
	mutex_lock(&le_cc.lock);
	le_cc.set_audio_mode = func;
	mutex_unlock(&le_cc.lock);
}
EXPORT_SYMBOL(cclogic_set_audio_mode_register);

void cclogic_set_typec_headset_with_analog(int val)
{
	mutex_lock(&typec_headset_lock);
	cclogic_typec_headset_with_analog = val;
	mutex_unlock(&typec_headset_lock);
}
EXPORT_SYMBOL(cclogic_set_typec_headset_with_analog);

static int cclogic_typec_headset_with_analog_parm_set(char *buffer, struct kernel_param *kp)
{
	if (cclogic_typec_headset_with_analog == 1) {
		return sprintf(buffer, "1");
	} else if (cclogic_typec_headset_with_analog == 0) {
		return sprintf(buffer, "0");
	} else {
		return sprintf(buffer, "-1");
	}
	return 0;
}

module_param_call(cclogic_typec_headset_with_analog, NULL,
		cclogic_typec_headset_with_analog_parm_set, &cclogic_typec_headset_with_analog, 0664);

void cclogic_set_vconn_register(void (*func)(bool))
{
	mutex_lock(&le_cc.lock);
	le_cc.set_vconn = func;
	mutex_unlock(&le_cc.lock);
}
EXPORT_SYMBOL(cclogic_set_vconn_register);

static int cclogic_reset_8904(void)
{
	struct usb_device *udev;
	int ret = 0;

	if (letv_audio_mode_supported(&udev))
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x0a,
				      0x40, 0x0, 0x0, NULL, 0, 100);
	if (ret)
		pr_err("%s failed, err %d\n", __func__, ret);

	return 0;
}
int cclogic_set_vbus(bool on);

/*This func is for headset switch AD mode*/
void cclogic_set_audio_mode(bool mode)
{
	if ((mode == TYPEC_AUDIO_ANALOG) && !letv_audio_mode_supported(NULL))
		return;

	/* 
	 * @mode TRUE for Analog mode
	 *	 FALSE for Digit mode
	 */
	if (le_cc.set_audio_mode) {
		/* Digit -> Analog */
		if (mode == TYPEC_AUDIO_ANALOG) {
			/*
			 * Find USB device then send message to RESET 8904
			 */
			 cclogic_reset_8904();
		} else {
			cclogic_set_vbus(0);
		}

		le_cc.set_audio_mode(mode);

		if (mode == TYPEC_AUDIO_USB)
			cclogic_set_vbus(1);
	}
	le_cc.usb_audio_mode = mode;

	return;
}
EXPORT_SYMBOL(cclogic_set_audio_mode);

int cclogic_get_audio_mode(void)
{
	return le_cc.usb_audio_mode;
}
EXPORT_SYMBOL(cclogic_get_audio_mode);

static void ad_sel_work(struct work_struct *work)
{

	mutex_lock(&le_cc.lock);
	if (le_cc.current_state != le_cc.next_state) {
		cclogic_set_audio_mode(le_cc.next_state);
		le_cc.current_state = le_cc.next_state;
	}
	mutex_unlock(&le_cc.lock);
}

void cclogic_set_audio_mode_delay(bool mode, u32 delay_in_ms)
{
	mutex_lock(&le_cc.lock);
	cancel_delayed_work_sync(&le_cc.ad_sel_work);
	le_cc.next_state = mode;
	schedule_delayed_work(&le_cc.ad_sel_work,
			      msecs_to_jiffies(delay_in_ms));
	mutex_unlock(&le_cc.lock);
}
EXPORT_SYMBOL(cclogic_set_audio_mode_delay);

/*This func is for headset switch AD mode*/
void cclogic_set_vconn(bool mode)
{
/*true for Vconn 5V supply enable*/
	if (le_cc.set_vconn)
		le_cc.set_vconn(mode);
}
EXPORT_SYMBOL(cclogic_set_vconn);

void cclogic_set_smbcharge_init_done(int done)
{
	le_cc.wait_smbcharge_init_done = done;
}
EXPORT_SYMBOL(cclogic_set_smbcharge_init_done);

int cclogic_get_smbcharge_init_done(void)
{
	return le_cc.wait_smbcharge_init_done;
}
EXPORT_SYMBOL(cclogic_get_smbcharge_init_done);

void cclogic_updata_port_state(typec_port_state state)
{
	if (state >= TYPEC_PORT_MAX)
		return;

	mutex_lock(&le_cc.lock);
	le_cc.port_state = state;
	mutex_unlock(&le_cc.lock);
}
EXPORT_SYMBOL(cclogic_updata_port_state);

typec_port_state cclogic_get_port_state(void)
{
	return le_cc.port_state;
}
EXPORT_SYMBOL(cclogic_get_port_state);

extern int msm_usb_vbus_set(void *_mdwc, bool on, bool ext_call);
int cclogic_set_vbus(bool on)
{
	return msm_usb_vbus_set(NULL, on, true);
}

static ssize_t cclogic_state_show(struct device *pdev,
			       struct device_attribute *attr, char *buf)
{
	struct usb_cclogic *le_cc = dev_get_drvdata(pdev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			typec_port_state_string[le_cc->port_state]);
}

static DEVICE_ATTR(cc_state, S_IRUGO, cclogic_state_show, NULL);

void cclogic_updata_port_polarity(typec_port_polarity polarity)
{
	if (polarity >= TYPEC_POLARITY_MAX)
		return;

	mutex_lock(&le_cc.lock);
	le_cc.port_polarity = polarity;
	mutex_unlock(&le_cc.lock);
}
EXPORT_SYMBOL(cclogic_updata_port_polarity);

static ssize_t cclogic_polarity_show(struct device *pdev,
			       struct device_attribute *attr, char *buf)
{
	struct usb_cclogic *le_cc = dev_get_drvdata(pdev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			typec_port_polarity_string[le_cc->port_polarity]);
}

static DEVICE_ATTR(cc_polarity, S_IRUGO, cclogic_polarity_show, NULL);


static ssize_t letv_supported_dev_show(struct device *pdev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",
			letv_audio_mode_supported(NULL));
}

static DEVICE_ATTR(supported_dev, S_IRUGO, letv_supported_dev_show,
			 NULL);

static int __init cclocic_init(void)
{
	int err;

	mutex_init(&typec_headset_lock);
	cclogic_typec_headset_with_analog = -1;
	mutex_init(&le_cc.lock);
	INIT_DELAYED_WORK(&le_cc.ad_sel_work, ad_sel_work);
	le_cc.next_state = le_cc.current_state = TYPEC_AUDIO_USB;
	le_cc.port_state = 0;
	le_cc.port_polarity = 0;
	typec_class = class_create(THIS_MODULE, "typec");
	if (IS_ERR(typec_class)) {
		pr_err("failed to create typec class --> %ld\n",
		       PTR_ERR(typec_class));
		return PTR_ERR(typec_class);
	}
	typec_dev = device_create(typec_class, NULL, MKDEV(0, 0), &le_cc,
				  "typec_device");
	if (IS_ERR(typec_dev)) {
		pr_err("%s: device_create fail\n", __func__);
		return PTR_ERR(typec_dev);
	}

	err = device_create_file(typec_dev, &dev_attr_cc_state);
	err = device_create_file(typec_dev, &dev_attr_cc_polarity);
	err |= device_create_file(typec_dev, &dev_attr_supported_dev);
	if (err) {
		pr_err("%s: device_create_file fail\n", __func__);
		device_destroy(typec_class, typec_dev->devt);
		return err;
	}

	return 0;
}
subsys_initcall(cclocic_init);

static void __exit cclocic_exit(void)
{
	class_destroy(typec_class);
}
module_exit(cclocic_exit);
MODULE_DEVICE_TABLE(usb, letv_usb_audio_id);

