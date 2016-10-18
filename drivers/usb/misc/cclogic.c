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
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <linux/usb.h>

static struct class *typec_class;
static struct device *typec_dev;
extern int letv_cdla_with_analog;

#define CDLA_DEV_MATCH_ID		0x0000
#define CDLA_DEV_MATCH_ID_RANGE		0x0001
#define CDLA_DEV_PID_HI_SHIFT		16
#define LETV_CDLA_VID			0x262A

struct cdla_id {
	u16		match_flags;
	u16		idVendor;
	u32		idProduct;
};

#define CDLA_DEV_ID(vend, prod) \
	.match_flags = CDLA_DEV_MATCH_ID, \
	.idVendor = (vend), \
	.idProduct = (prod)

#define CDLA_DEV_PID_LO(prod) ((u16)(prod))
#define CDLA_DEV_PID_HI(prod) ((u16)((prod) >> CDLA_DEV_PID_HI_SHIFT))

#define CDLA_DEV_ID_RANGE(vend, prod_lo, prod_hi) \
		.match_flags = CDLA_DEV_MATCH_ID_RANGE, \
		.idVendor = (vend), \
		.idProduct = ((u16)(prod_lo) | ((u16)(prod_hi) << \
			      CDLA_DEV_PID_HI_SHIFT))

static const struct cdla_id cdla_id_table[] = {
	{ CDLA_DEV_ID(LETV_CDLA_VID, 0x1530) },
	{ CDLA_DEV_ID(LETV_CDLA_VID, 0x1532) },
	{ CDLA_DEV_ID(LETV_CDLA_VID, 0x1534) },
	{ CDLA_DEV_ID_RANGE(LETV_CDLA_VID, (u16)0x1600, (u16)0x169f) },
	{ } /* Terminating entry */
};

enum typec_audio_mode {
	TYPEC_AUDIO_USB = 0,
	TYPEC_AUDIO_ANALOG,
};

typedef enum {
	TYPEC_PORT_NONE = 0,
	TYPEC_PORT_UFP,
	TYPEC_PORT_DFP,
	TYPEC_PORT_AUDIO,
	TYPEC_PORT_DEBUG,
	TYPEC_PORT_MAX,
} typec_port_state;

char *typec_port_state_string[] = {"none", "ufp", "dfp", "audio", "debug"};

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
	struct regulator *vconn;
	bool suspend_vbus_on;
	int (*set_vconn_helper)(bool on);
} le_cc;

static bool cdla_match_id(struct usb_device *udev, const struct cdla_id *id)
{

	if (id == NULL)
		return NULL;

	for (; id->idVendor || id->idProduct; id++) {
		if (le16_to_cpu(udev->descriptor.idVendor) != id->idVendor)
			return 0;

		if (id->match_flags == CDLA_DEV_MATCH_ID &&
		    le16_to_cpu(udev->descriptor.idProduct) == id->idProduct)
			return 1;

		if (id->match_flags == CDLA_DEV_MATCH_ID_RANGE &&
		    le16_to_cpu(udev->descriptor.idProduct) >= CDLA_DEV_PID_LO(id->idProduct) &&
		    le16_to_cpu(udev->descriptor.idProduct) <= CDLA_DEV_PID_HI(id->idProduct))
			return 1;
	}

	return 0;
}

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
	       cdla_match_id(udev, cdla_id_table));

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
	int ret;

	if (letv_audio_mode_supported(&udev)) {
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x0a,
				      0x40, 0x0, 0x0, NULL, 0, 100);
		msleep(50);
	}
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

bool cclogic_suspend_vbus_on(void)
{
	return le_cc.suspend_vbus_on;
}
EXPORT_SYMBOL(cclogic_suspend_vbus_on);

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

static ssize_t letv_supported_dev_show(struct device *pdev,
			       struct device_attribute *attr, char *buf)
{
	int letv_cdla_connected;

	letv_cdla_connected = (letv_audio_mode_supported(NULL) ||
		(letv_cdla_with_analog == 1 || letv_cdla_with_analog == 0));
	return snprintf(buf, PAGE_SIZE, "%d\n", letv_cdla_connected);
}

static ssize_t letv_supported_dev_store(struct device *pdev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct usb_cclogic *le_cc = dev_get_drvdata(pdev);

	if (!strcmp("suspend_vbus_on", strim((char *)buf)))
		le_cc->suspend_vbus_on = 1;
	if (!strcmp("suspend_vbus_off", strim((char *)buf)))
		le_cc->suspend_vbus_on = 0;
	return size;
}

static DEVICE_ATTR(supported_dev, S_IRWXUGO, letv_supported_dev_show,
			 letv_supported_dev_store);

static int cdla_notify(struct notifier_block *self, unsigned long action,
		      void *dev)
{
	struct usb_device *udev = (struct usb_device *)dev;

	switch (action) {
	case USB_DEVICE_ADD:
		if (cdla_match_id(udev, cdla_id_table)) {
			usb_enable_autosuspend(udev);
			pm_runtime_set_autosuspend_delay(&udev->dev, 600 * 1000);
		}
		break;
	case USB_DEVICE_REMOVE:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block cdla_nb = {
	.notifier_call = 	cdla_notify,
};

static int __init cclocic_init(void)
{
	int err;

	mutex_init(&le_cc.lock);
	INIT_DELAYED_WORK(&le_cc.ad_sel_work, ad_sel_work);
	le_cc.next_state = le_cc.current_state = TYPEC_AUDIO_USB;
	le_cc.port_state = 0;
	le_cc.suspend_vbus_on = 0;
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
	err |= device_create_file(typec_dev, &dev_attr_supported_dev);
	if (err) {
		pr_err("%s: device_create_file fail\n", __func__);
		device_destroy(typec_class, typec_dev->devt);
		return err;
	}

	usb_register_notify(&cdla_nb);

	return 0;
}
subsys_initcall(cclocic_init);

static void __exit cclocic_exit(void)
{
	usb_unregister_notify(&cdla_nb);
	class_destroy(typec_class);
}
module_exit(cclocic_exit);
MODULE_DEVICE_TABLE(usb, letv_usb_audio_id);

