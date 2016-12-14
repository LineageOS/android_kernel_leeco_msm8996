#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>

#include "pi5usb30216a.h"

#define DRIVER_NAME "pi5usb"


struct pi5usb_chip_info *pi5usb30216a_chip_info = NULL;
struct pi5usb_device_info *pi5usb30216a_device_info = NULL;
/*The follow code just for build sucessful, it's will be release  in later*/
/*
extern int start_usb_DRPMode(void);
extern int start_usb_DFPMode(void);
extern int start_usb_UFPMode(void);
extern int start_usb_DebugMode(void);
*/

#define PI5USB_REG2_ROLE_MODE_MASK	0x6
#define PI5USB_REG2_INTERRUPT_MASK	0x1

extern int pi5usb_set_msm_usb_host_mode(bool mode);
/*
Write data to chip
reg: register address
value: write's value from register address
return value:
0: write sucess
< 0: read fail

*/
/*The follow code just for build sucessful, it's will be delete in later*/
int start_usb_DRPMode(void)
{
	printk("%s:\n", __func__);
	return 0;
}

int start_usb_DFPMode(bool mode)
{
	printk("%s:\n", __func__);
	pi5usb_set_msm_usb_host_mode(mode);
	return 0;
}

int start_usb_UFPMode(void)
{
	printk("%s:\n", __func__);
	return 0;
}

int start_usb_DebugMode(bool debug_mode)
{
	struct pi5usb_chip_info *chip_info;

	chip_info = pi5usb30216a_chip_info;
	printk("%s: debug_mode=%d\n", __func__, debug_mode ? 1 : 0);
	if (debug_mode) {
		gpio_set_value(chip_info->id_gpio, 0);
	} else {
		printk("%s: will switch usbid mode\n", __func__);
		gpio_set_value(chip_info->id_gpio, 1);
	}
	return 0;
}




int start_usb_AudioMode(void)
{
	printk("%s:\n", __func__);
	return 0;
}

static int pi5usb30216a_write_bytes(u8 *value, int count)
{
	int i;
	u8 reg[4] = {0};
	int cnt;
	int ret = NORMAL_ERROR;
	struct i2c_msg msg[1];
	struct pi5usb_device_info *dev_info;

	if ((value == NULL) || (pi5usb30216a_device_info == NULL))
		return ret;

	dev_info = pi5usb30216a_device_info;
/*
		if (dev_info->enabled == 0)
		return ret;
*/
	if ((!(dev_info->i2c_client)) ||
		(!(dev_info->i2c_client->adapter)))
		return -ENODEV;

	cnt = count;
	printk("%s: reg =", __func__);
	for (i = 0; i < cnt; i++) {
		reg[i] = *(value + i);
		printk("0x%x, ", reg[i]);
	}
	printk("\n");

	msg[0].addr = dev_info->i2c_client->addr;
	msg[0].flags = 0;
	msg[0].buf = reg;
	msg[0].len = cnt;
	ret = i2c_transfer(dev_info->i2c_client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return 0;
}

/*
Read data from chip
value: read's value from register address
count: read byte number
return value:
0: read's sucess
< 0: read fail
*/
static int pi5usb30216a_read_bytes(u8 *value, int count)
{
	int i;
	int ret = NORMAL_ERROR;
	struct i2c_msg msg[1];
	struct pi5usb_device_info *dev_info;

	if ((value == NULL) || (pi5usb30216a_device_info == NULL))
		return ret;

	dev_info = pi5usb30216a_device_info;
/*
	if (dev_info->enabled == 0)
		return ret;
*/
	if ((NULL == dev_info->i2c_client) ||
		(NULL == dev_info->i2c_client->adapter))
		return -ENODEV;

	for (i = 0; i < count; i++)
		*(value + i) = 0;

	msg[0].addr = dev_info->i2c_client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].buf = value;
	msg[0].len = count;

	ret = i2c_transfer(dev_info->i2c_client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return ret;
}

static int pi5usb_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int pi5usb_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct pi5usb_device_info *dev_info;
	printk("%s:\n", __func__);
	dev_info = pi5usb30216a_device_info;
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
			"%s: SMBus byte data commands not supported by host\n",
			__func__);
		return -EIO;
	}
	dev_info->i2c_client = client;

	return 0;
}

static const struct i2c_device_id pi5usb_i2c_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, pi5usb_i2c_id_table);

#ifdef CONFIG_OF
static struct of_device_id pi5usb_match_table[] = {
	{ .compatible = "fairchild,pi5usb",},
	{ },
};
#else
#define pi5usb_match_table NULL
#endif

static struct i2c_driver pi5usb_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = pi5usb_match_table,
	},
	.probe = pi5usb_i2c_probe,
	.remove = pi5usb_i2c_remove,
	.id_table = pi5usb_i2c_id_table,
};

static int pi5usb30216a_init_i2c(void)
{
	return i2c_add_driver(&pi5usb_i2c_driver);
}

/*
enable:
true: enable chip
false: disable chip
*/
static int pi5usb30216a_enable_chip(bool enable)
{
	struct pi5usb_device_info *dev_info;

	dev_info = pi5usb30216a_device_info;
    /*because enb_gpio is always GND*/
	if (enable) {
		dev_info->enabled = 1;
	} else {
		dev_info->enabled = 0;
	}
	return 0;
}

/*
enable value:
true: enable interrupt.
false: disable interrupt
Notice:
When call this function, you need add delay 30ms after it.
*/
static int pi5usb30216a_enable_interrupt(bool enable)
{
	u8 reg[4] = {0};

	pi5usb30216a_read_bytes(reg, 4);
	printk("%s: reg=0x%x,0x%x,0x%x,0x%x\n",__func__,
		reg[0], reg[1],reg[2], reg[3]);
	if (!enable) {    /* 0.Mask interrupt */
		if (reg[1] & PI5USB_REG2_INTERRUPT_MASK)
			return 0;
		reg[1] = reg[1] | PI5USB_REG2_INTERRUPT_MASK;
		pi5usb30216a_write_bytes(reg, 2);
	} else {  /*enable interrut*/
		if (!(reg[1] & PI5USB_REG2_INTERRUPT_MASK))
			return 0;
		reg[1] = reg[1] & (~PI5USB_REG2_INTERRUPT_MASK);
		pi5usb30216a_write_bytes(reg, 2);
	}

	return 0;
}

static int pi5usb30216a_init_chip(void)
{
	u8 reg[4] = {0};
	struct pi5usb_device_info *dev_info;

	dev_info = pi5usb30216a_device_info;
	printk("%s:\n",__func__);
	pi5usb30216a_enable_chip(true);

	reg[1] = dev_info->usb_mode | PI5USB_REG2_INTERRUPT_MASK;
	pi5usb30216a_write_bytes(reg, 2);
	mdelay(30);

	reg[1] = dev_info->usb_mode;
	pi5usb30216a_write_bytes(reg, 2);
	mdelay(10);

	return 0;
}

static ssize_t pi5usb_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pi5usb_device_info *dev_info;
	u8 reg[4] = {0};

	pi5usb30216a_read_bytes(reg, 4);
	printk("%s: reg=0x%x,0x%x,0x%x,0x%x\n",__func__,
		reg[0], reg[1],reg[2], reg[3]);

	dev_info = pi5usb30216a_device_info;

	return snprintf(buf, PAGE_SIZE, "%s\n", dev_info->enabled?
							"enabled" : "disabled");
}

static ssize_t pi5usb_enable_store(struct device *dev,
		struct device_attribute *attr, const char*buf,
		size_t size)
{
	struct pi5usb_device_info *dev_info;

	dev_info = pi5usb30216a_device_info;
	printk("%s:buf=%s", __func__, buf);

	if (!strncmp(buf, "enable", 6)) {
		pi5usb30216a_init_chip();
		return size;
	} else if (!strncmp(buf, "disable", 7)) {
		pi5usb30216a_enable_chip(false);
		return size;
	}

	return -EINVAL;
}

static DEVICE_ATTR(pi5usb_enable, S_IRUGO | S_IWUSR,
		pi5usb_enable_show, pi5usb_enable_store);

static void pi5usb30216a_irq_handler(struct work_struct *w)
{
	u8 reg[4] = {0};
	struct pi5usb_device_info *dev_info;
	dev_info = pi5usb30216a_device_info;

	printk("%s:\n", __func__);
	pi5usb30216a_enable_interrupt(false);
	msleep(30);

	pi5usb30216a_read_bytes(reg, 4);
	if (reg[3] == 0x00) {
		printk("%s: clear try_sink\n", __func__);
		dev_info->usb_mode_try_sink = 0;
	}
	printk("%s:, reg[3]=0x%x\n", __func__, reg[3]);

	/*Unattached*/
	if ((reg[3] == 0x00) || (reg[3]==0x01) || (reg[3] == 0x80) ){ //detached
		if (dev_info->usb_mode == DFP_MODE)
			start_usb_DFPMode(false);
		if (dev_info->usb_mode == DEBUG_MODE)
			start_usb_DebugMode(false);
	
		start_usb_DRPMode();
		dev_info->usb_mode = DRP_MODE; //enter drp mode
	}

	/*Debug accessory attached*/
	if ((reg[3] == 0x13) || (reg[3] == 0x93)) {
		start_usb_DebugMode(true);
		dev_info->usb_mode = DEBUG_MODE;
	}

	/*Audio accessory attached*/
	if ((reg[3] == 0x0f) || (reg[3] == 0x8f)) {
		start_usb_AudioMode();
	}

	/* Host attached */
	if (reg[3] == 0xa9 || reg[3] == 0xaa || reg[3] == 0xc9
		|| reg[3] == 0xca || reg[3] == 0xe9 || reg[3] == 0xea) {
		start_usb_UFPMode();
		dev_info->usb_mode = UFP_MODE;
	}

	/*device attached*/
	if (reg[3] == 0x05 || reg[3] == 0x06) {
		start_usb_DFPMode(true);
		dev_info->usb_mode = DFP_MODE;
/*
		if(dev_info->usb_mode_try_sink == 1)
		{
			 start_usb_DFPMode();
			 dev_info->usb_mode = DFP_MODE;
		} else if ((dev_info->usb_mode_try_sink == 0)) {
			u8 reg_tmp[4] = {0};
			dev_info->usb_mode = UFP_MODE;
			reg_tmp[1] |= PI5USB_REG2_INTERRUPT_MASK;
			pi5usb30216a_write_bytes(reg_tmp, 2);
			msleep(500);

			pi5usb30216a_read_bytes(reg_tmp, 4);
			if(reg_tmp[3] == 0xa9 || reg_tmp[3] == 0xaa ||
				reg_tmp[3] == 0xc9 || reg_tmp[3] == 0xca ||
				reg_tmp[3] == 0xe9 || reg_tmp[3] == 0xea) {
				start_usb_UFPMode();
				dev_info->usb_mode = UFP_MODE;
			} else
				dev_info->usb_mode = DRP_MODE;

			 dev_info->usb_mode_try_sink = 1;
		}
*/
	}
	mdelay(20);
	pi5usb30216a_enable_interrupt(true);
}

/*
Handle the irq
*/
static irqreturn_t pi5usb30216a_irq(int irq, void *info)
{
	struct pi5usb_device_info *dev_info;
	printk("%s:\n",__func__);

	if (!pi5usb30216a_device_info) {
		return IRQ_HANDLED;
	}
	dev_info = pi5usb30216a_device_info;

	queue_work(dev_info->pi5usb_wq, &dev_info->q_work);
	return IRQ_HANDLED;
}

/*
Get pi5usb30216a's chip info
*/
static int pi5usb30216a_parse_dt(void)
{
	if (!pi5usb30216a_chip_info)
		return NORMAL_ERROR;

	pi5usb30216a_chip_info->port_gpio_set = 1;
	pi5usb30216a_chip_info->addr_gpio_set = 1;
	pi5usb30216a_chip_info->id_gpio = 0;
	pi5usb30216a_chip_info->enb_gpio = 0;

	return RET_OK;
}

static int pi5usb30216a_init_devinfo(void)
{
	int ret = NORMAL_ERROR;
	struct pi5usb_chip_info *chip_info;
	struct pi5usb_device_info *dev_info;

	if ((!pi5usb30216a_device_info) ||
			(!pi5usb30216a_chip_info))
		return NORMAL_ERROR;

	dev_info = pi5usb30216a_device_info;
	chip_info = pi5usb30216a_chip_info;
	dev_info->chip_info = pi5usb30216a_chip_info;

	ret = alloc_chrdev_region(&dev_info->pi5usb_dev_t, 0, 1, "pi5usb_dev_t");
	if (ret < 0) {
		pr_err("%s:Fail to alloc char dev region\n", __func__);
		return ret;
	}

	dev_info->pi5usb_class = class_create(THIS_MODULE, "pi5usb_class");
	if (IS_ERR(dev_info->pi5usb_class)) {
		printk("%s: create class fail \n", __func__);
		return NORMAL_ERROR;
	}

	dev_info->pi5usb_device = device_create(dev_info->pi5usb_class,
					NULL, dev_info->pi5usb_dev_t, NULL,
					"pi5usb_device");


	switch (chip_info->port_gpio_set)
	{
	case 0:
		dev_info->usb_mode = UFP_MODE;
		break;
	case 1:
		dev_info->usb_mode = DRP_MODE;
		break;
	case 2:
		dev_info->usb_mode = DFP_MODE;
		break;
	default:
		dev_info->usb_mode = DRP_MODE;
		break;
	}

	switch (chip_info->addr_gpio_set)
	{
	case 1:
		dev_info->i2c_address = 0x1D;
		break;
	case 0:
		dev_info->i2c_address = 0x1D;
		break;
	default:
		dev_info->i2c_address = 0x1D;
		break;
	}

	dev_info->pinctrl = devm_pinctrl_get(&dev_info->i2c_client->dev);
	if (IS_ERR_OR_NULL(dev_info->pinctrl)) {
		printk("%s: Unable to get pinctrl handle\n", __func__);
	}
	dev_info->intr_active = pinctrl_lookup_state(dev_info->pinctrl,
					"pi5usb_active");
	if (IS_ERR(dev_info->intr_active)) {
		printk("%s: could not get intr_active pinstate\n", __func__);
	}

	ret = pinctrl_select_state(dev_info->pinctrl,
					dev_info->intr_active);
	if (ret != 0) {
		printk("%s: Disable TLMM pins failed with %d\n",
			__func__, ret);
	}

	ret = gpio_request(chip_info->intb_gpio, "pi5usb_irq_gpio");
	if (ret)
		printk("%s:gpio_request error, intb_gpio=%d\n",
			__func__,chip_info->intb_gpio);
	ret = gpio_direction_input(chip_info->intb_gpio);
	if (ret) {
		printk("%s:set gpio input direction error, intb_gpio=%d\n",
			__func__,chip_info->intb_gpio);
		goto release_irq_gpio;
	}
	dev_info->irq = gpio_to_irq(chip_info->intb_gpio);
	printk("%s: irq = %d\n",__func__, dev_info->irq);
	dev_info->pi5usb_wq = alloc_ordered_workqueue("pi5usb_wq", 0);
	if (!dev_info->pi5usb_wq) {
		pr_err("%s: Unable to create workqueue otg_wq\n",
				__func__);
		goto release_irq_gpio;
	}
	INIT_WORK(&dev_info->q_work, pi5usb30216a_irq_handler);

	return ret;

release_irq_gpio:
	gpio_free(chip_info->intb_gpio);
	return ret;
}


#ifdef CONFIG_PM_SLEEP
static int pi5usb_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct pi5usb_device_info *dev_info;
	dev_info = pi5usb30216a_device_info;

	enable_irq_wake(dev_info->irq);

	return ret;
}

static int pi5usb_pm_resume(struct device *dev)
{
	int ret = 0;

	struct pi5usb_device_info *dev_info;
	dev_info = pi5usb30216a_device_info;

	disable_irq_wake(dev_info->irq);
	return ret;
}

static const struct dev_pm_ops pi5usb_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pi5usb_pm_suspend, pi5usb_pm_resume)
};
#endif /* CONFIG_PM_SLEEP */

static int pi5usb30216a_check_debugmode(void)
{
	u8 reg[4] = {0};

	pi5usb30216a_read_bytes(reg, 4);
	/*Debug accessory attached*/
	if ((reg[3] == 0x13) || (reg[3] == 0x93)) {
		start_usb_DebugMode(true);
	} else
		start_usb_DebugMode(false);

	return 0;
}

static int pi5usb_probe(struct platform_device *pdev)
{
	int ret = NORMAL_ERROR;
	struct pi5usb_chip_info *chip_info;
	struct pi5usb_device_info *dev_info;
	pr_info("%s\n", __func__);

	if ((pi5usb30216a_chip_info != NULL) ||
		(pi5usb30216a_device_info != NULL))
		return ret;

	pi5usb30216a_chip_info = kzalloc(sizeof(struct pi5usb_chip_info), GFP_KERNEL);
	if (!pi5usb30216a_chip_info) {
		ret = -ENOMEM;
		goto malloc_error;
	}
	chip_info = pi5usb30216a_chip_info;

	pi5usb30216a_device_info = kzalloc(sizeof(struct pi5usb_device_info), GFP_KERNEL);
	if (!pi5usb30216a_device_info) {
		ret = -ENOMEM;
		goto malloc_error;
	}
	dev_info = pi5usb30216a_device_info;

	ret = pi5usb30216a_parse_dt();
	if (ret)
		goto malloc_error;

	ret = pi5usb30216a_init_i2c();
	if (ret) {
		printk("%s: init i2c error\n", __func__);
		goto malloc_error;
	}

	pi5usb30216a_chip_info->intb_gpio =
		of_get_named_gpio_flags(dev_info->i2c_client->dev.of_node,
		"irq-gpio", 0, NULL);
	if (gpio_is_valid(pi5usb30216a_chip_info->intb_gpio))
		printk("%s:irq gpio=%d\n",__func__, pi5usb30216a_chip_info->intb_gpio);

	pi5usb30216a_chip_info->id_gpio =
		of_get_named_gpio_flags(dev_info->i2c_client->dev.of_node,
			"qcom,id-gpio", 0, NULL);
	if (gpio_is_valid(pi5usb30216a_chip_info->id_gpio))
		printk("%s: id gpio=%d\n",__func__, pi5usb30216a_chip_info->id_gpio);

	gpio_direction_output(chip_info->id_gpio, 0);

	ret = pi5usb30216a_init_devinfo();
	if (ret)
		goto malloc_error;

	device_create_file(dev_info->pi5usb_device, &dev_attr_pi5usb_enable);
	pi5usb30216a_enable_interrupt(false);
	ret = request_irq(dev_info->irq, pi5usb30216a_irq,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "pi5usb_irq", NULL);
	if (ret) {
		printk("%s:Cannot request irq\n", __func__);
		goto malloc_error;
	}
	pi5usb30216a_check_debugmode();
	pi5usb30216a_init_chip();

	return ret;
malloc_error:
	if (pi5usb30216a_chip_info)
		kfree(pi5usb30216a_chip_info);
	if (pi5usb30216a_device_info)
		kfree(pi5usb30216a_device_info);

	return ret;
}

static int pi5usb_remove(struct platform_device *pdev)
{
	struct pi5usb_device_info *dev_info;
	dev_info = pi5usb30216a_device_info;

	free_irq(dev_info->irq, NULL);
	pi5usb30216a_enable_chip(false);

	if (pi5usb30216a_chip_info != NULL) {
		kfree(pi5usb30216a_chip_info);
	}
	if (pi5usb30216a_device_info != NULL)
		dev_info = pi5usb30216a_device_info;

	device_remove_file(dev_info->pi5usb_device,
						&dev_attr_pi5usb_enable);
	return 0;
}

static struct of_device_id pi5usb_dt_match[] = {
	{
		.compatible = "letv,pi5usb_driver",
	},
	{}
};

static struct platform_driver pi5usb_platform_driver = {
	.driver = {
		.name = "pi5usb_driver",
		.of_match_table = pi5usb_dt_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &pi5usb_pm_ops,
#endif
	},
	.probe = pi5usb_probe,
	.remove = pi5usb_remove,
};

static int __init pi5usb30216a_init(void)
{
	int ret = NORMAL_ERROR;

	ret = platform_driver_register(&pi5usb_platform_driver);
	if (ret) {
		pr_err("%s(): Failed to register pi5usb"
				 "platform driver\n", __func__);
	}
	return ret;
}

static void __exit pi5usb30216a_exit(void)
{
	platform_driver_unregister(&pi5usb_platform_driver);
}


late_initcall(pi5usb30216a_init);
module_exit(pi5usb30216a_exit);

MODULE_AUTHOR("Letv, Inc.");
MODULE_DESCRIPTION("Letv PI5USB30216A Support Module");
MODULE_LICENSE("GPL v2");
