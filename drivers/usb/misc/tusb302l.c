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

#include "tusb302l.h"

#define DRIVER_NAME "tusb302l"


struct tusb302l_chip_info *tusb302l_chip_info = NULL;
struct tusb302l_device_info *tusb302l_device_info = NULL;

#define TUSB302L_REG2_ROLE_MODE_MASK	0x6
#define TUSB302L_REG2_INTERRUPT_MASK	0x1

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
int tusb302l_start_usb_DRPMode(void)
{
	printk("%s:\n", __func__);
	return 0;
}

int tusb302l_start_usb_DFPMode(bool mode)
{
	printk("%s:\n", __func__);
	pi5usb_set_msm_usb_host_mode(mode);
	return 0;
}

int tusb302l_start_usb_UFPMode(void)
{
	printk("%s:\n", __func__);
	return 0;
}

int tusb302l_start_usb_DebugMode(bool debug_mode)
{
	struct tusb302l_chip_info *chip_info;

	chip_info = tusb302l_chip_info;
	printk("%s: debug_mode=%d\n", __func__, debug_mode ? 1 : 0);
	if (debug_mode) {
		gpio_set_value(chip_info->id_gpio, 0);
	} else {
		printk("%s: will switch usbid mode\n", __func__);
		gpio_set_value(chip_info->id_gpio, 1);
	}
	return 0;
}




int tusb302l_start_usb_AudioMode(void)
{
	printk("%s:\n", __func__);
	return 0;
}

/*
write one byte to chip
start_reg: the register address for write
value: write value
return ret:
0: read's sucess
< 0: read fail
*/
static int tusb302l_write_byte(u8 start_reg, u8 value)
{
	int ret = NORMAL_ERROR;
	u8 data[2];
	struct i2c_msg msg[1];
	struct tusb302l_device_info *dev_info;

	if (start_reg < 0)
		return ret;
	if ((tusb302l_device_info == NULL))
		return ret;

	dev_info = tusb302l_device_info;
	if ((NULL == dev_info->i2c_client) ||
		(NULL == dev_info->i2c_client->adapter))
		return ret;

	data[0] = start_reg;
	data[1] = value;
	msg[0].addr = dev_info->i2c_client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = 2;
	ret = i2c_transfer(dev_info->i2c_client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return 0;
}

/*
Read data from chip
start_reg: the start register address for read
read_value: read's value
count: read byte number
return ret:
0: read's sucess
< 0: read fail
*/
static int tusb302l_read_bytes(u8 start_reg, u8 *read_value, int count)
{
	int ret = NORMAL_ERROR;

	struct i2c_msg msg[2];
	struct tusb302l_device_info *dev_info;

	if ((start_reg < 0) || (read_value == NULL)
						|| (count <= 0))
		return ret;
	if ((tusb302l_device_info == NULL))
		return ret;

	dev_info = tusb302l_device_info;
	if ((NULL == dev_info->i2c_client) ||
		(NULL == dev_info->i2c_client->adapter))
		return ret;

	msg[0].addr = dev_info->i2c_client->addr;
	msg[0].flags = 0;
	msg[0].buf = &start_reg;
	msg[0].len = 1;
	msg[1].addr = dev_info->i2c_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = read_value;
	msg[1].len = count;

	ret = i2c_transfer(dev_info->i2c_client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	return ret;
}

static int tusb302l_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int tusb302l_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct tusb302l_device_info *dev_info;
	printk("%s:\n", __func__);
	dev_info = tusb302l_device_info;
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

static const struct i2c_device_id tusb302l_i2c_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, tusb302l_i2c_id_table);

#ifdef CONFIG_OF
static struct of_device_id tusb302l_match_table[] = {
    { .compatible = "tusb302l",},
    { },
};
#else
#define tusb302l_match_table NULL
#endif

static struct i2c_driver tusb302l_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
        .of_match_table = tusb302l_match_table,
	},
	.probe = tusb302l_i2c_probe,
	.remove = tusb302l_i2c_remove,
	.id_table = tusb302l_i2c_id_table,
};

static int tusb302l_init_i2c(void)
{
	return i2c_add_driver(&tusb302l_i2c_driver);
}

/*
enable:
true: enable chip
false: disable chip
*/
static int tusb302l_enable_chip(bool enable)
{
	struct tusb302l_device_info *dev_info;

	dev_info = tusb302l_device_info;
    /*because enb_gpio is always GND*/
	if (enable) {
		dev_info->enabled = 1;
	} else {
		dev_info->enabled = 0;
	}
	return 0;
}

static int tusb302l_init_chip(void)
{
	u8 start_reg;
	int count;
	u8 write_value;
	u8 read_value[8] = {0};
	struct tusb302l_device_info *dev_info;

	dev_info = tusb302l_device_info;
	printk("%s:\n",__func__);

	tusb302l_enable_chip(true);

	start_reg = 0x00;
	count = 8;
	tusb302l_read_bytes(start_reg, read_value, count);
	printk("%s: the chip is %c%c%c%c%c%c%c%c\n", __func__,
		read_value[7],read_value[6],read_value[5],read_value[4],
		read_value[3],read_value[2],read_value[1],read_value[0]);

	start_reg = 0x0A;
	write_value = 0x32;
	tusb302l_write_byte(start_reg, write_value);

	start_reg = 0x0A;
	count = 1;
	tusb302l_read_bytes(start_reg, read_value, count);
	printk("%s:the chip is 0x%x\n", __func__, read_value[0]);

	start_reg = 0x08;
	write_value = 0x40;
	tusb302l_write_byte(start_reg, write_value);

	return 0;
}

static ssize_t tusb302l_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t tusb302l_enable_store(struct device *dev,
		struct device_attribute *attr, const char*buf,
		size_t size)
{
	return size;
}

static DEVICE_ATTR(tusb302l_enable, S_IRUGO | S_IWUSR,
		tusb302l_enable_show, tusb302l_enable_store);

static int tusb302l_clear_interrupt_status(void)
{
	int count;
	u8 start_reg;
	u8 reg_value;
	struct tusb302l_device_info *dev_info;

	dev_info = tusb302l_device_info;

	start_reg = 0x09;
	count = 1;
	tusb302l_read_bytes(start_reg, &reg_value, count);
	printk("%s:, Register 0x09=0x%x\n",
		__func__, reg_value);

	start_reg = 0x09;
	reg_value |= 0x10;
	tusb302l_write_byte(start_reg, reg_value);

	return 0;
}

static void tusb302l_irq_handler(struct work_struct *w)
{
	u8 start_reg;
	u8 reg_value[2];
	struct tusb302l_device_info *dev_info;

	dev_info = tusb302l_device_info;
	printk("%s:\n", __func__);

	start_reg = 0x08;
	tusb302l_read_bytes(start_reg, reg_value, 2);
    printk("%s:, Register 0x08=0x%x, 0x09=0x%x\n",
			__func__, reg_value[0], reg_value[1]);

	/*Unattached*/
	if ((reg_value[1] & 0xC0) == 0x00) {
		if (dev_info->usb_mode == DFP_MODE)
			tusb302l_start_usb_DFPMode(false);
		if (dev_info->usb_mode == DEBUG_MODE)
			tusb302l_start_usb_DebugMode(false);

		tusb302l_start_usb_DRPMode();
		dev_info->usb_mode = DRP_MODE; //enter drp mode
	}

	/*Debug accessory attached*/
	if ((reg_value[1] & 0xC0) == 0xC0) {
		if (((reg_value[0] & 0x0E) == 0x0E)
			|| ((reg_value[0] & 0x0E) == 0x0C)) {
			tusb302l_start_usb_DebugMode(true);
			dev_info->usb_mode = DEBUG_MODE;
		}
	}

	/*Audio accessory attached*/
	if ((reg_value[1] & 0xC0) == 0xC0) {
		if (((reg_value[0] & 0x0E) == 0x08)
			|| ((reg_value[0] & 0x0E) == 0x0A)) {
			tusb302l_start_usb_AudioMode();
		}
	}

	/* Host attached */
	if ((reg_value[1] & 0xC0) == 0x80) {
		tusb302l_start_usb_UFPMode();
		dev_info->usb_mode = UFP_MODE;
	}

	/*device attached*/
	if ((reg_value[1] & 0xC0) == 0x40) {
		tusb302l_start_usb_DFPMode(true);
		dev_info->usb_mode = DFP_MODE;
	}

	tusb302l_clear_interrupt_status();
}

/*
Handle the irq
*/
static irqreturn_t tusb302l_irq(int irq, void *info)
{
	struct tusb302l_device_info *dev_info;
	printk("%s:\n",__func__);

	if (!tusb302l_device_info) {
		return IRQ_HANDLED;
	}
	dev_info = tusb302l_device_info;

	queue_work(dev_info->tusb302l_wq, &dev_info->q_work);
	return IRQ_HANDLED;
}

/*
Get tusb302l's chip info
*/
static int tusb302l_parse_dt(void)
{
	if (!tusb302l_chip_info)
		return NORMAL_ERROR;

	tusb302l_chip_info->port_gpio_set = 1;
	tusb302l_chip_info->addr_gpio_set = 1;
	tusb302l_chip_info->id_gpio = 0;
	tusb302l_chip_info->enb_gpio = 0;

	return RET_OK;
}

static int tusb302l_init_devinfo(void)
{
	int ret = NORMAL_ERROR;
	struct tusb302l_chip_info *chip_info;
	struct tusb302l_device_info *dev_info;

	if ((!tusb302l_device_info) ||
			(!tusb302l_chip_info))
		return NORMAL_ERROR;

	dev_info = tusb302l_device_info;
	chip_info = tusb302l_chip_info;
	dev_info->chip_info = tusb302l_chip_info;

	ret = alloc_chrdev_region(&dev_info->tusb302l_dev_t, 0, 1, "tusb302l_dev_t");
	if (ret < 0) {
		pr_err("%s:Fail to alloc char dev region\n", __func__);
		return ret;
	}

	dev_info->tusb302l_class = class_create(THIS_MODULE, "tusb302l_class");
	if (IS_ERR(dev_info->tusb302l_class)) {
		printk("%s: create class fail \n", __func__);
		return NORMAL_ERROR;
	}

	dev_info->tusb302l_device = device_create(dev_info->tusb302l_class,
					NULL, dev_info->tusb302l_dev_t, NULL,
					"tusb302l_device");


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
		dev_info->i2c_address = 0x47;
		break;
	case 0:
		dev_info->i2c_address = 0x47;
		break;
	default:
		dev_info->i2c_address = 0x47;
		break;
	}

    dev_info->pinctrl = devm_pinctrl_get(&dev_info->i2c_client->dev);
    if (IS_ERR_OR_NULL(dev_info->pinctrl)) {
        printk("%s: Unable to get pinctrl handle\n", __func__);
    }
	dev_info->intr_active = pinctrl_lookup_state(dev_info->pinctrl,
					"tusb302l_active");
	if (IS_ERR(dev_info->intr_active)) {
		printk("%s: could not get intr_active pinstate\n", __func__);
	}

	ret = pinctrl_select_state(dev_info->pinctrl,
					dev_info->intr_active);
	if (ret != 0) {
		printk("%s: Disable TLMM pins failed with %d\n",
			__func__, ret);

	}

	ret = gpio_request(chip_info->intb_gpio, "tusb302l_irq_gpio");
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
	dev_info->tusb302l_wq = alloc_ordered_workqueue("tusb302l_wq", 0);
	if (!dev_info->tusb302l_wq) {
		pr_err("%s: Unable to create workqueue otg_wq\n",
				__func__);
		goto release_irq_gpio;
	}
	INIT_WORK(&dev_info->q_work, tusb302l_irq_handler);

    return ret;

release_irq_gpio:
	gpio_free(chip_info->intb_gpio);
	return ret;
}


#ifdef CONFIG_PM_SLEEP
static int tusb302l_pm_suspend(struct device *dev)
{
	int ret = 0;
	struct tusb302l_device_info *dev_info;
	dev_info = tusb302l_device_info;

	enable_irq_wake(dev_info->irq);

	return ret;
}

static int tusb302l_pm_resume(struct device *dev)
{
	int ret = 0;

	struct tusb302l_device_info *dev_info;
	dev_info = tusb302l_device_info;

	disable_irq_wake(dev_info->irq);
	return ret;
}

static const struct dev_pm_ops tusb302l_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tusb302l_pm_suspend, tusb302l_pm_resume)
};
#endif /* CONFIG_PM_SLEEP */

static int tusb302l_check_debugmode(void)
{
	int count;
	u8 start_reg;
	u8 reg_value[2] = {0};
	struct tusb302l_device_info *dev_info;

	dev_info = tusb302l_device_info;

	start_reg = 0x08;
	count = 2;
	tusb302l_read_bytes(start_reg, reg_value, count);
	printk("%s:, Register 0x08=0x%x, 0x09=0x%x\n",
			__func__, reg_value[0], reg_value[1]);

	if ((reg_value[1] & 0xC0) == 0xC0) {
		if (((reg_value[0] & 0x0E) == 0x0E)
			|| ((reg_value[0] & 0x0E) == 0x0C)) {
			/*Debug accessory attached*/
			tusb302l_start_usb_DebugMode(true);
			dev_info->usb_mode = DEBUG_MODE;
		} else
			tusb302l_start_usb_DebugMode(false);
	} else
		tusb302l_start_usb_DebugMode(false);

	return 0;
}

static int tusb302l_check_DFPMode(void)
{
	int count;
	u8 start_reg;
	u8 reg_value[2] = {0};
	struct tusb302l_device_info *dev_info;

	dev_info = tusb302l_device_info;

	start_reg = 0x08;
	count = 2;
	tusb302l_read_bytes(start_reg, reg_value, count);
	printk("%s:, Register 0x08=0x%x, 0x09=0x%x\n",
			__func__, reg_value[0], reg_value[1]);

	if ((reg_value[1] & 0xC0) == 0x40) {
		tusb302l_start_usb_DFPMode(true);
		dev_info->usb_mode = DFP_MODE;
	}

	return 0;
}

static int tusb302l_probe(struct platform_device *pdev)
{
	int ret = NORMAL_ERROR;
	struct tusb302l_chip_info *chip_info;
	struct tusb302l_device_info *dev_info;
	pr_info("%s\n", __func__);

	if ((tusb302l_chip_info != NULL) ||
		(tusb302l_device_info != NULL))
		return ret;

	tusb302l_chip_info = kzalloc(sizeof(struct tusb302l_chip_info), GFP_KERNEL);
	if (!tusb302l_chip_info) {
		ret = -ENOMEM;
		goto malloc_error;
	}
	chip_info = tusb302l_chip_info;

	tusb302l_device_info = kzalloc(sizeof(struct tusb302l_device_info), GFP_KERNEL);
	if (!tusb302l_device_info) {
		ret = -ENOMEM;
		goto malloc_error;
	}
	dev_info = tusb302l_device_info;

	ret = tusb302l_parse_dt();
	if (ret)
		goto malloc_error;

	ret = tusb302l_init_i2c();
	if (ret) {
		printk("%s: init i2c error\n", __func__);
		goto malloc_error;
	}

	tusb302l_chip_info->intb_gpio =
		of_get_named_gpio_flags(dev_info->i2c_client->dev.of_node,
		"irq-gpio", 0, NULL);
	if (gpio_is_valid(tusb302l_chip_info->intb_gpio))
		printk("%s:irq gpio=%d\n",__func__, tusb302l_chip_info->intb_gpio);

	tusb302l_chip_info->id_gpio =
		of_get_named_gpio_flags(dev_info->i2c_client->dev.of_node,
		"qcom,id-gpio", 0, NULL);
	if (gpio_is_valid(tusb302l_chip_info->id_gpio))
		printk("%s: id gpio=%d\n",__func__, tusb302l_chip_info->id_gpio);

	gpio_direction_output(chip_info->id_gpio, 0);

	ret = tusb302l_init_devinfo();
	if (ret)
		goto malloc_error;

	device_create_file(dev_info->tusb302l_device, &dev_attr_tusb302l_enable);

	tusb302l_init_chip();
	tusb302l_check_debugmode();
	tusb302l_check_DFPMode();
	ret = request_irq(dev_info->irq, tusb302l_irq,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "tusb302l_irq", NULL);
	if (ret) {
		printk("%s:Cannot request irq\n", __func__);
		goto malloc_error;
	}

	tusb302l_clear_interrupt_status();

	return ret;

malloc_error:
	if (tusb302l_chip_info)
		kfree(tusb302l_chip_info);
	if (tusb302l_device_info)
		kfree(tusb302l_device_info);

	return ret;
}

static int tusb302l_remove(struct platform_device *pdev)
{
	struct tusb302l_device_info *dev_info;
    dev_info = tusb302l_device_info;

	if (tusb302l_device_info != NULL)
		dev_info = tusb302l_device_info;
	else
		return 0;

	tusb302l_enable_chip(false);
	i2c_del_driver(&tusb302l_i2c_driver);
	free_irq(dev_info->irq, NULL);
	gpio_free(tusb302l_chip_info->intb_gpio);

	device_remove_file(dev_info->tusb302l_device,
					&dev_attr_tusb302l_enable);
	device_destroy(dev_info->tusb302l_class, dev_info->tusb302l_dev_t);
	class_destroy(dev_info->tusb302l_class);
	unregister_chrdev_region(dev_info->tusb302l_dev_t, 1);

	if (tusb302l_chip_info != NULL)
		kfree(tusb302l_chip_info);
	if (tusb302l_device_info != NULL)
		kfree(tusb302l_device_info);

	return 0;
}

static struct of_device_id tusb302l_dt_match[] = {
	{
        .compatible = "letv,tusb302l_driver",
	},
	{}
};

static struct platform_driver tusb302l_platform_driver = {
	.driver = {
		.name = "tusb302l_driver",
		.of_match_table = tusb302l_dt_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &tusb302l_pm_ops,
#endif
	},
	.probe = tusb302l_probe,
	.remove = tusb302l_remove,
};

static int __init tusb302l_init(void)
{
	int ret = NORMAL_ERROR;

	ret = platform_driver_register(&tusb302l_platform_driver);
	if (ret) {
		pr_err("%s(): Failed to register tusb302l"
				 "platform driver\n", __func__);
	}
	return ret;
}

static void __exit tusb302l_exit(void)
{
	platform_driver_unregister(&tusb302l_platform_driver);
}


late_initcall(tusb302l_init);
module_exit(tusb302l_exit);

MODULE_AUTHOR("Letv, Inc.");
MODULE_DESCRIPTION("Letv TUSB302L Support Module");
MODULE_LICENSE("GPL v2");
