#ifndef __TUSB302L_H
#define __TUSB302L_H

#define RET_OK	0
#define NORMAL_ERROR	-1

enum {
	UFP_MODE = 0,
	DRP_MODE = 0x4,
	DFP_MODE = 0x2,
	DEBUG_MODE = 0x5,
};

/*
port_gpio_set:
	0:usb as UFP
	1:usb as DRP
	2:usb as DFP

addr_gpio_set:
	0:i2c addr is
	1:i2c addr is
	other:pin control mode
intb_gpio: interrupt when tusb302l's register change
enb_gpio:
	1:disable state
	0:enable state


*/
struct tusb302l_chip_info {
	int port_gpio_set;
	int addr_gpio_set;
	int intb_gpio;
	int id_gpio;
	int enb_gpio;
};

/*
i2c_address: chip's i2c address
*/
struct tusb302l_device_info {
	int irq;     /* irq number*/
	u8 usb_mode;  /*UFP, DFP*/
	int usb_mode_try_sink;
	int i2c_address;  /*i2c's address*/
	struct i2c_client *i2c_client;  /**/
    struct pinctrl *pinctrl;
	struct pinctrl_state *intr_active;
	struct pinctrl_state *switch_usbid;
	struct class *tusb302l_class;
	dev_t tusb302l_dev_t;
	struct device *tusb302l_device;
	int enabled;
	struct workqueue_struct *tusb302l_wq;
	struct work_struct q_work;
	struct tusb302l_chip_info *chip_info;
};

#endif /*__TUSB302L_H */
