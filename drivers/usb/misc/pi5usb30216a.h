#ifndef __PI5USB30216A_H
#define __PI5USB30216A_H

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
intb_gpio: interrupt when pi5usb's register change
enb_gpio:
	1:disable state
	0:enable state


*/
struct pi5usb_chip_info {
	int port_gpio_set;
	int addr_gpio_set;
	int intb_gpio;
	int id_gpio;
	int enb_gpio;
};

/*
i2c_address: chip's i2c address
*/
struct pi5usb_device_info {
	int irq;     /* irq number*/
	u8 usb_mode;  /*UFP, DFP*/
	int usb_mode_try_sink;
	int i2c_address;  /*i2c's address*/
	struct i2c_client *i2c_client;  /**/
    struct pinctrl *pinctrl;
	struct pinctrl_state *intr_active;
	struct pinctrl_state *switch_usbid;
	struct class *pi5usb_class;
	dev_t pi5usb_dev_t;
	struct device *pi5usb_device;
	int enabled;
	struct workqueue_struct *pi5usb_wq;
	struct work_struct q_work;
	struct pi5usb_chip_info *chip_info;
};




#endif /*__PI5USB30216A_H */


