#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>
#include <linux/notifier.h>
/**********************************************************/
enum FP_MODE{
	GF_IMAGE_MODE = 0,
	GF_KEY_MODE,
	GF_SLEEP_MODE,
	GF_FF_MODE,
	GF_DEBUG_MODE = 0x56
};

#define GF_KEY_INPUT_HOME		KEY_HOME
#define GF_KEY_INPUT_MENU		KEY_MENU
#define GF_KEY_INPUT_BACK		KEY_BACK
#define GF_KEY_INPUT_POWER		KEY_POWER
#define GF_KEY_INPUT_CAMERA		KEY_CAMERA

typedef enum gf_key_event {
	GF_KEY_NONE = 0,
	GF_KEY_HOME,
	GF_KEY_POWER,
	GF_KEY_MENU,
	GF_KEY_BACK,
	GF_KEY_CAMERA,
} gf_key_event_t;

struct gf_key {
	enum gf_key_event key;
	uint32_t value;   /* key down = 1, key up = 0 */
};

struct gf_key_map {
	unsigned int type;
	unsigned int code;
};

struct gf_ioc_chip_info {
	unsigned char vendor_id;
	unsigned char mode;
	unsigned char operation;
	unsigned char reserved[5];
};

#define GF_IOC_MAGIC         'g'
#define GF_IOC_INIT			_IOR(GF_IOC_MAGIC, 0, uint8_t)
#define GF_IOC_EXIT			_IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET			_IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ		_IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ		_IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_SPI_CLK		_IOW(GF_IOC_MAGIC, 5, uint32_t)
#define GF_IOC_DISABLE_SPI_CLK		_IO(GF_IOC_MAGIC, 6)
#define GF_IOC_ENABLE_POWER		_IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER		_IO(GF_IOC_MAGIC, 8)
#define GF_IOC_INPUT_KEY_EVENT		_IOW(GF_IOC_MAGIC, 9, struct gf_key)
#define GF_IOC_ENTER_SLEEP_MODE		_IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO		_IOR(GF_IOC_MAGIC, 11, uint8_t)
#define GF_IOC_REMOVE			_IO(GF_IOC_MAGIC, 12)
#define GF_IOC_CHIP_INFO		_IOW(GF_IOC_MAGIC, 13, struct gf_ioc_chip_info)

#define GF_IOC_COOLBOOT			_IO(GF_IOC_MAGIC, 20)
#define GF_IOC_SETSPEED			_IOW(GF_IOC_MAGIC, 21, unsigned int)
#define GF_IOC_PM_FBCABCK		_IO(GF_IOC_MAGIC, 22)

#define  GF_IOC_MAXNR    14

//#define AP_CONTROL_CLK       1
//#define  USE_PLATFORM_BUS     1
#define  USE_SPI_BUS	1
#define GF_FASYNC   1	/*If support fasync mechanism.*/
#define NETLINK_TEST 25

struct gf_dev {
	dev_t devt;
	struct list_head device_entry;
#if defined(USE_SPI_BUS)
	struct spi_device *spi;
#elif defined(USE_PLATFORM_BUS)
	struct platform_device *spi;
#endif
	struct clk *core_clk;
	struct clk *iface_clk;

	struct input_dev *input;
	/* buffer is NULL unless this device is open (users > 0) */
	unsigned users;
	signed irq_gpio;
	signed reset_gpio;
	signed pwr_gpio;
	int irq;
	int irq_enabled;
	int clk_enabled;
#ifdef GF_FASYNC
	struct fasync_struct *async;
#endif
	struct notifier_block notifier;
	char device_available;
	char fb_black;
	struct regulator *vreg;
};

int gf_parse_dts(struct gf_dev* gf_dev);
void gf_cleanup(struct gf_dev *gf_dev);

int gf_power_on(struct gf_dev *gf_dev);
int gf_power_off(struct gf_dev *gf_dev);
int gf_power_on_8996(struct gf_dev *gf_dev);
int gf_power_off_8996(struct gf_dev *gf_dev);
int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms);
int gf_irq_num(struct gf_dev *gf_dev);

#endif /*__GF_SPI_H*/
