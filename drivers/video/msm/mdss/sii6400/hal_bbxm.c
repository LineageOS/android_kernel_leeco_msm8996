/*
  SiI6400 Linux Driver

  Hardware Abstraction Layer (HAL) implementation for BeagleBoard xM
  using SPI and GPIOs

  Copyright (C) 2012-2013 Silicon Image, Inc.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation version 2.

  This program is distributed "AS-IS" WITHOUT ANY WARRANTY of any
  kind, whether express or implied; INCLUDING without the implied warranty
  of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.
  See the GNU General Public License for more details at
  http://www.gnu.org/licenses/gpl-2.0.html.
*/

/*#define USE_INTERRUPT_DISABLE*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/mdss_io_util.h>

#include "osal.h"
#include "hal.h"
#include "hal_bbxm.h"
#include "device.h"
#include "sii6400.h"
#include "host_msg.h"

static uint8_t bit_reverse_table[256] = {
	0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
	0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
	0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
	0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
	0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
	0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
	0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
	0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
	0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
	0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
	0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
	0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
	0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
	0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
	0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
	0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
	0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
	0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
	0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
	0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
	0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
	0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
	0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
	0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
	0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
	0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
	0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
	0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
	0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
	0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
	0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
	0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff
};

struct spi_ctrl_blk {
	struct spi_device *spidev;
};

static struct spi_ctrl_blk spi_ctrl_info;
#ifdef USE_INTERRUPT_DISABLE
static bool sii6400_in_irq;
#endif /* USE_INTERRUPT_DISABLE */
static bool in_reset = true;

/* Semaphore to protect access to SPI communications */
#define MAX_WAIT_FOR_SPI_SEMAPHORE	500
static struct SiiOsSemaphore *sii6400_spi_sem;

/* static function prototypes */
static int sii6400_spi_write_read(struct spi_device *spi, const u8 *txbuf,
					u8 *rxbuf, size_t len);
static int enable_interrupts_spi(void);
static int disable_interrupts_spi(void);
static irqreturn_t spi_irq(int irq, void *handle);
static int spi_probe(struct spi_device *spidev);
static int spi_remove(struct spi_device *spidev);

extern struct of_device_id sii6400_match_table[];

/* SPI driver info */
static struct spi_driver sii6400_spi = {
	.probe  = spi_probe,
	.remove = spi_remove,
	.driver = {
		.name  = SII6400_SPI_DRIVER_NAME,
		.bus   = &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = sii6400_match_table,
	},
};

static int sii6400_spi_write_read(struct spi_device *spi, const u8 *txbuf,
				  u8 *rxbuf, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = txbuf,
		.rx_buf = rxbuf,
		.len = len,
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	return spi_sync(spi, &msg);
}

/*
 * Enable Sii6400 Interrupts
 * Returns: 0 if successful
 *          -1 if interrupts could not be enabled
 */
static int enable_interrupts_spi(void)
{
#ifdef USE_INTERRUPT_DISABLE
	int retval = -1;

	/*dbg("");*/

	if (!sii6400_in_irq) {
		enable_irq(gpio_to_irq(SII6400_GPIO_INT));
		retval = 0;
	}

	return retval;
#else /* USE_INTERRUPT_DISABLE */
	return 0;
#endif /* USE_INTERRUPT_DISABLE */
}

/*
 * Disable Sii6400 Interrupts
 * Returns: 0 if successful
 *          -1 if interrupts could not be disabled
 */
static int disable_interrupts_spi(void)
{
#ifdef USE_INTERRUPT_DISABLE
	int retval = -1;

	/*dbg("");*/

	if (!sii6400_in_irq) {
		disable_irq(gpio_to_irq(SII6400_GPIO_INT));
		retval = 0;
	}

	return retval;
#else /* USE_INTERRUPT_DISABLE */
	return 0;
#endif /* USE_INTERRUPT_DISABLE */
}

static irqreturn_t spi_irq(int irq, void *handle)
{
#ifdef USE_INTERRUPT_DISABLE
	sii6400_in_irq = true;
#endif /* USE_INTERRUPT_DISABLE */

	/*dbg("");*/

	if (!in_reset)
		SiiHalIsr();
	return IRQ_HANDLED;
}

static int get_dt_data(struct device *dev, struct sii6400_device_info *devinfo)
{
	int i, rc = 0;
	struct device_node *of_node = NULL;
	struct dss_gpio *temp_gpio = NULL;
	//struct dss_vreg *vreg_config = NULL;
	int dt_gpio;
	//struct regulator *reg = NULL;
#ifdef CONFIG_LIMIT_ON_HDMI
	struct device_node *hdmi_tx_node = NULL;
#endif

	if (!dev || !devinfo) {
		err("invalid input\n");
		return -EINVAL;
	}

	of_node = dev->of_node;
	if (!of_node) {
		err("invalid of_node\n");
		goto error;
	}

	dbg("id=%d\n", dev->id);

	/* HDMI DETECT */
	temp_gpio = NULL;
	temp_gpio = devm_kzalloc(dev, sizeof(struct dss_gpio), GFP_KERNEL);
	dbg("gpios allocd\n");
	if (!temp_gpio) {
		err("can't alloc hdmi-det-gpio mem\n");
		goto error;
	}

	dt_gpio = of_get_named_gpio(of_node, "hdmi-det", 0);
	info("hdmi-det-gpio <%d>\n", dt_gpio);
	if (dt_gpio < 0) {
		err("Can't get hdmi-det-gpio\n");
		goto error;
	}

	temp_gpio->gpio = dt_gpio;
	snprintf(temp_gpio->gpio_name, 32, "%s", "hdmi-det-gpio");
	dbg("hdmi-det gpio=[%d]\n", temp_gpio->gpio);
	devinfo->gpios[SII6400_HDMI_DET_GPIO] = temp_gpio;

	/* RESET */
	temp_gpio = NULL;
	temp_gpio = devm_kzalloc(dev, sizeof(struct dss_gpio), GFP_KERNEL);
	dbg("gpios allocd\n");
	if (!temp_gpio) {
		err("can't alloc rst-gpio mem\n");
		goto error;
	}

	dt_gpio = of_get_named_gpio(of_node, "rst-gpio", 0);
	info("rst-gpio <%d>\n", dt_gpio);
	if (dt_gpio < 0) {
		err("Can't get rst-gpio\n");
		goto error;
	}

	temp_gpio->gpio = dt_gpio;
	snprintf(temp_gpio->gpio_name, 32, "%s", "rst-gpio");
	dbg("rst gpio=[%d]\n", temp_gpio->gpio);
	devinfo->gpios[SII6400_RESET_GPIO] = temp_gpio;

	/* INTR */
	temp_gpio = NULL;
	temp_gpio = devm_kzalloc(dev, sizeof(struct dss_gpio), GFP_KERNEL);
	dbg("gpios allocd\n");
	if (!temp_gpio) {
		err("can't alloc intr_gpio mem\n");
		goto error;
	}
	dt_gpio = of_get_named_gpio(of_node, "intr-gpio", 0);
	info("intr-gpio <%d>\n", dt_gpio);
	if (dt_gpio < 0) {
		err("Can't get intr-gpio\n");
		goto error;
	}

	temp_gpio->gpio = dt_gpio;
	snprintf(temp_gpio->gpio_name, 32, "%s", "intr-gpio");
	dbg("intr gpio=[%d]\n", temp_gpio->gpio);
	devinfo->gpios[SII6400_INTR_GPIO] = temp_gpio;

	/* PWR */
	temp_gpio = NULL;
	temp_gpio = devm_kzalloc(dev, sizeof(struct dss_gpio), GFP_KERNEL);
	dbg("gpios allocd\n");
	if (!temp_gpio) {
		err("can't alloc pwr_gpio mem\n");
		goto error;
	}
	dt_gpio = of_get_named_gpio(of_node, "pwr-gpio", 0);
	info("pwr-gpio <%d>\n", dt_gpio);
	if (dt_gpio < 0) {
		err("Can't get pwr-gpio\n");
		goto error;
	}

	temp_gpio->gpio = dt_gpio;
	snprintf(temp_gpio->gpio_name, 32, "%s", "pwr-gpio");
	dbg("pwr gpio=[%d]\n", temp_gpio->gpio);
	devinfo->gpios[SII6400_PMIC_PWR_GPIO] = temp_gpio;

	/* MHL SELECT 1 */
	temp_gpio = NULL;
	temp_gpio = devm_kzalloc(dev, sizeof(struct dss_gpio), GFP_KERNEL);
	if (!temp_gpio) {
		err("can't alloc mhl_sel1_gpio mem\n");
		goto error;
	}
	dt_gpio = of_get_named_gpio(of_node, "mhl-sel1-gpio", 0);
	info("mhl-sel1-gpio <%d>\n", dt_gpio);
	if (dt_gpio < 0) {
		err("Can't get mhl-sel1-gpio\n");
		goto error;
	}

	temp_gpio->gpio = dt_gpio;
	snprintf(temp_gpio->gpio_name, 32, "%s", "mhl-sel1-gpio");
	dbg("mhl sel1 gpio=[%d]\n", temp_gpio->gpio);
	devinfo->gpios[SII6400_MHL_SEL1_GPIO] = temp_gpio;

#if 0
	/* 1.8V Regulator */
	vreg_config = devm_kzalloc(dev, sizeof(struct dss_vreg), GFP_KERNEL);
	if (!vreg_config) {
		err("can't alloc vreg mem\n");
		rc = -ENOMEM;
		goto error;
	}
	reg = regulator_get(dev, "avcc_18");
	if (IS_ERR(reg)) {
		rc = PTR_ERR(reg);
		err("could not get PMA8084_L14, rc = %d\n", rc);
		goto error;
	}

	vreg_config->vreg = reg;
	snprintf(vreg_config->vreg_name, 32, "%s", "avcc_18");
	devinfo->vregs[SII6400_1V8_VREG] = vreg_config;
#endif

#ifdef CONFIG_LIMIT_ON_HDMI
	/* parse phandle for hdmi tx */
	hdmi_tx_node = of_parse_phandle(of_node, "qcom,hdmi-tx-map", 0);
	if (!hdmi_tx_node) {
		err("%s: can't find hdmi phandle\n", __func__);
		goto error;
	}

	devinfo->hdmi_pdev = of_find_device_by_node(hdmi_tx_node);
	if (!devinfo->hdmi_pdev) {
		err("%s: can't find the device by node\n", __func__);
		goto error;
	}
	dbg("%s: hdmi_pdev [0X%lx] to devinfo->pdev\n",
	    __func__, (unsigned long)devinfo->hdmi_pdev);
#endif

	return 0;

error:
	err("return due to err %d\n", rc);
	for (i = 0; i < SII6400_MAX_GPIO; ++i) {
		if (devinfo->gpios[i]) {
			devm_kfree(dev, devinfo->gpios[i]);
			devinfo->gpios[i] = NULL;
		}
	}

	return rc;
}

int sii6400_hdmi_tx_device_show (char *buf)
{
	const char *dev_type_str = "wihd";

	scnprintf(buf, PAGE_SIZE, "%s", dev_type_str);

	return 0;
}

static int spi_probe(struct spi_device *spidev)
{
	int retval = 0;

	int hdmi_det_gpio = -1;
	int reset_gpio = -1;
	int intr_gpio = -1;
	int pwr_gpio = -1;

	struct sii6400_device_info * devinfo = NULL;
#ifdef CONFIG_LIMIT_ON_HDMI
	struct msm_hdmi_mhl_ops *hdmi_mhl_ops = NULL;
#endif

	dbg("");

	if (NULL == spidev) {
		err("Failed spi_setup: NULL spidev buffer\n");
		retval = -EFAULT;
		goto failed1;
	}

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Failed spi_probe: NULL devinfo buffer\n");
		retval = -EFAULT;
		goto failed1;
	}

	if (spidev->dev.of_node) {
		retval = get_dt_data(&spidev->dev, devinfo);
		if (retval) {
			err("FAILED: parsing device tree data; rc=%d\n", retval);
			goto failed1;
		}
	} else {
		err("FAILED: no device tree data attached in spi device\n");
		goto failed1;
	}

	spi_ctrl_info.spidev = spidev;
	spi_ctrl_info.spidev->mode = SII6400_SPI_MODE;
	spi_ctrl_info.spidev->bits_per_word = SII6400_SPI_BITS_PER_WORD;
	spi_ctrl_info.spidev->max_speed_hz = SII6400_SPI_MAX_SPEED_HZ;

	retval = spi_setup(spidev);
	if (retval < 0) {
		err("Failed spi_setup: %d\n", retval);
		goto failed1;
	}

	hdmi_det_gpio = devinfo->gpios[SII6400_HDMI_DET_GPIO]->gpio;
	if (!gpio_is_valid(hdmi_det_gpio)) {
		err("GPIO HDMI Detect %d not valid\n", reset_gpio);
		goto failed1;
	}

	reset_gpio = devinfo->gpios[SII6400_RESET_GPIO]->gpio;
	if (!gpio_is_valid(reset_gpio)) {
		err("GPIO Reset %d not valid\n", reset_gpio);
		goto failed1;
	}

	intr_gpio = devinfo->gpios[SII6400_INTR_GPIO]->gpio;
	if (!gpio_is_valid(intr_gpio)) {
		err("GPIO Interrupt %d not valid\n", intr_gpio);
		goto failed1;
	}

	pwr_gpio = devinfo->gpios[SII6400_PMIC_PWR_GPIO]->gpio;
	if (!gpio_is_valid(pwr_gpio)) {
		err("GPIO PWR %d not valid\n", pwr_gpio);
		goto failed1;
	}

	/* We request SiI6400 dedicated GPIOs in spi_probe,
	   and we free SiI6400 dedicated GPIOs in spi_remove */
	retval = gpio_request_one(hdmi_det_gpio, GPIOF_OUT_INIT_LOW,
					"SII6400 HDMI detect");
	if (retval < 0) {
		err("Failed GPIO DET request: %d\n", retval);
		goto failed1;
	}

	retval = gpio_request_one(reset_gpio, GPIOF_OUT_INIT_HIGH,
					"SII6400 reset");
	if (retval < 0) {
		err("Failed GPIO Reset request: %d\n", retval);
		goto failed2;
	}

	retval = gpio_request_one(pwr_gpio, GPIOF_OUT_INIT_LOW,
					"SII6400 pcim pwr");
	if (retval < 0) {
		err("Failed GPIO PWR request: %d\n", retval);
		goto failed3;
	}

	retval = gpio_request_one(intr_gpio, GPIOF_IN,
					"SII6400 interrupt");
	if (retval < 0) {
		err("Failed GPIO Interrupt request: %d\n", retval);
		goto failed4;
	}

	// TODO: move irq request logic to change_mode
	retval = request_threaded_irq(gpio_to_irq(intr_gpio), NULL,
					spi_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					spidev->dev.driver->name, NULL);
	if (retval < 0) {
		err("Failed request_irq: %d\n", retval);
		goto failed5;
	}

#ifdef CONFIG_LIMIT_ON_HDMI
	hdmi_mhl_ops = devm_kzalloc(&spidev->dev,
				    sizeof(struct msm_hdmi_mhl_ops),
				    GFP_KERNEL);
	if (!hdmi_mhl_ops) {
		err("%s: alloc hdmi mhl ops failed\n", __func__);
		retval = -ENOMEM;
		goto failed5;
	}

	hdmi_mhl_ops->notify = hdmi_tx_hpd_done_callback;

	if (devinfo->hdmi_pdev) {
		retval = msm_hdmi_register_mhl(devinfo->hdmi_pdev,
					 hdmi_mhl_ops, NULL);
		if (retval) {
			err("%s: register with hdmi failed\n", __func__);
			retval = -EPROBE_DEFER;
			goto failed6;
		}
	}

	if (!hdmi_mhl_ops || !hdmi_mhl_ops->tmds_enabled ||
	    !hdmi_mhl_ops->set_mhl_max_pclk) {
		err("%s: func ptr is NULL\n", __func__);
		retval = -EINVAL;
		goto failed6;
	}
	devinfo->hdmi_mhl_ops = hdmi_mhl_ops;
#endif
	if(msm_hdmi_device_show_register(sii6400_hdmi_tx_device_show)){
		err("%s: func ret err\n", __func__);
		retval = -EINVAL;
		goto failed6;
	}

	return 0;

#ifdef CONFIG_LIMIT_ON_HDMI
failed6:
	if (hdmi_mhl_ops) {
		devm_kfree(&spidev->dev, hdmi_mhl_ops);
		devinfo->hdmi_mhl_ops = NULL;
	}
#endif
failed5:
	gpio_free(intr_gpio);
failed4:
	gpio_free(pwr_gpio);
failed3:
	gpio_free(reset_gpio);
failed2:
	gpio_free(hdmi_det_gpio);
failed1:
	return retval;
}

static int spi_remove(struct spi_device *spidev)
{
	int retval = 0;
	int gpio = -1;
	int i = 0;

	struct sii6400_device_info * devinfo = NULL;

	dbg("");

	devinfo = get_sii6400_devinfo();
	if (NULL == devinfo) {
		err("Failed spi_remove: NULL devinfo buffer\n");
		retval = -EFAULT;
		goto failed;
	}

	/* free HDMI DETECT gpio */
	if (devinfo->gpios[SII6400_HDMI_DET_GPIO]) {
		gpio = devinfo->gpios[SII6400_HDMI_DET_GPIO]->gpio;
		gpio_free(gpio);
	}

	/* free RESET gpio */
	if (devinfo->gpios[SII6400_RESET_GPIO]) {
		gpio = devinfo->gpios[SII6400_RESET_GPIO]->gpio;
		gpio_free(gpio);
	}

	/* free PWR gpio */
	if (devinfo->gpios[SII6400_PMIC_PWR_GPIO]) {
		gpio = devinfo->gpios[SII6400_PMIC_PWR_GPIO]->gpio;
		gpio_free(gpio);
	}

	/* free INTR irq and gpio */
	if (devinfo->gpios[SII6400_INTR_GPIO]) {
		gpio = devinfo->gpios[SII6400_INTR_GPIO]->gpio;
		free_irq(gpio_to_irq(gpio), NULL);
		gpio_free(gpio);
	}

	for (i = 0; i < SII6400_MAX_GPIO; i++) {
		if (devinfo->gpios[i]) {
			devm_kfree(&spidev->dev, devinfo->gpios[i]);
			devinfo->gpios[i] = NULL;
		}
	}

	for (i = 0; i < SII6400_MAX_VREG; i++) {
		if (devinfo->vregs[i]) {
			devm_kfree(&spidev->dev, devinfo->vregs[i]);
			devinfo->vregs[i] = NULL;
		}
	}

#ifdef CONFIG_LIMIT_ON_HDMI
	if (devinfo->hdmi_pdev) {
		/* parent device info refer to device tree */
		devinfo->hdmi_pdev = NULL;
	}

	if (devinfo->hdmi_mhl_ops) {
		devm_kfree(&spidev->dev, devinfo->hdmi_mhl_ops);
		devinfo->hdmi_mhl_ops = NULL;
	}
#endif

failed:
	return retval;
}

/*
 * Initialize the HAL
 * Returns: SII_STATUS_SUCCESS if successful
 *          SII_STATUS_ERR_FAILED if initialization failed
 */
enum sii_status SiiHalInit(void)
{
	enum sii_status retval = SII_STATUS_SUCCESS;
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;
	int rv = 0;

	dbg("");

	sii_os_rv = SiiOsSemaphoreCreate("spi_semaphore", 1, 1,
						&sii6400_spi_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Unable to create SPI semaphore\n");
		retval = SII_STATUS_ERR_FAILED;
		goto init_failed;
	}

	rv = spi_register_driver(&sii6400_spi);
	if (rv < 0) {
		err("Unable to register SPI driver: %d\n", rv);
		retval = SII_STATUS_ERR_FAILED;
		goto init_failed;
	}

	/* ToDo: add code to wait for spi_probe to finish
	 * before continuing with initialization. */

	return SII_STATUS_SUCCESS;

init_failed:
	SiiHalTerm();
	return retval;
}

/*
 * Terminate the HAL
 */
void SiiHalTerm(void)
{
	dbg("");

	spi_unregister_driver(&sii6400_spi);

	if (NULL != sii6400_spi_sem) {
		if (SII_OS_STATUS_SUCCESS !=
				SiiOsSemaphoreDelete(sii6400_spi_sem))
			err("Failed to delete SPI semaphore\n");
		sii6400_spi_sem = NULL;
	}
}

/*
 * Issue a Register Write or Read command.
 * The uint32 command (in 'data') will not be checked for correct data.
 * The uint32 response will be set in 'pRetData' (if not NULL).
 * Returns: SII_STATUS_SUCCESS if successful
 *          SII_STATUS_ERR_FAILED an error occurred
 */
enum sii_status SiiHalSpiOp(uint32_t *pTxData, uint32_t *pRxData,
			    uint32_t size)
{
	enum sii_status retStatus = SII_STATUS_SUCCESS;
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;
	int rv = 0;
	uint32_t i;

	/*dbg("data out = 0x%x\n", data);*/

	if (NULL == spi_ctrl_info.spidev) {
		err("SPI not initialized\n");
		retStatus = SII_STATUS_ERR_FAILED;
		goto done;
	}

	if (NULL == pTxData || size == 0) {
		err("no input SPI data\n");
		retStatus = SII_STATUS_ERR_FAILED;
		goto done;
	}

	if (in_reset) {
		dbg("SPI Op does not work in_reset\n");
		retStatus = SII_STATUS_ERR_FAILED;
		goto done;
	}

	(void)disable_interrupts_spi();

	sii_os_rv = SiiOsSemaphoreTake(sii6400_spi_sem,
					MAX_WAIT_FOR_SPI_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not take SPI semaphore: %x\n", sii_os_rv);
		retStatus = SII_STATUS_ERR_FAILED;
		goto done_enable_ints;
	}

	for (i = 0; i < size; i++) {
		uint8_t *msb_data = (uint8_t *)(pTxData + i);
		int j;

		for (j = 0; j < 4; j++) {
			/* Convert data byte to LSB */
			uint8_t lsb_data = bit_reverse_table[msb_data[j]];
			uint8_t lsb_ret_data = 0;

			rv = sii6400_spi_write_read(spi_ctrl_info.spidev,
							&lsb_data,
							&lsb_ret_data, 1);
			if (rv < 0) {
				err("sii6400_spi_write_read failed: %d\n", rv);
				retStatus = SII_STATUS_ERR_FAILED;
				goto done_give_sem;
			}

			if (NULL != pRxData) {
				uint8_t *msb_ret_data =
						(uint8_t *)(pRxData + i);
				/* convert ret data byte to MSB */
				msb_ret_data[j] =
					bit_reverse_table[lsb_ret_data];
			}
		}
	}

done_give_sem:
	sii_os_rv = SiiOsSemaphoreGive(sii6400_spi_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv)
		err("Could not release SPI semaphore: %x\n", sii_os_rv);
done_enable_ints:
	(void)enable_interrupts_spi();
done:
	return retStatus;
}

/*
 * Send a reset command.
 * Input: true = put chip in reset (RESET0); false = release reset (RESET1)
 * Returns: SII_STATUS_SUCCESS (since this can never fail)
 */
enum sii_status SiiHalReset(bool reset)
{
	struct sii6400_device_info *devinfo = get_sii6400_devinfo();

	dbg("%s reset\n", reset ? "in" : "out of");

	if (NULL == devinfo) {
		err("NULL devinfo buffer\n");
		return SII_STATUS_ERR_INVALID_PARAM;
	}

	if (devinfo->gpios[SII6400_RESET_GPIO]) {
		gpio_set_value(devinfo->gpios[SII6400_RESET_GPIO]->gpio, reset ? 0 : 1);
		in_reset = reset;
	} else {
		err("empty SII6400_RESET_GPIO gpios\n");
		return SII_STATUS_ERR_FAILED;
	}

	return SII_STATUS_SUCCESS;
}

/*
 * Send a Bulk transfer.
 * Returns: SII_STATUS_SUCCESS if successful
 *          SII_STATUS_ERR_INVALID_PARAM if pBuffer is NULL or
 *					    size is not a multiple of 12
 *          SII_STATUS_ERR_FAILED an error occurred
 */
enum sii_status SiiHalBulkTxfr(uint32_t *pBuffer, uint32_t size)
{
	enum sii_status retStatus = SII_STATUS_SUCCESS;
	enum sii_os_status sii_os_rv = SII_OS_STATUS_SUCCESS;
	int rv = 0;
	uint8_t *bulk_data_xfer_buf = NULL;
	size_t bulk_data_xfer_buf_size = 0;
	uint8_t *next = NULL;
	uint32_t size_left = 0;
	uint32_t max_bytes_sent_per_buf =
				(((BULK_DATA_BUF_SIZE * 3) / 4) / 12) * 12;
	struct SiiOsTime start_time, end_time;

	dbg("");

	if (NULL == pBuffer) {
		err("NULL bulk data buffer\n");
		retStatus = SII_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (0 != size % 12) {
		err("Bulk data size %d not a multiple of 12\n", size);
		retStatus = SII_STATUS_ERR_INVALID_PARAM;
		goto done;
	}

	if (in_reset) {
		dbg("Bulk Tx does not work in_reset\n");
		retStatus = SII_STATUS_ERR_FAILED;
		goto done;
	}

	bulk_data_xfer_buf_size = BULK_DATA_BUF_SIZE;
	bulk_data_xfer_buf = SiiOsCalloc("SPIBulk", BULK_DATA_BUF_SIZE, 0);
	if (NULL == bulk_data_xfer_buf) {
		err("Out of memory\n");
		retStatus = SII_STATUS_ERR_FAILED;
		goto done;
	}

	dbg("total number bytes to be sent = %u\n", size);
	dbg("max_bytes_sent_per_buf = %u\n", max_bytes_sent_per_buf);
	dbg("bulk_data_xfer_buf_size = %u\n",
	    (uint32_t)bulk_data_xfer_buf_size);

	(void)disable_interrupts_spi();

	sii_os_rv = SiiOsSemaphoreTake(sii6400_spi_sem,
					MAX_WAIT_FOR_SPI_SEMAPHORE);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv) {
		err("Could not take SPI semaphore: %x\n", sii_os_rv);
		retStatus = SII_STATUS_ERR_FAILED;
		goto done_enable_ints;
	}

	SiiOsGetTimeCurrent(&start_time);

	next = (uint8_t *)pBuffer;
	size_left = size;
	while (0 != size_left) {
		uint32_t send_size = (size_left < max_bytes_sent_per_buf) ?
					size_left : max_bytes_sent_per_buf;
		uint8_t *bytes = bulk_data_xfer_buf;
		uint32_t i;

		for (i = 0; i < send_size; i += 3) {
			int8_t j;

			for (j = -1; j < 3; j++) {
				/* Convert data byte to LSB */
				*bytes++ = bit_reverse_table[(-1 == j) ?
					PAGE_WRITE_CMD_WORD : next[i + j]];
			}
		}

		rv = spi_write(spi_ctrl_info.spidev, bulk_data_xfer_buf,
				bytes - bulk_data_xfer_buf);
		if (rv < 0) {
			err("spi_write failed: %d\n", rv);
			retStatus = SII_STATUS_ERR_FAILED;
			goto done_give_sem;
		}

		next += send_size;
		if (send_size < size_left)
			size_left -= send_size;
		else
			size_left = 0;
		/*dbg("bytes sent = %u, bytes remaining = %u\n",
		    send_size, size_left);*/
	}

	SiiOsGetTimeCurrent(&end_time);

	dbg("xfer time: %lld msec\n",
	    SiiOsGetTimeDifferenceMs(&end_time, &start_time));

done_give_sem:
	sii_os_rv = SiiOsSemaphoreGive(sii6400_spi_sem);
	if (SII_OS_STATUS_SUCCESS != sii_os_rv)
		err("Could not release SPI semaphore: %x\n", sii_os_rv);

done_enable_ints:
	(void)enable_interrupts_spi();
done:
	SiiOsFree(bulk_data_xfer_buf);
	return retStatus;
}

/*
 * Called when all interrupts have been serviced.
 */
void SiiHalIsrDone(void)
{
#ifdef USE_INTERRUPT_DISABLE
	sii6400_in_irq = false;
#endif /* USE_INTERRUPT_DISABLE */

	/*dbg("");*/
}

