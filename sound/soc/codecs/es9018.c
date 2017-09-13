/*
 * es9018.c -- es9018 ALSA SoC audio driver
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <trace/events/asoc.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/wakelock.h>

#define INPUT_CONFIG_SOURCE 1
#define I2S_BIT_FORMAT_MASK (0x03 << 6)
#define MASTER_MODE_CONTROL 10
#define I2S_CLK_DIVID_MASK (0x03 << 5)
#define RIGHT_CHANNEL_VOLUME_15 15
#define LEFT_CHANNEL_VOLUME_16 16
#define MASTER_TRIM_VOLUME_17 17
#define MASTER_TRIM_VOLUME_18 18
#define MASTER_TRIM_VOLUME_19 19
#define MASTER_TRIM_VOLUME_20 20
#define HEADPHONE_AMPLIFIER_CONTROL 42

/* codec private data */
struct es9018_data {
	int clk_441k;
	int clk_48k;
	int ldo_0;
	int ldo_1;
	int ldo_2;
	int bypass_ctl;
	int reset_gpio;
};

struct es9018_priv {
	struct snd_soc_codec *codec;
	struct i2c_client *i2c_client;
	struct es9018_data *es9018_data;
	struct delayed_work sleep_work;
	struct mutex power_lock;
} es9018_priv;

struct es9018_reg {
	unsigned char num;
	unsigned char value;
};

static struct wake_lock hifi_wakelock;
static atomic_t crystal_45M_count;
static atomic_t crystal_49M_count;
static atomic_t hifi_45M_count_ref;
static atomic_t hifi_49M_count_ref;
static atomic_t power_supply_count;
static atomic_t bypass_control_count;
static atomic_t reset_control_count;
static struct mutex es9018_lock;
static struct delayed_work power_supply_45M_dwork;
static struct delayed_work power_supply_49M_dwork;
static struct delayed_work power_supply_bypass_dwork;
static struct delayed_work clk_divider_dwork;
static bool hifi_power_supply_45M = false;
static bool hifi_power_supply_49M = false;
static bool bypass_power_supply = false;
static bool bypass_waiting_close = false;
static bool hifi_45M_waiting_close = false;
static bool hifi_49M_waiting_close = false;
static bool hifi_switch = false;
static long divider_value;

#define POWER_SUPPLY_TIMEOUT 1500000
#define CLK_DIVIDER_TIMEOUT 100000

/* We only include the analogue supplies here; the digital supplies
 * need to be available well before this driver can be probed.
 */
struct es9018_reg init_reg_max1[40] = {
	{0x00, 0x00},
	{0x01, 0x8c},  /* I2S input */
	{0x02, 0x18},
	{0x03, 0x10},
	{0x04, 0x05},
	{0x05, 0x68},
	{0x06, 0x4a},  /* 47= 32KHz ; 57=44.1KHz; 67=48KHz */
	{0x07, 0x80},
	{0x08, 0x10},
	{0x09, 0x00},
	{0x0a, 0xe5},
	{0x0b, 0x02},
	{0x0c, 0x5a},
	{0x0d, 0x00},
	{0x0e, 0x8a},
	{0x0f, 0x00},
	{0x10, 0x00},
	{0x11, 0xff},
	{0x12, 0xff},
	{0x13, 0x6f},
	{0x14, 0x72},
	{0x15, 0x00},
	{0x16, 0x00},
	{0x17, 0x00},
	{0x18, 0x90},
	{0x19, 0xff},
	{0x1a, 0x00},
	{0x1b, 0x00},
	{0x1c, 0x00},
	{0x1d, 0x00},
	{0x1e, 0x00},
	{0x1f, 0x01},
	{0x21, 0xff},
	{0x27, 0x22},
	{0x28, 0x30},
	{0x29, 0x06},
	{0x2a, 0x70},
	{0x2b, 0x00},
	{0x31, 0x00},
	{0x33, 0x00}
};

static int es9018_register_dump = -1;
static struct es9018_priv *g_es9018_priv = NULL;
static int es9018_write_reg(struct i2c_client *client, int reg, u8 value);
static int es9018_read_reg(struct i2c_client *client, int reg);

#define ES9018_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |	\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |	\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |	\
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)

#define ES9018_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE | \
		SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE | \
		SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)


static int es9018_register_dump_parm_set(const char *val, struct kernel_param *kp)
{
	int i, reg_val;
	param_set_int(val, kp);

	if (1 == es9018_register_dump) {
		for (i=0; i<51; i++) {
			reg_val = es9018_read_reg(g_es9018_priv->i2c_client, i);
			pr_info("enter %s, regester[%#x] = %#x\n",
				__func__, i, reg_val);
			udelay(20);
		}
	} else {
		pr_info("enter %s, out of register dump!\n", __func__);
	}
	return 0;
}
module_param_call(es9018_register_dump, es9018_register_dump_parm_set,
			param_get_int, &es9018_register_dump, 0664);

static void power_gpio_0_H(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->ldo_0, 1);
	if (ret < 0) {
		pr_err("%s: gpio_0_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void power_gpio_0_L(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->ldo_0, 0);
	if (ret < 0) {
		pr_err("%s: gpio_0_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void power_gpio_1_H(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->ldo_1, 1);
	if (ret < 0) {
		pr_err("%s: gpio_1_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void power_gpio_1_L(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->ldo_1, 0);
	if (ret < 0) {
		pr_err("%s: gpio_1_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void power_gpio_2_H(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->ldo_2, 1);
	if (ret < 0) {
		pr_err("%s: gpio_2_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void power_gpio_2_L(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->ldo_2, 0);
	if (ret < 0) {
		pr_err("%s: gpio_2_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void clk_gpio_441k_H(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->clk_441k, 1);
	if (ret < 0) {
		pr_err("%s: clk_441k_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void clk_gpio_441k_L(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->clk_441k, 0);
	if (ret < 0) {
		pr_err("%s: clk_441k_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void clk_gpio_48k_H(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->clk_48k, 1);
	if (ret < 0) {
		pr_err("%s: clk_48k_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void clk_gpio_48k_L(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->clk_48k, 0);
	if (ret < 0) {
		pr_err("%s: clk_48k_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void reset_gpio_H(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->reset_gpio, 1);
	if (ret < 0) {
		pr_err("%s: reset_gpio_level direction failed\n",
				__func__);
		return;
	}
	msleep(2);
	return;
}

static void reset_gpio_L(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->reset_gpio, 0);
	if (ret < 0) {
		pr_err("%s: reset_gpio_level direction failed\n",
				__func__);
		return;
	}
	return;
}

static void bypass_ctl_gpio_H(void)
{
	int ret;

	pr_info("%s: enter\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->bypass_ctl, 1);
	if (ret < 0) {
		pr_err("%s: bypass_ctl_gpio direction failed\n",
				__func__);
		return;
	}
	return;
}

static void bypass_ctl_gpio_L(void)
{
	int ret;

	pr_info("enter %s\n", __func__);
	ret = gpio_direction_output(g_es9018_priv->es9018_data->bypass_ctl, 0);
	if (ret < 0) {
		pr_err("%s: bypass_ctl_gpio direction failed\n",
				__func__);
		return;
	}
	return;
}

void es9018_power_supply_enable(bool enable)
{
	mutex_lock(&es9018_lock);
	if (enable)
	{
		pr_debug("%s enter, enable!\n", __func__);
		if (atomic_inc_return(&power_supply_count) == 1) {
			pr_info("%s enable power!\n", __func__);
			power_gpio_0_H();
			power_gpio_1_H();
			power_gpio_2_H();
		}
		if (atomic_inc_return(&bypass_control_count) == 1) {
			pr_info("%s enable bypass!\n", __func__);
			bypass_ctl_gpio_H();
		}
	} else {
		pr_debug("%s enter, disable!\n", __func__);
		if (atomic_read(&power_supply_count) > 0) {
			if (atomic_dec_return(&power_supply_count) == 0) {
				pr_info("%s disable power!\n", __func__);
				power_gpio_2_L();
				power_gpio_1_L();
				power_gpio_0_L();
			}
		}
		if (atomic_read(&bypass_control_count) > 0) {
			if (atomic_dec_return(&bypass_control_count) == 0) {
				pr_info("%s disable bypass!\n", __func__);
				bypass_ctl_gpio_L();
			}
		}
	}
	mutex_unlock(&es9018_lock);
	return;
}

static void es9018_init_hifi_reg(void)
{
	int i = 0;
	int num;

	num = sizeof(init_reg_max1)/sizeof(struct es9018_reg);
	for (i = 0; i < num; i++) {
		es9018_write_reg(g_es9018_priv->i2c_client,
				init_reg_max1[i].num,
				init_reg_max1[i].value);
	}

	return;
}

static void power_supply_45M_handler(struct work_struct *work)
{
	pr_info("%s enter\n", __func__);

	mutex_lock(&es9018_lock);
	if (!hifi_45M_waiting_close) {
		pr_err("%s 45M HIFI schedule canceled!\n", __func__);
		mutex_unlock(&es9018_lock);
		return;
	}
	if (atomic_read(&reset_control_count) > 0) {
		if (atomic_dec_return(&reset_control_count) == 0) {
			pr_info("%s enter, disable reset!\n", __func__);
			reset_gpio_L();
		} else {
			pr_info("%s, reset used by others!\n", __func__);
		}
	}
	if (atomic_read(&power_supply_count) > 0) {
		if (atomic_dec_return(&power_supply_count) == 0) {
			pr_info("%s , disable power!\n", __func__);
			power_gpio_0_L();
			power_gpio_1_L();
			power_gpio_2_L();
		} else {
			pr_info("%s, power used by others!\n", __func__);
		}
	}
	if (atomic_read(&crystal_45M_count) > 0) {
		if (atomic_dec_return(&crystal_45M_count) == 0) {
			pr_info("%s , disable 45M!\n", __func__);
			clk_gpio_441k_L();
		} else {
			pr_info("%s, 45M used by others!\n", __func__);
		}
	}
	hifi_power_supply_45M = false;
	hifi_45M_waiting_close = false;
	mutex_unlock(&es9018_lock);
	return;
}

static int es9018_45M_open(void)
{
	pr_info("enter %s\n", __func__);
	mutex_lock(&es9018_lock);
	if (hifi_power_supply_49M) {
		pr_info("%s disable 49M crystal!\n", __func__);
reagin_49M_dec:
		if (atomic_read(&crystal_49M_count) > 0) {
			if (atomic_dec_return(&crystal_49M_count) == 0) {
				pr_info("%s enter, disable 49M crystal\n",
						__func__);
				/* software mute */
				es9018_write_reg(g_es9018_priv->i2c_client,
					0x07, 0x83);
				clk_gpio_48k_L();
			} else {
				pr_info("%s, 49M used by others!\n", __func__);
				goto reagin_49M_dec;
			}
		}
	}
	if (bypass_power_supply) {
		pr_info("%s disable bypass!\n", __func__);
reagin_bypass_dec_45M:
		if (atomic_read(&bypass_control_count) > 0) {
			if (atomic_dec_return(&bypass_control_count) == 0) {
				pr_info("%s enter, disable bypass\n",
						__func__);
				bypass_ctl_gpio_L();
			} else {
				pr_info("%s, bypass used by others!\n", __func__);
				goto reagin_bypass_dec_45M;
			}
		}
		msleep(600);
	}

	if (hifi_45M_waiting_close) {
		pr_info("%s, 45M hifi working still!\n", __func__);
		cancel_delayed_work(&power_supply_45M_dwork);
		hifi_45M_waiting_close = false;
		if (atomic_read(&crystal_45M_count) == 0) {
			if (atomic_inc_return(&crystal_45M_count) == 1) {
				pr_info("%s, enable 45M crystal\n",
						__func__);
				clk_gpio_441k_H();
				msleep(5);
			}
			es9018_init_hifi_reg();
		} else if (atomic_read(&crystal_45M_count) >= 1) {
			/* software unmute */
			es9018_write_reg(g_es9018_priv->i2c_client,
					0x07, 0x80);
		}
		goto hifi_45M_working;
	}

	if (atomic_inc_return(&crystal_45M_count) == 1) {
		pr_info("%s enter, enable 45M crystal\n",
				__func__);
		clk_gpio_441k_H();
		msleep(5);
	} else {
		pr_info("%s enter, 45M crystal has been enabled!\n",
				__func__);
	}
	if (atomic_inc_return(&power_supply_count) == 1) {
		pr_info("%s , enable power!\n", __func__);
		power_gpio_0_H();
		power_gpio_1_H();
		power_gpio_2_H();
	} else {
		pr_info("%s , power enabled by others!\n", __func__);
	}
	if (atomic_inc_return(&reset_control_count) == 1) {
		pr_info("%s enter, enable reset!\n", __func__);
		reset_gpio_H();
	} else {
		pr_info("%s , reset enabled by others!\n", __func__);
	}

	es9018_init_hifi_reg();

hifi_45M_working:
	hifi_power_supply_45M = true;
	mutex_unlock(&es9018_lock);

	return 0;
}

static int es9018_45M_close(void)
{
	schedule_delayed_work(&power_supply_45M_dwork,
			usecs_to_jiffies(POWER_SUPPLY_TIMEOUT));
	wake_lock_timeout(&hifi_wakelock, 3*HZ);

	/* software mute */
	es9018_write_reg(g_es9018_priv->i2c_client,
			0x07, 0x83);
	hifi_45M_waiting_close = true;
	pr_info("enter %s\n", __func__);
	return 0;
}

static void power_supply_49M_handler(struct work_struct *work)
{
	pr_info("%s enter\n", __func__);

	mutex_lock(&es9018_lock);
	if (!hifi_49M_waiting_close) {
		pr_err("%s 49M HIFI schedule canceled!\n", __func__);
		mutex_unlock(&es9018_lock);
		return;
	}
	if (atomic_read(&reset_control_count) > 0) {
		if (atomic_dec_return(&reset_control_count) == 0) {
			pr_info("%s enter, disable reset!\n", __func__);
			reset_gpio_L();
		} else {
			pr_info("%s, reset used by others!\n", __func__);
		}
	}
	if (atomic_read(&power_supply_count) > 0) {
		if (atomic_dec_return(&power_supply_count) == 0) {
			pr_info("%s , disable power!\n", __func__);
			power_gpio_0_L();
			power_gpio_1_L();
			power_gpio_2_L();
		} else {
			pr_info("%s, power used by others!\n", __func__);
		}
	}
	if (atomic_read(&crystal_49M_count) > 0) {
		if (atomic_dec_return(&crystal_49M_count) == 0) {
			pr_info("%s , disable 49M!\n", __func__);
			clk_gpio_48k_L();
		} else {
			pr_info("%s, 49M used by others!\n", __func__);
		}
	}
	hifi_power_supply_49M = false;
	hifi_49M_waiting_close = false;
	mutex_unlock(&es9018_lock);
	return;
}

static int es9018_49M_open(void)
{
	pr_info("enter %s\n", __func__);
	mutex_lock(&es9018_lock);
	if (hifi_power_supply_45M) {
		pr_info("%s disable 45M crystal!\n", __func__);
reagin_45M_dec:
		if (atomic_read(&crystal_45M_count) > 0) {
			if (atomic_dec_return(&crystal_45M_count) == 0) {
				pr_info("%s enter, disable 45M crystal\n",
						__func__);
				/* software mute */
				es9018_write_reg(g_es9018_priv->i2c_client,
					0x07, 0x83);
				clk_gpio_441k_L();
			} else {
				pr_info("%s, 45M used by others!\n", __func__);
				goto reagin_45M_dec;
			}
		}
	}
	if (bypass_power_supply) {
		pr_info("%s disable bypass!\n", __func__);
reagin_bypass_dec_49M:
		if (atomic_read(&bypass_control_count) > 0) {
			if (atomic_dec_return(&bypass_control_count) == 0) {
				pr_info("%s enter, disable bypass\n",
						__func__);
				bypass_ctl_gpio_L();
			} else {
				pr_info("%s, bypass used by others!\n", __func__);
				goto reagin_bypass_dec_49M;
			}
		}
		msleep(600);
	}

	if (hifi_49M_waiting_close) {
		pr_info("%s, 49M hifi working still!\n", __func__);
		cancel_delayed_work(&power_supply_49M_dwork);
		hifi_49M_waiting_close = false;
		if (atomic_read(&crystal_49M_count) == 0) {
			if (atomic_inc_return(&crystal_49M_count) == 1) {
				pr_info("%s, enable 49M crystal\n",
						__func__);
				clk_gpio_48k_H();
				msleep(5);
			}
			es9018_init_hifi_reg();
		} else if (atomic_read(&crystal_49M_count) >= 1) {
			/* software unmute */
			es9018_write_reg(g_es9018_priv->i2c_client,
					0x07, 0x80);
		}
		goto hifi_49M_working;
	}

	if (atomic_inc_return(&crystal_49M_count) == 1) {
		pr_info("%s enter, enable 49M crystal\n",
				__func__);
		clk_gpio_48k_H();
		msleep(5);
	} else {
		pr_info("%s enter, 49M crystal has been enabled!\n",
				__func__);
	}
	if (atomic_inc_return(&power_supply_count) == 1) {
		pr_info("%s , enable power!\n", __func__);
		power_gpio_0_H();
		power_gpio_1_H();
		power_gpio_2_H();
	} else {
		pr_info("%s , power enabled by others!\n", __func__);
	}
	if (atomic_inc_return(&reset_control_count) == 1) {
		pr_info("%s enter, enable reset!\n", __func__);
		reset_gpio_H();
	} else {
		pr_info("%s , reset enabled by others!\n", __func__);
	}

	es9018_init_hifi_reg();

hifi_49M_working:
	hifi_power_supply_49M = true;
	mutex_unlock(&es9018_lock);

	return 0;
}

static int es9018_49M_close(void)
{
	schedule_delayed_work(&power_supply_49M_dwork,
			usecs_to_jiffies(POWER_SUPPLY_TIMEOUT));
	wake_lock_timeout(&hifi_wakelock, 3*HZ);

	/* software mute */
	es9018_write_reg(g_es9018_priv->i2c_client,
			0x07, 0x83);
	hifi_49M_waiting_close = true;
	pr_info("enter %s\n", __func__);
	return 0;
}

static int es9018_get_hifi_45M_enum(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		(__gpio_get_value(g_es9018_priv->es9018_data->ldo_0) &&
		__gpio_get_value(g_es9018_priv->es9018_data->ldo_1));

	return 0;
}

static int es9018_put_hifi_45M_enum(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;

	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
			__func__, ucontrol->value.integer.value[0]);

	if (ucontrol->value.integer.value[0]) {
		if (atomic_inc_return(&hifi_45M_count_ref) == 1) {
			ret = es9018_45M_open();
		} else {
			pr_info("enter %s, has been opened by other!\n",
					__func__);
		}
	} else {
		if (atomic_read(&hifi_45M_count_ref) > 0) {
			if (atomic_dec_return(&hifi_45M_count_ref) == 0) {
				ret = es9018_45M_close();
			}
		} else
			pr_info("enter %s, counter error!\n", __func__);
	}

	return ret;
}

static int es9018_get_hifi_49M_enum(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		(__gpio_get_value(g_es9018_priv->es9018_data->ldo_0) &&
		__gpio_get_value(g_es9018_priv->es9018_data->ldo_1));

	return 0;
}

static int es9018_put_hifi_49M_enum(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;

	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
			__func__, ucontrol->value.integer.value[0]);

	if (ucontrol->value.integer.value[0]) {
		if (atomic_inc_return(&hifi_49M_count_ref) == 1) {
			ret = es9018_49M_open();
		} else {
			pr_info("enter %s, has been opened by other!\n",
					__func__);
		}
	} else {
		if (atomic_read(&hifi_49M_count_ref) > 0) {
			if (atomic_dec_return(&hifi_49M_count_ref) == 0) {
				ret = es9018_49M_close();
			}
		} else
			pr_info("enter %s, counter error!\n", __func__);
	}

	return ret;
}

static int es9018_get_i2s_length(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;

	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				INPUT_CONFIG_SOURCE);
	reg_val = reg_val >> 6;
	ucontrol->value.integer.value[0] = reg_val;

	pr_info("%s: i2s_length = 0x%x\n", __func__, reg_val);

	return 0;
}

static int es9018_set_i2s_length(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;

	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
		__func__, ucontrol->value.integer.value[0]);

	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				INPUT_CONFIG_SOURCE);

	reg_val &= ~(I2S_BIT_FORMAT_MASK);
	reg_val |=  ucontrol->value.integer.value[0] << 6;

	es9018_write_reg(g_es9018_priv->i2c_client,
				INPUT_CONFIG_SOURCE, reg_val);
	return 0;
}

static int crystal_45M_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		__gpio_get_value(g_es9018_priv->es9018_data->clk_441k);

	return 0;
}

static int crystal_45M_set(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	pr_info("%s enter\n", __func__);
	if (ucontrol->value.integer.value[0] == 1) {
		if (atomic_inc_return(&crystal_45M_count) == 1) {
			pr_info("%s enter, enable 45M crystal\n",
					__func__);
			clk_gpio_441k_H();
		} else {
			pr_info("%s enter, 45M crystal has been enabled!\n",
					__func__);
		}
	} else if (ucontrol->value.integer.value[0] == 0) {
		if (atomic_dec_return(&crystal_45M_count) == 0) {
			pr_info("%s enter, disable 45M crystal\n",
					__func__);
			clk_gpio_441k_L();
		} else {
			pr_info("%s enter, 45M crystal is also used by others!\n",
					__func__);
		}
	}

	return 0;
}

static int crystal_49M_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		__gpio_get_value(g_es9018_priv->es9018_data->clk_48k);

	return 0;
}

static int crystal_49M_set(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{

	pr_info("%s enter\n", __func__);
	if (ucontrol->value.integer.value[0] == 1) {
		if (atomic_inc_return(&crystal_49M_count) == 1) {
			pr_info("%s enter, enable 49M crystal\n",
					__func__);
			clk_gpio_48k_H();
		} else {
			pr_info("%s enter, 49M crystal has been enabled!\n",
					__func__);
		}
	} else if (ucontrol->value.integer.value[0] == 0) {
		if (atomic_dec_return(&crystal_49M_count) == 0) {
			pr_info("%s enter, disable 49M crystal\n",
					__func__);
			clk_gpio_48k_L();
		} else {
			pr_info("%s enter, 49M crystal is also used by others!\n",
					__func__);
		}
	}
	return 0;
}

static void clk_divider_handler(struct work_struct *work)
{
	u8 reg_val;

	pr_info("%s enter\n", __func__);
	if (atomic_read(&bypass_control_count) > 0) {
		pr_info("%s: bypass!\n", __func__);
		return;
	}
	if (!(atomic_read(&crystal_45M_count) > 0) &&
		!(atomic_read(&crystal_49M_count) > 0)) {
		msleep(50);
	}
	mutex_lock(&es9018_lock);
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				MASTER_MODE_CONTROL);

	reg_val &= ~(I2S_CLK_DIVID_MASK);
	reg_val |=  divider_value;

	es9018_write_reg(g_es9018_priv->i2c_client,
				MASTER_MODE_CONTROL, reg_val);
	mutex_unlock(&es9018_lock);

	return;
}

static int es9018_get_clk_divider(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;

	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
				MASTER_MODE_CONTROL);
	reg_val = reg_val >> 5;
	ucontrol->value.integer.value[0] = reg_val;

	pr_info("%s: i2s_length = 0x%x\n", __func__, reg_val);

	return 0;
}

static int es9018_set_clk_divider(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	u8 reg_val;
	int ret;

	if (atomic_read(&bypass_control_count) > 0) {
		pr_info("%s: bypass!\n", __func__);
		return 0;
	}
	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",
			__func__, ucontrol->value.integer.value[0]);

	divider_value = ucontrol->value.integer.value[0] << 5;
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,
			MASTER_MODE_CONTROL);
	if (reg_val < 0) {
		pr_info("%s: read fail, scheduled!\n", __func__);
		schedule_delayed_work(&clk_divider_dwork,
				usecs_to_jiffies(CLK_DIVIDER_TIMEOUT));
		return 0;
	}

	reg_val &= ~(I2S_CLK_DIVID_MASK);
	reg_val |=  ucontrol->value.integer.value[0] << 5;

	ret = es9018_write_reg(g_es9018_priv->i2c_client,
			MASTER_MODE_CONTROL, reg_val);
	if (ret < 0) {
		pr_info("%s: write fail schedule!\n", __func__);
		schedule_delayed_work(&clk_divider_dwork,
				usecs_to_jiffies(CLK_DIVIDER_TIMEOUT));
	}
	return 0;
}

static void power_supply_bypass_handler(struct work_struct *work)
{
	pr_info("%s enter\n", __func__);

	mutex_lock(&es9018_lock);
	if (!bypass_waiting_close) {
		pr_err("%s bypass schedule canceled!\n", __func__);
		mutex_unlock(&es9018_lock);
		return;
	}

	if (atomic_read(&bypass_control_count) > 0) {
		if (atomic_dec_return(&bypass_control_count) == 0) {
			pr_info("%s, disable bypass\n", __func__);
			bypass_ctl_gpio_L();
		} else {
			pr_info("%s, bypass used by others!\n", __func__);
		}
	}

	if (atomic_read(&power_supply_count) > 0) {
		if (atomic_dec_return(&power_supply_count) == 0) {
			pr_info("%s , disable power!\n", __func__);
			power_gpio_0_L();
			power_gpio_1_L();
			power_gpio_2_L();
		} else {
			pr_info("%s, power used by others!\n", __func__);
		}
	}
	bypass_power_supply = false;
	bypass_waiting_close = false;
	mutex_unlock(&es9018_lock);

	return;
}

static int es9018_bypass_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
		__gpio_get_value(g_es9018_priv->es9018_data->bypass_ctl);

	return 0;
}

static int es9018_bypass_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	pr_info("%s enter\n", __func__);
	if (ucontrol->value.integer.value[0] == 1) {
		mutex_lock(&es9018_lock);
		if (bypass_waiting_close) {
			pr_info("%s working still, waiting close!\n", __func__);
			cancel_delayed_work(&power_supply_bypass_dwork);
			if (atomic_read(&bypass_control_count) == 0) {
				if (atomic_inc_return(&bypass_control_count) == 1) {
					pr_info("%s, enable bypass\n", __func__);
					bypass_ctl_gpio_H();
				}
			}
			bypass_waiting_close = false;
			goto bypass_enabled;
		}
		if (bypass_power_supply) {
			pr_info("%s bypass working still!\n", __func__);
			if (atomic_read(&bypass_control_count) == 0) {
				if (atomic_inc_return(&bypass_control_count) == 1) {
					pr_info("%s, enable bypass\n", __func__);
					bypass_ctl_gpio_H();
				}
			}
			goto bypass_enabled;
		}
		if (atomic_inc_return(&power_supply_count) == 1) {
			pr_info("%s, enable power\n", __func__);
			power_gpio_0_H();
			power_gpio_1_H();
			power_gpio_2_H();
		} else {
			pr_info("%s, power enabled by others\n", __func__);
		}

		if (atomic_inc_return(&bypass_control_count) == 1) {
			pr_info("%s, enable bypass\n", __func__);
			bypass_ctl_gpio_H();
		} else {
			pr_info("%s, bypass enabled by others\n", __func__);
		}

bypass_enabled:
		bypass_power_supply = true;
		mutex_unlock(&es9018_lock);
	} else {
		schedule_delayed_work(&power_supply_bypass_dwork,
			usecs_to_jiffies(POWER_SUPPLY_TIMEOUT));
		wake_lock_timeout(&hifi_wakelock, 3*HZ);
		bypass_waiting_close = true;
	}
	return 0;
}

static int es9018_hifi_get(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hifi_switch;
	return 0;
}

static int es9018_hifi_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&es9018_lock);
	if (ucontrol->value.integer.value[0] == 1) {
		hifi_switch = true;
	} else {
		hifi_switch = false;
	}
	mutex_unlock(&es9018_lock);
	pr_info("%s enter, hifi_switch(%s)\n",
		__func__, (hifi_switch?"ON":"OFF"));
	return 0;
}

static const char * const es9018_hifi_state_texts[] = {
	"Off", "On"
};

static const char * const es9018_i2s_length_texts[] = {
	"16bit", "24bit", "32bit", "32bit"
};

static const char * const es9018_clk_divider_texts[] = {
	"DIV4", "DIV8", "DIV16", "DIV16"
};

static const struct soc_enum es9018_hifi_state_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9018_hifi_state_texts),
		es9018_hifi_state_texts);

static const struct soc_enum es9018_i2s_length_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9018_i2s_length_texts),
		es9018_i2s_length_texts);

static const struct soc_enum es9018_clk_divider_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9018_clk_divider_texts),
		es9018_clk_divider_texts);

static struct snd_kcontrol_new es9018_digital_ext_snd_controls[] = {
	/* commit controls */
	SOC_ENUM_EXT("Es9018 HIFI 45M deep-buffer",
			es9018_hifi_state_enum,
			es9018_get_hifi_45M_enum,
			es9018_put_hifi_45M_enum),
	SOC_ENUM_EXT("Es9018 HIFI 49M deep-buffer",
			es9018_hifi_state_enum,
			es9018_get_hifi_49M_enum,
			es9018_put_hifi_49M_enum),
	SOC_ENUM_EXT("Es9018 HIFI 45M compress-offload",
			es9018_hifi_state_enum,
			es9018_get_hifi_45M_enum,
			es9018_put_hifi_45M_enum),
	SOC_ENUM_EXT("Es9018 HIFI 49M compress-offload",
			es9018_hifi_state_enum,
			es9018_get_hifi_49M_enum,
			es9018_put_hifi_49M_enum),
	SOC_ENUM_EXT("Es9018 HIFI 45M low-latency",
			es9018_hifi_state_enum,
			es9018_get_hifi_45M_enum,
			es9018_put_hifi_45M_enum),
	SOC_ENUM_EXT("Es9018 HIFI 49M low-latency",
			es9018_hifi_state_enum,
			es9018_get_hifi_49M_enum,
			es9018_put_hifi_49M_enum),
	SOC_ENUM_EXT("Es9018 I2s Length",
			es9018_i2s_length_enum,
			es9018_get_i2s_length,
			es9018_set_i2s_length),
	SOC_ENUM_EXT("Es9018 CLK Divider",
			es9018_clk_divider_enum,
			es9018_get_clk_divider,
			es9018_set_clk_divider),
	SOC_SINGLE_EXT("Es9018 Crystal 45M Switch", SND_SOC_NOPM,
			0, 0, 0, crystal_45M_get, crystal_45M_set),
	SOC_SINGLE_EXT("Es9018 Crystal 49M Switch", SND_SOC_NOPM,
			0, 0, 0, crystal_49M_get, crystal_49M_set),
	SOC_SINGLE_EXT("Es9018 Bypass Switch", SND_SOC_NOPM, 0, 0, 0,
			es9018_bypass_get, es9018_bypass_set),
	SOC_SINGLE_EXT("Es9018 Hifi Switch", SND_SOC_NOPM, 0, 0, 0,
			es9018_hifi_get, es9018_hifi_set),
};

static int es9018_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int es9018_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static void es9018_request_gpio(struct es9018_data *pdata)
{
	int ret;

	ret = gpio_request(pdata->reset_gpio, "es9018_reset");
	if (ret < 0) {
		pr_err("%s(): reset request failed",
				__func__);
		goto err;
	}

	ret = gpio_request(pdata->clk_441k, "clk_gpio_45M");
	if (ret < 0) {
		pr_err("%s(): clk_gpio_45M request failed",
				__func__);
		goto err;
	}

	ret = gpio_request(pdata->clk_48k, "clk_gpio_49M");
	if (ret < 0) {
		pr_err("%s(): clk_gpio_49M request failed",
				__func__);
		goto err;
	}

	ret = gpio_request(pdata->ldo_0, "ldo_ena_92");
	if (ret < 0) {
		pr_err("%s(): ldo_ena_92 request failed",
				__func__);
		goto err;
	}

	ret = gpio_request(pdata->ldo_1, "ldo_ena_96");
	if (ret < 0) {
		pr_err("%s(): ldo_ena_96 request failed",
				__func__);
		goto err;
	}

	ret = gpio_request(pdata->bypass_ctl, "bypass_ctl_89");
	if (ret < 0) {
		pr_err("%s(): bypass_ctl_89 request failed",
				__func__);
		goto err;
	}
err:
	return;
}

static int es9018_populate_get_pdata(struct device *dev,
		struct es9018_data *pdata)
{

	pdata->reset_gpio = of_get_named_gpio(dev->of_node,
			"es9018,reset-gpio", 0);
	if (pdata->reset_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,reset-gpio", dev->of_node->full_name,
				pdata->reset_gpio);
		goto err;
	}
	dev_dbg(dev, "%s: reset gpio %d", __func__, pdata->reset_gpio);

	pdata->clk_441k = of_get_named_gpio(dev->of_node,
			"es9018,clk_gpio_45M", 0);
	if (pdata->clk_441k < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,clk_gpio_45M", dev->of_node->full_name,
				pdata->clk_441k);
		goto err;
	}
	dev_dbg(dev, "%s: clk_gpio_45M %d", __func__, pdata->clk_441k);

	pdata->clk_48k = of_get_named_gpio(dev->of_node,
			"es9018,clk_gpio_49M", 0);
	if (pdata->clk_48k < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,clk_gpio_49M", dev->of_node->full_name,
				pdata->clk_48k);
		goto err;
	}
	dev_dbg(dev, "%s: clk_gpio_49M %d", __func__, pdata->clk_48k);

	pdata->ldo_0 = of_get_named_gpio(dev->of_node,
			"es9018,ldo_en_16", 0);
	if (pdata->ldo_0 < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,ldo_en_16", dev->of_node->full_name,
				pdata->ldo_0);
		goto err;
	}
	dev_dbg(dev, "%s: ldo_ena_16 %d", __func__, pdata->ldo_0);

	pdata->ldo_1 = of_get_named_gpio(dev->of_node,
			"es9018,ldo_en_40", 0);
	if (pdata->ldo_1 < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,ldo_en_40", dev->of_node->full_name,
				pdata->ldo_1);
		goto err;
	}
	dev_dbg(dev, "%s: ldo_en_40 %d", __func__, pdata->ldo_1);

	pdata->ldo_2 = of_get_named_gpio(dev->of_node,
			"es9018,ldo_en_84", 0);
	if (pdata->ldo_2 < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,ldo_en_84", dev->of_node->full_name,
				pdata->ldo_2);
		goto err;
	}
	dev_dbg(dev, "%s: ldo_en_84 %d", __func__, pdata->ldo_2);

	pdata->bypass_ctl = of_get_named_gpio(dev->of_node,
			"es9018,bypass-ctrl", 0);
	if (pdata->bypass_ctl < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
				"es9018,bypass-ctl", dev->of_node->full_name,
				pdata->bypass_ctl);
		goto err;
	}
	dev_dbg(dev, "%s: bypass-ctrl %d", __func__, pdata->bypass_ctl);

	return 0;
err:
	devm_kfree(dev, pdata);
	return -1;
}

static unsigned int es9018_codec_read(struct snd_soc_codec *codec,
		unsigned int reg)
{
	/* struct es9018_priv *priv = codec->control_data; */
	return 0;
}

static int es9018_codec_write(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	/* struct es9018_priv *priv = codec->control_data; */
	return 0;
}

static int es9018_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int es9018_resume(struct snd_soc_codec *codec)
{

	return 0;
}

static int es9018_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *codec_dai)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */

	return 0;
}

static int es9018_mute(struct snd_soc_dai *dai, int mute)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */

	return 0;

}

static int es9018_set_clkdiv(struct snd_soc_dai *codec_dai,
				int div_id, int div)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */

	return 0;
}

static int es9018_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */

	return 0;
}


static int es9018_set_dai_fmt(struct snd_soc_dai *codec_dai,
				unsigned int fmt)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */

	return 0;
}

static int es9018_set_fll(struct snd_soc_dai *codec_dai,
		int pll_id, int source, unsigned int freq_in,
		unsigned int freq_out)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */

	return 0;
}

static int es9018_pcm_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *codec_dai)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */
	/* struct es9018_priv *priv = codec->control_data; */

	return 0;
}

static const struct snd_soc_dai_ops es9018_dai_ops = {
	.hw_params	= es9018_pcm_hw_params,
	.digital_mute	= es9018_mute,
	.trigger	= es9018_pcm_trigger,
	.set_fmt	= es9018_set_dai_fmt,
	.set_sysclk	= es9018_set_dai_sysclk,
	.set_pll	= es9018_set_fll,
	.set_clkdiv	= es9018_set_clkdiv,
};

static struct snd_soc_dai_driver es9018_dai = {
	.name = "es9018-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ES9018_RATES,
		.formats = ES9018_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ES9018_RATES,
		.formats = ES9018_FORMATS,
	},
	.ops = &es9018_dai_ops,
};

static  int es9018_codec_probe(struct snd_soc_codec *codec)
{
	int rc = 0;
	struct es9018_priv *priv = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: enter\n", __func__);

	priv->codec = codec;

	codec->control_data = snd_soc_codec_get_drvdata(codec);


	rc = snd_soc_add_codec_controls(codec, es9018_digital_ext_snd_controls,
			ARRAY_SIZE(es9018_digital_ext_snd_controls));
	if (rc)
		dev_err(codec->dev, "%s(): es325_digital_snd_controls failed\n",
			__func__);

	return 0;
}

static int  es9018_codec_remove(struct snd_soc_codec *codec)
{
	struct es9018_priv *priv = snd_soc_codec_get_drvdata(codec);


	kfree(priv);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es9018 = {
	.probe = es9018_codec_probe,
	.remove = es9018_codec_remove,
	.suspend = es9018_suspend,
	.resume = es9018_resume,
	.read = es9018_codec_read,
	.write = es9018_codec_write,
};

static int es9018_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct es9018_priv *priv;
	struct es9018_data *pdata;
	int ret = 0;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%s: no support for i2c read/write"
				"byte data\n", __func__);
		return -EIO;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct es9018_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = es9018_populate_get_pdata(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Parsing DT failed(%d)", ret);
			return ret;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct es9018_priv),
			GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
	priv->i2c_client = client;
	priv->es9018_data = pdata;
	i2c_set_clientdata(client, priv);

	g_es9018_priv = priv;

	es9018_request_gpio(pdata);

	if (client->dev.of_node)
		dev_set_name(&client->dev, "%s", "es9018-codec");

	ret = snd_soc_register_codec(&client->dev, &soc_codec_dev_es9018,
			&es9018_dai, 1);

	power_gpio_0_L();
	power_gpio_1_L();
	power_gpio_2_L();
	clk_gpio_441k_L();
	clk_gpio_48k_L();
	reset_gpio_L();
	bypass_ctl_gpio_L();

	mutex_init(&es9018_lock);
	atomic_set(&crystal_45M_count, 0);
	atomic_set(&crystal_49M_count, 0);
	atomic_set(&hifi_45M_count_ref, 0);
	atomic_set(&hifi_49M_count_ref, 0);
	atomic_set(&power_supply_count, 0);
	atomic_set(&bypass_control_count, 0);
	atomic_set(&reset_control_count, 0);
	INIT_DELAYED_WORK(&power_supply_45M_dwork, power_supply_45M_handler);
	INIT_DELAYED_WORK(&power_supply_49M_dwork, power_supply_49M_handler);
	INIT_DELAYED_WORK(&power_supply_bypass_dwork, power_supply_bypass_handler);
	INIT_DELAYED_WORK(&clk_divider_dwork, clk_divider_handler);
	hifi_power_supply_49M = false;
	hifi_power_supply_45M = false;
	bypass_power_supply = false;
	bypass_waiting_close = false;
	hifi_45M_waiting_close = false;
	hifi_49M_waiting_close = false;
	hifi_switch = false;
	wake_lock_init(&hifi_wakelock, WAKE_LOCK_SUSPEND, "hifi_power_wakelock");

	return ret;
}

static int es9018_remove(struct i2c_client *client)
{
	gpio_free(g_es9018_priv->es9018_data->reset_gpio);
	gpio_free(g_es9018_priv->es9018_data->clk_441k);
	gpio_free(g_es9018_priv->es9018_data->clk_48k);
	gpio_free(g_es9018_priv->es9018_data->ldo_0);
	gpio_free(g_es9018_priv->es9018_data->ldo_1);
	gpio_free(g_es9018_priv->es9018_data->bypass_ctl);
	wake_lock_destroy(&hifi_wakelock);
	mutex_destroy(&es9018_lock);

	return 0;
}

static struct of_device_id es9018_match_table[] = {
	{ .compatible = "dac,es9018-codec", },
	{}
};

static const struct i2c_device_id es9018_id[] = {
	{ "es9018", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, isa1200_id);

static struct i2c_driver es9018_i2c_driver = {
	.driver	= {
		.name	= "es9018-codec",
		.of_match_table = es9018_match_table,
	},
	.probe		= es9018_probe,
	.remove		= es9018_remove,
	.id_table	= es9018_id,
};

static int __init es9018_init(void)
{
	return i2c_add_driver(&es9018_i2c_driver);
}

static void __exit es9018_exit(void)
{
	i2c_del_driver(&es9018_i2c_driver);
}

module_init(es9018_init);
module_exit(es9018_exit);

MODULE_DESCRIPTION("ASoC ES9018 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:es9018-codec");
