/* Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include <linux/debugfs.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom_letv.h"

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

DEFINE_MSM_MUTEX(msm_eeprom_mutex);
#ifdef CONFIG_COMPAT
static struct v4l2_file_operations msm_eeprom_v4l2_subdev_fops;
#endif

struct dentry *dirret = NULL;
struct dentry *cam_sensor_name = NULL;
static char str_line[83] = {0};

static int eeprom_debugfs_open(struct inode *inode, struct file *filp)
{
	filp->private_data = inode->i_private;

	return 0;
}

static ssize_t eeprom_debugfs_read(struct file *fp, char __user *user_buffer,
		size_t count, loff_t *position)
{
	int rc = 0;
	struct msm_eeprom_ctrl_t *eeprom_ctrl = (struct msm_eeprom_ctrl_t *)fp->private_data;
	char buffer[5] = {0};
	int i = 0;

	if (eeprom_ctrl == NULL)
		return rc;
	if (*position >= eeprom_ctrl->cal_data.num_data)
		return rc;

	memset(str_line, 0, sizeof(str_line));
	for (i = *position; i < eeprom_ctrl->cal_data.num_data; i++) {
		snprintf(buffer, sizeof(buffer), "0x%02X", eeprom_ctrl->cal_data.mapdata[i]);
		strlcat(str_line, buffer, sizeof(str_line));
		strlcat(str_line, " ", sizeof(str_line));
		 if ((i + 1) % 16 == 0 || eeprom_ctrl->cal_data.num_data < 16) {
			strlcat(str_line, "\r\n", sizeof(str_line));
			CDBG("%s: num_data: %u, position: %lld\n", __func__, eeprom_ctrl->cal_data.num_data, *position);
			break;
		}
	}

	rc = copy_to_user(user_buffer, str_line, strlen(str_line));
	if (0 == rc) {
		*position += strlen(str_line) / 5;
		return strlen(str_line);
	} else {
		return rc;
	}
}

static const struct file_operations eeprom_debugfs_operations = {
	.owner = THIS_MODULE,
	.open = eeprom_debugfs_open,
	.read = eeprom_debugfs_read,
};

static struct msm_camera_i2c_reg_array sensor_reg_array[] =
{
	/* init */
	{0x0103, 0x01, 0},
	/* Res 0 Normal 2688 x 1520 30fps */
	{0x3638, 0x00, 0x00},
	{0x0300, 0x04, 0x00},
	{0x0302, 0x64, 0x00},
	{0x0303, 0x01, 0x00},
	{0x0304, 0x03, 0x00},
	{0x030b, 0x00, 0x00},
	{0x030d, 0x1e, 0x00},
	{0x030e, 0x04, 0x00},
	{0x030f, 0x01, 0x00},
	{0x0312, 0x01, 0x00},
	{0x031e, 0x00, 0x00},
	{0x3000, 0x20, 0x00},
	{0x3002, 0x00, 0x00},
	{0x3018, 0x72, 0x00},
	{0x3019, 0x00, 0x00},
	{0x3020, 0x93, 0x00},
	{0x3021, 0x03, 0x00},
	{0x3022, 0x01, 0x00},
	{0x3031, 0x0a, 0x00},
	{0x3305, 0xf1, 0x00},
	{0x3307, 0x04, 0x00},
	{0x3309, 0x29, 0x00},
	{0x3500, 0x00, 0x00},
	{0x3501, 0x60, 0x00},
	{0x3502, 0x00, 0x00},
	{0x3503, 0x04, 0x00},
	{0x3504, 0x00, 0x00},
	{0x3505, 0x00, 0x00},
	{0x3506, 0x00, 0x00},
	{0x3507, 0x00, 0x00},
	{0x3508, 0x00, 0x00},
	{0x3509, 0x80, 0x00},
	{0x350a, 0x00, 0x00},
	{0x350b, 0x00, 0x00},
	{0x350c, 0x00, 0x00},
	{0x350d, 0x00, 0x00},
	{0x350e, 0x00, 0x00},
	{0x350f, 0x80, 0x00},
	{0x3510, 0x00, 0x00},
	{0x3511, 0x00, 0x00},
	{0x3512, 0x00, 0x00},
	{0x3513, 0x00, 0x00},
	{0x3514, 0x00, 0x00},
	{0x3515, 0x80, 0x00},
	{0x3516, 0x00, 0x00},
	{0x3517, 0x00, 0x00},
	{0x3518, 0x00, 0x00},
	{0x3519, 0x00, 0x00},
	{0x351a, 0x00, 0x00},
	{0x351b, 0x80, 0x00},
	{0x351c, 0x00, 0x00},
	{0x351d, 0x00, 0x00},
	{0x351e, 0x00, 0x00},
	{0x351f, 0x00, 0x00},
	{0x3520, 0x00, 0x00},
	{0x3521, 0x80, 0x00},
	{0x3522, 0x08, 0x00},
	{0x3524, 0x08, 0x00},
	{0x3526, 0x08, 0x00},
	{0x3528, 0x08, 0x00},
	{0x352a, 0x08, 0x00},
	{0x3602, 0x00, 0x00},
	{0x3604, 0x02, 0x00},
	{0x3605, 0x00, 0x00},
	{0x3606, 0x00, 0x00},
	{0x3607, 0x00, 0x00},
	{0x3609, 0x12, 0x00},
	{0x360a, 0x40, 0x00},
	{0x360c, 0x08, 0x00},
	{0x360f, 0xe5, 0x00},
	{0x3608, 0x8f, 0x00},
	{0x3611, 0x00, 0x00},
	{0x3613, 0xf7, 0x00},
	{0x3616, 0x58, 0x00},
	{0x3619, 0x99, 0x00},
	{0x361b, 0x60, 0x00},
	{0x361c, 0x7a, 0x00},
	{0x361e, 0x79, 0x00},
	{0x361f, 0x02, 0x00},
	{0x3631, 0x60, 0x00},
	{0x3632, 0x00, 0x00},
	{0x3633, 0x10, 0x00},
	{0x3634, 0x10, 0x00},
	{0x3635, 0x10, 0x00},
	{0x3636, 0x15, 0x00},
	{0x3646, 0x86, 0x00},
	{0x364a, 0x0b, 0x00},
	{0x3700, 0x17, 0x00},
	{0x3701, 0x22, 0x00},
	{0x3703, 0x10, 0x00},
	{0x370a, 0x37, 0x00},
	{0x3705, 0x00, 0x00},
	{0x3706, 0x63, 0x00},
	{0x3709, 0x3c, 0x00},
	{0x370b, 0x01, 0x00},
	{0x370c, 0x30, 0x00},
	{0x3710, 0x24, 0x00},
	{0x3711, 0x0c, 0x00},
	{0x3716, 0x00, 0x00},
	{0x3720, 0x28, 0x00},
	{0x3729, 0x7b, 0x00},
	{0x372a, 0x84, 0x00},
	{0x372b, 0xbd, 0x00},
	{0x372c, 0xbc, 0x00},
	{0x372e, 0x52, 0x00},
	{0x373c, 0x0e, 0x00},
	{0x373e, 0x33, 0x00},
	{0x3743, 0x10, 0x00},
	{0x3744, 0x88, 0x00},
	{0x374a, 0x43, 0x00},
	{0x374c, 0x00, 0x00},
	{0x374e, 0x23, 0x00},
	{0x3751, 0x7b, 0x00},
	{0x3752, 0x84, 0x00},
	{0x3753, 0xbd, 0x00},
	{0x3754, 0xbc, 0x00},
	{0x3756, 0x52, 0x00},
	{0x375c, 0x00, 0x00},
	{0x3760, 0x00, 0x00},
	{0x3761, 0x00, 0x00},
	{0x3762, 0x00, 0x00},
	{0x3763, 0x00, 0x00},
	{0x3764, 0x00, 0x00},
	{0x3767, 0x04, 0x00},
	{0x3768, 0x04, 0x00},
	{0x3769, 0x08, 0x00},
	{0x376a, 0x08, 0x00},
	{0x376b, 0x20, 0x00},
	{0x376c, 0x00, 0x00},
	{0x376d, 0x00, 0x00},
	{0x376e, 0x00, 0x00},
	{0x3773, 0x00, 0x00},
	{0x3774, 0x51, 0x00},
	{0x3776, 0xbd, 0x00},
	{0x3777, 0xbd, 0x00},
	{0x3781, 0x18, 0x00},
	{0x3783, 0x25, 0x00},
	{0x3800, 0x00, 0x00},
	{0x3801, 0x08, 0x00},
	{0x3802, 0x00, 0x00},
	{0x3803, 0x04, 0x00},
	{0x3804, 0x0a, 0x00},
	{0x3805, 0x97, 0x00},
	{0x3806, 0x05, 0x00},
	{0x3807, 0xfb, 0x00},
	{0x3808, 0x0a, 0x00},
	{0x3809, 0x80, 0x00},
	{0x380a, 0x05, 0x00},
	{0x380b, 0xf0, 0x00},
	{0x380c, 0x09, 0x00},
	{0x380d, 0xC4, 0x00},
	{0x380e, 0x07, 0x00},
	{0x380f, 0xc0, 0x00},
	{0x3810, 0x00, 0x00},
	{0x3811, 0x08, 0x00},
	{0x3812, 0x00, 0x00},
	{0x3813, 0x04, 0x00},
	{0x3814, 0x01, 0x00},
	{0x3815, 0x01, 0x00},
	{0x3819, 0x01, 0x00},
	{0x3820, 0x06, 0x00},
	{0x3821, 0x00, 0x00},
	{0x3829, 0x00, 0x00},
	{0x382a, 0x01, 0x00},
	{0x382b, 0x01, 0x00},
	{0x382d, 0x7f, 0x00},
	{0x3830, 0x04, 0x00},
	{0x3836, 0x01, 0x00},
	{0x3841, 0x02, 0x00},
	{0x3846, 0x08, 0x00},
	{0x3847, 0x07, 0x00},
	{0x3d85, 0x36, 0x00},
	{0x3d8c, 0x71, 0x00},
	{0x3d8d, 0xcb, 0x00},
	{0x3f0a, 0x00, 0x00},
	{0x4000, 0x71, 0x00},
	{0x4001, 0x40, 0x00},
	{0x4002, 0x04, 0x00},
	{0x4003, 0x14, 0x00},
	{0x400e, 0x00, 0x00},
	{0x4011, 0x00, 0x00},
	{0x401a, 0x00, 0x00},
	{0x401b, 0x00, 0x00},
	{0x401c, 0x00, 0x00},
	{0x401d, 0x00, 0x00},
	{0x401f, 0x00, 0x00},
	{0x4020, 0x00, 0x00},
	{0x4021, 0x10, 0x00},
	{0x4022, 0x07, 0x00},
	{0x4023, 0xcf, 0x00},
	{0x4024, 0x09, 0x00},
	{0x4025, 0x60, 0x00},
	{0x4026, 0x09, 0x00},
	{0x4027, 0x6f, 0x00},
	{0x4028, 0x00, 0x00},
	{0x4029, 0x02, 0x00},
	{0x402a, 0x06, 0x00},
	{0x402b, 0x04, 0x00},
	{0x402c, 0x02, 0x00},
	{0x402d, 0x02, 0x00},
	{0x402e, 0x0e, 0x00},
	{0x402f, 0x04, 0x00},
	{0x4302, 0xff, 0x00},
	{0x4303, 0xff, 0x00},
	{0x4304, 0x00, 0x00},
	{0x4305, 0x00, 0x00},
	{0x4306, 0x00, 0x00},
	{0x4308, 0x02, 0x00},
	{0x4500, 0x6c, 0x00},
	{0x4501, 0xc4, 0x00},
	{0x4502, 0x40, 0x00},
	{0x4503, 0x02, 0x00},
	{0x4600, 0x00, 0x00},
	{0x4601, 0xA7, 0x00},
	{0x4800, 0x24, 0x00},
	{0x4813, 0x08, 0x00},
	{0x481f, 0x40, 0x00},
	{0x4829, 0x78, 0x00},
	{0x4837, 0x28, 0x00},
	{0x4b00, 0x2a, 0x00},
	{0x4b0d, 0x00, 0x00},
	{0x4d00, 0x04, 0x00},
	{0x4d01, 0x42, 0x00},
	{0x4d02, 0xd1, 0x00},
	{0x4d03, 0x93, 0x00},
	{0x4d04, 0xf5, 0x00},
	{0x4d05, 0xc1, 0x00},
	{0x5000, 0xf3, 0x00},
	{0x5001, 0x11, 0x00},
	{0x5004, 0x00, 0x00},
	{0x500a, 0x00, 0x00},
	{0x500b, 0x00, 0x00},
	{0x5032, 0x00, 0x00},
	{0x5040, 0x00, 0x00},
	{0x5050, 0x0c, 0x00},
	{0x5500, 0x00, 0x00},
	{0x5501, 0x10, 0x00},
	{0x5502, 0x01, 0x00},
	{0x5503, 0x0f, 0x00},
	{0x8000, 0x00, 0x00},
	{0x8001, 0x00, 0x00},
	{0x8002, 0x00, 0x00},
	{0x8003, 0x00, 0x00},
	{0x8004, 0x00, 0x00},
	{0x8005, 0x00, 0x00},
	{0x8006, 0x00, 0x00},
	{0x8007, 0x00, 0x00},
	{0x8008, 0x00, 0x00},
	{0x3638, 0x00, 0x00},
	{0x3105, 0x31, 0x00},
	{0x301a, 0xf9, 0x00},
	{0x3508, 0x07, 0x00},
	{0x484b, 0x05, 0x00},
	{0x4805, 0x03, 0x00},
	{0x3601, 0x01, 0x00},
	/* start */
	{0x0100, 0x01, 10000},
	{0x3105, 0x11, 0x00},
	{0x301a, 0xF1, 0x00},
	{0x4805, 0x00, 0x00},
	{0x301a, 0xF0, 0x00},
	{0x3208, 0x00, 0x00},
	{0x302a, 0x00, 0x00},
	{0x302a, 0x00, 0x00},
	{0x302a, 0x00, 0x00},
	{0x302a, 0x00, 0x00},
	{0x302a, 0x00, 0x00},
	{0x3601, 0x00, 0x00},
	{0x3638, 0x00, 0x00},
	{0x3208, 0x10, 0x00},
	{0x3208, 0xa0, 0x00},
	/* OTP sequency */
	{0x5000, 0xd3, 0},
	{0x3d84, 0xc0, 0},
	{0x3d88, 0x70, 0},
	{0x3d89, 0x00, 0},
	{0x3d8a, 0x71, 0},
	{0x3d8b, 0x3d, 0},
	{0x3d81, 0x01, 5000},
};

static struct msm_camera_i2c_reg_array sensor_reg_array_ov8865[] =
{
	/* init */
	{0x0103, 0x01, 0x00}, /* software reset */
	{0x0100, 0x00, 0x00}, /* software standby */
	{0x0100, 0x00, 0x00},
	{0x0100, 0x00, 0x00},
	{0x0100, 0x00, 0x00},
	{0x3638, 0xff, 0x00}, /* analog control */
	{0x0302, 0x1e, 0x00}, /* PLL */
	{0x0303, 0x00, 0x00}, /* PLL */
	{0x0304, 0x03, 0x00}, /* PLL */
	{0x030e, 0x00, 0x00}, /* PLL */
	{0x030f, 0x09, 0x00}, /* PLL */
	{0x0312, 0x01, 0x00}, /* PLL */
	{0x031e, 0x0c, 0x00}, /* PLL */
	{0x3015, 0x01, 0x00}, /* clock Div */
	{0x3018, 0x72, 0x00}, /* MIPI 4 lane */
	{0x3020, 0x93, 0x00}, /* clock normal, pclk/1 */
	{0x3022, 0x01, 0x00}, /* pd_mini enable when rst_sync */
	{0x3031, 0x0a, 0x00}, /* 10-bit */
	{0x3106, 0x01, 0x00}, /* PLL */
	{0x3305, 0xf1, 0x00},
	{0x3308, 0x00, 0x00},
	{0x3309, 0x28, 0x00},
	{0x330a, 0x00, 0x00},
	{0x330b, 0x20, 0x00},
	{0x330c, 0x00, 0x00},
	{0x330d, 0x00, 0x00},
	{0x330e, 0x00, 0x00},
	{0x330f, 0x40, 0x00},
	{0x3307, 0x04, 0x00},
	{0x3604, 0x04, 0x00}, /* analog control */
	{0x3602, 0x30, 0x00},
	{0x3605, 0x00, 0x00},
	{0x3607, 0x20, 0x00},
	{0x3608, 0x11, 0x00},
	{0x3609, 0x68, 0x00},
	{0x360a, 0x40, 0x00},
	{0x360c, 0xdd, 0x00},
	{0x360e, 0x0c, 0x00},
	{0x3610, 0x07, 0x00},
	{0x3612, 0x86, 0x00},
	{0x3613, 0x58, 0x00},
	{0x3614, 0x28, 0x00},
	{0x3617, 0x40, 0x00},
	{0x3618, 0x5a, 0x00},
	{0x3619, 0x9b, 0x00},
	{0x361c, 0x00, 0x00},
	{0x361d, 0x60, 0x00},
	{0x3631, 0x60, 0x00},
	{0x3633, 0x10, 0x00},
	{0x3634, 0x10, 0x00},
	{0x3635, 0x10, 0x00},
	{0x3636, 0x10, 0x00},
	{0x3641, 0x55, 0x00}, /* MIPI settings */
	{0x3646, 0x86, 0x00}, /* MIPI settings */
	{0x3647, 0x27, 0x00}, /* MIPI settings */
	{0x364a, 0x1b, 0x00}, /* MIPI settings */
	{0x3500, 0x00, 0x00}, /* exposurre HH */
	{0x3501, 0x4c, 0x00}, /* expouere H */
	{0x3502, 0x00, 0x00}, /* exposure L */
	{0x3503, 0x00, 0x00}, /* gain no delay, exposure no delay */
	{0x3508, 0x02, 0x00}, /* gain H */
	{0x3509, 0x00, 0x00}, /* gain L */
	{0x3700, 0x24, 0x00}, /* sensor control */
	{0x3701, 0x0c, 0x00},
	{0x3702, 0x28, 0x00},
	{0x3703, 0x19, 0x00},
	{0x3704, 0x14, 0x00},
	{0x3705, 0x00, 0x00},
	{0x3706, 0x38, 0x00},
	{0x3707, 0x04, 0x00},
	{0x3708, 0x24, 0x00},
	{0x3709, 0x40, 0x00},
	{0x370a, 0x00, 0x00},
	{0x370b, 0xb8, 0x00},
	{0x370c, 0x04, 0x00},
	{0x3718, 0x12, 0x00},
	{0x3719, 0x31, 0x00},
	{0x3712, 0x42, 0x00},
	{0x3714, 0x12, 0x00},
	{0x371e, 0x19, 0x00},
	{0x371f, 0x40, 0x00},
	{0x3720, 0x05, 0x00},
	{0x3721, 0x05, 0x00},
	{0x3724, 0x02, 0x00},
	{0x3725, 0x02, 0x00},
	{0x3726, 0x06, 0x00},
	{0x3728, 0x05, 0x00},
	{0x3729, 0x02, 0x00},
	{0x372a, 0x03, 0x00},
	{0x372b, 0x53, 0x00},
	{0x372c, 0xa3, 0x00},
	{0x372d, 0x53, 0x00},
	{0x372e, 0x06, 0x00},
	{0x372f, 0x10, 0x00},
	{0x3730, 0x01, 0x00},
	{0x3731, 0x06, 0x00},
	{0x3732, 0x14, 0x00},
	{0x3733, 0x10, 0x00},
	{0x3734, 0x40, 0x00},
	{0x3736, 0x20, 0x00},
	{0x373a, 0x02, 0x00},
	{0x373b, 0x0c, 0x00},
	{0x373c, 0x0a, 0x00},
	{0x373e, 0x03, 0x00},
	{0x3755, 0x40, 0x00},
	{0x3758, 0x00, 0x00},
	{0x3759, 0x4c, 0x00},
	{0x375a, 0x06, 0x00},
	{0x375b, 0x13, 0x00},
	{0x375c, 0x40, 0x00},
	{0x375d, 0x02, 0x00},
	{0x375e, 0x00, 0x00},
	{0x375f, 0x14, 0x00},
	{0x3767, 0x1c, 0x00},
	{0x3768, 0x04, 0x00},
	{0x3769, 0x20, 0x00},
	{0x376c, 0xc0, 0x00},
	{0x376d, 0xc0, 0x00},
	{0x376a, 0x08, 0x00},
	{0x3761, 0x00, 0x00},
	{0x3762, 0x00, 0x00},
	{0x3763, 0x00, 0x00},
	{0x3766, 0xff, 0x00},
	{0x376b, 0x42, 0x00},
	{0x3772, 0x23, 0x00},
	{0x3773, 0x02, 0x00},
	{0x3774, 0x16, 0x00},
	{0x3775, 0x12, 0x00},
	{0x3776, 0x08, 0x00},
	{0x37a0, 0x44, 0x00},
	{0x37a1, 0x3d, 0x00},
	{0x37a2, 0x3d, 0x00},
	{0x37a3, 0x01, 0x00},
	{0x37a4, 0x00, 0x00},
	{0x37a5, 0x08, 0x00},
	{0x37a6, 0x00, 0x00},
	{0x37a7, 0x44, 0x00},
	{0x37a8, 0x58, 0x00},
	{0x37a9, 0x58, 0x00},
	{0x3760, 0x00, 0x00},
	{0x376f, 0x01, 0x00},
	{0x37aa, 0x44, 0x00},
	{0x37ab, 0x2e, 0x00},
	{0x37ac, 0x2e, 0x00},
	{0x37ad, 0x33, 0x00},
	{0x37ae, 0x0d, 0x00},
	{0x37af, 0x0d, 0x00},
	{0x37b0, 0x00, 0x00},
	{0x37b1, 0x00, 0x00},
	{0x37b2, 0x00, 0x00},
	{0x37b3, 0x42, 0x00},
	{0x37b4, 0x42, 0x00},
	{0x37b5, 0x33, 0x00},
	{0x37b6, 0x00, 0x00},
	{0x37b7, 0x00, 0x00},
	{0x37b8, 0x00, 0x00},
	{0x37b9, 0xff, 0x00}, /* sensor control */
	{0x3800, 0x00, 0x00}, /* X start H */
	{0x3801, 0x0c, 0x00}, /* X start L */
	{0x3802, 0x00, 0x00}, /* Y start H */
	{0x3803, 0x0c, 0x00}, /* Y start L */
	{0x3804, 0x0c, 0x00}, /* X end H */
	{0x3805, 0xd3, 0x00}, /* X end L */
	{0x3806, 0x09, 0x00}, /* Y end H */
	{0x3807, 0xa3, 0x00}, /* Y end L */
	{0x3808, 0x06, 0x00}, /* X output size H */
	{0x3809, 0x60, 0x00}, /* X output size L */
	{0x380a, 0x04, 0x00}, /* Y output size H */
	{0x380b, 0xc8, 0x00}, /* Y output size L */
	{0x380c, 0x07, 0x00}, /* HTS H */
	{0x380d, 0x83, 0x00}, /* HTS L */
	{0x380e, 0x04, 0x00}, /* VTS H */
	{0x380f, 0xe0, 0x00}, /* VTS L */
	{0x3810, 0x00, 0x00}, /* ISP X win H */
	{0x3811, 0x04, 0x00}, /* ISP X win L */
	{0x3813, 0x04, 0x00}, /* ISP Y win L */
	{0x3814, 0x03, 0x00}, /* X inc odd */
	{0x3815, 0x01, 0x00}, /* X inc even */
	{0x3820, 0x00, 0x00}, /* flip off */
	{0x3821, 0x67, 0x00}, /* hsync_en_o, fst_vbin, mirror on */
	{0x382a, 0x03, 0x00}, /* Y inc odd */
	{0x382b, 0x01, 0x00}, /* Y inc even */
	{0x3830, 0x08, 0x00}, /* ablc_use_num[5:1] */
	{0x3836, 0x02, 0x00}, /* zline_use_num[5:1] */
	{0x3837, 0x18, 0x00}, /* vts_add_dis, cexp_gt_vts_offs=8 */
	{0x3841, 0xff, 0x00}, /* auto size */
	{0x3846, 0x88, 0x00}, /* Y/X boundary pixel numbber for auto size mode */
	{0x3d85, 0x06, 0x00}, /* OTP power up load data enable, OTP power up load setting enable */
	{0x3d8c, 0x75, 0x00}, /* OTP setting start address H */
	{0x3d8d, 0xef, 0x00}, /* OTP setting start address L */
	{0x3f08, 0x0b, 0x00},
	{0x4000, 0xf1, 0x00}, /* our range trig en, format chg en, gan chg en, exp chg en, median en */
	{0x4001, 0x14, 0x00}, /* left 32 column, final BLC offset limitation enable */
	{0x4005, 0x10, 0x00}, /* BLC target */
	{0x400b, 0x0c, 0x00}, /* start line =0, offset limitation en, cut range function en */
	{0x400d, 0x10, 0x00}, /* offset trigger threshold */
	{0x401b, 0x00, 0x00},
	{0x401d, 0x00, 0x00},
	{0x4020, 0x01, 0x00}, /* anchor left start H */
	{0x4021, 0x20, 0x00}, /* anchor left start L */
	{0x4022, 0x01, 0x00}, /* anchor left end H */
	{0x4023, 0x9f, 0x00}, /* anchor left end L */
	{0x4024, 0x03, 0x00}, /* anchor right start H */
	{0x4025, 0xe0, 0x00}, /* anchor right start L */
	{0x4026, 0x04, 0x00}, /* anchor right end H */
	{0x4027, 0x5f, 0x00}, /* anchor right end L */
	{0x4028, 0x00, 0x00}, /* top zero line start */
	{0x4029, 0x02, 0x00}, /* top zero line number */
	{0x402a, 0x04, 0x00}, /* top black line start */
	{0x402b, 0x04, 0x00}, /* top black line number */
	{0x402c, 0x02, 0x00}, /* bottom zero line start */
	{0x402d, 0x02, 0x00}, /* bottom zero line number */
	{0x402e, 0x08, 0x00}, /* bottom black line start */
	{0x402f, 0x02, 0x00}, /* bottom black line number */
	{0x401f, 0x00, 0x00}, /* anchor one disable */
	{0x4034, 0x3f, 0x00}, /* limitation BLC offset */
	{0x4300, 0xff, 0x00}, /* clip max H */
	{0x4301, 0x00, 0x00}, /* clip min H */
	{0x4302, 0x0f, 0x00}, /* clip min L/clip max L */
	{0x4500, 0x40, 0x00}, /* ADC sync control */
	{0x4503, 0x10, 0x00},
	{0x4601, 0x74, 0x00}, /* V FIFO control */
	{0x481f, 0x32, 0x00}, /* clk_prepare_min */
	{0x4837, 0x16, 0x00}, /* clock period */
	{0x4850, 0x10, 0x00}, /* lane select */
	{0x4851, 0x32, 0x00}, /* lane select */
	{0x4b00, 0x2a, 0x00}, /* LVDS settings */
	{0x4b0d, 0x00, 0x00}, /* LVDS settings */
	{0x4d00, 0x04, 0x00}, /* temperature sensor */
	{0x4d01, 0x18, 0x00}, /* temperature sensor */
	{0x4d02, 0xc3, 0x00}, /* temperature sensor */
	{0x4d03, 0xff, 0x00}, /* temperature sensor */
	{0x4d04, 0xff, 0x00}, /* temperature sensor */
	{0x4d05, 0xff, 0x00}, /* temperature sensor */
	{0x5000, 0x96, 0x00}, /* LENC on, MWB on, BPC on, WPC on */
	{0x5001, 0x01, 0x00}, /* BLC on */
	{0x5002, 0x08, 0x00}, /* vario pixel off */
	{0x5901, 0x00, 0x00},
	{0x5e00, 0x00, 0x00}, /* test pattern off */
	{0x5e01, 0x41, 0x00}, /* window cut enable */
	{0x0100, 0x01, 0x00}, /* wake up, streaming */
	{0x5b00, 0x02, 0x00}, /* OTP DPC start address H */
	{0x5b01, 0xd0, 0x00}, /* OTP DPC start address L */
	{0x5b02, 0x03, 0x00}, /* OTP DPC end address H */
	{0x5b03, 0xff, 0x00}, /* OTP DPC end address L */
	{0x5b05, 0x6c, 0x00}, /* Recover method 11, use 0x3ff to recover cluster, flip option enable */
	{0x5780, 0xfc, 0x00}, /* DPC */
	{0x5781, 0xdf, 0x00}, //
	{0x5782, 0x3f, 0x00}, //
	{0x5783, 0x08, 0x00}, //
	{0x5784, 0x0c, 0x00}, //
	{0x5786, 0x20, 0x00}, //
	{0x5787, 0x40, 0x00}, //
	{0x5788, 0x08, 0x00}, //
	{0x5789, 0x08, 0x00}, //
	{0x578a, 0x02, 0x00}, //
	{0x578b, 0x01, 0x00}, //
	{0x578c, 0x01, 0x00}, //
	{0x578d, 0x0c, 0x00}, //
	{0x578e, 0x02, 0x00}, //
	{0x578f, 0x01, 0x00}, //
	{0x5790, 0x01, 0x00}, /* DPC */
	{0x5800, 0x1d, 0x00}, /* lens correction */
	{0x5801, 0x0e, 0x00},
	{0x5802, 0x0c, 0x00},
	{0x5803, 0x0c, 0x00},
	{0x5804, 0x0f, 0x00},
	{0x5805, 0x22, 0x00},
	{0x5806, 0x0a, 0x00},
	{0x5807, 0x06, 0x00},
	{0x5808, 0x05, 0x00},
	{0x5809, 0x05, 0x00},
	{0x580a, 0x07, 0x00},
	{0x580b, 0x0a, 0x00},
	{0x580c, 0x06, 0x00},
	{0x580d, 0x02, 0x00},
	{0x580e, 0x00, 0x00},
	{0x580f, 0x00, 0x00},
	{0x5810, 0x03, 0x00},
	{0x5811, 0x07, 0x00},
	{0x5812, 0x06, 0x00},
	{0x5813, 0x02, 0x00},
	{0x5814, 0x00, 0x00},
	{0x5815, 0x00, 0x00},
	{0x5816, 0x03, 0x00},
	{0x5817, 0x07, 0x00},
	{0x5818, 0x09, 0x00},
	{0x5819, 0x06, 0x00},
	{0x581a, 0x04, 0x00},
	{0x581b, 0x04, 0x00},
	{0x581c, 0x06, 0x00},
	{0x581d, 0x0a, 0x00},
	{0x581e, 0x19, 0x00},
	{0x581f, 0x0d, 0x00},
	{0x5820, 0x0b, 0x00},
	{0x5821, 0x0b, 0x00},
	{0x5822, 0x0e, 0x00},
	{0x5823, 0x22, 0x00},
	{0x5824, 0x23, 0x00},
	{0x5825, 0x28, 0x00},
	{0x5826, 0x29, 0x00},
	{0x5827, 0x27, 0x00},
	{0x5828, 0x13, 0x00},
	{0x5829, 0x26, 0x00},
	{0x582a, 0x33, 0x00},
	{0x582b, 0x32, 0x00},
	{0x582c, 0x33, 0x00},
	{0x582d, 0x16, 0x00},
	{0x582e, 0x14, 0x00},
	{0x582f, 0x30, 0x00},
	{0x5830, 0x31, 0x00},
	{0x5831, 0x30, 0x00},
	{0x5832, 0x15, 0x00},
	{0x5833, 0x26, 0x00},
	{0x5834, 0x23, 0x00},
	{0x5835, 0x21, 0x00},
	{0x5836, 0x23, 0x00},
	{0x5837, 0x05, 0x00},
	{0x5838, 0x36, 0x00},
	{0x5839, 0x27, 0x00},
	{0x583a, 0x28, 0x00},
	{0x583b, 0x26, 0x00},
	{0x583c, 0x24, 0x00},
	{0x583d, 0xdf, 0x00}, /* lens correction */
	/* pll for 23.88M clk */
	{0x0300, 0x00, 0x00},
	{0x0301, 0x00, 0x00},
	{0x0302, 0x1e, 0x00},
	{0x0303, 0x00, 0x00},
	{0x0304, 0x03, 0x00},
	{0x0305, 0x01, 0x00},
	{0x0306, 0x01, 0x00},
	{0x030a, 0x00, 0x00},
	{0x030b, 0x00, 0x00},
	{0x030c, 0x00, 0x00},
	{0x030d, 0x1f, 0x00},
	{0x030e, 0x07, 0x00},
	{0x030f, 0x00, 0x00},
	{0x0312, 0x01, 0x00},
	{0x3020, 0x93, 0x00},
	{0x3032, 0x80, 0x00},
	{0x3033, 0x24, 0x00},
	{0x3106, 0x01, 0x00},
	{0x4837, 0x16, 0x00},
	{0x3031, 0x0a, 0x00},
	/* 3264x2448_4lane_30fps */
	{0x3501, 0x98, 0x00},
	{0x3502, 0x60, 0x00},
	{0x3700, 0x48, 0x00},
	{0x3701, 0x18, 0x00},
	{0x3702, 0x50, 0x00},
	{0x3703, 0x32, 0x00},
	{0x3704, 0x28, 0x00},
	{0x3705, 0x00, 0x00},
	{0x3706, 0x70, 0x00},
	{0x3707, 0x08, 0x00},
	{0x3708, 0x48, 0x00},
	{0x3709, 0x80, 0x00},
	{0x370a, 0x01, 0x00},
	{0x370b, 0x70, 0x00},
	{0x370c, 0x07, 0x00},
	{0x3718, 0x14, 0x00},
	{0x3719, 0x31, 0x00},
	{0x3712, 0x44, 0x00},
	{0x3714, 0x12, 0x00},
	{0x371e, 0x31, 0x00},
	{0x371f, 0x7f, 0x00},
	{0x3720, 0x0a, 0x00},
	{0x3721, 0x0a, 0x00},
	{0x3724, 0x04, 0x00},
	{0x3725, 0x04, 0x00},
	{0x3726, 0x0c, 0x00},
	{0x3728, 0x0a, 0x00},
	{0x3729, 0x03, 0x00},
	{0x372a, 0x06, 0x00},
	{0x372b, 0xa6, 0x00},
	{0x372c, 0xa6, 0x00},
	{0x372d, 0xa6, 0x00},
	{0x372e, 0x0c, 0x00},
	{0x372f, 0x20, 0x00},
	{0x3730, 0x02, 0x00},
	{0x3731, 0x0c, 0x00},
	{0x3732, 0x28, 0x00},
	{0x3733, 0x10, 0x00},
	{0x3734, 0x40, 0x00},
	{0x3736, 0x30, 0x00},
	{0x373a, 0x04, 0x00},
	{0x373b, 0x18, 0x00},
	{0x373c, 0x14, 0x00},
	{0x373e, 0x06, 0x00},
	{0x3755, 0x40, 0x00},
	{0x3758, 0x00, 0x00},
	{0x3759, 0x4c, 0x00},
	{0x375a, 0x0c, 0x00},
	{0x375b, 0x26, 0x00},
	{0x375c, 0x40, 0x00},
	{0x375d, 0x04, 0x00},
	{0x375e, 0x00, 0x00},
	{0x375f, 0x28, 0x00},
	{0x3767, 0x1e, 0x00},
	{0x3768, 0x04, 0x00},
	{0x3769, 0x20, 0x00},
	{0x376c, 0xc0, 0x00},
	{0x376d, 0xc0, 0x00},
	{0x376a, 0x08, 0x00},
	{0x3761, 0x00, 0x00},
	{0x3762, 0x00, 0x00},
	{0x3763, 0x00, 0x00},
	{0x3766, 0xff, 0x00},
	{0x376b, 0x42, 0x00},
	{0x3772, 0x46, 0x00},
	{0x3773, 0x04, 0x00},
	{0x3774, 0x2c, 0x00},
	{0x3775, 0x13, 0x00},
	{0x3776, 0x10, 0x00},
	{0x37a0, 0x88, 0x00},
	{0x37a1, 0x7a, 0x00},
	{0x37a2, 0x7a, 0x00},
	{0x37a3, 0x02, 0x00},
	{0x37a4, 0x00, 0x00},
	{0x37a5, 0x09, 0x00},
	{0x37a6, 0x00, 0x00},
	{0x37a7, 0x88, 0x00},
	{0x37a8, 0xb0, 0x00},
	{0x37a9, 0xb0, 0x00},
	{0x3760, 0x00, 0x00},
	{0x376f, 0x01, 0x00},
	{0x37aa, 0x88, 0x00},
	{0x37ab, 0x5c, 0x00},
	{0x37ac, 0x5c, 0x00},
	{0x37ad, 0x55, 0x00},
	{0x37ae, 0x19, 0x00},
	{0x37af, 0x19, 0x00},
	{0x37b0, 0x00, 0x00},
	{0x37b1, 0x00, 0x00},
	{0x37b2, 0x00, 0x00},
	{0x37b3, 0x84, 0x00},
	{0x37b4, 0x84, 0x00},
	{0x37b5, 0x66, 0x00},
	{0x37b6, 0x00, 0x00},
	{0x37b7, 0x00, 0x00},
	{0x37b8, 0x00, 0x00},
	{0x37b9, 0xff, 0x00},
	{0x3800, 0x00, 0x00},
	{0x3801, 0x0c, 0x00},
	{0x3802, 0x00, 0x00},
	{0x3803, 0x0c, 0x00},
	{0x3804, 0x0c, 0x00},
	{0x3805, 0xd3, 0x00},
	{0x3806, 0x09, 0x00},
	{0x3807, 0xa3, 0x00},
	{0x3808, 0x0c, 0x00},
	{0x3809, 0xc0, 0x00},
	{0x380a, 0x09, 0x00},
	{0x380b, 0x90, 0x00},
	{0x380c, 0x07, 0x00},
	{0x380d, 0xb0, 0x00},
	{0x380e, 0x09, 0x00},
	{0x380f, 0xca, 0x00},
	{0x3810, 0x00, 0x00},
	{0x3811, 0x04, 0x00},
	{0x3813, 0x02, 0x00},
	{0x3814, 0x01, 0x00},
	{0x3815, 0x01, 0x00},
	{0x3820, 0x00, 0x00},
	{0x3821, 0x46, 0x00},
	{0x382a, 0x01, 0x00},
	{0x382b, 0x01, 0x00},
	{0x3830, 0x04, 0x00},
	{0x3836, 0x01, 0x00},
	{0x3837, 0x18, 0x00},
	{0x3841, 0xff, 0x00},
	{0x3846, 0x48, 0x00},
	{0x3f08, 0x16, 0x00},
	{0x4000, 0xf1, 0x00},
	{0x4001, 0x04, 0x00},
	{0x4005, 0x10, 0x00},
	{0x400b, 0x0c, 0x00},
	{0x400d, 0x10, 0x00},
	{0x401b, 0x00, 0x00},
	{0x401d, 0x00, 0x00},
	{0x4020, 0x02, 0x00},
	{0x4021, 0x40, 0x00},
	{0x4022, 0x03, 0x00},
	{0x4023, 0x3f, 0x00},
	{0x4024, 0x07, 0x00},
	{0x4025, 0xc0, 0x00},
	{0x4026, 0x08, 0x00},
	{0x4027, 0xbf, 0x00},
	{0x4028, 0x00, 0x00},
	{0x4029, 0x02, 0x00},
	{0x402a, 0x04, 0x00},
	{0x402b, 0x04, 0x00},
	{0x402c, 0x02, 0x00},
	{0x402d, 0x02, 0x00},
	{0x402e, 0x08, 0x00},
	{0x402f, 0x02, 0x00},
	{0x401f, 0x00, 0x00},
	{0x4034, 0x3f, 0x00},
	{0x4300, 0xff, 0x00},
	{0x4301, 0x00, 0x00},
	{0x4302, 0x0f, 0x00},
	{0x4500, 0x68, 0x00},
	{0x4503, 0x10, 0x00},
	{0x4601, 0x10, 0x00},
	{0x481f, 0x32, 0x00},
	{0x4850, 0x10, 0x00},
	{0x4851, 0x32, 0x00},
	{0x4b00, 0x2a, 0x00},
	{0x4b0d, 0x00, 0x00},
	{0x4d00, 0x04, 0x00},
	{0x4d01, 0x18, 0x00},
	{0x4d02, 0xc3, 0x00},
	{0x4d03, 0xff, 0x00},
	{0x4d04, 0xff, 0x00},
	{0x4d05, 0xff, 0x00},
	{0x5000, 0x96, 0x00},
	{0x5001, 0x01, 0x00},
	{0x5002, 0x08, 0x00},
	{0x5901, 0x00, 0x00},
	/* start */
	{0x0100, 0x01, 0x00},
	/* OTP sequency */
	{0x5002, 0x00, 0},
	{0x3d84, 0xc0, 0},
	{0x3d88, 0x70, 0},
	{0x3d89, 0x10, 0},
	{0x3d8a, 0x71, 0},
	{0x3d8b, 0x33, 0},
	{0x3d81, 0x01, 5000},
};

static struct msm_otp_data msm_otp_data_tbl[] = {
	{OTP_FRONT_CAMERA_ID, msm_get_otp_front_camera_id},
	{OTP_FRONT_CAMERA_AWB, msm_get_otp_front_camera_awb},
	{OTP_FRONT_CAMERA_DATE, msm_get_otp_front_camera_date},
	{OTP_FRONT_CAMERA_MODULE_ID, msm_get_otp_front_camera_module_id},
	{OTP_FRONT_CAMERA_ALL, msm_get_otp_front_camera_all},
	{OTP_REAR_CAMERA_ID, msm_get_otp_rear_camera_id},
	{OTP_REAR_CAMERA_AWB, msm_get_otp_rear_camera_awb},
	{OTP_REAR_CAMERA_AF, msm_get_otp_rear_camera_af},
	{OTP_REAR_CAMERA_OIS, msm_get_otp_rear_camera_ois},
	{OTP_REAR_CAMERA_DATE, msm_get_otp_rear_camera_date},
	{OTP_REAR_CAMERA_CLAF, msm_get_otp_rear_camera_claf},
	{OTP_REAR_CAMERA_PDAF, msm_get_otp_rear_camera_pdaf},
	{OTP_REAR_CAMERA_MODULE_ID, msm_get_otp_rear_camera_module_id},
	{OTP_REAR_CAMERA_VCM_ID, msm_get_otp_rear_camera_vcm_id},
	{OTP_REAR_CAMERA_ALL, msm_get_otp_rear_camera_all}
};

static struct msm_camera_i2c_reg_setting conf_array =
{
	.reg_setting = sensor_reg_array,
	.size = ARRAY_SIZE(sensor_reg_array),
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting conf_array_ov8865 =
{
	.reg_setting = sensor_reg_array_ov8865,
	.size = ARRAY_SIZE(sensor_reg_array_ov8865),
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static uint8_t msm_set_ov4688_module_id(
	uint8_t *otp_buf, uint32_t num)
{
	uint8_t flag = 0;
	/* Group 1 */
	flag = otp_buf[16];
	flag = (flag >> 6) & 0x3;
	if (flag == 0x1)
		return otp_buf[17];
	/* Group 2 */
	flag = otp_buf[32];
	flag = (flag >> 6) & 0x3;
	if (flag == 0x1)
		return otp_buf[33];
	/* Group 3 */
	flag = otp_buf[48];
	flag = (flag >> 6) & 0x3;
	if (flag == 0x1)
		return otp_buf[49];
	pr_err("%s: ERR: ov4688 invalid 0x%x\n",
		__func__, flag);
	return 0xFF;
}

static uint8_t msm_set_ov8865_module_id(
	uint8_t *otp_buf, uint32_t num)
{
	uint8_t flag = 0;
	flag = otp_buf[0];
	/* Group 1 */
	if( ((flag >> 6) & 0x03) == 0x1 )
		return otp_buf[3];
	/* Group 2 */
	if( ((flag >> 4) & 0x03) == 0x1 )
		return otp_buf[23];
	/* Group 3 */
	if( ((flag >> 2) & 0x03) == 0x1 )
		return otp_buf[43];
	pr_err("%s: ERR: ov8865 invalid 0x%x\n",
		__func__, flag);
	return 0xFF;
}

static uint8_t msm_set_ov8865_plus_module_id(
	uint8_t *otp_buf, uint32_t num)
{
	uint8_t flag = 0;
	flag = otp_buf[0];
	/* Group 1 */
	if( ((flag >> 6) & 0x03) == 0x1 )
		return otp_buf[1];
	/* Group 2 */
	if( ((flag >> 4) & 0x03) == 0x1 )
		return otp_buf[19];
	/* Group 3 */
	if( ((flag >> 2) & 0x03) == 0x1 )
		return otp_buf[37];
	pr_err("%s: ERR: ov8865_plus invalid 0x%x\n",__func__, flag);
	return 0xFF;
}

static uint8_t msm_get_ov8865_date(
	uint8_t *otp_buf, uint32_t num)
{
	uint8_t flag = 0;
	flag = otp_buf[0];
	/* Group 1 */
	if( ((flag >> 6) & 0x03) == 0x1 )
		return 1;
	/* Group 2 */
	if( ((flag >> 4) & 0x03) == 0x1 )
		return 21;
	/* Group 3 */
	if( ((flag >> 2) & 0x03) == 0x1 )
		return 41;
	pr_err("%s: ERR: ov8865 invalid 0x%x\n",__func__, flag);
	return 0;
}

/**
  * msm_get_ov8865_plus_date() - get address offset which is saved ov8865 module id
  * @otp_buf:	buffer saved ov8865 otp data
  * @num:	    camera otp buffer size
  *
  * This function uses to get address offset which is saved ov8865 module id.
  */
static uint8_t msm_get_ov8865_plus_date(
	uint8_t *otp_buf, uint32_t num)
{
	uint8_t flag = 0;
	flag = otp_buf[0];
	/* Group 1 */
	if( ((flag >> 6) & 0x03) == 0x1 )
		return 1;
	/* Group 2 */
	if( ((flag >> 4) & 0x03) == 0x1 )
		return 19;
	/* Group 3 */
	if( ((flag >> 2) & 0x03) == 0x1 )
		return 37;
	pr_err("%s: ERR: ov8865_plus invalid 0x%x\n",__func__, flag);
	return 0;
}

static int8_t msm_otp_do_checksum(uint8_t *otp_buf, uint32_t num,
		uint8_t checksum_low, uint8_t checksum_high, uint8_t flag)
{
	uint8_t loop = 0;
	uint32_t sum = 0;

	for (loop = 0; loop < num; loop++)
		sum = sum + otp_buf[loop];

	sum = sum % 1023;

	if (flag == 1)
		sum = sum + 1;

	if (checksum_low != (sum & 0xff)) {
		pr_err("%s: ERR: Checksum Low byte is err 0x%x\n",
			__func__, checksum_low);
		return -EINVAL;
	}

	if (checksum_high != (sum >> 8)) {
		pr_err("%s: ERR: Checksum High byte is err 0x%x\n",
			__func__, checksum_high);
		return -EINVAL;
	}
	pr_err("%s: succeed\n", __func__);
	return 0;
}

static int msm_set_otp_data(uint8_t *otp_buf, uint32_t num,
		uint32_t subdev_id, const char *eeprom_name)
{
	if (!otp_buf) {
		pr_err("%s: ERR: otp_buff is NULL\n",
			__func__);
		return -EINVAL;
	}

	if (!eeprom_name) {
		pr_err("%s: ERR: eeprom_name is NULL\n",
			__func__);
		return -EINVAL;
	}

	if (num > OTP_CAMERA_BUFF_SIZE) {
		pr_err("%s: ERR: size is overflow %d\n",
			__func__, num);
		return -EINVAL;
	}

	pr_err("%s: Enter\n", __func__);
	if (0 == subdev_id) {
		msm_rear_otp_type_str.otp_camera_num = num;
		msm_rear_otp_type_str.module_id = 0;
		memset(msm_rear_otp_type_str.eeprom_name,
			0, 19);
		memcpy(msm_rear_otp_type_str.eeprom_name,
			eeprom_name, strlen(eeprom_name));
		memset(msm_rear_otp_type_str.otp_camera_buf,
			0, OTP_CAMERA_BUFF_SIZE);
		memcpy(msm_rear_otp_type_str.otp_camera_buf,
			otp_buf, num);
		msm_rear_otp_type_str.otp_data_tbl = msm_otp_data_tbl;
	} else {
		msm_front_otp_type_str.otp_camera_num = num;
		msm_front_otp_type_str.module_id = 0;
		memset(msm_front_otp_type_str.eeprom_name,
			0, 19);
		memcpy(msm_front_otp_type_str.eeprom_name,
			eeprom_name, strlen(eeprom_name));
		memset(msm_front_otp_type_str.otp_camera_buf,
			0, OTP_CAMERA_BUFF_SIZE);
		memcpy(msm_front_otp_type_str.otp_camera_buf,
			otp_buf, num);
		msm_front_otp_type_str.otp_data_tbl = msm_otp_data_tbl;
	}
	pr_err("%s: succeed\n", __func__);
	return 0;
}

static int msm_get_otp_front_camera_id(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	int i = 0, j = 0;

	if (size < MSM_OTP_FRONT_CAMERA_ID_BUFF_SIZE) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size, MSM_OTP_FRONT_CAMERA_ID_BUFF_SIZE);
		return -EINVAL;
	}

	pr_err("%s: %s\n", __func__, msm_front_otp_type_str.eeprom_name);
	if (!strcmp(msm_front_otp_type_str.eeprom_name, "ov4688")) {
		for (i = 0, j = 0; i < 16; i++, j += 2) {
			snprintf(&otp_buf[j], 36, "%02X",
				msm_front_otp_type_str.otp_camera_buf[i]);
		}
	} else if (!strcmp(msm_front_otp_type_str.eeprom_name, "ov8865")) {
		i = msm_front_otp_type_str.otp_camera_num - 16;
		for (j = 0; i < 16; i++, j += 2) {
			snprintf(&otp_buf[j], 36, "%02X",
				msm_front_otp_type_str.otp_camera_buf[i]);
		}
	} else if (!strcmp(msm_front_otp_type_str.eeprom_name, "ov8865_plus")) {
		i = msm_front_otp_type_str.otp_camera_num - 16;
		for (j = 0; i < 16; i++, j += 2) {
			snprintf(&otp_buf[j], 36, "%02X",
				msm_front_otp_type_str.otp_camera_buf[i]);
		}
	} else {
		pr_err("%s: WARNING: eeprom name %s\n", __func__,
				msm_front_otp_type_str.eeprom_name);
		return -EINVAL;
	}
	pr_err("%s: %s succeed\n", __func__, msm_front_otp_type_str.eeprom_name);
	return 0;
}

static int msm_get_otp_front_camera_awb(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	return 0;
}

static int msm_get_otp_front_camera_date(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	int8_t rc;
	uint8_t flag;

	if (size < MSM_OTP_FRONT_CAMERA_DATE_BUFF_SIZE) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size, MSM_OTP_FRONT_CAMERA_DATE_BUFF_SIZE);
		return -EINVAL;
	}

	if (!strcmp(msm_front_otp_type_str.eeprom_name, "ov4688")) {
		pr_err("%s: date\n", __func__);
	} else if (!strcmp(msm_front_otp_type_str.eeprom_name, "ov8865")) {
		flag = msm_get_ov8865_date(
			msm_front_otp_type_str.otp_camera_buf,
			msm_front_otp_type_str.otp_camera_num);
		if (flag == 0) {
			pr_err("%s: ERR: date flag is err\n", __func__);
			return -EINVAL;
		}
		rc = msm_otp_do_checksum(msm_front_otp_type_str.otp_camera_buf+flag,
			18, msm_front_otp_type_str.otp_camera_buf[flag+18],
			msm_front_otp_type_str.otp_camera_buf[flag+19], 1);
		if (rc) {
			pr_err("%s: ERR: date checksum is err\n", __func__);
			return -EINVAL;
		}
		memcpy(otp_buf, msm_front_otp_type_str.otp_camera_buf+flag+15, size);
	}else if (!strcmp(msm_front_otp_type_str.eeprom_name, "ov8865_plus")) {
		flag = msm_get_ov8865_plus_date(
			msm_front_otp_type_str.otp_camera_buf,
			msm_front_otp_type_str.otp_camera_num);
		if (flag == 0) {
			pr_err("%s: ERR: date flag is err\n", __func__);
			return -EINVAL;
		}
		rc = msm_otp_do_checksum(msm_front_otp_type_str.otp_camera_buf+flag,
			16, msm_front_otp_type_str.otp_camera_buf[flag+16],
			msm_front_otp_type_str.otp_camera_buf[flag+17], 1);
		if (rc) {
			pr_err("%s: ERR: date checksum is err\n", __func__);
			return -EINVAL;
		}
		memcpy(otp_buf, msm_front_otp_type_str.otp_camera_buf+flag+1, size);
	} else {
		pr_err("%s: Warning: unknown eeprom name %s\n", __func__,
			msm_front_otp_type_str.eeprom_name);
		return -EINVAL;
	}
	pr_err("%s: %s succeed\n", __func__, msm_front_otp_type_str.eeprom_name);
	return 0;
}

static int msm_get_otp_front_camera_module_id(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	if (size < MSM_OTP_FRONT_CAMERA_MODULE_ID_BUFF_SIZE) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size,
			MSM_OTP_FRONT_CAMERA_MODULE_ID_BUFF_SIZE);
		return -EINVAL;
	}

	if (!strcmp(msm_front_otp_type_str.eeprom_name, "ov4688"))
		msm_front_otp_type_str.module_id = msm_set_ov4688_module_id(
			msm_front_otp_type_str.otp_camera_buf,
			msm_front_otp_type_str.otp_camera_num);
	else if (!strcmp(msm_front_otp_type_str.eeprom_name, "ov8865"))
		msm_front_otp_type_str.module_id = msm_set_ov8865_module_id(
			msm_front_otp_type_str.otp_camera_buf,
			msm_front_otp_type_str.otp_camera_num);
    else if(!strcmp(msm_front_otp_type_str.eeprom_name, "ov8865_plus")){
		msm_front_otp_type_str.module_id = msm_set_ov8865_plus_module_id(
			msm_front_otp_type_str.otp_camera_buf,
			msm_front_otp_type_str.otp_camera_num);
    }
	else {
		pr_err("%s: ERR: unknown eeprom name %s\n", __func__,
				msm_front_otp_type_str.eeprom_name);
		return -EINVAL;
	}
	otp_buf[0] = msm_front_otp_type_str.module_id;
	pr_err("%s: INFO: module_id is 0x%x\n", __func__, otp_buf[0]);
	pr_err("%s: %s succeed\n", __func__, msm_front_otp_type_str.eeprom_name);
	return 0;
}

static int msm_get_otp_rear_camera_id(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	int i = 0, j = 0;

	return 0;
	if (size < MSM_OTP_REAR_CAMERA_ID_BUFF_SIZE) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size, MSM_OTP_REAR_CAMERA_ID_BUFF_SIZE);
		return -EINVAL;
	}

	if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx298")) {
		for (i = 0, j = 0; i < 8; i++, j += 2)
			snprintf(&otp_buf[j], MSM_OTP_REAR_CAMERA_ID_BUFF_SIZE, "%02X",
					msm_rear_otp_type_str.otp_camera_buf[i]);
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx318")) {
		for (i = 15, j = 0; i < 4; i++, j += 2)
			snprintf(&otp_buf[j], MSM_OTP_REAR_CAMERA_ID_BUFF_SIZE, "%02X",
					msm_rear_otp_type_str.otp_camera_buf[i]);
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx230")) {
		for (i = 0, j = 0; i < 8; i++, j += 2)
			snprintf(&otp_buf[j], MSM_OTP_REAR_CAMERA_ID_BUFF_SIZE, "%02X",
					msm_rear_otp_type_str.otp_camera_buf[i]);
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx230_plus")) {
		for (i = 0, j = 0; i < 8; i++, j += 2)
			snprintf(&otp_buf[j], MSM_OTP_REAR_CAMERA_ID_BUFF_SIZE, "%02X",
					msm_rear_otp_type_str.otp_camera_buf[i]);
	} else {
		pr_err("%s: Warning: unknown eeprom name %s\n", __func__,
				msm_rear_otp_type_str.eeprom_name);
		return -EINVAL;
	}
	pr_err("%s: %s succeed\n", __func__, msm_rear_otp_type_str.eeprom_name);
	return 0;
}

static int msm_get_otp_rear_camera_awb(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	return 0;
}

static int msm_get_otp_rear_camera_af(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	return 0;
}

static int msm_get_otp_rear_camera_ois(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	int8_t rc = 0;
	uint8_t flag = 0;

	if (size < MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size, MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE);
		return -EINVAL;
	}

	if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx318")) {
		flag = msm_rear_otp_type_str.otp_camera_buf[2861];
		flag = flag & 0x03;
		if (flag != 0x01) {
			pr_err("%s: ERR: ois flag is NOT valid 0x%x\n", __func__, flag);
			return -EINVAL;
		}
		rc = msm_otp_do_checksum(msm_rear_otp_type_str.otp_camera_buf+2862,
			size, msm_rear_otp_type_str.otp_camera_buf[2900],
			msm_rear_otp_type_str.otp_camera_buf[2901], 0);
		if (rc) {
			pr_err("%s: ERR: ois checksum is err\n", __func__);
			return -EINVAL;
		}
		memcpy(otp_buf, msm_rear_otp_type_str.otp_camera_buf+2862, size);
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx230")) {
		memcpy(otp_buf, msm_rear_otp_type_str.otp_camera_buf+24, 34);
		otp_buf[34] = msm_rear_otp_type_str.otp_camera_buf[63];
		otp_buf[35] = msm_rear_otp_type_str.otp_camera_buf[71];
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx230_plus")) {
		flag = msm_rear_otp_type_str.otp_camera_buf[27];
		flag = (flag >> 6) & 0x03;
		if (flag != 0x01) {
			pr_err("%s: ERR: ois flag is NOT valid 0x%x\n", __func__, flag);
			return -EINVAL;
		}
		rc = msm_otp_do_checksum(msm_rear_otp_type_str.otp_camera_buf+28,
			size-1, msm_rear_otp_type_str.otp_camera_buf[68],
			msm_rear_otp_type_str.otp_camera_buf[69], 1);
		if (rc) {
			pr_err("%s: ERR: ois checksum is err\n", __func__);
			return -EINVAL;
		}
		memcpy(otp_buf, msm_rear_otp_type_str.otp_camera_buf+27, size);
	} else {
		pr_err("%s: Warning: unknown eeprom name %s\n", __func__,
			msm_rear_otp_type_str.eeprom_name);
		return -EINVAL;
	}
	pr_err("%s: %s succeed\n", __func__, msm_rear_otp_type_str.eeprom_name);
	return 0;
}

static int msm_get_otp_rear_camera_date(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	int8_t rc = 0;
	uint8_t flag = 0;

	if (size < MSM_OTP_REAR_CAMERA_DATE_BUFF_SIZE) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size, MSM_OTP_REAR_CAMERA_DATE_BUFF_SIZE);
		return -EINVAL;
	}

	if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx298")) {
		memcpy(otp_buf, msm_rear_otp_type_str.otp_camera_buf, size);
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx318")) {
		flag = msm_rear_otp_type_str.otp_camera_buf[0];
		flag = flag & 0x03;
		if (flag != 0x01) {
			pr_err("%s: ERR: date flag is NOT valid 0x%x\n", __func__, flag);
			return -EINVAL;
		}
		rc = msm_otp_do_checksum(msm_rear_otp_type_str.otp_camera_buf+1,
			18, msm_rear_otp_type_str.otp_camera_buf[19],
			msm_rear_otp_type_str.otp_camera_buf[20], 0);
		if (rc) {
			pr_err("%s: ERR: date checksum is err\n", __func__);
			return -EINVAL;
		}
		memcpy(otp_buf, msm_rear_otp_type_str.otp_camera_buf+4, size);
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx230")) {
		pr_err("%s: warning: date\n", __func__);
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx230_plus")) {
		memcpy(otp_buf, msm_rear_otp_type_str.otp_camera_buf+395, size);
	} else {
		pr_err("%s: Warning: unknown eeprom name %s\n", __func__,
			msm_rear_otp_type_str.eeprom_name);
		return -EINVAL;
	}
	pr_err("%s: %s succeed\n", __func__, msm_rear_otp_type_str.eeprom_name);
	return 0;
}

static int msm_get_otp_rear_camera_claf(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	int8_t rc = 0;
	uint8_t flag = 0;

	if (size < MSM_OTP_REAR_CAMERA_CLAF_BUFF_SIZE) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size, MSM_OTP_REAR_CAMERA_CLAF_BUFF_SIZE);
		return -EINVAL;
	}

	if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx318")) {
		pr_err("%s: INFO: NO CLAF\n", __func__);
		return -EINVAL;
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx230")) {
		pr_err("%s: INFO: NO CLAF\n", __func__);
		return -EINVAL;
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx230_plus")) {
		flag = msm_rear_otp_type_str.otp_camera_buf[555];
		flag = (flag >> 6) & 0x03;
		if (flag != 0x01) {
			pr_err("%s: ERR: claf flag is NOT valid 0x%x\n", __func__, flag);
			return -EINVAL;
		}
		rc = msm_otp_do_checksum(msm_rear_otp_type_str.otp_camera_buf+556,
			size, msm_rear_otp_type_str.otp_camera_buf[566],
			msm_rear_otp_type_str.otp_camera_buf[567], 1);
		if (rc) {
			pr_err("%s: ERR: claf checksum is err\n", __func__);
			return -EINVAL;
		}
		memcpy(otp_buf, msm_rear_otp_type_str.otp_camera_buf+556, size);
	} else {
		pr_err("%s: ERR: unknown eeprom name %s\n", __func__,
			msm_rear_otp_type_str.eeprom_name);
		return -EINVAL;
	}
	pr_err("%s: %s succeed\n", __func__, msm_rear_otp_type_str.eeprom_name);
	return 0;
}

static int msm_get_otp_rear_camera_pdaf(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	return 0;
}

static int msm_get_otp_rear_camera_vcm_id(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	uint8_t vid[4];
	int i = 0;

	if (size < MSM_OTP_REAR_CAMERA_VCM_ID_BUFF_SIZE) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size,
			MSM_OTP_REAR_CAMERA_VCM_ID_BUFF_SIZE);
		return -EINVAL;
	}

	if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx298")) {
		for(i =0; i < MSM_OTP_REAR_CAMERA_VCM_ID_BUFF_SIZE; i++)
			vid[i] = msm_rear_otp_type_str.otp_camera_buf[10+i];
	} else {
		pr_err("%s: Warning: unknown eeprom name %s\n", __func__,
				msm_rear_otp_type_str.eeprom_name);
		return -EINVAL;
	}

	for(i =0; i < MSM_OTP_REAR_CAMERA_VCM_ID_BUFF_SIZE; i++) {
		msm_rear_otp_type_str.vcm_id[i] = vid[i];
		otp_buf[i] = msm_rear_otp_type_str.vcm_id[i];
		pr_err("%s: INFO: vcm_id[%d] is 0x%02x\n", __func__, i, otp_buf[i]);
	}

	return 0;
}
static int msm_get_otp_rear_camera_module_id(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	uint32_t n;
	uint8_t mid;

	if (size < MSM_OTP_REAR_CAMERA_MODULE_ID_BUFF_SIZE) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size,
			MSM_OTP_REAR_CAMERA_MODULE_ID_BUFF_SIZE);
		return -EINVAL;
	}

	if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx298")) {
		mid = msm_rear_otp_type_str.otp_camera_buf[4];
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx318")) {
		mid = msm_rear_otp_type_str.otp_camera_buf[0];
		if ((mid & 0x03) == 0x0) {
			pr_err("%s: %s: flag invalid\n", __func__, msm_rear_otp_type_str.eeprom_name);
			return -EINVAL;
		}
		mid = msm_rear_otp_type_str.otp_camera_buf[1];
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx230")) {
		n = msm_rear_otp_type_str.otp_camera_num;
		mid = msm_rear_otp_type_str.otp_camera_buf[n-1];
	} else if (!strcmp(msm_rear_otp_type_str.eeprom_name, "sony_imx230_plus")) {
		mid = msm_rear_otp_type_str.otp_camera_buf[392];
	} else {
		pr_err("%s: Warning: unknown eeprom name %s\n", __func__,
				msm_rear_otp_type_str.eeprom_name);
		return -EINVAL;
	}

	msm_rear_otp_type_str.module_id = mid;
	otp_buf[0] = msm_rear_otp_type_str.module_id;
	pr_err("%s: INFO: module_id is 0x%x\n", __func__, otp_buf[0]);
	pr_err("%s: %s succeed\n", __func__, msm_rear_otp_type_str.eeprom_name);
	return 0;
}

static int msm_get_otp_rear_camera_all(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	if (size < msm_rear_otp_type_str.otp_camera_num) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size, msm_rear_otp_type_str.otp_camera_num);
		return -EINVAL;
	}

	memcpy(otp_buf, msm_rear_otp_type_str.otp_camera_buf,
		msm_rear_otp_type_str.otp_camera_num);

	return 0;
}

static int msm_get_otp_front_camera_all(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	if (size < msm_front_otp_type_str.otp_camera_num) {
		pr_err("%s: ERR: size is too small %d(%d)\n",
			__func__, size, msm_front_otp_type_str.otp_camera_num);
		return -EINVAL;
	}

	memcpy(otp_buf, msm_front_otp_type_str.otp_camera_buf,
		msm_front_otp_type_str.otp_camera_num);

	return 0;
}

int msm_get_otp_data(uint8_t *otp_buf, uint32_t size,
	uint16_t data_flag)
{
	pr_err("%s: Enter\n", __func__);
	if (data_flag >= OTP_REAR_CAMERA_ID) {
		if (msm_rear_otp_type_str.otp_camera_num < 1 ||
		msm_rear_otp_type_str.otp_camera_num > OTP_CAMERA_BUFF_SIZE) {
			pr_err("%s: ERR: rear_otp_num=%d\n", __func__,
				msm_rear_otp_type_str.otp_camera_num);
			return -EINVAL;
		}
		if (msm_rear_otp_type_str.otp_camera_buf == NULL) {
			pr_err("%s: ERR: rear_otp is NULL\n", __func__);
			return -EINVAL;
		}
	} else {
		if (msm_front_otp_type_str.otp_camera_num < 1 ||
		msm_front_otp_type_str.otp_camera_num > OTP_CAMERA_BUFF_SIZE) {
			pr_err("%s: ERR: front_otp_num=%d\n", __func__,
				msm_front_otp_type_str.otp_camera_num);
			return -EINVAL;
		}
		if (msm_front_otp_type_str.otp_camera_buf == NULL) {
			pr_err("%s: ERR: front_otp is NULL\n", __func__);
			return -EINVAL;
		}
	}
	if (data_flag == msm_otp_data_tbl[data_flag].data_type)
		return msm_otp_data_tbl[data_flag].otp_data_func(otp_buf,
			size, data_flag);
	else {
		pr_err("%s: ERR: data_flag=%d\n", __func__, data_flag);
		return -EINVAL;
	}
}
EXPORT_SYMBOL(msm_get_otp_data);

/**
  * msm_eeprom_verify_sum - verify crc32 checksum
  * @mem:	data buffer
  * @size:	size of data buffer
  * @sum:	expected checksum
  *
  * Returns 0 if checksum match, -EINVAL otherwise.
  */
static int msm_eeprom_verify_sum(const char *mem, uint32_t size, uint32_t sum)
{
	uint32_t crc = ~0;

	/* check overflow */
	if (size > crc - sizeof(uint32_t))
		return -EINVAL;

	crc = crc32_le(crc, mem, size);
	if (~crc != sum) {
		CDBG("%s: expect 0x%x, result 0x%x\n", __func__, sum, ~crc);
		return -EINVAL;
	}
	CDBG("%s: checksum pass 0x%x\n", __func__, sum);
	return 0;
}

/**
  * msm_eeprom_match_crc - verify multiple regions using crc
  * @data:	data block to be verified
  *
  * Iterates through all regions stored in @data.  Regions with odd index
  * are treated as data, and its next region is treated as checksum.  Thus
  * regions of even index must have valid_size of 4 or 0 (skip verification).
  * Returns a bitmask of verified regions, starting from LSB.  1 indicates
  * a checksum match, while 0 indicates checksum mismatch or not verified.
  */
static uint32_t msm_eeprom_match_crc(struct msm_eeprom_memory_block_t *data)
{
	int j, rc;
	uint32_t *sum;
	uint32_t ret = 0;
	uint8_t *memptr;
	struct msm_eeprom_memory_map_t *map;

	if (!data) {
		pr_err("%s data is NULL", __func__);
		return -EINVAL;
	}
	map = data->map;
	memptr = data->mapdata;

	for (j = 0; j + 1 < data->num_map; j += 2) {
		/* empty table or no checksum */
		if (!map[j].mem.valid_size || !map[j+1].mem.valid_size) {
			memptr += map[j].mem.valid_size
				+ map[j+1].mem.valid_size;
			continue;
		}
		if (map[j+1].mem.valid_size != sizeof(uint32_t)) {
			CDBG("%s: malformatted data mapping\n", __func__);
			return -EINVAL;
		}
		sum = (uint32_t *) (memptr + map[j].mem.valid_size);
		rc = msm_eeprom_verify_sum(memptr, map[j].mem.valid_size,
					   *sum);
		if (!rc)
			ret |= 1 << (j/2);
		memptr += map[j].mem.valid_size + map[j+1].mem.valid_size;
	}
	return ret;
}

static int msm_eeprom_get_cmm_data(struct msm_eeprom_ctrl_t *e_ctrl,
				       struct msm_eeprom_cfg_data *cdata)
{
	int rc = 0;
	struct msm_eeprom_cmm_t *cmm_data = &e_ctrl->eboard_info->cmm_data;
	cdata->cfg.get_cmm_data.cmm_support = cmm_data->cmm_support;
	cdata->cfg.get_cmm_data.cmm_compression = cmm_data->cmm_compression;
	cdata->cfg.get_cmm_data.cmm_size = cmm_data->cmm_size;
	return rc;
}

static int eeprom_config_read_cal_data(struct msm_eeprom_ctrl_t *e_ctrl,
	struct msm_eeprom_cfg_data *cdata)
{
	int rc;

	/* check range */
	if (cdata->cfg.read_data.num_bytes >
		e_ctrl->cal_data.num_data) {
		CDBG("%s: Invalid size. exp %u, req %u\n", __func__,
			e_ctrl->cal_data.num_data,
			cdata->cfg.read_data.num_bytes);
		return -EINVAL;
	}
	if (!e_ctrl->cal_data.mapdata)
		return -EFAULT;

	rc = copy_to_user(cdata->cfg.read_data.dbuffer,
		e_ctrl->cal_data.mapdata,
		cdata->cfg.read_data.num_bytes);

	return rc;
}

static int msm_eeprom_config(struct msm_eeprom_ctrl_t *e_ctrl,
	void __user *argp)
{
	struct msm_eeprom_cfg_data *cdata =
		(struct msm_eeprom_cfg_data *)argp;
	int rc = 0;

	CDBG("%s E\n", __func__);
	switch (cdata->cfgtype) {
	case CFG_EEPROM_GET_INFO:
		CDBG("%s E CFG_EEPROM_GET_INFO\n", __func__);
		cdata->is_supported = e_ctrl->is_supported;
		memcpy(cdata->cfg.eeprom_name,
			e_ctrl->eboard_info->eeprom_name,
			sizeof(cdata->cfg.eeprom_name));
		break;
	case CFG_EEPROM_GET_CAL_DATA:
		CDBG("%s E CFG_EEPROM_GET_CAL_DATA\n", __func__);
		cdata->cfg.get_data.num_bytes =
			e_ctrl->cal_data.num_data;
		break;
	case CFG_EEPROM_READ_CAL_DATA:
		CDBG("%s E CFG_EEPROM_READ_CAL_DATA\n", __func__);
		rc = eeprom_config_read_cal_data(e_ctrl, cdata);
		break;
	case CFG_EEPROM_GET_MM_INFO:
		CDBG("%s E CFG_EEPROM_GET_MM_INFO\n", __func__);
		rc = msm_eeprom_get_cmm_data(e_ctrl, cdata);
		break;
	default:
		break;
	}

	CDBG("%s X rc: %d\n", __func__, rc);
	return rc;
}

static int msm_eeprom_get_subdev_id(struct msm_eeprom_ctrl_t *e_ctrl,
				    void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("%s E\n", __func__);
	if (!subdev_id) {
		pr_err("%s failed\n", __func__);
		return -EINVAL;
	}
	*subdev_id = e_ctrl->subdev_id;
	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("%s X\n", __func__);
	return 0;
}

static long msm_eeprom_subdev_ioctl(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("%s E\n", __func__);
	CDBG("%s:%d a_ctrl %p argp %p\n", __func__, __LINE__, e_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_eeprom_get_subdev_id(e_ctrl, argp);
	case VIDIOC_MSM_EEPROM_CFG:
		return msm_eeprom_config(e_ctrl, argp);
	default:
		return -ENOIOCTLCMD;
	}

	CDBG("%s X\n", __func__);
}

static struct msm_camera_i2c_fn_t msm_eeprom_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
	msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll = msm_camera_cci_i2c_poll,
};

static struct msm_camera_i2c_fn_t msm_eeprom_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
	msm_camera_qup_i2c_write_table_w_microdelay,
};

static struct msm_camera_i2c_fn_t msm_eeprom_spi_func_tbl = {
	.i2c_read = msm_camera_spi_read,
	.i2c_read_seq = msm_camera_spi_read_seq,
};

static int msm_eeprom_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_eeprom_ctrl_t *e_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("%s E\n", __func__);
	if (!e_ctrl) {
		pr_err("%s failed e_ctrl is NULL\n", __func__);
		return -EINVAL;
	}
	CDBG("%s X\n", __func__);
	return rc;
}

static int msm_eeprom_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_eeprom_ctrl_t *e_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("%s E\n", __func__);
	if (!e_ctrl) {
		pr_err("%s failed e_ctrl is NULL\n", __func__);
		return -EINVAL;
	}
	CDBG("%s X\n", __func__);
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_eeprom_internal_ops = {
	.open = msm_eeprom_open,
	.close = msm_eeprom_close,
};
/**
  * read_eeprom_memory() - read map data into buffer
  * @e_ctrl:	eeprom control struct
  * @block:	block to be read
  *
  * This function iterates through blocks stored in block->map, reads each
  * region and concatenate them into the pre-allocated block->mapdata
  */
static int read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
			      struct msm_eeprom_memory_block_t *block)
{
        int rc = 0;
	int j;
	uint16_t data = 0;
	struct msm_eeprom_memory_map_t *emap = block->map;
	struct msm_eeprom_board_info *eb_info;
	uint8_t *memptr = block->mapdata;

	if (!e_ctrl) {
		pr_err("%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}

	eb_info = e_ctrl->eboard_info;

	if (!strcmp(eb_info->eeprom_name, "ov4688")) {
		pr_err("%s:%d ov4688 write table\n", __func__, __LINE__);
		e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write_table(&(e_ctrl->i2c_client), &conf_array);
		if (rc < 0) {
			pr_err("%s: Err ov4688 write table failed\n",
					__func__);
			return rc;
		}
		msleep(20);
		pr_err("%s:%d ov4688 write end\n", __func__, __LINE__);
	}

	if (!strcmp(eb_info->eeprom_name, "ov8865")) {
		pr_err("%s:%d ov8865 write table\n", __func__, __LINE__);
		e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write_table(&(e_ctrl->i2c_client), &conf_array_ov8865);
		if (rc < 0) {
			pr_err("%s: Err ov8865 write table failed\n",
					__func__);
			return rc;
		}
		msleep(20);
		pr_err("%s:%d ov8865 write end\n", __func__, __LINE__);
	}

	if (!strcmp(eb_info->eeprom_name, "ov8865_plus")) {
		pr_err("%s:%d ov8865_plus write table\n", __func__, __LINE__);
		e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write_table(&(e_ctrl->i2c_client), &conf_array_ov8865);
		if (rc < 0) {
			pr_err("%s: Err ov8865_plus write table failed\n",
					__func__);
			return rc;
		}
		msleep(20);
		pr_err("%s:%d ov8865_plus write end\n", __func__, __LINE__);
	}

	for (j = 0; j < block->num_map; j++) {
		if (emap[j].saddr.addr) {
			eb_info->i2c_slaveaddr = emap[j].saddr.addr;
			e_ctrl->i2c_client.cci_client->sid =
					eb_info->i2c_slaveaddr >> 1;
			pr_err("qcom,slave-addr = 0x%X\n",
				eb_info->i2c_slaveaddr);
		}

		if (emap[j].page.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].page.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&(e_ctrl->i2c_client), emap[j].page.addr,
				emap[j].page.data, emap[j].page.data_t);
				msleep(emap[j].page.delay);
			if (rc < 0) {
				pr_err("%s: page write failed\n", __func__);
				return rc;
			}
		}
		if (emap[j].pageen.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].pageen.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&(e_ctrl->i2c_client), emap[j].pageen.addr,
				emap[j].pageen.data, emap[j].pageen.data_t);
				msleep(emap[j].pageen.delay);
			if (rc < 0) {
				pr_err("%s: page enable failed\n", __func__);
				return rc;
			}
		}
		if (emap[j].poll.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].poll.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_poll(
				&(e_ctrl->i2c_client), emap[j].poll.addr,
				emap[j].poll.data, emap[j].poll.data_t, emap[j].poll.delay);
				/* msleep(emap[j].poll.delay); */
			if (rc < 0) {
				pr_err("%s: poll failed\n", __func__);
				return rc;
			}
		}

		if (emap[j].mem.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].mem.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), emap[j].mem.addr,
				memptr, emap[j].mem.valid_size);
			if (rc < 0) {
				pr_err("%s: read failed\n", __func__);
				return rc;
			}
			memptr += emap[j].mem.valid_size;
		}
		if (emap[j].pageen.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].pageen.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&(e_ctrl->i2c_client), emap[j].pageen.addr,
				0, emap[j].pageen.data_t);
			if (rc < 0) {
				pr_err("%s: page disable failed\n", __func__);
				return rc;
			}
		}
	}
	if (!strcmp(eb_info->eeprom_name, "ov4688")) {
		pr_err("%s: ov4688 enable 5000[5]\n", __func__);
		e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
			&(e_ctrl->i2c_client), 0x5000, 0xf3, 1);
	}
	if (!strcmp(eb_info->eeprom_name, "ov8865")) {
		pr_err("%s: ov8865 enable OTP_DPC 5002\n", __func__);
		e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
			&(e_ctrl->i2c_client), 0x5002, 0x08, 1);
	}
	if (!strcmp(eb_info->eeprom_name, "ov8865_plus")) {
		pr_err("%s: ov8865_plus enable OTP_DPC 5002\n", __func__);
		e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
			&(e_ctrl->i2c_client), 0x5002, 0x08, 1);
	}
	if (!strcmp(eb_info->eeprom_name, "sony_imx318")) {
		pr_err("%s:%d sony_imx318 read table\n", __func__, __LINE__);
		e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
		rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read(&(e_ctrl->i2c_client), 0x34, &data, 2);
		if (rc < 0) {
			pr_err("%s: Err sony imx318 read failed\n",
					__func__);
			return rc;
		}
		pr_err("%s:%d ES=0x%x\n", __func__, __LINE__, data);
		pr_err("%s:%d sony imx318 read end\n", __func__, __LINE__);
	}
	return rc;
}
/**
  * msm_eeprom_parse_memory_map() - parse memory map in device node
  * @of:	device node
  * @data:	memory block for output
  *
  * This functions parses @of to fill @data.  It allocates map itself, parses
  * the @of node, calculate total data length, and allocates required buffer.
  * It only fills the map, but does not perform actual reading.
  */
static int msm_eeprom_parse_memory_map(struct device_node *of,
				       struct msm_eeprom_memory_block_t *data)
{
	int i, rc = 0;
	char property[PROPERTY_MAXSIZE];
	uint32_t count = 6;
	struct msm_eeprom_memory_map_t *map;

	snprintf(property, PROPERTY_MAXSIZE, "qcom,num-blocks");
	rc = of_property_read_u32(of, property, &data->num_map);
	CDBG("%s: %s %d\n", __func__, property, data->num_map);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		return rc;
	}

	map = kzalloc((sizeof(*map) * data->num_map), GFP_KERNEL);
	if (!map) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return -ENOMEM;
	}
	data->map = map;

	for (i = 0; i < data->num_map; i++) {
		snprintf(property, PROPERTY_MAXSIZE, "qcom,page%d", i);
		rc = of_property_read_u32_array(of, property,
				(uint32_t *) &map[i].page, count);
		if (rc < 0) {
			pr_err("%s: failed %d\n", __func__, __LINE__);
			goto ERROR;
		}

		snprintf(property, PROPERTY_MAXSIZE,
					"qcom,pageen%d", i);
		rc = of_property_read_u32_array(of, property,
			(uint32_t *) &map[i].pageen, count);
		if (rc < 0)
			CDBG("%s: pageen not needed\n", __func__);

		snprintf(property, PROPERTY_MAXSIZE, "qcom,saddr%d", i);
		rc = of_property_read_u32_array(of, property,
			(uint32_t *) &map[i].saddr.addr, 1);
		if (rc < 0)
			CDBG("%s: saddr not needed - block %d\n", __func__, i);

		snprintf(property, PROPERTY_MAXSIZE, "qcom,poll%d", i);
		rc = of_property_read_u32_array(of, property,
				(uint32_t *) &map[i].poll, count);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR;
		}

		snprintf(property, PROPERTY_MAXSIZE, "qcom,mem%d", i);
		rc = of_property_read_u32_array(of, property,
				(uint32_t *) &map[i].mem, count);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR;
		}
		data->num_data += map[i].mem.valid_size;
	}

	CDBG("%s num_bytes %d\n", __func__, data->num_data);

	data->mapdata = kzalloc(data->num_data, GFP_KERNEL);
	if (!data->mapdata) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}
	return rc;

ERROR:
	kfree(data->map);
	memset(data, 0, sizeof(*data));
	return rc;
}

static struct msm_cam_clk_info cam_8960_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_clk", 24000000},
};

static struct msm_cam_clk_info cam_8974_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_src_clk", 19200000},
	[SENSOR_CAM_CLK] = {"cam_clk", 0},
};

static struct v4l2_subdev_core_ops msm_eeprom_subdev_core_ops = {
	.ioctl = msm_eeprom_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_eeprom_subdev_ops = {
	.core = &msm_eeprom_subdev_core_ops,
};

static int msm_eeprom_i2c_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_eeprom_ctrl_t *e_ctrl = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s E\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s i2c_check_functionality failed\n", __func__);
		goto probe_failure;
	}

	e_ctrl = kzalloc(sizeof(*e_ctrl), GFP_KERNEL);
	if (!e_ctrl) {
		pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
		return -ENOMEM;
	}
	e_ctrl->eeprom_v4l2_subdev_ops = &msm_eeprom_subdev_ops;
	e_ctrl->eeprom_mutex = &msm_eeprom_mutex;
	CDBG("%s client = 0x%p\n", __func__, client);
	e_ctrl->eboard_info = (struct msm_eeprom_board_info *)(id->driver_data);
	if (!e_ctrl->eboard_info) {
		pr_err("%s:%d board info NULL\n", __func__, __LINE__);
		rc = -EINVAL;
		goto ectrl_free;
	}
	power_info = &e_ctrl->eboard_info->power_info;
	e_ctrl->i2c_client.client = client;

	/* Set device type as I2C */
	e_ctrl->eeprom_device_type = MSM_CAMERA_I2C_DEVICE;
	e_ctrl->i2c_client.i2c_func_tbl = &msm_eeprom_qup_func_tbl;

	if (e_ctrl->eboard_info->i2c_slaveaddr != 0)
		e_ctrl->i2c_client.client->addr =
					e_ctrl->eboard_info->i2c_slaveaddr;
	power_info->clk_info = cam_8960_clk_info;
	power_info->clk_info_size = ARRAY_SIZE(cam_8960_clk_info);
	power_info->dev = &client->dev;

	/*IMPLEMENT READING PART*/
	/* Initialize sub device */
	v4l2_i2c_subdev_init(&e_ctrl->msm_sd.sd,
		e_ctrl->i2c_client.client,
		e_ctrl->eeprom_v4l2_subdev_ops);
	v4l2_set_subdevdata(&e_ctrl->msm_sd.sd, e_ctrl);
	e_ctrl->msm_sd.sd.internal_ops = &msm_eeprom_internal_ops;
	e_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&e_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	e_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	e_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_EEPROM;
	msm_sd_register(&e_ctrl->msm_sd);
	CDBG("%s success result=%d X\n", __func__, rc);
	return rc;

ectrl_free:
	kfree(e_ctrl);
probe_failure:
	pr_err("%s failed! rc = %d\n", __func__, rc);
	return rc;
}

static int msm_eeprom_i2c_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct msm_eeprom_ctrl_t  *e_ctrl;
	if (!sd) {
		pr_err("%s: Subdevice is NULL\n", __func__);
		return 0;
	}

	e_ctrl = (struct msm_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		pr_err("%s: eeprom device is NULL\n", __func__);
		return 0;
	}

	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
	if (e_ctrl->eboard_info) {
		kfree(e_ctrl->eboard_info->power_info.gpio_conf);
		kfree(e_ctrl->eboard_info);
	}
	kfree(e_ctrl);
	return 0;
}

#define msm_eeprom_spi_parse_cmd(spic, str, name, out, size)		\
	{								\
		if (of_property_read_u32_array(				\
			spic->spi_master->dev.of_node,			\
			str, out, size)) {				\
			return -EFAULT;					\
		} else {						\
			spic->cmd_tbl.name.opcode = out[0];		\
			spic->cmd_tbl.name.addr_len = out[1];		\
			spic->cmd_tbl.name.dummy_len = out[2];		\
		}							\
	}

static int msm_eeprom_spi_parse_of(struct msm_camera_spi_client *spic)
{
	int rc = -EFAULT;
	uint32_t tmp[3];
	msm_eeprom_spi_parse_cmd(spic, "qcom,spiop,read", read, tmp, 3);
	msm_eeprom_spi_parse_cmd(spic, "qcom,spiop,readseq", read_seq, tmp, 3);
	msm_eeprom_spi_parse_cmd(spic, "qcom,spiop,queryid", query_id, tmp, 3);

	rc = of_property_read_u32_array(spic->spi_master->dev.of_node,
					"qcom,eeprom-id", tmp, 2);
	if (rc) {
		pr_err("%s: Failed to get eeprom id\n", __func__);
		return rc;
	}
	spic->mfr_id0 = tmp[0];
	spic->device_id0 = tmp[1];

	return 0;
}

static int msm_eeprom_match_id(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc;
	struct msm_camera_i2c_client *client = &e_ctrl->i2c_client;
	uint8_t id[2];

	rc = msm_camera_spi_query_id(client, 0, &id[0], 2);
	if (rc < 0)
		return rc;
	CDBG("%s: read 0x%x 0x%x, check 0x%x 0x%x\n", __func__, id[0],
		id[1], client->spi_client->mfr_id0,
			client->spi_client->device_id0);
	if (id[0] != client->spi_client->mfr_id0
		    || id[1] != client->spi_client->device_id0)
		return -ENODEV;

	return 0;
}

static int msm_eeprom_get_dt_data(struct msm_eeprom_ctrl_t *e_ctrl)
{
	int rc = 0, i = 0;
	struct msm_eeprom_board_info *eb_info;
	struct msm_camera_power_ctrl_t *power_info =
		&e_ctrl->eboard_info->power_info;
	struct device_node *of_node = NULL;
	struct msm_camera_gpio_conf *gconf = NULL;
	int8_t gpio_array_size = 0;
	uint16_t *gpio_array = NULL;

	eb_info = e_ctrl->eboard_info;
	if (e_ctrl->eeprom_device_type == MSM_CAMERA_SPI_DEVICE)
		of_node = e_ctrl->i2c_client.
			spi_client->spi_master->dev.of_node;
	else if (e_ctrl->eeprom_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		of_node = e_ctrl->pdev->dev.of_node;

	if (!of_node) {
		pr_err("%s: %d of_node is NULL\n", __func__ , __LINE__);
		return -ENOMEM;
	}
	rc = msm_camera_get_dt_vreg_data(of_node, &power_info->cam_vreg,
					     &power_info->num_vreg);
	if (rc < 0)
		return rc;

	rc = msm_camera_get_dt_power_setting_data(of_node,
		power_info->cam_vreg, power_info->num_vreg,
		power_info);
	if (rc < 0)
		goto ERROR1;

	power_info->gpio_conf = kzalloc(sizeof(struct msm_camera_gpio_conf),
					GFP_KERNEL);
	if (!power_info->gpio_conf) {
		rc = -ENOMEM;
		goto ERROR2;
	}
	gconf = power_info->gpio_conf;
	gpio_array_size = of_gpio_count(of_node);
	CDBG("%s gpio count %d\n", __func__, gpio_array_size);

	if (gpio_array_size > 0) {
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
			GFP_KERNEL);
		if (!gpio_array) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR3;
		}
		for (i = 0; i < gpio_array_size; i++) {
			gpio_array[i] = of_get_gpio(of_node, i);
			CDBG("%s gpio_array[%d] = %d\n", __func__, i,
				gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR4;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR4;
		}
		kfree(gpio_array);
	}

	return rc;
ERROR4:
	kfree(gpio_array);
ERROR3:
	kfree(power_info->gpio_conf);
ERROR2:
	kfree(power_info->cam_vreg);
ERROR1:
	kfree(power_info->power_setting);
	return rc;
}


static int msm_eeprom_cmm_dts(struct msm_eeprom_board_info *eb_info,
				struct device_node *of_node)
{
	int rc = 0;
	struct msm_eeprom_cmm_t *cmm_data = &eb_info->cmm_data;

	cmm_data->cmm_support =
		of_property_read_bool(of_node, "qcom,cmm-data-support");
	if (!cmm_data->cmm_support)
		return -EINVAL;
	cmm_data->cmm_compression =
		of_property_read_bool(of_node, "qcom,cmm-data-compressed");
	if (!cmm_data->cmm_compression)
		CDBG("No MM compression data\n");

	rc = of_property_read_u32(of_node, "qcom,cmm-data-offset",
				  &cmm_data->cmm_offset);
	if (rc < 0)
		CDBG("No MM offset data\n");

	rc = of_property_read_u32(of_node, "qcom,cmm-data-size",
				  &cmm_data->cmm_size);
	if (rc < 0)
		CDBG("No MM size data\n");

	CDBG("cmm_support: cmm_compr %d, cmm_offset %d, cmm_size %d\n",
		cmm_data->cmm_compression,
		cmm_data->cmm_offset,
		cmm_data->cmm_size);
	return 0;
}

static int msm_eeprom_spi_setup(struct spi_device *spi)
{
	struct msm_eeprom_ctrl_t *e_ctrl = NULL;
	struct msm_camera_i2c_client *client = NULL;
	struct msm_camera_spi_client *spi_client;
	struct msm_eeprom_board_info *eb_info;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	int rc = 0;

	e_ctrl = kzalloc(sizeof(*e_ctrl), GFP_KERNEL);
	if (!e_ctrl) {
		pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
		return -ENOMEM;
	}
	e_ctrl->eeprom_v4l2_subdev_ops = &msm_eeprom_subdev_ops;
	e_ctrl->eeprom_mutex = &msm_eeprom_mutex;
	client = &e_ctrl->i2c_client;
	e_ctrl->is_supported = 0;

	spi_client = kzalloc(sizeof(*spi_client), GFP_KERNEL);
	if (!spi_client) {
		pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
		kfree(e_ctrl);
		return -ENOMEM;
	}

	rc = of_property_read_u32(spi->dev.of_node, "cell-index",
				  &e_ctrl->subdev_id);
	CDBG("cell-index %d, rc %d\n", e_ctrl->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	e_ctrl->eeprom_device_type = MSM_CAMERA_SPI_DEVICE;
	client->spi_client = spi_client;
	spi_client->spi_master = spi;
	client->i2c_func_tbl = &msm_eeprom_spi_func_tbl;
	client->addr_type = MSM_CAMERA_I2C_3B_ADDR;

	eb_info = kzalloc(sizeof(*eb_info), GFP_KERNEL);
	if (!eb_info)
		goto spi_free;
	e_ctrl->eboard_info = eb_info;
	rc = of_property_read_string(spi->dev.of_node, "qcom,eeprom-name",
		&eb_info->eeprom_name);
	CDBG("%s qcom,eeprom-name %s, rc %d\n", __func__,
		eb_info->eeprom_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto board_free;
	}

	rc = msm_eeprom_cmm_dts(e_ctrl->eboard_info, spi->dev.of_node);
	if (rc < 0)
		CDBG("%s MM data miss:%d\n", __func__, __LINE__);

	power_info = &eb_info->power_info;

	power_info->clk_info = cam_8974_clk_info;
	power_info->clk_info_size = ARRAY_SIZE(cam_8974_clk_info);
	power_info->dev = &spi->dev;

	rc = msm_eeprom_get_dt_data(e_ctrl);
	if (rc < 0)
		goto board_free;

	/* set spi instruction info */
	spi_client->retry_delay = 1;
	spi_client->retries = 0;

	rc = msm_eeprom_spi_parse_of(spi_client);
	if (rc < 0) {
		dev_err(&spi->dev,
			"%s: Error parsing device properties\n", __func__);
		goto board_free;
	}

	/* prepare memory buffer */
	rc = msm_eeprom_parse_memory_map(spi->dev.of_node,
					 &e_ctrl->cal_data);
	if (rc < 0)
		CDBG("%s: no cal memory map\n", __func__);

	/* power up eeprom for reading */
	rc = msm_camera_power_up(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		goto caldata_free;
	}

	/* check eeprom id */
	rc = msm_eeprom_match_id(e_ctrl);
	if (rc < 0) {
		CDBG("%s: eeprom not matching %d\n", __func__, rc);
		goto power_down;
	}
	/* read eeprom */
	if (e_ctrl->cal_data.map) {
		rc = read_eeprom_memory(e_ctrl, &e_ctrl->cal_data);
		if (rc < 0) {
			pr_err("%s: read cal data failed\n", __func__);
			goto power_down;
		}
		e_ctrl->is_supported |= msm_eeprom_match_crc(
						&e_ctrl->cal_data);
	}

	rc = msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		goto caldata_free;
	}

	/* initiazlie subdev */
	v4l2_spi_subdev_init(&e_ctrl->msm_sd.sd,
		e_ctrl->i2c_client.spi_client->spi_master,
		e_ctrl->eeprom_v4l2_subdev_ops);
	v4l2_set_subdevdata(&e_ctrl->msm_sd.sd, e_ctrl);
	e_ctrl->msm_sd.sd.internal_ops = &msm_eeprom_internal_ops;
	e_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&e_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	e_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	e_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_EEPROM;
	msm_sd_register(&e_ctrl->msm_sd);
	e_ctrl->is_supported = (e_ctrl->is_supported << 1) | 1;
	CDBG("%s success result=%d supported=%x X\n", __func__, rc,
	     e_ctrl->is_supported);

	return 0;

power_down:
	msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
caldata_free:
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
board_free:
	kfree(e_ctrl->eboard_info);
spi_free:
	kfree(spi_client);
	kfree(e_ctrl);
	return rc;
}

static int msm_eeprom_spi_probe(struct spi_device *spi)
{
	int irq, cs, cpha, cpol, cs_high;

	CDBG("%s\n", __func__);
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi_setup(spi);

	irq = spi->irq;
	cs = spi->chip_select;
	cpha = (spi->mode & SPI_CPHA) ? 1 : 0;
	cpol = (spi->mode & SPI_CPOL) ? 1 : 0;
	cs_high = (spi->mode & SPI_CS_HIGH) ? 1 : 0;
	CDBG("%s: irq[%d] cs[%x] CPHA[%x] CPOL[%x] CS_HIGH[%x]\n",
			__func__, irq, cs, cpha, cpol, cs_high);
	CDBG("%s: max_speed[%u]\n", __func__, spi->max_speed_hz);

	return msm_eeprom_spi_setup(spi);
}

static int msm_eeprom_spi_remove(struct spi_device *sdev)
{
	struct v4l2_subdev *sd = spi_get_drvdata(sdev);
	struct msm_eeprom_ctrl_t  *e_ctrl;
	if (!sd) {
		pr_err("%s: Subdevice is NULL\n", __func__);
		return 0;
	}

	e_ctrl = (struct msm_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		pr_err("%s: eeprom device is NULL\n", __func__);
		return 0;
	}

	kfree(e_ctrl->i2c_client.spi_client);
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
	if (e_ctrl->eboard_info) {
		kfree(e_ctrl->eboard_info->power_info.gpio_conf);
		kfree(e_ctrl->eboard_info);
	}
	kfree(e_ctrl);
	return 0;
}

#ifdef CONFIG_COMPAT
static int eeprom_config_read_cal_data32(struct msm_eeprom_ctrl_t *e_ctrl,
	void __user *arg)
{
	int rc;
	uint8_t *ptr_dest = NULL;
	struct msm_eeprom_cfg_data32 *cdata32 =
		(struct msm_eeprom_cfg_data32 *) arg;
	struct msm_eeprom_cfg_data cdata;

	cdata.cfgtype = cdata32->cfgtype;
	cdata.is_supported = cdata32->is_supported;
	cdata.cfg.read_data.num_bytes = cdata32->cfg.read_data.num_bytes;
	/* check range */
	if (cdata.cfg.read_data.num_bytes >
	    e_ctrl->cal_data.num_data) {
		CDBG("%s: Invalid size. exp %u, req %u\n", __func__,
			e_ctrl->cal_data.num_data,
			cdata.cfg.read_data.num_bytes);
		return -EINVAL;
	}
	if (!e_ctrl->cal_data.mapdata)
		return -EFAULT;

	ptr_dest = (uint8_t *) compat_ptr(cdata32->cfg.read_data.dbuffer);

	rc = copy_to_user(ptr_dest, e_ctrl->cal_data.mapdata,
		cdata.cfg.read_data.num_bytes);

	return rc;
}

static int msm_eeprom_config32(struct msm_eeprom_ctrl_t *e_ctrl,
	void __user *argp)
{
	struct msm_eeprom_cfg_data *cdata = (struct msm_eeprom_cfg_data *)argp;
	int rc = 0;

	CDBG("%s E\n", __func__);
	switch (cdata->cfgtype) {
	case CFG_EEPROM_GET_INFO:
		CDBG("%s E CFG_EEPROM_GET_INFO\n", __func__);
		cdata->is_supported = e_ctrl->is_supported;
		memcpy(cdata->cfg.eeprom_name,
			e_ctrl->eboard_info->eeprom_name,
			sizeof(cdata->cfg.eeprom_name));
		break;
	case CFG_EEPROM_GET_CAL_DATA:
		CDBG("%s E CFG_EEPROM_GET_CAL_DATA\n", __func__);
		cdata->cfg.get_data.num_bytes =
			e_ctrl->cal_data.num_data;
		break;
	case CFG_EEPROM_READ_CAL_DATA:
		CDBG("%s E CFG_EEPROM_READ_CAL_DATA\n", __func__);
		rc = eeprom_config_read_cal_data32(e_ctrl, argp);
		break;
	default:
		break;
	}

	CDBG("%s X rc: %d\n", __func__, rc);
	return rc;
}

static long msm_eeprom_subdev_ioctl32(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	struct msm_eeprom_ctrl_t *e_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;

	CDBG("%s E\n", __func__);
	CDBG("%s:%d a_ctrl %p argp %p\n", __func__, __LINE__, e_ctrl, argp);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_eeprom_get_subdev_id(e_ctrl, argp);
	case VIDIOC_MSM_EEPROM_CFG32:
		return msm_eeprom_config32(e_ctrl, argp);
	default:
		return -ENOIOCTLCMD;
	}

	CDBG("%s X\n", __func__);
}

static long msm_eeprom_subdev_do_ioctl32(
	struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);

	return msm_eeprom_subdev_ioctl32(sd, cmd, arg);
}

static long msm_eeprom_subdev_fops_ioctl32(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_eeprom_subdev_do_ioctl32);
}

#endif

static int msm_eeprom_platform_probe(struct platform_device *pdev)
{
	int rc = 0;
	int j = 0;
	uint32_t temp;

	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_eeprom_ctrl_t *e_ctrl = NULL;
	struct msm_eeprom_board_info *eb_info = NULL;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	CDBG("%s E\n", __func__);

	e_ctrl = kzalloc(sizeof(*e_ctrl), GFP_KERNEL);
	if (!e_ctrl) {
		pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
		return -ENOMEM;
	}
	e_ctrl->eeprom_v4l2_subdev_ops = &msm_eeprom_subdev_ops;
	e_ctrl->eeprom_mutex = &msm_eeprom_mutex;

	e_ctrl->is_supported = 0;
	if (!of_node) {
		pr_err("%s dev.of_node NULL\n", __func__);
		return -EINVAL;
	}

	rc = of_property_read_u32(of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}
	e_ctrl->subdev_id = pdev->id;

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&e_ctrl->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", e_ctrl->cci_master, rc);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		return rc;
	}
	rc = of_property_read_u32(of_node, "qcom,slave-addr",
		&temp);
	if (rc < 0) {
		pr_err("%s failed rc %d\n", __func__, rc);
		return rc;
	}

	rc = of_property_read_u32(of_node, "qcom,i2c-freq-mode",
		&e_ctrl->i2c_freq_mode);
	CDBG("qcom,i2c_freq_mode %d, rc %d\n", e_ctrl->i2c_freq_mode, rc);
	if (rc < 0) {
		pr_err("%s qcom,i2c-freq-mode read fail. Setting to 0 %d\n",
			__func__, rc);
		e_ctrl->i2c_freq_mode = 0;
	}

	if (e_ctrl->i2c_freq_mode >= I2C_MAX_MODES) {
		pr_err("%s:%d invalid i2c_freq_mode = %d\n", __func__, __LINE__,
			e_ctrl->i2c_freq_mode);
		return -EINVAL;
	}
	/* Set platform device handle */
	e_ctrl->pdev = pdev;
	/* Set device type as platform device */
	e_ctrl->eeprom_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	e_ctrl->i2c_client.i2c_func_tbl = &msm_eeprom_cci_func_tbl;
	e_ctrl->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!e_ctrl->i2c_client.cci_client) {
		pr_err("%s failed no memory\n", __func__);
		return -ENOMEM;
	}

	e_ctrl->eboard_info = kzalloc(sizeof(
		struct msm_eeprom_board_info), GFP_KERNEL);
	if (!e_ctrl->eboard_info) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto cciclient_free;
	}
	eb_info = e_ctrl->eboard_info;
	power_info = &eb_info->power_info;
	eb_info->i2c_slaveaddr = temp;

	power_info->clk_info = cam_8974_clk_info;
	power_info->clk_info_size = ARRAY_SIZE(cam_8974_clk_info);
	power_info->dev = &pdev->dev;


	rc = of_property_read_u32(of_node, "qcom,i2c-freq-mode",
		&eb_info->i2c_freq_mode);
	if (rc < 0 || (eb_info->i2c_freq_mode >= I2C_MAX_MODES)) {
		eb_info->i2c_freq_mode = I2C_STANDARD_MODE;
		CDBG("%s Default I2C standard speed mode.\n", __func__);
	}

	CDBG("qcom,slave-addr = 0x%X\n", eb_info->i2c_slaveaddr);
	cci_client = e_ctrl->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = e_ctrl->cci_master;
	cci_client->i2c_freq_mode = e_ctrl->i2c_freq_mode;
	cci_client->sid = eb_info->i2c_slaveaddr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = eb_info->i2c_freq_mode;

	rc = of_property_read_string(of_node, "qcom,eeprom-name",
		&eb_info->eeprom_name);
	CDBG("%s qcom,eeprom-name %s, rc %d\n", __func__,
		eb_info->eeprom_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto board_free;
	}

	rc = msm_eeprom_cmm_dts(e_ctrl->eboard_info, of_node);
	if (rc < 0)
		CDBG("%s MM data miss:%d\n", __func__, __LINE__);

	rc = msm_eeprom_get_dt_data(e_ctrl);
	if (rc)
		goto board_free;

	rc = msm_eeprom_parse_memory_map(of_node, &e_ctrl->cal_data);
	if (rc < 0)
		goto board_free;

	rc = msm_camera_power_up(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
	if (rc) {
		pr_err("failed rc %d\n", rc);
		goto memdata_free;
	}
	rc = read_eeprom_memory(e_ctrl, &e_ctrl->cal_data);
	if (rc < 0) {
		pr_err("%s read_eeprom_memory failed\n", __func__);
		goto power_down;
	}
	for (j = 0; j < e_ctrl->cal_data.num_data; j++)
		pr_debug("memory_data[%d] = 0x%X\n", j,
			e_ctrl->cal_data.mapdata[j]);

	rc = msm_set_otp_data(e_ctrl->cal_data.mapdata,
		e_ctrl->cal_data.num_data, e_ctrl->subdev_id,
			eb_info->eeprom_name);
	if (rc < 0)
		pr_err("%s set otp data failed %d\n", __func__, rc);

	e_ctrl->is_supported |= msm_eeprom_match_crc(&e_ctrl->cal_data);

	rc = msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
	if (rc) {
		pr_err("failed rc %d\n", rc);
		goto memdata_free;
	}
	v4l2_subdev_init(&e_ctrl->msm_sd.sd,
		e_ctrl->eeprom_v4l2_subdev_ops);
	v4l2_set_subdevdata(&e_ctrl->msm_sd.sd, e_ctrl);
	platform_set_drvdata(pdev, &e_ctrl->msm_sd.sd);
	e_ctrl->msm_sd.sd.internal_ops = &msm_eeprom_internal_ops;
	e_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(e_ctrl->msm_sd.sd.name,
		ARRAY_SIZE(e_ctrl->msm_sd.sd.name), "msm_eeprom");
	media_entity_init(&e_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	e_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	e_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_EEPROM;
	msm_sd_register(&e_ctrl->msm_sd);

#ifdef CONFIG_COMPAT
	msm_eeprom_v4l2_subdev_fops = v4l2_subdev_fops;
	msm_eeprom_v4l2_subdev_fops.compat_ioctl32 =
		msm_eeprom_subdev_fops_ioctl32;
	e_ctrl->msm_sd.sd.devnode->fops = &msm_eeprom_v4l2_subdev_fops;
#endif

	e_ctrl->is_supported = (e_ctrl->is_supported << 1) | 1;
	if (dirret == NULL) {
		dirret = debugfs_create_dir("msm_eeprom", NULL);
	}
	cam_sensor_name = debugfs_create_dir(eb_info->eeprom_name, dirret);
	debugfs_create_file("otp_data_dump", S_IFREG | S_IRUGO, cam_sensor_name, e_ctrl,
                    &eeprom_debugfs_operations);
	CDBG("%s X\n", __func__);
	return rc;

power_down:
	msm_camera_power_down(power_info, e_ctrl->eeprom_device_type,
		&e_ctrl->i2c_client);
memdata_free:
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
board_free:
	kfree(e_ctrl->eboard_info);
cciclient_free:
	kfree(e_ctrl->i2c_client.cci_client);
	kfree(e_ctrl);
	return rc;
}

static int msm_eeprom_platform_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct msm_eeprom_ctrl_t  *e_ctrl;
	if (!sd) {
		pr_err("%s: Subdevice is NULL\n", __func__);
		return 0;
	}

	e_ctrl = (struct msm_eeprom_ctrl_t *)v4l2_get_subdevdata(sd);
	if (!e_ctrl) {
		pr_err("%s: eeprom device is NULL\n", __func__);
		return 0;
	}

	kfree(e_ctrl->i2c_client.cci_client);
	kfree(e_ctrl->cal_data.mapdata);
	kfree(e_ctrl->cal_data.map);
	if (e_ctrl->eboard_info) {
		kfree(e_ctrl->eboard_info->power_info.gpio_conf);
		kfree(e_ctrl->eboard_info);
	}
	kfree(e_ctrl);
	return 0;
}

static const struct of_device_id msm_eeprom_dt_match[] = {
	{ .compatible = "qcom,eeprom" },
	{ }
};

MODULE_DEVICE_TABLE(of, msm_eeprom_dt_match);

static struct platform_driver msm_eeprom_platform_driver = {
	.driver = {
		.name = "qcom,eeprom",
		.owner = THIS_MODULE,
		.of_match_table = msm_eeprom_dt_match,
	},
	.probe = msm_eeprom_platform_probe,
	.remove = msm_eeprom_platform_remove,
};

static const struct i2c_device_id msm_eeprom_i2c_id[] = {
	{ "msm_eeprom", (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver msm_eeprom_i2c_driver = {
	.id_table = msm_eeprom_i2c_id,
	.probe  = msm_eeprom_i2c_probe,
	.remove = msm_eeprom_i2c_remove,
	.driver = {
		.name = "msm_eeprom",
	},
};

static struct spi_driver msm_eeprom_spi_driver = {
	.driver = {
		.name = "qcom_eeprom",
		.owner = THIS_MODULE,
		.of_match_table = msm_eeprom_dt_match,
	},
	.probe = msm_eeprom_spi_probe,
	.remove = msm_eeprom_spi_remove,
};

static int __init msm_eeprom_init_module(void)
{
	int rc = 0;
	CDBG("%s E\n", __func__);
	rc = platform_driver_register(&msm_eeprom_platform_driver);
	CDBG("%s:%d platform rc %d\n", __func__, __LINE__, rc);
	rc = spi_register_driver(&msm_eeprom_spi_driver);
	CDBG("%s:%d spi rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&msm_eeprom_i2c_driver);
}

static void __exit msm_eeprom_exit_module(void)
{
	platform_driver_unregister(&msm_eeprom_platform_driver);
	spi_unregister_driver(&msm_eeprom_spi_driver);
	i2c_del_driver(&msm_eeprom_i2c_driver);
}

module_init(msm_eeprom_init_module);
module_exit(msm_eeprom_exit_module);
MODULE_DESCRIPTION("MSM EEPROM driver");
MODULE_LICENSE("GPL v2");
