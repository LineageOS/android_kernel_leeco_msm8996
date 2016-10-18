/* add by LETV */
int qpnp_pon_spare_reg_masked_read(u8 off, u8 mask)
{
	struct qpnp_pon *pon = sys_reset_dev;
	int rc = 0;
	u8 reg;

	if (!pon)
		return -ENODEV;

	/* only 4 regs available */
	if ((off > 0x8F) || (off < 0x8C))
		return -EINVAL;

	rc = spmi_ext_register_readl(pon->spmi->ctrl, pon->spmi->sid,
			(pon->base + off), &reg, 1);
	if (rc) {
		dev_err(&pon->spmi->dev,
			"Unable to read addr=%x, rc(%d)\n",
			(pon->base + off), rc);
		return rc;
	}

	dev_info(&pon->spmi->dev, "read addr=%x, reg=0x%x)\n", (pon->base + off), reg);
	reg &= mask;

	return reg;
}

int qpnp_pon_spare_reg_masked_write(u8 off, u8 mask, u8 reg)
{
	struct qpnp_pon *pon = sys_reset_dev;

	if (!pon)
		return -ENODEV;

	/* only 4 regs available */
	if ((off > 0x8F) || (off < 0x8C))
		return -EINVAL;

	return qpnp_pon_masked_write(pon, (pon->base + off), mask, reg);
}

static const char * const qpnp_warm_reset_reason[] = {
	[0] = "Triggered from Soft Reset",
	[1] = "Triggered from PS_HOLD",
	[2] = "Triggered from PMIC_WD",
	[3] = "Triggered from GP1 (Keypad_Reset1)",
	[4] = "Triggered from GP2 (Keypad Reset2)",
	[5] = "Triggered from KPDPWR_AND_RESIN",
	[6] = "Triggered from RESIN",
	[7] = "Triggered from KPDPWR",
	[8] = "N/A",
	[9] = "N/A",
	[10] = "N/A",
	[11] = "N/A",
	[12] = "AFP",
	[13] = "N/A",
	[14] = "N/A",
	[15] = "N/A",
};

static inline const char * index_to_str(const char* const array[], u32 arr_size, u32 index)
{
	return (index > arr_size) ? NULL : array[index];
}

const char *const pon_to_str(u8 pon)
{
	return index_to_str(qpnp_pon_reason, ARRAY_SIZE(qpnp_pon_reason), ffs(pon) - 1);
}
const char * const poff_to_str(u16 poff)
{
	return index_to_str(qpnp_poff_reason, ARRAY_SIZE(qpnp_poff_reason), ffs(poff) - 1);
}

const char * const wam_reset_to_str(u16 rst_rsn)
{
	return index_to_str(qpnp_warm_reset_reason, ARRAY_SIZE(qpnp_warm_reset_reason), ffs(rst_rsn) - 1);
}

void set_sahara_mode(int on)
{
	u8 reg = on ? BIT(1) : 0;
	qpnp_pon_spare_reg_masked_write(0x8D, BIT(1), reg);
}
