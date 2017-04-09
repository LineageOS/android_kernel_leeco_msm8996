#ifndef PUBLIC_INTERFACE_H
#define PUBLIC_INTERFACE_H
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include "anx_ohio_private_interface.h"
#include "anx_ohio_public_interface.h"

/**
 * @desc: The Interface that AP sends the specific USB PD command to Ohio
 *
 * @param:
 *	type: PD message type, define enum PD_MSG_TYPE.
 *	buf: the sepecific paramter pointer according to the message type
 *		eg: when AP update its source capability type=TYPE_PWR_SRC_CAP
 *		"buf" contains the content of PDO object,its format USB PD spec
 *		customer can easily packeted it through PDO_FIXED_XXX macro:
 *		default5Vsafe 5V,0.9A -> PDO_FIXED(5000,900, PDO_FIXED_FLAGS)
 *	size: the paramter content length. if buf is null, it should be 0
 *
 * @return:  0: success, Error: 1, reject 2: fail, 3: busy
 */
inline u8 send_pd_msg(PD_MSG_TYPE type, const char *buf, u8 size)
{
	u8 rst = 0;
	u8 wait_cmd_response_time = 0;

	switch (type) {
	case TYPE_PWR_SRC_CAP:
		rst = send_src_cap(buf, size);
		break;
	case TYPE_PWR_SNK_CAP:
		rst = send_snk_cap(buf, size);
		break;
	case TYPE_DP_SNK_IDENDTITY:
		rst = interface_send_msg_timeout(TYPE_DP_SNK_IDENDTITY,
					(u8 *)buf, size, INTERFACE_TIMEOUT);
		break;
	case TYPE_SVID:
		rst = send_svid(buf, size);
		break;
	case TYPE_GET_DP_SNK_CAP:
		rst = interface_send_msg_timeout(TYPE_GET_DP_SNK_CAP, NULL, 0,
					       INTERFACE_TIMEOUT);
		break;
	case TYPE_PSWAP_REQ:
		rst = send_power_swap();
		wait_cmd_response_time = 200;
		break;
	case TYPE_DSWAP_REQ:
		rst = send_data_swap();
		wait_cmd_response_time = 200;
		break;
	case TYPE_GOTO_MIN_REQ:
		rst = interface_send_gotomin();
		break;
	case TYPE_VDM:
		rst = send_vdm(buf, size);
		break;
	case TYPE_DP_SNK_CFG:
		rst = send_dp_snk_cfg(buf, size);
		break;
	case TYPE_PD_STATUS_REQ:
		rst = interface_get_pd_status();
		wait_cmd_response_time = 200;
		break;
	case TYPE_PWR_OBJ_REQ:
		rst = send_rdo(buf, size);
		break;
	case TYPE_ACCEPT:
		rst = interface_send_accept();
		break;
	case TYPE_REJECT:
		rst = interface_send_reject();
		break;
	case TYPE_SOFT_RST:
		rst = interface_send_soft_rst();
		wait_cmd_response_time = 0;
		break;
	case TYPE_HARD_RST:
		rst = interface_send_hard_rst();
		break;
	default:
		pr_info("unknown type %x\n", type);
		rst = 0;
		break;
	}
	if (rst == CMD_FAIL) {
		pr_err("Cmd %x Fail.\n", type);
		return CMD_FAIL;
	}
	/* need wait command's response */
	if (wait_cmd_response_time)
		rst = wait_pd_cmd_timeout((u8) type, wait_cmd_response_time);
	else
		rst = CMD_SUCCESS;

	return rst;
}

extern int get_charger_charging_current(void);
extern int get_battery_voltage(void);
extern int get_charging_status(void);
extern int get_typec_temperature(void);

/*6000mA = 0xFF, 25mA delta equal to 0x1 */
static u8 report_charging_current(void)
{
	u8 charging_current;
	int tmp;

	tmp = -get_charger_charging_current();
	charging_current = 0xFF - (6000 - tmp) / 25;
	printk("charging current=%dmA = 0x%x; ",tmp, charging_current);
	return charging_current;
}

/* 4.7V=0xFF; 0.01V delta equal to 0x1*/
static u8 report_battery_voltage(void)
{
	u8 battery_voltage;
	int tmp;

	tmp = get_battery_voltage();
	battery_voltage = 0xFF - (4700 - tmp) / 10;
	printk("battery voltage=%dmV = 0x%x; ", tmp, battery_voltage);
	return battery_voltage;
}

/*0x00 stop charging;0x01 CC charging;0x02 CV charging;0x03 Battery Full;0x04 resume charging*/
static u8 report_charging_status(void)
{
	int tmp;
	u8 charging_status = 0;

	tmp = get_charging_status();
	switch (tmp)
	{
	case 0:		/* POWER_SUPPLY_CHARGE_TYPE_UNKNOWN */
	case 1:		/* POWER_SUPPLY_CHARGE_TYPE_NONE */
	case 2:		/* POWER_SUPPLY_CHARGE_TYPE_TRICKLE */
		charging_status = 0x00;
		break;
	case 3:		/* POWER_SUPPLY_CHARGE_TYPE_FAST */
		charging_status = 0x01;
		break;
	case 4:		/* POWER_SUPPLY_CHARGE_TYPE_TAPER */
		charging_status = 0x02;
	case 5:		/* POWER_SUPPLY_STATUS_FULL */
		charging_status = 0x03;
		break;
	}
	printk("0x%x; ", charging_status);
	return charging_status;
}

/*25degree equal to 0x80,  0.5degree delta equal to 0x1*/
static u8 report_typec_temperature(void)
{
	int tmp;
	u8 temp;
	return 0x70;

	tmp = get_typec_temperature();
	if (tmp >= 25) {
		temp = 0x80 + (tmp - 25) * 2;
	} else {
		temp = 0x80 - (25 - tmp) * 2;
	}
	printk("temperature=%d=0x%x;", tmp, temp);
	return temp;
}

/**
 * @desc:   The Interface that AP handle the specific USB PD command from Ohio
 *
 * @param:
 *	type: PD message type, define enum PD_MSG_TYPE.
 *	para: the sepecific paramter pointer
 *	para_len: the paramter ponter's content length
 *		if buf is null, it should be 0
 *
 * @return:  0: success 1: fail
 *
 */
inline u8 dispatch_rcvd_pd_msg(PD_MSG_TYPE type, void *para, u8 para_len)
{
	u8 rst = 0;
	int i;
	u8 vdm_vdo[8] = {0};
	u8 *tmp = (u8*)para;

	pd_callback_t fnc = get_pd_callback_fnc(type);
	if (fnc != 0) {
		rst = (*fnc) (para, para_len);
		return rst;
	}
	printk("ohio %s: type=%s, para_len=%d, ", __func__, interface_to_str(type), para_len);
	for (i = 0; i < para_len; i++)
		printk("len%d = 0x%x, ", i, *(tmp + i));
	printk("\n");

	switch (type) {
	case TYPE_PWR_SRC_CAP:
		/* execute the receved source capability's  handle function */
		rst = recv_pd_source_caps_default_callback(para, para_len);
		break;
	case TYPE_PWR_SNK_CAP:
		/* received peer's sink caps */
		rst = recv_pd_sink_caps_default_callback(para, para_len);
		break;
	case TYPE_PWR_OBJ_REQ:
		/* evaluate RDO and give accpet or reject */
		rst = recv_pd_pwr_object_req_default_callback(para, para_len);
		break;
	case TYPE_DSWAP_REQ:
		/* execute the receved handle function */
		rst = recv_pd_dswap_default_callback(para, para_len);
		break;
	case TYPE_PSWAP_REQ:
		/* execute the receved handle function */
		rst = recv_pd_pswap_default_callback(para, para_len);
		break;
	case TYPE_ACCEPT:
		rst = recv_pd_accept_default_callback(para, para_len);
		break;
	case TYPE_RESPONSE_TO_REQ:
		/* execute the receved handle function */
		rst = recv_pd_cmd_rsp_default_callback(para, para_len);
		break;
	case TYPE_DP_ALT_ENTER:
		break;
	case TYPE_DP_ALT_EXIT:
		break;
	case TYPE_HARD_RST:
		rst = recv_pd_hard_rst_default_callback(para, para_len);
		break;
#ifdef SUPP_VDM_CHARGING
	case TYPE_VDM:
		if (para_len < 2) {
			printk("%s:TYPE_VDM error para_len\n", __func__);
			break;
		}
		if((tmp[0] == 0x00) &&
		   (tmp[1] == 0x01)) {
			tmp[0] = 0x40;
			rst = recv_pd_vdm_default_callback(para, para_len);
		} else if ((tmp[0] == 0x01) && (tmp[1] == 0x01)) {
			vdm_vdo[0] = 0x41;
			vdm_vdo[1] = 0x1;
			vdm_vdo[2] = 0x0;
			vdm_vdo[3] = 0x0;

			printk("ohio. : ");
			vdm_vdo[4] = report_charging_current();
			vdm_vdo[5] = report_battery_voltage();
			vdm_vdo[6] = report_charging_status();
			vdm_vdo[7] = report_typec_temperature();
			printk("\n");
			rst = recv_pd_vdm_default_callback(vdm_vdo, 8);
		}
		break;
#endif
	default:
		rst = 0;
		break;
	}
	return rst;
}
/**
 * @desc:  The Interface helps customer to register one's
 *	interesting callback function of the specific
 *	USB PD message type, when the REGISTERED message
 *	arrive, the customer's callback function will be executed.
 * !!!! Becarefully:
 *  Because the USB PD TIMING limatation, the callback function
 *  should be designed to follow USB PD timing requiment.
 *
 * @param:
 *	type: PD message type, define enum PD_MSG_TYPE.
 *	func: callback function pointer
 *		it's sepecific definaction is:u8 (*)(void *, u8)
 *
 * @return:  1: success 0: fail
 *
 */
u8 register_pd_msg_callback_func(PD_MSG_TYPE type, pd_callback_t fnc)
{
	if (type > 256)
		return 0;
	set_pd_callback_fnc(type, fnc);

	return 1;
}

#endif
