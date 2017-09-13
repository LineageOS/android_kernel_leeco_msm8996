#include <linux/module.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/random.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>

enum usb_mode {
	LETV_USB_UFP_MODE,
	LETV_USB_DFP_MODE,
	LETV_USB_DRP_MODE,
};

enum letv_vdm_type
{
	LETV_VDM_NONE,                    /*not letv vdm */
	LETV_VDM_RQ_RANDOM_DATA,          /*  request random data; */
	LETV_VDM_SD_RANDOM_DATA,          /* send random data from charger; */
	LETV_VDM_SD_ENCRY_DATA,           /* send encrypt data from charger; */
	LETV_VDM_CK_ENCRY_DATA_OK,        /* encrypt data ok; */
	LETV_VDM_CK_ENCRY_DATA_FAIL,      /* encrypt data fail; */
	LETV_VDM_RQ_ADJUST_VOL_CUR,       /* adjust voltage & current */
	LETV_VDM_ACK_ADJUST_VOL_CUR_OK,   /* ack adjust voltage & current ok */
	LETV_VDM_ACK_ADJUST_VOL_CUR_FAIL, /* ack adjust voltage & current fail */
	LETV_VDM_SD_CHARGER_PN,           /* send charger pn from charger;*/
};

enum pd_message {
	LETV_PD_MESSAGE_GOODCRC,
	LETV_PD_MESSAGE_GOTOMIN,
	LETV_PD_MESSAGE_ACCEPT,
	LETV_PD_MESSAGE_REJECT,
	LETV_PD_MESSAGE_PS_RDY,
	LETV_PD_MESSAGE_GET_SOURCE_CAP,
	LETV_PD_MESSAGE_GET_SINK_CAP,
	LETV_PD_MESSAGE_DR_SWAP,
	LETV_PD_MESSAGE_PR_SWAP,
	LETV_PD_MESSAGE_VCONN_SWAP,
	LETV_PD_MESSAGE_WAIT,
	LETV_PD_MESSAGE_SOURCE_CAP,
	LETV_PD_MESSAGE_SINK_CAP,
	LETV_PD_MESSAGE_VDM,
	LETV_PD_MESSAGE_SOFT_RESET,
	LETV_PD_MESSAGE_HARD_RESET,
	LETV_PD_MESSAGE_CABLE_RESET,
	LETV_PD_MESSAGE_REQUEST,
	LETV_PD_MESSAGE_END,    /* finish*/
	LETV_PD_MESSAGE_VDM_GET_CHARGER_PN,
};

enum {
	TIMER_NUMBER_UFP_TASK,
	TIMER_NUMBER_CHARGER_ENV_CHECK,
	TIMER_NUMBER_CHARGER_SETUP,
	TIMER_NUMBER_OFF_CHARGER,
	TIMER_NUMBER_END,
};

enum {
	TIMER_COMMAND_ENV_CHECK_ON,
	TIMER_COMMAND_ENV_CHECK_OFF,
	TIMER_COMMAND_CHG_SETUP,
	TIMER_COMMAND_OFF_CHARGER,
	TIMER_COMMAND_END,
};

enum {
	PD_CONTROL_STATUS_UNKOWN,
	PD_CONTROL_STATUS_SUSPEND,
	PD_CONTROL_STATUS_WORKING,
};

static char *pd_message_strings[] = {
	"LETV_PD_MESSAGE_GOODCRC",
	"LETV_PD_MESSAGE_GOTOMIN",
	"LETV_PD_MESSAGE_ACCEPT",
	"LETV_PD_MESSAGE_REJECT",
	"LETV_PD_MESSAGE_PS_RDY",
	"LETV_PD_MESSAGE_GET_SOURCE_CAP",
	"LETV_PD_MESSAGE_GET_SINK_CAP",
	"LETV_PD_MESSAGE_DR_SWAP",
	"LETV_PD_MESSAGE_PR_SWAP",
	"LETV_PD_MESSAGE_VCONN_SWAP",
	"LETV_PD_MESSAGE_WAIT",
	"LETV_PD_MESSAGE_SOURCE_CAP",
	"LETV_PD_MESSAGE_SINK_CAP",
	"LETV_PD_MESSAGE_VDM",
	"LETV_PD_MESSAGE_SOFT_RESET",
	"LETV_PD_MESSAGE_HARD_RESET",
	"LETV_PD_MESSAGE_CABLE_RESET",
	"LETV_PD_MESSAGE_REQUEST",
	"LETV_PD_MESSAGE_ERROR",           /*LETV_PD_MESSAGE_END*/
};

struct message_handle_struct{
	int is_timer_handle;
	int timer_handle_command;
	struct hpi_msg msg;
	enum pd_message message;
	struct list_head list;
};

struct source_cap_m {
	int supply_mode;  /*page: 153--fixed: 00; battery: 01; variable:10;*/
	int voltage;
	int mA;
};

#define LETV_TYPEC_TEMP_HIGH_LINE	70
#define LETV_TYPEC_TEMP_LOW_LINE	50
#define LETV_VDM_ENCRPT_DATA_LEN	16
#define LETV_PD_PDO_NUMBER	7

static struct source_cap_m source_cap_save[LETV_PD_PDO_NUMBER];
static struct task_struct  *letv_pd_thread;
static int letv_pd_control_status;
static enum usb_mode LETV_USB_MODE;
static int letv_pd_charger_setup_retry;
static int letv_pd_charger_not_start_vdm;
static int letv_pd_allow_adjust_power;
static int letv_pd_usb_off_charger_mode;
static bool letv_pd_charger_confirmed;

struct message_handle_struct *timer_msg_struct[TIMER_NUMBER_END];
extern int letv_pd_notice_charger_in_parameter(int voltage, int mA);
extern int letv_pd_notice_ver_just_vol(unsigned char *pd_ver, int size);

static LIST_HEAD(handle_list_head);
static DEFINE_SPINLOCK(letv_pd_lock);
static DECLARE_COMPLETION(letv_pd_completion);

static int letv_pd_send_message_soft_reset(void);
static void letv_pd_send_message_source_cap(void);
static void letv_pd_send_message_vdm_adjust_charger_parameter(int voltage, int mA);
static int letv_pd_message_request_get_valid_pdo(unsigned char *data, int *voltage, int *mA);
static int letv_pd_send_message_get_source_cap(void);
static int letv_pd_timer_handler(int command);

char *letv_pd_message_to_string(enum pd_message message)
{
	if (message >= LETV_PD_MESSAGE_END) {
		pr_err("%s: message error\n", __func__);
		message = LETV_PD_MESSAGE_END;
	}

	return pd_message_strings[message];
}

int letv_pd_get_register(int reg, int count, u8 *value)
{
	int ret;

	pr_err("%s:DEBUG: entry\n", __func__);
	if (!value) {
		pr_err("%s:ERROR: fail\n", __func__);
		return -1;
	}
	ret = cyccg_i2c_read(g_cyccg, value, count, reg);
	if (ret)
		return -1;

	return 0;
}

static int adjust_charger_voltage;
static int adjust_charger_mA;
static void letv_pd_init_adjust_charger_parameter(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	adjust_charger_voltage = 0;
	adjust_charger_mA = 0;
	return;
}

/*
when charger status change, need call this function to set charger parameter
voltage is mV;
*/
void letv_pd_set_adjust_charger_parameter(int voltage, int mA)
{
	int Vol_raw[31] = {3000, 3200, 3400, 3600, 3800,
						4000, 4200, 4400, 4600, 4800,
						5000, 5200, 5400, 5600, 5800,
						6000, 6200, 6400, 6600, 6800,
						7000, 7200, 7400, 7600, 7800,
						8000, 8200, 8400, 8600, 8800, 9000};
	unsigned char Vol_map[31] = {0x00, 0x08, 0x10, 0x18, 0x20,
								0x08, 0x30, 0x38, 0x40, 0x48,
								0x50, 0x58, 0x60, 0x68, 0x70,
								0x78, 0x80, 0x88, 0x90, 0x98,
								0xA0, 0xA8, 0xB0, 0xB8, 0xC0,
								0xC8, 0xD0, 0xD8, 0xE0, 0xE8, 0xF0};
	int mA_raw[16] = {0, 1200, 1400, 1600,
					1800, 2000, 2200, 2400,
					2600, 2800, 3000, 3200,
					3400, 3600, 3800, 4000};
	unsigned char mA_map[16] = {0x38, 0x4C, 0x54, 0x5C,
								0x64, 0x6C, 0x74, 0x7C,
								0x84, 0x8C, 0x94, 0x9C,
								0xA4, 0xAC, 0xB4, 0xBC};
	int i;
	int set_vol = 0;
	int set_mA = 0;

	pr_err("%s:DEBUG: entry, vol=%d, mA=%d\n", __func__, voltage, mA);
	if ((voltage < 3000) || (voltage > 9000) ||
		(mA < 0) || (mA > 4800))
		return;

	adjust_charger_voltage = voltage;
	adjust_charger_mA = mA;

	for (i = 0; i < 30; i++) {
		if (Vol_raw[i] == voltage) {
			set_vol = Vol_map[i];
		} else if (Vol_raw[i+1] == voltage) {
			set_vol = Vol_map[i+1];
		} else if ((Vol_raw[i] < voltage) && (voltage < Vol_raw[i+1])) {
			set_vol = Vol_map[i];
		}
	}

	for (i = 0; i < 14; i++) {
		if (mA_raw[i] == mA) {
			set_mA = mA_map[i];
		} else if (mA_raw[i+1] == mA) {
			set_mA = mA_map[i+1];
		} else if ((mA_raw[i] < mA) && (mA < mA_raw[i+1])){
			set_mA = mA_map[i];
		}
	}

	if ((set_vol == 0) || (set_mA == 0))
		set_vol = set_mA = 0;

	pr_err("%s:DEBUG:vol=%x, mA=%x\n", __func__, set_vol, set_mA);
	if (letv_pd_allow_adjust_power)
		letv_pd_send_message_vdm_adjust_charger_parameter(\
					set_vol, set_mA);

	return;
}
EXPORT_SYMBOL(letv_pd_set_adjust_charger_parameter);

int letv_pd_charger_in_voltage;
int letv_pd_charger_in_mA;
void letv_pd_set_charger_in_parameter(int voltage, int mA)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	letv_pd_charger_in_voltage = voltage * 1000;
	letv_pd_charger_in_mA = mA;
	return;
}

int letv_pd_charger_out_voltage;
int letv_pd_charger_out_mA;
void letv_pd_set_charger_out_parameter(int voltage, int mA)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	letv_pd_charger_out_voltage = voltage;
	letv_pd_charger_out_mA = mA;
	return;
}

int letv_pd_notice_charger_out_parameter(int voltage, int mA)
{
	pr_err("%s:DEBUG: entry: voltage=%d, mA=%d\n", __func__, voltage, mA);
	return 0;
}
EXPORT_SYMBOL(letv_pd_notice_charger_out_parameter);

static int typec_temp = 20;
int letv_pd_get_typec_temperature(void)
{
/*
Need add code for set typec_temp;
*/
	pr_err("%s:DEBUG: entry typec_temp=%d\n", __func__, typec_temp);
	return typec_temp;
}
EXPORT_SYMBOL(letv_pd_get_typec_temperature);

void letv_pd_set_typec_temperature(int temp)
{
	typec_temp = temp;
	return;
}
EXPORT_SYMBOL(letv_pd_set_typec_temperature);

/*
return value:
-1: error
0: suspend;
1: working;
*/
int letv_pd_get_cccontroller_work_status(void)
{
	pr_err("%s:ERROR: entry\n", __func__);
	if ((LETV_USB_MODE == LETV_USB_UFP_MODE)
		&& (letv_pd_control_status == PD_CONTROL_STATUS_SUSPEND))
		return 0;
	else if ((LETV_USB_MODE == LETV_USB_UFP_MODE)
		&& (letv_pd_control_status == PD_CONTROL_STATUS_WORKING))
		return 1;

	return -1;
}

/*
work:
1: start work
0: stop work
*/
int letv_pd_set_cccontroller_work_status(int work)
{
	pr_err("%s:ERROR: entry, work=%d\n", __func__, work);
	if (work) {
		cyccg_port_reset(g_cyccg);
		letv_pd_control_status = PD_CONTROL_STATUS_WORKING;
	} else {
		cyccg_port_disable(g_cyccg);
		letv_pd_control_status = PD_CONTROL_STATUS_SUSPEND;
	}
	return 1;
}

/************************timer start***************************/
static int letv_pd_timer_active(int timer_number, int command)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if ((timer_number >= TIMER_NUMBER_END)|| (command >= TIMER_COMMAND_END)) {
		pr_err("%s:DEBUG: number or command fail\n", __func__);
		return -1;
	}

	timer_msg_struct[timer_number]->timer_handle_command = command;
	spin_lock(&letv_pd_lock);
	list_add_tail(&timer_msg_struct[timer_number]->list, &handle_list_head);
	spin_unlock(&letv_pd_lock);
	complete(&letv_pd_completion);
	return 0;
}

static int timer_ufp_task_active;
static struct timer_list timer_ufp_task;
static void letv_pd_timer_ufp_task_function(unsigned long data)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	timer_ufp_task_active = 0;
	pr_err("%s:DEBUG: it's not PD charger\n", __func__);
	return;
}
static void letv_pd_del_timer_ufp_task_function(void)
{
	if (timer_ufp_task_active) {
		del_timer_sync(&timer_ufp_task);
		timer_ufp_task_active = 0;
	}
	return;
}

int letv_pd_timer_handle_charger_status(void)
{
	int ret;

	pr_err("%s:DEBUG: entry\n", __func__);
	ret = letv_pd_get_typec_temperature();
	if (ret >= LETV_TYPEC_TEMP_HIGH_LINE) {
		if (letv_pd_get_cccontroller_work_status() == 1) {
			letv_pd_timer_active(TIMER_NUMBER_CHARGER_ENV_CHECK, TIMER_COMMAND_ENV_CHECK_OFF);
		}
	} else if (ret < LETV_TYPEC_TEMP_LOW_LINE) {
		if (!letv_pd_get_cccontroller_work_status()) {
			letv_pd_timer_active(TIMER_NUMBER_CHARGER_ENV_CHECK, TIMER_COMMAND_ENV_CHECK_ON);
		}
	}

	return 0;
}

static int start_check_charger_env;
static int timer_charger_env_check_active;
static struct timer_list timer_charger_env_check;
static void letv_pd_timer_charger_env_check(unsigned long data)
{
	pr_err("%s:DEBUG: entry\n", __func__);

	timer_charger_env_check_active = 0;
	letv_pd_timer_handle_charger_status();
	if (start_check_charger_env) {
		timer_charger_env_check.expires = jiffies + HZ * 2;
		timer_charger_env_check_active = 1;
		add_timer(&timer_charger_env_check);
	}
	return;
}

static void letv_pd_del_timer_charger_env_check(void)
{
	start_check_charger_env = 0;
	if (timer_charger_env_check_active) {
		del_timer_sync(&timer_charger_env_check);
		timer_charger_env_check_active = 0;
	}

	return;
}

static int timer_letv_pd_charger_setup_active;
static struct timer_list timer_letv_pd_charger_setup;
static void letv_pd_timer_letv_pd_charger_setup(unsigned long data)
{
	pr_err("%s:ERROR: entry!!\n", __func__);
	timer_letv_pd_charger_setup_active = 0;

	letv_pd_charger_setup_retry++;
	letv_pd_charger_setup_retry = letv_pd_charger_setup_retry % 5;
	if (!letv_pd_charger_setup_retry) {
		pr_err("%s:ERROR: charger problem!!\n", __func__);
		letv_pd_charger_not_start_vdm = 1;
	}

	letv_pd_timer_active(TIMER_NUMBER_CHARGER_SETUP, TIMER_COMMAND_CHG_SETUP);
	return;
}

static void letv_pd_del_timer_letv_pd_charger_setup(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);

	if (timer_letv_pd_charger_setup_active) {
		del_timer_sync(&timer_letv_pd_charger_setup);
		timer_letv_pd_charger_setup_active = 0;
	}

	return;
}

static int timer_letv_pd_off_charger_active;
static struct timer_list timer_letv_pd_off_charger;
static void letv_pd_timer_letv_pd_off_charger(unsigned long data)
{
	pr_err("%s:ERROR: entry!!\n", __func__);
	timer_letv_pd_off_charger_active = 0;
	letv_pd_timer_active(TIMER_NUMBER_OFF_CHARGER, TIMER_COMMAND_OFF_CHARGER);
	return;
}

static void letv_pd_del_timer_letv_pd_off_charger(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);

	if (timer_letv_pd_off_charger_active) {
		del_timer_sync(&timer_letv_pd_off_charger);
		timer_letv_pd_off_charger_active = 0;
	}

	return;
}

/************************timer end***************************/
static void letv_pd_set_usb_mode(enum usb_mode mode)
{
	pr_err("%s:DEBUG: entry, mode=%d\n", __func__, mode);
	if (mode == LETV_USB_UFP_MODE) {
		LETV_USB_MODE = LETV_USB_UFP_MODE;
		letv_pd_charger_setup_retry = 0;
		letv_pd_charger_not_start_vdm = 0;
		letv_pd_allow_adjust_power = 0;
		letv_pd_charger_confirmed = false;
		letv_pd_del_timer_ufp_task_function();
		letv_pd_del_timer_charger_env_check();
		letv_pd_del_timer_letv_pd_charger_setup();
		letv_pd_init_adjust_charger_parameter();
		timer_ufp_task.expires = jiffies + (HZ/4);
		timer_ufp_task_active = 1;
		add_timer(&timer_ufp_task);
	} else if (mode == LETV_USB_DFP_MODE) {
		letv_pd_control_status = PD_CONTROL_STATUS_UNKOWN;
		LETV_USB_MODE = LETV_USB_DFP_MODE;
		letv_pd_usb_off_charger_mode = 0;
		letv_pd_charger_confirmed = false;
		letv_pd_del_timer_ufp_task_function();
		letv_pd_del_timer_charger_env_check();
		letv_pd_del_timer_letv_pd_charger_setup();
		letv_pd_send_message_source_cap();
	} else if (mode == LETV_USB_DRP_MODE) {
		letv_pd_control_status = PD_CONTROL_STATUS_UNKOWN;
		LETV_USB_MODE = LETV_USB_DRP_MODE;
		letv_pd_allow_adjust_power = 0;
		letv_pd_usb_off_charger_mode = 0;
		letv_pd_charger_confirmed = false;
		letv_pd_init_adjust_charger_parameter();
		letv_pd_del_timer_ufp_task_function();
		letv_pd_del_timer_charger_env_check();
		letv_pd_del_timer_letv_pd_charger_setup();
	}

	return;
}

static int letv_pd_get_usb_mode(void)
{
	return LETV_USB_MODE;
}

static unsigned char vdm_rand_data[LETV_VDM_ENCRPT_DATA_LEN];
static void letv_pd_set_random_data(unsigned char *rand_data, int size)
{
	int i;
	pr_err("%s:DEBUG: entry\n", __func__);
	if ((size != LETV_VDM_ENCRPT_DATA_LEN) || (rand_data == NULL))
		return;
	for (i = 0; i < size; i++)
		vdm_rand_data[i] = rand_data[i];
	return;
}

static int letv_pd_get_random_data(unsigned char *rand_data, int size)
{
	int i;
	pr_err("%s:DEBUG: entry\n", __func__);
	if ((size != LETV_VDM_ENCRPT_DATA_LEN) || (rand_data == NULL))
		return -1;
	for (i = 0; i < size; i++)
		rand_data[i] = vdm_rand_data[i];

	return 0;
}

#include "letv_pd_aes.c"
/*
return value:
0: OK,
else fail,
*/
static int letv_pd_encrypt_random_data(unsigned char *source_data, unsigned char *target_data, int size)
{
	int i;

	pr_err("%s:DEBUG: entry\n", __func__);
	if ((!source_data) || (!target_data) || (size != LETV_VDM_ENCRPT_DATA_LEN))
		return -1;

	letv_pd_aes_encrypt(source_data, size);
	for (i = 0; i < LETV_VDM_ENCRPT_DATA_LEN; i++)
		target_data[i] = source_data[i];

	return 0;
}

/*
return value:
0: OK,
else fail,
*/
static int letv_pd_check_encrypt_random_data(unsigned char *source_data, int size)
{
	int i;
	unsigned char vdm_random_data[LETV_VDM_ENCRPT_DATA_LEN];

	pr_err("%s:DEBUG: entry\n", __func__);
	if ((!source_data) || (size != LETV_VDM_ENCRPT_DATA_LEN))
		return -1;

	pr_err("%s:DEBUG:charger source_data=\n" \
		"0x%x,0x%x,0x%x,0x%x,\n" \
		"0x%x,0x%x,0x%x,0x%x,\n" \
		"0x%x,0x%x,0x%x,0x%x,\n" \
		"0x%x,0x%x,0x%x,0x%x,\n",
		__func__,
		source_data[0],source_data[1],source_data[2],source_data[3],
		source_data[4],source_data[5],source_data[6],source_data[7],
		source_data[8],source_data[9],source_data[10],source_data[11],
		source_data[12],source_data[13],source_data[14],source_data[15]);

	letv_pd_get_random_data(vdm_random_data, LETV_VDM_ENCRPT_DATA_LEN);
	letv_pd_aes_encrypt(vdm_random_data, LETV_VDM_ENCRPT_DATA_LEN);
	pr_err("%s:DEBUG:pd encrypt_data=\n" \
		"0x%x,0x%x,0x%x,0x%x,\n" \
		"0x%x,0x%x,0x%x,0x%x,\n" \
		"0x%x,0x%x,0x%x,0x%x,\n" \
		"0x%x,0x%x,0x%x,0x%x,\n",
		__func__,
		vdm_random_data[0],vdm_random_data[1],vdm_random_data[2],vdm_random_data[3],
		vdm_random_data[4],vdm_random_data[5],vdm_random_data[6],vdm_random_data[7],
		vdm_random_data[8],vdm_random_data[9],vdm_random_data[10],vdm_random_data[11],
		vdm_random_data[12],vdm_random_data[13],vdm_random_data[14],vdm_random_data[15]);

	for (i = 0; i < LETV_VDM_ENCRPT_DATA_LEN; i++) {
		if (vdm_random_data[i] != source_data[i]) {
			pr_err("%s:ERROR: vdm data is not equal to send\n", __func__);
			return -1;
		}
	}

	return 0;
}


int letv_pd_descrypt_random_data(unsigned char *source_data, unsigned char *target_data, int size)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if ((!source_data) || (!target_data) || (size != LETV_VDM_ENCRPT_DATA_LEN))
		return -1;

	pr_err("%s:DEBUG:encrypt_data=\n" \
		"0x%x,0x%x,0x%x,0x%x,\n" \
		"0x%x,0x%x,0x%x,0x%x,\n" \
		"0x%x,0x%x,0x%x,0x%x,\n" \
		"0x%x,0x%x,0x%x,0x%x,\n",
		__func__,
		source_data[0],source_data[1],source_data[2],source_data[3],
		source_data[4],source_data[5],source_data[6],source_data[7],
		source_data[8],source_data[9],source_data[10],source_data[11],
		source_data[12],source_data[13],source_data[14],source_data[15]);
//for test
	letv_pd_get_random_data(target_data, LETV_VDM_ENCRPT_DATA_LEN);
//for test end
	return 0;
}

static int letv_pd_get_current_rdo(int *voltage, int *mA)
{
	int ret;
	u8 value[4];
	pr_err("%s:DEBUG: entry\n", __func__);

	if ((!voltage) || (!mA)) {
		pr_err("%s:ERROR: parameter fail\n", __func__);
		return -1;
	}

	ret = letv_pd_get_register(CY_PD_REG_CURRENT_RDO, 4, value);
	if (ret) {
		pr_err("%s:ERROR: read CY_PD_REG_CURRENT_RDO fail\n", __func__);
		return -1;
	}
	ret = letv_pd_message_request_get_valid_pdo(value, voltage, mA);
	if (ret) {
		pr_err("%s:ERROR: get pdo fail\n", __func__);
		return -1;
	}

	return 0;
}

static int letv_pd_check_off_charger_status(void)
{
	int voltage, mA;
	pr_err("%s:DEBUG: entry\n", __func__);

	if (letv_pd_get_current_rdo(&voltage, &mA)) {
		pr_err("%s:ERROR: get rdo fail\n", __func__);
		return -1;
	}
	if (mA <= 0) {
		pr_err("%s:ERROR: get mA fail\n", __func__);
		return -1;
	}

	timer_letv_pd_off_charger.expires = jiffies + (HZ/5);
	timer_letv_pd_off_charger_active = 1;
	add_timer(&timer_letv_pd_off_charger);
	return 0;
}

static int letv_pd_send_message_soft_reset(void)
{
	struct hpi_msg resp_msg;
	unsigned char excepted_events[] = { CY_PD_RESP_SRC_CAP_RCVD,
					    CY_PD_RESP_ACCEPT_MESSAGE,
					    CY_PD_RESP_REJECT_MESSAGE,
					    CY_PD_SENDER_RESPONSE_TIMER_TIMEOUT,
					    0 };
	unsigned char data[32];
	int err;
	struct cyccg *cyccg = g_cyccg;

	pr_err("%s:DEBUG: entry\n", __func__);
	/* Send PD control retrieve source capability command. */
	data[0] = CY_PD_CONTROL_SEND_SOFT_RESET;
	err = cyccg_hpi_cmd_sync(cyccg, data, 1, CY_PD_REG_PD_CONTROL,
				 &resp_msg, excepted_events, 1000);
	if (err) {
		if (err != ETIMEDOUT) {
			pr_err("%s:ERROR: retrieve source capabilities, (%d)\n",
				__func__, err);
			return err;
		}

		err = cyccg_hpi_poll_events(cyccg,
					    &resp_msg, excepted_events);
		if (err) {
			pr_err("%s:ERROR: retrieve source capabilities, ETIMEDOUT\n",
				__func__);
			return err;
		}
	}

	if (resp_msg.head.code == CY_PD_RESP_SUCCESS) {
		pr_err("%s:DEBUG: Get_Source_Cap message is transmitted successfully\n", __func__);
		return 0;
	}

	pr_err("%s:ERROR:resp_msg.head.code=%d\n", __func__, resp_msg.head.code);
	return -1;
}

static void letv_pd_send_message_source_cap(void)
{
// it's work by firmware
	return;
}

static int letv_pd_send_message_get_source_cap(void)
{
	struct hpi_msg resp_msg;
	unsigned char excepted_events[] = { CY_PD_RESP_SRC_CAP_RCVD,
					    CY_PD_RESP_ACCEPT_MESSAGE,
					    CY_PD_RESP_REJECT_MESSAGE,
					    CY_PD_SENDER_RESPONSE_TIMER_TIMEOUT,
					    0 };
	unsigned char data[32];
	int err;
	struct cyccg *cyccg = g_cyccg;

	pr_err("%s:DEBUG: entry\n", __func__);

	/* Send PD control retrieve source capability command. */
	data[0] = CY_PD_CONTROL_RETRIEVE_SOURCE_CAP;
	err = cyccg_hpi_cmd_sync(cyccg, data, 1, CY_PD_REG_PD_CONTROL,
				 &resp_msg, excepted_events, 1000);
	if (err) {
		if (err != ETIMEDOUT) {
			pr_err("%s:ERROR: retrieve source capabilities, (%d)\n",
				__func__, err);
			return err;
		}

		err = cyccg_hpi_poll_events(cyccg,
					    &resp_msg, excepted_events);
		if (err) {
			pr_err("%s:ERROR: retrieve source capabilities, ETIMEDOUT\n",
				__func__);
			return err;
		}
	}

	if (resp_msg.head.code == CY_PD_RESP_SUCCESS) {
		pr_err("%s:DEBUG: Get_Source_Cap message is transmitted successfully\n", __func__);
		return 0;
	}

	pr_err("%s:ERROR:resp_msg.head.code=%d\n", __func__, resp_msg.head.code);
	return -1;
}

static int letv_pd_send_message_dr_swap(void)
{
	int ret;
	struct cyccg_ext_ap_cmd ext_cmd;

	pr_err("%s:DEBUG: entry\n", __func__);

	memset(&ext_cmd, 0, sizeof(ext_cmd));
	ext_cmd.cmd = CY_PD_CONTROL_DATA_ROLE_SWAP;
	ext_cmd.flags = 0;

	ret = mutex_lock_interruptible(&g_cyccg->lock);
	if (ret)
		return ret;
	atomic_inc(&g_cyccg->is_irq_owned_by_drv);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!g_cyccg->async_queue)
		enable_irq(g_cyccg->client->irq);
#endif
	ret = cyccg_swap_cmd_process(g_cyccg, &ext_cmd, NULL);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!g_cyccg->async_queue)
		disable_irq(g_cyccg->client->irq);
#endif
	atomic_dec(&g_cyccg->is_irq_owned_by_drv);
	mutex_unlock(&(g_cyccg->lock));

	return ret;

}

static void letv_pd_send_message_vdm_adjust_charger_parameter(int voltage, int mA)
{
	unsigned char vdm_charger_para[8] = {0x02,0x01,0x0e,0x2b,0x00,0x00,0x00,0x00};

	pr_err("%s:DEBUG: entry\n", __func__);
	/*current*/
	vdm_charger_para[4] = mA & 0xFF;
	vdm_charger_para[5] = (mA >> 8) & 0xFF;
	/*voltage*/
	vdm_charger_para[6] = voltage & 0xFF;
	vdm_charger_para[7] = (voltage >> 8) & 0xFF;
	pr_err("%s:DEBUG:charger_para voltage=0x%x%x,mA=0x%x%x,\n",
		__func__, vdm_charger_para[7],vdm_charger_para[6],vdm_charger_para[5],vdm_charger_para[4]);

	if ((letv_pd_get_usb_mode() == LETV_USB_UFP_MODE)
		|| letv_pd_usb_off_charger_mode)
		cyccg_send_vdm_data(g_cyccg, vdm_charger_para, sizeof(vdm_charger_para));
	return;
}

static void letv_pd_send_message_vdm_random_data(void)
{
	int ret;
	unsigned char vdm_random_data[20] = {0x40,0x04,0x0e,0x2b,
										0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00};

	pr_err("%s:DEBUG: entry\n", __func__);
	get_random_bytes(&vdm_random_data[4], LETV_VDM_ENCRPT_DATA_LEN);
	pr_err("%s:DEBUG:rand_data=\n" \
			"0x%x,0x%x,0x%x,0x%x,\n" \
			"0x%x,0x%x,0x%x,0x%x,\n" \
			"0x%x,0x%x,0x%x,0x%x,\n" \
			"0x%x,0x%x,0x%x,0x%x,\n",
			__func__,
			vdm_random_data[4],vdm_random_data[5],vdm_random_data[6],vdm_random_data[7],
			vdm_random_data[8],vdm_random_data[9],vdm_random_data[10],vdm_random_data[11],
			vdm_random_data[12],vdm_random_data[13],vdm_random_data[14],vdm_random_data[15],
			vdm_random_data[16],vdm_random_data[17],vdm_random_data[18],vdm_random_data[19]);

	ret = cyccg_send_vdm_data(g_cyccg, vdm_random_data, sizeof(vdm_random_data));
	if (!ret)
		letv_pd_set_random_data(&vdm_random_data[4], LETV_VDM_ENCRPT_DATA_LEN);

	return;
}

/*
sucess:
1: ack ok;
0:ack fail
*/
static void letv_pd_send_message_vdm_ack_data(int sucess)
{
	int ret;
	unsigned char vdm_ack_ok[4] = {0x41,0x00,0x0e,0x2b};
	unsigned char vdm_ack_fail[4] = {0x81,0x00,0x0e,0x2b};
//leijundebug
	return;
//
	pr_err("%s:DEBUG: entry\n", __func__);
	if (letv_pd_get_usb_mode() == LETV_USB_UFP_MODE) {
		if (sucess)
			ret = cyccg_send_vdm_data(g_cyccg, vdm_ack_ok, sizeof(vdm_ack_ok));
		else
			ret = cyccg_send_vdm_data(g_cyccg, vdm_ack_fail, sizeof(vdm_ack_fail));
	}
	return;
}

static int letv_pd_send_message_vdm_read_charger_pn(void)
{
	int ret;
	unsigned char vdm_read_pn[4] = {0x03,0x00,0x0e,0x2b};

	pr_err("%s:DEBUG: entry\n", __func__);
	ret = cyccg_send_vdm_data(g_cyccg, vdm_read_pn, sizeof(vdm_read_pn));
	if (ret)
		pr_err("%s:ERROR: send fail\n", __func__);

	return ret;
}

static int letv_pd_message_handle_goodcrc(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	return 0;
}

static int letv_pd_message_handle_gotomin(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	return 0;
}

static int letv_pd_message_handle_accept(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	return 0;
}

static int letv_pd_message_handle_reject(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	return 0;
}

/***********************ps ready handle start***********************/
static int letv_pd_message_request_get_valid_pdo(unsigned char *data, int *voltage, int *mA)
{
//usb pd pro: page 160
	int index;
	int supply_mode;

	pr_err("%s:DEBUG: entry\n", __func__);
	if (!data)
		return -1;

	index = (int)get_bits_value(data, 28, 3) - 1;
	if (index >= LETV_PD_PDO_NUMBER)
		return -1;

	supply_mode = source_cap_save[index].supply_mode;
	if ((supply_mode == 0) || (supply_mode == 2)) {  /*fixed mode*/ /*variable mode*/
		*mA = ((int)get_bits_value(data, 10, 10)) * 10;
	}else if (supply_mode == 1) { /*battery mode*/
		*mA = ((int)get_bits_value(data, 10, 10) * 250) / source_cap_save[index].voltage;
	}else {
		pr_err("%s:ERROR: get current RDO fail\n", __func__);
		return -1;
	}
	*voltage = source_cap_save[index].voltage;
	pr_err("%s:DEBUG: index=%d, vol=%d, mA=%d\n", __func__, index, *voltage, *mA);
	return 0;
}

static int letv_pd_message_handle_ps_rdy(struct hpi_msg *msg)
{
	int usb_mode;
	int ret;
	u8 value[4];
	int voltage, mA;

	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	if (letv_pd_usb_off_charger_mode) {
		if (letv_pd_get_current_rdo(&voltage, &mA)) {
			pr_err("%s:ERROR: get pdo fail\n", __func__);
			return -1;
		}

		letv_pd_set_charger_in_parameter(voltage, mA);
		letv_pd_notice_charger_in_parameter(letv_pd_charger_in_voltage, letv_pd_charger_in_mA);
		return 0;
	}

	usb_mode = letv_pd_get_usb_mode();
	switch(usb_mode) {
	case LETV_USB_UFP_MODE:
		if (letv_pd_get_current_rdo(&voltage, &mA)) {
			pr_err("%s:ERROR: get pdo fail\n", __func__);
			break;
		}

		letv_pd_set_charger_in_parameter(voltage, mA);
		letv_pd_notice_charger_in_parameter(letv_pd_charger_in_voltage, letv_pd_charger_in_mA);
		break;
	case LETV_USB_DFP_MODE:
		/*DFP mode charger, it's not used now*/
		if (0) {
			ret = letv_pd_get_register(CY_PD_REG_EFFECTIVE_SOURCE_PDO_MASK, 1, value);
			if (ret) {
				pr_err("%s:ERROR: read effective souce pdo mask fail\n", __func__);
				break;
			}

			switch (value[0]) {
			case 1:
				letv_pd_set_charger_out_parameter(5, 1500);
				break;
			case 2:
				letv_pd_set_charger_out_parameter(5, 3000);
				break;
			case 4:
				letv_pd_set_charger_out_parameter(9, 2700);
				break;
			default:
				letv_pd_set_charger_out_parameter(5, 1500);
				break;
			}
			letv_pd_notice_charger_out_parameter(letv_pd_charger_out_voltage, letv_pd_charger_out_mA);
		}
		break;
	default:
		break;
	}

	return 0;
}

/***********************ps ready handle end***********************/
static int letv_pd_message_handle_get_source_cap(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	return 0;
}

static int letv_pd_message_handle_get_sink_cap(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	return 0;
}
/***********************dr swap handle start***********************/
static int letv_pd_message_dr_swap_accept(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);

	if (!letv_pd_charger_confirmed)
		return -1;

	letv_pd_send_message_vdm_read_charger_pn();
	start_check_charger_env = 1;
	timer_charger_env_check.expires = jiffies + HZ * 2;
	timer_charger_env_check_active = 1;
	add_timer(&timer_charger_env_check);
	return 0;
}

static int letv_pd_message_dr_swap_reject(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_dr_swap_wait(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_dr_swap_noresponse(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_handle_dr_swap(struct hpi_msg *msg)
{
	unsigned char data;
	/*it's will handle by firmware, this is just for check swap sucess or fail, page 31*/
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	data = msg->data[0];
	if ((data & 0xF0) == 0) {
		/* DR SWAP accept */
		letv_pd_message_dr_swap_accept();
	} else if ((data & 0XF0) == 1) {
		/* DR SWAP reject */
		letv_pd_message_dr_swap_reject();
	} else if ((data & 0xF0) == 2) {
		/* DR SWAP wait */
		letv_pd_message_dr_swap_wait();
	} else if ((data & 0xF0) == 3) {
		/* DR SWAP not response */
		letv_pd_message_dr_swap_noresponse();
	} else {
		pr_err("%s:DEBUG: dr swap hard reset\n", __func__);
	}

	return 0;
}
/***********************dr swap handle end***********************/

/***********************pr swap handle start***********************/
static int letv_pd_message_pr_swap_accept(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_pr_swap_reject(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_pr_swap_wait(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_pr_swap_noresponse(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_handle_pr_swap(struct hpi_msg *msg)
{
	unsigned char data;

	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}
	data = msg->data[0];
	if ((data & 0xF0) == 0) {
		/* PR SWAP accept */
		letv_pd_message_pr_swap_accept();
	} else if ((data & 0XF0) == 1) {
		/* PR  SWAP reject */
		letv_pd_message_pr_swap_reject();
	} else if ((data & 0xF0) == 2) {
		/* PR  SWAP wait */
		letv_pd_message_pr_swap_wait();
	} else if ((data & 0xF0) == 3) {
		/* PR  SWAP not response */
		letv_pd_message_pr_swap_noresponse();
	} else {
		pr_err("%s:DEBUG: pr swap hard reset\n", __func__);
	}

	return 0;
}
/***********************pr swap handle end***********************/
/***********************vconn swap handle start***********************/
static int letv_pd_message_vconn_swap_accept(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_vconn_swap_reject(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_vconn_swap_wait(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_vconn_swap_noresponse(void)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	return 0;
}

static int letv_pd_message_handle_vconn_swap(struct hpi_msg *msg)
{
	unsigned char data;

	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	data = msg->data[0];
	if ((data & 0xF0) == 0) {
		/* PR SWAP accept */
		letv_pd_message_vconn_swap_accept();
	} else if ((data & 0XF0) == 1) {
		/* PR  SWAP reject */
		letv_pd_message_vconn_swap_reject();
	} else if ((data & 0xF0) == 2) {
		/* PR  SWAP wait */
		letv_pd_message_vconn_swap_wait();
	} else if ((data & 0xF0) == 3) {
		/* PR  SWAP not response */
		letv_pd_message_vconn_swap_noresponse();
	} else {
		pr_err("%s:DEBUG: pr swap hard reset\n", __func__);
	}

	return 0;
}

/***********************vconn swap handle end***********************/
static int letv_pd_message_handle_wait(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	return 0;
}
/***********************source capabilities handle start***********************/
/*
return value:
2: it is 9V
1: it is 5V
0: others
*/
static int letv_pd_message_source_cap_get_valid_pdo(unsigned char *pdo, int *voltage, int *mA, signed int *supply_mode)
{
	unsigned char *data = pdo;
	unsigned long val;
	unsigned long min_val;
	unsigned long default_val;
	unsigned long fval;

	val = get_bits_value(data, 30, 2);
	if (val == 0) {
		pr_err("%s:DEBUG:Power Capabilities: Fixed supply (Vmin = Vmax)\n", __func__);
		val = get_bits_value(data, 10, 10);
		fval = val * 50 / 1000;
		min_val = get_bits_value(data, 0, 10) * 10;
		pr_err("%s:DEBUG:Vmin = %ld V, Imin = %ld mA\n",
				__func__, fval, min_val);
		if ((int)fval == 9) {
			*voltage = (int)fval;
			*mA = (int)min_val;
			*supply_mode = 0;
			return 2;
		} else if ((int)fval == 5) {
			*voltage = (int)fval;
			*mA = (int)min_val;
			*supply_mode = 0;
			return 1;
		}
	} else if (val == 1) {
		pr_err("%s:DEBUG:Power Capabilities:Battery\n", __func__);
		val = get_bits_value(data, 20, 10);
		fval = val * 50 / 1000;
		min_val = get_bits_value(data, 10, 10) * 50;
		default_val = get_bits_value(data, 0, 10) * 250;
		pr_err("%s:DEBUG:Vmin = %ld V, Imin = %ld mA, Idef=%ld mA\n",
				__func__, fval, min_val, default_val);
		if ((int)fval == 9) {
			*voltage = (int)fval;
			*mA = (int)min_val;
			*supply_mode = 1;
			return 2;
		} else if ((int)fval == 5) {
			*voltage = (int)fval;
			*mA = (int)min_val;
			*supply_mode = 1;
			return 1;
		}
	} else if (val == 2) {
		pr_err("%s:DEBUG:Power Capabilities: Variable Supply (non-battery)\n", __func__);
		val = get_bits_value(data, 20, 10);
		fval = val * 50 / 1000;
		min_val = get_bits_value(data, 10, 10) * 50;
		default_val = get_bits_value(data, 0, 10) * 10;
		pr_err("%s:DEBUG:Vmin = %ld V, Imin = %ld mA, Idef=%ld mA\n",
				__func__, fval, min_val, default_val);
		if ((int)fval == 9) {
			*voltage = (int)fval;
			*mA = (int)min_val;
			*supply_mode = 2;
			return 2;
		} else if ((int)fval == 5) {
			*voltage = (int)fval;
			*mA = (int)min_val;
			*supply_mode = 2;
			return 1;
		}
	}

	return 0;
}

/*
return value:
0: sucess
< 0: fail
*/
static int letv_pd_message_set_source_cap_pdo(int pdo_index)
{
	struct hpi_msg resp_msg;
	unsigned char excepted_events[] = { CY_PD_RESP_ACCEPT_MESSAGE,
					    CY_PD_RESP_REJECT_MESSAGE,
					    CY_PD_RESP_WAIT_MESSAGE,
					    CY_PD_RESP_HARD_RESET,
					    CY_PD_SOURCE_DISBALED_STATE_ENTERED,
					    CY_PD_SENDER_RESPONSE_TIMER_TIMEOUT,
					    0 };
	unsigned char data[32];
	int err;
	int external_bit = -1;
	struct cyccg *cyccg = g_cyccg;
	/*This function not use now, so return -1*/
	return -1;

	/* Read current EFFECTIVE_SOURCE_PDO_MASK before update. */
	err = cyccg_i2c_read(cyccg, data, 1,
			     CY_PD_REG_EFFECTIVE_SOURCE_PDO_MASK);
	if (err) {
		pr_err("%s:ERROR: failed to read EFFECTIVE_SOURCE_PDO_MASK register, (%d)\n",
			__func__, err);
		return err;
	}
	pr_err("%s:DEBUG: current EFFECTIVE_SOURCE_PDO_MASK register: \n",
			__func__);

	/*
	 * Advertise the \'Externally Powered\' status bit
	 * if not force set in argument.
	 */
	if (external_bit < 0)
		external_bit = (data[0] & 0x80);

	/* Set SELECT_SOURCE_PDO register. */
	data[0] = (0x01 << pdo_index) | (external_bit ? 0x80 : 0x00) | 0x01;
	pr_err("%s:DEBUG: set SELECT_SOURCE_PDO_MASK register: 0x%x\n",
			__func__, data[0]);

	err = cyccg_hpi_cmd_sync(cyccg, data, 1,
				 CY_PD_REG_SELECT_SOURCE_PDO,
				 &resp_msg, excepted_events, 100);
	if (err) {
		if (err != -ETIMEDOUT) {
			pr_err("%s:ERROR: set SELECT_SOURCE_PDO register, (%d)\n",
				__func__, err);
			return err;
		}

		err = cyccg_hpi_poll_events(cyccg,
					    &resp_msg, excepted_events);
		if (err) {
			pr_err("%s:ERROR: poll the set SELECT_SOURCE_PDO register response, ETIMEDOUT\n",
				__func__);
			return err;
		}
	}

	if (resp_msg.head.code == CY_PD_RESP_SUCCESS) {
		pr_err("%s:DEBUG:update EFFECTIVE_SOURCE_PDO_MASK register with PDO MASK successfully.\n",
			__func__);
		return 0;
	}

	/*
	 * If re-negotiation process happened in CCG device, much more event
	 * will be sent to host, most of them are display through the
	 * cyccg_hpi_default_event_process() routine, except below events,
	 * much more detail description is output for these events when
	 * immediately received after CY_PD_RESP_SUCCESS.
	 */
	 pr_err("%s:ERROR:resp_msg.head.code=%d\n", __func__, resp_msg.head.code);
	return 0;
}

/*
return value:
0: sucess
< 0: fail
*/
static int letv_pd_message_handle_source_cap(struct hpi_msg *msg)
{
	int pdo_len;
	unsigned char *msg_head;
	int i, ret;
	int voltage, mA;
	int voltage_max, mA_max;
	int tmpW, powerW = 0;
	int index = -1;
	int index_tmp = -1;
	int usb_mode;
	signed int supply_mode = -1;

	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR msg is null\n", __func__);
		return -1;
	}

	letv_pd_del_timer_ufp_task_function();
	letv_pd_del_timer_letv_pd_charger_setup();
	letv_pd_del_timer_charger_env_check();
	letv_pd_init_adjust_charger_parameter();
	letv_pd_control_status = PD_CONTROL_STATUS_WORKING;
	memset(source_cap_save, 0, sizeof(source_cap_save));

	msg_head = msg->data;
	pdo_len = get_bits_value(msg_head, 12, 3);

	pr_err("%s: Parsed source PDOs data:\n", __func__);
	for (i = 4; i < msg->head.len; i+=4) {
		pr_err("%s:DEBUG:Source PDO%d: %02X %02X %02X %02X\n",
				__func__, (i/4) - 1,msg->data[i],msg->data[i + 1],
				msg->data[i + 2],msg->data[i + 3]);

		ret = letv_pd_message_source_cap_get_valid_pdo(&msg->data[i], &voltage, &mA, &supply_mode);
		if (ret <= 0)
			continue;

		tmpW = voltage * mA;
		if (powerW < tmpW) {
			powerW = tmpW;
			voltage_max = voltage;
			mA_max = mA;
			index = i / 4 - 1;
		}

		index_tmp = i / 4 - 1;
		pr_err("%s:DEBUG: index_tmp=%d,voltage=%d, mA=%d\n",__func__, index_tmp, voltage, mA);
		if ((index_tmp < LETV_PD_PDO_NUMBER) && (supply_mode >= 0)) {
			source_cap_save[index_tmp].supply_mode = supply_mode;
			source_cap_save[index_tmp].voltage = voltage;
			source_cap_save[index_tmp].mA = mA;
		}
	}
	pr_err("%s:DEBUG: will select index=%d\n",__func__, index);
	if (index < 0)
		return -1;
/*
The follow code is not used by cyccg, because firmware will select the max pdo.
*/
	ret = letv_pd_message_set_source_cap_pdo(index);
	if (!ret) {
		usb_mode = letv_pd_get_usb_mode();
		switch(usb_mode) {
		case LETV_USB_UFP_MODE:
			letv_pd_set_charger_in_parameter(voltage_max, mA_max);
			break;
		default:
			break;
		}
	}
/*
above
*/
	return ret;
}
/***********************source capabilities handle end***********************/

static int letv_pd_message_handle_sink_cap(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	return 0;
}

/***********************vdm handle start***********************/
static int letv_pd_message_vdm_check_vdm_type(unsigned char *buf, int size)
{
	if (( *buf == 0x00) && (*(buf+1) == 0x00)
		&& (*(buf+2) == 0x0e) && (*(buf+3) == 0x2b)) {
		return LETV_VDM_RQ_RANDOM_DATA;
	} else if (( *buf == 0x40) && (*(buf+1) == 0x04)
				&& (*(buf+2) == 0x0e) && (*(buf+3) == 0x2b)) {
		return LETV_VDM_SD_RANDOM_DATA;
	} else if (( *buf == 0x01) && (*(buf+1) == 0x04)
				&& (*(buf+2) == 0x0e) && (*(buf+3) == 0x2b)) {
		return LETV_VDM_SD_ENCRY_DATA;
	} else if (( *buf == 0x41) && (*(buf+1) == 0x00)
				&& (*(buf+2) == 0x0e) && (*(buf+3) == 0x2b)) {
		return LETV_VDM_CK_ENCRY_DATA_OK;
	} else if (( *buf == 0x81) && (*(buf+1) == 0x00)
				&& (*(buf+2) == 0x0e) && (*(buf+3) == 0x2b)) {
		return LETV_VDM_CK_ENCRY_DATA_FAIL;
	} else if (( *buf == 0x02) && (*(buf+1) == 0x01)
				&& (*(buf+2) == 0x0e) && (*(buf+3) == 0x2b)) {
		return LETV_VDM_RQ_ADJUST_VOL_CUR;
	} else if (( *buf == 0x42) && (*(buf+1) == 0x00)
				&& (*(buf+2) == 0x0e) && (*(buf+3) == 0x2b)) {
		return LETV_VDM_ACK_ADJUST_VOL_CUR_OK;
	} else if (( *buf == 0x82) && (*(buf+1) == 0x00)
				&& (*(buf+2) == 0x0e) && (*(buf+3) == 0x2b)) {
		return LETV_VDM_ACK_ADJUST_VOL_CUR_FAIL;
	} else if (( *buf == 0x43) && (*(buf+1) == 0x00)
				&& (*(buf+2) == 0x0e) && (*(buf+3) == 0x2b)) {
		return LETV_VDM_SD_CHARGER_PN;
	} else
		return LETV_VDM_NONE;
}

/*
return value:
0: OK
-1: fail;
*/
static int letv_pd_message_vdm_encrypt_data(unsigned char *random_data,
										unsigned char *encrpypt_data, int size)
{
	int ret;

	pr_err("%s:DEBUG: entry\n", __func__);
	if ((!random_data) || (!encrpypt_data) || (size != LETV_VDM_ENCRPT_DATA_LEN)) {
		pr_err("%s:ERROR: vdm data fail, size=%d\n", __func__, size);
		return -1;
	}

	ret = letv_pd_encrypt_random_data(random_data, encrpypt_data, size);
	return ret;
}

static int letv_pd_send_message_vdm_encrpt_data(unsigned char *random_data, int size)
{
	int ret;
	unsigned char vdm_encrypt_data[4 + LETV_VDM_ENCRPT_DATA_LEN] = {
							0x01,0x04,0x0e,0x2b,
							0x00,0x00,0x00,0x00,
							0x00,0x00,0x00,0x00,
							0x00,0x00,0x00,0x00,
							0x00,0x00,0x00,0x00};

	pr_err("%s:DEBUG: entry\n", __func__);
	if ((!random_data) || (size != LETV_VDM_ENCRPT_DATA_LEN)) {
		pr_err("%s:ERROR: random data fail, size=%d\n", __func__, size);
		return -1;
	}

	letv_pd_message_vdm_encrypt_data(random_data, &vdm_encrypt_data[4], size);
	if (letv_pd_get_usb_mode() == LETV_USB_UFP_MODE)
		ret = cyccg_send_vdm_data(g_cyccg, vdm_encrypt_data, sizeof(vdm_encrypt_data));
	return ret;
}

static int letv_pd_message_vdm_handle_charger_pn(unsigned char *vdm_data, int size)
{
	int i;
	unsigned char charger_pn[4];

	pr_err("%s:DEBUG: entry\n", __func__);
	if (size < 4) {
		pr_err("%s:ERROR: charger pn fail\n", __func__);
		return -1;
	}
	for (i = 0; i < 4; i++)
		charger_pn[i] = vdm_data[3-i];

	pr_err("%s:DEBUG: charger pn=0x%x, 0x%x, 0x%x, 0x%x\n",
			__func__,charger_pn[0],charger_pn[1],charger_pn[2],charger_pn[3]);
	letv_pd_notice_ver_just_vol(charger_pn, 4);
	return 0;
}

/*
return value:
0: OK
-1: fail;
*/
static int letv_pd_message_vdm_check_encrypt_data(unsigned char *vdm_data, int size)
{
	int ret;

	pr_err("%s:DEBUG: entry\n", __func__);
	if ((!vdm_data) || (size < LETV_VDM_ENCRPT_DATA_LEN)) {
		pr_err("%s:ERROR: vdm data fail, size=%d\n", __func__, size);
		return -1;
	}

	ret = letv_pd_check_encrypt_random_data(vdm_data, LETV_VDM_ENCRPT_DATA_LEN);
	if (ret)
		return -1;

	return 0;
}

/*
size: vdm_header + data
*/
static int letv_pd_message_vdm_resp_charger_data(unsigned char *vdm_buf, int size)
{
	int i;
	int ret;
	unsigned char vdm_header[4];

	pr_err("%s:DEBUG: entry\n", __func__);
	if (size < 4) {
		pr_err("%s:ERROR: vdm data fail\n", __func__);
		return -1;
	}
	for (i = 0; i < 4; i++)
		vdm_header[i] = *(vdm_buf + i);
	pr_err("%s:DEBUG: vdm_header = 0x%x%x%x%x\n", __func__,
		vdm_header[3],vdm_header[2],vdm_header[1],vdm_header[0]);

	ret = letv_pd_message_vdm_check_vdm_type(vdm_header, 4);
	switch (ret)
	{
	case LETV_VDM_RQ_RANDOM_DATA:
		letv_pd_del_timer_letv_pd_off_charger();
		timer_letv_pd_charger_setup.expires = jiffies + HZ * 5;
		timer_letv_pd_charger_setup_active = 1;
		add_timer(&timer_letv_pd_charger_setup);
		letv_pd_send_message_vdm_random_data();
		break;
	case LETV_VDM_SD_RANDOM_DATA:
		letv_pd_send_message_vdm_encrpt_data(&vdm_buf[4], size - 4);
		break;
	case LETV_VDM_SD_ENCRY_DATA:
		if (size < 20) {
			pr_err("%s:ERROR: get vdm encrypt data fail\n", __func__);
			return -1;
		}
		ret = letv_pd_message_vdm_check_encrypt_data(&vdm_buf[4], size - 4);
		if (!ret) {
			letv_pd_charger_confirmed = true;
			letv_pd_send_message_dr_swap();
			letv_pd_send_message_vdm_ack_data(1);
		} else {
			letv_pd_send_message_vdm_ack_data(0);
		}
		break;
	case LETV_VDM_CK_ENCRY_DATA_OK:
		break;
	case LETV_VDM_CK_ENCRY_DATA_FAIL:
		break;
	case LETV_VDM_RQ_ADJUST_VOL_CUR:
		break;
	case LETV_VDM_ACK_ADJUST_VOL_CUR_OK:
		break;
	case LETV_VDM_ACK_ADJUST_VOL_CUR_FAIL:
//		letv_pd_send_message_vdm_adjust_charger_parameter(adjust_charger_voltage, adjust_charger_mA);
		break;
	case LETV_VDM_SD_CHARGER_PN:
		if (size < 8) {
			pr_err("%s:ERROR: get charger pn fail\n", __func__);
			return -1;
		}
		letv_pd_charger_setup_retry = 0;
		letv_pd_charger_not_start_vdm = 0;
		letv_pd_del_timer_letv_pd_charger_setup();
		letv_pd_message_vdm_handle_charger_pn(&vdm_buf[4], size - 4);

		letv_pd_allow_adjust_power = 1;
		if ((adjust_charger_voltage > 0) && (adjust_charger_mA > 0))
			letv_pd_set_adjust_charger_parameter(\
				adjust_charger_voltage, adjust_charger_mA);
		break;
	default:
		break;
	}

	return ret;
/*VDM response*/
}

static int letv_pd_message_handle_vdm(struct hpi_msg *msg)
{
	int data_objects;
	int vdm_type;
	unsigned char *vdm_header;

	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	if (msg->head.len < 2) {
		pr_err("%s:ERROR: message head len short!\n", __func__);
		return -1;
	}

	if (letv_pd_charger_not_start_vdm) {
		pr_err("%s:ERROR: charger can not start vdm\n", __func__);
		return -1;
	}

	data_objects = (int)get_bits_value(&(msg->data[0]), 12, 3);
	if (msg->head.len < (data_objects * 4 + 4)) {
		pr_err("%s:ERROR: message head len short!!\n", __func__);
		return -1;
	}

	vdm_header = &msg->data[4];
	vdm_type = (int)get_bits_value(vdm_header, 15, 1);
	if (vdm_type) {
		/* Structured VDM data. */
		pr_err("%s:DEBUG: Vendor structured VDO Data:\n", __func__);
	} else {
		pr_err("%s:DEBUG:Vendor Unstructured VDO Data:data_objects=%d\n",
				__func__, data_objects * 4);
		letv_pd_message_vdm_resp_charger_data(vdm_header, data_objects * 4);
	}

	return 0;
}
/***********************vdm handle end***********************/

static int letv_pd_message_handle_soft_reset(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	letv_pd_set_charger_in_parameter(0, 0);
	letv_pd_set_charger_out_parameter(0, 0);
	letv_pd_del_timer_letv_pd_charger_setup();
	letv_pd_del_timer_charger_env_check();

	return 0;
}

static int letv_pd_message_handle_hard_reset(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	letv_pd_set_charger_in_parameter(0, 0);
	letv_pd_set_charger_out_parameter(0, 0);
	letv_pd_del_timer_letv_pd_off_charger();
	letv_pd_del_timer_ufp_task_function();
	letv_pd_del_timer_letv_pd_charger_setup();
	letv_pd_del_timer_charger_env_check();

	return 0;
}

static int letv_pd_message_handle_cable_reset(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	return 0;
}

static int letv_pd_message_handle_request(struct hpi_msg *msg)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if (!msg) {
		pr_err("%s:ERROR: msg is null\n", __func__);
		return -1;
	}

	return 0;
}

static int (*letv_pd_message_handler[])(struct hpi_msg *msg) =
{
	letv_pd_message_handle_goodcrc,             /* LETV_PD_MESSAGE_GOODCRC */
	letv_pd_message_handle_gotomin,             /* LETV_PD_MESSAGE_GOTOMIN */
	letv_pd_message_handle_accept,              /* LETV_PD_MESSAGE_ACCEPT */
	letv_pd_message_handle_reject,              /* LETV_PD_MESSAGE_REJECT */
	letv_pd_message_handle_ps_rdy,              /* LETV_PD_MESSAGE_PS_RDY */
	letv_pd_message_handle_get_source_cap,      /* LETV_PD_MESSAGE_GET_SOURCE_CAP */
	letv_pd_message_handle_get_sink_cap,        /* LETV_PD_MESSAGE_GET_SINK_CAP */
	letv_pd_message_handle_dr_swap,             /* LETV_PD_MESSAGE_DR_SWAP */
	letv_pd_message_handle_pr_swap,             /* LETV_PD_MESSAGE_PR_SWAP */
	letv_pd_message_handle_vconn_swap,          /* LETV_PD_MESSAGE_VCONN_SWAP */
	letv_pd_message_handle_wait,                /* LETV_PD_MESSAGE_WAIT */
	letv_pd_message_handle_source_cap,          /* LETV_PD_MESSAGE_SOURCE_CAP */
	letv_pd_message_handle_sink_cap,            /* LETV_PD_MESSAGE_SINK_CAP */
	letv_pd_message_handle_vdm,                 /* LETV_PD_MESSAGE_VDM */
	letv_pd_message_handle_soft_reset,          /* LETV_PD_MESSAGE_SOFT_RESET */
	letv_pd_message_handle_hard_reset,          /* LETV_PD_MESSAGE_HARD_RESET */
	letv_pd_message_handle_cable_reset,         /* LETV_PD_MESSAGE_CABLE_RESET */
	letv_pd_message_handle_request,				/* LETV_PD_MESSAGE_REQUEST */
};

int letv_pd_message_handle(enum pd_message message, struct hpi_msg *msg)
{
	int ret;
	struct message_handle_struct *msg_struct;

	if ((message >= LETV_PD_MESSAGE_END) || (!msg)) {
		pr_err("%s:ERROR:message error, finish handle\n", __func__);
		goto handle_error;
	}
	pr_err("%s:DEBUG:will handle message-%s\n", __func__, letv_pd_message_to_string(message));

	msg_struct = kzalloc(sizeof(struct message_handle_struct), GFP_KERNEL);
	if (!msg_struct) {
		pr_err("%s:ERROR: kzalloc fail\n", __func__);
		goto handle_error;
	}

	INIT_LIST_HEAD(&msg_struct->list);
	memcpy(&msg_struct->msg, msg, sizeof(struct hpi_msg));
	msg_struct->message = message;

	spin_lock(&letv_pd_lock);
	list_add_tail(&msg_struct->list, &handle_list_head);
	spin_unlock(&letv_pd_lock);

	complete(&letv_pd_completion);

	return ret;

handle_error:
	return -1;
}
EXPORT_SYMBOL(letv_pd_message_handle);

static int letv_pd_thread_handle(void *__unused)
{
	int ret;
	struct message_handle_struct *msg_struct;

	pr_err("%s:DEBUG: entry\n", __func__);
	while (1) {
		init_completion(&letv_pd_completion);
		wait_for_completion(&letv_pd_completion);

check_list:
		if (!list_empty(&handle_list_head)) {
			spin_lock(&letv_pd_lock);
			msg_struct = list_first_entry(&handle_list_head,
									struct message_handle_struct, list);
			if (!msg_struct) {
				pr_err("%s:ERROR: msg struct get fail\n", __func__);
				spin_unlock(&letv_pd_lock);
				continue;
			}
			list_del(&msg_struct->list);
			spin_unlock(&letv_pd_lock);

			if (msg_struct->is_timer_handle) {
				letv_pd_timer_handler(msg_struct->timer_handle_command);
			} else {
				if (msg_struct->message < LETV_PD_MESSAGE_END)
					ret = letv_pd_message_handler[msg_struct->message](&msg_struct->msg);

				kfree(msg_struct);
			}
			goto check_list;

		} else
			continue;
	}

	return ret;
}

static int letv_pd_thread_init(void)
{
	letv_pd_thread = kthread_run(letv_pd_thread_handle, NULL, "letv_pd");
	if (!letv_pd_thread)
		pr_err("%s:ERROR: letv create thread fail\n", __func__);

	return 0;
}

static int letv_pd_timer_handler(int command)
{
	pr_err("%s:DEBUG: entry\n", __func__);
	if ((command != TIMER_COMMAND_OFF_CHARGER)
		&& (letv_pd_get_usb_mode() != LETV_USB_UFP_MODE))
		return -1;

    switch(command){
    case TIMER_COMMAND_ENV_CHECK_ON:
		letv_pd_set_cccontroller_work_status(1);
        break;
	case TIMER_COMMAND_ENV_CHECK_OFF:
		letv_pd_set_cccontroller_work_status(0);
		break;
	case TIMER_COMMAND_CHG_SETUP:
		letv_pd_send_message_soft_reset();
		break;
	case TIMER_COMMAND_OFF_CHARGER:
		letv_pd_usb_off_charger_mode = 1;
		letv_pd_send_message_get_source_cap();
		break;
    default:
        break;
    }

    return 0;
}

static int letv_pd_timer_init(void)
{
	int i;

	timer_ufp_task_active = 0;
	init_timer(&timer_ufp_task);
	timer_ufp_task.function = letv_pd_timer_ufp_task_function;

	start_check_charger_env = 0;
	timer_charger_env_check_active= 0;
	init_timer(&timer_charger_env_check);
	timer_charger_env_check.function = letv_pd_timer_charger_env_check;

	timer_letv_pd_charger_setup_active= 0;
	init_timer(&timer_letv_pd_charger_setup);
	timer_letv_pd_charger_setup.function = letv_pd_timer_letv_pd_charger_setup;

	timer_letv_pd_off_charger_active = 0;
	init_timer(&timer_letv_pd_off_charger);
	timer_letv_pd_off_charger.function = letv_pd_timer_letv_pd_off_charger;

	for (i = 0; i < TIMER_NUMBER_END; i++) {
		timer_msg_struct[i] = kzalloc(sizeof(struct message_handle_struct), GFP_KERNEL);
		if (!timer_msg_struct[i]) {
			pr_err("%s:ERROR: kzalloc fail\n", __func__);
			return -1;
		}
		timer_msg_struct[i]->is_timer_handle = 1;
		INIT_LIST_HEAD(&timer_msg_struct[i]->list);
	}

	return 0;
}

static int letv_pd_init(void)
{
	LETV_USB_MODE = LETV_USB_DRP_MODE;
	letv_pd_charger_setup_retry = 0;
	letv_pd_charger_not_start_vdm = 0;
	letv_pd_allow_adjust_power = 0;
	letv_pd_charger_confirmed = false;
	memset(source_cap_save, 0, sizeof(source_cap_save));
	letv_pd_init_adjust_charger_parameter();
	letv_pd_timer_init();
	letv_pd_set_charger_in_parameter(0, 0);
	letv_pd_set_charger_out_parameter(0, 0);
	letv_pd_thread_init();
	letv_pd_check_off_charger_status();

	return 0;
}

