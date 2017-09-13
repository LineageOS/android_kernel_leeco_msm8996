/*

 * Cypress USB Type-C and PD Controller device driver
 *
 * Copyright (c) 2015 Cypress Semiconductor Corp.
 *
 * Author: Dudley Du <dudl@cypress.com>
 * Version: 1.0.0
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/list.h>
#include <linux/major.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <asm/unaligned.h>
#include <asm/ioctl.h>
#include <linux/compat.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/version.h>
#include <linux/firmware.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#define CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR 0
#define CYCCG_ENABLE_IRQ_READY_POLLING	1
#define CYCCG_CCG_SUPPORT_SWAP_COMPLETE 1
#define CYCCG_POWER_CONTROLLED_BY_EXTERNAL_AP 1
#define CYCCG_ENABLE_RESUME_IRQ_WORK_THREAD 0

#define CYCCG_AUTO_INTERNAL_UPDATE 1
#if CYCCG_AUTO_INTERNAL_UPDATE
  #define CYCCG_FW_NAME		"cyccg.cybin"
  #define CYCCG_TABLE_NAME	"cyccg_table.cybin"
  #define CYCCG_ENABLE_FW_IMAGE_APP_NAME_CHECK 0
  #define CYCCG_ENABLE_STATIC_FW_VER 1
  #if CYCCG_ENABLE_STATIC_FW_VER
    #define CYCCG_LATEST_FW_MAJ_VER 2
    #define CYCCG_LATEST_FW_MIN_VER 9
  #endif
#endif

#define CYCCG_DEBUG 1
#define CYCCG_DEBUG_FW 0
#define CYCCG_DEBUG_IRQ 1
#define LOG_TAG "cyccg"
#if CYCCG_DEBUG
#define cyccg_dbg(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#else
#define cyccg_dbg(fmt, ...)
#endif
#if CYCCG_DEBUG_FW
#define cyccg_dbg_fw(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#else
#define cyccg_dbg_fw(fmt, ...)
#endif
#if CYCCG_DEBUG_IRQ
#define cyccg_dbg_irq(fmt, ...) pr_err(fmt, ##__VA_ARGS__)
#else
#define cyccg_dbg_irq(fmt, ...)
#endif

#define CYCCG_SILICON_ID 0x1406
#define CYCCG_DEV_NAME "cyccg"
#define CYCCG_I2C_REGISTER_MAP_SIZE 256
#define CYCCG_MAX_NAME_SIZE 32

#define CYCCG_I2C_READ		_IOWR('c', 0x01, int)
#define CYCCG_I2C_WRITE		_IOWR('c', 0x02, int)
#define CYCCG_GET_I2C_INFO	_IOWR('c', 0x03, int)
#define CYCCG_CLEAR_IRNT	_IOWR('c', 0x04, int)
/*
 * Used to communicate with external AP/modules
 * that co-working with the CCG2 device.
 */
#define CYCCG_EXT_AP_CTRL	_IOWR('C', 0x05, int)
struct cyccg_ext_ap_cmd {
	u32 cmd;  /* PD_CONTROL command defined in cy_pd_hpi_control_t */
	/*
	 * bit0 : 0-Command CCGx, then external AP to the operation.
	 *        1-Ignre CCGx, only command exteranl AP do the operation.
	 * bit1 - bit31 : Resereved, should be 0.
	 */
	u32 flags;
	u32 arg_size;  /* Size in bytes of the arg data. */
	u8 __user *arg;
} __attribute((packed));

/* CCG PD Controller HPI Register Address. */
enum cy_pd_hpi_reg_t {
	CY_PD_REG_DEVICE_MODE_ADDR,
	CY_PD_BOOT_MODE_REASON,
	CY_PD_SILICON_ID,
	CY_PD_BOOL_LOADER_LAST_ROW = 0x04,
	CY_PD_REG_INTR_REG_ADDR = 0x06,
	CY_PD_JUMP_TO_BOOT_REG_ADDR,
	CY_PD_REG_RESET_ADDR,
	CY_PD_REG_ENTER_FLASH_MODE_ADDR = 0x0a,
	CY_PD_REG_VALIDATE_FW_ADDR,
	CY_PD_REG_FLASH_READ_WRITE_ADDR,
	CY_PD_GET_VERSION = 0x10,
	CY_PD_REG_U_VDM_CTRL_ADDR = 0x20,
	CY_PD_REG_READ_PD_PROFILE = 0x22,
	CY_PD_REG_EFFECTIVE_SOURCE_PDO_MASK = 0x24,
	CY_PD_REG_EFFECTIVE_SINK_PDO_MASK,
	CY_PD_REG_SELECT_SOURCE_PDO,
	CY_PD_REG_SELECT_SINK_PDO,
	CY_PD_REG_PD_CONTROL,
	CY_PD_REG_PD_STATUS = 0x2c,
	CY_PD_REG_TYPE_C_STATUS = 0x30,
	CY_PD_REG_CURRENT_PDO = 0x34,
	CY_PD_REG_CURRENT_RDO = 0x38,
	CY_PD_REG_CURRENT_CABLE_VDO = 0x3c,
	CY_PD_REG_DISPLAY_PORT_STATUS = 0x40,
	CY_PD_REG_DISPLAY_PORT_CONFIG = 0x44,
	CY_PD_REG_ALTERNATE_MODE_MUX_SELECTION = 0x45,
	CY_PD_REG_EVENT_MASK = 0x48,
	CY_PD_REG_SWAP_RESPONSE = 0x4c,
	CY_PD_REG_RESPONSE_ADDR = 0x7e,
	CY_PD_REG_BOOTDATA_MEMEORY_ADDR = 0x80,
	CY_PD_REG_FWDATA_MEMEORY_ADDR = 0xc0,
};

enum cy_pd_hpi_control_t {
	CY_PD_CONTROL_TYPEC_DEFAULT,
	CY_PD_CONTROL_TYPEC_1_5A,
	CY_PD_CONTROL_TYPEC_3A,
	CY_PD_CONTROL_ENTER_USB_SUSPEND,
	CY_PD_CONTROL_EXIT_USB_SUSPEND,
	CY_PD_CONTROL_DATA_ROLE_SWAP,
	CY_PD_CONTROL_POWER_ROLE_SWAP,
	CY_PD_CONTROL_VCON_ON,
	CY_PD_CONTROL_VCON_OFF,
	CY_PD_CONTROL_VCON_ROLE_SWAP,
	CY_PD_CONTROL_RETRIEVE_SOURCE_CAP,
	CY_PD_CONTROL_RETRIEVE_SINK_CAP,
	CY_PD_CONTROL_GOTO_MIN,
	CY_PD_CONTROL_SEND_HARD_RESET,
	CY_PD_CONTROL_SEND_SOFT_RESET,
	CY_PD_CONTROL_SEND_CABLE_RESET,
	CY_PD_CONTROL_INIT_COMPLETE = 0x10,
	CY_PD_EC_DISABLE,
};

enum cy_pd_hpi_resp_t {
	/* Responses, 0x00-0x7F */
	CY_PD_RESP_NO_RESPONSE,
	CY_PD_RESP_SUCCESS = 0x02,
	CY_PD_RESP_FLASH_DATA_AVAILABLE,
	CY_PD_RESP_INVALID_COMMAND = 0x05,
	CY_PD_RESP_COLLISION_DETECTED,   /* Reserved in HPI spec V11. */
	CY_PD_RESP_FLASH_UPDATE_FAILED,
	CY_PD_RESP_INVALID_FW,
	CY_PD_RESP_INVALID_ARGUMENTS,
	CY_PD_RESP_NOT_SUPPORTED,
	CY_PD_RESP_TRANSACTION_FAILED = 0x0C,
	CY_PD_RESP_PD_COMMAND_FAILED,
	CY_PD_RESP_UNDEFINED,

	/* Device Specific Events, 0x80-0x81 */
	CY_PD_RESP_RESET_COMPLETE = 0x80,
	CY_PD_RESP_MESSAGE_QUEUE_OVERFLOW,

	/* Type C specific events, 0x82-0x85 */
	CY_PD_RESP_OVER_CURRENT_DETECTED,
	CY_PD_RESP_OVER_VOLTAGE_DETECTED,
	CY_PD_RESP_TYPE_C_CONNECTED,
	CY_PD_RESP_TYPE_C_DISCONNECTED,

	/* PD Specific events and asynchronous messages, 0x86-0x8F */
	CY_PD_RESP_PD_CONTRACT_ESTABLISHED,
	/* SWAP_CONPLETE in v11, DR_SWAP reserved. */
#if CYCCG_CCG_SUPPORT_SWAP_COMPLETE
	/*
	 * Support in HPI spec v11, it replace DR_SWAP, PR_SWAP and VCON_SWAP
	 * events.
	 */
	CY_PD_RESP_SWAP_COMPLETE,
#else
	CY_PD_RESP_DR_SWAP,    /* Reserved in HPI spec v11 */
#endif
	CY_PD_RESP_PR_SWAP,    /* Reserved in HPI spec v11 */
	CY_PD_RESP_VCON_SWAP,  /* Reserved in HPI spec v11 */
	CY_PD_RESP_PS_RDY,
	CY_PD_RESP_GOTOMIN,
	CY_PD_RESP_ACCEPT_MESSAGE,
	CY_PD_RESP_REJECT_MESSAGE,
	CY_PD_RESP_WAIT_MESSAGE,
	CY_PD_RESP_HARD_RESET,

	/* PD Data Message Specific Events, 0x90 */
	CY_PD_RESP_VDM_RECEIVED,

	/* Capability Message Specific Events, 0x91-0x92*/
	CY_PD_RESP_SRC_CAP_RCVD,
	CY_PD_RESP_SINK_CAP_RCVD,

	/* DP and Alternate mode Specific Events, 0x93-0x99*/
	CY_PD_RESP_DP_ALTERNATE_MODE,
	CY_PD_RESP_DP_DEVICE_CONNECTED,       /* Connected at UFP_U */
	CY_PD_RESP_DP_DEVICE_NOT_CONNECTED,   /* Not connected at UFP_U */
	CY_PD_RESP_DP_SID_NOT_FOUND,
	CY_PD_RESP_MULTIPLE_SVID_DISCOVERED,
	CY_PD_RESP_DP_FUNCTION_NOT_SUPPORTED,     /* Not supported by Cable */
	CY_PD_RESP_DP_PORT_CONFIG_NOT_SUPPORTED,  /* Not supported by UFP */

	/* Resets and Error Scenario Events, 0x9A-0xA5*/
	CY_PD_HARD_RESET_SENT,
	CY_PD_SOFT_RESET_SENT,
	CY_PD_CABLE_RESET,   /* New in HPI spec V11*/
	CY_PD_SOURCE_DISBALED_STATE_ENTERED,
	CY_PD_SENDER_RESPONSE_TIMER_TIMEOUT,
	CY_PD_NO_VDM_RESPONSE_RECEIVED,
	CY_PD_UNEXPECTED_VOLTAGE_VBUS,  /* New in HPI spec V11*/
	CY_PD_TYPE_C_ERROR_RECOVERY,    /* New in HPI spec V11*/

	/* EMCA Related Events, 0xA6 */
	CY_PD_EMCA_DETECTED = 0xA6,

	/* Rp Change Detected, 0xAA */
	CY_PD_RP_CHANGE_DETECTED = 0xAA,
};

/* GET_SILICON_ID Command Signature : "S". */
#define CY_PD_GET_SILICON_ID_CMD_SIG			0x53
/* INTR_REG Clear INTR Command. */
#define CY_PD_REG_INTR_REG_CLEAR_RQT			0x01
/* JUMP_TO_BOOT request signature 'J'. */
#define CY_PD_JUMP_TO_BOOT_CMD_SIG			0x4a
/* Device RESET Command Signature : "R". */
#define CY_PD_DEVICE_RESET_CMD_SIG			0x52
/* Device_Reset Request code. */
#define CY_PD_REG_RESET_DEVICE_CMD			0x01
/* ENTER FLASHING MODE Command Signature : "P". */
#define CY_PD_ENTER_FLASHING_MODE_CMD_SIG		0x50
/* FLASH_READ_WRITE Command Signature : "F". */
#define CY_PD_FLASH_READ_WRITE_CMD_SIG			0x46
/* Flash read command. */
#define CY_PD_REG_FLASH_ROW_READ_CMD			0x00
/* Flash write command. */
#define CY_PD_REG_FLASH_ROW_WRITE_CMD			0x01
/* FLASH_READ_WRITE row number LSB offset. */
#define CY_PD_REG_FLASH_READ_WRITE_ROW_LSB		0x02
/* FLASH_READ_WRITE row number MSB offset. */
#define CY_PD_REG_FLASH_READ_WRITE_ROW_MSB		0x03
/* U_VDM Type. */
#define CY_PD_U_VDM_TYPE				0x00

/* Bit mask for PD CONTRACT ESTABLISHED Field in PD_STATUS register. */
#define CY_PD_CONTRACT_ESTABLISHED_BIT_MASK		0x04
/* Bit mask for CURRENT DATA ROLE Field in PD_STATUS register. */
#define CY_PD_CURRENT_DATA_ROLE_BIT_MASK		0x40

#define CY_PD_MSG_MAX_DATA_SIZE		128
#define CY_PD_MSG_TYPE_MASK		0x80
#define CY_PD_MSG_CODE_MASK		0x7f
#define CY_PD_MSG_TYPE_CMD_RESP		0x00
#define CY_PD_MSG_TYPE_ASYNC_EVENT	0x80
#define IS_CY_PD_CMD_RESP_MSG(head) (\
	(((head)->code) & CY_PD_MSG_TYPE_MASK) == CY_PD_MSG_TYPE_CMD_RESP)
#define IS_CY_PD_ASYNC_EVENT_MSG(head) (\
	(((head)->code) & CY_PD_MSG_TYPE_MASK) == CY_PD_MSG_TYPE_ASYNC_EVENT)

#define CY_PD_RESP_MSG_CODE(head) (\
	((head)->code) & CY_PD_MSG_CODE_MASK)

#if CYCCG_CCG_SUPPORT_SWAP_COMPLETE
/* SWAP_STATUS macros */
#define HPI_SWAP_TYPE_MASK		0x0f
#define		HPI_DR_SWAP		0x00
#define		HPI_PR_SWAP		0x01
#define		HPI_VONN_SWAP		0x02
#define HPI_SWAP_RESP_MASK		0xf0
#define		HPI_SWAP_ACCEPT		0x00
#define		HPI_SWAP_REJECT		0x10
#define		HPI_SWAP_WAIT		0x20
#define		HPI_SWAP_NO_RESP	0x30
#define		HPI_SWAP_HARD_RESET	0x40
static u8 hpi_swap_type_pd_cmd_map[] = {
	CY_PD_CONTROL_DATA_ROLE_SWAP,
	CY_PD_CONTROL_POWER_ROLE_SWAP,
	CY_PD_CONTROL_VCON_ROLE_SWAP,
};
#endif

#define CYBIN_FLASH_HEAD_SIZE    6
#define CYBIN_FLASH_ROW_SIZE     128
#define CYBIN_LAST_FLASH_ROW_NUM 0x00ff
struct cybin_head {
	u16 silicon_id;
	u16 silicon_id_ext; /* Not used yet. */
	u8 silicon_rev;
	u8 checksum_type;
} __packed;

struct cybin_row {
	char colon;  /* colon is the start of each row data in .cybin file. */
	u8 flash_array_id;  /* Alwasy 0 */
	__be16 row_num;
	__be16 row_size;
	u8 row_data[CYBIN_FLASH_ROW_SIZE];
	u8 checksum;
} __packed;

struct cybin_image {
	struct cybin_head head;
	struct cybin_row data[0];
} __packed;

struct hpi_msg_head {
	u8 code;
	u8 len;
} __packed;

struct hpi_msg {
	struct hpi_msg_head head;
	u8 data[CY_PD_MSG_MAX_DATA_SIZE];
} __packed;

struct cyccg_i2c_data {
	u16 offset;
	u16 flags;
	u16 len;
	u16 buflen;
	u8 __user *buf;
} __packed;

struct cyccg_i2c_info {
	int bus;
	int addr;
	char name[CYCCG_MAX_NAME_SIZE];
} __packed;

struct ccg_version {
	u16 bl_build_num;
	u8 bl_maj_ver;
	u8 bl_min_ver;
	u8 bl_patch_ver;
	u8 app_maj_ver;
	u8 app_min_ver;
	u8 app_ext_ver;   /* External Circuit Specific Version. */
	char app_name[2];  /* Application Name. 2 character. */
};


/* PD Status */
#define CCG_STATUS_DEFAULT_PORT_DATA_ROLE_UFP  0
#define CCG_STATUS_DEFAULT_PORT_DATA_ROLE_DFP  1
#define CCG_STATUS_DEFAULT_PORT_DATA_ROLE_DRP  2

#define CCG_STATUS_DEFAULT_DATA_ROLE_OF_DRP_IS_UFP	0
#define CCG_STATUS_DEFAULT_DATA_ROLE_OF_DRP_IS_DFP	1

#define CCG_STATUS_DEFAULT_PORT_POWER_ROLE_SINK		0
#define CCG_STATUS_DEFAULT_PORT_POWER_ROLE_SOURCE	1
#define CCG_STATUS_DEFAULT_PORT_POWER_ROLE_DUAL_ROLE	2

#define CCG_STATUS_DEFAULT_POWER_ROLE_OF_DUAL_ROLE_IS_SINK    0
#define CCG_STATUS_DEFAULT_POWER_ROLE_OF_DUAL_ROLE_IS_SOURCE  1

#define CCG_STATUS_CURRENT_DATA_ROLE_UFP	0
#define CCG_STATUS_CURRENT_DATA_ROLE_DFP	1

#define CCG_STATUS_CURRENT_POWER_ROLE_SINK    0
#define CCG_STATUS_CURRENT_POWER_ROLE_SOURCE  1

#define CCG_STATUS_CONTRACT_NOTEXIST	0
#define CCG_STATUS_CONTRACT_EXIST	1

#define CCG_STATUS_EMCA_NOT_PRESENT	0
#define CCG_STATUS_EMCA_PRESENT		1

#define CCG_STATUS_CCG_IS_NOT_VCONN_SUPPLIER	0
#define CCG_STATUS_CCG_IS_VCONN_SUPPLIER	1

#define CCG_STATUS_CCG_IS_NOT_SOURCING_VCONN	0
#define CCG_STATUS_CCG_IS_SOURCING_VCONN	1

/* Type-C Status */
#define CCG_STATUS_TYPE_C_PORT_DISCONNECTED  0
#define CCG_STATUS_TYPE_C_PORT_CONNECTEd     1

#define CCG_STATUS_CC_POLARITY_CC1	0
#define CCG_STATUS_CC_POLARITY_CC2	1

#define CCG_STATUS_ATTACHED_DEV_TYPE_NOTHING			0
#define CCG_STATUS_ATTACHED_DEV_TYPE_UFP			1
#define CCG_STATUS_ATTACHED_DEV_TYPE_DFP			2
#define CCG_STATUS_ATTACHED_DEV_TYPE_DEBUG_ACCESSORY		3
#define CCG_STATUS_ATTACHED_DEV_TYPE_AUDIO_ACCESSORY		4
#define CCG_STATUS_ATTACHED_DEV_TYPE_POWERED_ACCESSORY		5
#define CCG_STATUS_ATTACHED_DEV_TYPE_UNSUPPORTED_ACCESSORY	6

#define CCG_STATUS_NOT_DETECT_RA  0
#define CCG_STATUS_DETECTED_RA    1

#define CCG_STATUS_TYPE_C_CURRENT_LEVEL_DEFAULT		0
#define CCG_STATUS_TYPE_C_CURRENT_LEVEL_1_5_A		1
#define CCG_STATUS_TYPE_C_CURRENT_LEVEL_3_0_A		2

struct ccg_status {
	u32 valid;  /* 0-This data is invalid, other value indicates valid. */
	u32 pd_status;
	u32 type_c_status;

	/* PD Status */
	u32 default_port_data_role : 2;
	u32 default_port_data_role_in_case_of_drp : 1;
	u32 default_port_power_role : 2;
	u32 default_port_power_role_in_case_of_dual_role : 1;

	/*
	 * Current Port Data Role and Current Port power role fields are
	 * valid only if Contract Status bit is set
	 */
	u32 current_port_data_role : 1;
	u32 current_port_power_role : 1;
	u32 contract_status : 1;

	u32 EMCA_present : 1;

	u32 VCONN_supplier : 1;
	u32 VCONN_status : 1;

	/* Type-C Status */
	u32 port_connected : 1;
	u32 CC_polarity : 1;
	u32 attached_dev_type : 3;
	u32 Ra_status : 1;
	u32 type_c_current_level : 2;
};

enum {
	DRP_MODE = 0,
	UFP_MODE,
	DFP_MODE,
	DEBUG_MODE,
	AUDIO_MODE,
};

struct cyccg_platform_data {
	int mode;
	int gpio_reset;
	int gpio_intr_comm;
	int gpio_uart_sw;
	int gpio_ad_sel;
	int external_5v_en_gpio;
	bool external_5v_en_flag;
	spinlock_t lock;
};
/* to access global platform data */
static struct cyccg_platform_data *cyccg_pdata;
static struct cyccg *g_cyccg;
static int cyccg_soft_reset = 0;
extern int pi5usb_set_msm_usb_host_mode(bool mode);
extern int wcd_mbhc_plug_detect(void);
extern void usb_audio_if_letv(bool *letv, int *pid);
extern void cclogic_set_audio_mode_register(void (*func)(bool));
extern void cclogic_set_vconn_register(void (*func)(bool));
extern int cclogic_set_vbus(bool on);
extern void cclogic_set_vconn(bool mode);
extern int cclogic_get_smbcharge_init_done(void);
extern void cclogic_updata_port_state(int state);
static bool cyccg_analog_headset_plugin = false;
bool letv_typec_plug_state;
extern bool typec_set_cc_state;
static struct mutex typec_headset_lock;
static struct delayed_work typec_uart_swap_dwork;
static bool typec_uart_scheduled = false;
static bool typec_vbus_vconn_state = false;
static bool typec_connect_state = false;
static int typec_headset_with_analog = -1;

enum cyccg_init_work_state {
	CYCCG_WORK_NONE,
	CYCCG_WORK_QUEUED,
	CYCCG_WORK_RUNNING,
};

#define CYCCG_EVT_CMD_SUCCESS	(0)
#define CYCCG_EVT_CMD_FAIL	(-EFAULT)

#define CYCCG_MAX_MSG_QUEUE 4
struct cyccg {
	struct i2c_client *client;
	struct device *ccgdev;
	struct list_head list;  /* ccgdev list. */
	int minor;  /* minor numder of the the char device of ccgdev. */
	char name[CYCCG_MAX_NAME_SIZE];  /* char device name of ccgdev. */

	bool irq_wake;

	struct work_struct init_work;
	struct mutex init_work_lock;
	int init_work_state;

	struct work_struct irq_work;
	struct mutex irq_work_lock;
	int irq_work_state;

#if CYCCG_AUTO_INTERNAL_UPDATE
	struct work_struct auto_fw_update_work;
	struct mutex auto_fw_update_work_lock;
	int auto_fw_update_work_state;
	struct mutex fw_lock;
#endif

#if CYCCG_DEBUG
	struct work_struct vdm_work;
	struct mutex vdm_work_lock;
	int vdm_work_state;
#endif

	u16 silicon_id;
	struct ccg_version bl_ver;
	struct ccg_version app_ver;

	struct mutex lock;  /* protect cyccg data structure. */
	int open;

	/* These two variables must be protected by global @lock variable. */
	int cmd_max_timeout;
	int cmd_wait_internal;

	atomic_t is_irq_owned_by_drv;  /* driver internal processing. */

	struct mutex cmd_lock;
	atomic_t cmd_issued;
	/* points to excepted event code array, end with 0. */
	u8 *excepted_events;
	struct hpi_msg *resp_msg;
	struct completion resp_ready;
	/* used to protect msg_queue and relstive variables. */
	struct mutex msg_queue_lock;
	bool enable_msg_queue;
	/* Max store CYCCG_MAX_MSG_QUEUE - 1 hip_msgs. */
	struct hpi_msg msg_queue[CYCCG_MAX_MSG_QUEUE];
	int enqueue_index;
	int dequeue_index;

	struct mutex ext_evt_lock;
//letv_pd s
	struct mutex letv_i2c_write;
//letv_pd e
	int ext_evt_waiting;
	int ext_evt_cmd_err_code;
	int ext_evt_reason;  /* power/data/vconn role swap command. */
	struct completion ext_evt_done;
	bool is_swap_initiated_by_partner;
	bool is_ap_synced;
	/*
	 * This status records the status when the Type-C is connected or the PD
	 * Contract is established, so it's always maintains the latest status
	 * of the CCGx device.
	 * During the SWAP process, the value Type-C and PD Status may be have
	 * been changed or not, unknown, so at that time, the status read out
	 * is not realible, so must be the recorded status here.
	 */
	struct ccg_status ccg_status;
	struct mutex ccg_status_lock;

	atomic_t sigio_queued;
	struct fasync_struct *async_queue;
	struct completion ioctl_clear_intr;
	struct cyccg_platform_data *pdata;

	/* For debug purpose, directly read/write on I2C bus. */
	int offset;
	int count;

	/* used to avoid kmalloc/kfree in I2C bus write, protected by @lock. */
	u8 xfer_buf[CYCCG_I2C_REGISTER_MAP_SIZE + 1] ____cacheline_aligned;
	struct regulator *boost_5v;
};

static DEFINE_MUTEX(cyccgdev_lock);
#define CYCCG_DYNAMIC_MINORS 64
static DECLARE_BITMAP(cyccgdev_minors, CYCCG_DYNAMIC_MINORS);
static struct class *cyccg_class;
static int cyccg_major;
static LIST_HEAD(cyccgdev_list);

static int cyccg_hpi_clear_intr(struct cyccg *cyccg);
static int cyccg_hpi_poll_events(struct cyccg *cyccg, struct hpi_msg *resp,
				 u8 *excepted_events);
static int cyccg_hpi_swap_cmd_send(struct cyccg *cyccg, u8 swap_cmd);
static int cyccg_get_ccg_status(struct cyccg *cyccg,
				struct ccg_status *ccg_status);
static int cyccg_send_vdm_data(struct cyccg *cyccg, u8 *vdm_data, int count);
static void cyccg_dump_pending_events(struct cyccg *cyccg);
static int cyccg_hpi_cmd_sync(struct cyccg *cyccg,
		u8 *cmd, int cmd_len, int cmd_addr,
		struct hpi_msg *cmd_resp, u8 *excepted_events,
		int timeout);

//letv_pd s
static unsigned long get_bits_value(unsigned char *arr,
				    int bit_offset, int bit_num);
static int cyccg_i2c_read(struct cyccg *cyccg, u8 *buf, int count, int offset);
static int cyccg_hpi_cmd_sync(struct cyccg *cyccg,
		u8 *cmd, int cmd_len, int cmd_addr,
		struct hpi_msg *cmd_resp, u8 *excepted_events,
		int timeout);
static int cyccg_swap_cmd_process(struct cyccg *cyccg,
				  struct cyccg_ext_ap_cmd *ext_cmd,
				  void *ext_cmd_arg);
static int cyccg_port_disable(struct cyccg *cyccg);
static int cyccg_port_reset(struct cyccg *cyccg);
#include "letv_pd.c"
//letv_pd e

#if CYCCG_DEBUG
/* This interface is only for test purpose. */
static void mem_dump(int flag, const u8 *buf, int size)
{
	#define BYTES_PRE_ROW 16
	int i, j, rows;
	int offset;
	char one_byte_str[4];
	char row_byte_str[BYTES_PRE_ROW * 4];

	if (!flag)
		return;

	rows = size / BYTES_PRE_ROW;
	for (i = 0; i <= rows; i++) {
		offset = BYTES_PRE_ROW * i;
		if (offset >= size) {
			/*
			 * avoid output duplicate empty line when total bytes
			 * is properly multiple of BYTES_PRE_ROW.
			 */
			goto out;
		}

		memset(row_byte_str, 0, sizeof(row_byte_str));
		for (j = 0; j < BYTES_PRE_ROW; j++) {
			offset = BYTES_PRE_ROW * i + j;
			if (offset >= size) {
				/* end of the buffer bytes output. */
				pr_err("%s\n", row_byte_str);
				goto out;
			}

			/* output bytes in same row. */
			sprintf(one_byte_str, "%02x ", buf[offset]);
			strcat(row_byte_str, one_byte_str);
		}
		/* 16 bytes output, turned to new line. */
		pr_err("%s\n", row_byte_str);
	}

out:
	return;
}
#else
static void mem_dump(int flag, const u8 *buf, int size) {}
#endif

static struct cyccg *inode_to_cyccg(struct inode *inode)
{
	int minor = iminor(inode);
	struct cyccg *cyccg;
	bool found = false;

	mutex_lock(&cyccgdev_lock);
	list_for_each_entry(cyccg, &cyccgdev_list, list) {
		if (cyccg->minor == minor) {
			found = true;
			break;
		}
	}
	mutex_unlock(&cyccgdev_lock);

	return found ? cyccg : NULL;
}

static int cyccg_fasync_unlocked(int fd, struct file *file, int on)
{
	struct cyccg *cyccg = file->private_data;
	int ret;

	ret = fasync_helper(fd, file, on, &cyccg->async_queue);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (ret > 0) {
		if (on)
			enable_irq(cyccg->client->irq);
		else
			disable_irq(cyccg->client->irq);
	}
#endif

	return ret < 0 ? ret : 0;
}

static int cyccg_fasync(int fd, struct file *file, int on)
{
	struct cyccg *cyccg = file->private_data;
	int ret;

	mutex_lock(&cyccg->lock);
	ret = cyccg_fasync_unlocked(fd, file, on);
	mutex_unlock(&cyccg->lock);

	return ret;
}

static int cyccg_open(struct inode *inode, struct file *file)
{
	struct cyccg *cyccg = inode_to_cyccg(inode);
	int err = 0;

	if (!cyccg)
		return -EINVAL;

	mutex_lock(&cyccg->lock);
	if (cyccg->open) {
		err = -EBUSY;
		goto out;
	}

	cyccg->open++;
	file->private_data = cyccg;
out:
	mutex_unlock(&cyccg->lock);
	return err;
}

static int cyccg_close(struct inode *inode, struct file *file)
{
	struct cyccg *cyccg = file->private_data;
	int err = 0;

	if (!cyccg)
		return -EINVAL;

	mutex_lock(&cyccg->lock);
	if (!cyccg->open) {
		err = -EINVAL;
		goto out;
	}

	cyccg_fasync_unlocked(-1, file, 0);

	cyccg->open--;
	file->private_data = NULL;
out:
	mutex_unlock(&cyccg->lock);
	return err;
}

static int cyccg_i2c_read(struct cyccg *cyccg, u8 *buf, int count, int offset)
{
	struct i2c_client *client = cyccg->client;
	u8 read_offset = (u8)offset;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &read_offset,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = count,
			.buf = buf,
		},
	};
	int i = 0, ret;

	if (!buf || offset > (CYCCG_I2C_REGISTER_MAP_SIZE - 1) ||
		(count + offset) > CYCCG_I2C_REGISTER_MAP_SIZE)
		return -EINVAL;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		/* Retry I2C when error */
		if (ret < 0) {
			cyccg_dbg("%s: I2C error(%d), retry\n", __func__, ret);
			while (i < 3) {
				ret = i2c_transfer(client->adapter, msgs,
						   ARRAY_SIZE(msgs));
				if (ret == ARRAY_SIZE(msgs)) {
					cyccg_dbg("%s: retry successfully\n",
						  __func__);
					return 0;
				} else if (ret >= 0) {
					break;
				}
				i++;
				cyccg_dbg("%s: retrying(%d) fail, ret=%d\n",
					  __func__, i, ret);
			}
			pr_err("%s: retry fail, ret=%d\n", __func__, ret);
		}
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int cyccg_i2c_write(struct cyccg *cyccg, u8 *buf, int count, int offset)
{
	struct i2c_client *client = cyccg->client;
	int write_count = count + 1;
	int i = 0, ret;

	if (!buf || offset > (CYCCG_I2C_REGISTER_MAP_SIZE - 1) ||
		(count + offset) > CYCCG_I2C_REGISTER_MAP_SIZE)
		return -EINVAL;
//letv_pd s
	mutex_lock(&cyccg->letv_i2c_write);
//letv_pd e
	cyccg->xfer_buf[0] = (u8)offset;
	memcpy(&cyccg->xfer_buf[1], buf, count);
	ret = i2c_master_send(client, cyccg->xfer_buf, write_count);
	if (ret != write_count) {
		/* Retry I2C when error */
		if (ret < 0) {
			cyccg_dbg("%s: I2C error(%d), retry\n", __func__, ret);
			while (i < 3) {
				ret = i2c_master_send(client, cyccg->xfer_buf,
						      write_count);
				if (ret == write_count) {
					cyccg_dbg("%s: retry successfully\n",
						  __func__);
//letv_pd s
					mutex_unlock(&cyccg->letv_i2c_write);
//letv_pd e
					return 0;
				} else if (ret >= 0) {
					break;
				}
				i++;
				cyccg_dbg("%s: retrying(%d) fail, ret=%d\n",
					  __func__, i, ret);
			}
			pr_err("%s: retry fail, ret=%d\n", __func__, ret);
		}
//letv_pd s
		mutex_unlock(&cyccg->letv_i2c_write);
//letv_pd e
		return ret < 0 ? ret : -EIO;
	}
//letv_pd s
	mutex_unlock(&cyccg->letv_i2c_write);
//letv_pd e
	return 0;
}

static int cyccg_i2c_rw(struct cyccg *cyccg,
		unsigned int cmd, unsigned long arg, int compat_mode)
{
	struct i2c_client *client = cyccg->client;
	struct cyccg_i2c_data i2c_data;
	int write_len;
	int err = 0;
	int ret;

	if (copy_from_user(&i2c_data,
			   (struct cyccg_i2c_data __user *)arg,
			   sizeof(i2c_data)))
		return -EFAULT;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
#ifdef CONFIG_COMPAT
	if (compat_mode)
		i2c_data.buf =
			(u8 __user *)compat_ptr((unsigned long)i2c_data.buf);
#endif
#endif

	if ((i2c_data.offset + i2c_data.len) > CYCCG_I2C_REGISTER_MAP_SIZE ||
		i2c_data.offset >= CYCCG_I2C_REGISTER_MAP_SIZE ||
		!i2c_data.buf || i2c_data.buflen < i2c_data.len)
		return -EINVAL;

	switch (cmd) {
	case CYCCG_I2C_READ:
		err = cyccg_i2c_read(cyccg,
				     cyccg->xfer_buf,
				     (int)i2c_data.len,
				     (int)i2c_data.offset);
		if (err)
			return err;

		if (copy_to_user(i2c_data.buf, cyccg->xfer_buf, i2c_data.len))
			return -EFAULT;

		break;
	case CYCCG_I2C_WRITE:
		cyccg->xfer_buf[0] = (u8)i2c_data.offset;
		if (copy_from_user(&cyccg->xfer_buf[1],
				   i2c_data.buf,
				   i2c_data.len))
			return -EFAULT;
		write_len = i2c_data.len + 1;

		ret = i2c_master_send(client, cyccg->xfer_buf, write_len);
		if (ret != write_len)
			return ret < 0 ? ret : -EIO;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int cyccg_get_latest_ccg_status(struct cyccg *cyccg,
				       struct ccg_status *ccg_status)
{
	int err;

	mutex_lock(&cyccg->ccg_status_lock);
	err = cyccg_get_ccg_status(cyccg, &cyccg->ccg_status);
	if (err)  /* Try to retry once again. */
		err = cyccg_get_ccg_status(cyccg, &cyccg->ccg_status);
	if (!err && ccg_status)
		memcpy(ccg_status, &cyccg->ccg_status,
		       sizeof(struct ccg_status));
	mutex_unlock(&cyccg->ccg_status_lock);
	if (err) {
		cyccg_dbg("%s: failed to read ccg_status\n", __func__);
		return err;
	}

	return 0;
}

#define TYPEC_UART_SWAP_TIMEOUT 1000000
static void typec_uart_swap_handler(struct work_struct *work)
{
	struct cyccg_platform_data *pdata = cyccg_pdata;

	pr_info("%s enter!!\n", __func__);
	gpio_set_value(pdata->gpio_uart_sw, 1);
	if (typec_uart_scheduled)
		typec_uart_scheduled = false;

	return;
}

/*This func is for headset switch AD mode*/
void cyccg_cclogic_set_audio_mode(bool mode)
{
	/*true for Analog mode and false for Digit mode*/
	struct cyccg_platform_data *pdata = cyccg_pdata;
	mutex_lock(&typec_headset_lock);
	if (mode) {
		if (typec_uart_scheduled) {
			pr_info("%s, cancel schedule!\n", __func__);
			typec_uart_scheduled = false;
			cancel_delayed_work(&typec_uart_swap_dwork);
		}
		pr_info("cyccg switch to audio mode\n");
		gpio_set_value(pdata->gpio_uart_sw, 1);
		gpio_set_value(pdata->gpio_ad_sel, 1);
		/* 50 ms should be enough for usb device to disconnect.
		 * This is to avoid usb device disconnect during usb host
		 * stopping which will cause race condition
		 */
		msleep(50);

		pi5usb_set_msm_usb_host_mode(false);
		cclogic_set_vconn(true);

		typec_vbus_vconn_state = true;
		if (!typec_connect_state) {
			pr_info("%s, typec has been unplug!\n", __func__);
			mutex_unlock(&typec_headset_lock);
			return;
		}
		cyccg_pdata->mode = AUDIO_MODE;
		if (!letv_typec_plug_state)
			letv_typec_plug_state = true;
		//notify audio module for headset plug in
		if (!cyccg_analog_headset_plugin) {
			pr_info("audio headset plug in!!\n");
			wcd_mbhc_plug_detect();
			cyccg_analog_headset_plugin = true;
		}
		typec_headset_with_analog = 1;
	} else {
		cclogic_set_vconn(false);
		gpio_set_value(pdata->gpio_uart_sw, 0);
		gpio_set_value(pdata->gpio_ad_sel, 0);

		if (typec_connect_state) {
			pr_info("%s, typec has been plug!\n", __func__);
		}
		//notify audio module for headset plug out
		if (cyccg_analog_headset_plugin) {
			pr_info("audio headset plug out!!\n");
			wcd_mbhc_plug_detect();
			cyccg_analog_headset_plugin = false;
		}
		msleep(60);
		pi5usb_set_msm_usb_host_mode(true);
		pr_info("cyccg set usb to host mode\n");
		pdata->mode = DFP_MODE;
		typec_headset_with_analog = 0;
		typec_vbus_vconn_state = false;
		if (letv_typec_plug_state)
			letv_typec_plug_state = false;
		if (!typec_uart_scheduled) {
			pr_info("scheduled uart_sw to (1)!!\n");
			schedule_delayed_work(&typec_uart_swap_dwork,
				usecs_to_jiffies(TYPEC_UART_SWAP_TIMEOUT));
			typec_uart_scheduled = true;
		}
	}
	mutex_unlock(&typec_headset_lock);
	return;
}
EXPORT_SYMBOL(cyccg_cclogic_set_audio_mode);

static int typec_headset_with_analog_parm_get(char *buffer, struct kernel_param *kp)
{
	if (typec_headset_with_analog == 1) {
		return sprintf(buffer, "1");
	} else if (typec_headset_with_analog == 0) {
		return sprintf(buffer, "0");
	} else {
		return sprintf(buffer, "-1");
	}
	return 0;
}

static int typec_headset_with_analog_parm_set(const char *val, struct kernel_param *kp)
{
	param_set_int(val, kp);
	return 0;
}
module_param_call(typec_headset_with_analog, typec_headset_with_analog_parm_set,
		typec_headset_with_analog_parm_get, &typec_headset_with_analog, 0664);

#if CYCCG_POWER_CONTROLLED_BY_EXTERNAL_AP
/*
 * Sync exteranl AP status with the status specificed by the @ccg_status.
 * Variable @ccg_status here indicates the real CCGx device status
 * that AP needs to support.
 */
static int cyccg_sync_ap_status(struct cyccg *cyccg,
				struct ccg_status *ccg_status)
{
	int err = 0;

	cyccg_dbg("%s: contract_status=%u, port_connected=%u\n", __func__,
		  ccg_status->contract_status, ccg_status->port_connected);

	/*
	 * No PD contract established and Type-C connected.
	 * e.g.: attached device removed.
	 */
	if (!ccg_status->contract_status && !ccg_status->port_connected) {
		cyccg_dbg("%s: sync AP with no PD Contract Established.\n",
			  __func__);
		cyccg_dbg("%s: sync AP with no Type-C connected.\n", __func__);
		/*
		 * TODO(third-party):
		 *   Here, depending on customer's strategy on how to sync
		 *   with AP statusl.
		 *   e.g.: turn VBSU off, set to DRP, or other status.
		 */
		pr_info("%s: cable unplug\n", __func__);

		pr_info("cyccg switch to SBU1/SBU2,when cable disconnect\n");
		gpio_set_value(cyccg_pdata->gpio_uart_sw, 1);
		gpio_set_value(cyccg_pdata->gpio_ad_sel, 0);
		cclogic_updata_port_state(0);
		pr_info("cyccg disable usb host mode,when cable disconnect\n");
		cclogic_set_vbus(0);
		pi5usb_set_msm_usb_host_mode(false);
		if (typec_headset_with_analog != -1) {
			pr_info("%s: clear type-c connect state!\n", __func__);
			typec_headset_with_analog = -1;
		}
		//notify audio module for headset plug out
		if (cyccg_analog_headset_plugin) {
			pr_info("%s, audio headset plug out!!\n", __func__);
			wcd_mbhc_plug_detect();
			cyccg_analog_headset_plugin = false;
		}
		if (letv_typec_plug_state)
			letv_typec_plug_state = false;
		if (typec_vbus_vconn_state) {
			pr_info("%s: clear vbus and vconn state!\n", __func__);
			cclogic_set_vbus(0);
			typec_vbus_vconn_state = false;
		}
		if (typec_set_cc_state) {
			pr_info("%s: clear typec_set_cc_state!\n", __func__);
			typec_set_cc_state = false;
		}
		typec_connect_state = false;

		cyccg_pdata->mode = DRP_MODE;
		return err;
	}

	/* PD Contract has established, Type-C may connected or not. */
	if (ccg_status->contract_status) {
		/*
		 * Because on Huawei board, the external AP does not support
		 * swap the data role singally. For the external AP, when
		 * swap the data role to UFP, the power role will also be
		 * swapped to Sink, and DFP to Source. This will cause the power
		 * role be swapped as unexcepted, which will cause HARD_RESET
		 * and other protocol and state's issues.
		 * So the data role swap can be supported on the Huawei board,
		 * must not try to sync the data role for the AP.
		 */
		if (ccg_status->current_port_data_role ==
			CCG_STATUS_CURRENT_DATA_ROLE_DFP) {
			cyccg_dbg("%s: set to OTG host.\n", __func__);
			/*
			 * TODO(third-party):
			 *   Notify AP to set as otg host.
			 *   If AP has set otg host mode, do nothing.
			 */
		} else {
			cyccg_dbg("%s: set to OTG device.\n", __func__);
			/*
			 * TODO(third-party):
			 *   Notify AP to set as otg device.
			 *   If AP has set otg device mode, do nothing.
			 */
		}

		if (ccg_status->current_port_power_role ==
			CCG_STATUS_CURRENT_POWER_ROLE_SOURCE) {
			cyccg_dbg("%s: turn VBUS on.\n", __func__);
			cclogic_set_vbus(1);
			/*
			 * TODO(third-party):
			 *   Notify AP to supply power, turn VBUS on.
			 *   If AP has VBUS on, do nothing.
			 */
		} else {
			cyccg_dbg("%s: turn VBUS off.\n", __func__);
			cclogic_set_vbus(0);
			/*
			 * TODO(third-party):
			 *   Notify AP not to supply power, turn VBUS off.
			 *   If AP has VBUS off, do nothing.
			 */
		}

		return err;
	}

	/* Only Type-C connected. */
	if (ccg_status->port_connected) {
		switch (ccg_status->attached_dev_type) {
		case CCG_STATUS_ATTACHED_DEV_TYPE_UFP:
			if(!cclogic_get_smbcharge_init_done())
			{
				cyccg_dbg("%s: smbcharger not ready.\n", __func__);
				break;
			}
			cyccg_dbg("%s: turn VBUS on.\n", __func__);
			cyccg_dbg("%s: set to OTG host.\n", __func__);
			cclogic_set_vbus(1);
			/*
			 * TODO(third-party):
			 *   Notify AP to set as otg host and turn VBUS on.
			 *   If AP has set otg host mode and VBUS is on,
			 *   do nothing.
			 */
			pr_info("cyccg switch to SBU1/SBU2\n");
			gpio_set_value(cyccg_pdata->gpio_uart_sw, 1);
			gpio_set_value(cyccg_pdata->gpio_ad_sel, 0);
			pr_info("cyccg set usb to host mode\n");
			pi5usb_set_msm_usb_host_mode(true);
			cyccg_pdata->mode = DFP_MODE;//mobile is DFP, sink device is UFP
			cclogic_updata_port_state(2);
//letv_pd s
			letv_pd_set_usb_mode(LETV_USB_DFP_MODE);
//letv_pd e
			typec_connect_state = true;
			break;
		case CCG_STATUS_ATTACHED_DEV_TYPE_DFP:
			cyccg_dbg("%s: turn VBUS off.\n", __func__);
			cclogic_set_vbus(0);
			cyccg_dbg("%s: set to OTG device.\n", __func__);
			/*
			 * TODO(third-party):
			 *   Notify AP to set as otg device and turn VBUS off.
			 *   If AP has set otg device mode and VUS is off,
			 *   do nothing.
			 */
			pr_info("cyccg switch to SBU1/SBU2\n");
			gpio_set_value(cyccg_pdata->gpio_uart_sw, 1);
			gpio_set_value(cyccg_pdata->gpio_ad_sel, 0);
			cyccg_pdata->mode = UFP_MODE;
			cclogic_updata_port_state(1);
//letv_pd s
			letv_pd_set_usb_mode(LETV_USB_UFP_MODE);
//letv_pd e
			break;
		case CCG_STATUS_ATTACHED_DEV_TYPE_DEBUG_ACCESSORY:
			cyccg_dbg("%s: attached_dev_type: DEBUG_ACCESSORY.\n",
				  __func__);
			/* TODO: if supports it. */
			pr_info("cyccg switch uart_sw to uart mode\n");
			gpio_set_value(cyccg_pdata->gpio_uart_sw, 0);
			gpio_set_value(cyccg_pdata->gpio_ad_sel, 0);

			cyccg_pdata->mode = DEBUG_MODE;
			cclogic_updata_port_state(4);
			break;
		case CCG_STATUS_ATTACHED_DEV_TYPE_AUDIO_ACCESSORY:
			cyccg_dbg("%s: attached_dev_type: AUDIO_ACCESSORY.\n",
				  __func__);
			/* TODO: if supports it. */
			pr_info("cyccg switch to audio mode\n");
			gpio_set_value(cyccg_pdata->gpio_uart_sw, 1);
			gpio_set_value(cyccg_pdata->gpio_ad_sel, 1);

			cyccg_pdata->mode = AUDIO_MODE;
			cclogic_updata_port_state(3);
			//notify audio module for headset plug in
			if (!cyccg_analog_headset_plugin) {
				pr_info("%s, audio headset plug in!!\n", __func__);
				wcd_mbhc_plug_detect();
				cyccg_analog_headset_plugin = true;
			}
			break;
		case CCG_STATUS_ATTACHED_DEV_TYPE_POWERED_ACCESSORY:
			cyccg_dbg("%s: attached_dev_type: POWERED_ACCESSORY.\n",
				  __func__);
			/* TODO: if supports it. */
			break;
		case CCG_STATUS_ATTACHED_DEV_TYPE_NOTHING:
			cyccg_dbg("%s: attached_dev_type: NOTHING.\n",
				  __func__);
			break;
		case CCG_STATUS_ATTACHED_DEV_TYPE_UNSUPPORTED_ACCESSORY:
		default:
			cyccg_dbg("%s: attached_dev_type = %u, UNSUPPORTED.\n",
				  __func__, ccg_status->attached_dev_type);
			break;
		}
	}

	return err;
}

static int cyccg_update_external_ap_status(struct cyccg *cyccg, u8 event)
{
	struct ccg_status ccg_status;
	int err = 0;

	cyccg_dbg("%s: event = 0x%02x.\n", __func__, event);
	if (event == 0) {
		cyccg_dbg("%s: try to sync CCG and AP status.\n", __func__);
		/*
		 * In this case, enterred when PR_SWAP command encounter
		 * failed, so sync the status with CCG and AP to vaoid further
		 * possible issues.
		 */
		err = cyccg_get_latest_ccg_status(cyccg, &ccg_status);
		if (err) {
			cyccg_dbg("%s: failed to get latest ccg_status.\n",
				  __func__);
			return err;
		}

		/* Force sync with the AP status */
		mutex_lock(&cyccg->ext_evt_lock);
		err = cyccg_sync_ap_status(cyccg, &ccg_status);
		/* CCG and AP in same status now. */
		cyccg->is_ap_synced = true;
		mutex_unlock(&cyccg->ext_evt_lock);
		return err;
	}

	mutex_lock(&cyccg->ccg_status_lock);
	if (!cyccg->ccg_status.valid)
		err = -EINVAL;
	else
		memcpy(&ccg_status, &cyccg->ccg_status, sizeof(ccg_status));
	mutex_unlock(&cyccg->ccg_status_lock);
	if (err) {
		cyccg_dbg("%s: invalid ccg_status data structure.\n", __func__);
		goto out;
	}

	mutex_lock(&cyccg->ext_evt_lock);
	if (!cyccg->ext_evt_waiting) {
		cyccg_dbg("%s: CCG doesn't send Role Swap command.\n",
			  __func__);
		switch (event) {
		case CY_PD_RESP_TYPE_C_CONNECTED:
			cyccg_dbg("%s: Type-C connected, sync AP status.\n",
				  __func__);
			/* Currently, whatever only depending Type-C status. */
			ccg_status.contract_status = 0;

			err = cyccg_sync_ap_status(cyccg, &ccg_status);

			/* CCG and AP in same status now. */
			cyccg->is_ap_synced = true;
			break;
		case CY_PD_RESP_TYPE_C_DISCONNECTED:
			cyccg_dbg("%s: type-c disconnected, sync AP status.\n",
				  __func__);
			/* Must be no Type-C and PD connect established. */
			ccg_status.contract_status = 0;
			ccg_status.port_connected = 0;

			err = cyccg_sync_ap_status(cyccg, &ccg_status);

			/* CCG and AP in same status. */
			cyccg->is_ap_synced = true;
			break;
		case CY_PD_RESP_PR_SWAP:
			/*
			 * The PR_SWAP is initiated by Port Partner.
			 *
			 * Note, when PR_SWAP event is received by the driver,
			 * the register value of the PD_STATUS has already
			 * become invaild, so cannot use the PD_STATUS at time
			 * time.
			 */
			cyccg_dbg("%s: not wait, CY_PD_RESP_PR_SWAP\n",
				  __func__);

			cyccg->is_swap_initiated_by_partner = true;
			cyccg->is_ap_synced = false;

			if (ccg_status.current_port_power_role ==
				CCG_STATUS_CURRENT_POWER_ROLE_SOURCE) {
				cyccg_dbg("%s: current, source\n", __func__);
				/*
				 * PR_SWAP initiated by Port Partner,
				 * and CCG is Source, so Swap the CCG power
				 * role to Sink.
				 */
				ccg_status.current_port_power_role =
					CCG_STATUS_CURRENT_POWER_ROLE_SINK;

				err = cyccg_sync_ap_status(cyccg, &ccg_status);

				/* Keep in pair, done.*/
				cyccg->is_swap_initiated_by_partner = false;
				cyccg->is_ap_synced = true;
			} else {
				cyccg_dbg("%s: current, sink\n", __func__);
				/*
				 * Do nothing, waiting for the PS_RDY event.
				 * Now, is_swap_initiated_by_partner = true;
				 */
			}
			break;
		case CY_PD_RESP_PS_RDY:
			if ((cyccg->is_swap_initiated_by_partner == true) &&
			    (ccg_status.current_port_power_role ==
				CCG_STATUS_CURRENT_POWER_ROLE_SINK)) {
				cyccg_dbg("%s: no wait, PS_RDY, SINK\n",
					  __func__);
				/*
				 * RP_SWAP initiated by Port Partner,
				 * and CCG is Sink, Swap to Source.
				 */
				ccg_status.current_port_power_role =
					CCG_STATUS_CURRENT_POWER_ROLE_SOURCE;

				err = cyccg_sync_ap_status(cyccg, &ccg_status);

				/* Keep in pair, done.*/
				cyccg->is_swap_initiated_by_partner = false;
				cyccg->is_ap_synced = true;
			}
			break;
		}
		goto out;
	}

	if (!ccg_status.contract_status ||
		cyccg->is_swap_initiated_by_partner == true) {
		cyccg_dbg("%s: invalid: ccg_status: %s=%u, %s=%u", __func__,
			"contract_status", ccg_status.contract_status,
			"is_swap_initiated_by_partner",
			cyccg->is_swap_initiated_by_partner);
		/* No PD contract established, do nothing. */
		err = -EINVAL;
		goto out;
	}

	switch (event) {
	case CY_PD_RESP_ACCEPT_MESSAGE:
		if (ccg_status.current_port_power_role ==
			CCG_STATUS_CURRENT_POWER_ROLE_SOURCE) {
			cyccg_dbg("%s: wait, ACCEPT_MESSAGE, SOURCE\n",
				  __func__);
			/*
			 * The PR_SWAP initiated by CCG host, and CCG is source.
			 * Swap to sink.
			 */
			ccg_status.current_port_power_role =
					CCG_STATUS_CURRENT_POWER_ROLE_SINK;

			err = cyccg_sync_ap_status(cyccg, &ccg_status);

			cyccg->is_ap_synced = true;
		}
		break;
	case CY_PD_RESP_PS_RDY:
		if ((cyccg->is_ap_synced == false) &&
		    (ccg_status.current_port_power_role ==
			CCG_STATUS_CURRENT_POWER_ROLE_SINK)) {
			cyccg_dbg("%s: wait, PS_RDY, SINK\n",
				  __func__);
			/*
			 * The PR_SWAP initiated by CCG host, and CCG is Sink.
			 * Swap to Source.
			 */
			ccg_status.current_port_power_role =
					CCG_STATUS_CURRENT_POWER_ROLE_SOURCE;

			err = cyccg_sync_ap_status(cyccg, &ccg_status);

			cyccg->is_ap_synced = true;
		}
		break;
	}

out:
	mutex_unlock(&cyccg->ext_evt_lock);
	return err;
}
#endif  /* CYCCG_POWER_CONTROLLED_BY_EXTERNAL_AP */

static int cyccg_swap_cmd_process(struct cyccg *cyccg,
				  struct cyccg_ext_ap_cmd *ext_cmd,
				  void *ext_cmd_arg)
{
	struct ccg_status ccg_status;
	int err;

	/* Read CCG device status. */
	mutex_lock(&cyccg->ccg_status_lock);
	if (cyccg->ccg_status.valid) {
		memcpy(&ccg_status, &cyccg->ccg_status, sizeof(ccg_status));
		err = 0;
	} else {
		/*
		 * Becase after DR_SWAP command the TYPE_C_CONNECTED event won't
		 * be reported again, so there is no way to update the
		 * ccg_status if previous command is DR_SWAP, so to avoid the
		 * invalid ccg_status issue which caused by the previous
		 * DR_SWAP command, must make sure the ccg_status is synced with
		 * the CCGx device before next SWAP command.
		 * Note, here, assue the CCGx device's status is stable before
		 * the SWAP command, caller must make sure this.
		 */
		err = cyccg_get_ccg_status(cyccg, &cyccg->ccg_status);
		if (!err)
			memcpy(&ccg_status,
			       &cyccg->ccg_status, sizeof(ccg_status));
	}
	mutex_unlock(&cyccg->ccg_status_lock);
	if (err) {
		dev_err(&cyccg->client->dev, "%s: %s or %s\n", __func__,
			"invalid ccg_status data structure",
			"no attached device.\n");
		return err;
	}

	switch (ext_cmd->cmd) {
	case CY_PD_CONTROL_POWER_ROLE_SWAP:
		cyccg_dbg("%s: CY_PD_CONTROL_POWER_ROLE_SWAP\n", __func__);
		/* 1. Command CCG2 to do power role swap. */
		if (!(ext_cmd->flags & 0x01)) {
			err = cyccg_hpi_swap_cmd_send(
					cyccg, CY_PD_CONTROL_POWER_ROLE_SWAP);
			if (err)
				break;
		}

		/*
		 * 2. Power role swap has done, do addtional operation here
		 *    if required by external AP.
		 *    Because the the real AP control are applied when PS_READY
		 *    event is processed.
		 *    So, here, is just for left operations if required,
		 *    normally, should do nothing here.
		 */
		err = 0;
		break;
	case CY_PD_CONTROL_DATA_ROLE_SWAP:
		cyccg_dbg("%s: CY_PD_CONTROL_DATA_ROLE_SWAP\n", __func__);
		/* 1. Command CCG2 to do data role swap. */
		if (!(ext_cmd->flags & 0x01)) {
			err = cyccg_hpi_swap_cmd_send(
					cyccg, CY_PD_CONTROL_DATA_ROLE_SWAP);
			if (err)
				break;
		}

		/*
		 * 2. Power role swap has done, do addtional operation here
		 *    if required by external AP.
		 *    Because the the real AP control are applied when PS_READY
		 *    event is processed.
		 *    So, here, is just for left operations if required,
		 *    normally, should do nothing here.
		 */
		err = 0;
		break;
	case CY_PD_CONTROL_VCON_ROLE_SWAP:
		cyccg_dbg("%s: CY_PD_CONTROL_VCON_ROLE_SWAP\n", __func__);
		/* 1. Command CCG2 to do power role swap. */
		if (!(ext_cmd->flags & 0x01)) {
			err = cyccg_hpi_swap_cmd_send(
					cyccg, CY_PD_CONTROL_VCON_ROLE_SWAP);
			if (err)
				break;
		}

		/*
		 * 2. Power role swap has done, do addtional operation here
		 *    if required by external AP.
		 *    Because the the real AP control are applied when PS_READY
		 *    event is processed.
		 *    So, here, is just for left operations if required,
		 *    normally, should do nothing here.
		 */
		err = 0;
		break;
	default:
		cyccg_dbg("%s: unknown: 0x%08x, flags: 0x%08x, arg_size=%u\n",
			  __func__, ext_cmd->cmd, ext_cmd->flags,
			  ext_cmd->arg_size);
		err = -EINVAL;
		break;
	}

	return err;
}

static int cyccg_interact_with_exteranl_ap(struct cyccg *cyccg,
					   unsigned long arg,
					   int compat_mode)
{
	struct cyccg_ext_ap_cmd ext_cmd;
	void *ext_cmd_arg = NULL;
	int err;

	/* Copy command data structure. */
	if (copy_from_user(&ext_cmd,
			   (struct cyccg_ext_ap_cmd __user *)arg,
			   sizeof(struct cyccg_ext_ap_cmd))) {
		cyccg_dbg("%s: failed to copy ext_cmd data from user mode\n",
			  __func__);
		return -EFAULT;
	}
	/* Copy command argument data if exists. */
	if (ext_cmd.arg_size) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
#ifdef CONFIG_COMPAT
		if (compat_mode)
			ext_cmd.arg = (u8 __user *)compat_ptr(
						(unsigned long)ext_cmd.arg);
#endif
#endif
		cyccg_dbg("%s: copy ext_cmd.arg data from user mode\n",
			  __func__);
		ext_cmd_arg = kmalloc(ext_cmd.arg_size, GFP_KERNEL);
		if (!ext_cmd_arg)
			return -ENOMEM;

		if (copy_from_user(ext_cmd_arg,
				   ext_cmd.arg, ext_cmd.arg_size)) {
			err = -EFAULT;
			goto out;
		}
	}

	atomic_inc(&cyccg->is_irq_owned_by_drv);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		enable_irq(cyccg->client->irq);
#endif

	err = cyccg_swap_cmd_process(cyccg, &ext_cmd, ext_cmd_arg);

#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		disable_irq(cyccg->client->irq);
#endif
	atomic_dec(&cyccg->is_irq_owned_by_drv);
out:
	kfree(ext_cmd_arg);
	return err;
}

static long cyccg_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg, int compat_mode)
{
	struct cyccg *cyccg = file->private_data;
	struct i2c_client *client = cyccg->client;
	struct cyccg_i2c_info i2c_info;
	int err = 0;

	if (!cyccg)
		return -EINVAL;

	mutex_lock(&cyccg->lock);
	if (!cyccg->open) {
		err = -EINVAL;
		goto out;
	}

	switch (cmd) {
	case CYCCG_I2C_READ:
	case CYCCG_I2C_WRITE:
		err = cyccg_i2c_rw(cyccg, cmd, arg, compat_mode);
		break;
	case CYCCG_GET_I2C_INFO:
		i2c_info.bus = client->adapter->nr;
		i2c_info.addr = client->addr;
		snprintf(i2c_info.name, CYCCG_MAX_NAME_SIZE, client->name);
		if (copy_to_user((void __user *)arg,
				  &i2c_info,
				  sizeof(i2c_info))) {
			err = -EFAULT;
			goto out;
		}
		break;
	case CYCCG_CLEAR_IRNT:
		if (atomic_cmpxchg(&cyccg->sigio_queued, 1, 0))
			complete(&cyccg->ioctl_clear_intr);
		else
			err = cyccg_hpi_clear_intr(cyccg);
		break;
	case CYCCG_EXT_AP_CTRL:
		err = cyccg_interact_with_exteranl_ap(cyccg, arg, compat_mode);
		break;
	default:
		err = -EINVAL;
		break;
	}

out:
	mutex_unlock(&cyccg->lock);
	return err;
}

static long cyccg_unlocked_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	return cyccg_ioctl(file, cmd, arg, 0);
}

#ifdef CONFIG_COMPAT
static long cyccg_compat_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	return cyccg_ioctl(file, cmd, (unsigned long)compat_ptr(arg), 1);
}
#endif

static const struct file_operations cyccg_fops = {
	.owner = THIS_MODULE,
	.open = cyccg_open,
	.release = cyccg_close,
	.unlocked_ioctl = cyccg_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cyccg_compat_ioctl,
#endif
	.fasync = cyccg_fasync,
};

static void cyccg_msg_queue_init(struct cyccg *cyccg)
{
	mutex_init(&cyccg->msg_queue_lock);
	cyccg->enable_msg_queue = false;
	memset(cyccg->msg_queue, 0,
	       sizeof(struct hpi_msg) * CYCCG_MAX_MSG_QUEUE);
	cyccg->enqueue_index = 0;
	cyccg->dequeue_index = 0;
}

static bool _cyccg_is_msg_queue_empty(struct cyccg *cyccg)
{
	if (cyccg->enqueue_index == cyccg->dequeue_index)
		return true;
	return false;
}

#if 0  /* Suppress compile warnning, not used yet. */
static bool cyccg_is_msg_queue_empty(struct cyccg *cyccg)
{
	bool is_empty;

	mutex_lock(&cyccg->msg_queue_lock);
	is_empty = _cyccg_is_msg_queue_empty(cyccg);
	mutex_unlock(&cyccg->msg_queue_lock);
	return is_empty;
}
#endif

static bool _cyccg_is_msg_queue_full(struct cyccg *cyccg)
{
	int index = (cyccg->enqueue_index + 1) % CYCCG_MAX_MSG_QUEUE;

	if (index == cyccg->dequeue_index)
		return true;
	return false;
}

#if 0 /* Suppress compile warnning, not used yet. */
static bool cyccg_is_msg_queue_full(struct cyccg *cyccg)
{
	bool is_full;

	mutex_lock(&cyccg->msg_queue_lock);
	is_full = _cyccg_is_msg_queue_full(cyccg);
	mutex_unlock(&cyccg->msg_queue_lock);
	return is_full;
}
#endif

static void cyccg_enable_msg_queue(struct cyccg *cyccg)
{
	mutex_lock(&cyccg->msg_queue_lock);
	cyccg->enable_msg_queue = true;
	cyccg->enqueue_index = 0;
	cyccg->dequeue_index = 0;
	mutex_unlock(&cyccg->msg_queue_lock);
}

static void cyccg_disable_msg_queue(struct cyccg *cyccg)
{
	mutex_lock(&cyccg->msg_queue_lock);
	cyccg->enable_msg_queue = false;
	cyccg->enqueue_index = 0;
	cyccg->dequeue_index = 0;
	mutex_unlock(&cyccg->msg_queue_lock);
}

#if 0
static struct hpi_msg *_cyccg_get_enqueue_msg_buf(struct cyccg *cyccg)
{
	int index = 0;

	if (!cyccg->enable_msg_queue)
		return NULL;
	if (_cyccg_is_msg_queue_full(cyccg))
		return NULL;

	index = cyccg->enqueue_index++;
	cyccg->enqueue_index %= CYCCG_MAX_MSG_QUEUE;
	return &cyccg->msg_queue[index];
}
#endif

static int cyccg_enqueue_msg(struct cyccg *cyccg, struct hpi_msg *msg)
{
	int err = 0;

	mutex_lock(&cyccg->msg_queue_lock);
	if (!cyccg->enable_msg_queue) {
		err = -EINVAL;
		goto out;
	}
	if (_cyccg_is_msg_queue_full(cyccg)) {
		err = -ENOMEM;
		goto out;
	}

	if (msg->head.len > CY_PD_MSG_MAX_DATA_SIZE)
		msg->head.len = CY_PD_MSG_MAX_DATA_SIZE;
	memcpy(&cyccg->msg_queue[cyccg->enqueue_index++],
	       msg, msg->head.len + 2);
	cyccg->enqueue_index %= CYCCG_MAX_MSG_QUEUE;

out:
	mutex_unlock(&cyccg->msg_queue_lock);
	return err;
}

static int cyccg_dequeue_msg(struct cyccg *cyccg, struct hpi_msg *msg)
{
	struct hpi_msg *msg_buf;
	int err = 0;

	mutex_lock(&cyccg->msg_queue_lock);
	if (!cyccg->enable_msg_queue) {
		err = -EINVAL;
		goto out;
	}
	if (_cyccg_is_msg_queue_empty(cyccg)) {
		err = -ENODATA;
		goto out;
	}

	msg_buf = &cyccg->msg_queue[cyccg->dequeue_index++];
	if (msg_buf->head.len > CY_PD_MSG_MAX_DATA_SIZE)
		msg_buf->head.len = CY_PD_MSG_MAX_DATA_SIZE;
	memcpy(msg, msg_buf, msg_buf->head.len + 2);
	cyccg->dequeue_index %= CYCCG_MAX_MSG_QUEUE;

out:
	mutex_unlock(&cyccg->msg_queue_lock);
	return err;
}

static void cyccg_version_parse(struct ccg_version *ver, u8 *buf)
{
	memset(ver, 0, sizeof(struct ccg_version));
	ver->bl_maj_ver = (buf[3] >> 4) & 0x0f;
	ver->bl_min_ver = buf[3] & 0x0f;
	ver->bl_patch_ver = buf[2];
	ver->bl_build_num = get_unaligned_le16(&buf[0]);
	ver->app_maj_ver = (buf[7] >> 4) & 0x0f;
	ver->app_min_ver = buf[7] & 0x0f;
	ver->app_ext_ver = buf[6];
	ver->app_name[0] = buf[5];
	ver->app_name[1] = buf[4];
}

static int cyccg_read_all_version(struct cyccg *cyccg)
{
	struct device *dev = &cyccg->client->dev;
	struct ccg_version *bl_ver = &cyccg->bl_ver;
	struct ccg_version *app_ver = &cyccg->app_ver;
	u8 buf[16];
	int err;

	err = cyccg_i2c_read(cyccg, buf, sizeof(buf), CY_PD_GET_VERSION);
	if (err) {
		dev_err(dev, "failed to read all version.\n");
		return err;
	}

	cyccg_version_parse(bl_ver, &buf[0]);
	cyccg_version_parse(app_ver, &buf[8]);

	err = cyccg_i2c_read(cyccg, buf, 2, CY_PD_SILICON_ID);
	if (err) {
		dev_err(dev, "failed to read silicon id.\n");
		return err;
	}
	cyccg->silicon_id  = get_unaligned_le16(&buf[0]);

	cyccg_dbg("%s: bl FW version: %d.%d.%d.%d\n",
		__func__,
		cyccg->bl_ver.bl_maj_ver, cyccg->bl_ver.bl_min_ver,
		cyccg->bl_ver.bl_patch_ver, cyccg->bl_ver.bl_build_num);
	cyccg_dbg("%s: app FW version: %d.%d,%d,%c%c\n",
		__func__,
		cyccg->app_ver.app_maj_ver, cyccg->app_ver.app_min_ver,
		cyccg->app_ver.app_ext_ver, cyccg->app_ver.app_name[0],
		cyccg->app_ver.app_name[1]);

	return 0;
}

static unsigned long get_bits_value(unsigned char *arr,
				    int bit_offset, int bit_num)
{
#define CHAR_BIT 8

	int byte_offset = bit_offset / CHAR_BIT;
	int shift = bit_offset % CHAR_BIT;
	unsigned long data = (unsigned long)*(unsigned long *)&arr[byte_offset];
	unsigned long mask = 0;
	int i;

	for (i = 0; i < bit_num; i++) {
		mask <<= 1;
		mask |= 1;
	}

	data >>= shift;
	return (unsigned long)(data & mask);
}

#if CYCCG_DEBUG
static char dev_type[][24] = {
	"NOTHING",
	"Sink",
	"Source",
	"DEBUG_ACCESSORY",
	"AUDIO_ACCESSORY",
	"POWERED_ACCESSORY",
	"UNSUPPORTED_ACCESSORY",
};

static char type_c_current_level[][8] = {
	"Default",
	"1.5A",
	"3A",
	"Unknown",
};

static void dump_ccg_status(struct ccg_status *ccg_status)
{
	cyccg_dbg("%s: TYPE_C_STATUS (0x%02x):\n", __func__,
		  ccg_status->type_c_status);
	cyccg_dbg("%s:	 port_connected: %s\n", __func__,
		  ccg_status->port_connected ? "Yes" : "No");
	if (ccg_status->port_connected) {
		cyccg_dbg("%s:	 attached_dev_type: %s\n", __func__,
			  dev_type[ccg_status->attached_dev_type]);
		cyccg_dbg("%s:	 port connected to %s\n", __func__,
			  (ccg_status->CC_polarity ==
				CCG_STATUS_CC_POLARITY_CC1) ?
			  "CC1" : "CC2");
		cyccg_dbg("%s:	 type_c_current_level: %s\n", __func__,
			  type_c_current_level[
				ccg_status->type_c_current_level]);
		cyccg_dbg("%s:	 Ra_status: Ra %s\n", __func__,
			  ccg_status->Ra_status ? "detected" : "not detected");
	}
	cyccg_dbg("%s: PD_STATUS (0x%08x):\n", __func__, ccg_status->pd_status);
	cyccg_dbg("%s:	 Contract Established: %s\n", __func__,
		  ccg_status->contract_status ? "Yes" : "No");
	if (ccg_status->contract_status) {
		cyccg_dbg("%s:	 current_port_power_role: %s\n", __func__,
			  ccg_status->current_port_power_role ?
			  "Source" : "Sink");
		cyccg_dbg("%s:	 current_port_data_role: %s\n", __func__,
			  ccg_status->current_port_data_role ? "DFP" : "UFP");
	}
	cyccg_dbg("%s:	 EMCA_present: %s\n", __func__,
		  ccg_status->EMCA_present ? "Yes" : "No");
	cyccg_dbg("%s:	 VCONN_supplier: %s\n", __func__,
		  ccg_status->VCONN_supplier ? "Yes" : "No");
	cyccg_dbg("%s:	 VCONN_status: %s\n", __func__,
		  ccg_status->VCONN_status ? "Souring" : "Not Souring");
}
#endif

static int cyccg_get_ccg_status(struct cyccg *cyccg,
				struct ccg_status *ccg_status)
{
	struct ccg_status tmp_ccg_status;
	unsigned char data[4];
	int err;

	memset(&tmp_ccg_status, 0, sizeof(struct ccg_status));

	/* Read PD_Status */
	err = cyccg_i2c_read(cyccg, (u8 *)data, 4, CY_PD_REG_PD_STATUS);
	if (err)
		return err;
	memcpy(&tmp_ccg_status.pd_status, (u8 *)data, 4);

	/*
	 * Current Port Data Role and Current Port Power Role fields are valid
	 * only if Contract State bit is set.
	 */
	tmp_ccg_status.contract_status = get_bits_value(data, 10, 1);
	if (tmp_ccg_status.contract_status) {
		tmp_ccg_status.current_port_data_role =
				get_bits_value(data, 6, 1);
		tmp_ccg_status.current_port_power_role =
				get_bits_value(data, 8, 1);
	}

	tmp_ccg_status.EMCA_present = get_bits_value(data, 11, 1);

	tmp_ccg_status.VCONN_supplier = get_bits_value(data, 12, 1);
	tmp_ccg_status.VCONN_status = get_bits_value(data, 13, 1);

	/* Default PD status */
	tmp_ccg_status.default_port_data_role = get_bits_value(data, 0, 2);
	tmp_ccg_status.default_port_data_role_in_case_of_drp =
			get_bits_value(data, 2, 1);
	tmp_ccg_status.default_port_power_role = get_bits_value(data, 3, 2);
	tmp_ccg_status.default_port_power_role_in_case_of_dual_role =
			get_bits_value(data, 5, 1);

	/* Read Type-C Status */
	err = cyccg_i2c_read(cyccg, (u8 *)data, 1, CY_PD_REG_TYPE_C_STATUS);
	if (err)
		return err;
	memcpy(&tmp_ccg_status.type_c_status, (u8 *)data, 1);

	tmp_ccg_status.port_connected = get_bits_value(data, 0, 1);
	tmp_ccg_status.CC_polarity = get_bits_value(data, 1, 1);
	tmp_ccg_status.attached_dev_type = get_bits_value(data, 2, 3);
	if (tmp_ccg_status.current_port_data_role)
		tmp_ccg_status.Ra_status = get_bits_value(data, 5, 1);
	tmp_ccg_status.type_c_current_level = get_bits_value(data, 6, 2);

	/* The CCG status is valid. */
	tmp_ccg_status.valid = 1;

	if (ccg_status)
		memcpy(ccg_status, &tmp_ccg_status, sizeof(struct ccg_status));

#if CYCCG_DEBUG
	dump_ccg_status(&tmp_ccg_status);
#endif
	return 0;
}

static bool is_cyccg_drv_internal_processing(struct cyccg *cyccg)
{
	if (atomic_read(&cyccg->is_irq_owned_by_drv))
		return true;
	return false;
}

static int cyccg_hpi_clear_intr(struct cyccg *cyccg)
{
	u8 clear_intr = CY_PD_REG_INTR_REG_CLEAR_RQT;
	int err;

	err = cyccg_i2c_write(cyccg, &clear_intr, 1, CY_PD_REG_INTR_REG_ADDR);
	cyccg_dbg_irq("%s: write INTR_REG_CLEAR_RQT, (%d)\n", __func__, err);

	return err;
}

static int cyccg_port_disable(struct cyccg *cyccg)
{
	struct hpi_msg cmd_resp;
	u8 cmd[] = { CY_PD_EC_DISABLE };
	u8 device_mode;
	int err;

	/*
	 * Skip the the port disable command when in bootloader mode to avoid
	 * the unwanted 0x05 response error code.
	 */
	err = cyccg_i2c_read(cyccg, &device_mode, 1,
			     CY_PD_REG_DEVICE_MODE_ADDR);
	if (err || device_mode == 0)
		return err;

	/*
	 * Do not need to check the return code, because the HPI interface
	 * previous v13 do not access any interrupt to host and no response
	 * code. HPI v13 has the interrupt and responce code.
	 */
	cyccg_hpi_cmd_sync(cyccg, cmd, 1, CY_PD_REG_PD_CONTROL,
			   &cmd_resp, NULL, 100);

	/*
	 * Depending on the HPI spec, starting from HPI spec v13, the new
	 * firmware will always on respond with response code to host when
	 * received the disable port command, so need to read out and clear
	 * the response event.
	 * Note, before v13 HPI spec, no response event will report to host
	 * when received same command.
	 */
	cyccg_dump_pending_events(cyccg);
	return err;
}

static int cyccg_hpi_cmd_sync(struct cyccg *cyccg,
		u8 *cmd, int cmd_len, int cmd_addr,
		struct hpi_msg *cmd_resp, u8 *excepted_events,
		int timeout)
{
	int count;
	int err;

	if (!cmd || !cmd_len || !cmd_resp || !timeout)
		return -EINVAL;

	mutex_lock(&cyccg->cmd_lock);
	cyccg_enable_msg_queue(cyccg);

	init_completion(&cyccg->resp_ready);
	cyccg->resp_msg = cmd_resp;
	cyccg->excepted_events = excepted_events;
	atomic_inc(&cyccg->cmd_issued);

	err = cyccg_i2c_write(cyccg, cmd, cmd_len, cmd_addr);
	if (err) {
		atomic_dec(&cyccg->cmd_issued);
		goto err;
	}

	timeout = wait_for_completion_timeout(&cyccg->resp_ready,
					      msecs_to_jiffies(timeout));
	if (timeout == 0) {
		atomic_cmpxchg(&cyccg->cmd_issued, 1, 0);
		cyccg_dbg_irq("%s: command wait timeout.\n", __func__);

		if (cyccg->cmd_wait_internal)
			count = (cyccg->cmd_max_timeout /
				 cyccg->cmd_wait_internal) + 1;
		else
			count = 0;
		do {
			err = cyccg_hpi_poll_events(cyccg,
						    cmd_resp, excepted_events);
			if (!err)
				break;

			err = -ETIMEDOUT;
			msleep(cyccg->cmd_wait_internal);
		} while (--count > 0);
	}

err:
	cyccg_disable_msg_queue(cyccg);
	cyccg->excepted_events = NULL;
	cyccg->resp_msg = NULL;
	cyccg_dbg_irq("%s: cmd_issued = %d\n", __func__,
		      atomic_read(&cyccg->cmd_issued));
	mutex_unlock(&cyccg->cmd_lock);
	return err;
}

static int cyccg_hpi_wait_events_sync_no_lock(struct cyccg *cyccg,
		u8 *events, struct hpi_msg *event_resp,
		int timeout)
{
	int err = 0;

	if (!events || !event_resp || !timeout)
		return -EINVAL;

	init_completion(&cyccg->resp_ready);
	cyccg->resp_msg = event_resp;
	cyccg->excepted_events = events;
	atomic_inc(&cyccg->cmd_issued);

	timeout = wait_for_completion_timeout(&cyccg->resp_ready,
					      msecs_to_jiffies(timeout));
	if (timeout == 0) {
		atomic_cmpxchg(&cyccg->cmd_issued, 1, 0);

		err = cyccg_hpi_poll_events(cyccg, event_resp, events);
		if (err)
			err = -ETIMEDOUT;
	}

	cyccg->excepted_events = NULL;
	cyccg->resp_msg = NULL;
	cyccg_dbg_irq("%s: cmd_issued = %d\n", __func__,
		      atomic_read(&cyccg->cmd_issued));

	return err;
}

#if 0  /* TODO(DUDL): Suppress compile warnning for not used yet. */
static int cyccg_hpi_wait_events_sync(struct cyccg *cyccg,
		u8 *events, struct hpi_msg *event_resp,
		int timeout)
{
	int err = 0;

	if (!events || !event_resp || !timeout)
		return -EINVAL;

	mutex_lock(&cyccg->cmd_lock);
	cyccg_enable_msg_queue(cyccg);
	err = cyccg_hpi_wait_events_sync_no_lock(cyccg, events,
						 event_resp, timeout);

	cyccg_disable_msg_queue(cyccg);
	mutex_unlock(&cyccg->cmd_lock);
	return err;
}

/*
 * This function checks if there was a response/event pending for processing.
 * return value > 0, means there is respose data needs to be processed.
 * return value == 0, means no respose data exists.
 * return value < 0, indicated the error code of the failure it encounterred.
 */
static int cyccg_hpi_pending_resp(struct cyccg *cyccg)
{
	struct hpi_msg_head msg_head;
	int err;

	err = cyccg_i2c_read(cyccg, (u8 *)&msg_head, sizeof(msg_head),
			     CY_PD_REG_RESPONSE_ADDR);
	if (err)
		return err;

	cyccg_dbg("%s: pending event: code=%#02x, len=%d\n",
		  __func__, msg_head.code, msg_head.len);

	if (msg_head.code == CY_PD_RESP_NO_RESPONSE)
		return 0;

	return (int)msg_head.code;
}
#endif

static bool cyccg_excepted_events_match(struct cyccg *cyccg, u8 code)
{
	int i = 0;

	if (!cyccg->excepted_events)
		return false;

	/*
	 * The excepted code value in @excepted_events must be bigger than 0x7F,
	 * and the last element of the array must be 0 which indicates the
	 * end of the array.
	 */
	while (cyccg->excepted_events[i] != 0) {
		if (cyccg->excepted_events[i++] == code)
			return true;
	}

	return false;
}

/*
 * return value:
 *   0   means in bootloader mode;
 *   > 0 means in operational mode;
 *   < 0 means encounter error, and the return value is the error code.
 */
static int cyccg_hpi_get_device_mode(struct cyccg *cyccg)
{
	int err;
	u8 data[1];

	err = cyccg_i2c_read(cyccg, data, 1, CY_PD_REG_DEVICE_MODE_ADDR);
	if (err)
		return err;

	if (data[0] == 0x01)
		return 1;
	else if (data[0] == 0x00)
		return 0;
	else
		return -EINVAL;
}


/* Try to poll the response message after timeout. */
static int cyccg_hpi_poll_resp_msg(struct cyccg *cyccg, struct hpi_msg *resp)
{
	int err, ret;

	memset(&resp->head, 0, sizeof(struct hpi_msg_head));
	err = cyccg_i2c_read(cyccg, (u8 *)&resp->head,
		     sizeof(struct hpi_msg_head),
		     CY_PD_REG_RESPONSE_ADDR);
	if (err) {
		cyccg_dbg("%s: failed to read response head, (%d)\n", __func__,
			  err);
		return err;
	}

	cyccg_dbg("%s: code = %#02x, len = %d\n", __func__,
		  resp->head.code, resp->head.len);

	if (resp->head.code == CY_PD_RESP_NO_RESPONSE)
		return 0;

	if (resp->head.len > CY_PD_MSG_MAX_DATA_SIZE)
		resp->head.len = CY_PD_MSG_MAX_DATA_SIZE;
	if (resp->head.len)
		err = cyccg_i2c_read(cyccg,
				     resp->data,
				     resp->head.len,
				     CY_PD_REG_BOOTDATA_MEMEORY_ADDR);

	ret = cyccg_hpi_clear_intr(cyccg);
	if (ret)
		return ret;
	return err;
}

/* Try to poll the response message after timeout. */
static int cyccg_hpi_poll_events(struct cyccg *cyccg, struct hpi_msg *resp,
		u8 *excepted_events)
{
	int i;
	int err;

	/* Must wait to enter to process the queued message at once. */
	do {
		mutex_lock(&cyccg->irq_work_lock);
		if (cyccg->irq_work_state == CYCCG_WORK_RUNNING) {
			mutex_unlock(&cyccg->irq_work_lock);
			msleep(1);
			continue;
		}
		cyccg->irq_work_state = CYCCG_WORK_RUNNING;
		mutex_unlock(&cyccg->irq_work_lock);
		break;
	} while (1);

	while (1) {
		err = cyccg_dequeue_msg(cyccg, resp);
		if (err) {
			err = cyccg_hpi_poll_resp_msg(cyccg, resp);
			if (err)
				goto out;
		}

		cyccg_dbg("%s: code = %#02x, len= %d\n", __func__,
			resp->head.code, resp->head.len);

		/* No excepted event founnd, return error. */
		if (resp->head.code == CY_PD_RESP_NO_RESPONSE) {
			err = -EAGAIN;
			goto out;
		}

		/* Comamnd response event. */
		if (IS_CY_PD_CMD_RESP_MSG(&resp->head))
			break;

		/*
		 * If no except event specified, consider it as read correct
		 * event.
		 */
		if (!excepted_events)
			break;

		/* Any expcted event found, return as success. */
		i = 0;
		while (excepted_events[i] != 0) {
			if (excepted_events[i++] == resp->head.code)
				goto out;
		}
	}

out:
	mutex_lock(&cyccg->irq_work_lock);
	cyccg->irq_work_state = CYCCG_WORK_NONE;
	mutex_unlock(&cyccg->irq_work_lock);
	return err;
}

static void cyccg_dump_pending_events_no_sync(struct cyccg *cyccg)
{
	struct hpi_msg resp_msg;
	int count = 0;

	cyccg_dbg("%s:\n", __func__);
	do {
		if (cyccg_hpi_poll_resp_msg(cyccg, &resp_msg))
			return;
		/*
		 * Avoid one CY_PD_RESP_NO_RESPONSE message existing in the
		 * internal of the message queue, which will cause the message
		 * only be partially dumpped.
		 */
		if (resp_msg.head.code == CY_PD_RESP_NO_RESPONSE)
			count++;
		else
			count = 0;
	} while (count < 2);
}
static void cyccg_dump_pending_events(struct cyccg *cyccg)
{
	do {
		mutex_lock(&cyccg->irq_work_lock);
		if (cyccg->irq_work_state == CYCCG_WORK_RUNNING) {
			mutex_unlock(&cyccg->irq_work_lock);
			msleep(1);
			continue;
		}
		cyccg->irq_work_state = CYCCG_WORK_RUNNING;
		mutex_unlock(&cyccg->irq_work_lock);
		break;
	} while (1);

	cyccg_dump_pending_events_no_sync(cyccg);

	mutex_lock(&cyccg->irq_work_lock);
	cyccg->irq_work_state = CYCCG_WORK_NONE;
	mutex_unlock(&cyccg->irq_work_lock);
}

#if 0
static int cyccg_hpi_get_pd_status(struct cyccg *cyccg,
			    u8 *status_buf, int buf_len)
{
	int err;

	if (!status_buf || buf_len < 4)
		return -EINVAL;

	err = cyccg_i2c_read(cyccg, status_buf, 4, CY_PD_REG_PD_STATUS);
	if (err)
		return err;

	return 0;
}
#endif

#if CYCCG_CCG_SUPPORT_SWAP_COMPLETE
static int cyccg_hpi_get_swap_status(struct cyccg *cyccg,
			    u8 *status_buf, int buf_len)
{
	int err;

	if (!status_buf || buf_len < 1)
		return -EINVAL;

	err = cyccg_i2c_read(cyccg, status_buf, 1,
				CY_PD_REG_BOOTDATA_MEMEORY_ADDR);
	if (err)
		return err;

	return 0;
}
#endif

static int cyccg_hpi_pd_control(struct cyccg *cyccg, u8 cmd)
{
	struct hpi_msg resp_msg;
	int err;

	cyccg_dbg("%s: send PD_CONTORL comamnd: 0x%02x\n", __func__, cmd);
	err = cyccg_hpi_cmd_sync(cyccg, &cmd, 1, CY_PD_REG_PD_CONTROL,
				 &resp_msg, NULL, 200);
	if (err || resp_msg.head.code != CY_PD_RESP_SUCCESS) {
		cyccg_dbg("%s: %s: 0x%02x: code: 0x%02x, (%d)\n",
			  __func__, "failed to execute PD_CONTROL command",
			  cmd, resp_msg.head.code, err);
		return err ? err : -EINVAL;
	}

	return 0;
}
static void cyccg_ext_evt_cmd_complete(struct cyccg *cyccg, int errcode)
{
	mutex_lock(&cyccg->ext_evt_lock);
	if (cyccg->ext_evt_waiting) {
		cyccg_dbg("%s: wait ext event reason: 0x%02x, errcode: %d\n",
			  __func__, cyccg->ext_evt_reason, errcode);

		cyccg->ext_evt_cmd_err_code = errcode;
		complete(&cyccg->ext_evt_done);
	}
	mutex_unlock(&cyccg->ext_evt_lock);
}
static int cyccg_hpi_swap_cmd_send(struct cyccg *cyccg, u8 swap_cmd)
{
	int timeout;
	int err;

	cyccg_dbg("%s: <<<<\n", __func__);
	cyccg_dump_pending_events(cyccg);

	err = mutex_lock_interruptible(&cyccg->ext_evt_lock);
	if (err)
		return err;
	if (cyccg->ext_evt_waiting) {
		mutex_unlock(&cyccg->ext_evt_lock);
		cyccg_dbg("%s: -EBUSY: ext_evt_reason = 0x%02x\n",
			  __func__, cyccg->ext_evt_reason);
		return -EBUSY;
	}
	cyccg->ext_evt_cmd_err_code = 0;
	cyccg->ext_evt_waiting = true;
	cyccg->ext_evt_reason = swap_cmd;
	cyccg->is_swap_initiated_by_partner = false;
	cyccg->is_ap_synced = false;
	mutex_unlock(&cyccg->ext_evt_lock);

	init_completion(&cyccg->ext_evt_done);

	/* Send power role swap command to CCG device. */
	err = cyccg_hpi_pd_control(cyccg, swap_cmd);
	if (err)
		goto out;

	/* Monitoring the CCG device continue process events. */
	timeout = 2000;  /* ms */
	timeout = wait_for_completion_timeout(&cyccg->ext_evt_done,
					      msecs_to_jiffies(timeout));
	if (timeout == 0) {
		cyccg_dbg("%s: wait for swap comamnd done timeout.\n",
			  __func__);
		err = -ETIMEDOUT;
	}

out:
	mutex_lock(&cyccg->ext_evt_lock);
	err = err ? err : cyccg->ext_evt_cmd_err_code;
	cyccg_dbg("%s: swap_cmd: 0x%02x, resp result: %d\n", __func__,
		  swap_cmd, err);

	cyccg->ext_evt_reason = 0;
	cyccg->ext_evt_cmd_err_code = 0;
	cyccg->ext_evt_waiting = false;
	mutex_unlock(&cyccg->ext_evt_lock);
	cyccg_dbg("%s: >>>>: err = %d\n", __func__, err);

#if CYCCG_POWER_CONTROLLED_BY_EXTERNAL_AP
	/* Encounter error, so required to re-sync with AP. */
	if (err)
		cyccg_update_external_ap_status(cyccg, 0);
#endif
	return err;
}

static int cyccg_hpi_reset(struct cyccg *cyccg)
{
	u8 reset_cmd[] = { CY_PD_DEVICE_RESET_CMD_SIG,
			   CY_PD_REG_RESET_DEVICE_CMD };
	u8 excepted_events[] = { CY_PD_RESP_RESET_COMPLETE, 0 };
	struct hpi_msg event_resp;
	u8 data[1];
	int timeout;
	int err;

	cyccg_dbg("%s: >>>>\n", __func__);
	cyccg_port_disable(cyccg);

	mutex_lock(&cyccg->cmd_lock);
	cyccg_enable_msg_queue(cyccg);

	init_completion(&cyccg->resp_ready);
	cyccg->resp_msg = &event_resp;
	cyccg->excepted_events = excepted_events;
	atomic_inc(&cyccg->cmd_issued);

	err = cyccg_i2c_write(cyccg, reset_cmd, sizeof(reset_cmd),
			      CY_PD_REG_RESET_ADDR);
	if (err) {
		atomic_dec(&cyccg->cmd_issued);
		cyccg_dbg("%s: I2C write error, (%d)\n",
			__func__, err);
		goto out;
	}

	/* Wait the RESET_COMPLETE when enter bootloader mode. */
	timeout = 200;  /* ms */
	timeout = wait_for_completion_timeout(&cyccg->resp_ready,
					      msecs_to_jiffies(timeout));
	if (timeout == 0) {
		atomic_cmpxchg(&cyccg->cmd_issued, 1, 0);

		err = cyccg_hpi_poll_events(cyccg,
					    &event_resp, excepted_events);
		if (err) {
			err = -ETIMEDOUT;
			cyccg_dbg("%s: BL RESET_COMPLETE timeout, (%d)\n",
				  __func__, err);
			goto dev_mode_check;
		}
	}
	if (event_resp.head.code != CY_PD_RESP_RESET_COMPLETE) {
		/*
		 * When the CCG2 device's hard reset process is in progress,
		 * the soft reset command sent to the CCG2 device will
		 * immediately respond with a 0x02 success response code,
		 * and after the hard reset finished, the soft reset will
		 * not be executed again, so no 0x80 events after the 0x02
		 * event. So to cover this case, we need to double check
		 * the device mode of the CCG2 device when 0x02 event is
		 * received after sent the soft reset commmand.
		 * If it has enterred application mode, the CCG2 device should
		 * have reset hard reset and working correctly.
		 * So continue the process.
		 */
		if (event_resp.head.code == CY_PD_RESP_SUCCESS)
			goto dev_mode_check;

		dev_err(&cyccg->client->dev,
			"failed to do device reset, resp code: (0x%02x)\n",
			event_resp.head.code);
		err = -EIO;
		goto out;
	}
	if (timeout) {
		/*
		 * Remove the enqueued message when it's process in normal irq
		 * command process to avoid the event_resp data is reused
		 * by next wait timeout.
		 */
		cyccg_dequeue_msg(cyccg, &event_resp);
	}

dev_mode_check:
	err = cyccg_hpi_get_device_mode(cyccg);
	if (err > 0) {
		/* Reset success. */
		err = 0;
		cyccg_dbg("%s: reset success, return early.\n", __func__);
		goto out;
	} else if (err < 0) {
		/* Unabled to get device mode. */
		cyccg_dbg("%s: device mode read error, (%d)\n", __func__, err);
		goto wait;
	}

	/*
	 * Read bootloader reason.
	 * bit0: 0-Bo boot mode request; 1-JUMP_TO_BOOT request.
	 * bit1: 0-Table valid; 1-Table invalid.
	 * bit2: 0-Application image valid; 1-Application image invalid.
	 * bit3: Reserved, always 1.
	 * bit4-7: Rserved, always 0.
	 */
	err = cyccg_i2c_read(cyccg, data, 1, CY_PD_BOOT_MODE_REASON);
	if (err) {
		cyccg_dbg("%s: failed to read BL mode reason register, (%d)\n",
			  __func__, err);
		goto wait;
	}
	if ((data[0] & 0xf0) || !(data[0] & 0x08)) {
		/*
		 * The BL_MODE_REASON data read is invalid, may be caused by
		 * the CCGx device was still in resetting, reset has been fully
		 * done yet. So the I2C register map data still in invalid
		 * state. But unfortunately, it was read out.
		 * So we must wait for the CCGx becomes fully ready, here, 15ms
		 * fully enough. Because for CCGx the max time for validate
		 * flash contents is only 11ms.
		 */
		usleep_range(15000, 20000);
		goto wait;
	} else if (data[0] & 0x07) {
		if (data[0] & 0x01)
			cyccg_dbg("%s: JUMP_TO_BOOT request\n", __func__);
		if (data[0] & 0x02)
			cyccg_dbg("%s: Table invalid\n", __func__);
		if (data[0] & 0x04)
			cyccg_dbg("%s: Application image invalid\n", __func__);

		cyccg_dbg("%s: bootloadter reason: BL_MOEE_REASON=%#02x\n",
			  __func__, data[0]);
		goto out;
	}

wait:
	/* Check if the waiting event has already captured. */
	while (!cyccg_dequeue_msg(cyccg, &event_resp)) {
		if (event_resp.head.code == CY_PD_RESP_RESET_COMPLETE &&
		    cyccg_hpi_get_device_mode(cyccg) > 0) {
			/*
			 * The application RESET_COMPLETE event captured,
			 * so reset completed successfully.
			 */
			err = 0;
			goto out;
		}
	}

	/* Wait the RESET_COMPLETE when entering operational mode. */
	cyccg_dbg("%s: waiting app enter reset complete event.\n", __func__);
	timeout = 200;
	err = cyccg_hpi_wait_events_sync_no_lock(cyccg, excepted_events,
						 &event_resp, timeout);
	if (err || event_resp.head.code != CY_PD_RESP_RESET_COMPLETE) {
		/* Double check if has already enterred operational mode. */
		if (cyccg_hpi_get_device_mode(cyccg) > 0) {
			err = 0;
		} else {
			err = err ? err : -EIO;
			dev_err(&cyccg->client->dev,
				"failed to do dev reset, code: 0x%02x, (%d)\n",
				event_resp.head.code, err);
		}
		goto out;
	}

out:
	cyccg_disable_msg_queue(cyccg);
	cyccg_dbg_irq("%s: cmd_issued = %d\n", __func__,
		      atomic_read(&cyccg->cmd_issued));
	mutex_unlock(&cyccg->cmd_lock);
	cyccg_dump_pending_events(cyccg);
	cyccg_dbg("%s: <<<<, (%d)\n", __func__, err);
	return err;
}

static int cyccg_hpi_initialize(struct cyccg *cyccg)
{
	struct hpi_msg resp_msg;
	u8 data[32];
	int timeout;
	int err;

	cyccg_dbg("%s: >>>>\n", __func__);

	/*
	 * Check device mode is in application mode, must wait at least 50 ms
	 * for device enter operational mode.
	 */
	for (timeout = 1000; timeout > 0; timeout -= 10) {
		err = cyccg_hpi_get_device_mode(cyccg);
		if (err > 0)
			break;

		/* Wait for BL mode timeout and enter operational mode. */
		usleep_range(10, 20);
	}
	if (err < 0) {
		dev_err(&cyccg->client->dev,
			"failed to read device mode status register.\n");
		return -EIO;
	} else if (err == 0) {
		err = cyccg_i2c_read(cyccg, data, 1, CY_PD_BOOT_MODE_REASON);
		if (err) {
			cyccg_dbg("%s: I2C read failed, (%d)\n",
				  __func__, err);
			return err;
		}

		if (data[0] & 0x01)
			dev_warn(&cyccg->client->dev,
				"device in BL mode, JUMP_TO_BOOT request.\n");
		if (data[0] & 0x02)
			dev_warn(&cyccg->client->dev,
				"device in BL mode, invalid table.\n");
		if (data[0] & 0x04)
			dev_warn(&cyccg->client->dev,
				"device in BL mode, invalid app image.\n");
		if (!(data[0] & 0x07))
			dev_warn(&cyccg->client->dev,
				"device in BL mode, unknown reason.\n");

		return -EINVAL;
	}

	cyccg_dump_pending_events(cyccg);

	cyccg_dbg("%s: >>>> Set EVENT MASK\n", __func__);
	/* Set EVENT MASK, enable all events notify host by default. */
	data[0] = 0xff;
	data[1] = 0xff;
	data[2] = 0xff;
	data[3] = 0xff;
	err = cyccg_hpi_cmd_sync(cyccg, data, 4, CY_PD_REG_EVENT_MASK,
				 &resp_msg, NULL, 200);
	if (err || resp_msg.head.code != CY_PD_RESP_SUCCESS) {
		dev_err(&cyccg->client->dev,
			"failed to set EVENT MASK, code: 0x%02x, (%d)\n",
			resp_msg.head.code, err);
		return -EIO;
	}

	cyccg_dbg("%s: >>>> Set SOURCE_PDO_MASK\n", __func__);
	/*
	 * Set SOURCE_PDO_MASK, enable only fixed 5V PDO by default.
	 * If there other default setting by default, just remove this part.
	 */
//letv_pd s  page 52   letv_pd_send_message_source_cap();
	data[0] = 0x01;
	err = cyccg_hpi_cmd_sync(cyccg, data, 1, CY_PD_REG_SELECT_SOURCE_PDO,
				 &resp_msg, NULL, 200);
	if (err || resp_msg.head.code != CY_PD_RESP_SUCCESS) {
		dev_err(&cyccg->client->dev,
			"failed to set SOURCE_PDO_MASK, code: 0x%02x, (%d)\n",
			resp_msg.head.code, err);
		return -EIO;
	}

	data[0] = 0x06;
	err = cyccg_hpi_cmd_sync(cyccg, data, 1, CY_PD_REG_SELECT_SINK_PDO,
				 &resp_msg, NULL, 200);
	if (err || resp_msg.head.code != CY_PD_RESP_SUCCESS) {
		dev_err(&cyccg->client->dev,
			"failed to set SELECT SINK PDO, code: 0x%02x, (%d)\n",
			resp_msg.head.code, err);
		return -EIO;
	}
//letv_pd e
	cyccg_dbg("%s: >>>> Send INIT_COMPLETE\n", __func__);
	/* Send PD control INIT_COMPLETE command. */
	data[0] = CY_PD_CONTROL_INIT_COMPLETE;
	err = cyccg_hpi_cmd_sync(cyccg, data, 1, CY_PD_REG_PD_CONTROL,
				 &resp_msg, NULL, 200);
	cyccg_dbg("%s: INIT_COMPLETE resp code: %#02x, (%d)\n",
		  __func__, resp_msg.head.code, err);
	if (err ||
	    (resp_msg.head.code != CY_PD_RESP_SUCCESS &&
	     resp_msg.head.code != CY_PD_RESP_PD_COMMAND_FAILED)) {
		/*
		 * Following HPI spec, INIT_COMPLETE should response with
		 * CY_PD_RESP_SUCCESS.
		 * And based on HPI spec, firmware will start a 100ms timer
		 * after the operational RESET_COMPLETE event, if the
		 * INIT_COMPLETE command not sent within the 100ms, CCG device
		 * will automatially enable the type-c port as received the
		 * INTI_COMPLETE command, so later INIT_COMPLETE commands after
		 * the 100ms timeout, firmware will all response with an event
		 * event CY_PD_RESP_PD_COMMAND_FAILED, this should be recoginzed
		 * as success. And on slow host system, it's normal the
		 * INIT_COMPLETE command is sent after the 100ms.
		 * That's why the CY_PD_RESP_PD_COMMAND_FAILED
		 * also traded as cuccess here.
		 */
		dev_err(&cyccg->client->dev,
			"PD INIT_COMPLETE failed, code: %#02x, (%d)\n",
			resp_msg.head.code, err);
		return -EIO;
	}

	cyccg_dbg("%s: INIT_COMPLETE successfully done.\n", __func__);
	return 0;
}

static int cyccg_device_init(struct cyccg *cyccg, bool force_reset)
{
	struct device *dev = &cyccg->client->dev;
	struct ccg_status ccg_status;
	int tries;
	int err;

	/* Reset the ccg status. */
	mutex_lock(&cyccg->ccg_status_lock);
	memset(&cyccg->ccg_status, 0, sizeof(struct ccg_status));
	mutex_unlock(&cyccg->ccg_status_lock);

	err = cyccg_get_ccg_status(cyccg, &ccg_status);
	if (err)
		goto out;

	/*
	 * If type-c has been connected, then do not do the reset to avoid
	 * the type-c disconnect and reconnect events.
	 * Only reset the CCG2 when there is no type-c connected.
	 */

	err = 0;
	if ((!ccg_status.port_connected) || force_reset){
		if(cyccg_soft_reset){
			err = cyccg_hpi_reset(cyccg);
		}
	}

	if (err) {
		dev_err(dev, "%s: error: cyccg_hpi_reset, (%d)\n",
			__func__, err);
	} else {
		/* Try to recovery unknown error in system boot. */
		tries = 3;
		do {
			err = cyccg_hpi_initialize(cyccg);
			if (!err) {
				break;
			}
			/* Wait CCGx becomes stable and ready. */
			msleep(500);
			continue;
		} while (--tries > 0);

		if (err)
			dev_err(dev, "%s: error: cyccg_hpi_initialize, (%d)\n",
				__func__, err);
	}

out:
	err = cyccg_read_all_version(cyccg);
	/*
	 * Always tries to let this driver loaded successfully when system
	 * booting, so can support firmware update if possible.
	 * Update the version information from device that stored in driver.
	 */
	mutex_lock(&cyccg->init_work_lock);
	if (cyccg->init_work_state == CYCCG_WORK_NONE)
		err = 0;
	mutex_unlock(&cyccg->init_work_lock);

	return err;
}

static int cyccg_init(struct cyccg *cyccg)
{
	int err;

	mutex_lock(&cyccg->lock);
	atomic_inc(&cyccg->is_irq_owned_by_drv);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		enable_irq(cyccg->client->irq);
#endif

	err = cyccg_device_init(cyccg, false);
	if (err)
		goto out;

out:
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		disable_irq(cyccg->client->irq);
#endif
	atomic_dec(&cyccg->is_irq_owned_by_drv);
	mutex_unlock(&cyccg->lock);
	return err;
}

static void cyccg_init_worker(struct work_struct *work)
{
	struct cyccg *cyccg = container_of(work, struct cyccg, init_work);
	int err;

	mutex_lock(&cyccg->init_work_lock);
	cyccg->init_work_state = CYCCG_WORK_RUNNING;
	mutex_unlock(&cyccg->init_work_lock);

	cyccg_dbg("%s: <<<<: start CCG reinitialize\n", __func__);
	err = cyccg_init(cyccg);
	cyccg_dbg("%s: >>>>: CCG reinitialize done, (%d)\n", __func__, err);

	mutex_lock(&cyccg->init_work_lock);
	cyccg->init_work_state = CYCCG_WORK_NONE;
	mutex_unlock(&cyccg->init_work_lock);
}
static void cyccg_queue_init_work(struct cyccg *cyccg)
{
	mutex_lock(&cyccg->init_work_lock);
	if (cyccg->init_work_state == CYCCG_WORK_NONE) {
		cyccg->init_work_state = CYCCG_WORK_QUEUED;
		schedule_work(&cyccg->init_work);

		cyccg_dbg("%s: device reinitialize queued.\n",
			  __func__);
	} else {
		cyccg_dbg("%s: current init_work_state = %d\n",
			  __func__, cyccg->init_work_state);
	}
	mutex_unlock(&cyccg->init_work_lock);
}
#if CYCCG_DEBUG
/* When the command send in IRQ thread, it must be executed in other thread. */
static void cyccg_vdm_worker(struct work_struct *work)
{
	struct cyccg *cyccg = container_of(work, struct cyccg, vdm_work);
	/* structured vdm sample. */
	u8 structured_vdm[] = { 0x41, 0x80, 0xd1, 0x12 };
	int structured_vdm_size = sizeof(structured_vdm);
	/* unstructured vdm sample. */
	u8 unstructured_vdm[] = { 0x4f, 0x12, 0x00, 0x00 };
	int unstructured_vdm_size = sizeof(unstructured_vdm);
	int err;

	mutex_lock(&cyccg->vdm_work_lock);
	cyccg->vdm_work_state = CYCCG_WORK_RUNNING;
	mutex_unlock(&cyccg->vdm_work_lock);

	cyccg_dbg("%s: <<<<: Send structured VMD data:\n", __func__);
	mem_dump(CYCCG_DEBUG, structured_vdm, structured_vdm_size);
	err = cyccg_send_vdm_data(cyccg, structured_vdm, structured_vdm_size);
	cyccg_dbg("%s: >>>>: result of send structured VDM data, err = %d\n",
		  __func__, err);

	msleep(500);

	cyccg_dbg("%s: <<<<: Send unstructured VMD data:\n", __func__);
	mem_dump(CYCCG_DEBUG, unstructured_vdm, unstructured_vdm_size);
	err = cyccg_send_vdm_data(cyccg,
				  unstructured_vdm, unstructured_vdm_size);
	cyccg_dbg("%s: >>>>: result of send unstructured VDM data, err = %d\n",
		  __func__, err);

	mutex_lock(&cyccg->vdm_work_lock);
	cyccg->vdm_work_state = CYCCG_WORK_NONE;
	mutex_unlock(&cyccg->vdm_work_lock);
}

static void cyccg_queue_vdm_work(struct cyccg *cyccg)
{
	mutex_lock(&cyccg->vdm_work_lock);
	if (cyccg->vdm_work_state == CYCCG_WORK_NONE) {
		cyccg->vdm_work_state = CYCCG_WORK_QUEUED;
		schedule_work(&cyccg->vdm_work);

		cyccg_dbg("%s: VMD work queued.\n", __func__);
	} else {
		cyccg_dbg("%s: current vdm_work_state = %d\n",
			  __func__, cyccg->vdm_work_state);
	}
	mutex_unlock(&cyccg->vdm_work_lock);
}
#endif

struct device *g_cyccg_device;
static ssize_t cyccg_usb_audio_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
	int rv = 0;
	bool if_letv = false;
	int pid;

	usb_audio_if_letv(&if_letv,&pid);
	if (if_letv)
	    rv = scnprintf(buf, PAGE_SIZE, "%d\n", if_letv);
	else
	    rv = scnprintf(buf, PAGE_SIZE, "%d\n", 0);

	return rv;
}

static ssize_t cyccg_usb_audio_pid_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
	int rv = 0;
	bool if_letv = false;
	int pid;

	usb_audio_if_letv(&if_letv,&pid);
	if (if_letv)
	    rv = scnprintf(buf, PAGE_SIZE, "0x%x\n", pid);
	else
	    rv = scnprintf(buf, PAGE_SIZE, "%d\n", 0);

	return rv;
}

static ssize_t cyccg_dev_id_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
	int rv = 0;
	int err;
	struct cyccg *cyccg = g_cyccg;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	mutex_unlock(&cyccg->lock);
	if (CYCCG_SILICON_ID == cyccg->silicon_id)
	    rv = scnprintf(buf, PAGE_SIZE, "CYPD2122\n");
	else
	    rv = scnprintf(buf, PAGE_SIZE, "\n");
	return rv;
}

static struct device_attribute cyccg_cclogic_attrs[] = {
	__ATTR(devid, S_IRUGO, cyccg_dev_id_show,NULL),
	__ATTR(usb_audio, S_IRUGO, cyccg_usb_audio_show,NULL),
	__ATTR(usb_audio_pid, S_IRUGO, cyccg_usb_audio_pid_show,NULL)
};

/* This routine must be called with @cyccgdev_lock acquired. */
static int cyccg_register_chrdev_class(struct cyccg *cyccg)
{
	int err;
	struct device *dev = &cyccg->client->dev;

	/* Dynamically allocate and register a major and return its number. */
	if (!cyccg_major) {
		cyccg_major = register_chrdev(UNNAMED_MAJOR,
					      CYCCG_DEV_NAME, &cyccg_fops);
		if (cyccg_major <= 0) {
			err = cyccg_major == 0 ? -EBUSY : cyccg_major;
			dev_err(dev, "%s: failed to get chrdev major, (%d)\n",
				__func__, cyccg_major);
			goto err_chrdev_major;
		}
	}

	/* Create a struct class structure for device create. */
	if (!cyccg_class) {
		cyccg_class = class_create(THIS_MODULE, CYCCG_DEV_NAME);
		err = PTR_ERR(cyccg_class);
		if (IS_ERR(cyccg_class))
			goto err_class;
	}

	return 0;
err_class:
	class_destroy(cyccg_class);
	cyccg_class = NULL;
err_chrdev_major:
	unregister_chrdev(cyccg_major, CYCCG_DEV_NAME);
	cyccg_major = 0;
	return err;
}

static int cyccg_register_chrdev(struct cyccg *cyccg)
{
	struct device *dev = &cyccg->client->dev;
	struct device *ccgdev;
	int minor;
	int err;
	int i;

	mutex_lock(&cyccgdev_lock);

	err = cyccg_register_chrdev_class(cyccg);
	if (err)
		goto out;

	minor = find_first_zero_bit(cyccgdev_minors, CYCCG_DYNAMIC_MINORS);
	if (minor >= CYCCG_DYNAMIC_MINORS) {
		err = -EBUSY;
		goto out;
	}

	ccgdev = device_create(cyccg_class, NULL,
			       MKDEV(cyccg_major, minor), cyccg,
			       "%s%d", CYCCG_DEV_NAME, minor);
	if (IS_ERR(ccgdev)) {
		dev_err(dev, "%s: failed to create cyccg dev instance, (%d)\n",
			__func__, err);
		goto out;
	}

	cyccg->ccgdev = ccgdev;
	cyccg->minor = minor;
	set_bit(minor, cyccgdev_minors);
	sprintf(cyccg->name, "%s%d", CYCCG_DEV_NAME, minor);
	list_add(&cyccg->list, &cyccgdev_list);

	g_cyccg_device = device_create(cyccg_class, NULL, dev->devt, NULL, "cclogic_device");

	for (i = 0; i < ARRAY_SIZE(cyccg_cclogic_attrs); i++)
		if (device_create_file(g_cyccg_device, &cyccg_cclogic_attrs[i]))
			goto error;

	goto out;

error:
	for (i=0; i >= 0; i++)
		device_remove_file(dev, &cyccg_cclogic_attrs[i]);
	device_destroy(cyccg_class, dev->devt);
	pr_err("%s %s: Unable to create interface", LOG_TAG, __func__);
out:
	mutex_unlock(&cyccgdev_lock);
	return err;
}

/* HARD_RESET event handler for Huawei board with external AP. */
static int cyccg_hard_reset_handler(struct cyccg *cyccg)
{
	struct ccg_status ccg_status;
	int err;

	err = cyccg_get_ccg_status(cyccg, &ccg_status);
	if (err) {
		cyccg_dbg("%s: failed to get ccg_status, (%d).\n",
			  __func__, err);
		return err;
	}

	if (ccg_status.contract_status &&
	    (ccg_status.current_port_power_role ==
		CCG_STATUS_CURRENT_POWER_ROLE_SOURCE)) {
		/* 1. Wait 25-35ms  for the HARD_RESET command prepare ready. */
		usleep_range(25000, 35000);

		/* 2. Turn VBus OFF. */
		/* TODO(third-party): Notify AP to turn VBus off. */
		cclogic_set_vbus(0);

		/* 3. Wait 700-740ms for the HARD_RESET completed. */
		usleep_range(700000, 740000);

		/* 4. Set OTG host. */
		/* TODO(third-party): Notify AP to set to OTG host. */

		/* 5. Turn VBus ON. */
		/* TODO(third-party): Notify AP to turn VBus on. */
		cclogic_set_vbus(1);
	} else {
		/* 1. Set OTG device. */
		/* TODO(third-party): Notify AP to set to OTG device. */
	}
	return 0;
}

static int cyccg_unregister_chrdev(struct cyccg *cyccg)
{
	int i;
	struct device *dev = &cyccg->client->dev;

	if (WARN_ON(list_empty(&cyccg->list)))
		return -EINVAL;

	mutex_lock(&cyccgdev_lock);

	for (i = 0; i < ARRAY_SIZE(cyccg_cclogic_attrs); i++)
		device_remove_file(dev, &cyccg_cclogic_attrs[i]);
	device_destroy(cyccg_class, dev->devt);

	list_del(&cyccg->list);
	device_destroy(cyccg_class, MKDEV(cyccg_major, cyccg->minor));
	if (cyccg->minor >= 0 && cyccg->minor < CYCCG_DYNAMIC_MINORS) {
		clear_bit(cyccg->minor, cyccgdev_minors);
		cyccg->minor = CYCCG_DYNAMIC_MINORS;
	}

	/* Do unregister only when no device connected to the class. */
	if (list_empty(&cyccgdev_list)) {
		unregister_chrdev(cyccg_major, CYCCG_DEV_NAME);
		class_destroy(cyccg_class);
		cyccg_major = 0;
		cyccg_class = NULL;
	}

	mutex_unlock(&cyccgdev_lock);
	return 0;
}
static int cyccg_drv_internal_event_process(struct cyccg *cyccg,
					    struct hpi_msg *msg)
{
	struct device *dev = &cyccg->client->dev;
	struct hpi_msg resp_msg;
	struct ccg_status ccg_status;
#if CYCCG_CCG_SUPPORT_SWAP_COMPLETE
	u8 data[32];
#endif
	int err = 0;

	if (!msg) {
		msg = &resp_msg;
		err = cyccg_i2c_read(cyccg, (u8 *)msg, sizeof(struct hpi_msg),
				     CY_PD_REG_RESPONSE_ADDR);
		if (err) {
			cyccg_dbg("%s: remote I/O error\n", __func__);
#if CYCCG_ENABLE_RESUME_IRQ_WORK_THREAD
			cyccg_hpi_clear_intr(cyccg);
#endif
			return err;
		}
	}

	cyccg_dbg("%s: msg: code=%#02x, len=%d\n", __func__,
		  msg->head.code, msg->head.len);
	mem_dump(CYCCG_DEBUG, msg->data, msg->head.len);
	/*
	 * Try to recapture the command response message for polling process
	 * event the command thread has timeout but before exits.
	 */
	if (IS_CY_PD_CMD_RESP_MSG(&resp_msg.head))
		cyccg_enqueue_msg(cyccg, &resp_msg);

	switch (msg->head.code) {
	/* Command response events. */
	case CY_PD_RESP_NO_RESPONSE:
		cyccg_dbg("%s: CY_PD_RESP_NO_RESPONSE", __func__);
		return 0;
	case CY_PD_RESP_SUCCESS:
		cyccg_dbg("%s: CY_PD_RESP_SUCCESS\n", __func__);
		break;
	case CY_PD_RESP_FLASH_DATA_AVAILABLE:
		/*
		 * Data memory contains flash data.
		 * Read 128 bytes of flash row data.
		 * This is used while updating FW/Config table over I2C.
		 */
		cyccg_dbg("%s: CY_PD_RESP_FLASH_DATA_AVAILABLE\n", __func__);
		break;
	case CY_PD_RESP_INVALID_COMMAND:
		cyccg_dbg("%s: CY_PD_RESP_INVALID_COMMAND\n", __func__);
		cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		break;
	case CY_PD_RESP_COLLISION_DETECTED:
		cyccg_dbg("%s: CY_PD_RESP_COLLISION_DETECTED\n", __func__);
		break;
	case CY_PD_RESP_FLASH_UPDATE_FAILED:
		cyccg_dbg("%s: CY_PD_RESP_FLASH_UPDATE_FAILED\n", __func__);
		break;
	case CY_PD_RESP_INVALID_FW:
		cyccg_dbg("%s: CY_PD_RESP_INVALID_FW\n", __func__);
		break;
	case CY_PD_RESP_INVALID_ARGUMENTS:
		cyccg_dbg("%s: CY_PD_RESP_INVALID_ARGUMENTS\n", __func__);
		cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		break;
	case CY_PD_RESP_NOT_SUPPORTED:
		cyccg_dbg("%s: CY_PD_RESP_NOT_SUPPORTED\n", __func__);
		cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		break;
	case CY_PD_RESP_TRANSACTION_FAILED:
		/* PD message transmission failed. */
		cyccg_dbg("%s: CY_PD_RESP_TRANSACTION_FAILED\n", __func__);
		cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		break;
	case CY_PD_RESP_PD_COMMAND_FAILED:
		/* PD comamnd failed. */
		cyccg_dbg("%s: CY_PD_RESP_PD_COMMAND_FAILED\n", __func__);

		cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		break;
	case CY_PD_RESP_UNDEFINED:
		cyccg_dbg("%s: CY_PD_RESP_UNDEFINED\n", __func__);
		break;

	/* Device Specific Events, 0x80 - 0x81 */
	case CY_PD_RESP_RESET_COMPLETE:
		/* RESET_COMPLETE event detected. */
		cyccg_dbg("%s: CY_PD_RESP_RESET_COMPLETE\n", __func__);

		mutex_lock(&cyccg->ext_evt_lock);
		cyccg->is_swap_initiated_by_partner = false;
		cyccg->is_ap_synced = false;
		mutex_unlock(&cyccg->ext_evt_lock);
		mutex_lock(&cyccg->ccg_status_lock);
		memset(&cyccg->ccg_status, 0, sizeof(struct ccg_status));
		mutex_unlock(&cyccg->ccg_status_lock);

		if (cyccg->ccgdev) {
			/*
			 * The RESET_COMPLETE event not triggerred by commands,
			 * it should be caused by the CCGx deevice, so queue
			 * the work to reinitialize the CCGx device.
			 */
			cyccg_queue_init_work(cyccg);
		}

		break;
	case CY_PD_RESP_MESSAGE_QUEUE_OVERFLOW:
		/* Message queue overflow event. */
		cyccg_dbg("%s: CY_PD_RESP_MESSAGE_QUEUE_OVERFLOW\n", __func__);
		cyccg_dump_pending_events_no_sync(cyccg);
		/* The IRQ bit will be cleared in above function. */
		return 0;

	/* Type C specific events, 0x82 - 0x85 */
	case CY_PD_RESP_OVER_CURRENT_DETECTED:
		cyccg_dbg("%s: CY_PD_RESP_OVER_CURRENT_DETECTED\n", __func__);
		break;
	case CY_PD_RESP_OVER_VOLTAGE_DETECTED:
		cyccg_dbg("%s: CY_PD_RESP_OVER_VOLTAGE_DETECTED\n", __func__);
		break;
	case CY_PD_RESP_TYPE_C_CONNECTED:
		/* Type C connect event detected. */
		cyccg_dbg("%s: CY_PD_RESP_TYPE_C_CONNECTED\n", __func__);

		/* Record the status, if failed to read, retry it. */
		err = cyccg_get_latest_ccg_status(cyccg, &ccg_status);
		if (err || !ccg_status.port_connected) {
			/* If firwmare hasn't update this, wait for a moment. */
			usleep_range(1000, 2000);
			err = cyccg_get_latest_ccg_status(cyccg, &ccg_status);
			if (err || !ccg_status.port_connected) {
				err = err ? err : -EINVAL;
				break;
			}
		}

#if CYCCG_POWER_CONTROLLED_BY_EXTERNAL_AP
		/* Sync AP status with attached device. */
		cyccg_update_external_ap_status(cyccg,
			CY_PD_RESP_TYPE_C_CONNECTED);
#endif

		break;
	case CY_PD_RESP_TYPE_C_DISCONNECTED:
		/* Type C disconnect event detected. */
		cyccg_dbg("%s: CY_PD_RESP_TYPE_C_DISCONNECTED\n", __func__);
//letv_pd s
		letv_pd_set_usb_mode(LETV_USB_DRP_MODE);
//letv_pd e

		/* Record the status, if failed to read, retry it. */
		err = cyccg_get_latest_ccg_status(cyccg, &ccg_status);
		if (err || ccg_status.port_connected ||
		    ccg_status.port_connected) {
			/* If firwmare hasn't update this, wait for a moment. */
			usleep_range(1000, 2000);
			err = cyccg_get_latest_ccg_status(cyccg, &ccg_status);
			if (err || ccg_status.port_connected ||
			    ccg_status.port_connected) {
				err = err ? err : -EINVAL;
				/*
				 * Mostly, it may be that the TYPE_C_CONNECTED
				 * has been conneceted again and has been queued
				 * to the message queue, and the CCG states have
				 * already been udpated. So break and then try
				 * to process next events.
				 * This will happen when doing unplug-plug
				 * quickly.
				 */
				break;
			}
		}

#if CYCCG_POWER_CONTROLLED_BY_EXTERNAL_AP
		/* Sync AP status with attached device. */
		cyccg_update_external_ap_status(cyccg,
			CY_PD_RESP_TYPE_C_DISCONNECTED);
#endif

		break;

	/* PD specific events and asynchronous messages, 0x86 - 0x8F */
	case CY_PD_RESP_PD_CONTRACT_ESTABLISHED:
		/* PD contract established event detected. */
		cyccg_dbg("%s: CY_PD_RESP_PD_CONTRACT_ESTABLISHED\n", __func__);

#if CYCCG_POWER_CONTROLLED_BY_EXTERNAL_AP
		/* Sync AP status with attached device. */
		cyccg_update_external_ap_status(cyccg, 0);
#else
#if CYCCG_DEBUG
		mutex_lock(&cyccg->ccg_status_lock);
		cyccg_get_ccg_status(cyccg, NULL);
		mutex_unlock(&cyccg->ccg_status_lock);
#endif
#endif
		break;

#if CYCCG_CCG_SUPPORT_SWAP_COMPLETE
	case CY_PD_RESP_SWAP_COMPLETE:
		cyccg_dbg("%s: CY_PD_RESP_SWAP_COMPLETE\n", __func__);

		err = cyccg_hpi_get_swap_status(cyccg, data, 1);
		if (err) {
			cyccg_dbg("%s: error read SWAP complete status, (%d)\n",
				  __func__, err);
			cyccg_ext_evt_cmd_complete(cyccg,
					CYCCG_EVT_CMD_FAIL);
			break;
		}
		cyccg_dbg("%s: SWAP reason: 0x%02x\n",
			  __func__, cyccg->ext_evt_reason);
		cyccg_dbg("%s: SWAP Type: 0x%02x, mapped SWAP reason: 0x%02x\n",
			  __func__, data[0] & HPI_SWAP_TYPE_MASK,
			  hpi_swap_type_pd_cmd_map[data[0] & HPI_SWAP_TYPE_MASK]
			  );
		cyccg_dbg("%s: SWAP complete status: 0x%02x\n",
			  __func__, data[0] & HPI_SWAP_RESP_MASK);

		if (hpi_swap_type_pd_cmd_map[data[0] & HPI_SWAP_TYPE_MASK] ==
		    cyccg->ext_evt_reason) {
			if ((data[0] & HPI_SWAP_RESP_MASK) == HPI_SWAP_ACCEPT) {
				/* SWAP complete and success. */
				cyccg_ext_evt_cmd_complete(cyccg,
						CYCCG_EVT_CMD_SUCCESS);
			} else {
				/* SWAP failed. */
				cyccg_ext_evt_cmd_complete(cyccg,
						CYCCG_EVT_CMD_FAIL);
			}
		} else {
			/*
			 * When do PR_SWAP command, the data role will also be
			 * swapped in some platforms, so there will be two
			 * SWAP_COMPLETE events.
			 * The first is the SWAP_COMPLETE of data role swap,
			 * the second is the SWAP_COMPLETE of power role swap.
			 * If the SWAP type not match with the trigger reason,
			 * the cyccg_hpi_swap_cmd_send() routine must skip it
			 * and continue wait for the correct SWAP_COMPLETE evet.
			 */
			cyccg_dbg("%s: %s", __func__,
				"SWAP Type not match SWAP reason, continue\n");
		}

//letv_pd s
		memset(msg->data, 0, CY_PD_MSG_MAX_DATA_SIZE);
		msg->data[0] = data[0];
		msg->head.len = 1;
		if ((data[0] & HPI_SWAP_TYPE_MASK) == 0) {
			letv_pd_message_handle(LETV_PD_MESSAGE_DR_SWAP, msg);
		} else if ((data[0] & HPI_SWAP_TYPE_MASK) == 1) {
			letv_pd_message_handle(LETV_PD_MESSAGE_PR_SWAP, msg);
		} else if ((data[0] & HPI_SWAP_TYPE_MASK) == 2) {
			letv_pd_message_handle(LETV_PD_MESSAGE_VCONN_SWAP, msg);
		}
//letv_pd e

#if CYCCG_DEBUG
		mutex_lock(&cyccg->ccg_status_lock);
		cyccg_get_ccg_status(cyccg, NULL);
		mutex_unlock(&cyccg->ccg_status_lock);
#endif

		break;
#else  /* CYCCG_CCG_SUPPORT_SWAP_COMPLETE */
	case CY_PD_RESP_DR_SWAP:
		cyccg_dbg("%s: CY_PD_RESP_DR_SWAP\n", __func__);
		break;
#endif
	case CY_PD_RESP_PR_SWAP:
		/* The PR_SWAP is initiated by the Port Partner. */
		cyccg_dbg("%s: CY_PD_RESP_PR_SWAP: Initiate by Port Partner\n",
			  __func__);

#if CYCCG_POWER_CONTROLLED_BY_EXTERNAL_AP
		cyccg_hpi_clear_intr(cyccg);
		return cyccg_update_external_ap_status(cyccg,
				CY_PD_RESP_PR_SWAP);
#else
		break;
#endif

	case CY_PD_RESP_VCON_SWAP:
		cyccg_dbg("%s: CY_PD_RESP_VCON_SWAP\n", __func__);
		break;

	case CY_PD_RESP_PS_RDY:
		cyccg_dbg("%s: CY_PD_RESP_PS_RDY\n", __func__);

#if CYCCG_POWER_CONTROLLED_BY_EXTERNAL_AP
		cyccg_hpi_clear_intr(cyccg);
//letv_pd s
		letv_pd_message_handle(LETV_PD_MESSAGE_PS_RDY, msg);
//letv_pd e
		return cyccg_update_external_ap_status(cyccg,
				CY_PD_RESP_PS_RDY);
#else
		break;
#endif

	case CY_PD_RESP_GOTOMIN:
		cyccg_dbg("%s: CY_PD_RESP_GOTOMIN\n", __func__);
		break;
	case CY_PD_RESP_ACCEPT_MESSAGE:
		cyccg_dbg("%s: CY_PD_RESP_ACCEPT_MESSAGE\n", __func__);

#if CYCCG_POWER_CONTROLLED_BY_EXTERNAL_AP
		cyccg_hpi_clear_intr(cyccg);
		return cyccg_update_external_ap_status(cyccg,
				CY_PD_RESP_ACCEPT_MESSAGE);
#else
		break;
#endif

	case CY_PD_RESP_REJECT_MESSAGE:
		cyccg_dbg("%s: CY_PD_RESP_REJECT_MESSAGE\n", __func__);

		cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		break;
	case CY_PD_RESP_WAIT_MESSAGE:
		cyccg_dbg("%s: CY_PD_RESP_WAIT_MESSAGE\n", __func__);

		cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		break;
	case CY_PD_RESP_HARD_RESET:
		/* The HARD_RESET is initiated by the Port Partner. */
		cyccg_dbg("%s: CY_PD_RESP_HARD_RESET: from Port Partner.\n",
			  __func__);
		cyccg_hpi_clear_intr(cyccg);

		mutex_lock(&cyccg->ext_evt_lock);
		if (cyccg->ext_evt_waiting) {
			cyccg_dbg("%s: swap failed by RESP_HARD_RESET.\n",
				  __func__);
			cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		}
		mutex_unlock(&cyccg->ext_evt_lock);

		/* Normal hard reset woring with external AP. */
		err = cyccg_hard_reset_handler(cyccg);

		return err;

	/* PD Data Message Specific Events, 0x90 */
	case CY_PD_RESP_VDM_RECEIVED:
		/*
		 * VDM received event detected.
		 * Collect the VDM response.
		 * User can use this info to handle various VDM related events.
		 */
		cyccg_dbg("%s: CY_PD_RESP_VDM_RECEIVED\n", __func__);

		cyccg_dbg("%s: received VDM data (%d):\n", __func__,
			  msg->head.len);
		if (msg->head.len)
			mem_dump(CYCCG_DEBUG, msg->data, msg->head.len);
//letv_pd s
		letv_pd_message_handle(LETV_PD_MESSAGE_VDM, msg);
//letv_pd e
#if CYCCG_DEBUG
		/*
		 * Note, when want to send VDM data in IRQ thread, must use
		 * the cyccg_queue_vdm_work() as example.
		 */
		if (0)  /* Just as an example. */
			cyccg_queue_vdm_work(cyccg);
#endif
		break;

	/* Capability Message Specific Events, 0x91 - 0x92 */
	case CY_PD_RESP_SRC_CAP_RCVD:
		cyccg_dbg("%s: CY_PD_RESP_SRC_CAP_RCVD\n", __func__);
//letv_pd s
		letv_pd_message_handle(LETV_PD_MESSAGE_SOURCE_CAP, msg);
//letv_pd e
		break;
	case CY_PD_RESP_SINK_CAP_RCVD:
		//letv_pd
		/* SELECT_SINK_PDO */
		cyccg_dbg("%s: CY_PD_RESP_SINK_CAP_RCVD\n", __func__);
		break;

	/* DP and Alternate mode Specific Events, 0x93 - 0x99 */
	case CY_PD_RESP_DP_ALTERNATE_MODE:
		cyccg_dbg("%s: CY_PD_RESP_DP_ALTERNATE_MODE\n", __func__);
		break;
	case CY_PD_RESP_DP_DEVICE_CONNECTED:
		cyccg_dbg("%s: CY_PD_RESP_DP_DEVICE_CONNECTED\n", __func__);
		break;
	case CY_PD_RESP_DP_DEVICE_NOT_CONNECTED:
		cyccg_dbg("%s: CY_PD_RESP_DP_DEVICE_NOT_CONNECTED\n", __func__);
		break;
	case CY_PD_RESP_DP_SID_NOT_FOUND:
		cyccg_dbg("%s: CY_PD_RESP_DP_SID_NOT_FOUND\n", __func__);
		break;
	case CY_PD_RESP_MULTIPLE_SVID_DISCOVERED:
		cyccg_dbg("%s: CY_PD_RESP_MULTIPLE_SVID_DISCOVERED\n",
			  __func__);
		break;
	case CY_PD_RESP_DP_FUNCTION_NOT_SUPPORTED:
		cyccg_dbg("%s: CY_PD_RESP_DP_FUNCTION_NOT_SUPPORTED\n",
			  __func__);
		break;
	case CY_PD_RESP_DP_PORT_CONFIG_NOT_SUPPORTED:
		cyccg_dbg("%s: CY_PD_RESP_DP_PORT_CONFIG_NOT_SUPPORTED\n",
			  __func__);
		break;

	/* Resets and Error Scenario Events, 0x9A - 0xA5 */
	case CY_PD_HARD_RESET_SENT:
		/* CCG initiates the HARD RESET command. */
		cyccg_dbg("%s: CY_PD_HARD_RESET_SENT: from CCGx device.\n",
			  __func__);
		cyccg_hpi_clear_intr(cyccg);

		mutex_lock(&cyccg->ext_evt_lock);
		if (cyccg->ext_evt_waiting) {
			cyccg_dbg("%s: swap failed by HARD_RESET_SENT.\n",
				  __func__);
			cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		}
		mutex_unlock(&cyccg->ext_evt_lock);

		/* Normal hard reset woring with external AP. */
		err = cyccg_hard_reset_handler(cyccg);

		return err;
	case CY_PD_SOFT_RESET_SENT:
		cyccg_dbg("%s: CY_PD_SOFT_RESET_SENT\n", __func__);
		break;
	case CY_PD_CABLE_RESET:
		cyccg_dbg("%s: CY_PD_CABLE_RESET\n", __func__);
		break;
	case CY_PD_SOURCE_DISBALED_STATE_ENTERED:
		cyccg_dbg("%s: CY_PD_SOURCE_DISBALED_STATE_ENTERED\n",
			  __func__);

		cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		break;
	case CY_PD_SENDER_RESPONSE_TIMER_TIMEOUT:
		cyccg_dbg("%s: CY_PD_SENDER_RESPONSE_TIMER_TIMEOUT\n",
			  __func__);

		cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		break;
	case CY_PD_NO_VDM_RESPONSE_RECEIVED:
		cyccg_dbg("%s: CY_PD_NO_VDM_RESPONSE_RECEIVED\n", __func__);
		break;
	case CY_PD_UNEXPECTED_VOLTAGE_VBUS:
		cyccg_dbg("%s: CY_PD_UNEXPECTED_VOLTAGE_VBUS\n", __func__);
		/*
		 * Disable Type-C port to avoid CCG2 keep report this events,
		 * which will easily cause the CCG2 internal message queue
		 * got overflowed.
		 */
		//cyccg_port_disable(cyccg);
		//Turn off Vbus to avoid CCG2 disabling status
		cclogic_set_vbus(0);
		/* Queue the work to reset and reinitialize the CCG2 device. */
		//cyccg_queue_init_work(cyccg);

		break;
	case CY_PD_TYPE_C_ERROR_RECOVERY:
		cyccg_dbg("%s: CY_PD_TYPE_C_ERROR_RECOVERY\n", __func__);

		cyccg_ext_evt_cmd_complete(cyccg, CYCCG_EVT_CMD_FAIL);
		break;

	/* EMCA Related EVents, 0xA6 */
	case CY_PD_EMCA_DETECTED:
		cyccg_dbg("%s: CY_PD_EMCA_DETECTED\n", __func__);
		break;

	case CY_PD_RP_CHANGE_DETECTED:
		cyccg_dbg("%s: CY_PD_RP_CHANGE_DETECTED\n", __func__);
		break;

	default:
		/*
		 * Those events which are not being handled in this callback
		 * function will fall here.
		 */
		dev_err(dev, "Event Received: %#02x\n", msg->head.code);
		break;
	}
	cyccg_hpi_clear_intr(cyccg);
	return 0;
}

static int cyccg_irq_drv_internal_cmd_process(struct cyccg *cyccg)
{
	struct hpi_msg resp_msg;
	int length;
	int err;

	err = cyccg_i2c_read(cyccg, (u8 *)&resp_msg, sizeof(struct hpi_msg),
			     CY_PD_REG_RESPONSE_ADDR);
	if (err) {
		cyccg_dbg("%s: remote I/O error\n", __func__);
#if CYCCG_ENABLE_RESUME_IRQ_WORK_THREAD
		cyccg_hpi_clear_intr(cyccg);
#endif
		return err;
	}

	cyccg_dbg_irq("%s: msg: code=%#02x, len=%d\n", __func__,
		      resp_msg.head.code, resp_msg.head.len);

	length = (resp_msg.head.len > CY_PD_MSG_MAX_DATA_SIZE) ?
		 CY_PD_MSG_MAX_DATA_SIZE : resp_msg.head.len;

	/* Miss triggerred interrupt for no response data. */
	if (resp_msg.head.code == CY_PD_RESP_NO_RESPONSE)
		return 0;

	/*
	 * The purpose of adding excepted events checking is to avoid that
	 * when driver is running some commans, and at the same time, external
	 * device is plugin or unplugged that will generate events, these
	 * events from the CCG device should be filterred out as the response
	 * events of the processing command.
	 * So when send sync command that will receive events as its responses,
	 * caller must set the excepted response in the cyccg->excepted_events
	 * variable. If not events response required, just set it to NULL.
	 * The purpose of reset the @cmd_issued here is to avoid the interrupt
	 * comes too fast and host processes too slow issue, which
	 * will the cause the command response be processed the twice, and
	 * the event may be missed for other process.
	 */
	if (IS_CY_PD_CMD_RESP_MSG(&resp_msg.head) ||
	    cyccg_excepted_events_match(cyccg, resp_msg.head.code)) {
		/*
		 * Try to enqueue the msg when the there were multi-response
		 * events for a command. And the host cannot run as quickly
		 * as possible to start to listen on the next response event
		 * before the CCG device assert the interrupt for the next
		 * event.
		 * The driver must set the enable_msg_queue before it writes the
		 * command which will cause multi-response event.
		 * This issue will easily heppend in low speed host.
		 * Also for avoid the issue that on slow host, that is the
		 * device interrupt will come and be processed before the
		 * resp_ready completion object is queued which will cause
		 * the complete(&cyccg->resp_ready) takes no effect,
		 * and cause the event message be missed.
		 */
		cyccg_enqueue_msg(cyccg, &resp_msg);

		if (atomic_cmpxchg(&cyccg->cmd_issued, 1, 0)) {
			cyccg_dbg_irq("%s: issued command resp process.\n",
				      __func__);
			/* Copy the comamnd response data. */
			if (cyccg->resp_msg)
				memcpy(cyccg->resp_msg, &resp_msg, length + 2);

			cyccg_hpi_clear_intr(cyccg);
			/* Notify command process thread response is ready. */
			complete(&cyccg->resp_ready);
			return 0;
		}

		cyccg_hpi_clear_intr(cyccg);
		return 0;
	}

	cyccg_drv_internal_event_process(cyccg, &resp_msg);
	return 0;
}

static bool cyccg_irq_next_ready(struct cyccg *cyccg)
{
#if CYCCG_ENABLE_IRQ_READY_POLLING
	struct hpi_msg_head head;
	u8 irq_state;
	int err;

	irq_state = 0;
	err = cyccg_i2c_read(cyccg, &irq_state, 1, CY_PD_REG_INTR_REG_ADDR);
	if (err || !(irq_state & 0x01))
		return false;

	memset(&head, 0, sizeof(struct hpi_msg_head));
	err = cyccg_i2c_read(cyccg, (u8 *)&head, sizeof(struct hpi_msg_head),
			     CY_PD_REG_RESPONSE_ADDR);
	if (err || !head.code)
		return false;

	cyccg_dbg_irq("%s: ---- irq_next_ready_polling\n", __func__);
	return true;
#else
	return false;
#endif
}

static void cyccg_irq_thread_handler(struct cyccg *cyccg)
{
	int timeout;

	mutex_lock(&cyccg->irq_work_lock);
	if (cyccg->irq_work_state == CYCCG_WORK_RUNNING) {
		cyccg_dbg("%s: return early, thread handler is in running\n",
			  __func__);
		mutex_unlock(&cyccg->irq_work_lock);
		return;
	}
	cyccg->irq_work_state = CYCCG_WORK_RUNNING;
	mutex_unlock(&cyccg->irq_work_lock);

	do {
		if (is_cyccg_drv_internal_processing(cyccg)) {
			cyccg_dbg_irq("%s: driver internal monitor process.\n",
				      __func__);
			cyccg_irq_drv_internal_cmd_process(cyccg);
		} else {
			mutex_lock(&cyccg->ext_evt_lock);
			if (cyccg->async_queue && !cyccg->ext_evt_waiting) {
				mutex_unlock(&cyccg->ext_evt_lock);

				timeout = 1000;  /* ms */
				init_completion(&cyccg->ioctl_clear_intr);

				/* Notify user mode application data arrives. */
				kill_fasync(&cyccg->async_queue,
					    SIGIO, POLL_IN);

				cyccg_dbg_irq("%s: fire SIGIO.\n", __func__);

				/*
				 * Wait user mode application finish process,
				 * and notify through ioctl.
				 */
				atomic_inc(&cyccg->sigio_queued);
				timeout = wait_for_completion_timeout(
						&cyccg->ioctl_clear_intr,
						msecs_to_jiffies(timeout));
				if (timeout == 0)
					atomic_cmpxchg(&cyccg->sigio_queued,
						       1, 0);
				/*
				 * Clear INTR_REG, so CCG device can update
				 * respose register and assert interrupt for
				 * next event or command response.
				 */
				cyccg_hpi_clear_intr(cyccg);
			} else {
				mutex_unlock(&cyccg->ext_evt_lock);
				cyccg_dbg_irq("%s: driver default process.\n",
					      __func__);
				cyccg_drv_internal_event_process(cyccg, NULL);
			}
		}

		/*
		 * Polling for next ready irq event, it's aimed to fix the issue
		 * on some platforms that when next event queued before the
		 * prevous event be processed and cleared, then the irq for the
		 * next event won't be triggerred in host and no irq handler
		 * will be executed to read and process the next event.
		 * It will block both driver process and CCGx device report
		 * new events.
		 */
	} while (cyccg_irq_next_ready(cyccg));

	mutex_lock(&cyccg->irq_work_lock);
	cyccg->irq_work_state = CYCCG_WORK_NONE;
	mutex_unlock(&cyccg->irq_work_lock);
}

static irqreturn_t cyccg_irq_therad(int irq, void *dev_id)
{
	struct cyccg *cyccg = dev_id;
	struct device *dev = &cyccg->client->dev;

	cyccg_dbg_irq("%s: <<<< irq arrive\n", __func__);

	/* Notify the PM core of a wakeup event. */
	if (device_may_wakeup(dev))
		pm_wakeup_event(dev, 0);

	cyccg_irq_thread_handler(cyccg);

	cyccg_dbg_irq("%s: >>>> IRQ_HANDLED\n", __func__);
	return IRQ_HANDLED;
}

static void cyccg_irq_thread_worker(struct work_struct *work)
{
	struct cyccg *cyccg = container_of(work, struct cyccg, irq_work);

	cyccg_dbg("%s: <<<<: start\n", __func__);
	cyccg_irq_thread_handler(cyccg);
	cyccg_dbg("%s: >>>>: end\n", __func__);
}

static void cyccg_queue_irq_thread_work(struct cyccg *cyccg)
{
	mutex_lock(&cyccg->irq_work_lock);
	if (cyccg->irq_work_state == CYCCG_WORK_NONE) {
		cyccg->irq_work_state = CYCCG_WORK_QUEUED;
		schedule_work(&cyccg->irq_work);

		cyccg_dbg("%s: irq thread work queued.\n", __func__);
	} else {
		cyccg_dbg("%s: current irq_thread_work_state = %d\n",
			  __func__, cyccg->irq_work_state);
	}
	mutex_unlock(&cyccg->irq_work_lock);
}

static ssize_t cyccg_show_char_dev_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int size;
	int err;

	err = mutex_lock_interruptible(&cyccgdev_lock);
	if (err)
		return err;
	size = scnprintf(buf, PAGE_SIZE, "/dev/%s\n", cyccg->name);
	mutex_unlock(&cyccgdev_lock);

	return size;
}

/* output string, 1 - Normal operation mode; 0 - Bootload Mode. */
static ssize_t cyccg_show_device_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	u8 device_mode;
	int size;
	int err;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;

	err = cyccg_i2c_read(cyccg, &device_mode, 1,
			     CY_PD_REG_DEVICE_MODE_ADDR);
	if (err)
		goto out;
	size = scnprintf(buf, PAGE_SIZE, "%d\n", device_mode);

out:
	mutex_unlock(&cyccg->lock);
	return err ? err : size;
}

/* The output value string in decimal format. */
static ssize_t cyccg_show_i2c_offset_count(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int size;
	int err;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	size = scnprintf(buf, PAGE_SIZE, "offset: %d\ncount: %d\n",
					cyccg->offset, cyccg->count);
	mutex_unlock(&cyccg->lock);

	return size;
}

/* The input value string must be in decimal format. */
static ssize_t cyccg_store_i2c_offset_count(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	char **data_arr;
	int data_arr_count;
	unsigned long val;
	int err = 0;

	data_arr = argv_split(GFP_KERNEL, buf, &data_arr_count);
	if (!data_arr || data_arr_count < 1)
		return -EINVAL;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		goto free;

	if (kstrtoul(data_arr[0], 16, &val)) {
		err = -EINVAL;
		goto out;
	}
	cyccg->offset = val;

	if (data_arr_count > 1) {
		if (kstrtoul(data_arr[1], 16, &val)) {
			err = -EINVAL;
			goto out;
		}
		cyccg->count = val;
	} else {
		cyccg->count = CYCCG_I2C_REGISTER_MAP_SIZE - cyccg->offset;
	}

	cyccg_dbg("%s: offset=%d, count=%d\n", __func__,
		  cyccg->offset, cyccg->count);

	if (cyccg->offset < 0 || cyccg->count <= 0 ||
	    cyccg->offset > (CYCCG_I2C_REGISTER_MAP_SIZE - 1) ||
	    (cyccg->offset + cyccg->count) > CYCCG_I2C_REGISTER_MAP_SIZE) {
		err = -EINVAL;
		goto out;
	}

out:
	mutex_unlock(&cyccg->lock);
free:

	argv_free(data_arr);
	return err ? err : count;
}

/* Output value string in in hexadecimal format, no hexadecimal prefix. */
static ssize_t cyccg_show_formated_i2c(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	u8 data[CYCCG_I2C_REGISTER_MAP_SIZE];
	int count;
	int size;
	int i;
	int err;

	if (cyccg->offset < 0 || cyccg->count <= 0 ||
	    cyccg->offset > (CYCCG_I2C_REGISTER_MAP_SIZE - 1) ||
	    (cyccg->offset + cyccg->count) > CYCCG_I2C_REGISTER_MAP_SIZE) {
		return -EINVAL;
	}

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;

	err = cyccg_i2c_read(cyccg, data, cyccg->count, cyccg->offset);
	if (err)
		goto out;

	cyccg_dbg("%s: i2c read data:\n", __func__);
	mem_dump(CYCCG_DEBUG, data, cyccg->count);

	size = 0;
	for (i = 0; i < cyccg->count; i++) {
		if ((i != 0) && ((i % 16) == 0)) {
			count =	scnprintf(buf + size, PAGE_SIZE - size, "\n");
			size += count;
		}

		count =	scnprintf(buf + size, PAGE_SIZE - size,
				  "%02X ", data[i]);
		size += count;
	}
	count =	scnprintf(buf + size, PAGE_SIZE - size, "\n");
	size += count;

out:
	mutex_unlock(&cyccg->lock);

	return err ? err : size;
}

/*
 * The input data value must be in hexadecimal for and without hexadecimal
 * format prefix.
 */
static ssize_t cyccg_store_formated_i2c(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	char **data_arr;
	int data_arr_count;
	unsigned long byte_val;
	u8 data[CYCCG_I2C_REGISTER_MAP_SIZE];
	int i;
	int err = 0;

	data_arr = argv_split(GFP_KERNEL, buf, &data_arr_count);
	if (!data_arr || !data_arr_count)
		return -EINVAL;

	data_arr_count = data_arr_count > CYCCG_I2C_REGISTER_MAP_SIZE ?
			 CYCCG_I2C_REGISTER_MAP_SIZE : data_arr_count;
	data_arr_count = data_arr_count < cyccg->count ?
			 data_arr_count : cyccg->count;

	for (i = 0; i < data_arr_count; i++) {
		if (kstrtoul(data_arr[i], 16, &byte_val) || byte_val > 255)
			return -EINVAL;

		data[i] = byte_val;
	}

	cyccg_dbg("%s: i2c write count = %d\n", __func__, data_arr_count);
	mem_dump(CYCCG_DEBUG, data, data_arr_count);

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		goto out;

	err = cyccg_i2c_write(cyccg, data, data_arr_count, cyccg->offset);

	mutex_unlock(&cyccg->lock);
out:
	argv_free(data_arr);
	return err ? err : count;
}

//letv_pd s
static ssize_t cyccg_show_typec_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	int temp;
	temp = letv_pd_get_typec_temperature();
	ret = sprintf(buf, "%d\n", temp);
	return ret;
}

static ssize_t cyccg_store_typec_temp(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int temp;
	if (kstrtoint(buf, 10, &temp)) {
		letv_pd_set_typec_temperature(temp);
		return count;
	}
	return -EINVAL;
}
//letv_pd e

static int cyccg_enter_bootloader(struct cyccg *cyccg)
{
	struct device *dev = &cyccg->client->dev;
	struct hpi_msg resp_msg;
	u8 excepted_events[] = { CY_PD_RESP_RESET_COMPLETE, 0 };
	u8 cmd[32];
	int err;

	err = cyccg_hpi_get_device_mode(cyccg);
	if (err == 0) {
		/* Already in bootloader mode, early return. */
		return 0;
	} else if (err < 0) {
		dev_err(dev, "%s: failed to read device mode, (%d)\n",
			__func__, err);
		return err;
	}

	/*
	 * Move out of PD contract and Disable Type-C Interface.
	 * CCG does not generate interrupt and response for this command.
	 */
	cyccg_port_disable(cyccg);

	/* Jump to Bootloader mode. */
	cmd[0] = CY_PD_JUMP_TO_BOOT_CMD_SIG;
	err = cyccg_hpi_cmd_sync(cyccg, cmd, 1,
				 CY_PD_JUMP_TO_BOOT_REG_ADDR,
				 &resp_msg, excepted_events,
				 1000);
	cyccg_dbg("%s: jump to bootlaoder, err=%d, code=%#02x\n",
		  __func__, err, resp_msg.head.code);
	if (err || resp_msg.head.code != CY_PD_RESP_RESET_COMPLETE) {
		/*
		 * When jusp to bootlaoder, on slow platform the previous irq
		 * thread may clear the 0x80 event when CCG device enters
		 * bootloader mode befer the previous irq thread clear the
		 * previous event's irq flag.
		 */
		if (err == -ETIMEDOUT)
			goto check_dev_status;

		dev_err(dev,
			"%s: failed to enter bootloader, code: %#02x, (%d)\n",
			__func__, resp_msg.head.code, err);
		return -EIO;
	}

check_dev_status:
	/* Double check is already in bootloader mode. */
	err = cyccg_hpi_get_device_mode(cyccg);
	if (err != 0) {
		dev_err(dev, "%s: failed to enter bootloader, (%d)\n",
			__func__, err);
		return err < 0 ? err : -EAGAIN;
	}

	return 0;
}

static int cyccg_flash_row_write(struct cyccg *cyccg,
				 u8 *data, int size, int row_num)
{
	struct device *dev = &cyccg->client->dev;
	struct hpi_msg resp_msg;
	u8 cmd[32];
	int err;

	if (!data || size != 128)
		return -EINVAL;

	err = cyccg_i2c_write(cyccg, data, size,
			      CY_PD_REG_BOOTDATA_MEMEORY_ADDR);
	if (err) {
		dev_err(dev, "failed to update flash row data, row:%d, %d\n",
			row_num, err);
		return err;
	}

	cyccg->cmd_max_timeout = 500;
	cyccg->cmd_wait_internal = 5;
	cmd[0] = CY_PD_FLASH_READ_WRITE_CMD_SIG;
	cmd[1] = 1;
	put_unaligned_le16(row_num, &cmd[2]);
	err = cyccg_hpi_cmd_sync(cyccg, cmd, 4,
				 CY_PD_REG_FLASH_READ_WRITE_ADDR,
				 &resp_msg, NULL, 40);
	cyccg->cmd_max_timeout = 0;
	cyccg->cmd_wait_internal = 0;
	if (err || resp_msg.head.code != CY_PD_RESP_SUCCESS) {
		dev_err(dev, "%s, row: %d, code: 0x%02x, (%d)\n",
			"failed to write flash row data",
			row_num, resp_msg.head.code, err);
		return -EIO;
	}

	return 0;
}

#if 0  /* Tempalely not used, suppress compile warning. */
static int cyccg_flash_row_read(struct cyccg *cyccg,
				u8 *data, int size, int row_num)
{
	struct device *dev = &cyccg->client->dev;
	struct hpi_msg resp_msg;
	u8 cmd[32];
	int err;

	if (!data || size != 128)
		return -EINVAL;

	cmd[0] = CY_PD_FLASH_READ_WRITE_CMD_SIG;
	cmd[1] = 0;
	put_unaligned_le16(row_num, &cmd[2]);
	err = cyccg_hpi_cmd_sync(cyccg, cmd, 4,
				 CY_PD_REG_ENTER_FLASH_MODE_ADDR,
				 &resp_msg, NULL, 500);
	if (err || resp_msg.head.code != CY_PD_RESP_FLASH_DATA_AVAILABLE) {
		dev_err(dev, "%s, row:%d, code: 0x%02x, (%d)\n",
			"failed to read flash row data",
			row_num, resp_msg.head.code, err);
		return -EIO;
	}

	err = cyccg_i2c_read(cyccg, data, size,
			     CY_PD_REG_BOOTDATA_MEMEORY_ADDR);
	if (err) {
		dev_err(dev, "failed to read flash row data, row:%d, %d\n",
			row_num, err);
		return err;
	}

	return 0;
}
#endif

static u8 cyccg_checksum(const u8 *buf, size_t count)
{
	int i;
	u8 csum = 0;

	for (i = 0; i < count; i++)
		csum += buf[i];

	return (u8)((0xff - csum) + 1);
}

static int cyccg_validate_fw(struct cyccg *cyccg)
{
	struct device *dev = &cyccg->client->dev;
	struct hpi_msg resp_msg;
	u8 cmd[1];
	int err;

	cyccg->cmd_max_timeout = 10000;
	cyccg->cmd_wait_internal = 10;
	cmd[0] = 0x01;
	err = cyccg_hpi_cmd_sync(cyccg, cmd, 1,
				 CY_PD_REG_VALIDATE_FW_ADDR,
				 &resp_msg, NULL, 20);
	cyccg->cmd_max_timeout = 0;
	cyccg->cmd_wait_internal = 0;
	if (err || resp_msg.head.code != CY_PD_RESP_SUCCESS) {
		if (resp_msg.head.code == CY_PD_RESP_INVALID_FW) {
			dev_err(dev,
				"%s: failed to velidate FW, code:%#02x, (%d)\n",
				__func__, resp_msg.head.code, err);
			return -EINVAL;
		}

		dev_err(dev, "%s: VALIDATE_FW cmd error, code:%#02x, (%d)\n",
			__func__, resp_msg.head.code, err);
		return -EIO;
	}

	return 0;
}

/**
 * The @image_boundary is the firstly invalid address immediately
 * after the end of whole image.
 */
static struct cybin_row *cyccg_skip_carriage_get_row_data(
		struct cyccg *cyccg, const u8 *image, const u8 *image_boundary)
{
	struct device *dev = &cyccg->client->dev;
	struct cybin_row *row = NULL;

	if ((image + sizeof(struct cybin_row)) >= image_boundary)
		return NULL;

	if (image[0] == '\n' && image[1] == ':') {
		row = (struct cybin_row *)&image[1];
	} else if (image[0] == '\r' && image[1] == '\n' && image[2] == ':') {
		row = (struct cybin_row *)&image[2];
	} else {
		dev_err(dev, "%s: invalid row data[0-2]: %02x %02x %02x\n",
			__func__, image[0], image[1], image[2]);
	}

	return row;
}

/**
 * Check the new input firmware image is valid or not, and its version states.
 * Return values can be:
 *   >  0 : The input and device firmware image is different.
 *          1 - The input firmware image is newer then the device firmware.
 *          2 - The device firmware is newer than the input firmware image.
 *   == 0 : The new input firmware image vesrion is same as the the device
 *          firmare version.
 *          For the configuration image, always return 0 when passed the
 *          verificatioin.
 *   <  0 : Error encounter in firmware checking, stop update.
 */
static int cyccg_check_fw_version(struct cyccg *cyccg,
				  const u8 *image, size_t size)
{
	struct device *dev = &cyccg->client->dev;
	struct ccg_version image_ver;
	struct cybin_row *row;
	u16 image_silicon_id;
	int row_num;
	int row_size;
	int rows_count;
	u16 bl_last_row;
	u8 checksum;
	int err;

	memset(&image_ver, 0, sizeof(struct ccg_version));

	/* Check silicon id. */
	image_silicon_id = get_unaligned_be16(&image[0]);
	if (cyccg->silicon_id != image_silicon_id) {
		dev_err(dev, "image silicon id not match device silicon id\n");
		cyccg_dbg("%s: firmware image silicon id = %#04x\n",
			__func__, image_silicon_id);
		cyccg_dbg("%s: CCG2 device silicon id = %#04x\n",
			__func__, cyccg->silicon_id);
		return -EINVAL;
	}

	/* Check each row data of the firmware image. */
	err = cyccg_i2c_read(cyccg, (u8 *)&bl_last_row, 2,
			     CY_PD_BOOL_LOADER_LAST_ROW);
	if (err) {
		dev_err(dev, "failed to read bl last row number, (%d)\n", err);
		return err;
	}
	bl_last_row = get_unaligned_le16(&bl_last_row);

	cyccg_dbg_fw("\ncyccg: head + row data (4 plus):\n");
	mem_dump(CYCCG_DEBUG_FW, image,
		 sizeof(struct cybin_head) + sizeof(struct cybin_row) + 4);

	rows_count = 0;
	row = cyccg_skip_carriage_get_row_data(cyccg,
			image + sizeof(struct cybin_head), (image + size));
	while (row &&
	       ((u8 *)row + sizeof(struct cybin_row)) < (image + size)) {
		/* Validate the data before writing. */
		rows_count++;
		row_num = get_unaligned_be16(&row->row_num);
		row_size = get_unaligned_be16(&row->row_size);

		checksum = cyccg_checksum(&row->flash_array_id,
				(5 + CYBIN_FLASH_ROW_SIZE));

		cyccg_dbg_fw("%s: bl_last_row = %d, %#x\n", __func__,
			     bl_last_row, bl_last_row);
		cyccg_dbg_fw("%s: row_num = %d, %#x\n", __func__,
			     row_num, row_num);
		cyccg_dbg_fw("%s: row_size = %d, %#x\n", __func__,
			     row_size, row_size);
		cyccg_dbg_fw("%s: rows_count = %d\n", __func__, rows_count);
		cyccg_dbg_fw("%s: next row = 0x%p\n", __func__, row);
		cyccg_dbg_fw("%s: row+row_size = 0x%p\n", __func__,
			     ((u8 *)row + sizeof(struct cybin_row)));
		cyccg_dbg_fw("%s: image end = 0x%p\n", __func__,
			     (image + size));

		cyccg_dbg_fw("%s: checksum = %#02x\n", __func__, checksum);
		cyccg_dbg_fw("%s: row->checksum = %#02x\n", __func__,
			     row->checksum);
		mem_dump(CYCCG_DEBUG_FW, row->row_data, row_size);

		if (row->colon != ':' || row->flash_array_id != 0 ||
		    row_size != CYBIN_FLASH_ROW_SIZE ||
		    ((bl_last_row && (row_num <= bl_last_row)) ||
		     row_num > CYBIN_LAST_FLASH_ROW_NUM) ||
		     checksum != row->checksum) {
			dev_err(dev, "%s: invalid row data, count:%d\n",
				__func__, rows_count);
			return -EINVAL;
		}

		/*
		 * Old bootloader and firmware image starts from row 0x34 and
		 * the image version data locates at row 0x36 offset 0, 8 bytes.
		 * New bootloader and firmware image starts from row 0x24 and
		 * the image version data locates at row 0x26 offset 0, 8 bytes.
		 * Note, rows_count starts from 1.
		 */
		if (rows_count == 3 && (row_num == 0x0036 || row_num == 0x0026))
			cyccg_version_parse(&image_ver, &row->row_data[0]);

		if (row_num >= CYBIN_LAST_FLASH_ROW_NUM)
			break;

		cyccg_dbg_fw("\ncyccg: row data (4 plus):\n");
		mem_dump(CYCCG_DEBUG_FW, (u8 *)row + sizeof(struct cybin_row),
			 sizeof(struct cybin_row) + 4);

		row = cyccg_skip_carriage_get_row_data(cyccg,
				(u8 *)row + sizeof(struct cybin_row),
				(image + size));
	}

	if (rows_count == 4 && (row_num == 0x0023 || row_num == 0x0033)) {
		cyccg_dbg("%s: table image verified success.\n", __func__);
		return 0;
	}

	cyccg_dbg("%s: image bootloader FW version: %d.%d.%d.%d\n",
		__func__,
		image_ver.bl_maj_ver, image_ver.bl_min_ver,
		image_ver.bl_patch_ver, image_ver.bl_build_num);
	cyccg_dbg("%s: device bootloader FW version: %d.%d.%d.%d\n",
		__func__,
		cyccg->bl_ver.bl_maj_ver, cyccg->bl_ver.bl_min_ver,
		cyccg->bl_ver.bl_patch_ver, cyccg->bl_ver.bl_build_num);

	cyccg_dbg("%s: image app FW version: %d.%d,%d,%c%c\n",
		__func__,
		image_ver.app_maj_ver, image_ver.app_min_ver,
		image_ver.app_ext_ver, image_ver.app_name[0],
		image_ver.app_name[1]);
	cyccg_dbg("%s: device app FW version: %d.%d,%d,%c%c\n",
		__func__,
		cyccg->app_ver.app_maj_ver, cyccg->app_ver.app_min_ver,
		cyccg->app_ver.app_ext_ver, cyccg->app_ver.app_name[0],
		cyccg->app_ver.app_name[1]);

#if CYCCG_ENABLE_FW_IMAGE_APP_NAME_CHECK
	/*
	 * Check firmware image app_name.
	 * If the app name is different, normally, the firmware should be aimed
	 * for different device type, so it should not be used.
	 * But for common firmware and at the initial development, the app name
	 * always keep same, or the 'nb' is used for 'mb' devices.
	 * So just enable this code when make sure the app_name is consistent
	 * and meaningful.
	 * By default, this code is disabled.
	 */
	if (memcmp(cyccg->app_ver.app_name, image_ver.app_name, 2)) {
		dev_err(dev, "%s: image FW Application name: %c%c not match\n",
				__func__, image_ver.app_name[0],
				image_ver.app_name[1]);
		dev_err(dev, "%s: Device's FW Application name: %c%c\n",
				__func__, cyccg->app_ver.app_name[0],
				cyccg->app_ver.app_name[1]);
		return -EINVAL;
	}
#endif

	if (image_ver.app_maj_ver > cyccg->app_ver.app_maj_ver)
		return 1;  /* Input version is newer. */
	else if (image_ver.app_maj_ver < cyccg->app_ver.app_maj_ver)
		return 2;  /* Device version is newer. */

	if (image_ver.app_min_ver == cyccg->app_ver.app_min_ver)
		return 0;  /* Input and device versions are same. */
	else if (image_ver.app_min_ver > cyccg->app_ver.app_min_ver)
		return 1;  /* Input version is newer. */
	else
		return 2;  /* Device version is newer. */
}

/*
 * The last flash row 0xFF is always the metadata row, so if the last flash
 * row 0xff is not included in the image, then the image is not intented to
 * update the metadata flash fow. Such as, only the configuration table image.
 */
static bool cyccg_is_last_metadata_row_contained(const u8 *image, size_t size)
{
	int offset = size - sizeof(struct cybin_row) - 3;
	struct cybin_row *row = (struct cybin_row *)&image[offset];
	u16 last_row_num = 0;

	do {
		if (row->colon == ':' && row->flash_array_id == 0) {
			last_row_num = get_unaligned_be16(&row->row_num);
			break;
		}

		row = (struct cybin_row *)&image[offset++];
	} while ((offset + sizeof(struct cybin_row)) < size);

	if (last_row_num == 0x00ff)
		return true;
	return false;
}

static int cyccg_update_fw(struct cyccg *cyccg,
			   const u8 *image, size_t size)
{
	struct device *dev = &cyccg->client->dev;
	struct cybin_row *row;
	int row_num;
	int row_size;
	struct hpi_msg resp_msg;
	u8 data[CYBIN_FLASH_ROW_SIZE];
	u16 bl_last_row;
	int err;

	/* 1. Enter bootloader mode. */
	err = cyccg_enter_bootloader(cyccg);
	if (err) {
		dev_err(dev, "failed to enter bootloader mode.\n");
		return err;
	}

	/* Read bootloader last row number for image row verification. */
	err = cyccg_i2c_read(cyccg, (u8 *)&bl_last_row, 2,
			     CY_PD_BOOL_LOADER_LAST_ROW);
	if (err) {
		dev_err(dev, "failed to read bl last row number, (%d)\n", err);
		return err;
	}
	bl_last_row = get_unaligned_le16(&bl_last_row);

	/* 2. Enter flashing mode. */
	data[0] = CY_PD_ENTER_FLASHING_MODE_CMD_SIG;
	err = cyccg_hpi_cmd_sync(cyccg, data, 1,
				 CY_PD_REG_ENTER_FLASH_MODE_ADDR,
				 &resp_msg, NULL, 1000);
	if (err || resp_msg.head.code != CY_PD_RESP_SUCCESS) {
		dev_err(dev,
			"failed to enter flashing mode, code: 0x%02x, (%d)\n",
			resp_msg.head.code, err);
		return -EIO;
	}

	/*
	 * Clear the metadata table to avoid any possible checksum miss-check.
	 * Skip with it when just updating configration table.
	 */
	if (cyccg_is_last_metadata_row_contained(image, size)) {
		cyccg_dbg_irq("%s: clear metadata table, flash row: %d, %#x\n",
			__func__, CYBIN_LAST_FLASH_ROW_NUM,
			CYBIN_LAST_FLASH_ROW_NUM);
		memset(data, 0, CYBIN_FLASH_ROW_SIZE);
		err = cyccg_flash_row_write(cyccg, data, CYBIN_FLASH_ROW_SIZE,
					    CYBIN_LAST_FLASH_ROW_NUM);
		if (err) {
			cyccg_dbg_irq(
				"%s: failed to clear metadata table, %d\n",
				__func__, err);
			return err;
		}
	}
	/* Do firmware image update. */
	row = cyccg_skip_carriage_get_row_data(cyccg,
			image + sizeof(struct cybin_head), (image + size));
	while (row &&
		((u8 *)row + sizeof(struct cybin_row)) < (image + size)) {
		row_num = get_unaligned_be16(&row->row_num);
		row_size = get_unaligned_be16(&row->row_size);

		/* Write the flash row data. */
		cyccg_dbg_irq("%s: write flash row: %d, %#x\n", __func__,
			      row_num, row_num);
		err = cyccg_flash_row_write(cyccg, row->row_data, row_size,
					    row_num);
		if (err)
			return err;

		if (row_num >= CYBIN_LAST_FLASH_ROW_NUM)
			break;

		row = cyccg_skip_carriage_get_row_data(cyccg,
				(u8 *)row + sizeof(struct cybin_row),
				(image + size));
	}

	/* Command bootloader to validate the just updated firmware image. */
	err = cyccg_validate_fw(cyccg);
	if (err) {
		dev_err(dev,
			"%s: failed to validate written firmare image, (%d)\n",
			__func__, err);
		return err;
	}

	cyccg_dbg("%s: write fw image done and validated successfully.\n",
		  __func__);
	return 0;
}

static ssize_t cyccg_store_update_fw(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	const struct firmware *fw;
	char fw_name[NAME_MAX];
	int ret = 0;
	int err;

	if (count >= NAME_MAX) {
		dev_err(dev, "file name too long\n");
		return -EINVAL;
	}

	memcpy(fw_name, buf, count);
	if (fw_name[count - 1] == '\n')
		fw_name[count - 1] = '\0';
	else
		fw_name[count] = '\0';

#if CYCCG_AUTO_INTERNAL_UPDATE
	if (!mutex_trylock(&cyccg->fw_lock)) {
		cyccg_dbg("%s: fw_lock trylock failed, busyings.\n", __func__);
		return -EBUSY;
	}
#endif

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		goto fw_unlock;

	cyccg_dbg("%s: <<<<: start of fw update process.\n", __func__);

	atomic_inc(&cyccg->is_irq_owned_by_drv);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		enable_irq(cyccg->client->irq);
#endif

	err = request_firmware(&fw, fw_name, dev);
	if (err) {
		dev_err(dev, "failed to load firmware image: %s, (%d)\n",
			fw_name, err);
		goto unlock;
	}

	ret = cyccg_check_fw_version(cyccg, fw->data, fw->size);
	if (ret < 0) {
		dev_err(dev, "failed to check firmware image, (%d)\n", ret);
		err = ret;
		ret = 0;
		goto free_fw;
	}

	/* Always do update when manually through the update_fw interface. */
	err = cyccg_update_fw(cyccg, fw->data, fw->size);
	if (err)
		dev_err(dev, "%s: failed to update firmware image: %s, (%d)\n",
			__func__, fw_name, err);

	/* Reset to re-initialize the device into the operational mode. */
	cyccg_dbg("%s: start to re-initialize the CCG device.\n", __func__);
	ret = cyccg_device_init(cyccg, true);
	if (ret)
		dev_err(dev, "%s: failed to reinitialize the device, (%d)\n",
			__func__, ret);

free_fw:
	release_firmware(fw);

unlock:
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		disable_irq(cyccg->client->irq);
#endif
	atomic_dec(&cyccg->is_irq_owned_by_drv);
	mutex_unlock(&cyccg->lock);
fw_unlock:
#if CYCCG_AUTO_INTERNAL_UPDATE
	mutex_unlock(&cyccg->fw_lock);
#endif

	cyccg_dbg("%s: >>>>: end of fw update, err=%d, ret=%d\n", __func__,
		  err, ret);
	return err ? err : count;
}
#if CYCCG_AUTO_INTERNAL_UPDATE
static int cyccg_request_firmware(struct cyccg *cyccg,
				  const struct firmware **fw_p, char *name)
{
	struct device *dev = &cyccg->client->dev;
	int retries;
	int err = -ENOENT;

	retries = 60;  /* Max 30 seconds. */
	while (retries-- > 0) {
		err = request_firmware(fw_p, name, dev);
		if (!err)
			break;

		if (retries) {
			cyccg_dbg("%s: load fw image %s failed (%d). Retry!\n",
				  __func__, name, err);
			if (err == -ENOENT) {
				/*
				 * Wait for the firmware load modules ready
				 * when CONFIG_FW_LOADER_USER_HELPER is not set
				 * and the CONFIG_CYCCG is set to y.
				 *
				 * Note, normally, the firmware image and
				 * the table image should be always there
				 * in the system.
				 */
				msleep(500);
			}
			continue;
		}
	}

	return err;
}

static int cyccg_auto_fw_update_thread_handler(struct cyccg *cyccg)
{
	struct device *dev = &cyccg->client->dev;
	const struct firmware *fw;
	const struct firmware *table;
	char fw_name[NAME_MAX];
	char table_name[NAME_MAX];
	u8 device_mode;
	int retries;
	int ret = 0;
	int err;

	if (strlen(CYCCG_FW_NAME) >= NAME_MAX ||
	    strlen(CYCCG_TABLE_NAME) >= NAME_MAX) {
		dev_err(dev, "file name too long\n");
		return -EINVAL;
	}

	strcpy(fw_name, CYCCG_FW_NAME);
	strcpy(table_name, CYCCG_TABLE_NAME);

	mutex_lock(&cyccg->fw_lock);
	err = cyccg_request_firmware(cyccg, &fw, fw_name);
	if (err) {
		if (err == -ENOENT) {
			cyccg_dbg("%s: no fw image %s found. Return early.\n",
				  __func__, fw_name);
			err = 0;
		} else {
			dev_err(dev, "failed to load fw image: %s, (%d)\n",
				fw_name, err);
		}
		goto fw_unlock;
	}

	err = cyccg_request_firmware(cyccg, &table, table_name);
	if (err) {
		if (err == -ENOENT) {
			cyccg_dbg("%s: no tbl image %s found. Return early.\n",
				  __func__, table_name);
			err = 0;
		} else {
			dev_err(dev, "failed to load table image: %s, (%d)\n",
				table_name, err);
		}
		goto free_fw;
	}

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		goto free_table;

	cyccg_dbg("%s: <<<<: start of fw update process.\n", __func__);

	atomic_inc(&cyccg->is_irq_owned_by_drv);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		enable_irq(cyccg->client->irq);
#endif

	ret = cyccg_check_fw_version(cyccg, table->data, table->size);
	if (ret < 0) {
		dev_err(dev, "failed to check table image, (%d)\n", ret);
		err = ret;
		goto unlock;
	}
	ret = cyccg_check_fw_version(cyccg, fw->data, fw->size);
	if (ret < 0) {
		dev_err(dev, "failed to check firmware image, (%d)\n", ret);
		err = ret;
		goto unlock;
	}
	if (ret != 1) {
		if (ret == 0)
			cyccg_dbg("%s: input fw is same as device fw. %s.\n",
				  __func__, "Return early");
		else
			cyccg_dbg("%s: input fw old than device fw. %s.\n",
				  __func__, "Return early");
		ret = 0;
		/*
		 * If in bootloader mode, the image may invalid, but
		 * the version information is same or old, so force update.
		 */
		if (cyccg_i2c_read(cyccg, &device_mode, 1,
				   CY_PD_REG_DEVICE_MODE_ADDR))
			device_mode = 0;
		if (device_mode)
			goto unlock;
	}

	cyccg_dbg("%s: <<<<<<<< start to update fw image: %s\n",
		  __func__, fw_name);
	retries = 3;
	while (retries-- > 0) {
		err = cyccg_update_fw(cyccg, fw->data, fw->size);
		if (!err)
			break;

		if (retries)
			dev_err(dev, "%s: update fw failed: %s, (%d). Retry!\n",
				__func__, fw_name, err);
	}
	cyccg_dbg("%s: >>>>>>>> update fw image %s: %s, (%d).\n",
		  __func__, fw_name, err ? "failed" : "success", err);
	cyccg_dbg("%s: <<<<<<<< start to update table image: %s\n",
		  __func__, table_name);
	retries = 3;
	while (retries-- > 0) {
		err = cyccg_update_fw(cyccg, table->data, table->size);
		if (!err)
			break;

		if (retries)
			dev_err(dev,
				"%s: update table failed: %s, (%d). Retry!\n",
				__func__, table_name, err);
	}
	cyccg_dbg("%s: >>>>>>>> update table image %s: %s, (%d).\n",
		  __func__, table_name, err ? "failed" : "success", err);
	/* Reset to re-initialize the device into the operational mode. */
	cyccg_dbg("%s: start to re-initialize the CCG device.\n", __func__);
	ret = cyccg_device_init(cyccg, true);
	if (ret)
		dev_err(dev, "%s: failed to reinitialize the device, (%d)\n",
			__func__, ret);

unlock:
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		disable_irq(cyccg->client->irq);
#endif
	atomic_dec(&cyccg->is_irq_owned_by_drv);
	mutex_unlock(&cyccg->lock);
free_table:
	release_firmware(table);
free_fw:
	release_firmware(fw);
fw_unlock:
	mutex_unlock(&cyccg->fw_lock);

	cyccg_dbg("%s: >>>>: end of fw update, err=%d, ret=%d\n",
		  __func__, err, ret);
	return err;
}

static void cyccg_auto_fw_update_worker(struct work_struct *work)
{
	struct cyccg *cyccg =
		container_of(work, struct cyccg, auto_fw_update_work);

	cyccg_dbg("%s: <<<<: start\n", __func__);
	cyccg_auto_fw_update_thread_handler(cyccg);
	cyccg_dbg("%s: >>>>: end\n", __func__);
}

static void cyccg_queue_auto_fw_update_thread_work(struct cyccg *cyccg)
{
	mutex_lock(&cyccg->auto_fw_update_work_lock);
	if (cyccg->auto_fw_update_work_state == CYCCG_WORK_NONE) {
		cyccg->auto_fw_update_work_state = CYCCG_WORK_QUEUED;
		schedule_work(&cyccg->auto_fw_update_work);

		cyccg_dbg("%s: internal auto fw update thread work queued.\n",
			  __func__);
	} else {
		cyccg_dbg("%s: current auto_fw_udpate_thread_work_state = %d\n",
			  __func__, cyccg->auto_fw_update_work_state);
	}
	mutex_unlock(&cyccg->auto_fw_update_work_lock);
}
#endif


static ssize_t cyccg_show_bl_fw_ver(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	struct ccg_version *ver = &cyccg->bl_ver;
	int size;
	int err;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	size = scnprintf(buf, PAGE_SIZE, "%d.%d.%d.%d\n",
			 ver->bl_maj_ver, ver->bl_min_ver,
			 ver->bl_patch_ver, ver->bl_build_num);
	mutex_unlock(&cyccg->lock);

	return size;
}

static ssize_t cyccg_show_app_fw_ver(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	struct ccg_version *ver = &cyccg->app_ver;
	int size;
	int err;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	size = scnprintf(buf, PAGE_SIZE, "%d.%d,%d,%c%c\n",
			 ver->app_maj_ver, ver->app_min_ver,
			 ver->app_ext_ver,
			 ver->app_name[0], ver->app_name[1]);
	mutex_unlock(&cyccg->lock);

	return size;
}

static ssize_t cyccg_show_silicon_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int size;
	int err;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	size = scnprintf(buf, PAGE_SIZE, "%#04x\n", cyccg->silicon_id);
	mutex_unlock(&cyccg->lock);

	return size;
}

//letv_pd s
static int cyccg_port_reset(struct cyccg *cyccg)
{
		int err;

		err = mutex_lock_interruptible(&cyccg->lock);
		if (err)
			return err;
		atomic_inc(&cyccg->is_irq_owned_by_drv);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
		if (!cyccg->async_queue)
			enable_irq(cyccg->client->irq);
#endif

		/* Reset to re-initialize the device into the operational mode. */
		err = cyccg_device_init(cyccg, true);
		if (err)
			pr_err("%s: failed to reinitialize the device, (%d)\n",
				__func__, err);

#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
		if (!cyccg->async_queue)
			disable_irq(cyccg->client->irq);
#endif
		atomic_dec(&cyccg->is_irq_owned_by_drv);
		mutex_unlock(&cyccg->lock);

		return err ? err : 0;
}
//letv_pd e
/* Test purpose: This interface is supplied to reset the device easily. */
static ssize_t cyccg_store_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	int err;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	atomic_inc(&cyccg->is_irq_owned_by_drv);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		enable_irq(cyccg->client->irq);
#endif

	/* Reset to re-initialize the device into the operational mode. */
	err = cyccg_device_init(cyccg, true);
	if (err)
		dev_err(dev, "%s: failed to reinitialize the device, (%d)\n",
			__func__, err);

#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		disable_irq(cyccg->client->irq);
#endif
	atomic_dec(&cyccg->is_irq_owned_by_drv);
	mutex_unlock(&cyccg->lock);

	return err ? err : count;
}

static ssize_t cyccg_store_role_swap(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	char **data_arr;
	int data_arr_count;
	unsigned long swap_cmd;
	unsigned long swap_flags;
	struct cyccg_ext_ap_cmd ext_cmd;
	int err;

	data_arr = argv_split(GFP_KERNEL, buf, &data_arr_count);
	if (!data_arr || !data_arr_count)
		return -EINVAL;

	swap_flags = 0;
	if (kstrtoul(data_arr[0], 16, &swap_cmd))
		return -EINVAL;

	if (data_arr_count > 1 && kstrtoul(data_arr[1], 16, &swap_flags))
		return -EINVAL;

	memset(&ext_cmd, 0, sizeof(ext_cmd));
	ext_cmd.cmd = (u32)swap_cmd;
	ext_cmd.flags = (u32)swap_flags;

	cyccg_dbg("%s: do swap cmd: 0x%02x, flags=0x%02x\n", __func__,
		 ext_cmd.cmd, ext_cmd.flags);

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	atomic_inc(&cyccg->is_irq_owned_by_drv);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		enable_irq(cyccg->client->irq);
#endif

	err = cyccg_swap_cmd_process(cyccg, &ext_cmd, NULL);

#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		disable_irq(cyccg->client->irq);
#endif
	atomic_dec(&cyccg->is_irq_owned_by_drv);
	mutex_unlock(&cyccg->lock);

	argv_free(data_arr);
	return err ? -EIO : count;

}

static int cyccg_write_vdm_data(struct cyccg *cyccg,
				u8 vdm_mode, u8 *vdm_data, int count)
{
	struct hpi_msg resp;
	u8 cmd[32];
	int err;

	if (!vdm_data || !count || count % 4 || count > 28) {
		cyccg_dbg("%s: invalid input, vdm buffer=0x%p, buf count=%d\n",
			  __func__, vdm_data, count);
		return -EINVAL;
	}

	err = cyccg_i2c_write(cyccg, vdm_data, count,
			      CY_PD_REG_FWDATA_MEMEORY_ADDR);
	if (err) {
		cyccg_dbg("%s: failed to write vdm data, (%d)\n",
			  __func__, err);
		return err;
	}

	cmd[0] = vdm_mode;  /* VDM mode, 0x00-SOP, 0x01-SOP', 0x02-SOP'' */
	cmd[1] = count;  /* VDM data length */
	err = cyccg_hpi_cmd_sync(cyccg, cmd, 2, CY_PD_REG_U_VDM_CTRL_ADDR,
				 &resp, NULL, 500);
	if (err || resp.head.code != CY_PD_RESP_SUCCESS) {
		cyccg_dbg("%s: failed to write vdm data, code: 0x%02x, (%d)\n",
			  __func__, resp.head.code, err);
		return err;
	}

	cyccg_dbg("%s: VDM data sent success.\n", __func__);
	return 0;
}

static int cyccg_send_vdm_data(struct cyccg *cyccg, u8 *vdm_data, int count)
{
	int err;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	atomic_inc(&cyccg->is_irq_owned_by_drv);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		enable_irq(cyccg->client->irq);
#endif

	/* VDM mode, 0x00-SOP, 0x01-SOP', 0x02-SOP'' */
	err = cyccg_write_vdm_data(cyccg, 0x00, vdm_data, count);

#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		disable_irq(cyccg->client->irq);
#endif
	atomic_dec(&cyccg->is_irq_owned_by_drv);
	mutex_unlock(&cyccg->lock);

	return err;
}

static ssize_t cyccg_store_vdm(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	char **data_arr;
	int data_arr_count;
	unsigned long byte_val;
	u8 data[CYCCG_I2C_REGISTER_MAP_SIZE];
	int i;
	int err = 0;

	data_arr = argv_split(GFP_KERNEL, buf, &data_arr_count);
	if (!data_arr || !data_arr_count) {
		cyccg_dbg("%s: invalid input vdm data parameters.\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < data_arr_count; i++) {
		if (kstrtoul(data_arr[i], 16, &byte_val) || byte_val > 255)
			return -EINVAL;

		data[i] = byte_val;
	}

	cyccg_dbg("%s: vdm data (%d)\n", __func__, data_arr_count);
	mem_dump(CYCCG_DEBUG, data, data_arr_count);

	err = cyccg_send_vdm_data(cyccg, data, data_arr_count);

	argv_free(data_arr);
	return err ? err : count;
}

#if CYCCG_DEBUG
static ssize_t cyccg_store_vdm_work_test(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);

	cyccg_dbg("%s: start VDM work.\n", __func__);
	cyccg_queue_vdm_work(cyccg);
	return count;
}

static ssize_t cyccg_show_ccg_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	struct ccg_status ccg_status;
	int size;
	int err;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	mutex_lock(&cyccg->ccg_status_lock);
	cyccg_dbg("%s: dump ccg_status:\n", __func__);
	err = cyccg_get_ccg_status(cyccg, &ccg_status);
	mutex_unlock(&cyccg->ccg_status_lock);
	if (err)
		cyccg_dbg("%s: failed to get ccg_status.\n", __func__);
	else {
		size = scnprintf(buf, PAGE_SIZE,
			"TYPE_C_STATUS (0x%02x):\n", ccg_status.type_c_status);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"	 port_connected: %s\n",
			ccg_status.port_connected ? "Yes" : "No");
		if (ccg_status.port_connected) {
			size += scnprintf(buf + size, PAGE_SIZE - size,
				"	 attached_dev_type: %s\n",
				dev_type[ccg_status.attached_dev_type]);
			size += scnprintf(buf + size, PAGE_SIZE - size,
				"	 port connected to %s\n",
				(ccg_status.CC_polarity ==
					CCG_STATUS_CC_POLARITY_CC1) ?
					"CC1" : "CC2");
			size += scnprintf(buf + size, PAGE_SIZE - size,
				"	 type_c_current_level: %s\n",
				type_c_current_level[
					ccg_status.type_c_current_level]);
			size += scnprintf(buf + size, PAGE_SIZE - size,
				"	 Ra_status: Ra %s\n",
				ccg_status.Ra_status ? "detected" :
						       "not detected");
		}
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"PD_STATUS (0x%08x):\n", ccg_status.pd_status);
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"	 Contract Established: %s\n",
			ccg_status.contract_status ? "Yes" : "No");
		if (ccg_status.contract_status) {
			size += scnprintf(buf + size, PAGE_SIZE - size,
				"	 current_port_power_role: %s\n",
				ccg_status.current_port_power_role ?
					"Source" : "Sink");
			size += scnprintf(buf + size, PAGE_SIZE - size,
				"	 current_port_data_role: %s\n",
				ccg_status.current_port_data_role ?
					"DFP" : "UFP");
		}
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"	 EMCA_present: %s\n",
			ccg_status.EMCA_present ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"	 VCONN_supplier: %s\n",
			ccg_status.VCONN_supplier ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"	 VCONN_status: %s\n",
			ccg_status.VCONN_status ? "Souring" : "Not Souring");
	}
	mutex_unlock(&cyccg->lock);

	return err ? err : size;
}
#endif

static ssize_t cyccg_show_vconn_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	struct ccg_status ccg_status;
	int size;
	int err;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	mutex_lock(&cyccg->ccg_status_lock);
	cyccg_dbg("%s: dump ccg_status:\n", __func__);
	err = cyccg_get_ccg_status(cyccg, &ccg_status);
	mutex_unlock(&cyccg->ccg_status_lock);
	if (err)
		cyccg_dbg("%s: failed to get ccg_status.\n", __func__);
	else {
		size = scnprintf(buf, PAGE_SIZE,
			"VCONN_supplier: %s\n",
			ccg_status.VCONN_supplier ? "Yes" : "No");
		size += scnprintf(buf + size, PAGE_SIZE - size,
			"VCONN_status: %s\n",
			ccg_status.VCONN_status ? "Souring" : "Not Souring");
	}
	mutex_unlock(&cyccg->lock);

	return err ? err : size;
}

static int cyccg_hpi_set_vconn_status(struct cyccg *cyccg, bool on)
{
	struct ccg_status ccg_status;
	int err;

	err = mutex_lock_interruptible(&cyccg->lock);
	if (err)
		return err;
	atomic_inc(&cyccg->is_irq_owned_by_drv);
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		enable_irq(cyccg->client->irq);
#endif

	cyccg_dbg("%s: set vconn: %s\n", __func__, on ? "ON" : "OFF");

	/* Send switch VCONN of/off command to CCG device. */
	err = cyccg_hpi_pd_control(cyccg,
			on ? CY_PD_CONTROL_VCON_ON : CY_PD_CONTROL_VCON_OFF);
	cyccg_dbg("%s: PD_CONTROL set vconn on/off resp: %d\n", __func__, err);
	if (err) {
		/* Check if type-c port is connected. */
		mutex_lock(&cyccg->ccg_status_lock);
		cyccg_dbg("%s: get ccg_status:\n", __func__);
		err = cyccg_get_ccg_status(cyccg, &ccg_status);
		mutex_unlock(&cyccg->ccg_status_lock);
		if (err) {
			cyccg_dbg("%s: failed to get ccg_status, (%d)\n",
				  __func__, err);
			err = -EIO;
			goto out;
		}
		if (!ccg_status.contract_status) {
			cyccg_dbg("%s: Type C port is not connected.\n",
				  __func__);
			err = -ENOTCONN;
			goto out;
		}

		cyccg_dbg("%s: switch VCONN on/off not support.\n", __func__);
		err = -ENOTSUPP;
		goto out;
	}

out:
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	if (!cyccg->async_queue)
		disable_irq(cyccg->client->irq);
#endif
	atomic_dec(&cyccg->is_irq_owned_by_drv);
	mutex_unlock(&cyccg->lock);

	return err;
}
void cyccg_set_vconn(bool on)
{
	int ret = 0;

	if (!g_cyccg)
		return;

	if(cyccg_pdata && !cyccg_pdata->external_5v_en_flag) {
		if (IS_ERR_OR_NULL(g_cyccg->boost_5v)) {
			g_cyccg->boost_5v = devm_regulator_get(&g_cyccg->client->dev,
					    "pmi8994_boost_5v");
			if (IS_ERR_OR_NULL(g_cyccg->boost_5v)) {
				pr_info("fail to get boost_5v regulator\n");
				return;
			}
		}

		if (on)
			ret = regulator_enable(g_cyccg->boost_5v);
		else
			ret = regulator_disable(g_cyccg->boost_5v);
	}

	if (!ret)
		ret = cyccg_hpi_set_vconn_status(g_cyccg, on);

	/*
	 * If cyccg_hpi_set_vconn_status() returns an error
	 * code, it's beacuse Type-C device already disconnected
	 */
	if (ret)
		cclogic_set_vbus(0);
}
EXPORT_SYMBOL(cyccg_set_vconn);

static ssize_t cyccg_store_vconn_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cyccg *cyccg = dev_get_drvdata(dev);
	char on_off[10];
	bool on;
	int err;

	if (count >= 10) {
		dev_err(dev, "invalid vconn status set value: %s\n", buf);
		return -EINVAL;
	}
	memcpy(on_off, buf, count);
	if (on_off[count - 1] == '\n')
		on_off[count - 1] = '\0';
	else
		on_off[count] = '\0';
	if (!strcmp(on_off, "1") || !strcmp(on_off, "on") ||
	    !strcmp(on_off, "enable")) {
		on = true;
	}else if (!strcmp(on_off, "0") || !strcmp(on_off, "off") ||
		 !strcmp(on_off, "disable")) {
		on = false;
	} else {
		dev_err(dev, "unknown vconn status set value: %s\n", on_off);
		return -EINVAL;
	}

	cyccg_dbg("%s: set VCONN status: %s\n", __func__, on ? "ON" : "OFF");

	err = cyccg_hpi_set_vconn_status(cyccg, on);

	return err ? err : count;
}

static DEVICE_ATTR(char_dev_name, S_IRUGO, cyccg_show_char_dev_name, NULL);
static DEVICE_ATTR(device_mode, S_IRUGO, cyccg_show_device_mode, NULL);
static DEVICE_ATTR(i2c_offset_count, S_IRUGO | S_IWUSR | S_IWGRP,
	cyccg_show_i2c_offset_count, cyccg_store_i2c_offset_count);
static DEVICE_ATTR(formated_i2c, S_IRUGO | S_IWUSR | S_IWGRP,
	cyccg_show_formated_i2c, cyccg_store_formated_i2c);
static DEVICE_ATTR(update_fw, S_IWUSR | S_IWGRP, NULL, cyccg_store_update_fw);
static DEVICE_ATTR(bl_fw_ver, S_IRUGO, cyccg_show_bl_fw_ver, NULL);
static DEVICE_ATTR(app_fw_ver, S_IRUGO, cyccg_show_app_fw_ver, NULL);
static DEVICE_ATTR(silicon_id, S_IRUGO, cyccg_show_silicon_id, NULL);
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, cyccg_store_reset);
static DEVICE_ATTR(role_swap, S_IWUSR | S_IWGRP, NULL, cyccg_store_role_swap);
static DEVICE_ATTR(send_vdm, S_IWUSR | S_IWGRP, NULL, cyccg_store_vdm);
#if CYCCG_DEBUG
static DEVICE_ATTR(vdm_work_test, S_IWUSR | S_IWGRP,
		   NULL, cyccg_store_vdm_work_test);
static DEVICE_ATTR(ccg_status, S_IRUGO, cyccg_show_ccg_status, NULL);
//letv_pd s
static DEVICE_ATTR(typec_temp, S_IRUGO | S_IWUSR | S_IWGRP,
							cyccg_show_typec_temp, cyccg_store_typec_temp);
//letv_pd e
#endif
static DEVICE_ATTR(vconn_status, S_IRUGO | S_IWUSR | S_IWGRP,
	cyccg_show_vconn_status, cyccg_store_vconn_status);

static struct attribute *cyccg_sysfs_entries[] = {
	&dev_attr_char_dev_name.attr,
	&dev_attr_device_mode.attr,
	&dev_attr_i2c_offset_count.attr,
	&dev_attr_formated_i2c.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_bl_fw_ver.attr,
	&dev_attr_app_fw_ver.attr,
	&dev_attr_silicon_id.attr,
	&dev_attr_reset.attr,
	&dev_attr_role_swap.attr,
	&dev_attr_send_vdm.attr,
#if CYCCG_DEBUG
	&dev_attr_vdm_work_test.attr,
	&dev_attr_ccg_status.attr,
//letv_pd s
	&dev_attr_typec_temp.attr,
//letv_pd e
#endif
	&dev_attr_vconn_status.attr,
	NULL,
};

static const struct attribute_group cyccg_sysfs_group = {
	.attrs = cyccg_sysfs_entries,
};

static void cyccg_remove_sysfs_group(void *data)
{
	struct cyccg *cyccg = data;

	sysfs_remove_group(&cyccg->client->dev.kobj, &cyccg_sysfs_group);
}

static int cyccg_init_gpio(struct cyccg *cyccg, int init_gpio)
{
	int ret = 0;

	if(true == init_gpio){
	    pr_info("%s %s: init gpio\n", LOG_TAG, __func__);
	}
	else{
	    pr_info("%s %s: release gpio\n", LOG_TAG, __func__);
	    goto err3;
	}

	/*  gpio for chip reset  */
	ret = gpio_request(cyccg->pdata->gpio_reset, "cyccg_reset_n");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       cyccg->pdata->gpio_reset);
		goto err0;
	}
    /*reset need first pull low after 10ms then pull up*/
	//gpio_direction_output(cyccg->pdata->gpio_reset, 0);
    //mdelay(10);
	gpio_direction_output(cyccg->pdata->gpio_reset, 1);

	mdelay(30);

	/*  gpio for chip interface communaction */
	ret = gpio_request(cyccg->pdata->gpio_intr_comm, "cyccg_intr_comm");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       cyccg->pdata->gpio_intr_comm);
		goto err1;
	}
    /*interrupt need HIGH and Input*/
	gpio_direction_input(cyccg->pdata->gpio_intr_comm);
	/*  gpio for uart switch */
	ret = gpio_request(cyccg->pdata->gpio_uart_sw, "cyccg_uart_sw");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
				cyccg->pdata->gpio_uart_sw);
		goto err2;
	}
	gpio_direction_output(cyccg->pdata->gpio_uart_sw, 1);

	/*  gpio for audio D+/1 select */
	ret = gpio_request(cyccg->pdata->gpio_ad_sel, "cyccg_ad_sel");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
				cyccg->pdata->gpio_ad_sel);
		goto err3;
	}
	gpio_direction_output(cyccg->pdata->gpio_ad_sel, 0);

	/*  gpio for external_5v_en_gpio 1 select */
	if (cyccg->pdata->external_5v_en_flag) {
		ret = gpio_request(cyccg->pdata->external_5v_en_gpio, "external_5v_en_gpio");
		if (ret) {
			pr_err("%s : failed to request gpio %d\n", __func__,
					cyccg->pdata->external_5v_en_gpio);
			goto err4;
		}
		gpio_direction_output(cyccg->pdata->external_5v_en_gpio, 1);
		gpio_set_value(cyccg->pdata->external_5v_en_gpio, 1);
	}

	goto out;

err4:
	gpio_free(cyccg->pdata->external_5v_en_gpio);
err3:
	gpio_free(cyccg->pdata->gpio_ad_sel);
err2:
	gpio_free(cyccg->pdata->gpio_uart_sw);
err1:
	gpio_free(cyccg->pdata->gpio_intr_comm);
err0:
	gpio_free(cyccg->pdata->gpio_reset);

	return 1;
out:
	return 0;
}

static int cyccg_parse_dt(struct device *dev, struct cyccg_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->gpio_reset =
	    of_get_named_gpio_flags(np, "reset-gpio", 0, NULL);
	pdata->gpio_intr_comm =
	    of_get_named_gpio_flags(np, "intr-comm-gpio", 0, NULL);
	pdata->gpio_uart_sw =
		of_get_named_gpio_flags(np, "uart-sw-gpio", 0, NULL);
	pdata->gpio_ad_sel =
		of_get_named_gpio_flags(np, "ad-sel-gpio", 0, NULL);
	pdata->external_5v_en_gpio =
		of_get_named_gpio_flags(np, "external-5v-en-gpio", 0, NULL);
	pdata->external_5v_en_flag =
		of_property_read_bool(np, "external-5v-en-flag");


	pr_info("cyccg_parse_dt gpio reset : %d, intr : %d,  uart %d ad-sel %d external_5v_en_flag is %s\n",
		pdata->gpio_reset,
		pdata->gpio_intr_comm, pdata->gpio_uart_sw,pdata->gpio_ad_sel,pdata->external_5v_en_flag?"true":"false");

	return 0;
}

static irqreturn_t cyccg_device_recheck_irq_therad(int irq, void *dev_id)
{
	struct cyccg *cyccg = dev_id;
	struct i2c_client *client = cyccg->client;
	struct device *dev = &client->dev;
	u8 cmd[2] = { CY_PD_ENTER_FLASHING_MODE_CMD_SIG, 0 };
	struct hpi_msg_head head;
	unsigned long end_time;
	int err;

	/*
	 * Directly force enter flashing mode,
	 * avoid the Boot Wait windows (32ms) was missed.
	 */
	dev_info(dev, "%s: force enter bootloader mode\n", __func__);
	err = cyccg_i2c_write(cyccg, cmd, 1,
			      CY_PD_REG_ENTER_FLASH_MODE_ADDR);
	if (err) {
		dev_err(dev, "%s: failed to write flashing mode command, %d\n",
			__func__, err);
		goto out;
	}

	/* Max wait for Boot Wait Window: 32ms time. */
	end_time = jiffies + msecs_to_jiffies(32);
	while (time_before(jiffies, end_time)) {
		memset(&head, 0, sizeof(struct hpi_msg_head));
		err = cyccg_i2c_read(cyccg, (u8 *)&head,
				     sizeof(struct hpi_msg_head),
				     CY_PD_REG_RESPONSE_ADDR);
		if (err) {
			dev_err(dev, "%s: failed to read response regs, %d\n",
				__func__, err);
			goto out;
		}

		if (head.code == CY_PD_RESP_NO_RESPONSE) {
			usleep_range(1000, 2000);
			continue;
		}

		err = cyccg_hpi_clear_intr(cyccg);
		if (err) {
			dev_err(dev, "%s: failed to clear the intr, %d\n",
				__func__, err);
			goto out;
		}

		if (head.code == CY_PD_RESP_SUCCESS) {
			dev_info(dev, "%s: CY_PD_RESP_SUCCESS\n",
				 __func__);
			break;
		} else {
			dev_info(dev, "%s: message code = 0x%02x\n",
				 __func__, head.code);
		}
	}

out:
	complete(&cyccg->resp_ready);
	return IRQ_HANDLED;
}

static int cyccg_device_recheck(struct i2c_client *client, struct cyccg *cyccg)
{
	struct device *dev = &client->dev;
	union i2c_smbus_data dummy;
	unsigned long timeout;
	int err;

	cyccg->client = client;
	mutex_init(&cyccg->letv_i2c_write);

	/* Release XRES pin to disable the CCG for preparing HARD_RESET it. */
	gpio_direction_output(cyccg->pdata->gpio_reset, 0);
	usleep_range(500, 1000);

	/* Register the specific irq thread handler for CCG device recheck. */
	err = request_threaded_irq(client->irq,
				NULL, cyccg_device_recheck_irq_therad,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"cyccg_device_recheck", cyccg);
	if (err) {
		dev_err(dev, "failed to request threaded irq, %d\n", err);
		return err;
	}

	/*
	 * Re-assert the XRES pin to RESET the CCG device, then
	 * wait the for the IRQ event.
	 */
	init_completion(&cyccg->resp_ready);  /* re-used here for short time. */
	gpio_direction_output(cyccg->pdata->gpio_reset, 1);

	/*
	 * Flash validate time (11ms) + Boot Wait Windows (32ms) +
	 * system threads scheduling time 10~50ms.
	 */
	timeout = 100;
	timeout = wait_for_completion_timeout(&cyccg->resp_ready,
					      msecs_to_jiffies(timeout));
	if (timeout == 0)
		dev_warn(dev, "%s: wait IRQ from CCG device timeout\n",
			__func__);

	/* Last time double check. */
	if (i2c_smbus_xfer(client->adapter, client->addr, 0,
			I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &dummy) < 0) {
		dev_err(dev, "%s: last time double check also failed\n",
			__func__);
		err = -ENODEV;
	}

		free_irq(client->irq, cyccg);
	dev_info(dev, "%s: device recheck result: %d\n", __func__, err);
	return err;
}

static int cyccg_probe(struct i2c_client *client,
		       const struct i2c_device_id *dev_id)
{
	struct device *dev = &client->dev;
	union i2c_smbus_data dummy;
	struct cyccg *cyccg;
	int err = 0;
	struct cyccg_platform_data *pdata;
#if CYCCG_ENABLE_STATIC_FW_VER
	u8 device_mode;
#endif
	dev_info(dev, "%s: loading cyccg driver, i2c addr: %d-%04x.\n",
		  __func__, client->adapter->nr, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "not a supported I2C adapter\n");
		return -EIO;
	}

	cyccg = devm_kzalloc(dev, sizeof(struct cyccg), GFP_KERNEL);
	if (!cyccg)
		return -ENOMEM;

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct cyccg_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			err = -ENOMEM;
			goto err0;
		}

		client->dev.platform_data = pdata;

		/* device tree parsing function call */
		err = cyccg_parse_dt(&client->dev, pdata);
		if (err != 0){	/* if occurs error */
			pr_err("%s: Failed to cyccg_parse_dt\n", __func__);
            goto err1;
		}

		cyccg->pdata = pdata;
	} else {
		cyccg->pdata = client->dev.platform_data;
        goto err1;
	}
	/* to access global platform data */
	cyccg_pdata = cyccg->pdata;
	g_cyccg = cyccg;

	err = cyccg_init_gpio(cyccg,true);
	if (err != 0){   /* if occurs error */
		pr_err("%s: Failed to cyccg_init_gpio\n", __func__);
		goto err1;
	}

	/* Make sure there is something at this address*/
	if (i2c_smbus_xfer(client->adapter, client->addr, 0,
			I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &dummy) < 0 &&
			cyccg_device_recheck(client, cyccg)) {
		cyccg_dbg("%s: slave addr %d-%04x not detected\n",
			  __func__, client->adapter->nr, client->addr);
		goto err2;
	}

	INIT_LIST_HEAD(&cyccg->list);
	mutex_init(&cyccg->lock);
	mutex_init(&cyccg->cmd_lock);
	mutex_init(&cyccg->init_work_lock);
	mutex_init(&cyccg->ccg_status_lock);
	INIT_WORK(&cyccg->init_work, cyccg_init_worker);
#if CYCCG_DEBUG
	mutex_init(&cyccg->vdm_work_lock);
	INIT_WORK(&cyccg->vdm_work, cyccg_vdm_worker);
#endif
	mutex_init(&cyccg->irq_work_lock);
	INIT_WORK(&cyccg->irq_work, cyccg_irq_thread_worker);
#if CYCCG_AUTO_INTERNAL_UPDATE
	mutex_init(&cyccg->auto_fw_update_work_lock);
	INIT_WORK(&cyccg->auto_fw_update_work, cyccg_auto_fw_update_worker);
	mutex_init(&cyccg->fw_lock);
#endif
	cyccg_msg_queue_init(cyccg);
	init_completion(&cyccg->resp_ready);
	init_completion(&cyccg->ioctl_clear_intr);
	atomic_set(&cyccg->is_irq_owned_by_drv, 0);
	atomic_set(&cyccg->cmd_issued, 0);
	atomic_set(&cyccg->sigio_queued, 0);
	cyccg->minor = CYCCG_DYNAMIC_MINORS;
	mutex_init(&cyccg->ext_evt_lock);
//letv_pd s
	mutex_init(&cyccg->letv_i2c_write);
//letv_pd e
	init_completion(&cyccg->ext_evt_done);
	cyccg->client = client;
	cyccg->is_ap_synced = false;
	i2c_set_clientdata(client, cyccg);
	client->irq = gpio_to_irq(cyccg->pdata->gpio_intr_comm);
	if (client->irq < 0) {
		pr_err("%s : failed to get gpio comm irq\n", __func__);
		goto err2;
	}
//letv_typec_headset
	mutex_init(&typec_headset_lock);
//letv_typec_headset

	cyccg_dbg("%s: client->irq=%d\n", __func__, client->irq);
//letv_pd s
	letv_pd_init();
//letv_pd e
	err = devm_request_threaded_irq(dev, client->irq,
					NULL, cyccg_irq_therad,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"cyccg", cyccg);
	if (err) {
		dev_err(dev, "failed to request threaded irq, (%d)\n", err);
		goto err2;
	}
#if CYCCG_DISBALE_IRQ_WHEN_NO_MONITOR
	disable_irq(client->irq);
#endif

	err = cyccg_init(cyccg);
	if (err) {
		dev_err(dev, "failed to initialize device, (%d)\n", err);
		goto err3;
	}

	cyccg_soft_reset = 1;
	/*init device wakeup CAP*/
	device_init_wakeup(dev, 1);

	err = cyccg_register_chrdev(cyccg);
	if (err) {
		dev_err(dev, "failed to register CHAR device, (%d)\n", err);
		goto err3;
	}

	err = sysfs_create_group(&client->dev.kobj, &cyccg_sysfs_group);
	if (err) {
		dev_err(dev, "failed to create sysfs interfaces, (%d)\n", err);
		goto err3;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
	err = devm_add_action(dev, cyccg_remove_sysfs_group, cyccg);
	if (err) {
		cyccg_remove_sysfs_group(cyccg);
		dev_err(dev, "failed to create sysfs interfaces, (%d)\n", err);
		goto err3;
	}
#endif

	cclogic_set_audio_mode_register(cyccg_cclogic_set_audio_mode);
	cclogic_set_vconn_register(cyccg_set_vconn);
	INIT_DELAYED_WORK(&typec_uart_swap_dwork, typec_uart_swap_handler);

	/*read irq status for boot detect*/
	cyccg_update_external_ap_status(cyccg,0);

#if CYCCG_AUTO_INTERNAL_UPDATE
	/*
	 * Try to update the firmware image and table image only when the newer
	 * firmware image was found. The firmware and table images will be
	 * updated same time.
	 * Note, the table image will only be updated when the firmware image
	 * exists and is newer than the device firmware image.
	 * Because the table image don't contain the version information,
	 * so to avoid multiple useless update, whatever in order to update the
	 * firmware image or the table image, the firmware image and its
	 * embeded version information must be also updated when released.
	 */
	#if CYCCG_ENABLE_STATIC_FW_VER
	if (cyccg_i2c_read(cyccg, &device_mode, 1,
			   CY_PD_REG_DEVICE_MODE_ADDR))
		device_mode = 0;

	if (device_mode && ((cyccg->app_ver.app_maj_ver > CYCCG_LATEST_FW_MAJ_VER) ||
	    ((cyccg->app_ver.app_maj_ver == CYCCG_LATEST_FW_MAJ_VER) &&
	      (cyccg->app_ver.app_min_ver >= CYCCG_LATEST_FW_MIN_VER)))) {
		/* Return early.
		 * The device firmware version is newer than the specified
		 * external firmware version assigned by the static macro
		 * values.
		 */
		cyccg_dbg("%s: %s. fw:%d.%d, drv fw:%d.%d.\n",
			  __func__, "check auto fw update, return early",
			  cyccg->app_ver.app_maj_ver,
			  cyccg->app_ver.app_min_ver,
			  CYCCG_LATEST_FW_MAJ_VER, CYCCG_LATEST_FW_MIN_VER);
		goto end;
	}
	cyccg_dbg("%s: queue fw update worker, fw:%d.%d, drv fw:%d.%d.\n",
		  __func__,
		  cyccg->app_ver.app_maj_ver,
		  cyccg->app_ver.app_min_ver,
		  CYCCG_LATEST_FW_MAJ_VER, CYCCG_LATEST_FW_MIN_VER);
	#endif
	cyccg_queue_auto_fw_update_thread_work(cyccg);
#endif

	goto end;

err3:
	free_irq(client->irq, cyccg);
err2:
	cyccg_init_gpio(cyccg,false);
err1:
	if (client->dev.of_node) {
		kfree(pdata);
	}
	cyccg_pdata = NULL;
	g_cyccg = NULL;
err0:
	kfree(cyccg);
end:
	return err;
}

static int cyccg_remove(struct i2c_client *client)
{
	struct cyccg *cyccg = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	if (device_may_wakeup(dev) && cyccg->irq_wake) {
		disable_irq_wake(client->irq);
		cyccg->irq_wake = false;
	}

	if (client->irq)
		disable_irq(client->irq);
	mutex_destroy(&typec_headset_lock);

#if CYCCG_DEBUG
	cancel_work_sync(&cyccg->vdm_work);
#endif
	cancel_work_sync(&cyccg->irq_work);
	cancel_work_sync(&cyccg->init_work);
	cyccg_port_disable(cyccg);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
	return cyccg_unregister_chrdev(cyccg);
#else
	cyccg_remove_sysfs_group(cyccg);
	return cyccg_unregister_chrdev(cyccg);
#endif
}

static int __maybe_unused cyccg_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cyccg *cyccg = i2c_get_clientdata(client);

	if (cyccg_irq_next_ready(cyccg)) {
		cyccg_dbg("%s: break suspend to process CCG queued events.\n",
			  __func__);

		/*
		 * There were queued events not processed,
		 * so stop suspend to process them.
		 * Normally, it should not happen.
		 */
		cyccg_queue_irq_thread_work(cyccg);
		return -EBUSY;
	}

	if (client->irq)
		disable_irq(client->irq);

	if (device_may_wakeup(dev))
		cyccg->irq_wake = (enable_irq_wake(client->irq) == 0);

	return 0;
}

static int __maybe_unused cyccg_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cyccg *cyccg = i2c_get_clientdata(client);

	if (device_may_wakeup(dev) && cyccg->irq_wake) {
		disable_irq_wake(client->irq);
		cyccg->irq_wake = false;
	}

	if (client->irq)
		enable_irq(client->irq);

#if CYCCG_ENABLE_RESUME_IRQ_WORK_THREAD
	/*
	 * Queue the irq handler try to check/read the response data and
	 * process it if the resume operation is triggered by the CCG device.
	 * So to avoid the event be lost or the event be queued always on some
	 * platform the IRQ LOW only will be triggered once even keep low.
	 */
	cyccg_queue_irq_thread_work(cyccg);
#endif

	return 0;
}

static const struct dev_pm_ops cyccg_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyccg_suspend, cyccg_resume)
};

static const struct i2c_device_id cyccg_id_table[] = {
	{ CYCCG_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cyccg_id_table);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
#ifdef CONFIG_ACPI
static const struct acpi_device_id cyccg_acpi_id[] = {
	{ "CYCC0000", 0 },  /* Type-C USB CCG Device */
	{ "CYCC0001", 0 },  /* Type-C USB CCG1 Device */
	{ "CYCC0002", 0 },  /* Type-C USB CCG2 Device */
	{ }
};
MODULE_DEVICE_TABLE(acpi, cyccg_acpi_id);
#endif
#endif

#ifdef CONFIG_OF
static const struct of_device_id cyccg_of_match[] = {
	{
		.compatible = "cypress,cyccg",
		.data = NULL,
	},
	{ }
};
#endif

static struct i2c_driver cyccg_driver = {
	.driver = {
		.name = "cyccg",
		.owner = THIS_MODULE,
		.pm = &cyccg_pm_ops,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
		.acpi_match_table = ACPI_PTR(cyccg_acpi_id),
#endif
		.of_match_table = of_match_ptr(cyccg_of_match),
	},

	.probe = cyccg_probe,
	.remove = cyccg_remove,
	.id_table = cyccg_id_table,
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0))
module_i2c_driver(cyccg_driver);
#else
static int __init cyccg_i2c_init(void)
{
	return i2c_add_driver(&cyccg_driver);
}

static void __exit cyccg_i2c_exit(void)
{
	i2c_del_driver(&cyccg_driver);
}

module_init(cyccg_i2c_init);
module_exit(cyccg_i2c_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dudley Du <dudl@cypress.com>");
MODULE_DESCRIPTION("Cypress USB Type-C and PD Controller Device Driver");
