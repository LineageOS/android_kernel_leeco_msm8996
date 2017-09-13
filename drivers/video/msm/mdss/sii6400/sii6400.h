/*
  SiI6400 Linux Driver

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

#ifndef _SII6400_H
#define _SII6400_H

#define MHL_POWER_OUT
#ifdef MHL_POWER_OUT
extern int dwc3_otg_set_mhl_power(bool enable);
extern void dwc3_otg_start_mhl_power(void);
#endif

#define strict_strtoul	kstrtoul
#define strict_strtol	kstrtol
#define strict_strtoull	kstrtoull
#define strict_strtoll	kstrtoll

#define SII6400_CLASS_NAME	"video"
#define SII6400_DEVNAME		"sii6400"
#define SII6400_WIHD_DEVNAME	"wihd"
#define SII6400_MHL_DEVNAME	"mhl"

#define SII6400_NUMBER_OF_DEVS	3

#define DEVICE_EVENT		"DEVICE_EVENT"
#define WIHD_EVENT		"WIHD_EVENT"
#define MHL_EVENT		"MHL_EVENT"
#define REMOTE_FW_UPDATE_EVENT	"REMOTE_FW_UPDATE_EVENT"

/* Device events */
#define MODE_CHANGE_EVENT	"mode_change"
#define MODE_CHANGE_EVENT_WIHD_ON	"mode_change_wihd_on"
#define MODE_CHANGE_EVENT_MHL_ON	"mode_change_mhl_on"
#define MODE_CHANGE_EVENT_WIHD_OFF	"mode_change_wihd_off"
#define MODE_CHANGE_EVENT_MHL_OFF	"mode_change_mhl_off"
#define DIAG_CMD_DONE_EVENT	"diag_cmd_done"

/* WIHD events */
#define WIHD_ASSOCIATED_EVENT		"associated"
#define WIHD_DISASSOCIATED_EVENT	"disassociated"
#define WIHD_CONNECTED_EVENT		"av_enabled"
#define WIHD_DISCONNECTED_EVENT		"disconnected"
#define WIHD_DEVICE_LIST_ENTRY_EVENT	"dev_list_entry"
#define WIHD_DEVICE_LIST_COMPLETE_EVENT	"dev_list_complete"
#define WIHD_SCAN_STARTED_EVENT		"wvan_scan_started"
#define WIHD_SCAN_COMPLETE_EVENT	"wvan_scan_complete"
#define WIHD_SCAN_STOPPED_EVENT		"wvan_scan_stopped"
#define WIHD_RECEIVED_RC_EVENT		"received_remote_control"
#define WIHD_VENDOR_MSG_RCVD_EVENT	"vendor_msg_received"
#define MHL_DCAP_CHG_EVENT	"device_capability_changed"

/* MHL events */
#define MHL_CONNECTED_EVENT	"connected"
#define MHL_DISCONNECTED_EVENT	"disconnected"
#define MHL_RAP_RECEIVED_EVENT	"received_rap"
#define MHL_RAP_ACKED_EVENT	"received_rapk"
#define MHL_RCP_RECEIVED_EVENT	"received_rcp"
#define MHL_RCP_ACKED_EVENT	"received_rcpk"
#define MHL_RCP_ERROR_EVENT	"received_rcpe"
#define MHL_UCP_RECEIVED_EVENT	"received_ucp"
#define MHL_UCP_ACKED_EVENT	"received_ucpk"
#define MHL_UCP_ERROR_EVENT	"received_ucpe"
#define MHL_VBUS_ON_EVENT	"provide_vbus_power"
#define MHL_VBUS_OFF_EVENT	"remove_vbus_power"

/* Remote Firmware Update events */
#define REMOTE_FW_UPDATE_FAILED		"remote_fw_update_failed"
#define REMOTE_FW_UPDATE_PROGRESS	"remote_fw_update_progress"
#define REMOTE_FW_UPDATE_COMPLETE	"remote_fw_update_complete"

#define MAX_EVENT_STRING_LEN	1024

#define FILEPATH_NAME_LEN	128

#define MAX_DRIVER_MODE_LEN	5
#define MAC_ADDRESS_STR_1_LEN	13
#define MAC_ADDR_LEN		6
#define MANUFACTURER_LEN	3
#define MONITOR_NAME_LEN	13
#define DEVICE_NAME_LEN		16
#define WVAN_NAME_LEN		32
#define MAX_CEC_MESSAGE_LEN	16
#define CEC_MESSAGE_STR_1_LEN	33
#define SPAD_OFFSET_LEN		5
#define SPAD_LENGTH_LEN		5
#define SPAD_DATA_LEN		5
#define SPAD_KEYCODE_LEN	5
#define SPAD_ERRORCODE_LEN	5
#define MAX_DIAG_CMD_STR_LEN	1023
#define MAX_VENDOR_ID_STR_LEN	7
#define VENDOR_ID_LEN		3
#define MAX_VENDOR_MSG_STR_LEN	33
#define MAX_VENDOR_MSG_LEN	16

#define MAX_SCAN_DURATION	86400
#define MAX_SCAN_INTERVAL	3600

#define MHL_IMPEDANCE_OHMS		0x3e8
#define USB_MINI_A_IMPEDANCE_OHMS	0
#define USB_MINI_B_IMPEDANCE_OHMS	(~0)
#define AUDIO_SEND_END_BUTTON		0x7d0

#define MAX_SCRATCHPAD_REGISTER		64

#define MAX_MHL_DEVCAP_SIZE		16
#define MAX_DEVCAP_OFFSET		15

#define MAX_MHL_UCP_STATUS		1
#define MAX_MHL_RCP_STATUS		2
#define MAX_MHL_RAP_STATUS		3

#define SELF_CONNECT_TIMEOUT_DEFAULT	10
#define HDCP_POLICY_DEFAULT		0
#define HDCP_STREAM_TYPE_DEFAULT	0

#define INPUT_DEVICE_APP		0
#define INPUT_DEVICE_DRIVER		1
#define INPUT_DEVICE_DEFAULT_VALUE	INPUT_DEVICE_DRIVER

#define DEBUG_OUTPUT_PATH_UART		1
#define DEBUG_OUTPUT_PATH_SPI		2
#define DEBUG_OUTPUT_PATH_DEFAULT	DEBUG_OUTPUT_PATH_UART

#include <linux/platform_device.h>
struct msm_hdmi_device_ops{
	int (*hdmi_tx_device_show)(char *buf);
};

#define SII6400_COMPATIBLE_NAME "qcom, mhl-wihd-sii6400"

extern int msm_hdmi_device_show_register(int (*func)(char*));

struct msm_hdmi_mhl_ops {
	u8 (*tmds_enabled)(struct platform_device *pdev);
	int (*set_mhl_max_pclk)(struct platform_device *pdev, u32 max_val);
	int (*set_upstream_hpd)(struct platform_device *pdev, uint8_t on);
	void (*notify)(void *data);
};
extern int msm_hdmi_register_mhl(struct platform_device *pdev,
				struct msm_hdmi_mhl_ops *ops, void *data);

enum sii6400_gpio_type {
	SII6400_HDMI_DET_GPIO,
	SII6400_RESET_GPIO,
	SII6400_INTR_GPIO,
	SII6400_PMIC_PWR_GPIO,

	SII6400_MHL_SEL1_GPIO,

	SII6400_MAX_GPIO,
};

enum sii6400_vreg_type {
	SII6400_1V8_VREG,
	SII6400_MAX_VREG,
};

enum sii6400_mode {
	SII6400_MODE_UNDEFINED = -1,
	SII6400_MODE_CONFIG = 0,
	SII6400_MODE_WIHD = 1,
	SII6400_MODE_MHL = 2,
	SII6400_MODE_OFF = 3,
};

enum sii6400_wihd_state {
	WIHD_STATE_NONE = 0,
	WIHD_STATE_IDLE,
	WIHD_STATE_WVAN_SCAN,
	WIHD_STATE_ASSOCIATING,
	WIHD_STATE_ASSOCIATED,
	WIHD_STATE_CONNECTING,
	WIHD_STATE_CONNECTED,
};

enum sii6400_mhl_state {
	MHL_STATE_NONE = 0,
	MHL_STATE_DISCONNECTED,
	MHL_STATE_CONNECTED,
};

enum sii6400_mhl_device_type {
	MHL_DEV_TYPE_TV = 0,
	MHL_DEV_TYPE_ARMET,
};

enum sii6400_wihd_category {
	WIHD_CATEGORY_OTHER = 0xff,
	WIHD_CATEGORY_DTV = 0x00,
	WIHD_CATEGORY_BD_DVD_PLAYER = 0x01,
	WIHD_CATEGORY_BD_DVD_RECORDER = 0x02,
	WIHD_CATEGORY_STB = 0x03,
	WIHD_CATEGORY_AUDIO_AMP = 0x04,
	WIHD_CATEGORY_HDD_RECORDER = 0x05,
	WIHD_CATEGORY_MONITOR = 0x06,
	WIHD_CATEGORY_PROJECTOR = 0x07,
	WIHD_CATEGORY_PC = 0x08,
	WIHD_CATEGORY_CAMCORDER = 0x09,
	WIHD_CATEGORY_GAME = 0x0a,
	WIHD_CATEGORY_ADAPTER = 0x0b,
	WIHD_CATEGORY_MOBILE_PHONE = 0x0c,
	WIHD_CATEGORY_PDA = 0x0d,
	WIHD_CATEGORY_PMP = 0x0e,
	WIHD_CATEGORY_DSC = 0x0f,
	WIHD_CATEGORY_RESERVED = 0x10,
};

struct sii6400_chip_version {
	uint8_t length;
	char *chip_version;
};

struct sii6400_firmware_version {
	uint8_t length;
	char *fw_version;
};

struct sii6400_connect_timeout {
	uint16_t connect_timeout;
};

struct sii6400_mac_addr {
	uint8_t mac_addr[MAC_ADDR_LEN];
};

struct sii6400_name {
	char name[DEVICE_NAME_LEN + 1];
};

struct sii6400_dev_category {
	enum sii6400_wihd_category category;
};

struct sii6400_manufacturer {
	char manufacturer[MANUFACTURER_LEN + 1];
};

struct sii6400_monitor_name {
	char monitor_name[MONITOR_NAME_LEN + 1];
};

struct sii6400_dev_type {
	bool video_source;
	bool video_sink;
	bool audio_source;
	bool audio_sink;
};

struct sii6400_signal_strength {
	uint8_t signal_strength;
};

struct sii6400_scan_duration {
	uint32_t scan_duration;
};

struct sii6400_scan_interval {
	uint16_t scan_interval;
};

struct sii6400_scan_status {
	bool scan_enabled;
};

struct sii6400_wvan_info {
	uint8_t wvan_id;
	uint8_t hr_channel;
	uint8_t lr_channel;
};

struct sii6400_connect {
	uint8_t mac_addr[MAC_ADDR_LEN];
	uint16_t connect_timeout;
};

struct sii6400_remote_device {
	struct sii6400_mac_addr mac_addr;
	struct sii6400_name name;
	struct sii6400_manufacturer manufacturer;
	struct sii6400_dev_category category;
	struct sii6400_dev_type type;
};

struct sii6400_wvan_connection_status {
	uint8_t status;
};

struct sii6400_remote_device_list {
	uint8_t num_dev;
	struct sii6400_remote_device *remote_devices;
};

struct sii6400_wvan {
	char wvan_name[WVAN_NAME_LEN + 1];
	uint8_t wvan_id;
	uint8_t hr_channel;
	uint8_t lr_channel;
	uint8_t signal_strength;
};

struct sii6400_scan_results {
	uint8_t num_wvan;
	struct sii6400_wvan *wvans;
};

struct sii6400_cec_message {
	uint8_t length;
	uint8_t cec_message[MAX_CEC_MESSAGE_LEN];
};

struct sii6400_wihd_rc {
	uint8_t ctrlcode;
};

struct sii6400_hdcp_policy {
	uint8_t policy;
};

struct sii6400_hdcp_stream_type {
	uint8_t stream_type;
};

struct sii6400_remote_fw_update_status {
	bool remote_fw_update_enabled;
};

struct sii6400_vendor_id {
	uint8_t vendor_id[VENDOR_ID_LEN];
};

struct sii6400_vendor_msg_payload {
	uint8_t data[MAX_VENDOR_MSG_LEN];
	uint8_t dataLength;
};

struct sii6400_vendor_msg {
	uint8_t vendor_id[VENDOR_ID_LEN];
	uint8_t mac_addr[MAC_ADDR_LEN];
	uint8_t data[MAX_VENDOR_MSG_LEN];
	uint8_t dataLength;
};

struct sii6400_diag_cmd {
	char *diag_cmd;
	uint16_t length;
	uint16_t cmd_id;
	bool is_sync_cmd;
};

struct sii6400_diag_cmd_output {
	char *output;
	uint16_t length;
	uint16_t cmd_id;
};

struct sii6400_debug_output_path {
	uint8_t debug_output_path;
};

struct sii6400_mhl_dev_type {
	uint32_t type;
};

struct sii6400_mhl_connection_state {
	enum sii6400_mhl_state connect_state;
};

struct sii6400_id_impedance {
	uint32_t ohms;
};

struct sii6400_clock_swing {
	uint8_t clock_swing;
};

struct sii6400_cbus_voltage {
	uint8_t high;
	uint16_t peroid;
};

struct sii6400_mhl_devcap {
	uint8_t data[MAX_MHL_DEVCAP_SIZE];
	uint8_t length;
	uint8_t offset;
};

struct sii6400_mhl_devcap_offset {
	uint8_t offset;
};

struct sii6400_mhl_ucp {
	uint8_t charcode;
};

struct sii6400_mhl_ucp_status {
	uint8_t status;
};

struct sii6400_mhl_rcp {
	uint8_t keycode;
};

struct sii6400_mhl_rcp_status {
	uint8_t status;
};

struct sii6400_mhl_rap {
	uint8_t actioncode;
};

struct sii6400_mhl_rap_status {
	uint8_t status;
};

struct sii6400_wihd_device_info {
	dev_t devnum;
	struct cdev *cdev;
	struct device *device;

	enum sii6400_wihd_state state;
};

struct sii6400_mhl_device_info {
	dev_t devnum;
	struct cdev *cdev;
	struct device *device;

	enum sii6400_mhl_state state;
};

struct sii6400_device_info {
	dev_t devnum;
	struct cdev *cdev;
	struct device *device;
	struct class *dev_class;

	/* Data filled from device tree nodes */
	struct dss_gpio *gpios[SII6400_MAX_GPIO];
	struct dss_vreg *vregs[SII6400_MAX_VREG];
#ifdef CONFIG_LIMIT_ON_HDMI
	struct platform_device *hdmi_pdev;
	struct msm_hdmi_mhl_ops *hdmi_mhl_ops;
#endif

	struct sii6400_wihd_device_info *wihd;
	struct sii6400_mhl_device_info *mhl;

	enum sii6400_mode mode;
	int vbus_power_state;

	struct sii6400_connect_timeout my_connect_timeout;
	struct sii6400_hdcp_policy my_hdcp_policy;
	struct sii6400_hdcp_stream_type my_hdcp_stream_type;

	struct sii6400_debug_output_path my_debug_output_path;
	struct file *debug_log_filp;

	uint8_t my_rc_input_device;
	uint8_t my_rap_input_device;
	uint8_t my_rcp_input_device;
	uint8_t my_ucp_input_device;
	struct notifier_block fb_notify;
	struct work_struct fb_notify_work;
};

struct sii6400_module_info {
	struct sii6400_device_info *sii6400_devinfo;
};

struct sii6400_firmware_memory {
	unsigned char *cfg_buf;
	uint32_t **firmware_buf;
};
struct sii6400_firmware_memory *get_firmware_memory(void);

extern struct device *sii6400_device;
extern int debug_level;

#ifdef SII6400_COMBINED_FIRMWARE
#define DEFAULT_WIHD_MHL_FIRMWARE_FILENAME "sii6400_wihd_mhl.fw"

extern char *wihd_mhl_firmware;	/* WiHD/MHL Firmware filepath */
#else /* SII6400_COMBINED_FIRMWARE */
#define DEFAULT_WIHD_FIRMWARE_FILENAME	"sii6400_wihd.fw"
#define DEFAULT_MHL_FIRMWARE_FILENAME	"sii6400_mhl.fw"

extern char *wihd_firmware;	/* WiHD Firmware filepath */
extern char *mhl_firmware;	/* MHL Firmware filepath */
#endif /* SII6400_COMBINED_FIRMWARE */

#define NV_STORAGE_SIZE			8192
extern char *configdata_file;	/* Non-Volatile Storage filepath */

/* sii6400.c */
int send_sii6400_uevent(struct device *device, const char *event_cat,
			const char *event_type, const char *event_data);
struct sii6400_device_info *get_sii6400_devinfo(void);
const char *get_wihd_state_string(enum sii6400_wihd_state wihd_state);
const char *get_category_string(struct sii6400_dev_category *device_category);
bool sii6400_mode_is_config(enum sii6400_mode mode);
bool sii6400_mode_is_off(enum sii6400_mode mode);
bool sii6400_mode_is_wihd(enum sii6400_mode mode);
bool sii6400_mode_is_mhl(enum sii6400_mode mode);
bool sii6400_char_string_is_valid(const char *data, size_t max_strlen);
bool sii6400_utf8_data_is_valid(const unsigned char *data, size_t size);
bool sii6400_module_is_exiting(void);
void cleanup_and_exit(void);

/* sii6400_wihd.c */
int sii6400_wihd_dev_add(struct sii6400_device_info *devinfo);
int sii6400_wihd_dev_init(struct sii6400_device_info *devinfo);
void sii6400_wihd_dev_exit(struct sii6400_device_info *devinfo);
void sii6400_wihd_dev_remove(struct sii6400_device_info *devinfo);

/*callback*/
void hdmi_tx_hpd_done_callback(void *data);

/* sii6400_mhl.c */
void change_mode(struct sii6400_device_info *devinfo,
			enum sii6400_mode new_mode);
int sii6400_mhl_dev_add(struct sii6400_device_info *devinfo);
int sii6400_mhl_dev_init(struct sii6400_device_info *devinfo);
void sii6400_mhl_dev_exit(struct sii6400_device_info *devinfo);
void sii6400_mhl_dev_remove(struct sii6400_device_info *devinfo);
int get_sii6400_mhl_id_impedance_after_mode_changed(void);

#ifdef SII6400_DEBUG
/* sii6400_dbg.c */
int sii6400_device_dbg_open(struct inode *inode, struct file *filp);
int sii6400_device_dbg_close(struct inode *inode, struct file *filp);
ssize_t sii6400_device_dbg_write(struct file *filp, const char __user *buffer,
					size_t length, loff_t *offset);
ssize_t sii6400_device_dbg_read(struct file *filp, char __user *buffer,
					size_t length, loff_t *offset);
int sii6400_dbg_init(struct sii6400_device_info *devinfo);
void sii6400_dbg_exit(struct sii6400_device_info *devinfo);
#endif /* SII6400_DEBUG */

#endif /* !_SII6400_H */

