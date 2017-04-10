#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/rwlock_types.h>
#include <linux/completion.h>
#include "anx_ohio_private_interface.h"
#include "anx_ohio_public_interface.h"

/**
 * @desc:   The Interface AP set the source capability to Ohio
 *
 * @param:  pdo_buf: PDO buffer pointer of source capability,
 *                              which can be packed by PDO_FIXED_XXX macro
 *                eg: default5Vsafe src_cap(5V, 0.9A fixed) -->
 *			PDO_FIXED(5000,900, PDO_FIXED_FLAGS)
 *
 *                src_caps_size: source capability's size
 *
 * @return:  0: success 1: fail
 *
 */
inline u8 send_src_cap(const u8 *src_caps, u8 src_caps_size)
{
	if (NULL == src_caps)
		return CMD_FAIL;
	if ((src_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
	    (src_caps_size / PD_ONE_DATA_OBJECT_SIZE) >
	    PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}
	memcpy(pd_src_pdo, src_caps, src_caps_size);
	pd_src_pdo_cnt = src_caps_size / PD_ONE_DATA_OBJECT_SIZE;

	/*send source capabilities message to Ohio really */
	return interface_send_msg_timeout(TYPE_PWR_SRC_CAP, pd_src_pdo,
					  pd_src_pdo_cnt *
					  PD_ONE_DATA_OBJECT_SIZE,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface AP fetch the source capability from Ohio
 *
 * @param:  pdo_buf: PDO buffer pointer of source capability in Ohio
 *          src_caps_size: source capability's size
 *
 * @return:  0: success 1: fail
 *
 */
u8 get_src_cap(const u8 *src_caps, u8 src_caps_size)
{

	src_caps = src_caps;
	src_caps_size = src_caps_size;
	return 1;
}

/**
 * @desc:   Interface that AP fetch the sink capability from Ohio's downstream device
 *
 * @param:  sink_caps: PDO buffer pointer of sink capability
 *            which will be responsed by Ohio's SINK Capablity Message
 *
 *          snk_caps_len: sink capability max length of the array
 *
 * @return:  sink capability array length>0: success.  0: fail
 *
 */
u8 get_snk_cap(u8 *snk_caps, u8 snk_caps_len)
{
	snk_caps = snk_caps;
	snk_caps_len = snk_caps_len;
	return 1;
}

/**
 * @desc:   Interface that AP send(configure) the sink capability to Ohio's downstream device
 *
 * @param:  snk_caps: PDO buffer pointer of sink capability
 *
 *                snk_caps_size: sink capability length
 *
 * @return:  1: success.  0: fail
 *
 */
inline u8 send_snk_cap(const u8 *snk_caps, u8 snk_caps_size)
{
	memcpy(pd_snk_pdo, snk_caps, snk_caps_size);
	pd_snk_pdo_cnt = snk_caps_size / PD_ONE_DATA_OBJECT_SIZE;

	/*configure sink cap */
	return interface_send_msg_timeout(TYPE_PWR_SNK_CAP, pd_snk_pdo,
					  pd_snk_pdo_cnt * 4,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the DP's sink capability to Ohio's downstream device
 *
 * @param:  dp_snk_caps: PDO buffer pointer of DP sink capability
 *
 *                dp_snk_caps_size: DP sink capability length
 *
 * @return:  1: success.  0: fail
 *
 */
inline u8 send_dp_snk_cfg(const u8 *dp_snk_caps, u8 dp_snk_caps_size)
{
	memcpy(configure_DP_caps, dp_snk_caps, dp_snk_caps_size);
	interface_send_dp_caps();
	return 1;
}

/**
 * @desc:   Interface that AP initialze the DP's capability of Ohio, as source device
 *
 * @param:  dp_caps: DP's capability  pointer of source
 *
 *                dp_caps_size: source DP capability length
 *
 * @return:  1: success.  0: fail
 *
 */
inline u8 send_src_dp_cap(const u8 *dp_caps, u8 dp_caps_size)
{
	if (NULL == dp_caps)
		return CMD_FAIL;
	if ((dp_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
	    (dp_caps_size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}

	memcpy(src_dp_caps, dp_caps, dp_caps_size);

	/*configure source DP cap */
	return interface_send_msg_timeout(TYPE_DP_SNK_IDENDTITY,
					  src_dp_caps, dp_caps_size,
					  INTERFACE_TIMEOUT);
}

inline u8 send_dp_snk_identity(const u8 *snk_ident, u8 snk_ident_size)
{
	return interface_send_msg_timeout(TYPE_DP_SNK_IDENDTITY,
					  (u8 *) snk_ident, snk_ident_size,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   The Interface AP set the VDM packet to Ohio
 *
 * @param:  vdm:  object buffer pointer of VDM
 *
 *                size: vdm packet size
 *
 * @return:  0: success 1: fail
 *
 */
inline u8 send_vdm(const u8 *vdm, u8 size)
{
	u8 tmp[32] = { 0 };
	if (NULL == vdm)
		return CMD_FAIL;
	if (size > 3 && size < 32) {
		memcpy(tmp, vdm, size);
		if (tmp[2] == 0x01 && tmp[3] == 0x00) {
			tmp[3] = 0x40;
			return interface_send_msg_timeout(TYPE_VDM, tmp, size,
							  INTERFACE_TIMEOUT);
		}
	}
	return 1;
}

/**
 * @desc:   The Interface AP set the SVID packet to Ohio
 *
 * @param:  svid:  object buffer pointer of svid
 *
 *                size: svid packet size
 *
 * @return:  0: success 1: fail
 *
 */
inline u8 send_svid(const u8 *svid, u8 size)
{
	u8 tmp[4] = {
		0
	};
	if (NULL == svid || size != 4)
		return CMD_FAIL;
	memcpy(tmp, svid, size);
	return interface_send_msg_timeout(TYPE_SVID, tmp, size,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the sink capability to Ohio's downstream device
 *
 * @param:  snk_caps: PDO buffer pointer of sink capability
 *
 *                snk_caps_size: sink capability length
 *
 * @return:  1: success.  0: fail
 *
 */
inline u8 send_rdo(const u8 *rdo, u8 size)
{
	u8 i;
	if (NULL == rdo)
		return CMD_FAIL;
	if ((size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
	    (size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
		return CMD_FAIL;
	}
	for (i = 0; i < size; i++)
		pd_rdo[i] = *rdo++;

	return interface_send_msg_timeout(TYPE_PWR_OBJ_REQ, pd_rdo, size,
					  INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will get the ohio's data role
 *
 * @param:  none
 *
 * @return:  data role , dfp 1 , ufp 0, other error: -1, not ready
 *
 */
s8 get_data_role(void)
{
	u8 status = 0x00;

	/* cable unplug check */
	if (atomic_read(&ohio_power_status) != 1)
		return ERR_CABLE_UNPLUG;

	/*fetch the data role */
	status = OhioReadReg(OHIO_SYSTEM_STSTUS);

	return ((status & DATA_ROLE) != 0);

}

/**
 * @desc:   The interface AP will get the ohio's power role
 *
 * @param:  none
 *
 * @return:  data role , source 1 , sink 0, other error, -1, not ready
 *
 */
s8 get_power_role(void)
{
	u8 status = 0x00;

	/* cable unplug check */
	if (atomic_read(&ohio_power_status) != 1)
		return ERR_CABLE_UNPLUG;

	/*fetch the power role */
	status = OhioReadReg(0x40);

	return ((status & 0x08) == 0);
}

/**
 * @desc:   The interface AP will send  PR_Swap command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
u8 send_power_swap(void)
{
	return interface_pr_swap();
}

/**
 * @desc:   The interface AP will send DR_Swap command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
inline u8 send_data_swap(void)
{
	return interface_dr_swap();
}

/**
 * @desc:   The interface AP will send accpet command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
inline u8 send_accept(void)
{
	return interface_send_msg_timeout(TYPE_ACCEPT, 0, 0, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send reject command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
inline u8 send_reject(void)
{
	return interface_send_msg_timeout(TYPE_REJECT, 0, 0, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send soft reset command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
inline u8 send_soft_reset(void)
{
	return interface_send_soft_rst();
}

/**
 * @desc:   The interface AP will send hard reset command to Ohio
 *
 * @param:  none
 *
 * @return:  1: success.  0: fail
 *
 */
inline u8 send_hard_reset(void)
{
	return interface_send_hard_rst();
}

inline char *interface_to_str(unsigned char header_type)
{
	return (header_type == TYPE_PWR_SRC_CAP) ? "TYPE_PWR_SRC_CAP" :
	    (header_type == TYPE_PWR_SNK_CAP) ? "TYPE_PWR_SNK_CAP" :
	    (header_type == TYPE_PWR_OBJ_REQ) ? "TYPE_PWR_OBJ_REQ" :
	    (header_type == TYPE_DP_SNK_IDENDTITY) ? "TYPE_DP_SNK_IDENDTITY" :
	    (header_type == TYPE_SVID) ? "TYPE_SVID" :
	    (header_type == TYPE_PSWAP_REQ) ? "TYPE_PSWAP_REQ" :
	    (header_type == TYPE_DSWAP_REQ) ? "TYPE_DSWAP_REQ" :
	    (header_type == TYPE_GOTO_MIN_REQ) ? "TYPE_GOTO_MIN_REQ" :
	    (header_type == TYPE_DP_ALT_ENTER) ? "TYPE_DP_ALT_ENTER" :
	    (header_type == TYPE_DP_ALT_EXIT) ? "TYPE_DP_ALT_EXIT" :
	    (header_type == TYPE_VCONN_SWAP_REQ) ? "TYPE_VCONN_SWAP_REQ" :
	    (header_type == TYPE_GET_DP_SNK_CAP) ? "TYPE_GET_DP_SNK_CAP" :
	    (header_type == TYPE_DP_SNK_CFG) ? "TYPE_DP_SNK_CFG" :
	    (header_type == TYPE_SOFT_RST) ? "TYPE_SOFT_RST" :
	    (header_type == TYPE_HARD_RST) ? "TYPE_HARD_RST" :
	    (header_type == TYPE_RESTART) ? "TYPE_RESTART" :
	    (header_type == TYPE_PD_STATUS_REQ) ? "TYPE_PD_STATUS_REQ" :
	    (header_type == TYPE_ACCEPT) ? "TYPE_ACCEPT" :
	    (header_type == TYPE_REJECT) ? "TYPE_REJECT" :
	    (header_type == TYPE_VDM) ? "TYPE_VDM" :
	    (header_type ==
	     TYPE_RESPONSE_TO_REQ) ? "Response to Request" : "Unknown";
}

inline unsigned char cac_checksum(unsigned char *pSendBuf, unsigned char len)
{
	unsigned char i;
	unsigned char checksum;
	checksum = 0;
	for (i = 0; i < len; i++)
		checksum += *(pSendBuf + i);

	return (u8) (0 - checksum);
}

#ifdef USB_PD_WAIT_LOCK
static DEFINE_RWLOCK(usb_pd_cmd_rwlock);
int usb_pd_cmd_counter = 0;
u8 usb_pd_cmd = 0;

#else
DECLARE_COMPLETION(usb_pd_complete);
DEFINE_SPINLOCK(usb_pd_cmd_lock);
u8 CUR_REQUESTING_PD_CMD = 0xff;

#endif /*  */
u8 usb_pd_cmd_status = 0xff;

#define DATA_ROLE_IS_DFP    0x2
#define POWER_ROLE_IS_SOURCE    0x1

u8 fetch_data_role_from_pd_result(void)
{
	u8 pd_status = 0x00;
#ifdef USB_PD_WAIT_LOCK
	read_lock(&usb_pd_cmd_rwlock);
	pd_status = usb_pd_cmd_status;
	write_unlock_irq(&usb_pd_cmd_rwlock);
#else
	spin_lock_irq(&usb_pd_cmd_lock);
	pd_status = usb_pd_cmd_status;
	spin_unlock_irq(&usb_pd_cmd_lock);
#endif

	/* DFP 1, UFP 0 */
	return pd_status & DATA_ROLE_IS_DFP;
}

u8 fetch_power_role_from_pd_result(void)
{
	u8 pd_status = 0x00;
#ifdef USB_PD_WAIT_LOCK
	read_lock(&usb_pd_cmd_rwlock);
	pd_status = usb_pd_cmd_status;
	write_unlock_irq(&usb_pd_cmd_rwlock);
#else
	spin_lock_irq(&usb_pd_cmd_lock);
	pd_status = usb_pd_cmd_status;
	spin_unlock_irq(&usb_pd_cmd_lock);
#endif

	/* DFP 1, UFP 0 */
	return pd_status & POWER_ROLE_IS_SOURCE;
}

inline u8 wait_pd_cmd_timeout(PD_MSG_TYPE pd_cmd, int pd_cmd_timeout)
{
#ifdef USB_PD_WAIT_LOCK
	unsigned long expire;
	u8 cmd_status = 0;
	write_lock_irq(&usb_pd_cmd_rwlock);
	usb_pd_cmd_counter = 1;
	write_unlock_irq(&usb_pd_cmd_rwlock);
	pr_info("wait_pd_cmd_timeout\n");

	/*looply check counter to be changed to 0 in interface interrupt */
	expire = msecs_to_jiffies(pd_cmd_timeout) + jiffies;
	while (1) {
		if (time_before(expire, jiffies)) {
			write_lock_irq(&usb_pd_cmd_rwlock);
			usb_pd_cmd_counter = 0;
			cmd_status = 0;
			write_unlock_irq(&usb_pd_cmd_rwlock);

			return CMD_FAIL;
		}
		read_lock(&usb_pd_cmd_rwlock);
		if (usb_pd_cmd_counter <= 0) {
			cmd_status = usb_pd_cmd_status;
			read_unlock(&usb_pd_cmd_rwlock);

			if (usb_pd_cmd == pd_cmd) {
				read_unlock(&usb_pd_cmd_rwlock);
				return CMD_SUCCESS;
			}
		}
		read_unlock(&usb_pd_cmd_rwlock);
	}
	return CMD_FAIL;

#else /*  */
	unsigned long left_time = msecs_to_jiffies(pd_cmd_timeout);
	u8 cmd_status = 0xff;
	while (left_time > 0) {
		left_time =
		    wait_for_completion_timeout(&usb_pd_complete, left_time);
		if (0 == left_time) {
			pr_info("Wait for PD timeout\n");
			return CMD_FAIL;
		}
		if (left_time > 0) {
			spin_lock_irq(&usb_pd_cmd_lock);
			if (CUR_REQUESTING_PD_CMD != pd_cmd) {
				pr_info("pd_cmd %x not match rsp %x\n",
					pd_cmd, CUR_REQUESTING_PD_CMD);
				spin_unlock_irq(&usb_pd_cmd_lock);
				continue;
			}
			cmd_status = usb_pd_cmd_status;
			spin_unlock_irq(&usb_pd_cmd_lock);
			return CMD_SUCCESS;
		}
	}

#endif /*  */
	return CMD_SUCCESS;
}

u8 pd_src_pdo_cnt = 2;
u8 pd_src_pdo[VDO_SIZE] = {
	/*5V 0.9A , 5V 1.5 */
	0x5A, 0x90, 0x01, 0x2A, 0x96, 0x90, 0x01, 0x2A
};

u8 sink_svid_vdo[PD_ONE_DATA_OBJECT_SIZE];
u8 pd_snk_pdo_cnt = 3;
u8 pd_snk_pdo[VDO_SIZE];
u8 pd_rdo[PD_ONE_DATA_OBJECT_SIZE];
u8 DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 configure_DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 src_dp_caps[PD_ONE_DATA_OBJECT_SIZE];
inline void printb(const char *buf, size_t size)
{
#ifdef OHIO_DEBUG
	while (size--)
		printk("%0x ", *buf++);
	printk("\n");
#else
	buf = buf;
	size = size;
#endif
}

inline void send_initialized_setting(bool init)
{
	/* init setting for TYPE_PWR_SRC_CAP */
	static u32 init_src_caps[1] = {
		/*5V, 1.5A, Fixed */
		PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_1500MA, PDO_FIXED_FLAGS)
	};

	/* init setting for TYPE_PWR_SNK_CAP */
	static u32 init_snk_cap[3] = {
		/*5V, 0.9A, Fixed */
		PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_900MA, PDO_FIXED_FLAGS),
		/*min 5V, max 20V, power 60W, battery */
		PDO_BATT(PD_VOLTAGE_5V, PD_MAX_VOLTAGE_21V, PD_POWER_15W),
		/*min5V, max 5V, current 3A, variable */
		PDO_VAR(PD_VOLTAGE_5V, PD_MAX_VOLTAGE_21V, PD_CURRENT_3A)
	};

	/* init setting for TYPE_DP_SNK_IDENDTITY */
	static u8 snk_ident[16] = { 0 };
	static u8 snk_id_hdr[PD_ONE_DATA_OBJECT_SIZE] = { 0x00, 0x00, 0x00, 0x2c };
	static u8 snk_cert[PD_ONE_DATA_OBJECT_SIZE] = { 0x00, 0x00, 0x00, 0x00 };
	static u8 snk_prd[PD_ONE_DATA_OBJECT_SIZE] = { 0x00, 0x00, 0x00, 0x00 };
	static u8 snk_ama[PD_ONE_DATA_OBJECT_SIZE] = { 0x39, 0x00, 0x00, 0x51 };

	/* init setting for TYPE_SVID */
	static u8 init_svid[PD_ONE_DATA_OBJECT_SIZE] = { 0x00, 0x00, 0x01, 0xff };

	if (1 == init) {
		memcpy(snk_ident, snk_id_hdr, 4);
		memcpy(snk_ident + 4, snk_cert, 4);
		memcpy(snk_ident + 8, snk_prd, 4);
		memcpy(snk_ident + 12, snk_ama, 4);
	} else {

		/* send TYPE_PWR_SRC_CAP init setting */
		send_pd_msg(TYPE_PWR_SRC_CAP, (const char *)init_src_caps,
			    sizeof(init_src_caps));

		/* send TYPE_PWR_SNK_CAP init setting */
		send_pd_msg(TYPE_PWR_SNK_CAP, (const char *)init_snk_cap,
			    sizeof(init_snk_cap));

		/* send TYPE_DP_SNK_IDENDTITY init setting */
		send_pd_msg(TYPE_DP_SNK_IDENDTITY, snk_ident,
			    sizeof(snk_ident));

		/* send TYPE_SVID init setting */
		send_pd_msg(TYPE_SVID, init_svid, sizeof(init_svid));

		/* send TYPE_PWR_SRC_CAP init setting */
		send_pd_msg(TYPE_PWR_SRC_CAP, (const char *)init_src_caps,
			    sizeof(init_src_caps));
	}

}

/* circular buffer driver */
unsigned char InterfaceSendBuf[32];
unsigned char InterfaceRecvBuf[32];

#define MAX_SEND_BUF_SIZE 8
#define MAX_RECV_BUF_SIZE 8

#define AP_BUF_FRONT     0x11
#define AP_BUF_REAR      0x12
#define OCM_BUF_FRONT    0x13
#define OCM_BUF_REAR     0x14

#define AP_ACK_STATUS 0x15
#define OCM_ACK_STATUS 0x16

#define AP_BUF_START      0x18	/* 0x18-0x1f */
#define OCM_BUF_START      0x20	/* 0x20-0x27 */
unsigned char pbuf_rx_front = 0;
unsigned char pbuf_tx_rear = 0;

#define TX_BUF_FRONT AP_BUF_FRONT
#define TX_BUF_REAR AP_BUF_REAR
#define TX_BUF_START AP_BUF_START

#define RX_BUF_FRONT OCM_BUF_FRONT
#define RX_BUF_REAR OCM_BUF_REAR
#define RX_BUF_START OCM_BUF_START

#define RCVDER_ACK_STATUS AP_ACK_STATUS
#define SENDER_ACK_STATUS OCM_ACK_STATUS

#define tx_buf_rear() pbuf_tx_rear
#define rx_buf_front() pbuf_rx_front

#define up_rx_front()	OhioWriteReg(RX_BUF_FRONT, rx_buf_front())
#define up_tx_rear()	OhioWriteReg(TX_BUF_REAR, tx_buf_rear())

/*
#define tx_buf_rear() OhioReadReg(TX_BUF_REAR)
#define rx_buf_front() OhioReadReg(RX_BUF_FRONT)
#define up_rx_front()
#define up_tx_rear()
*/


#define rx_buf_rear() OhioReadReg(RX_BUF_REAR)
#define tx_buf_front() OhioReadReg(TX_BUF_FRONT)




#define buf_read(val) \
	do {	\
		*val = OhioReadReg(RX_BUF_START + rx_buf_front());\
		rx_buf_front() = (rx_buf_front() + 1) % MAX_SEND_BUF_SIZE;\
	} while (0)

#define buf_write(val) \
	do {	\
		OhioWriteReg(TX_BUF_START + tx_buf_rear(), val);	\
		tx_buf_rear() = (tx_buf_rear() + 1) % MAX_SEND_BUF_SIZE;\
	}	\
	while (0)

#define receiver_set_ack_status(val) OhioWriteReg(RCVDER_ACK_STATUS, val)
#define receiver_get_ack_status() OhioReadReg(RCVDER_ACK_STATUS)

#define sender_get_ack_status() ((OhioReadReg(SENDER_ACK_STATUS)) & 0x7F)
#define sender_set_ack_status(val) OhioWriteReg(SENDER_ACK_STATUS, val)

#define full_tx_buf()	\
	(tx_buf_front() == (tx_buf_rear() + 1) % MAX_SEND_BUF_SIZE)

inline u8 len_tx_buf(void)
{
	s8 rear = tx_buf_rear();
	u8 length = 0;
	s8 front = tx_buf_front();

	/* full, return 0 */
	if (front == ((rear + 1) % MAX_SEND_BUF_SIZE)) {
		length = 0;
#ifdef OHIO_DEBUG
		pr_info("Tx len=%x rear=%x front=%x\n", length, rear, front);
#endif
		return length;
	}

	/* if circle buffer is writable, write block data to Max high top */
	if (rear >= front)
		length = MAX_SEND_BUF_SIZE - 1 - rear;
	else
		length = front - rear - 1;

	if (rear == (MAX_SEND_BUF_SIZE - 1))
		length = 1;
#ifdef OHIO_DEBUG
	pr_info("Tx len=%x rear=%x front=%x\n", length, rear, front);
#endif

	return length;
}

#define empty_rx_buf()	\
	(rx_buf_front() == rx_buf_rear())

inline u8 len_rx_buf(void)
{
	u8 rear = rx_buf_rear();
	u8 length = 0;
	u8 front = rx_buf_front();

	if (front == rear) {
		length = 0;
#ifdef OHIO_DEBUG
		pr_info("Rx1 len=%x rear=%x front=%x\n", length, rear, front);
#endif
		return length;
	}

	if (front >= rear)
		length = (MAX_SEND_BUF_SIZE - 1 - front);
	else
		length = rear - front;

	/* read circle top */
	if (front == (MAX_SEND_BUF_SIZE - 1))
		length = 1;

#ifdef OHIO_DEBUG
	pr_info("Rx len=%x rear=%x front=%x\n", length, rear, front);
#endif
	return length;
}

inline bool buf_enque(unsigned char byte)
{
	buf_write(byte);
	return 1;
}

inline bool buf_deque(unsigned char *val)
{
	buf_read(val);
	return 1;
}

inline bool burst_deque(u8 *dat, u8 len)
{
	int ack_len = 0;
	ack_len = OhioReadBlockReg(RX_BUF_START + rx_buf_front(), len, dat);
	if (ack_len < 0 || ack_len != len)
		return 0;

	rx_buf_front() = (rx_buf_front() + len) % MAX_SEND_BUF_SIZE;

	return 1;
}

inline int burst_enque(const u8 *dat, u8 len)
{
	int rest = 0;
	if (len >= MAX_SEND_BUF_SIZE) {
		pr_info
		    ("too long length than one block size support %d (<=8)\n",
		     len);
		return 0;
	}

	rest = OhioWriteBlockReg(TX_BUF_START + tx_buf_rear(), len, dat);
	if (rest < 0) {
		pr_info("act wr error %d %d\n", rest, len);
		return 0;
	}

	tx_buf_rear() = (tx_buf_rear() + len) % MAX_SEND_BUF_SIZE;

	return len;
}

inline void reciever_reset_queue(void)
{
	rx_buf_front() = rx_buf_rear();
	up_rx_front();
}

inline void reset_queue(void)
{
	OhioWriteReg(AP_BUF_FRONT, 0);
	OhioWriteReg(AP_BUF_REAR, 0);
	OhioWriteReg(AP_ACK_STATUS, 0);
	OhioWriteReg(OCM_BUF_FRONT, 0);
	OhioWriteReg(OCM_BUF_REAR, 0);
	OhioWriteReg(OCM_ACK_STATUS, 0);
	pbuf_tx_rear = 0;
	pbuf_rx_front = 0;
}

inline void handle_intr_vector(void)
{
#ifdef SUP_OHIO_INT_VECTOR
#ifdef SUP_I2C_BURST_MODE
	u8 intr_vector[6] = { 0x00 };
	u8 status;
	if (OhioReadBlockReg(OHIO_INTERFACE_CHANGE_INT, 3, intr_vector) == 3)
		if (intr_vector[0]) {
			OhioWriteReg(OHIO_INTERFACE_CHANGE_INT, 0);
#ifdef OHIO_DEBUG
			pr_info("intr vector %x\n", intr_vector[0]);
#endif
			if (intr_vector[0] & RECEIVED_MSG) {
				polling_interface_msg(INTERACE_TIMEOUT_MS);
			}
			status = intr_vector[1];
			if (intr_vector[0] & VBUS_CHANGE) {
				/* vbus process */
				pd_vbus_control_default_func(status &
							     VBUS_STATUS);
			}
			if (intr_vector[0] & VCONN_CHANGE) {
				pd_vconn_control_default_func(status &
							      VBUS_STATUS);
			}
			if (intr_vector[0] & CC_STATUS_CHANGE) {
				pd_cc_status_default_func(intr_vector[2]);
			}
			if (intr_vector[0] & DATA_ROLE_CHANGE) {
				pd_drole_change_default_func(status &
							     VBUS_STATUS);
			}
		}
#else
	u8 intr_vector = OhioReadReg(OHIO_INTERFACE_CHANGE_INT);
	u8 status;

	if (intr_vector) {
		OhioWriteReg(OHIO_INTERFACE_CHANGE_INT, 0);
		pr_info("Intr vector %x\n", intr_vector);
		if (intr_vector & VBUS_CHANGE) {
			status = OhioReadReg(OHIO_SYSTEM_STSTUS);
			pd_vbus_control_default_func(status & VBUS_STATUS);
		}
		if (intr_vector & VCONN_CHANGE) {
			status = OhioReadReg(OHIO_SYSTEM_STSTUS);
			pd_vconn_control_default_func(status & VBUS_STATUS);
		}
		if (intr_vector & CC_STATUS_CHANGE) {
			status = OhioReadReg(NEW_CC_STATUS);
			pd_cc_status_default_func(status);
		}
		if (intr_vector & DATA_ROLE_CHANGE) {
			status = OhioReadReg(OHIO_SYSTEM_STSTUS);
			pd_drole_change_default_func(status & VBUS_STATUS);
		}
		if (intr_vector & RECEIVED_MSG)
			polling_interface_msg(INTERACE_TIMEOUT_MS);
	}
#endif
#endif
}

/* in Tx's side routine check timeout & cable down */
#define TX_ROUTINE_CHECK() do {\
	if (time_before(expire, jiffies)) {\
		pr_info("ohio TX Timeout %d\n", snd_msg_total_len);\
		return CMD_FAIL;\
	}	\
	if (atomic_read(&ohio_power_status) != 1) {\
		pr_info("ohio cable down\n");\
		return CMD_FAIL;\
	}	\
} while (0)


inline int BurstWrite(u8 RegAddr, u8 len, u8 rear, const u8 *dat)
{
	int rest = 0;
	u8 cur_len = 0;

	if ((RegAddr + len) > (TX_BUF_START + MAX_SEND_BUF_SIZE -1)) {
		cur_len = MAX_SEND_BUF_SIZE - rear;

		rest = OhioWriteBlockReg(RegAddr, cur_len, dat);
		if (rest < 0) {
			return rest;
		}
		rest = OhioWriteBlockReg(TX_BUF_START, len - cur_len, dat + cur_len);
		if (rest < 0) {
			return rest;
		}
	}
	else {
		rest = OhioWriteBlockReg(RegAddr, len, dat);
		if (rest < 0) {
			return rest;
		}
	}

	return rest;
}

/* 0, send interface msg timeout
 * 1 successfull
 */
inline u8 interface_send_msg_timeout(u8 type, u8 *pbuf, u8 len, int timeout_ms)
{
	int snd_msg_total_len = 0;
	unsigned long expire = 0;
	u8 tmp_len = 0;
	s8 rear, front;

	/* full, return 0 */
	InterfaceSendBuf[0] = len + 1;	/* cmd */
	InterfaceSendBuf[1] = type;
	if (len > 0)
		memcpy(InterfaceSendBuf + 2, pbuf, len);
	/* cmd + checksum */
	InterfaceSendBuf[len + 2] = cac_checksum(InterfaceSendBuf, len + 1 + 1);
	snd_msg_total_len = len + 3;

#ifdef OHIO_DEBUG
	pr_info("snd type=%d len=%d\n", type, snd_msg_total_len);
#endif

	expire = msecs_to_jiffies(timeout_ms) + jiffies;
	while (snd_msg_total_len > 0) {
		rear = tx_buf_rear();
		front = tx_buf_front();

		if (front != ((rear + 1 ) % MAX_SEND_BUF_SIZE)) {
			if (front) {
				tmp_len = MAX_SEND_BUF_SIZE - 1;
			}
			if (front > rear) {
				tmp_len = front - rear - 1;
			}
			else {
				tmp_len = MAX_SEND_BUF_SIZE - 1 - (rear  - front);
			}

			if (tmp_len > snd_msg_total_len)
				tmp_len = snd_msg_total_len;
			if ( tmp_len ) {
#ifdef OHIO_DEBUG
				pr_info("Tx len=%d left=%d, front=%d, rear=%d\n", tmp_len, snd_msg_total_len, front, rear);
#endif
				if (BurstWrite(TX_BUF_START + tx_buf_rear(), tmp_len, rear, InterfaceSendBuf + len + 3 - snd_msg_total_len ) < 0) {
					pr_info("send failed\n");
					return CMD_FAIL;
				}
				tx_buf_rear() = (tx_buf_rear() + tmp_len) % MAX_SEND_BUF_SIZE;
				up_tx_rear();
				snd_msg_total_len -= tmp_len;
			}
		}

		TX_ROUTINE_CHECK();
	}

	expire = msecs_to_jiffies(timeout_ms) + jiffies;
	while ((snd_msg_total_len = sender_get_ack_status()) == 0) {
		TX_ROUTINE_CHECK();
	}
	if (snd_msg_total_len == 0x01) {
#ifdef OHIO_DEBUG
		pr_info("ohio succ << %s\n", interface_to_str(InterfaceSendBuf[1]));
#endif
		return CMD_SUCCESS;
	} else {
		pr_info("ohio Ack error %d\n", snd_msg_total_len);
		printb(InterfaceSendBuf, len + 3);
		sender_set_ack_status(0x00);
		return CMD_FAIL;
	}
	return CMD_SUCCESS;
}

inline void interface_init(void)
{
	pbuf_rx_front = 0;
	pbuf_tx_rear = 0;
	memset(InterfaceRecvBuf, 0, 32);
	memset(InterfaceSendBuf, 0, 32);
	send_initialized_setting(1);
}

inline void interface_initi_setting_en(bool en)
{
	OhioWriteReg(OCM_ACK_STATUS, (en == 1) ? 0x80 : 0x00);
}

inline void pd_vbus_control_default_func(bool on)
{
	/* to enable or disable VBus in 35ms */
	pr_info("vbus control %d\n", (int)on);
}

inline void pd_vconn_control_default_func(bool on)
{
	/* to enable or disable VConn */

}

inline void pd_cc_status_default_func(u8 cc_status)
{
	/* cc status */
	pr_info("cc status %x\n", cc_status);
}

inline void pd_drole_change_default_func(bool on)
{
	/* data role changed */

}

/* routine check cable unplug & timeout*/
#define RX_ROUTINE_CHECK() \
do {	\
	if (atomic_read(&ohio_power_status) != 1) {	\
		printk("ohio RX_ROUTINE_CHECK, power line=%d\n", __LINE__); \
		goto err_cbl_down;\
	} \
	if (time_before(expire, jiffies)) {\
		printk("ohio RX_ROUTINE_CHECK, time line=%d\n", __LINE__); \
		goto err_timeout;\
	} \
} while (0)

/* Desc: polling private interface interrupt request message
 * Args: timeout,  block timeout time, if = 0, is noblock
 * Ret: if return 0, success recv one message
 *  the message pointer, it's staticly alloced in the function
 *  if > 0,  Error happen
 */
inline u8 polling_interface_msg(int timeout_ms)
{
	u8 checksum = 0;
	u8 msg_total_len = 0;	/* whole interface packek size */
	unsigned long expire = 0;
	u8 first_byte_len_data = 0x00;
	u8 length = 0;
	u8 front, rear;
	int ack_len = 0;
	u8 i = 0, j = 0;
	u8 buf[32] = { 0 };
	u8 cur_idx_len = 0;


	expire = msecs_to_jiffies(timeout_ms) + jiffies;
	first_byte_len_data = 0x00;

	/*	fetch the first byte(Length of Data) to decide how long to read
	*	Interface's Format:
	*	1Byte Len + 1Byte Type + Len Bytes Data + 1Byte checksum
	*/
	while (first_byte_len_data == 0) {
		front = rx_buf_front();
		rear = rx_buf_rear();

		RX_ROUTINE_CHECK();

		if (front == rear) {
			continue;
		}

		receiver_set_ack_status(0x00);

		if (front > rear) {
			/* read to Max Top */
			length = (MAX_RECV_BUF_SIZE - front);
			ack_len = OhioReadBlockReg(RX_BUF_START + rx_buf_front(), length, buf);
			if (ack_len < 0 || ack_len != length)
				goto err_cbl_down;

			rx_buf_front() = (rx_buf_front() + length) % MAX_RECV_BUF_SIZE;
			cur_idx_len = length;
			#ifdef OHIO_DEBUG
			pr_info("Rx0 len=%x rear=%x front=%x\n", length, rear, front);
			#endif
			/* read from 0 to rear */
			length = rear;
			if (length != 0) {
				ack_len = OhioReadBlockReg(RX_BUF_START + rx_buf_front(), length, buf + cur_idx_len);
				if (ack_len < 0 || ack_len != length)
					goto err_cbl_down;

				rx_buf_front() = (rx_buf_front() + length) % MAX_RECV_BUF_SIZE;
				cur_idx_len += length;

				#ifdef OHIO_DEBUG
				pr_info("Rx1 len=%x rear=%x front=%x\n", length, rear, rx_buf_front() );
				#endif
			}
			up_rx_front();
		}
		else {
			length = rear - front;

			ack_len = OhioReadBlockReg(RX_BUF_START + rx_buf_front(), length, buf);
			if (ack_len < 0 || ack_len != length)
				goto err_cbl_down;
			rx_buf_front() = (rx_buf_front() + length) % MAX_RECV_BUF_SIZE;
			up_rx_front();
			cur_idx_len = length;
			#ifdef OHIO_DEBUG
			pr_info("Rx2 len=%x rear=%x front=%x\n", length, rear, front);
			#endif
		}

		first_byte_len_data = buf[0];
		/* Check Data arrived */
		if (first_byte_len_data > 32 || first_byte_len_data == 0) {
			goto err_rcv_len;
		}

		msg_total_len = buf[0] + 2;
		/* read two packet */
		if ((first_byte_len_data + 2) < cur_idx_len) {
			checksum = 0;
			for (j = 0; j < msg_total_len; j++)
				checksum += buf[j];

			/* correct chksm */
			if (checksum != 0)
				goto err_chk_sum;

			/* ack OCM that recieved sucess */
			receiver_set_ack_status(0x01);
			pr_info("\nohio >>%s\n", interface_to_str(buf[1]));
			dispatch_rcvd_pd_msg((PD_MSG_TYPE) buf[1],
						&(buf[2]), buf[0] - 1);

			memcpy(InterfaceRecvBuf, buf + msg_total_len, cur_idx_len - msg_total_len);
			cur_idx_len -= msg_total_len;
			goto LEFT_PART;
		}
		else if ((first_byte_len_data + 2) == cur_idx_len) {
			checksum = 0;
			for (j = 0; j < msg_total_len; j++)
				checksum += buf[j];

			/* correct chksm */
			if (checksum != 0)
				goto err_chk_sum;

			memcpy(InterfaceRecvBuf, buf, cur_idx_len);
			goto succ_ack;
		}
		else { /* not complete*/
			memcpy(InterfaceRecvBuf, buf, cur_idx_len);
			goto LEFT_PART;
		}

	}


LEFT_PART:

	/* reset timer's value */
	expire = msecs_to_jiffies(timeout_ms) + jiffies;

	/*msg_total_len =	1B Data length + 1B type + Raw Data +1B checksum
	*	If Length is Ok, fetch Left interface (msg_total_len - 1) :
	*	Type + Raw Data + checksum
	*/
	msg_total_len = InterfaceRecvBuf[0] + 2;

#ifdef OHIO_DEBUG
	pr_info("Rx begtin index_len=%x msg_total=%x\n", cur_idx_len, msg_total_len);
#endif

	/* read the left data (start &InterfaceRecvBuf[1]) */

	for (i = cur_idx_len; i < msg_total_len; ) {
		front = rx_buf_front();
		rear = rx_buf_rear();

		RX_ROUTINE_CHECK();

		if (front == rear) {
			continue;
		}

		/* QUICKLY read until circle buffer is empty !
		*Rcv_len is Max the once can be read(burst mode)
		*/
		if (front > rear) {
			/* read to Max Top */
			length = (MAX_RECV_BUF_SIZE - front);
			ack_len = OhioReadBlockReg(RX_BUF_START + rx_buf_front(), length, buf);
			if (ack_len < 0 || ack_len != length)
				goto err_cbl_down;

			rx_buf_front() = (rx_buf_front() + length) % MAX_RECV_BUF_SIZE;
			memcpy(InterfaceRecvBuf + i, buf, length);
			i += length;

			#ifdef OHIO_DEBUG
			pr_info("Rx00 len=%x rear=%x front=%x\n", length, rear, front);
			#endif

			/* read from 0 to rear */
			length = rear;
			if (length != 0) {
				ack_len = OhioReadBlockReg(RX_BUF_START + rx_buf_front(), length, buf);
				if (ack_len < 0 || ack_len != length)
					goto err_cbl_down;

				memcpy(InterfaceRecvBuf + i, buf, length);
				rx_buf_front() = (rx_buf_front() + length) % MAX_RECV_BUF_SIZE;
				i += length;

				#ifdef OHIO_DEBUG
				pr_info("Rx11 len=%x rear=%x front=%x\n", length, rear, front);
				#endif
			}

			up_rx_front();
		}
		else {
			length = rear - front;

			ack_len = OhioReadBlockReg(RX_BUF_START + rx_buf_front(), length, buf);
			if (ack_len < 0 || ack_len != length)
				goto err_cbl_down;
			memcpy(InterfaceRecvBuf + i, buf, length);
			rx_buf_front() = (rx_buf_front() + length) % MAX_RECV_BUF_SIZE;
			i += length;
			up_rx_front();

			#ifdef OHIO_DEBUG
			pr_info("Rx22 len=%x rear=%x front=%x\n", length, rear, front);
			#endif
		}

#ifdef OHIO_DEBUG
		pr_info("! Rx index_len=%x msg_total=%x\n", i, msg_total_len);
#endif
		if (i == msg_total_len){
			checksum = 0;
			for (j = 0; j < msg_total_len; j++)
				checksum += InterfaceRecvBuf[j];

			/* correct chksm */
			if (checksum != 0)
				goto err_chk_sum;

			goto succ_ack;
		}
		else if (i > msg_total_len){
			checksum = 0;
			for (j = 0; j < msg_total_len; j++)
				checksum += InterfaceRecvBuf[j];

			/* correct chksm */
			if (checksum != 0)
				goto err_chk_sum;

			/* ack OCM that recieved sucess */
			receiver_set_ack_status(0x01);
			pr_info("\nohio2 >>%s\n", interface_to_str(buf[1]));
			dispatch_rcvd_pd_msg((PD_MSG_TYPE) InterfaceRecvBuf[1],
						&(InterfaceRecvBuf[2]), InterfaceRecvBuf[0] - 1);
			memcpy(buf, InterfaceRecvBuf, i);
			memcpy(InterfaceRecvBuf, buf + msg_total_len, i - msg_total_len);
			i -= msg_total_len;
			cur_idx_len = i;

			goto LEFT_PART;
		}

	}
succ_ack:
	/* ack OCM that recieved sucess */
	receiver_set_ack_status(0x01);
	pr_info("ohio3 >>%s\n", interface_to_str(InterfaceRecvBuf[1]));
	/* dispatch received message to meet in critical timming 25ms */
	dispatch_rcvd_pd_msg((PD_MSG_TYPE) InterfaceRecvBuf[1],
						&(InterfaceRecvBuf[2]), InterfaceRecvBuf[0] - 1);
	return 0;


err_timeout:
	receiver_set_ack_status(0x03);
	printb(InterfaceRecvBuf, i);
	reciever_reset_queue();
	pr_info("err: rx timeout\n");
	return 2;

err_rcv_len:
	pr_info("err: rx length error len = %d\n", first_byte_len_data);
	receiver_set_ack_status(0x02);
	reciever_reset_queue();
	return 3;

err_chk_sum:
	/* incorrect chksm */
	receiver_set_ack_status(0x02);
	reciever_reset_queue();
	printb(InterfaceRecvBuf, i);
	pr_info("err: rx checksum error\n");
	return 4;

err_cbl_down:
	pr_info("err: when receiving, calbe unplug\n");
	reciever_reset_queue();

	return 5;
}

inline void interface_send_dp_caps(void)
{
	memcpy(InterfaceSendBuf + 2, configure_DP_caps, 4);
	memcpy(InterfaceSendBuf + 2 + 4, DP_caps, 4);
	interface_send_msg_timeout(TYPE_DP_SNK_CFG, InterfaceSendBuf + 2,
				   4 + 4, INTERFACE_TIMEOUT);
}

inline void interface_send_status(u8 cmd_type, u8 status)
{
	InterfaceSendBuf[2] = cmd_type;
	InterfaceSendBuf[3] = status;
	interface_send_msg_timeout(TYPE_RESPONSE_TO_REQ, InterfaceSendBuf + 2,
				   2, INTERFACE_TIMEOUT);
}

/* define max request current 3A and voltage 5V */
#define MAX_REQUEST_VOLTAGE 5000

#define MAX_REQUEST_CURRENT 2000

#define set_rdo_value(v0, v1, v2, v3)	\
	do {				\
		pd_rdo[0] = (v0);	\
		pd_rdo[1] = (v1);	\
		pd_rdo[2] = (v2);	\
		pd_rdo[3] = (v3);	\
	} while (0)

u8 sel_voltage_pdo_index = 0x02;
/* default request max RDO */
inline u8 build_rdo_from_source_caps(u8 obj_cnt, u8 *buf)
{
	u8 i = 0;
	u16 pdo_h = 0, pdo_l = 0, pdo_h_tmp, pdo_l_tmp;
	u16 max_request_ma;
	u32 pdo_max = 0, pdo_max_tmp = 0;

	obj_cnt &= 0x07;

	/* find the max voltage pdo */
	for (i = 0; i < obj_cnt; i++) {
		pdo_l_tmp = buf[i * 4 + 0];
		pdo_l_tmp |= (u16) buf[i * 4 + 1] << 8;
		pdo_h_tmp = buf[i * 4 + 2];
		pdo_h_tmp |= (u16) buf[i * 4 + 3] << 8;

		/* get max voltage now */
		pdo_max_tmp =
		    (u16) (((((pdo_h_tmp & 0xf) << 6) | (pdo_l_tmp >> 10)) &
			    0x3ff) * 50);
		if (pdo_max_tmp > pdo_max) {
			pdo_max = pdo_max_tmp;
			pdo_l = pdo_l_tmp;
			pdo_h = pdo_h_tmp;
			sel_voltage_pdo_index = i;
		}
	}
	pr_info("ohio maxV=%d, cnt %d index %d\n", pdo_max_tmp, obj_cnt,
		sel_voltage_pdo_index);
	if ((pdo_h & (3 << 14)) != (PDO_TYPE_BATTERY >> 16)) {
		max_request_ma = (u16) ((pdo_l & 0x3ff) * 10);
		pr_info("ohio maxMa %d\n", max_request_ma);
		/* less than 900mA */
		if (max_request_ma < MAX_REQUEST_CURRENT) {
			pdo_max =
			    RDO_FIXED(sel_voltage_pdo_index + 1, max_request_ma,
				      max_request_ma, 0);
			pdo_max |= RDO_CAP_MISMATCH;
			set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
				      (pdo_max >> 16) & 0xff,
				      (pdo_max >> 24) & 0xff);
			return 1;
		} else {
			pdo_max =
			    RDO_FIXED(sel_voltage_pdo_index + 1,
				      MAX_REQUEST_CURRENT, MAX_REQUEST_CURRENT,
				      0);
			set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
				      (pdo_max >> 16) & 0xff,
				      (pdo_max >> 24) & 0xff);

			return 1;
		}
	} else {
		pdo_max =
		    RDO_FIXED(sel_voltage_pdo_index + 1, MAX_REQUEST_CURRENT,
			      MAX_REQUEST_CURRENT, 0);
		set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
			      (pdo_max >> 16) & 0xff, (pdo_max >> 24) & 0xff);
		return 1;
	}

	pr_info("ohio RDO Mismatch !!!\n");
	set_rdo_value(0x0A, 0x28, 0x00, 0x10);

	return 0;
}

inline u32 change_bit_order(u8 *pbuf)
{
    return ((u32)pbuf[3] << 24) | ((u32)pbuf[2] << 16)
        | ((u32)pbuf[1] << 8) | pbuf[0];
}
inline u8 pd_check_requested_voltage(u32 rdo)
{
        int max_ma = rdo & 0x3FF;
        int op_ma = (rdo >> 10) & 0x3FF;
        int idx = rdo >> 28;

        u32 pdo;
        u32 pdo_max;

        if (!idx || idx > pd_src_pdo_cnt)
        {
               pr_info("rdo = %x, Requested RDO is %d, Provided RDO number is %d\n", rdo, (unsigned int)idx, (unsigned int)pd_src_pdo_cnt);
                return 0; /* Invalid index */
        }
        //Update to pass TD.PD.SRC.E12 Reject Request
        pdo = change_bit_order(pd_src_pdo + ((idx - 1) * 4));
        pdo_max = (pdo & 0x3ff);
        pr_info("pdo_max = %x\n", pdo_max);

        //TRACE3("Requested  %d/~%d mA, idx %d\n",      (u16)op_ma * 10, (u16)max_ma *10, (u16)idx);
        /* check current ... */
        if (op_ma > pdo_max)//Update to pass TD.PD.SRC.E12 Reject Request
                return 0; /* too much op current */
        if (max_ma > pdo_max)//Update to pass TD.PD.SRC.E12 Reject Request
                return 0; /* too much max current */

        return 1;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * return:  0, fail;   1, success
  */
inline u8 recv_pd_source_caps_default_callback(void *para, u8 para_len)
{
	u8 *pdo = 0;
	pdo = (u8 *) para;
	if (para_len % 4 != 0)
		return 0;
	if (build_rdo_from_source_caps(para_len / 4, para)) {
		interface_send_request();
		pr_info("ohio Snd RDO %x %x %x %x succ\n", pd_rdo[0], pd_rdo[1],
			pd_rdo[2], pd_rdo[3]);
	}
	return 1;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * return:  0, fail;   1, success
  */
inline u8 recv_pd_sink_caps_default_callback(void *para, u8 para_len)
{
	u8 *pdo = 0;
	pr_info("enterrecv_pd_sink_caps_default_callback()\n");
	pdo = (u8 *) para;
	if (para_len % 4 != 0)
		return 0;
	if (para_len > VDO_SIZE)
		return 0;
	memcpy(pd_snk_pdo, para, para_len);
	pd_snk_pdo_cnt = para_len / 4;
	return 1;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * return:  0, fail;   1, success
  */
inline u8 recv_pd_pwr_object_req_default_callback(void *para, u8 para_len)
{
	u8 *pdo = (u8 *) para;
	u32 rdo = 0;
	if (para_len != 4)
		return 0;

	rdo = pdo[0] | (pdo[1] << 8) | (pdo[2] << 16) | (pdo[3] << 24);
	if (pd_check_requested_voltage(rdo))
		send_accept();
	else
		interface_send_reject();

	return 1;
}

/* Recieve accept message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
  * return:  0, fail;   1, success
  */
inline u8 recv_pd_accept_default_callback(void *para, u8 para_len)
{
	para = para;
	para_len = para_len;
	return 1;
}

/* Recieve reject message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
  * return:  0, fail;   1, success
  */
inline u8 recv_pd_reject_default_callback(void *para, u8 para_len)
{
	para = para;
	para_len = para_len;
	return 1;
}

/* Recieve reject message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
  * return:  0, fail;   1, success
  */
inline u8 recv_pd_goto_min_default_callback(void *para, u8 para_len)
{
	para = para;
	para_len = para_len;
	return 1;
}

#ifdef SUPP_VDM_CHARGING
u8 recv_pd_vdm_default_callback(void *para, u8 para_len)
{
	interface_send_vdm_data(para, para_len);
	return 1;
}
#endif

/*PD Status command response, default callback function.
  *It can be change by customer for redevelopment
  * Byte0: CC status from ohio
  * Byte1: misc status from ohio
  * Byte2: debug ocm FSM state from ohio
  */
u8 cc_status = 0;
u8 misc_status = 0;
u8 pd_fsm_status = 0;
inline void interface_get_status_result(void)
{
	cc_status = InterfaceRecvBuf[3];
	pr_info("ohio CC Status:%x\n", cc_status);
	misc_status = InterfaceRecvBuf[4];
	pr_info("ohio misc status:%x\n", misc_status);
	pd_fsm_status = InterfaceRecvBuf[5];
	pr_info("ohio pd_fsm_status:%x\n", pd_fsm_status);
}

inline u8 recv_pd_cmd_rsp_default_callback(void *para, u8 para_len)
{
	u8 need_notice_pd_cmd = 0;
	u8 pd_cmd = 0;
	para = para;
	para_len = para_len;
	pd_cmd = RESPONSE_REQ_TYPE();
	switch (RESPONSE_REQ_TYPE()) {
	case TYPE_PD_STATUS_REQ:
		interface_get_status_result();
		need_notice_pd_cmd = 1;
		break;
	case TYPE_DSWAP_REQ:
		need_notice_pd_cmd = 1;
		if (RESPONSE_REQ_RESULT() == CMD_SUCCESS)
			pr_info("pd_cmd DRSwap result is successful\n");
		else if (RESPONSE_REQ_RESULT() == CMD_REJECT)
			pr_info("pd_cmd DRSwap result is rejected\n");
		else if (RESPONSE_REQ_RESULT() == CMD_BUSY)
			pr_info("pd_cmd DRSwap result is busy\n");
		else if (RESPONSE_REQ_RESULT() == CMD_FAIL)
			pr_info("pd_cmd DRSwap result is fail\n");
		else
			pr_info("pd_cmd DRSwap result is unknown\n");
		break;
	case TYPE_PSWAP_REQ:
		need_notice_pd_cmd = 1;
		if (RESPONSE_REQ_RESULT() == CMD_SUCCESS)
			pr_info("pd_cmd PRSwap result is successful\n");
		else if (RESPONSE_REQ_RESULT() == CMD_REJECT)
			pr_info("pd_cmd PRSwap result is rejected\n");
		else if (RESPONSE_REQ_RESULT() == CMD_BUSY)
			pr_info("pd_cmd PRSwap result is busy\n");
		else if (RESPONSE_REQ_RESULT() == CMD_FAIL)
			pr_info("pd_cmd PRSwap result is fail\n");
		else
			pr_info("pd_cmd PRSwap result is unknown\n");
		break;
	default:
		break;
	}
	if (need_notice_pd_cmd) {

#ifdef USB_PD_WAIT_LOCK
		/* check pd cmd has been locked */
		write_lock_irq(&usb_pd_cmd_rwlock);
		usb_pd_cmd_status = RESPONSE_REQ_RESULT();
		usb_pd_cmd = pd_cmd;
		if (usb_pd_cmd_counter)
			usb_pd_cmd_counter = 0;

		write_unlock_irq(&usb_pd_cmd_rwlock);

#else /*  */
		spin_lock_irq(&usb_pd_cmd_lock);
		CUR_REQUESTING_PD_CMD = pd_cmd;
		usb_pd_cmd_status = RESPONSE_REQ_RESULT();
		spin_unlock_irq(&usb_pd_cmd_lock);
		complete(&usb_pd_complete);

#endif /*  */
	}
	return CMD_SUCCESS;
}

inline u8 recv_pd_hard_rst_default_callback(void *para, u8 para_len)
{
	pr_info("recv pd hard reset\n");

	return CMD_SUCCESS;
}

/* Recieve Data Role Swap message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through init_pd_msg_callback, it it pd_callback is not 0, using the default
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * return:  0, fail;   1, success
  */
inline u8 recv_pd_dswap_default_callback(void *para, u8 para_len)
{
	/* dswap just notice AP, do nothing */
	return 1;
}

/* Recieve Power Role Swap message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through init_pd_msg_callback, it it pd_callback is not 0, using the default
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
  * return:  0, fail;   1, success
  */
inline u8 recv_pd_pswap_default_callback(void *para, u8 para_len)
{
	/* pswap just notice AP, do nothing */
	return 1;
}
static pd_callback_t pd_callback_array[256] = { 0 };

inline pd_callback_t get_pd_callback_fnc(PD_MSG_TYPE type)
{
	pd_callback_t fnc = 0;
	if (type < 256)
		fnc = pd_callback_array[type];
	return fnc;
}

void set_pd_callback_fnc(PD_MSG_TYPE type, pd_callback_t fnc)
{
	pd_callback_array[type] = fnc;
}

void init_pd_msg_callback(void)
{
	u8 i = 0;
	for (i = 0; i < 256; i++)
		pd_callback_array[i] = 0x0;
}
