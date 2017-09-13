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
/**
 * @file host_msg_private.h
 *
 * @brief Definition
 *      This is a private include file to be used by the HostMsg module only
 *
 ******************************************************************************/

#ifndef __HOST_MSG_PRIVATE_H__
#define __HOST_MSG_PRIVATE_H__

#include "struct_bitfield.h"
#include "host_msg.h"
#include "hal.h"

/******************************************************************************/
/* register bitfield definitions for Host SPI interface */
/*
 * Mapping:
 *
 * AP->Bow Mailboxes
 *   REG_GLUE_MBOX_00_ADDR - REG_GLUE_MBOX_25_ADDR
 *
 * Bow->AP Mailboxes
 *   REG_GLUE_MBOX_26_ADDR - REG_GLUE_MBOX_51_ADDR
 */

#define HOSTMSG_MBOX_REG_COUNT          26

#define HOSTMSG_AP_TO_BOW_MBOX_ADDR     UDATA(SII_REG_MBOX_00)
#define HOSTMSG_BOW_TO_AP_MBOX_ADDR     \
	(HOSTMSG_AP_TO_BOW_MBOX_ADDR + HOSTMSG_MBOX_REG_COUNT)

/* Word 0 for REG_GLUE_AP_CTRL_INT_EN_ADDR */
#define REG_GLUE_AP_CTRL_INT_EN_ADDR_SRAM_MUX_SEL_EN        \
	FIELD_DEFINE(1, 0, 0)   /* b0:0 */
#define REG_GLUE_AP_CTRL_INT_EN_ADDR_AHB_MASTER_MUX_SEL_EN  \
	FIELD_DEFINE(1, 1, 0)   /* b1:1 */
#define REG_GLUE_AP_CTRL_INT_EN_ADDR_AP_LOAD_DONE_EN        \
	FIELD_DEFINE(1, 2, 0)   /* b2:2 */
#define REG_GLUE_AP_CTRL_INT_EN_ADDR_SHUTDOWN_REQ_EN        \
	FIELD_DEFINE(1, 3, 0)   /* b3:3 */
#define REG_GLUE_AP_CTRL_INT_EN_ADDR_AP_HOST_SUPPORT_EN     \
	FIELD_DEFINE(1, 4, 0)   /* b4:4 */
#define REG_GLUE_AP_CTRL_INT_EN_ADDR_AP_HOST_READY_EN       \
	FIELD_DEFINE(1, 5, 0)   /* b5:5 */
#define REG_GLUE_AP_CTRL_INT_EN_ADDR_SW_DEFINED_RSV_0_EN    \
	FIELD_DEFINE(2, 6, 0)   /* b7:6 */
#define REG_GLUE_AP_CTRL_INT_EN_ADDR_HOST_MSG_EN            \
	FIELD_DEFINE(3, 8, 0)   /* b10:8 */
#define REG_GLUE_AP_CTRL_INT_EN_ADDR_SW_DEFINED_RSV_1_EN    \
	FIELD_DEFINE(13, 11, 0) /* b23:11 */

/* All SW defined interrupt bits combined.
 * Use HOST_MSG field only instead of this. */
#define REG_GLUE_AP_CTRL_INT_EN_ADDR_SW_DEFINED_EN          \
	FIELD_DEFINE(18, 6, 0)  /* b23:6 */

/* Word 0 of REG_GLUE_AP_CTRL_INT_STATUS_ADDR */
#define REG_GLUE_AP_CTRL_INT_STATUS_ADDR_SRAM_MUX_SEL       \
	FIELD_DEFINE(1, 0, 0)   /* b0:0 */
#define REG_GLUE_AP_CTRL_INT_STATUS_ADDR_AHB_MASTER_MUX_SEL \
	FIELD_DEFINE(1, 1, 0)   /* b1:1 */
#define REG_GLUE_AP_CTRL_INT_STATUS_ADDR_AP_LOAD_DONE       \
	FIELD_DEFINE(1, 2, 0)   /* b2:2 */
#define REG_GLUE_AP_CTRL_INT_STATUS_ADDR_SHUTDOWN_REQ       \
	FIELD_DEFINE(1, 3, 0)   /* b3:3 */
#define REG_GLUE_AP_CTRL_INT_STATUS_ADDR_SW_DEFINED_RSV_0   \
	FIELD_DEFINE(4, 4, 0)   /* b7:4 */
#define REG_GLUE_AP_CTRL_INT_STATUS_ADDR_HOST_MSG           \
	FIELD_DEFINE(3, 8, 0)   /* b10:8 */
#define REG_GLUE_AP_CTRL_INT_STATUS_ADDR_SW_DEFINED_RSV_1   \
	FIELD_DEFINE(13, 11, 0) /* b23:11 */

/* All SW defined interrupt bits combined.
 * Use HOST_MSG field only instead of this. */
#define REG_GLUE_AP_CTRL_INT_STATUS_ADDR_SW_DEFINED         \
	FIELD_DEFINE(20, 4, 0)  /* b23:4 */

/* Word 0 of REG_GLUE_INT_N_ENABLE_ADDR */
#define REG_GLUE_INT_N_ENABLE_ADDR_DSI_INT_EN               \
	FIELD_DEFINE(1, 0, 0)   /* b0:0 */
#define REG_GLUE_INT_N_ENABLE_ADDR_ETDMS_INT_EN             \
	FIELD_DEFINE(1, 1, 0)   /* b1:1 */
#define REG_GLUE_INT_N_ENABLE_ADDR_MHL_INT_EN               \
	FIELD_DEFINE(1, 2, 0)   /* b2:2 */
#define REG_GLUE_INT_N_ENABLE_ADDR_WATCHDOG_INT_EN          \
	FIELD_DEFINE(1, 3, 0)   /* b3:3 */
#define REG_GLUE_INT_N_ENABLE_ADDR_BOOT_READY_EN            \
	FIELD_DEFINE(1, 4, 0)   /* b4:4 */
#define REG_GLUE_INT_N_ENABLE_ADDR_BOOT_SUCCESS_EN          \
	FIELD_DEFINE(1, 5, 0)   /* b5:5 */
#define REG_GLUE_INT_N_ENABLE_ADDR_BOOT_FAIL_EN             \
	FIELD_DEFINE(1, 6, 0)   /* b6:6 */
#define REG_GLUE_INT_N_ENABLE_ADDR_SHUTDOWN_COMPLETE_EN     \
	FIELD_DEFINE(1, 7, 0)   /* b7:7 */
#define REG_GLUE_INT_N_ENABLE_ADDR_HOST_MSG_EN              \
	FIELD_DEFINE(3, 8, 0)   /* b10:8 */
#define REG_GLUE_INT_N_ENABLE_ADDR_SW_DEFINED_RSV_EN        \
	FIELD_DEFINE(13, 11, 0) /* b23:11 */
#define REG_GLUE_INT_N_STATUS_ADDR_SW_DEFINED_INT           \
	FIELD_DEFINE(16, 8, 0)  /* b23:8 */

/* Word 0 of REG_GLUE_INT_N_STATUS_ADDR */
#define REG_GLUE_INT_N_STATUS_ADDR_DSI_INT                  \
	FIELD_DEFINE(1, 0, 0)   /* b0:0 */
#define REG_GLUE_INT_N_STATUS_ADDR_ETDMS_INT                \
	FIELD_DEFINE(1, 1, 0)   /* b1:1 */
#define REG_GLUE_INT_N_STATUS_ADDR_MHL_INT                  \
	FIELD_DEFINE(1, 2, 0)   /* b2:2 */
#define REG_GLUE_INT_N_STATUS_ADDR_WATCHDOG_INT             \
	FIELD_DEFINE(1, 3, 0)   /* b3:3 */
#define REG_GLUE_INT_N_STATUS_ADDR_BOOT_READY               \
	FIELD_DEFINE(1, 4, 0)   /* b4:4 */
#define REG_GLUE_INT_N_STATUS_ADDR_BOOT_SUCCESS             \
	FIELD_DEFINE(1, 5, 0)   /* b5:5 */
#define REG_GLUE_INT_N_STATUS_ADDR_BOOT_FAIL                \
	FIELD_DEFINE(1, 6, 0)   /* b6:6 */
#define REG_GLUE_INT_N_STATUS_ADDR_SHUTDOWN_COMPLETE        \
	FIELD_DEFINE(1, 7, 0)   /* b7:7 */
#define REG_GLUE_INT_N_STATUS_ADDR_HOST_MSG                 \
	FIELD_DEFINE(3, 8, 0)   /* b10:8 */
#define REG_GLUE_INT_N_STATUS_ADDR_SW_DEFINED_RSV           \
	FIELD_DEFINE(13, 11, 0) /* b23:11 */
#define REG_GLUE_INT_N_STATUS_ADDR_SW_DEFINED_INT           \
	FIELD_DEFINE(16, 8, 0)  /* b23:8 */

/* Defined this way because the interrupt bits, within their field,
 * have the same relative order in both AP->Bow and Bow->AP directions,
 * even though their absolute positions differ. */
#define HOSTMSG_SW_INT_MASK     0x7         /* low 3 bits of field */

#define HOSTMSG_SW_INT_PKT_RDY  0x1
#define HOSTMSG_SW_INT_ACK      0x2
#define HOSTMSG_SW_INT_NAK      0x4

#define HOSTMSG_SW_INT_HANDSHAKE_MASK   \
	(HOSTMSG_SW_INT_ACK | HOSTMSG_SW_INT_NAK)

/****************************************************************************/
/* Message Level Definitions */

/*
 * Message Format
 *
 * Header:
 *   byte 0: Low 8 bits of Opcode
 *   byte 1: High 8 bits of Opcode
 *   byte 3: Low 8 bits of Msg Payload length
 *   byte 4: High 8 bits of Msg Payload length
 *   byte 5: Tag (0-0xFF) - same as sequence number in Cmd and Notification
 *						(input value for Cmd Response)
 *   byte 6: CRC (0-0xFF) - CRC of header (CRC field set to 0) and payload
 *
 * Payload:
 *   byte 7: First byte of Payload
 *   ....
 *   byte (Msg Payload length + 6)  last byte of message
 */

/* Word 0 of HOSTMSG_PKT (header) */
#define HOSTMSG_MSG_OPCODE          FIELD_DEFINE(16, 0, 0)  /* b15:0 */
#define HOSTMSG_MSG_DATALEN         FIELD_DEFINE(10, 16, 0) /* b25:16 */
#define HOSTMSG_MSG_RSVD            FIELD_DEFINE(6, 26, 0)  /* b31:26 */

/* Word 1 of HostMSG_MSG (header) */
#define HOSTMSG_MSG_TAG             FIELD_DEFINE(8, 0, 1)   /* b7:0 */
#define HOSTMSG_MSG_CRC             FIELD_DEFINE(8, 8, 1)   /* b15:8 */

#define HOSTMSG_MSG_HDR_SIZE        6

/* Max datalength 1023 - 0x3ff */
#define HOSTMSG_MSG_MAX_DATALEN     0x3ff
#define HOSTMSG_MSG_MAX_MSGLEN      \
	(HOSTMSG_MSG_HDR_SIZE + HOSTMSG_MSG_MAX_DATALEN)

/******************************************************************************/
/* Packet Level Definitions */

/*
 * Packet Format
 *
 * Header:
 *   byte 0:  Sequence Number - 0-0xff
 *   byte 1:  Fragmentation Info - fragment number (0-0x7f) | Last Fragment
 *							(false:0 true:0x80)
 *   byte 2:  Data Length - 1-MAX_PACKET_PAYLOAD
 *   byte 3:  CRC (8 bit CCITT CRC of header (CRC field set to 0) and payload
 *
 * Payload:
 *   byte 4:  First byte of Payload
 *   ....
 *   byte MAX_PACKET_PAYLOAD + 4: Last possible Payload byte
 */

#define MAX_HOSTMSG_PKT_SEQNUM      0xff

#define MAX_HOSTMSG_PKT_FRAGNUM     0x7f

/* May be better to access using field macros below */
#define LAST_HOSTMSG_PKT_FRAG       0x80

/* Word 0 of HOSTMSG_PKT (header) */
#define HOSTMSG_PKT_SEQNUM          FIELD_DEFINE(8, 0, 0)   /* b7:0 */
#define HOSTMSG_PKT_FRAGNUM         FIELD_DEFINE(7, 8, 0)   /* b14:8 */
#define HOSTMSG_PKT_LASTFRAG        FIELD_DEFINE(1, 15, 0)  /* b15:15 */
#define HOSTMSG_PKT_DATALEN         FIELD_DEFINE(8, 16, 0)  /* b23:16 */
#define HOSTMSG_PKT_CRC             FIELD_DEFINE(8, 24, 0)  /* b31:24 */

#define HOSTMSG_PKT_HDR_SIZE        4
#define HOSTMSG_PKT_MAX_DATALEN     \
	((HOSTMSG_MBOX_REG_COUNT * 3) - HOSTMSG_PKT_HDR_SIZE)   /* 74 bytes */
#define HOSTMSG_PKT_MAX_PKTLEN      \
	(HOSTMSG_PKT_HDR_SIZE + HOSTMSG_PKT_MAX_DATALEN)

#define MAX_HOSTMSG_PKT_RETRIES     2

/*
 * Low-level routine to send a single already-formated packet.
 * Returns SB_SUCCESS if successfully sent and ACK interrupt received.
 * Returns SB_ERROR if length > HOSTMSG_PKT_MAX_PKTLEN, timeout, or retries
 *  exceed MAX_HOSTMSG_PKT_RETRIES.
 */
int HostMsgSendPacket(uint8_t pktLen, uint8_t *pMsgPkt);

/*
 * Upper face of HostMst packet level.
 * Pass in sequence number, length of message, and pointer to formatted message.
 * Will build and transmit the packets needed to carry the message.
 * Returns SB_SUCCESS if successfully packetizes and sends the message.
 * Returns SB_ERROR if message is overlength or transmission errors occur.
 */
int HostMsgFormatAndSendPackets(uint8_t seqNum, uint16_t msgLen,
				uint8_t *pMsgData);

/******************************************************************************
 * Message buffer management functions and definitions
 ******************************************************************************/

/* Size of buffer segments.
 * Each segment in array can hold just enough data for one packet payload. */
#define HOSTMSG_BUF_SEG_SIZE        HOSTMSG_PKT_MAX_DATALEN

/* Number of segments in array.
 * May need to adjust to ensure sufficient buffer depth with no wasted space. */
#define HOSTMSG_BUF_NUM_SEGMENTS    20

/* Used to manage send and receive buffers. */
struct HostMsgBufHdl {
	uint8_t msgTag;     /* On rx, should be set as soon as message header is
			     * read. */
	uint16_t msgLength; /* length of message body and header combined
			     * calculated from header values on RX. */
	uint8_t curPktSeq;  /* scratch value. Should always be the same seqNum
			     * for all packets in a messages. */
	uint8_t curFrag;    /* scratch value. Should always match expected
			     * fragment number. */
	uint16_t curBytes;  /* Scratch value. Current number of bytes read of
			     * expected length. */
	uint8_t *pMsgBody;  /* If NULL, no data allocated (should never be this
			     * for valid handle) */
};

/*
 * Allocate memory buffer and get handle.
 * Always returns msgLength and pMsgBody set correctly (treat as constants).
 * All other fields set to 0 or NULL.
 * Returns either valid buffer or NULL.
 */
struct HostMsgBufHdl *HostMsgAllocateBuf(uint16_t expectedSize);

/* Free associated memory and return handle to buffer pool. */
void HostMsgFreeBuf(struct HostMsgBufHdl *pBufHdl);

/******************************************************************************/

/*
 * If inline single character CRC is needed, use DO_CCITT8_CRC macro. Otherwise,
 * genCrc8CCITT() will probably give better performance.
 */
#define POLY_CCITT_CRC8     0x7
#define START_CCITT_CRC8    0xff
#define DO_CCITT8_CRC(cur, next) \
({ \
	unsigned char bit; \
	unsigned short genCrc; \
	genCrc = cur ^ next; \
	for (bit = 8; bit > 0; bit--) { \
		genCrc <<= 1; \
		if (genCrc & 0xff00) \
			genCrc = 0xff & (genCrc ^ POLY_CCITT_CRC8); \
	} cur = (unsigned char) genCrc; \
})

/*
 * Helper function for HostMsg module. Generates 8 bit CCITT CRC over "length"
 *  bytes of "*data".
 *
 * Expected usage on send:
 * 1) Build complete packet or msg, including header, with CRC field set to 0.
 * 2) Call genCrc8CCITT() for complete packet or message (header and payload).
 * 3) Set CRC field to generated CRC.
 *
 * Expected usage on receive:
 * 1) Receive complete packet or message.
 * 2) Copy CRC header and set header CRC field to 0.
 * 3) Call genCrc8CCITT() for complete packet or message (header and payload).
 * 4) If generated CRC != received CRC, error detected.
 */
uint8_t GenCrc8CCITT(uint8_t *data, int32_t length);

/******************************************************************************/

/******************************************************************************
 * Message Register Access Functions & Macros
 ******************************************************************************/

/*
 * Increments pCurInput and pCurReg.
 * pCurInput should be big enough to hold all input and should be a multiple
 *  of 3 bytes.
 * This macro assumes little-endian byte order.
 *
 * pCurReg   - the address of mailbox register
 * pCurInput - the current input buffer location
 * pTemp     - pointer to a scratch uint32_t scratch register
 */
#define GET_MAILBOX_BYTES(pCurReg, pCurInput, pTemp) \
({ \
	*(pTemp) = REG_READ(pCurReg); \
	*(pTemp) &= 0xffffff; \
	*pCurInput++ = ((uint8_t *) pTemp)[0]; \
	*pCurInput++ = ((uint8_t *) pTemp)[1]; \
	*pCurInput++ = ((uint8_t *) pTemp)[2]; \
	pCurReg++; \
})

/*
 * Increments pCurOut and pCurREg.
 * pCurInput should be a multiple of 3 bytes in size.
 * This macro assumes little-endian byte order.
 *
 * pCurReg - the address of mailbox register
 * pCurOut - the current output buffer location
 * pTemp   - pointer to a uint32_t scratch register
 */
#define PUT_MAILBOX_BYTES(pCurReg, pCurOut, pTemp) \
({ \
	*(pTemp) = 0; \
	((uint8_t *)pTemp)[0] = *pCurOut++; \
	((uint8_t *)pTemp)[1] = *pCurOut++; \
	((uint8_t *)pTemp)[2] = *pCurOut++; \
	REG_WRITE(pCurReg, *pTemp); \
	pCurReg++; \
})

int ReadMailBoxBytes(uint32_t startRegNum, uint8_t byteCount,
			uint8_t *pReadBytes);
int WriteMailBoxBytes(uint32_t startRegNum, uint8_t byteCount,
			uint8_t *pWriteBytes);

/*
 * Raw read of packet from mailbox registers.
 * Verifies length, but leaves handling sequence number and fragmentation
 *  to upper-level callers.
 */
int HostMsgReadMboxPacket(uint8_t *pPacketBuffer);

/*
 * Raw write of packet to mailbox registers.
 * Verifies length, but that's about it.
 */
int HostMsgWriteMboxPacket(uint8_t *pPacketBuffer);

/*
 * Clears current inbound and outbound HostMsg interrupts.
 * Leaves inbound interrupts enabled/disabled per "enable" setting.
 */
void HostMsgClearAndSetupInts(bool enable);

/*
 * Handler for the interrupt from the AP.
 * Needs to be hooked into the interrupt handlers.
 */
void HostMsgHandleApIsr(void);

/* Return ACK/NAK for packets we have received. */
void HostMsgSetACKStatus(bool doAck);

/* Set or clear PktReady interrupt to AP. */
void HostMsgSetPktReady(bool ready);

/*
 * Updates sequence number and returns new value.
 * Semaphore protected.
 */
uint8_t HostMsgGetSeqNum(void);

/*
 * Single call to send all message types used by internal functions.
 * Semaphore protected to serialize message transmission.
 */
int HostMsgSendInternal(uint16_t msgCode,   /* bitwise OR of message Type */
			uint8_t msgSeqNum,  /* sequence number for message */
			uint8_t msgTagNum,  /* ignored except for Cmd Resp */
			uint16_t msgLength, /* length of pMsgData buffer */
			void *pMsgData);    /* message payload */

#endif /* !__HOST_MSG_PRIVATE_H__ */

