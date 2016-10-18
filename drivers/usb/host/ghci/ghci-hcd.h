#ifndef GHCI_HCD_H_
#define GHCI_HCD_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else /* __KERNEL__ */
#include <stdint.h>
#include <sys/ioctl.h>
#endif /* __KERNEL__ */

#define GHCI_IOCTL_MAGIC 'g'

#define GHCI_CLAIM_PORT   _IOW(GHCI_IOCTL_MAGIC, 0xf0, unsigned int)
#define GHCI_RELEASE_PORT _IOW(GHCI_IOCTL_MAGIC, 0xf1, unsigned int)

#define GHCI_SPEED_UNKNOWN 0
#define GHCI_SPEED_LOW     1
#define GHCI_SPEED_FULL    2
#define GHCI_SPEED_HIGH    3
#define GHCI_SPEED_SUPER   4

#define GHCI_URB_FUNCTION_MASK 0xff
#define GHCI_URB_CONTROL       0x00
#define GHCI_URB_ISOC          0x01
#define GHCI_URB_BULK          0x02
#define GHCI_URB_INT           0x03

#define GHCI_URB_TYPE_IN     0x8000
#define GHCI_URB_TYPE_CANCEL 0x4000

#define ghci_urb_function(type)    ((type) & GHCI_URB_FUNCTION_MASK)
#define ghci_urb_type_in(type)     ((type) & GHCI_URB_TYPE_IN)
#define ghci_urb_type_out(type)    (!ghci_urb_type_in(type))
#define ghci_urb_type_cancel(type) ((type) & GHCI_URB_TYPE_CANCEL)

#define GHCI_URB_SHORT_NOT_OK 0x0001
#define GHCI_URB_ISO_ASAP     0x0002
#define GHCI_URB_ZERO_PACKET  0x0004

#ifdef __KERNEL__

struct ghci_urb_header {
	u32 id;
	u16 type;
	u8 epnum;
	u32 flags;
	s32 status;
	s32 start_frame;
	s32 number_of_packets;
	s32 interval;
	s32 error_count;
	u32 length;
} __attribute__ ((packed));

struct ghci_setup_packet {
	u8 bRequestType;
	u8 bRequest;
	u16 wValue;
	u16 wIndex;
	u16 wLength;
} __attribute__ ((packed));

struct ghci_iso_packet_descriptor {
	u32 offset;
	u32 length;
	s32 status;
} __attribute__ ((packed));

#else /* __KERNEL__ */

struct ghci_urb_header {
	uint32_t id;
	uint16_t type;
	uint8_t epnum;
	uint32_t flags;
	int32_t status;
	int32_t start_frame;
	int32_t number_of_packets;
	int32_t interval;
	int32_t error_count;
	uint32_t length;
} __attribute__ ((packed));

struct ghci_setup_packet {
	uint8_t bRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __attribute__ ((packed));

struct ghci_iso_packet_descriptor {
	uint32_t offset;
	uint32_t length;
	int32_t status;
} __attribute__ ((packed));

#endif /* __KERNEL__ */

#endif /* GHCI_HCD_H_ */
