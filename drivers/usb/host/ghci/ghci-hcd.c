/*
 * ghci-hcd.c -- Generic USB host controller driver.
 *
 * Copyright (C) 2011-2013 silex technology, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* #define DEBUG */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/uio.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/wait.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
#include <../drivers/usb/core/hcd.h>
#else
#include <linux/usb/hcd.h>
#endif

#include <asm/bitops.h>
#include <asm/byteorder.h>
#include <asm/uaccess.h>

#include "ghci-hcd.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 27)
#define hcd_name(pdev) (pdev)->dev.bus_id
#else
#define hcd_name(pdev) dev_name(&(pdev)->dev)
#define device_create(cls, parent, devt, ...) \
	device_create(cls, parent, devt, NULL, __VA_ARGS__)
#endif

#define DRIVER_DESC    "Generic Host Controller"
#define DRIVER_VERSION "1.1.1a01"

static const char driver_name[] = "ghci_hcd";
static const char driver_desc[] = DRIVER_DESC;
static const char device_name[] = "ghci_device";

MODULE_AUTHOR("silex technology, Inc.");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

static int ghci_major = 0;
module_param_named(major, ghci_major, int, S_IRUGO);
MODULE_PARM_DESC(major, "Major number for ghci devices (read only)");

/* See drivers/usb/core/hcd.c */
#define USB_MAXBUS 64

#define NUM_CTL_MINORS 1
#define NUM_DEV_MINORS 15
#define NUM_MINORS (NUM_CTL_MINORS + NUM_DEV_MINORS)

#define MKDEV_DEV(dev, i) MKDEV(MAJOR(dev), MINOR(dev) + NUM_CTL_MINORS + (i))
#define MINOR_TO_DEV_INDEX(dev, minor) ((minor) - (MINOR(dev) + NUM_CTL_MINORS))

#define POWER_BUDGET 500 /* mA */

/* See USB 2.0 spec. Table 11-22 */
#define PORT_C_MASK \
	((USB_PORT_STAT_C_CONNECTION \
	| USB_PORT_STAT_C_ENABLE \
	| USB_PORT_STAT_C_SUSPEND \
	| USB_PORT_STAT_C_OVERCURRENT \
	| USB_PORT_STAT_C_RESET) << 16)

#define GHCI_MAXBUS ((NUM_DEV_MINORS + USB_MAXCHILDREN - 1) / USB_MAXCHILDREN)
#if GHCI_MAXBUS > USB_MAXBUS
#error GHCI_MAXBUS must not exceed USB_MAXBUS
#endif

#define GHCI_URB_MAX_ID    0xffff
#define GHCI_URB_NUM_FRAGS 3

struct ghci_urb {
	struct list_head rw_list;
	struct list_head cancel_list;
	struct ghci_device *dev;

	struct urb *urb;
	struct ghci_urb_header header;
	struct ghci_setup_packet *setup_packet;
	struct ghci_iso_packet_descriptor *iso_frame_desc;
};

struct ghci_port;

struct ghci_device {
	struct ghci_port *port;
	int devnum;

	struct list_head urb_read_list;
	struct list_head urb_write_list;
	struct list_head urb_cancel_list;
	size_t urb_read_pos;
	size_t urb_write_pos;
	size_t urb_cancel_pos;
	u8 urb_write_buf[sizeof(struct ghci_urb_header)];
};

struct ghci_hcd;

struct ghci_port {
	struct file *claimed;
	int speed;

	struct ghci_device *ghci_dev;
	u32 status;

	struct device *dev;
	struct ghci_hcd *ghci;
	int dev_index;
	int port_index;
};

struct ghci_platform_hcd {
	int nr_ports;
};

struct ghci_hcd {
	spinlock_t lock;
	struct ghci_port port[USB_MAXCHILDREN];
	u8 addr_map[128];
	u32 next_urb_id;
};

struct ghci_driver {
	wait_queue_head_t read_wait;

	struct platform_device *pdev[GHCI_MAXBUS];
	struct device *ctl_device;
	struct class *hcd_class;

	dev_t dev;
	struct cdev ctl_cdev;
	struct cdev dev_cdev;
};

static struct ghci_driver ghci_drv;

static inline struct usb_hcd *ghci_to_hcd(struct ghci_hcd *ghci)
{
	return container_of((void *)ghci, struct usb_hcd, hcd_priv);
}

static inline struct ghci_hcd *hcd_to_ghci(struct usb_hcd *hcd)
{
	return (struct ghci_hcd *)(hcd->hcd_priv);
}

static inline struct platform_device *hcd_to_pdev(struct usb_hcd *hcd)
{
	return container_of(hcd->self.controller, struct platform_device, dev);
}

static const char *ghci_get_speed_string(int speed)
{
	const char *str;

	switch (speed) {
	case GHCI_SPEED_HIGH: str = "high";    break;
	case GHCI_SPEED_FULL: str = "full";    break;
	case GHCI_SPEED_LOW:  str = "low";     break;
	default:              str = "unknown"; break;
	}

	return str;
}

static struct ghci_port *ghci_get_port_with_dev_index(unsigned int i)
{
	const int hcd_index = i / USB_MAXCHILDREN;
	const int port_index = i % USB_MAXCHILDREN;
	struct usb_hcd *hcd;
	struct ghci_platform_hcd *priv;
	struct ghci_hcd *ghci;

	if (hcd_index >= ARRAY_SIZE(ghci_drv.pdev)) {
		return NULL;
	}

	hcd = platform_get_drvdata(ghci_drv.pdev[hcd_index]);
	if (!hcd) {
		return NULL;
	}

	priv = hcd->self.controller->platform_data;
	if (port_index >= priv->nr_ports) {
		return NULL;
	}

	ghci = hcd_to_ghci(hcd);

	return &ghci->port[port_index];
}

/*
 * Caller must have a private hcd lock.
 */
static struct ghci_device *ghci_get_device_with_address0(struct ghci_hcd *ghci)
{
	struct usb_hcd *hcd = ghci_to_hcd(ghci);
	struct ghci_platform_hcd *priv = hcd->self.controller->platform_data;
	struct ghci_port *port;
	struct ghci_device *dev = NULL;
	int i;

	for (i = 0; i < priv->nr_ports; i++) {
		port = &ghci->port[i];
		if (!port->ghci_dev) {
			continue;
		}

		if (port->ghci_dev->devnum == 0) {
			dev = port->ghci_dev;
			break;
		}
	}

	return dev;
}

/*
 * Caller must have a private hcd lock.
 */
static struct ghci_device *ghci_get_device_with_address(
	struct ghci_hcd *ghci, int devnum)
{
	int port_index;

	if (devnum >= ARRAY_SIZE(ghci->addr_map)) {
		return NULL;
	}

	if (devnum == 0) {
		return ghci_get_device_with_address0(ghci);
	}

	port_index = ghci->addr_map[devnum];

	return ghci->port[port_index].ghci_dev;
}

static int ghci_set_address(struct ghci_device *dev, int devnum)
{
	struct ghci_port *port = dev->port;
	struct ghci_hcd *ghci = port->ghci;

	if (devnum >= ARRAY_SIZE(ghci->addr_map)) {
		return -EPROTO;
	}

	dev->devnum = devnum;
	ghci->addr_map[devnum] = port->port_index;

	return 0;
}

static void ghci_reset_address(struct ghci_device *dev)
{
	struct ghci_hcd *ghci = dev->port->ghci;

	/* reset address-port mapping */
	ghci->addr_map[dev->devnum] = 0;

	(void)ghci_set_address(dev, 0);
}

static struct ghci_urb *ghci_alloc_urb(gfp_t mem_flags)
{
	struct ghci_urb *hcurb;

	hcurb = kzalloc(sizeof(*hcurb), mem_flags);
	if (!hcurb) {
		return NULL;
	}

	INIT_LIST_HEAD(&hcurb->rw_list);
	INIT_LIST_HEAD(&hcurb->cancel_list);

	return hcurb;
}

static void ghci_free_urb(struct ghci_urb *hcurb)
{
	if (!hcurb) {
		return;
	}

	kfree(hcurb->setup_packet);
	kfree(hcurb->iso_frame_desc);

	kfree(hcurb);
}

static u32 ghci_get_next_urb_id(struct ghci_hcd *ghci)
{
	unsigned long flags;
	u32 id;

	spin_lock_irqsave(&ghci->lock, flags);

	id = ghci->next_urb_id;

	if (ghci->next_urb_id == GHCI_URB_MAX_ID) {
		ghci->next_urb_id = 0;
	} else {
		ghci->next_urb_id++;
	}

	spin_unlock_irqrestore(&ghci->lock, flags);

	return id;
}

static void ghci_fill_urb_header(
	struct ghci_hcd *ghci,
	const struct urb *urb,
	struct ghci_urb_header *hdr)
{
	int pipe = urb->pipe;

	hdr->id = ghci_get_next_urb_id(ghci);

	switch (usb_pipetype(pipe)) {
	case PIPE_CONTROL:     hdr->type = GHCI_URB_CONTROL; break;
	case PIPE_ISOCHRONOUS: hdr->type = GHCI_URB_ISOC;    break;
	case PIPE_BULK:        hdr->type = GHCI_URB_BULK;    break;
	case PIPE_INTERRUPT:   hdr->type = GHCI_URB_INT;     break;
	}

	hdr->epnum = usb_pipeendpoint(pipe);

	hdr->flags = 0;
	if (urb->transfer_flags & URB_SHORT_NOT_OK) {
		hdr->flags |= GHCI_URB_SHORT_NOT_OK;
	}
	if (urb->transfer_flags & URB_ISO_ASAP) {
		hdr->flags |= GHCI_URB_ISO_ASAP;
	}
	if (urb->transfer_flags & URB_ZERO_PACKET) {
		hdr->flags |= GHCI_URB_ZERO_PACKET;
	}

	hdr->status = urb->status;
	hdr->start_frame = urb->start_frame;
	hdr->number_of_packets = urb->number_of_packets;
	hdr->interval = urb->interval;
	hdr->error_count = urb->error_count;
	hdr->length = urb->transfer_buffer_length;

	if (usb_pipein(pipe)) {
		hdr->type |= GHCI_URB_TYPE_IN;
		if (hdr->epnum != 0) {
			hdr->epnum |= USB_DIR_IN;
		}
	}
}

static struct ghci_urb *ghci_create_control_urb(
	struct ghci_hcd *ghci,
	struct urb *urb,
	gfp_t mem_flags)
{
	struct ghci_urb *hcurb;
	struct ghci_urb_header *hdr;
	struct ghci_setup_packet *ghci_req;
	struct usb_ctrlrequest *usb_req;

	hcurb = ghci_alloc_urb(mem_flags);
	if (!hcurb) {
		return NULL;
	}
	hdr = &hcurb->header;

	hcurb->urb = urb;
	ghci_fill_urb_header(ghci, urb, hdr);

	ghci_req = kmalloc(sizeof(*ghci_req), mem_flags);
	if (!ghci_req) {
		ghci_free_urb(hcurb);
		return NULL;
	}

	usb_req = (struct usb_ctrlrequest *)urb->setup_packet;
	ghci_req->bRequestType = usb_req->bRequestType;
	ghci_req->bRequest = usb_req->bRequest;
	ghci_req->wValue = le16_to_cpu(usb_req->wValue);
	ghci_req->wIndex = le16_to_cpu(usb_req->wIndex);
	ghci_req->wLength = le16_to_cpu(usb_req->wLength);

	hcurb->setup_packet = ghci_req;

	dev_dbg(ghci_to_hcd(ghci)->self.controller,
		"> id 0x%04x epnum 0x%02x <ctrl-%s> "
		"type 0x%02x request 0x%02x value 0x%04x index 0x%04x len %u\n",
		hdr->id, hdr->epnum,
		(ghci_req->bRequestType & USB_DIR_IN) ? "in" : "out",
		ghci_req->bRequestType, ghci_req->bRequest,
		ghci_req->wValue, ghci_req->wIndex, ghci_req->wLength);

	return hcurb;
}

static struct ghci_urb *ghci_create_isoc_urb(
	struct ghci_hcd *ghci,
	struct urb *urb,
	gfp_t mem_flags)
{
	struct ghci_urb *hcurb;
	struct ghci_urb_header *hdr;
	struct ghci_iso_packet_descriptor *ghci_iso_packets;
	int nr_packets = urb->number_of_packets;
	int i;

	hcurb = ghci_alloc_urb(mem_flags);
	if (!hcurb) {
		return NULL;
	}
	hdr = &hcurb->header;

	hcurb->urb = urb;
	ghci_fill_urb_header(ghci, urb, hdr);

	ghci_iso_packets = kmalloc(
		nr_packets * sizeof(struct ghci_iso_packet_descriptor),
		mem_flags);
	if (!ghci_iso_packets) {
		ghci_free_urb(hcurb);
		return NULL;
	}

	for (i = 0; i < nr_packets; i++) {
		ghci_iso_packets[i].offset = urb->iso_frame_desc[i].offset;
		ghci_iso_packets[i].length = urb->iso_frame_desc[i].length;
		ghci_iso_packets[i].status = urb->iso_frame_desc[i].status;
	}

	hcurb->iso_frame_desc = ghci_iso_packets;

	dev_dbg(ghci_to_hcd(ghci)->self.controller,
		"> id 0x%04x epnum 0x%02x <isoc-%s> len %u\n",
		hdr->id, hdr->epnum,
		ghci_urb_type_in(hdr->type) ? "in" : "out",
		hdr->length);

	return hcurb;
}

static struct ghci_urb *ghci_create_bulk_or_int_urb(
	struct ghci_hcd *ghci,
	struct urb *urb,
	gfp_t mem_flags)
{
	struct ghci_urb *hcurb;
	struct ghci_urb_header *hdr;

	hcurb = ghci_alloc_urb(mem_flags);
	if (!hcurb) {
		return NULL;
	}
	hdr = &hcurb->header;

	hcurb->urb = urb;
	ghci_fill_urb_header(ghci, urb, hdr);

	dev_dbg(ghci_to_hcd(ghci)->self.controller,
		"> id 0x%04x epnum 0x%02x <%s-%s> len %u\n",
		hdr->id, hdr->epnum,
		(ghci_urb_function(hdr->type) == GHCI_URB_BULK) ?
			"bulk" : "intr",
		ghci_urb_type_in(hdr->type) ? "in" : "out",
		hdr->length);

	return hcurb;
}

static struct ghci_urb *ghci_create_xfer_urb(
	struct ghci_hcd *ghci,
	struct urb *urb,
	gfp_t mem_flags)
{
	struct ghci_urb *hcurb;

	switch (usb_pipetype(urb->pipe)) {
	case PIPE_CONTROL:
		hcurb = ghci_create_control_urb(ghci, urb, mem_flags);
		break;

	case PIPE_ISOCHRONOUS:
		hcurb = ghci_create_isoc_urb(ghci, urb, mem_flags);
		break;

	/* case PIPE_BULK: */
	/* case PIPE_INTERRUPT: */
	default:
		hcurb = ghci_create_bulk_or_int_urb(ghci, urb, mem_flags);
		break;
	}

	return hcurb;
}

static int ghci_internal_submit_urb(struct ghci_hcd *ghci, struct urb *urb)
{
	struct usb_hcd *hcd = ghci_to_hcd(ghci);
	struct usb_ctrlrequest *setup_packet;
	struct ghci_device *dev;
	unsigned long flags;
	int port_index;
	int retval;

	if (usb_pipedevice(urb->pipe) != 0) {
		return -EINPROGRESS;
	}

	setup_packet = (struct usb_ctrlrequest *)urb->setup_packet;
	dev_dbg(hcd->self.controller,
		"%s addr 0 request 0x%02x\n",
		__func__, setup_packet->bRequest);

	spin_lock_irqsave(&ghci->lock, flags);

	dev = ghci_get_device_with_address0(ghci);
	if (!dev) {
		retval = -ENODEV;
		goto done;
	}
	port_index = dev->port->port_index;

	switch (setup_packet->bRequest) {
	case USB_REQ_SET_ADDRESS:
		retval = ghci_set_address(
			dev, le16_to_cpu(setup_packet->wValue));
		if (retval == 0) {
			dev_dbg(hcd->self.controller,
				"SET_ADDRESS %d done, port#%d hcd#%d\n",
				dev->devnum, port_index, hcd_to_pdev(hcd)->id);
		} else {
			dev_err(hcd->self.controller,
				"SET_ADDRESS %u failed, port#%d hcd#%d\n",
				le16_to_cpu(setup_packet->wValue),
				port_index, hcd_to_pdev(hcd)->id);
		}
		break;

	default:
		retval = -EINPROGRESS;
	}

done:
	spin_unlock_irqrestore(&ghci->lock, flags);

	if (retval == 0) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
		if (urb->status == -EINPROGRESS) {
			urb->status = 0;
		}
		usb_hcd_giveback_urb(hcd, urb);
#else
		usb_hcd_giveback_urb(hcd, urb, 0);
#endif
	}

	return retval;
}

static int ghci_submit_urb(struct ghci_hcd *ghci, struct ghci_urb *hcurb)
{
	struct urb *urb = hcurb->urb;
	struct ghci_device *dev;
	unsigned long flags;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 23)
	int retval;
#endif

	spin_lock_irqsave(&ghci->lock, flags);

	dev = ghci_get_device_with_address(ghci, usb_pipedevice(urb->pipe));
	if (!dev) {
		spin_unlock_irqrestore(&ghci->lock, flags);
		return -ENODEV;
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 23)
	retval = usb_hcd_link_urb_to_ep(ghci_to_hcd(ghci), urb);
	if (retval < 0) {
		spin_unlock_irqrestore(&ghci->lock, flags);
		return retval;
	}
#endif

	hcurb->dev = dev;
	urb->hcpriv = hcurb;
	list_add_tail(&hcurb->rw_list, &dev->urb_read_list);

	spin_unlock_irqrestore(&ghci->lock, flags);

	wake_up_interruptible(&ghci_drv.read_wait);

	return 0;
}

static int ghci_update_urb(
	struct ghci_urb *hcurb,
	const struct ghci_urb_header *hdr)
{
	struct urb *urb = hcurb->urb;

	if (hdr->length > hcurb->header.length) {
		return -EOVERFLOW;
	}

	if (urb) {
		urb->status = hdr->status;
		urb->actual_length = hdr->length;
		urb->start_frame = hdr->start_frame;
		urb->interval = hdr->interval;
		urb->error_count = hdr->error_count;
	}

	return 0;
}

/*
 * Caller must have a private hcd lock.
 */
static void ghci_complete_urb(
	struct ghci_hcd *ghci,
	struct ghci_urb *hcurb,
	int status)
{
	struct usb_hcd *hcd = ghci_to_hcd(ghci);
	struct urb *urb = hcurb->urb;

	dev_dbg(hcd->self.controller,
		"< id 0x%04x len %u hcurb->status %d status %d\n",
		hcurb->header.id, hcurb->header.length,
		hcurb->header.status, status);

	if (urb && usb_pipeisoc(urb->pipe)) {
		struct ghci_iso_packet_descriptor *desc = hcurb->iso_frame_desc;
		int i;

		urb->actual_length = 0;

		for (i = 0; i < urb->number_of_packets; i++) {
			urb->actual_length += desc[i].length;

			urb->iso_frame_desc[i].status = desc[i].status;
			urb->iso_frame_desc[i].actual_length = desc[i].length;
		}
	}

	list_del(&hcurb->rw_list);
	list_del(&hcurb->cancel_list);
	ghci_free_urb(hcurb);

	if (urb) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 23)
		usb_hcd_unlink_urb_from_ep(hcd, urb);
#endif

		urb->hcpriv = NULL;

		/* usb_hcd_giveback_urb() can reenter this function */
		spin_unlock(&ghci->lock);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
		if (urb->status == -EINPROGRESS) {
			urb->status = status;
		}
		usb_hcd_giveback_urb(hcd, urb);
#else
		usb_hcd_giveback_urb(hcd, urb, status);
#endif

		spin_lock(&ghci->lock);
	}
}

static ssize_t ghci_build_read_kvec(
	struct kvec *iov,
	unsigned long nr,
	struct ghci_urb *hcurb)
{
	struct urb *urb = hcurb->urb;
	int i = 0;

	if (nr < GHCI_URB_NUM_FRAGS) {
		return -ENOSPC;
	}

	memset(iov, 0, nr * sizeof(struct kvec));

	/* header */
	iov[i].iov_base = &hcurb->header;
	iov[i].iov_len = sizeof(hcurb->header);
	i++;

	/* leading bytes of data */
	switch (ghci_urb_function(hcurb->header.type)) {
	case GHCI_URB_CONTROL:
		iov[i].iov_base = hcurb->setup_packet;
		iov[i].iov_len = sizeof(struct ghci_setup_packet);
		break;

	case GHCI_URB_ISOC:
		iov[i].iov_base = hcurb->iso_frame_desc;
		iov[i].iov_len =
			hcurb->header.number_of_packets *
			sizeof(struct ghci_iso_packet_descriptor);
		break;

	/* case GHCI_URB_BULK: */
	/* case GHCI_URB_INT: */
	default:
		break;
	}
	if (iov[i].iov_base) {
		i++;
	}

	/* data */
	if (urb && usb_pipeout(urb->pipe) &&
		(urb->transfer_buffer_length > 0)) {

		iov[i].iov_base = urb->transfer_buffer;
		iov[i].iov_len = urb->transfer_buffer_length;
		i++;
	}

	return i;
}

/*
 * Note that a urb header is not included in write kvec.
 * It is received by an application in user-space separately.
 */
static ssize_t ghci_build_write_kvec(
	struct kvec *iov,
	unsigned long nr,
	struct ghci_urb *hcurb)
{
	struct urb *urb = hcurb->urb;
	int i = 0;

	if (nr < GHCI_URB_NUM_FRAGS) {
		return -ENOSPC;
	}

	memset(iov, 0, nr * sizeof(struct kvec));

	/* no header here */

	/* leading bytes of data */
	switch (ghci_urb_function(hcurb->header.type)) {
	case GHCI_URB_ISOC:
		iov[i].iov_base = hcurb->iso_frame_desc;
		iov[i].iov_len =
			hcurb->header.number_of_packets *
			sizeof(struct ghci_iso_packet_descriptor);
		break;

	/* case GHCI_URB_CONTROL: */
	/* case GHCI_URB_BULK: */
	/* case GHCI_URB_INT: */
	default:
		break;
	}
	if (iov[i].iov_base) {
		i++;
	}

	/* data */
	if (urb && usb_pipein(urb->pipe) && (urb->actual_length > 0)) {
		iov[i].iov_base = urb->transfer_buffer;
		iov[i].iov_len = urb->actual_length;
		i++;
	}

	return i;
}

static void ghci_seek_kvec(
	struct kvec *iov,
	unsigned long nr,
	size_t offset,
	void **pos,
	ssize_t *len)
{
	unsigned long i;

	for (i = 0; i < nr; i++) {
		if (offset < iov[i].iov_len) {
			break;
		}

		offset -= iov[i].iov_len;
	}

	if (i == nr) {
		*pos = iov[nr - 1].iov_base + iov[nr - 1].iov_len;
		*len = 0;
		return;
	}

	*pos = iov[i].iov_base + offset;
	*len = iov[i].iov_len - offset;
}

static int ghci_dev_readable(struct ghci_port *port)
{
	struct ghci_hcd *ghci = port->ghci;
	struct ghci_device *dev;
	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&ghci->lock, flags);

	dev = port->ghci_dev;

	if (!list_empty(&dev->urb_read_list) ||
		!list_empty(&dev->urb_cancel_list)) {

		retval = 1;
	}

	spin_unlock_irqrestore(&ghci->lock, flags);

	return retval;
}

/*
 * Caller must have a private hcd lock.
 */
static ssize_t ghci_read_cancel_urb(
	struct ghci_device *dev,
	char __user *buf,
	size_t count,
	unsigned long *irqflags)
{
	struct ghci_hcd *ghci = dev->port->ghci;
	struct ghci_urb *hcurb;
	struct ghci_urb_header hdr;
	ssize_t len = 0;

	if (list_empty(&dev->urb_cancel_list)) {
		return 0;
	}

	hcurb = list_entry(
		dev->urb_cancel_list.next,
		struct ghci_urb, cancel_list);

	memcpy(&hdr, &hcurb->header, sizeof(hdr));
	hdr.type |= GHCI_URB_TYPE_CANCEL;
	hdr.length = 0;

	if (dev->urb_cancel_pos < sizeof(hdr)) {
		len = sizeof(hdr) - dev->urb_cancel_pos;
		if (len > count) {
			len = count;
		}

		spin_unlock_irqrestore(&ghci->lock, *irqflags);

		if (copy_to_user(buf, (u8 *)&hdr + dev->urb_cancel_pos, len)) {
			spin_lock_irqsave(&ghci->lock, *irqflags);
#ifdef DEBUG
			printk(KERN_DEBUG "%s EFAULT dst %p src %p len %d/%u\n",
				__func__, buf, (u8 *)&hdr + dev->urb_cancel_pos,
				len, count);
#endif
			return -EFAULT;
		}

		spin_lock_irqsave(&ghci->lock, *irqflags);

		dev->urb_cancel_pos += len;
	}

	if (dev->urb_cancel_pos == sizeof(hdr)) {
		list_del_init(&hcurb->cancel_list);
		dev->urb_cancel_pos = 0;
	}

	return len;
}

/*
 * Caller must have a private hcd lock and ensure one or more urbs in
 * read or cancel list of the device.
 */
static ssize_t ghci_read_urb(
	struct ghci_device *dev,
	char __user *buf,
	size_t count,
	unsigned long *irqflags)
{
	struct kvec frags[GHCI_URB_NUM_FRAGS];
	struct ghci_hcd *ghci = dev->port->ghci;
	struct ghci_urb *hcurb;
	size_t hcurb_length;
	ssize_t len = 0;
	ssize_t nr;
	void *pos = NULL;

	if (dev->urb_read_pos == 0) {
		len = ghci_read_cancel_urb(dev, buf, count, irqflags);
		if (len != 0) {
			goto done;
		}
	}

	hcurb = list_entry(dev->urb_read_list.next, struct ghci_urb, rw_list);

	nr = ghci_build_read_kvec(frags, ARRAY_SIZE(frags), hcurb);
	if (nr < 0) {
		len = nr;
		goto done;
	}

	ghci_seek_kvec(frags, nr, dev->urb_read_pos, &pos, &len);
	if (len > count) {
		len = count;
	}

	/* update length before unlock to notify this urb is just being read */
	dev->urb_read_pos += len;

	spin_unlock_irqrestore(&ghci->lock, *irqflags);
	if (copy_to_user(buf, pos, len)) {
#ifdef DEBUG
		printk(KERN_DEBUG "%s EFAULT dst %p src %p len %d/%u\n",
			__func__, buf, pos, len, count);
#endif
		spin_lock_irqsave(&ghci->lock, *irqflags);
		/* restore length if do nothing */
		dev->urb_read_pos -= len;
		len = -EFAULT;
		goto done;
	}
	spin_lock_irqsave(&ghci->lock, *irqflags);

	hcurb_length = iov_length((struct iovec *)frags, nr);
	if (dev->urb_read_pos == hcurb_length) {
		list_move_tail(&hcurb->rw_list, &dev->urb_write_list);
		dev->urb_read_pos = 0;
	}

done:
	return len;
}

static ssize_t ghci_dev_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *pos)
{
	struct ghci_port *port = file->private_data;
	struct ghci_hcd *ghci = port->ghci;
	struct usb_hcd *hcd = ghci_to_hcd(ghci);
	struct ghci_device *dev;
	unsigned long flags;
	ssize_t retval;

	retval = wait_event_interruptible(
		ghci_drv.read_wait, (ghci_dev_readable(port) != 0));
	if (retval < 0) {
		return retval;
	}

	spin_lock_irqsave(&ghci->lock, flags);

	dev = port->ghci_dev;

	if (unlikely(!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags))) {
		retval = -ESHUTDOWN;
		dev_dbg(hcd->self.controller, "%s hardware unaccessible\n",
			__func__);
		goto done;
	}

	retval = ghci_read_urb(dev, buf, count, &flags);

done:
	spin_unlock_irqrestore(&ghci->lock, flags);

	return retval;
}

/*
 * Caller must have a private hcd lock.
 */
static ssize_t ghci_store_urb_header(
	struct ghci_device *dev,
	const char __user *buf,
	size_t count,
	unsigned long *irqflags)
{
	struct ghci_hcd *ghci = dev->port->ghci;
	ssize_t len = 0;
	u8 *to;

	if (dev->urb_write_pos < sizeof(dev->urb_write_buf)) {
		len = sizeof(dev->urb_write_buf) - dev->urb_write_pos;
		if (len > count) {
			len = count;
		}

		spin_unlock_irqrestore(&ghci->lock, *irqflags);

		to = dev->urb_write_buf + dev->urb_write_pos;
		if (copy_from_user(to, buf, len)) {
			spin_lock_irqsave(&ghci->lock, *irqflags);
#ifdef DEBUG
			printk(KERN_DEBUG "%s EFAULT dst %p src %p len %d/%u\n",
				__func__, to, buf, len, count);
#endif
			return -EFAULT;
		}

		spin_lock_irqsave(&ghci->lock, *irqflags);
	}

	return len;
}

static int ghci_check_urb_header(const struct ghci_urb_header *hdr)
{
	u16 type_mask =
		GHCI_URB_TYPE_IN |
		GHCI_URB_TYPE_CANCEL |
		GHCI_URB_FUNCTION_MASK;
	u8 epnum_mask =
		USB_ENDPOINT_NUMBER_MASK |
		USB_ENDPOINT_DIR_MASK;
	u32 supported_flags =
		GHCI_URB_SHORT_NOT_OK |
		GHCI_URB_ISO_ASAP |
		GHCI_URB_ZERO_PACKET;

	if ((hdr->id > GHCI_URB_MAX_ID) ||
		(hdr->type & ~type_mask) ||
		(hdr->epnum & ~epnum_mask) ||
		(hdr->flags & ~supported_flags)) {

		return -EINVAL;
	}

	return 0;
}

/*
 * Caller must have a private hcd lock.
 */
static struct ghci_urb *ghci_find_write_urb(
	struct ghci_device *dev,
	const struct ghci_urb_header *hdr)
{
	const struct ghci_urb_header *reqhdr;
	struct ghci_urb *hcurb;
	int found = 0;

	list_for_each_entry(hcurb, &dev->urb_write_list, rw_list) {
		reqhdr = &hcurb->header;
		if ((reqhdr->epnum == hdr->epnum) && (reqhdr->id == hdr->id)) {
			found = 1;
			break;
		}
	}

	return found ? hcurb : NULL;
}

/*
 * Caller must have a private hcd lock.
 */
static ssize_t ghci_write_urb(
	struct ghci_device *dev,
	const char __user *buf,
	size_t count,
	unsigned long *irqflags)
{
	struct ghci_port *port = dev->port;
	struct ghci_hcd *ghci = port->ghci;
	struct ghci_urb_header *hdr =
		(struct ghci_urb_header *)dev->urb_write_buf;
	struct kvec frags[GHCI_URB_NUM_FRAGS];
	struct ghci_urb *hcurb;
	size_t hcurb_length;
	ssize_t len;
	ssize_t nr;
	int retval;
	void *pos = NULL;

	if (list_empty(&dev->urb_write_list)) {
		len = -ENOSPC;
		goto done;
	}

	len = ghci_store_urb_header(dev, buf, count, irqflags);
	if (len < 0) {
		goto done;
	}
	dev->urb_write_pos += len;

	if (dev->urb_write_pos == sizeof(dev->urb_write_buf)) {
		/* TODO: fix me: these checks are run twice */

		retval = ghci_check_urb_header(hdr);
		if (retval < 0) {
			dev->urb_write_pos = 0;
			len = retval;
			goto done;
		}
		if (ghci_urb_type_cancel(hdr->type)) {
			dev->urb_write_pos = 0;
			len = -EINVAL;
			goto done;
		}

		hcurb = ghci_find_write_urb(dev, hdr);
		if (!hcurb) {
			dev->urb_write_pos = 0;
			len = -EINVAL;
			goto done;
		}

		retval = ghci_update_urb(hcurb, hdr);
		if (retval < 0) {
			dev->urb_write_pos = 0;
			len = retval;
			goto done;
		}

		list_move(&hcurb->rw_list, &dev->urb_write_list);
	}

	if (dev->urb_write_pos >= sizeof(dev->urb_write_buf)) {
		hcurb = list_entry(
			dev->urb_write_list.next,
			struct ghci_urb, rw_list);

		nr = ghci_build_write_kvec(frags, ARRAY_SIZE(frags), hcurb);
		if (nr < 0) {
			len = nr;
			goto done;
		}

		hcurb_length =
			iov_length((struct iovec *)frags, nr) +
			sizeof(dev->urb_write_buf);

		if (len == 0) {
			ghci_seek_kvec(
				frags, nr, dev->urb_write_pos - sizeof(dev->urb_write_buf),
				&pos, &len);
			if (len > count) {
				len = count;
			}

			spin_unlock_irqrestore(&ghci->lock, *irqflags);
			if (copy_from_user(pos, buf, len)) {
#ifdef DEBUG
				printk(KERN_DEBUG "%s EFAULT dst %p src %p len %d/%u\n",
					__func__, pos, buf, len, count);
#endif
				spin_lock_irqsave(&ghci->lock, *irqflags);
				len = -EFAULT;
				goto done;
			}
			spin_lock_irqsave(&ghci->lock, *irqflags);

			dev->urb_write_pos += len;
		}

		if (dev->urb_write_pos == hcurb_length) {
			ghci_complete_urb(ghci, hcurb, hdr->status);
			dev->urb_write_pos = 0;
		}
	}

done:
	return len;
}

static ssize_t ghci_dev_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *pos)
{
	struct ghci_port *port = file->private_data;
	struct ghci_hcd *ghci = port->ghci;
	struct usb_hcd *hcd = ghci_to_hcd(ghci);
	struct ghci_device *dev;
	unsigned long flags;
	ssize_t retval;

	spin_lock_irqsave(&ghci->lock, flags);

	dev = port->ghci_dev;

	if (unlikely(!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags))) {
		retval = -ESHUTDOWN;
		dev_dbg(hcd->self.controller, "%s hardware unaccessible\n",
			__func__);
		goto done;
	}

	retval = ghci_write_urb(dev, buf, count, &flags);

done:
	spin_unlock_irqrestore(&ghci->lock, flags);

	return retval;
}

static unsigned int ghci_dev_poll(
	struct file *file,
	struct poll_table_struct *wait)
{
	struct ghci_port *port = file->private_data;
	unsigned int mask = 0;
	int status;

	poll_wait(file, &ghci_drv.read_wait, wait);

	status = ghci_dev_readable(port);
	if (status > 0) {
		mask |= POLLIN | POLLRDNORM;
	} else if (status < 0) {
		mask |= POLLERR | POLLHUP;
	}

	return mask;
}

static struct ghci_device *ghci_create_device(void)
{
	struct ghci_device *dev;

	dev = kmalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		return NULL;
	}

	memset(dev, 0, sizeof(*dev));

	INIT_LIST_HEAD(&dev->urb_read_list);
	INIT_LIST_HEAD(&dev->urb_write_list);
	INIT_LIST_HEAD(&dev->urb_cancel_list);
	dev->urb_read_pos = 0;
	dev->urb_write_pos = 0;
	dev->urb_cancel_pos = 0;

	return dev;
}

static void ghci_destroy_device(struct ghci_device *dev)
{
	kfree(dev);
}

/*
 * Caller must have a private hcd lock.
 */
static int ghci_connect_device(struct ghci_port *port, struct ghci_device *dev)
{
	struct ghci_hcd *ghci = port->ghci;
	struct usb_hcd *hcd = ghci_to_hcd(ghci);

	if (!port->claimed) {
		return -EINVAL;
	}
	if ((port->status & USB_PORT_STAT_CONNECTION) != 0) {
		return -EBUSY;
	}

	switch (port->speed) {
	case GHCI_SPEED_HIGH:
		port->status |= USB_PORT_STAT_HIGH_SPEED;
		break;

	case GHCI_SPEED_LOW:
		port->status |= USB_PORT_STAT_LOW_SPEED;
		break;

	case GHCI_SPEED_FULL:
		break;

	/* case GHCI_SPEED_UNKNOWN: */
	default:
		return -EINVAL;
	}

	port->status |= USB_PORT_STAT_CONNECTION;
	port->status |= (USB_PORT_STAT_C_CONNECTION << 16);
	port->ghci_dev = dev;
	dev->port = port;

	spin_unlock(&ghci->lock);
	dev_info(hcd->self.controller,
		"new %s speed device at port#%d hcd#%d\n",
		ghci_get_speed_string(port->speed),
		port->port_index, hcd_to_pdev(hcd)->id);
	usb_hcd_poll_rh_status(hcd);
	spin_lock(&ghci->lock);

	return 0;
}

/*
 * Caller must have a private hcd lock.
 */
static struct ghci_device *ghci_disconnect_device(struct ghci_port *port)
{
	struct ghci_hcd *ghci = port->ghci;
	struct usb_hcd *hcd = ghci_to_hcd(ghci);
	struct ghci_device *dev;

	if ((port->status & USB_PORT_STAT_CONNECTION) == 0) {
		return NULL;
	}

	port->status &= ~(
		USB_PORT_STAT_CONNECTION |
		USB_PORT_STAT_ENABLE |
		USB_PORT_STAT_SUSPEND |
		USB_PORT_STAT_LOW_SPEED |
		USB_PORT_STAT_HIGH_SPEED);
	port->status |= (USB_PORT_STAT_C_CONNECTION << 16);

	dev = port->ghci_dev;
	dev->port = NULL;
	port->ghci_dev = NULL;

	spin_unlock(&ghci->lock);
	dev_info(hcd->self.controller,
		"device disconnected, port#%d hcd#%d\n",
		port->port_index, hcd_to_pdev(hcd)->id);
	usb_hcd_poll_rh_status(hcd);
	spin_lock(&ghci->lock);

	return dev;
}

/*
 * Caller must have a private hcd lock.
 */
static void ghci_shutdown_device(
	struct ghci_hcd *ghci,
	struct ghci_device *dev)
{
	struct ghci_urb *hcurb;
	struct ghci_urb *next;
	struct usb_hcd *hcd = ghci_to_hcd(ghci);

	list_for_each_entry_safe(
		hcurb, next,
		&dev->urb_write_list, rw_list) {

		dev_dbg(hcd->self.controller,
			"forcibly unlinking hcurb #%08x\n",
			hcurb->header.id);
		ghci_complete_urb(ghci, hcurb, -ESHUTDOWN);
	}
	list_for_each_entry_safe(
		hcurb, next,
		&dev->urb_read_list, rw_list) {

		dev_dbg(hcd->self.controller,
			"forcibly unlinking hcurb #%08x\n",
			hcurb->header.id);
		ghci_complete_urb(ghci, hcurb, -ESHUTDOWN);
	}
}

static int ghci_dev_open(struct inode *inode, struct file *file)
{
	struct ghci_port *port;
	struct ghci_hcd *ghci;
	struct usb_hcd *hcd;
	struct ghci_device *dev;
	unsigned long flags;
	int retval;

	port = ghci_get_port_with_dev_index(
		MINOR_TO_DEV_INDEX(ghci_drv.dev, iminor(inode)));
	if (!port) {
		return -ENODEV;
	}
	ghci = port->ghci;
	hcd = ghci_to_hcd(ghci);

	dev_dbg(hcd->self.controller, "%s\n", __func__);

	dev = ghci_create_device();
	if (!dev) {
		dev_err(hcd->self.controller,
			"no available memory for a new ghci device\n");
		return -ENOMEM;
	}

	spin_lock_irqsave(&ghci->lock, flags);

	retval = ghci_connect_device(port, dev);
	if (retval < 0) {
		dev_err(hcd->self.controller,
			"failed to connect a device, port#%d hcd#%d err %d\n",
			port->port_index, hcd_to_pdev(hcd)->id, retval);
		goto done;
	}

	file->private_data = port;

	/* this call never fails */
	retval = nonseekable_open(inode, file);

done:
	spin_unlock_irqrestore(&ghci->lock, flags);

	if (retval < 0) {
		ghci_destroy_device(dev);
	}

	return retval;
}

static int ghci_dev_release(struct inode *inode, struct file *file)
{
	struct ghci_port *port;
	struct ghci_hcd *ghci;
	struct usb_hcd *hcd;
	struct ghci_device *dev;
	unsigned long flags;
	int retval;

	port = ghci_get_port_with_dev_index(
		MINOR_TO_DEV_INDEX(ghci_drv.dev, iminor(inode)));
	if (!port) {
		return -ENODEV;
	}
	ghci = port->ghci;
	hcd = ghci_to_hcd(ghci);

	dev_dbg(hcd->self.controller, "%s\n", __func__);

	spin_lock_irqsave(&ghci->lock, flags);

	dev = ghci_disconnect_device(port);
	if (!dev) {
		retval = -ENODEV;
		dev_err(hcd->self.controller,
			"cannot disconnect a device, port#%d hcd#%d\n",
			port->port_index, hcd_to_pdev(hcd)->id);
		goto done;
	}

	/*
	 * shutdown urbs that are still queued so drivers' usb_kill_urb()
	 * calls will return
	 */
	ghci_shutdown_device(ghci, dev);

	ghci_destroy_device(dev);
	retval = 0;

done:
	spin_unlock_irqrestore(&ghci->lock, flags);

	return retval;
}

static const struct file_operations ghci_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = ghci_dev_read,
	.write = ghci_dev_write,
	.poll = ghci_dev_poll,
	.open = ghci_dev_open,
	.release = ghci_dev_release,
};

static int ghci_start(struct usb_hcd *hcd)
{
	struct ghci_hcd *ghci = hcd_to_ghci(hcd);

	dev_dbg(hcd->self.controller, "%s\n", __func__);

	spin_lock_init(&ghci->lock);

	hcd->state = HC_STATE_RUNNING;

	return 0;
}

static void ghci_stop(struct usb_hcd *hcd)
{
	dev_dbg(hcd->self.controller, "%s\n", __func__);
}

static int ghci_get_frame_number(struct usb_hcd *hcd)
{
	struct timeval tv;

	dev_dbg(hcd->self.controller, "%s\n", __func__);

	do_gettimeofday(&tv);

	return tv.tv_usec / 1000;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
static int ghci_urb_enqueue(
	struct usb_hcd *hcd,
	struct usb_host_endpoint *ep,
	struct urb *urb,
	gfp_t mem_flags)
#else
static int ghci_urb_enqueue(
	struct usb_hcd *hcd,
	struct urb *urb,
	gfp_t mem_flags)
#endif
{
	struct ghci_hcd *ghci = hcd_to_ghci(hcd);
	struct ghci_urb *hcurb;
	int retval;

	if (unlikely(!test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags))) {
		dev_dbg(hcd->self.controller,
			"cannot enqueue a urb, hardware unaccessible\n");
		return -ESHUTDOWN;
	}

	retval = ghci_internal_submit_urb(ghci, urb);
	if (retval != -EINPROGRESS) {
		if (retval < 0) {
			dev_err(hcd->self.controller,
				"an internal error occurred while "
				"submitting a urb, err %d\n",
				retval);
		}
		return retval;
	}

	hcurb = ghci_create_xfer_urb(ghci, urb, mem_flags);
	if (!hcurb) {
		dev_err(hcd->self.controller,
			"no available memory for a hcurb\n");
		return -ENOMEM;
	}

	retval = ghci_submit_urb(ghci, hcurb);
	if (retval < 0) {
		dev_err(hcd->self.controller,
			"an error occurred while submitting a hcurb, err %d\n",
			retval);
		ghci_free_urb(hcurb);
	}

	return retval;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
static int ghci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb)
#else
static int ghci_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
#endif
{
	struct ghci_hcd *ghci = hcd_to_ghci(hcd);
	struct ghci_device *dev;
	struct ghci_urb *hcurb;
	unsigned long flags;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
	int status = urb->status;
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 23)
	int retval;
#endif
	int unlink = 0;

	spin_lock_irqsave(&ghci->lock, flags);

	dev = ghci_get_device_with_address(ghci, usb_pipedevice(urb->pipe));
	if (!dev) {
		spin_unlock_irqrestore(&ghci->lock, flags);
		return -ENODEV;
	}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 23)
	retval = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (retval < 0) {
		dev_err(hcd->self.controller,
			"check unlink failure, err %d\n",
			retval);
		spin_unlock_irqrestore(&ghci->lock, flags);
		return retval;
	}
#endif

	list_for_each_entry(hcurb, &dev->urb_read_list, rw_list) {
#if 1 /* DEBUG */
		dev_dbg(hcd->self.controller,
			"rd_ent(id: 0x%04x)\n",
			hcurb->header.id);
#endif
		if (hcurb->urb == urb) {
			unlink = 1;
			break;
		}
	}

	if (unlink && ((dev->urb_read_pos == 0) ||
		(dev->urb_read_list.next != &hcurb->rw_list))) {

		dev_dbg(hcd->self.controller,
			"dequeue NOW id 0x%04x status %d\n",
			hcurb->header.id, status);

		ghci_complete_urb(ghci, hcurb, status);
	} else {
		/* reuse the hcurb to queue a cancel request */
		hcurb = urb->hcpriv;
		hcurb->header.status = status;
		list_add_tail(&hcurb->cancel_list, &dev->urb_cancel_list);

		dev_dbg(hcd->self.controller,
			"dequeue POST id 0x%04x status %d\n",
			hcurb->header.id, status);

		wake_up_interruptible(&ghci_drv.read_wait);
	}

	spin_unlock_irqrestore(&ghci->lock, flags);

	return 0;
}

static int ghci_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb,
			   gfp_t mem_flags)
{
	/* This is dummy function, GHCI do not use DMA. */
	return 0;
}

static void ghci_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{
	/* This is dummy function, GHCI do not use DMA. */
	return;
}

/* See USB 2.0 spec. Figure 11-22 */
static int ghci_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct ghci_hcd *ghci = hcd_to_ghci(hcd);
	struct ghci_platform_hcd *priv = hcd->self.controller->platform_data;
	unsigned long flags;
	int changed = 0;
	int i;
	int len = 1 + priv->nr_ports / 8;
	int pos;

	dev_dbg(hcd->self.controller, "%s\n", __func__);

	memset(buf, 0, len);

	spin_lock_irqsave(&ghci->lock, flags);

	for (i = 0; i < priv->nr_ports; i++) {
		if ((ghci->port[i].status & PORT_C_MASK) == 0) {
			continue;
		}

		pos = (i + 1) / 8;
		buf[pos] |= 1 << ((i + 1) - pos * 8);
		changed = 1;
	}

	if (changed && HC_IS_SUSPENDED(hcd->state)) {
		usb_hcd_resume_root_hub(hcd);
	}

	spin_unlock_irqrestore(&ghci->lock, flags);

	return (changed ? len : 0);
}

static void ghci_hub_descriptor(
	struct usb_hcd *hcd,
	struct usb_hub_descriptor *desc)
{
	struct ghci_platform_hcd *priv = hcd->self.controller->platform_data;
	int len = 1 + priv->nr_ports / 8;
	u8 *device_removable;

	memset(desc, 0, sizeof(*desc));

	desc->bDescLength = 7 + 2 * len;
	desc->bDescriptorType = 0x29;
	desc->bNbrPorts = priv->nr_ports;
	desc->wHubCharacteristics = __constant_cpu_to_le16(0x0009);
	desc->bPwrOn2PwrGood = 0;
	desc->bHubContrCurrent = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 39)
	device_removable = desc->bitmap;
#else
	device_removable = desc->u.hs.DeviceRemovable;
#endif

	/* DeviceRemovable */
	memset(&device_removable[0], 0, len);

	/* PortPwrCtrlMask */
	memset(&device_removable[len], 0xff, len);
}

static int ghci_hub_control(
	struct usb_hcd *hcd,
	u16 typeReq,
	u16 wValue,
	u16 wIndex,
	char *buf,
	u16 wLength)
{
	struct ghci_hcd *ghci = hcd_to_ghci(hcd);
	unsigned long flags;
	int i;
	int retval = 0;

	switch (typeReq) {
	case ClearHubFeature:
		dev_dbg(hcd->self.controller,
			"ClearHubFeature value 0x%04x\n",
			wValue);
		break;

	case ClearPortFeature:
		i = (wIndex & 0x00ff) - 1; /* port index */

		dev_dbg(hcd->self.controller,
			"ClearPortFeature port %d value 0x%04x\n",
			i + 1, wValue);

		spin_lock_irqsave(&ghci->lock, flags);
		ghci->port[i].status &= ~(1 << wValue);
		spin_unlock_irqrestore(&ghci->lock, flags);

		break;

	case GetHubDescriptor:
		dev_dbg(hcd->self.controller, "GetHubDescriptor\n");
		ghci_hub_descriptor(hcd, (struct usb_hub_descriptor *)buf);
		break;

	case GetHubStatus:
		dev_dbg(hcd->self.controller, "GetHubStatus\n");
		*(__le32 *)buf = __constant_cpu_to_le32(0);
		break;

	case GetPortStatus:
		i = wIndex - 1; /* port index */

		dev_dbg(hcd->self.controller,
			"GetPortStatus port %u\n", wIndex);

		spin_lock_irqsave(&ghci->lock, flags);

		/* handle status change for port reset */
		if ((ghci->port[i].status & USB_PORT_STAT_RESET) != 0) {
			ghci->port[i].status |=
				(USB_PORT_STAT_C_RESET << 16);
			ghci->port[i].status &= ~USB_PORT_STAT_RESET;

			if (ghci->port[i].ghci_dev) {
				ghci_reset_address(ghci->port[i].ghci_dev);
				ghci->port[i].status |= USB_PORT_STAT_ENABLE;
			}
		}

		((__le16 *)buf)[0] = cpu_to_le16(ghci->port[i].status);
		((__le16 *)buf)[1] = cpu_to_le16(ghci->port[i].status >> 16);

		spin_unlock_irqrestore(&ghci->lock, flags);

		break;

	case SetHubFeature:
		dev_dbg(hcd->self.controller,
			"SetHubFeature value 0x%04x\n",
			wValue);
		retval = -EPIPE;
		break;

	case SetPortFeature:
		i = (wIndex & 0x00ff) - 1; /* port index */

		dev_dbg(hcd->self.controller,
			"SetPortFeature port %d value 0x%04x\n",
			i + 1, wValue);

		spin_lock_irqsave(&ghci->lock, flags);
		ghci->port[i].status |= (1 << wValue);
		spin_unlock_irqrestore(&ghci->lock, flags);

		break;

	default:
		dev_dbg(hcd->self.controller,
			"%s type 0x%04x value 0x%04x index 0x%04x length %u\n",
			__func__, typeReq, wValue, wIndex, wLength);
		retval = -EPIPE;
	}

	return retval;
}

static int ghci_bus_suspend(struct usb_hcd *hcd)
{
	struct ghci_hcd *ghci = hcd_to_ghci(hcd);
	unsigned long flags;

	dev_dbg(hcd->self.controller, "%s\n", __func__);

	spin_lock_irqsave(&ghci->lock, flags);
	hcd->state = HC_STATE_SUSPENDED;
	spin_unlock_irqrestore(&ghci->lock, flags);

	return 0;
}

static int ghci_bus_resume(struct usb_hcd *hcd)
{
	struct ghci_hcd *ghci = hcd_to_ghci(hcd);
	unsigned long flags;

	dev_dbg(hcd->self.controller, "%s\n", __func__);

	spin_lock_irqsave(&ghci->lock, flags);
	hcd->state = HC_STATE_RUNNING;
	spin_unlock_irqrestore(&ghci->lock, flags);

	return 0;
}

static const struct hc_driver ghci_hc_driver = {
	.description = driver_name,
	.product_desc = driver_desc,
	.hcd_priv_size = sizeof(struct ghci_hcd),

	.flags = HCD_USB2,

	.start = ghci_start,
	.stop = ghci_stop,

	.get_frame_number = ghci_get_frame_number,

	.urb_enqueue = ghci_urb_enqueue,
	.urb_dequeue = ghci_urb_dequeue,

	.map_urb_for_dma = ghci_map_urb_for_dma,
	.unmap_urb_for_dma = ghci_unmap_urb_for_dma,

	.hub_status_data = ghci_hub_status_data,
	.hub_control = ghci_hub_control,
	.bus_suspend = ghci_bus_suspend,
	.bus_resume = ghci_bus_resume,
};

static int ghci_hcd_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	int retval;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	hcd = usb_create_hcd(&ghci_hc_driver, &pdev->dev, hcd_name(pdev));
	if (!hcd) {
		dev_err(&pdev->dev, "no available memory for a new hcd\n");
		return -ENOMEM;
	}

	hcd->uses_new_polling = 1;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 25)
	hcd->has_tt = 1;
#endif
	hcd->power_budget = POWER_BUDGET;

	retval = usb_add_hcd(hcd, 0, 0);
	if (retval < 0) {
		dev_err(&pdev->dev, "cannot add a hcd, err %d\n", retval);
		usb_put_hcd(hcd);
		return retval;
	}

	return 0;
}

static int ghci_hcd_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	return 0;
}

static int ghci_hcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	return 0;
}

static int ghci_hcd_resume(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	return 0;
}

static struct platform_driver ghci_platform_driver = {
	.probe = ghci_hcd_probe,
	.remove = ghci_hcd_remove,
	.suspend = ghci_hcd_suspend,
	.resume = ghci_hcd_resume,
	.driver = {
		.name = driver_name,
		.owner = THIS_MODULE,
	},
};

static int ghci_ioctl_claim_port(struct file *file, unsigned int speed)
{
	struct usb_hcd *hcd;
	struct ghci_platform_hcd *priv;
	struct ghci_hcd *ghci;
	struct ghci_port *port;
	unsigned long flags;
	int i;
	int j;
	int retval = -EBUSY;

	if ((speed < GHCI_SPEED_LOW) || (speed > GHCI_SPEED_HIGH)) {
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(ghci_drv.pdev); i++) {
		hcd = platform_get_drvdata(ghci_drv.pdev[i]);
		if (!hcd) {
			continue;
		}

		priv = hcd->self.controller->platform_data;
		ghci = hcd_to_ghci(hcd);

		spin_lock_irqsave(&ghci->lock, flags);

		for (j = 0; j < priv->nr_ports; j++) {
			port = &ghci->port[j];

			if (!port->claimed && !port->ghci_dev) {
				port->claimed = file;
				port->speed = speed;

				spin_unlock_irqrestore(&ghci->lock, flags);

				dev_dbg(hcd->self.controller,
					"port#%d (dev_index %d) claimed\n",
					port->port_index, port->dev_index);

				retval = port->dev_index;
				goto done;
			}
		}

		spin_unlock_irqrestore(&ghci->lock, flags);
	}

done:
	return retval;
}

static int ghci_ioctl_release_port(struct file *file, unsigned int i)
{
	struct ghci_port *port;
	struct ghci_hcd *ghci;
	unsigned long flags;
	int retval;

	port = ghci_get_port_with_dev_index(i);
	if (!port) {
		return -ENODEV;
	}
	ghci = port->ghci;

	spin_lock_irqsave(&ghci->lock, flags);

	if (!port->claimed) {
		retval = -EINVAL;
		goto done;
	}
	if (port->claimed != file) {
		retval = -EPERM;
		goto done;
	}

	port->claimed = NULL;
	/* keep speed, the device might be still operating */
	/* port->speed = GHCI_SPEED_UNKNOWN; */

	dev_dbg(ghci_to_hcd(ghci)->self.controller,
		"port#%d (dev_index %d) released\n",
		port->port_index, port->dev_index);
	retval = 0;

done:
	spin_unlock_irqrestore(&ghci->lock, flags);

	return retval;
}

static long ghci_ctl_ioctl(
	struct file *file,
	unsigned int cmd,
	unsigned long arg)
{
	int retval;

	switch (cmd) {
	case GHCI_CLAIM_PORT:
		dev_dbg(ghci_drv.ctl_device, "ioctl GHCI_CLAIM_PORT\n");
		retval = ghci_ioctl_claim_port(file, (unsigned int)arg);
		break;

	case GHCI_RELEASE_PORT:
		dev_dbg(ghci_drv.ctl_device, "ioctl GHCI_RELEASE_PORT\n");
		retval = ghci_ioctl_release_port(file, (unsigned int)arg);
		break;

	default:
		retval = -EINVAL;
	}

	return retval;
}

static int ghci_ctl_release(struct inode *inode, struct file *file)
{
	unsigned int i;

	dev_dbg(ghci_drv.ctl_device, "%s\n", __func__);

	for (i = 0; i < NUM_DEV_MINORS; i++) {
		ghci_ioctl_release_port(file, i);
	}

	return 0;
}

static const struct file_operations ghci_ctl_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = ghci_ctl_ioctl,
	.open = nonseekable_open,
	.release = ghci_ctl_release,
};

static int __init ghci_register_platform_hcd(
	int id, unsigned int nr_ports,
	struct platform_device **new_pdev)
{
	struct platform_device *pdev;
	struct ghci_platform_hcd priv;
	int retval;

	if (nr_ports > USB_MAXCHILDREN) {
		return -EINVAL;
	}

	pdev = platform_device_alloc(driver_name, id);
	if (!pdev) {
		return -ENOMEM;
	}

	priv.nr_ports = nr_ports;

	retval = platform_device_add_data(pdev, &priv, sizeof(priv));
	if (retval < 0) {
		goto fail;
	}

	retval = platform_device_add(pdev);
	if (retval < 0) {
		goto fail;
	}

	*new_pdev = pdev;

	return 0;

fail:
	platform_device_put(pdev);

	return retval;
}

static void ghci_unregister_platform_hcd(struct platform_device *pdev)
{
	platform_device_unregister(pdev);
}

static void ghci_hcd_unregister(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ghci_drv.pdev); i++) {
		ghci_unregister_platform_hcd(ghci_drv.pdev[i]);
		ghci_drv.pdev[i] = NULL;
	}
}

static int __init ghci_hcd_register(void)
{
	struct platform_device *pdev = NULL;
	unsigned int nr_ports;
	unsigned int count = 0;
	int i;
	int retval;

	for (i = 0; i < ARRAY_SIZE(ghci_drv.pdev); i++) {
		nr_ports = NUM_DEV_MINORS - count;
		if (nr_ports > USB_MAXCHILDREN) {
			nr_ports = USB_MAXCHILDREN;
		}

		retval = ghci_register_platform_hcd(i, nr_ports, &pdev);
		if (retval < 0) {
			ghci_hcd_unregister();
			return retval;
		}

		ghci_drv.pdev[i] = pdev;

		count += nr_ports;
	}

	return 0;
}

static int __init ghci_cdev_register(dev_t *new_devt)
{
	dev_t ctl_dev;
	dev_t dev_dev;
	int retval;

	if (ghci_major == 0) {
		retval = alloc_chrdev_region(&ghci_drv.dev, 0, NUM_MINORS, driver_name);
	} else {
		ghci_drv.dev = MKDEV(ghci_major, 0);
		retval = register_chrdev_region(ghci_drv.dev, NUM_MINORS, driver_name);
	}

	if (retval < 0) {
		printk(KERN_ERR "failed to allocate chrdev region\n");
		return retval;
	}

	ctl_dev = ghci_drv.dev;
	dev_dev = MKDEV_DEV(ghci_drv.dev, 0);

	cdev_init(&ghci_drv.ctl_cdev, &ghci_ctl_fops);
	ghci_drv.ctl_cdev.owner = THIS_MODULE;
	cdev_init(&ghci_drv.dev_cdev, &ghci_dev_fops);
	ghci_drv.dev_cdev.owner = THIS_MODULE;

	retval = cdev_add(&ghci_drv.ctl_cdev, ctl_dev, NUM_CTL_MINORS);
	if (retval < 0) {
		printk(KERN_ERR "cannot add a ghci_ctl character device\n");
		goto err_ctl_cdev;
	}
	retval = cdev_add(&ghci_drv.dev_cdev, dev_dev, NUM_DEV_MINORS);
	if (retval < 0) {
		printk(KERN_ERR "cannot add ghci character devices\n");
		goto err_dev_cdev;
	}

	*new_devt = ghci_drv.dev;

	return 0;

err_dev_cdev:
	cdev_del(&ghci_drv.ctl_cdev);
err_ctl_cdev:
	unregister_chrdev_region(ghci_drv.dev, NUM_MINORS);

	return retval;
}

static void ghci_cdev_unregister(void)
{
	cdev_del(&ghci_drv.dev_cdev);
	cdev_del(&ghci_drv.ctl_cdev);
	unregister_chrdev_region(ghci_drv.dev, NUM_MINORS);
}

static void ghci_destroy_port_devices(struct usb_hcd *hcd, struct class *cls)
{
	struct ghci_hcd *ghci = hcd_to_ghci(hcd);
	struct ghci_platform_hcd *priv = hcd->self.controller->platform_data;
	struct ghci_port *port;
	int i;

	for (i = 0; i < priv->nr_ports; i++) {
		port = &ghci->port[i];
		if (!port->dev || IS_ERR(port->dev)) {
			break;
		}

#ifdef DEBUG
		printk(KERN_DEBUG "destroying dev#%d of hcd#%d\n",
			i, hcd_to_pdev(hcd)->id);
#endif

		device_destroy(cls, port->dev->devt);
		port->dev = NULL;
	}
}

static int __init ghci_init_port_devices(
	struct usb_hcd *hcd, struct class *cls, dev_t devt)
{
	struct ghci_hcd *ghci = hcd_to_ghci(hcd);
	struct ghci_platform_hcd *priv = hcd->self.controller->platform_data;
	struct platform_device *pdev = hcd_to_pdev(hcd);
	int dev_index_start = pdev->id * ARRAY_SIZE(ghci->port);
	struct ghci_port *port;
	int dev_index;
	int i;
	int retval = 0;

	for (i = 0; i < priv->nr_ports; i++) {
		port = &ghci->port[i];
		dev_index = i + dev_index_start;

#ifdef DEBUG
		printk(KERN_DEBUG "creating dev#%d of hcd#%d (%s%d)\n",
			i, pdev->id, device_name, dev_index);
#endif

		port->dev = device_create(
			cls, NULL, MKDEV_DEV(devt, dev_index),
			"%s%d", device_name, dev_index);
		if (IS_ERR(port->dev)) {
			retval = PTR_ERR(port->dev);
			printk(KERN_ERR "failed to create %s%d\n",
				device_name, dev_index);
			ghci_destroy_port_devices(hcd, cls);
			break;
		}

		port->ghci = ghci;
		port->dev_index = dev_index;
		port->port_index = i;
	}

	return retval;
}

static void ghci_destroy_devices(struct class *cls)
{
	struct usb_hcd *hcd;
	int i;

	for (i = 0; i < ARRAY_SIZE(ghci_drv.pdev); i++) {
		hcd = platform_get_drvdata(ghci_drv.pdev[i]);
		if (!hcd) {
			printk(KERN_WARNING "no hcd #%d\n", i);
			continue;
		}

		ghci_destroy_port_devices(hcd, cls);
	}
}

static int __init ghci_init_devices(struct class *cls, dev_t devt)
{
	struct usb_hcd *hcd;
	int i;
	int retval = 0;

	for (i = 0; i < ARRAY_SIZE(ghci_drv.pdev); i++) {
		hcd = platform_get_drvdata(ghci_drv.pdev[i]);
		if (!hcd) {
			printk(KERN_WARNING "usb_add_hcd() "
				"was failed, hcd#%d\n", i);
			continue;
		}

		retval = ghci_init_port_devices(hcd, cls, devt);
		if (retval < 0) {
			ghci_destroy_devices(cls);
			break;
		}
	}

	return retval;
}

static int __init ghci_device_register(dev_t devt)
{
	struct class *cls;
	struct device *ctl_dev;
	int retval;

	cls = class_create(THIS_MODULE, driver_name);
	if (IS_ERR(cls)) {
		printk(KERN_ERR "failed to create a class %s\n", driver_name);
		return PTR_ERR(cls);
	}

	ctl_dev = device_create(cls, NULL, devt, "ghci_ctl");
	if (IS_ERR(ctl_dev)) {
		printk(KERN_ERR "failed to create a ghci_ctl device\n");
		retval = PTR_ERR(ctl_dev);
		goto err_ctl_dev;
	}

	retval = ghci_init_devices(cls, devt);
	if (retval < 0) {
		goto err_init_devs;
	}

	ghci_drv.hcd_class = cls;
	ghci_drv.ctl_device = ctl_dev;

	return 0;

err_init_devs:
	device_destroy(cls, ctl_dev->devt);
err_ctl_dev:
	class_destroy(cls);

	return retval;
}

static void ghci_device_unregister(void)
{
	struct class *cls = ghci_drv.hcd_class;
	struct device *ctl_dev = ghci_drv.ctl_device;

	ghci_destroy_devices(cls);
	device_destroy(cls, ctl_dev->devt);
	class_destroy(cls);
}

static int __init ghci_init(void)
{
	dev_t devt = 0;
	int retval;

	printk(KERN_INFO "%s driver version %s\n", driver_desc, DRIVER_VERSION);

	init_waitqueue_head(&ghci_drv.read_wait);

	retval = platform_driver_register(&ghci_platform_driver);
	if (retval < 0) {
		printk(KERN_ERR "failed to register "
			"a platform driver %s, err %d\n",
			ghci_platform_driver.driver.name, retval);
		return retval;
	}

	retval = ghci_hcd_register();
	if (retval < 0) {
		printk(KERN_ERR "failed to register "
			"ghci hcds, err %d\n", retval);
		goto err_hcd;
	}

	retval = ghci_cdev_register(&devt);
	if (retval < 0) {
		printk(KERN_ERR "failed to register "
			"ghci character devices, err %d\n",
			retval);
		goto err_cdev;
	}

	retval = ghci_device_register(devt);
	if (retval < 0) {
		printk(KERN_ERR "failed to register "
			"ghci devices, err %d\n", retval);
		goto err_dev;
	}

	return 0;

err_dev:
	ghci_cdev_unregister();
err_cdev:
	ghci_hcd_unregister();
err_hcd:
	platform_driver_unregister(&ghci_platform_driver);

	return retval;
}

static void __exit ghci_exit(void)
{
	ghci_device_unregister();
	ghci_cdev_unregister();
	ghci_hcd_unregister();
	platform_driver_unregister(&ghci_platform_driver);
}

module_init(ghci_init);
module_exit(ghci_exit);
