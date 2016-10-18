#ifndef __PANIC_REASON_PON_H__
#define __PANIC_REASON_PON_H__

#include <linux/panic_reason.h>

#define PANIC_PON_REG_OFFSET 0x8E //reg is QPNP_PON_XVDD_RB_SPARE(x)
#define PANIC_PON_MASK (0x7F)
#define PANIC_PON_SHIFT (0)

#define ALIVE_REG_OFFSET 0x8D //reg is QPNP_PON_SOFT_RB_SPARE(x)
#define ALIVE_MASK (0x80)
#define ALIVE_SHIFT (7)

#endif
