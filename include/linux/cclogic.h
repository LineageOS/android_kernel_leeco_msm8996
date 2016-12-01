
#ifndef __LINUX_CCLOGIC_H
#define __LINUX_CCLOGIC_H

typedef enum {
	TYPEC_PORT_NONE = 0,
	TYPEC_PORT_UFP,
	TYPEC_PORT_DFP,
	TYPEC_PORT_AUDIO,
	TYPEC_PORT_DEBUG,
	TYPEC_PORT_MAX,
} typec_port_state;

typec_port_state cclogic_get_port_state(void);

#endif

