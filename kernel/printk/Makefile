obj-y	+= printk.o
obj-$(CONFIG_A11Y_BRAILLE_CONSOLE)	+= braille.o

obj-$(CONFIG_LAST_KMSG_LETV) += last_kmsg.o qcom_crc.o
ifeq ($(CONFIG_QPNP_POWER_ON),y)
	obj-$(CONFIG_LAST_KMSG_LETV) += panic_reason_pon.o
else
	obj-$(CONFIG_LAST_KMSG_LETV) += panic_reason.o
endif
