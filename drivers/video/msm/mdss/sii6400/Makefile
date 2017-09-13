TARGET = sii6400drv

# default HAL platform; can be overridden on the command line
PLATFORM := bbxm

# default version number
ifeq ($(DRIVER_VERSION),)
DRIVER_VERSION_MAJOR = 4.1.0
DRIVER_VERSION_BUILD = $(shell svnversion)
DRIVER_VERSION = "$(DRIVER_VERSION_MAJOR).$(DRIVER_VERSION_BUILD)"
endif

CONFIGDATA_FILE = /persist/sii6400.dat

# build option: compile flags for kernel debugging (kgdb)
ifeq ($(KDEBUG),y)
DBG_FLAGS = -O -g  # "-O" is needed to expand inlines
else
DBG_FLAGS = -O2
endif

# build option: enable sysfs unit test interface
ifeq ($(SYSDEBUG),y)
OPT_FLAGS = -DSII6400_DEBUG
else
OPT_FLAGS =
endif

#build option: run sparse static analysis
ifeq ($(SPARSE),y)
SPARSE_OPT = C=2
else
SPARSE_OPT =
endif

ccflags-y := -Wall -DDEBUG -D__CHECK_ENDIAN__ $(DBG_FLAGS) $(OPT_FLAGS)

ifeq ($(WEXTRA),y)
# all -Wextra flags except -Wunused-parameter
# (-Wunused-parameter generates many warnings in Linux header files)
ccflags-y += -Wclobbered \
	     -Wempty-body \
	     -Wignored-qualifiers \
	     -Wmissing-field-initializers \
	     -Wmissing-parameter-type \
	     -Wold-style-declaration \
	     -Woverride-init \
	     -Wsign-compare \
	     -Wtype-limits \
	     -Wuninitialized \
	     -Wunused-but-set-parameter
else
ccflags-y += -Werror
endif

# build option: combined firmware
ifneq ($(COMBINED_FIRMWARE),n)
ccflags-y += -DSII6400_COMBINED_FIRMWARE
endif

ifeq ($(PLATFORM),serial)
ccflags-y += -DSII6400_HAL_SERIAL
endif

# set driver version string
ccflags-y += -DSII6400_DRIVER_VERSION=\"$(DRIVER_VERSION)\"

# set configparam data filepath
ccflags-y += -DSII6400_CONFIGDATA_FILE=\"$(CONFIGDATA_FILE)\"

ifeq ($(KDEBUG),y)
ccflags-y += -DDEBUG_MUTEXES \
	     -DDEBUG_SPINLOCK \
	     -DDEBUG_ATOMIC_SLEEP \
	     -DDEBUG_SLAB \
	     -DDEBUG_OBJECTS \
	     -DFRAME_WARN \
	     -DENABLE_MUST_CHECK \
	     -DENABLE_WARN_DEPRECATED
endif

ccflags-y += -DCONFIG_LIMIT_ON_HDMI
ccflags-y += -DENABLE_MHL_UEVENT
ccflags-y += -DOLD_KEYMAP_TABLE


ifneq ($(KERNELRELEASE),)
# call from kernel build system

$(TARGET)-objs += sii6400.o
$(TARGET)-objs += sii6400_wihd.o
$(TARGET)-objs += sii6400_mhl.o
ifeq ($(SYSDEBUG),y)
$(TARGET)-objs += sii6400_dbg.o
endif
$(TARGET)-objs += isr.o
$(TARGET)-objs += osal_linux_kernel.o
$(TARGET)-objs += hal_$(PLATFORM).o
$(TARGET)-objs += device.o
$(TARGET)-objs += struct_bitfield.o
$(TARGET)-objs += host_msg.o
$(TARGET)-objs += state_machine.o
$(TARGET)-objs += wihd_sm.o
$(TARGET)-objs += mhl_sm.o
$(TARGET)-objs += diag_sm.o
$(TARGET)-objs += rcp_inputdev.o
$(TARGET)-objs += remote_fw_update.o

obj-y := $(TARGET).o

else

PWD := $(shell pwd)
KERNELDIR ?= /lib/modules/$(shell uname -r)/build

default:
ifeq ($(CONFIGDATA_FILE),)
	$(error "CONFIGDATA_FILE must be provided!")
endif
	$(MAKE) -C $(KERNELDIR) SUBDIRS=$(PWD) DRIVER_VERSION=$(DRIVER_VERSION) modules $(SPARSE_OPT)

endif

.PHONY: clean depend ins insdbg rem

clean:
	rm -rf *.o *.ko *~ core .depend *.mod.c .*.cmd .tmp_versions .*.o.d *.order *.symvers

depend .depend dep:
	$(CC) $(CFLAGS) -M *.c > .depend

ins: default rem
	insmod $(TARGET).ko

insdbg: default rem
	insmod $(TARGET).ko debug_level=2

rem:
	@if [ -n "`lsmod | grep -s $(TARGET)`" ]; then rmmod $(TARGET); echo "rmmod $(TARGET)"; fi

ifeq (.depend,$(wildcard .depend))
include .depend
endif
