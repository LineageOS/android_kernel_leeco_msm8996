ifneq ($(KERNELRELEASE),)
# kbuild part of Makefile
include Kbuild

else
# normal Makefile
KDIR ?= /lib/modules/`uname -r`/build

TARGETS := modules modules_install clean

.PHONY: all $(TARGETS)

all: modules

$(TARGETS):
	$(MAKE) -C $(KDIR) M=$$PWD $@

endif
