KDIR ?= /lib/modules/$(shell uname -r)/build
BUILD_DIR:=$(shell pwd)
# KBUILD_OPTIONS += CROSS_COMPILE=arm-linux- ARCH=arm

include $(KDIR)/.config
obj-m += spi-ft232h.o

all: modules
modules:
	$(MAKE) -C $(KDIR) M=$$PWD modules $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

