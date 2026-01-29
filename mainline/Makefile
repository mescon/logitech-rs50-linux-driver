KVERSION := $(shell uname -r)
KDIR := /lib/modules/$(KVERSION)/build

obj-m := hid-logitech-hidpp.o

# Auto-detect kernel compiler and match it if available
# User can override with: make CC=gcc
ifeq ($(origin CC),default)
  KERNEL_CLANG := $(shell grep -s "^CONFIG_CC_IS_CLANG=y" $(KDIR)/.config)
  CLANG_AVAIL  := $(shell command -v clang >/dev/null 2>&1 && echo 1)
  ifeq ($(KERNEL_CLANG)$(CLANG_AVAIL),CONFIG_CC_IS_CLANG=y1)
    CC := clang
    LLVM := 1
  endif
endif

ifdef LLVM
  MAKE_OPTS := CC=$(CC) LLVM=$(LLVM)
endif

all:
	$(MAKE) -C $(KDIR) M=$(PWD) $(MAKE_OPTS) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: all
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

load: all
	-sudo rmmod hid-logitech-hidpp 2>/dev/null || true
	sudo insmod ./hid-logitech-hidpp.ko

unload:
	-sudo rmmod hid-logitech-hidpp 2>/dev/null || true
