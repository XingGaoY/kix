CONFIG_MODULE_SIG=n
ifneq ($(KERNELRELEASE),)
subdir-ccflags-y = -I$(PWD)/inc

	obj-m			:= ixgbe.o

ixgbe-objs	:= ixgbe_main.o ixgbe_82599.o ixgbe_phy.o\
			   ixgbe_common.o ixgbe_api.o ixgbe_mbx.o
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD	  := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

endif

.PHONY: clean
clean:
	-rm -f *.o *.ko *.mod.c *.symvers *.order
