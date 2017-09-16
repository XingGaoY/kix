CONFIG_MODULE_SIG=n
ifneq ($(KERNELRELEASE),)
	obj-m			:= ixgbe.o

ixgbe-objs	:= ixgbe_main.o
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD	  := $(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

endif
