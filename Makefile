CONFIG_MODULE_SIG=n
ifneq ($(KERNELRELEASE),)
	subdir-ccflags-y = -I$(PWD)/inc

	ixgbe_objs		:= ixgbe/ixgbe_main.o ixgbe/ixgbe_82599.o ixgbe/ixgbe_ethdev.o 	\
					   ixgbe/ixgbe_phy.o ixgbe/ixgbe_common.o ixgbe/ixgbe_api.o 	\
					   ixgbe/ixgbe_mbx.o ixgbe/ixgbe_dcb.o ixgbe/ixgbe_dcb_82599.o

	obj-m 			+=  kix.o
	kix-objs		:=	init.o $(ixgbe_objs)
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD	 := 	$(shell pwd)
	DIRS :=		ixgbe
	SRC  :=		$(foreach n, $(DIRS), $(wildcard $(n)/*.c))
	OBJS :=		$(patsubst %.c, %.o, $(SRC))

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

endif

.PHONY: clean
clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean