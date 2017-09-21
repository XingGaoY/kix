#include <linux/init.h>
#include <linux/module.h>
#include <ixgbe/ixgbe.h>

MODULE_LICENSE("GPL");

static int __init kix_init_module(void){
	int retval = 0;

	retval = ixgbe_init();

	return retval;
}

static void __exit kix_exit_module(void){
	ixgbe_exit();
}

module_init(kix_init_module);
module_exit(kix_exit_module);