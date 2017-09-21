#include <linux/init.h>
#include <linux/module.h>
#include <cdev.h>

#include <ixgbe/ixgbe.h>

MODULE_LICENSE("GPL");

#define KIX_DEV_MAJOR	0
#define KIX_NR_DEVS		1

int kix_ioctl(struct inode *inode, struct file *filep, 
	unsigned int cmd, unsigned long arg){
	void *addr;
	if(cmd == 0){
		addr = (void*)arg;
		return 0;
	}
	return -1;
}

static struct file_operations kix_ops = {
	.ioctl		= kix_ioctl,
}

static int __init kix_init_module(void){
	int retval = 0;

	retval = register_chrdev(0, "kix", &kix_ops);
	if(retval < 0){
		printk(KERN_ERR "Unable to register kix dev");
	}
	retval = ixgbe_init();

	return retval;
}

static void __exit kix_exit_module(void){
	ixgbe_exit();
}

module_init(kix_init_module);
module_exit(kix_exit_module);