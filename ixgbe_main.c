/**
 * Modified from ix/igb_stub/igb_stub.c
 * Maybe need to check linux/ixgbe driver to complete it
 * Removed all the other codes irrelevant with 82599 to make it easy to read
 */

#include <linux/init.h>
#include <linux/module.h>

#include <linux/pci.h>		// for pci structures
#include <linux/dma-mapping.h>
#include <linux/aer.h>

#include "ixgbe_type.h"
#include "ixgbe.h"

MODULE_LICENSE("GPL");
char ixgbe_driver_name[] = "ixgbe";

/* ixgbe_pci_tbl - PCI Device ID Table
 *
 * Wildcard entries (PCI_ANY_ID) should come last
 * Last entry must be all 0s
 *
 * { Vendor ID, Device ID, SubVendor ID, SubDevice ID,
 *   Class, Class Mask, private data (not used) }
 */
static const struct pci_device_id ixgbe_pci_tbl[] = {
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_KX4), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_XAUI_LOM), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_KR), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_SFP), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_SFP_EM), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_KX4_MEZZ), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_CX4), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_BACKPLANE_FCOE), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_SFP_FCOE), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_T3_LOM), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_COMBO_BACKPLANE), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_X540T), board_X540 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_SFP_SF2), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_LS), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_QSFP_SF_QP), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599EN_SFP), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_SFP_SF_QP), board_82599 },
	/* required last entry */
	{0, }
};
MODULE_DEVICE_TABLE(pci, ixgbe_pci_tbl);

/**
 * ixgbe_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in ixgbe_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * ixgbe_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
/**
 * Porting eth_ixgbe_dev_init in deps/dpdk/drivers/net/ixgbe/ixgbe_ethdev.c
 */
static int ixgbe_probe(struct pci_dev *pdev, const struct pci_device_id *ent){
	int err;

	if (pdev->is_virtfn) {
		printk(KERN_ERR "%s (%hx:%hx) should not be a VF!\n",
		     pci_name(pdev), pdev->vendor, pdev->device);
		return -EINVAL;
	}

	printk(KERN_INFO "%s (%hx:%hx) correct VF device ID!\n",
		     pci_name(pdev), pdev->vendor, pdev->device);

	if(pci_enable_device(pdev)){
		printk(KERN_ERR "Cannot enable PCI device\n");
		goto fail;
	}

	if(pci_request_regions(pdev, "ixgbe")){
		printk(KERN_ERR "Cannot request regions\n");
		goto fail_disable;
	}

	pci_set_master(pdev);

	return 0;

fail_disable:
	pci_disable_device(dev);
fail:
    return -ENODEV;
}

static void ixgbe_remove(struct pci_dev *pdev){
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static struct pci_driver ixgbe_driver = {
	.name     = ixgbe_driver_name,
	.id_table = ixgbe_pci_tbl,
	.probe    = ixgbe_probe,
	.remove   = ixgbe_remove,
};

static int __init ixgbe_init_module(void){
	int ret = 0;
	printk(KERN_INFO "ixgbe driver loaded");

	ret = pci_register_driver(&ixgbe_driver);

	return ret;
}

static void __exit ixgbe_exit_module(void){
	printk(KERN_INFO "ixgbe driver removed");
	pci_unregister_driver(&ixgbe_driver);
}

module_init(ixgbe_init_module);
module_exit(ixgbe_exit_module);
