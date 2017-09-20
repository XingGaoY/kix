/**
 * Modified from ix/igb_stub/igb_stub.c
 * Maybe need to check linux/ixgbe driver to complete it
 * Removed all the other codes irrelevant with 82599 to make it easy to read
 */

#include <linux/init.h>
#include <linux/module.h>

#include <linux/pci.h>		// for pci structures
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/aer.h>
#include <linux/io.h>
#include <linux/bitops.h>

#include "ixgbe_type.h"
#include "ixgbe.h"
#include "ixgbe_ethdev.h"
#include "ixgbe_82599.h"
#include "ixgbe_common.h"
#include "ixgbe_dcb.h"
#include "ixgbe_api.h"
#include <ix/pci.h>

MODULE_LICENSE("GPL");

char ixgbe_driver_name[] = "ixgbe";
pci_addr_t pci_addr_tlb[PCI_TLB_SIZ];
int pci_ixgbe_registed_num = 0;

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
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_SFP_SF2), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_LS), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_QSFP_SF_QP), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599EN_SFP), board_82599 },
	{PCI_VDEVICE(INTEL, IXGBE_DEV_ID_82599_SFP_SF_QP), board_82599 },
	/* required last entry */
	{0, }
};
MODULE_DEVICE_TABLE(pci, ixgbe_pci_tbl);

static void ixgbe_remove_adapter(struct ixgbe_hw *hw)
{
	if (!hw->hw_addr)
		return;
	hw->hw_addr = NULL;
	printk(KERN_INFO "Adapter removed\n");
}

static void ixgbe_check_remove(struct ixgbe_hw *hw, u32 reg)
{
	u32 value;

	/* The following check not only optimizes a bit by not
	 * performing a read on the status register when the
	 * register just read was a status register read that
	 * returned IXGBE_FAILED_READ_REG. It also blocks any
	 * potential recursion.
	 */
	if (reg == IXGBE_STATUS) {
		ixgbe_remove_adapter(hw);
		return;
	}
	value = ixgbe_read_reg(hw, IXGBE_STATUS);
	if (value == IXGBE_FAILED_READ_REG)
		ixgbe_remove_adapter(hw);
}

/**
 * ixgbe_read_reg - Read from device register
 * @hw: hw specific details
 * @reg: offset of register to read
 *
 * Returns : value read or IXGBE_FAILED_READ_REG if removed
 *
 * This function is used to read device registers. It checks for device
 * removal by confirming any read that returns all ones by checking the
 * status register value for all ones. This function avoids reading from
 * the hardware if a removal was previously detected in which case it
 * returns IXGBE_FAILED_READ_REG (all ones).
 */
u32 ixgbe_read_reg(struct ixgbe_hw *hw, u32 reg)
{
	u8 __iomem *reg_addr = ACCESS_ONCE(hw->hw_addr);
	u32 value;

	value = readl(reg_addr + reg);
	if (unlikely(value == IXGBE_FAILED_READ_REG))
		ixgbe_check_remove(hw, reg);
	return value;
}

/*
 * This function resets queue statistics mapping registers.
 * From Niantic datasheet, Initialization of Statistics section:
 * "...if software requires the queue counters, the RQSMR and TQSM registers
 * must be re-programmed following a device reset.
 */
static void
ixgbe_reset_qstat_mappings(struct ixgbe_hw *hw)
{
	uint32_t i;

	for(i = 0; i != IXGBE_NB_STAT_MAPPING_REGS; i++) {
		IXGBE_WRITE_REG(hw, IXGBE_RQSMR(i), 0);
		IXGBE_WRITE_REG(hw, IXGBE_TQSM(i), 0);
	}
}

static void ixgbe_dcb_init(struct ixgbe_hw *hw, struct ixgbe_dcb_config *dcb_config){
	uint8_t i;
	struct ixgbe_dcb_tc_config *tc;
	uint8_t dcb_max_tc = IXGBE_DCB_MAX_TRAFFIC_CLASS;

	dcb_config->num_tcs.pg_tcs = dcb_max_tc;
	dcb_config->num_tcs.pfc_tcs = dcb_max_tc;
	for (i = 0; i < dcb_max_tc; i++) {
		tc = &dcb_config->tc_config[i];
		tc->path[IXGBE_DCB_TX_CONFIG].bwg_id = i;
		tc->path[IXGBE_DCB_TX_CONFIG].bwg_percent =
				 (uint8_t)(100/dcb_max_tc + (i & 1));
		tc->path[IXGBE_DCB_RX_CONFIG].bwg_id = i;
		tc->path[IXGBE_DCB_RX_CONFIG].bwg_percent =
				 (uint8_t)(100/dcb_max_tc + (i & 1));
		tc->pfc = ixgbe_dcb_pfc_disabled;
	}

	/* Initialize default user to priority mapping, UPx->TC0 */
	tc = &dcb_config->tc_config[0];
	tc->path[IXGBE_DCB_TX_CONFIG].up_to_tc_bitmap = 0xFF;
	tc->path[IXGBE_DCB_RX_CONFIG].up_to_tc_bitmap = 0xFF;
	for (i = 0; i< IXGBE_DCB_MAX_BW_GROUP; i++) {
		dcb_config->bw_percentage[IXGBE_DCB_TX_CONFIG][i] = 100;
		dcb_config->bw_percentage[IXGBE_DCB_RX_CONFIG][i] = 100;
	}
	dcb_config->rx_pba_cfg = ixgbe_dcb_pba_equal;
	dcb_config->pfc_mode_enable = false;
	dcb_config->vt_mode = true;
	dcb_config->round_robin_enable = false;
	/* support all DCB capabilities in 82599 */
	dcb_config->support.capabilities = 0xFF;
}

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
	struct rte_eth_dev *netdev;
	struct ixgbe_adapter *adapter = NULL;
	struct ixgbe_hw *hw;
	struct ixgbe_dcb_config *dcb_config;
	struct ixgbe_vfta *shadow_vfta;
	struct ixgbe_hwstrip *hwstrip;
	int pci_using_dac, err, i;
	bool disable_dev = false;
	uint16_t csum;

	if (pdev->is_virtfn) {
		printk(KERN_ERR "%s (%hx:%hx) should not be a VF!\n",
		     pci_name(pdev), pdev->vendor, pdev->device);
		return -EINVAL;
	}

	printk(KERN_INFO "%s (%hx:%hx) correct VF device ID!\n",
		     pci_name(pdev), pdev->vendor, pdev->device);

	pci_addr_tlb[pci_ixgbe_registed_num++].addr = pci_name(pdev);

	err = pci_enable_device_mem(pdev);
	if(err)
		return err;

	if(!dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64))){
		pci_using_dac = 1;
	}
	else{
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if(err){
			dev_err(&pdev->dev,
				"No usable DMA configuration, aborting\n");
			goto err_dma;
		}
	}

	err = pci_request_mem_regions(pdev, ixgbe_driver_name);
	if(err){
		dev_err(&pdev->dev,
			"pci_request_selected_regions failed 0x%x\n", err);
		goto err_pci_reg;
	}

	pci_enable_pcie_error_reporting(pdev);

	pci_set_master(pdev);
	pci_save_state(pdev);

	/**
	 * init eth dev, this is done in ix/ixgbe/ixgbe_ethdev.c
	 * moved here for it is usually done in probe function
	 */
	netdev = eth_dev_alloc(sizeof(struct ixgbe_adapter));
	if(!netdev){
		err = -ENOMEM;
		goto err_alloc_etherdev;
	}

	netdev_init(netdev, pdev);

	adapter = (struct ixgbe_adapter *)(netdev->data->dev_private);

	hw = IXGBE_DEV_PRIVATE_TO_HW(netdev->data->dev_private);
	hw->back = adapter;
	hw->device_id = pdev->device;
	hw->vendor_id = pdev->vendor;
	printk(KERN_INFO "ixgbe: vendorID=0x%x deviceID=0x%x",
		pdev->vendor, pdev->device);
	//adapter->msg_enable

	hw->hw_addr = ioremap(pci_resource_start(pdev, 0),
		pci_resource_len(pdev,0));
	if(!hw->hw_addr){
		err = -EIO;
		goto err_ioremap;
	}

	ixgbe_init_ops_82599(hw);

	dcb_config = &adapter->dcb_config;
	memset(dcb_config, 0, sizeof(struct ixgbe_dcb_config));
	ixgbe_dcb_init(hw, dcb_config);

	/* Get Hardware Flow Control setting */
	hw->fc.requested_mode = ixgbe_fc_full;
	hw->fc.current_mode = ixgbe_fc_full;
	hw->fc.pause_time = IXGBE_FC_PAUSE;
	for (i = 0; i < IXGBE_DCB_MAX_TRAFFIC_CLASS; i++) {
		hw->fc.low_water[i] = IXGBE_FC_LO;
		hw->fc.high_water[i] = IXGBE_FC_HI;
	}
	hw->fc.send_xon = 1;

	err = ixgbe_validate_eeprom_checksum(hw, &csum);;
	if(err != IXGBE_SUCCESS){
		printk(KERN_INFO "ixgbe: The EEPROM checksum is not valid: %d", err);
		goto err_eeprom_csum;
	}

	err = ixgbe_init_hw(hw);

	/*
	 * Devices with copper phys will fail to initialise if ixgbe_init_hw()
	 * is called too soon after the kernel driver unbinding/binding occurs.
	 * The failure occurs in ixgbe_identify_phy_generic() for all devices,
	 * but for non-copper devies, ixgbe_identify_sfp_module_generic() is
	 * also called. See ixgbe_identify_phy_82599(). The reason for the
	 * failure is not known, and only occuts when virtualisation features
	 * are disabled in the bios. A delay of 100ms  was found to be enough by
	 * trial-and-error, and is doubled to be safe.
	 */
	if (err && (hw->mac.ops.get_media_type(hw) == ixgbe_media_type_copper)) {
		msec_delay(200);
		err = ixgbe_init_hw(hw);
	}

	if (err == IXGBE_ERR_EEPROM_VERSION) {
		printk(KERN_INFO "ixgbe: This device is a pre-production adapter/"
		    "LOM.  Please be aware there may be issues associated "
		    "with your hardware.\n If you are experiencing problems "
		    "please contact your Intel or hardware representative "
		    "who provided you with this hardware.\n");
	} else if (err == IXGBE_ERR_SFP_NOT_SUPPORTED)
		printk(KERN_INFO "ixgbe: Unsupported SFP+ Module\n");
	if (err) {
		printk(KERN_INFO "ixgbe: Hardware Initialization Failure: %d", err);
		err = -EIO;
		goto err_init_hw;
	}

	/**
	 * Disable interrupt
	 * I'm not sure if I really need to mask this here
	 */
	ixgbe_irq_disable(hw);

	/* pick up the PCI bus settings for reporting later */
	ixgbe_get_bus_info(hw);

	/* reset mappings for queue statistics hw counters*/
	ixgbe_reset_qstat_mappings(hw);

	/* Allocate memory for storing MAC addresses */
	netdev->data->mac_addrs = kmalloc(hw->mac.num_rar_entries * ETH_ADDR_LEN, GFP_KERNEL);
	if (!netdev->data->mac_addrs) {
		printk(KERN_INFO "ixgbe: Failed to allocate %d bytes needed to store MAC addresses",
			ETH_ADDR_LEN * hw->mac.num_rar_entries);
		return -ENOMEM;
	}

	/* Copy the permanent MAC address */
	memcpy(&netdev->data->mac_addrs[0], hw->mac.perm_addr, ETH_ADDR_LEN);

	/* Allocate memory for storing hash filter MAC addresses */
	netdev->data->hash_mac_addrs = kmalloc(ETH_ADDR_LEN * IXGBE_VMDQ_NUM_UC_MAC, GFP_KERNEL);
	if (!netdev->data->hash_mac_addrs) {
		printk(KERN_INFO "ixgbe: Failed to allocate %d bytes needed to store MAC addresses",
			ETH_ADDR_LEN * IXGBE_VMDQ_NUM_UC_MAC);
		return -ENOMEM;
	}
	memset(netdev->data->hash_mac_addrs, 0, ETH_ADDR_LEN * IXGBE_VMDQ_NUM_UC_MAC);

	/* initialize the vfta */
	shadow_vfta = IXGBE_DEV_PRIVATE_TO_VFTA(netdev->data->dev_private);
	memset(shadow_vfta, 0, sizeof(*shadow_vfta));

	/* initialize the hw strip bitmap*/
	hwstrip = IXGBE_DEV_PRIVATE_TO_HWSTRIP_BITMAP(netdev->data->dev_private);
	memset(hwstrip, 0, sizeof(*hwstrip));

	return 0;
err_init_hw:
err_eeprom_csum:
err_ioremap:
	disable_dev = !test_and_set_bit(__IXGBE_DISABLED, &adapter->state);
	kfree(netdev->data->mac_addrs);
	kfree(netdev);
err_alloc_etherdev:
	pci_release_mem_regions(pdev);
err_pci_reg:
err_dma:
	if(!adapter || disable_dev)
		pci_disable_device(pdev);
	return err;
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
