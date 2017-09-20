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

/*
 * High threshold controlling when to start sending XOFF frames. Must be at
 * least 8 bytes less than receive packet buffer size. This value is in units
 * of 1024 bytes.
 */
#define IXGBE_FC_HI    0x80

/*
 * Low threshold controlling when to start sending XON frames. This value is
 * in units of 1024 bytes.
 */
#define IXGBE_FC_LO    0x40

/* Timer value included in XOFF frames. */
#define IXGBE_FC_PAUSE 0x680

#define IXGBE_LINK_DOWN_CHECK_TIMEOUT 4000 /* ms */
#define IXGBE_LINK_UP_CHECK_TIMEOUT   1000 /* ms */
#define IXGBE_VMDQ_NUM_UC_MAC         4096 /* Maximum nb. of UC MAC addr. */

#define IXGBE_QUEUE_STAT_COUNTERS (sizeof(hw_stats->qprc) / sizeof(hw_stats->qprc[0]))
char ixgbe_driver_name[] = "ixgbe";
pci_addr_t pci_addr_tlb[PCI_TLB_SIZ];
int pci_ixgbe_registed_num = 0;

static inline void ixgbe_irq_enable(struct rte_eth_dev *dev);
static void ixgbe_vlan_hw_strip_enable(struct rte_eth_dev *dev, uint16_t queue);
static void ixgbe_vlan_hw_strip_disable(struct rte_eth_dev *dev, uint16_t queue);
static void ixgbe_dev_promiscuous_enable(struct rte_eth_dev *dev);
static void ixgbe_dev_promiscuous_disable(struct rte_eth_dev *dev);
static void ixgbe_dev_allmulticast_enable(struct rte_eth_dev *dev);
static void ixgbe_dev_allmulticast_disable(struct rte_eth_dev *dev);
static int ixgbe_dev_link_update(struct rte_eth_dev *dev,
				int wait_to_complete);
static void ixgbe_dev_stats_get(struct rte_eth_dev *dev,
				struct rte_eth_stats *stats);
static void ixgbe_dev_stats_reset(struct rte_eth_dev *dev);
static int ixgbe_dev_queue_stats_mapping_set(struct rte_eth_dev *eth_dev,
					     uint16_t queue_id,
					     uint8_t stat_idx,
					     uint8_t is_rx);
static void ixgbe_dev_info_get(struct rte_eth_dev *dev,
				struct rte_eth_dev_info *dev_info);
static int ixgbe_vlan_filter_set(struct rte_eth_dev *dev,
		uint16_t vlan_id, int on);
static void ixgbe_vlan_tpid_set(struct rte_eth_dev *dev, uint16_t tpid_id);
static void ixgbe_vlan_hw_strip_bitmap_set(struct rte_eth_dev *dev,
		uint16_t queue, bool on);
static void ixgbe_vlan_strip_queue_set(struct rte_eth_dev *dev, uint16_t queue,
		int on);
static void ixgbe_vlan_offload_set(struct rte_eth_dev *dev, int mask);
static void ixgbe_vlan_hw_strip_enable(struct rte_eth_dev *dev, uint16_t queue);
static void ixgbe_vlan_hw_strip_disable(struct rte_eth_dev *dev, uint16_t queue);
static void ixgbe_vlan_hw_extend_enable(struct rte_eth_dev *dev);
static void ixgbe_vlan_hw_extend_disable(struct rte_eth_dev *dev);

static int ixgbe_dev_led_on(struct rte_eth_dev *dev);
static int ixgbe_dev_led_off(struct rte_eth_dev *dev);
static int  ixgbe_flow_ctrl_set(struct rte_eth_dev *dev,
		struct rte_eth_fc_conf *fc_conf);
static int ixgbe_priority_flow_ctrl_set(struct rte_eth_dev *dev,
		struct rte_eth_pfc_conf *pfc_conf);
static int ixgbe_dev_rss_reta_update(struct rte_eth_dev *dev,
		struct rte_eth_rss_reta *reta_conf);
static int ixgbe_dev_rss_reta_query(struct rte_eth_dev *dev,
		struct rte_eth_rss_reta *reta_conf);
static void ixgbe_dev_link_status_print(struct rte_eth_dev *dev);
static int ixgbe_dev_lsc_interrupt_setup(struct rte_eth_dev *dev);
static inline void ixgbe_irq_disable(struct ixgbe_hw *hw);
static void ixgbe_add_rar(struct rte_eth_dev *dev, struct eth_addr *mac_addr,
		uint32_t index, uint32_t pool);
static void ixgbe_remove_rar(struct rte_eth_dev *dev, uint32_t index);
static void ixgbe_dcb_init(struct ixgbe_hw *hw,struct ixgbe_dcb_config *dcb_config);

/* For Eth VMDQ APIs support */
static int ixgbe_uc_hash_table_set(struct rte_eth_dev *dev, struct
		eth_addr* mac_addr,uint8_t on);
static int ixgbe_uc_all_hash_table_set(struct rte_eth_dev *dev,uint8_t on);
static int  ixgbe_set_pool_rx_mode(struct rte_eth_dev *dev,  uint16_t pool,
		uint16_t rx_mask, uint8_t on);
static int ixgbe_set_pool_rx(struct rte_eth_dev *dev,uint16_t pool,uint8_t on);
static int ixgbe_set_pool_tx(struct rte_eth_dev *dev,uint16_t pool,uint8_t on);
static int ixgbe_set_pool_vlan_filter(struct rte_eth_dev *dev, uint16_t vlan,
		uint64_t pool_mask,uint8_t vlan_on);
static int ixgbe_mirror_rule_set(struct rte_eth_dev *dev,
		struct rte_eth_vmdq_mirror_conf *mirror_conf,
		uint8_t rule_id, uint8_t on);
static int ixgbe_mirror_rule_reset(struct rte_eth_dev *dev,
		uint8_t	rule_id);

/*
 * Define VF Stats MACRO for Non "cleared on read" register
 */
#define UPDATE_VF_STAT(reg, last, cur)	                        \
{                                                               \
	u32 latest = IXGBE_READ_REG(hw, reg);                   \
	cur += latest - last;                                   \
	last = latest;                                          \
}

#define UPDATE_VF_STAT_36BIT(lsb, msb, last, cur)                \
{                                                                \
	u64 new_lsb = IXGBE_READ_REG(hw, lsb);                   \
	u64 new_msb = IXGBE_READ_REG(hw, msb);                   \
	u64 latest = ((new_msb << 32) | new_lsb);                \
	cur += (0x1000000000LL + latest - last) & 0xFFFFFFFFFLL; \
	last = latest;                                           \
}

#define IXGBE_SET_HWSTRIP(h, q) do{\
		uint32_t idx = (q) / (sizeof ((h)->bitmap[0]) * NBBY); \
		uint32_t bit = (q) % (sizeof ((h)->bitmap[0]) * NBBY); \
		(h)->bitmap[idx] |= 1 << bit;\
	}while(0)

#define IXGBE_CLEAR_HWSTRIP(h, q) do{\
		uint32_t idx = (q) / (sizeof ((h)->bitmap[0]) * NBBY); \
		uint32_t bit = (q) % (sizeof ((h)->bitmap[0]) * NBBY); \
		(h)->bitmap[idx] &= ~(1 << bit);\
	}while(0)

#define IXGBE_GET_HWSTRIP(h, q, r) do{\
		uint32_t idx = (q) / (sizeof ((h)->bitmap[0]) * NBBY); \
		uint32_t bit = (q) % (sizeof ((h)->bitmap[0]) * NBBY); \
		(r) = (h)->bitmap[idx] >> bit & 1;\
	}while(0)

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

static int ixgbe_dev_configure(struct rte_eth_dev *dev)
{
	struct ixgbe_interrupt *intr =
		IXGBE_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	/* set flag to update link status after init */
	intr->flags |= IXGBE_FLAG_NEED_LINK_UPDATE;

	return 0;
}

/*
 * This function is the same as ixgbe_is_sfp() in ixgbe/ixgbe.h.
 */
static inline int
ixgbe_is_sfp(struct ixgbe_hw *hw)
{
	switch (hw->phy.type) {
	case ixgbe_phy_sfp_avago:
	case ixgbe_phy_sfp_ftl:
	case ixgbe_phy_sfp_intel:
	case ixgbe_phy_sfp_unknown:
	case ixgbe_phy_sfp_passive_tyco:
	case ixgbe_phy_sfp_passive_unknown:
		return 1;
	default:
		return 0;
	}
}

static inline int32_t
ixgbe_pf_reset_hw(struct ixgbe_hw *hw)
{
	uint32_t ctrl_ext;
	int32_t status;

	status = ixgbe_reset_hw(hw);

	ctrl_ext = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
	/* Set PF Reset Done bit so PF/VF Mail Ops can work */
	ctrl_ext |= IXGBE_CTRL_EXT_PFRSTD;
	IXGBE_WRITE_REG(hw, IXGBE_CTRL_EXT, ctrl_ext);
	IXGBE_WRITE_FLUSH(hw);

	return status;
}


static int
ixgbe_vlan_filter_set(struct rte_eth_dev *dev, uint16_t vlan_id, int on)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct ixgbe_vfta * shadow_vfta =
		IXGBE_DEV_PRIVATE_TO_VFTA(dev->data->dev_private);
	uint32_t vfta;
	uint32_t vid_idx;
	uint32_t vid_bit;

	vid_idx = (uint32_t) ((vlan_id >> 5) & 0x7F);
	vid_bit = (uint32_t) (1 << (vlan_id & 0x1F));
	vfta = IXGBE_READ_REG(hw, IXGBE_VFTA(vid_idx));
	if (on)
		vfta |= vid_bit;
	else
		vfta &= ~vid_bit;
	IXGBE_WRITE_REG(hw, IXGBE_VFTA(vid_idx), vfta);

	/* update local VFTA copy */
	shadow_vfta->vfta[vid_idx] = vfta;

	return 0;
}

static void
ixgbe_vlan_strip_queue_set(struct rte_eth_dev *dev, uint16_t queue, int on)
{
	if (on)
		ixgbe_vlan_hw_strip_enable(dev, queue);
	else
		ixgbe_vlan_hw_strip_disable(dev, queue);
}

static void
ixgbe_vlan_tpid_set(struct rte_eth_dev *dev, uint16_t tpid)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Only the high 16-bits is valid */
	IXGBE_WRITE_REG(hw, IXGBE_EXVET, tpid << 16);
}

void
ixgbe_vlan_hw_filter_disable(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t vlnctrl;

	/* Filter Table Disable */
	vlnctrl = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);
	vlnctrl &= ~IXGBE_VLNCTRL_VFE;

	IXGBE_WRITE_REG(hw, IXGBE_VLNCTRL, vlnctrl);
}

void
ixgbe_vlan_hw_filter_enable(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct ixgbe_vfta * shadow_vfta =
		IXGBE_DEV_PRIVATE_TO_VFTA(dev->data->dev_private);
	uint32_t vlnctrl;
	uint16_t i;

	/* Filter Table Enable */
	vlnctrl = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);
	vlnctrl &= ~IXGBE_VLNCTRL_CFIEN;
	vlnctrl |= IXGBE_VLNCTRL_VFE;

	IXGBE_WRITE_REG(hw, IXGBE_VLNCTRL, vlnctrl);

	/* write whatever is in local vfta copy */
	for (i = 0; i < IXGBE_VFTA_SIZE; i++)
		IXGBE_WRITE_REG(hw, IXGBE_VFTA(i), shadow_vfta->vfta[i]);
}

static void
ixgbe_vlan_hw_strip_bitmap_set(struct rte_eth_dev *dev, uint16_t queue, bool on)
{
	struct ixgbe_hwstrip *hwstrip =
		IXGBE_DEV_PRIVATE_TO_HWSTRIP_BITMAP(dev->data->dev_private);

	if(queue >= IXGBE_MAX_RX_QUEUE_NUM)
		return;

	if (on)
		IXGBE_SET_HWSTRIP(hwstrip, queue);
	else
		IXGBE_CLEAR_HWSTRIP(hwstrip, queue);
}

static void
ixgbe_vlan_hw_strip_disable(struct rte_eth_dev *dev, uint16_t queue)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;

	/* Other 10G NIC, the VLAN strip can be setup per queue in RXDCTL */
	ctrl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(queue));
	ctrl &= ~IXGBE_RXDCTL_VME;
	IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(queue), ctrl);

	/* record those setting for HW strip per queue */
	ixgbe_vlan_hw_strip_bitmap_set(dev, queue, 0);
}

static void
ixgbe_vlan_hw_strip_enable(struct rte_eth_dev *dev, uint16_t queue)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;

	/* Other 10G NIC, the VLAN strip can be setup per queue in RXDCTL */
	ctrl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(queue));
	ctrl |= IXGBE_RXDCTL_VME;
	IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(queue), ctrl);

	/* record those setting for HW strip per queue */
	ixgbe_vlan_hw_strip_bitmap_set(dev, queue, 1);
}

void
ixgbe_vlan_hw_strip_disable_all(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;
	uint16_t i;

	/* Other 10G NIC, the VLAN strip can be setup per queue in RXDCTL */
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		ctrl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(i));
		ctrl &= ~IXGBE_RXDCTL_VME;
		IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(i), ctrl);

		/* record those setting for HW strip per queue */
		ixgbe_vlan_hw_strip_bitmap_set(dev, i, 0);
	}
}

void
ixgbe_vlan_hw_strip_enable_all(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;
	uint16_t i;

	/* Other 10G NIC, the VLAN strip can be setup per queue in RXDCTL */
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		ctrl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(i));
		ctrl |= IXGBE_RXDCTL_VME;
		IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(i), ctrl);

		/* record those setting for HW strip per queue */
		ixgbe_vlan_hw_strip_bitmap_set(dev, i, 1);
	}
}

static void
ixgbe_vlan_hw_extend_disable(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;

	/* DMATXCTRL: Geric Double VLAN Disable */
	ctrl = IXGBE_READ_REG(hw, IXGBE_DMATXCTL);
	ctrl &= ~IXGBE_DMATXCTL_GDV;
	IXGBE_WRITE_REG(hw, IXGBE_DMATXCTL, ctrl);

	/* CTRL_EXT: Global Double VLAN Disable */
	ctrl = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
	ctrl &= ~IXGBE_EXTENDED_VLAN;
	IXGBE_WRITE_REG(hw, IXGBE_CTRL_EXT, ctrl);

}

static void
ixgbe_vlan_hw_extend_enable(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t ctrl;

	/* DMATXCTRL: Geric Double VLAN Enable */
	ctrl  = IXGBE_READ_REG(hw, IXGBE_DMATXCTL);
	ctrl |= IXGBE_DMATXCTL_GDV;
	IXGBE_WRITE_REG(hw, IXGBE_DMATXCTL, ctrl);

	/* CTRL_EXT: Global Double VLAN Enable */
	ctrl  = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
	ctrl |= IXGBE_EXTENDED_VLAN;
	IXGBE_WRITE_REG(hw, IXGBE_CTRL_EXT, ctrl);

	/*
	 * VET EXT field in the EXVET register = 0x8100 by default
	 * So no need to change. Same to VT field of DMATXCTL register
	 */
}

static void
ixgbe_vlan_offload_set(struct rte_eth_dev *dev, int mask)
{
	if(mask & ETH_VLAN_STRIP_MASK){
		if (dev->data->dev_conf.rxmode.hw_vlan_strip)
			ixgbe_vlan_hw_strip_enable_all(dev);
		else
			ixgbe_vlan_hw_strip_disable_all(dev);
	}

	if(mask & ETH_VLAN_FILTER_MASK){
		if (dev->data->dev_conf.rxmode.hw_vlan_filter)
			ixgbe_vlan_hw_filter_enable(dev);
		else
			ixgbe_vlan_hw_filter_disable(dev);
	}

	if(mask & ETH_VLAN_EXTEND_MASK){
		if (dev->data->dev_conf.rxmode.hw_vlan_extend)
			ixgbe_vlan_hw_extend_enable(dev);
		else
			ixgbe_vlan_hw_extend_disable(dev);
	}
}

static void
ixgbe_vmdq_vlan_hw_filter_enable(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	/* VLNCTRL: enable vlan filtering and allow all vlan tags through */
	uint32_t vlanctrl = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);
	vlanctrl |= IXGBE_VLNCTRL_VFE ; /* enable vlan filters */
	IXGBE_WRITE_REG(hw, IXGBE_VLNCTRL, vlanctrl);
}

static void
ixgbe_restore_statistics_mapping(struct rte_eth_dev * dev)
{
	struct ixgbe_stat_mapping_registers *stat_mappings =
		IXGBE_DEV_PRIVATE_TO_STAT_MAPPINGS(dev->data->dev_private);
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int i;

	/* write whatever was in stat mapping table to the NIC */
	for (i = 0; i < IXGBE_NB_STAT_MAPPING_REGS; i++) {
		/* rx */
		IXGBE_WRITE_REG(hw, IXGBE_RQSMR(i), stat_mappings->rqsmr[i]);

		/* tx */
		IXGBE_WRITE_REG(hw, IXGBE_TQSM(i), stat_mappings->tqsm[i]);
	}
}

static void
ixgbe_dev_allmulticast_enable(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t fctrl;

	fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
	fctrl |= IXGBE_FCTRL_MPE;
	IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);
}

static void
ixgbe_dev_allmulticast_disable(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t fctrl;

	if (dev->data->promiscuous == 1)
		return; /* must remain in all_multicast mode */

	fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
	fctrl &= (~IXGBE_FCTRL_MPE);
	IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);
}

/**
 * It clears the interrupt causes and enables the interrupt.
 * It will be called once only during nic initialized.
 *
 * @param dev
 *  Pointer to struct rte_eth_dev.
 *
 * @return
 *  - On success, zero.
 *  - On failure, a negative value.
 */
static int
ixgbe_dev_lsc_interrupt_setup(struct rte_eth_dev *dev)
{
	struct ixgbe_interrupt *intr =
		IXGBE_DEV_PRIVATE_TO_INTR(dev->data->dev_private);

	intr->mask |= IXGBE_EICR_LSC;

	return 0;
}

/*
 * Configure device link speed and setup link.
 * It returns 0 on success.
 */
static int
ixgbe_dev_start(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int err;
	uint32_t speed = 0;
	int mask = 0;
	int status;
	bool link_up = 0, negotiate = 0;

	/* IXGBE devices don't support half duplex */
	if ((dev->data->dev_conf.link_duplex != ETH_LINK_AUTONEG_DUPLEX) &&
			(dev->data->dev_conf.link_duplex != ETH_LINK_FULL_DUPLEX)) {
		printk(KERN_INFO "ixgbe: Invalid link_duplex (%u) for port %u\n",
			dev->data->dev_conf.link_duplex, dev->data->port_id);
		return -EINVAL;
	}

	/* stop adapter */
	hw->adapter_stopped = FALSE;
	ixgbe_stop_adapter(hw);

	/* reinitialize adapter
	 * this calls reset and start */
	status = ixgbe_pf_reset_hw(hw);
	if (status != 0)
		return -1;
	hw->mac.ops.start_hw(hw);

	/* configure PF module if SRIOV enabled */
	ixgbe_pf_host_configure(dev);

	/* initialize transmission unit */
	ixgbe_dev_tx_init(dev);

	/* This can fail when allocating mbufs for descriptor rings */
	err = ixgbe_dev_rx_init(dev);
	if (err) {
		printk(KERN_INFO "ixgbe: Unable to initialize RX hardware\n");
		goto error;
	}

	ixgbe_dev_rxtx_start(dev);

	if (ixgbe_is_sfp(hw) && hw->phy.multispeed_fiber) {
		err = hw->mac.ops.setup_sfp(hw);
		if (err)
			goto error;
	}

	/* Turn on the laser */
	ixgbe_enable_tx_laser(hw);

	/* Skip link setup if loopback mode is enabled for 82599. */
	if (hw->mac.type == ixgbe_mac_82599EB &&
			dev->data->dev_conf.lpbk_mode == IXGBE_LPBK_82599_TX_RX)
		goto skip_link_setup;

	err = ixgbe_check_link(hw, &speed, &link_up, 0);
	if (err)
		goto error;
	err = ixgbe_get_link_capabilities(hw, &speed, &negotiate);
	if (err)
		goto error;

	switch(dev->data->dev_conf.link_speed) {
	case ETH_LINK_SPEED_AUTONEG:
		speed = (hw->mac.type != ixgbe_mac_82598EB) ?
				IXGBE_LINK_SPEED_82599_AUTONEG :
				IXGBE_LINK_SPEED_82598_AUTONEG;
		break;
	case ETH_LINK_SPEED_100:
		/*
		 * Invalid for 82598 but error will be detected by
		 * ixgbe_setup_link()
		 */
		speed = IXGBE_LINK_SPEED_100_FULL;
		break;
	case ETH_LINK_SPEED_1000:
		speed = IXGBE_LINK_SPEED_1GB_FULL;
		break;
	case ETH_LINK_SPEED_10000:
		speed = IXGBE_LINK_SPEED_10GB_FULL;
		break;
	default:
		printk(KERN_INFO "ixgbe: Invalid link_speed (%u) for port %u\n",
			dev->data->dev_conf.link_speed, dev->data->port_id);
		goto error;
	}

	err = ixgbe_setup_link(hw, speed, negotiate, link_up);
	if (err)
		goto error;

skip_link_setup:

	/* check if lsc interrupt is enabled */
	if (dev->data->dev_conf.intr_conf.lsc != 0)
		ixgbe_dev_lsc_interrupt_setup(dev);

	/* resume enabled intr since hw reset */
	ixgbe_irq_enable(dev);

	mask = ETH_VLAN_STRIP_MASK | ETH_VLAN_FILTER_MASK | \
		ETH_VLAN_EXTEND_MASK;
	ixgbe_vlan_offload_set(dev, mask);

	if (dev->data->dev_conf.rxmode.mq_mode == ETH_MQ_RX_VMDQ_ONLY) {
		/* Enable vlan filtering for VMDq */
		ixgbe_vmdq_vlan_hw_filter_enable(dev);
	}

	/* Configure DCB hw */
	ixgbe_configure_dcb(dev);

	if (dev->data->dev_conf.fdir_conf.mode != RTE_FDIR_MODE_NONE) {
		err = ixgbe_fdir_configure(dev);
		if (err)
			goto error;
	}

	ixgbe_restore_statistics_mapping(dev);

	return (0);

error:
	printk(KERN_INFO "ixgbe: failure in ixgbe_dev_start(): %d", err);
	ixgbe_dev_clear_queues(dev);
	return -EIO;
}

/*
 * Stop device: disable rx and tx functions to allow for reconfiguring.
 */
static void
ixgbe_dev_stop(struct rte_eth_dev *dev)
{
	struct rte_eth_link link;
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* disable interrupts */
	ixgbe_irq_disable(hw);

	/* reset the NIC */
	ixgbe_pf_reset_hw(hw);
	hw->adapter_stopped = FALSE;

	/* stop adapter */
	ixgbe_stop_adapter(hw);

	/* Turn off the laser */
	ixgbe_disable_tx_laser(hw);

	ixgbe_dev_clear_queues(dev);

	/* Clear recorded link status */
	memset(&link, 0, sizeof(link));
	dev->data->dev_link = link;
}

/*
 * Reest and stop device.
 */
static void
ixgbe_dev_close(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	ixgbe_pf_reset_hw(hw);

	ixgbe_dev_stop(dev);
	hw->adapter_stopped = 1;

	ixgbe_disable_pcie_master(hw);

	/* reprogram the RAR[0] in case user changed it. */
	ixgbe_set_rar(hw, 0, hw->mac.addr, 0, IXGBE_RAH_AV);
}

/*
 * This function is based on ixgbe_update_stats_counters() in ixgbe/ixgbe.c
 */
static void
ixgbe_dev_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *stats)
{
	struct ixgbe_hw *hw =
			IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct ixgbe_hw_stats *hw_stats =
			IXGBE_DEV_PRIVATE_TO_STATS(dev->data->dev_private);
	uint32_t bprc, lxon, lxoff, total;
	uint64_t total_missed_rx, total_qbrc, total_qprc;
	unsigned i;

	total_missed_rx = 0;
	total_qbrc = 0;
	total_qprc = 0;

	hw_stats->crcerrs += IXGBE_READ_REG(hw, IXGBE_CRCERRS);
	hw_stats->illerrc += IXGBE_READ_REG(hw, IXGBE_ILLERRC);
	hw_stats->errbc += IXGBE_READ_REG(hw, IXGBE_ERRBC);
	hw_stats->mspdc += IXGBE_READ_REG(hw, IXGBE_MSPDC);

	for (i = 0; i < 8; i++) {
		uint32_t mp;
		mp = IXGBE_READ_REG(hw, IXGBE_MPC(i));
		/* global total per queue */
		hw_stats->mpc[i] += mp;
		/* Running comprehensive total for stats display */
		total_missed_rx += hw_stats->mpc[i];
		if (hw->mac.type == ixgbe_mac_82598EB)
			hw_stats->rnbc[i] +=
			    IXGBE_READ_REG(hw, IXGBE_RNBC(i));
		hw_stats->pxontxc[i] +=
		    IXGBE_READ_REG(hw, IXGBE_PXONTXC(i));
		hw_stats->pxonrxc[i] +=
		    IXGBE_READ_REG(hw, IXGBE_PXONRXC(i));
		hw_stats->pxofftxc[i] +=
		    IXGBE_READ_REG(hw, IXGBE_PXOFFTXC(i));
		hw_stats->pxoffrxc[i] +=
		    IXGBE_READ_REG(hw, IXGBE_PXOFFRXC(i));
		hw_stats->pxon2offc[i] +=
		    IXGBE_READ_REG(hw, IXGBE_PXON2OFFCNT(i));
	}
	for (i = 0; i < IXGBE_QUEUE_STAT_COUNTERS; i++) {
		hw_stats->qprc[i] += IXGBE_READ_REG(hw, IXGBE_QPRC(i));
		hw_stats->qptc[i] += IXGBE_READ_REG(hw, IXGBE_QPTC(i));
		hw_stats->qbrc[i] += IXGBE_READ_REG(hw, IXGBE_QBRC_L(i));
		hw_stats->qbrc[i] +=
		    ((uint64_t)IXGBE_READ_REG(hw, IXGBE_QBRC_H(i)) << 32);
		hw_stats->qbtc[i] += IXGBE_READ_REG(hw, IXGBE_QBTC_L(i));
		hw_stats->qbtc[i] +=
		    ((uint64_t)IXGBE_READ_REG(hw, IXGBE_QBTC_H(i)) << 32);
		hw_stats->qprdc[i] += IXGBE_READ_REG(hw, IXGBE_QPRDC(i));

		total_qprc += hw_stats->qprc[i];
		total_qbrc += hw_stats->qbrc[i];
	}
	hw_stats->mlfc += IXGBE_READ_REG(hw, IXGBE_MLFC);
	hw_stats->mrfc += IXGBE_READ_REG(hw, IXGBE_MRFC);
	hw_stats->rlec += IXGBE_READ_REG(hw, IXGBE_RLEC);

	/* Note that gprc counts missed packets */
	hw_stats->gprc += IXGBE_READ_REG(hw, IXGBE_GPRC);

	if (hw->mac.type != ixgbe_mac_82598EB) {
		hw_stats->gorc += IXGBE_READ_REG(hw, IXGBE_GORCL);
		hw_stats->gorc += ((u64)IXGBE_READ_REG(hw, IXGBE_GORCH) << 32);
		hw_stats->gotc += IXGBE_READ_REG(hw, IXGBE_GOTCL);
		hw_stats->gotc += ((u64)IXGBE_READ_REG(hw, IXGBE_GOTCH) << 32);
		hw_stats->tor += IXGBE_READ_REG(hw, IXGBE_TORL);
		hw_stats->tor += ((u64)IXGBE_READ_REG(hw, IXGBE_TORH) << 32);
		hw_stats->lxonrxc += IXGBE_READ_REG(hw, IXGBE_LXONRXCNT);
		hw_stats->lxoffrxc += IXGBE_READ_REG(hw, IXGBE_LXOFFRXCNT);
	} else {
		hw_stats->lxonrxc += IXGBE_READ_REG(hw, IXGBE_LXONRXC);
		hw_stats->lxoffrxc += IXGBE_READ_REG(hw, IXGBE_LXOFFRXC);
		/* 82598 only has a counter in the high register */
		hw_stats->gorc += IXGBE_READ_REG(hw, IXGBE_GORCH);
		hw_stats->gotc += IXGBE_READ_REG(hw, IXGBE_GOTCH);
		hw_stats->tor += IXGBE_READ_REG(hw, IXGBE_TORH);
	}

	/*
	 * Workaround: mprc hardware is incorrectly counting
	 * broadcasts, so for now we subtract those.
	 */
	bprc = IXGBE_READ_REG(hw, IXGBE_BPRC);
	hw_stats->bprc += bprc;
	hw_stats->mprc += IXGBE_READ_REG(hw, IXGBE_MPRC);
	if (hw->mac.type == ixgbe_mac_82598EB)
		hw_stats->mprc -= bprc;

	hw_stats->prc64 += IXGBE_READ_REG(hw, IXGBE_PRC64);
	hw_stats->prc127 += IXGBE_READ_REG(hw, IXGBE_PRC127);
	hw_stats->prc255 += IXGBE_READ_REG(hw, IXGBE_PRC255);
	hw_stats->prc511 += IXGBE_READ_REG(hw, IXGBE_PRC511);
	hw_stats->prc1023 += IXGBE_READ_REG(hw, IXGBE_PRC1023);
	hw_stats->prc1522 += IXGBE_READ_REG(hw, IXGBE_PRC1522);

	lxon = IXGBE_READ_REG(hw, IXGBE_LXONTXC);
	hw_stats->lxontxc += lxon;
	lxoff = IXGBE_READ_REG(hw, IXGBE_LXOFFTXC);
	hw_stats->lxofftxc += lxoff;
	total = lxon + lxoff;

	hw_stats->gptc += IXGBE_READ_REG(hw, IXGBE_GPTC);
	hw_stats->mptc += IXGBE_READ_REG(hw, IXGBE_MPTC);
	hw_stats->ptc64 += IXGBE_READ_REG(hw, IXGBE_PTC64);
	hw_stats->gptc -= total;
	hw_stats->mptc -= total;
	hw_stats->ptc64 -= total;
	hw_stats->gotc -= total * ETH_MIN_LEN;

	hw_stats->ruc += IXGBE_READ_REG(hw, IXGBE_RUC);
	hw_stats->rfc += IXGBE_READ_REG(hw, IXGBE_RFC);
	hw_stats->roc += IXGBE_READ_REG(hw, IXGBE_ROC);
	hw_stats->rjc += IXGBE_READ_REG(hw, IXGBE_RJC);
	hw_stats->mngprc += IXGBE_READ_REG(hw, IXGBE_MNGPRC);
	hw_stats->mngpdc += IXGBE_READ_REG(hw, IXGBE_MNGPDC);
	hw_stats->mngptc += IXGBE_READ_REG(hw, IXGBE_MNGPTC);
	hw_stats->tpr += IXGBE_READ_REG(hw, IXGBE_TPR);
	hw_stats->tpt += IXGBE_READ_REG(hw, IXGBE_TPT);
	hw_stats->ptc127 += IXGBE_READ_REG(hw, IXGBE_PTC127);
	hw_stats->ptc255 += IXGBE_READ_REG(hw, IXGBE_PTC255);
	hw_stats->ptc511 += IXGBE_READ_REG(hw, IXGBE_PTC511);
	hw_stats->ptc1023 += IXGBE_READ_REG(hw, IXGBE_PTC1023);
	hw_stats->ptc1522 += IXGBE_READ_REG(hw, IXGBE_PTC1522);
	hw_stats->bptc += IXGBE_READ_REG(hw, IXGBE_BPTC);
	hw_stats->xec += IXGBE_READ_REG(hw, IXGBE_XEC);
	hw_stats->fccrc += IXGBE_READ_REG(hw, IXGBE_FCCRC);
	hw_stats->fclast += IXGBE_READ_REG(hw, IXGBE_FCLAST);
	/* Only read FCOE on 82599 */
	if (hw->mac.type != ixgbe_mac_82598EB) {
		hw_stats->fcoerpdc += IXGBE_READ_REG(hw, IXGBE_FCOERPDC);
		hw_stats->fcoeprc += IXGBE_READ_REG(hw, IXGBE_FCOEPRC);
		hw_stats->fcoeptc += IXGBE_READ_REG(hw, IXGBE_FCOEPTC);
		hw_stats->fcoedwrc += IXGBE_READ_REG(hw, IXGBE_FCOEDWRC);
		hw_stats->fcoedwtc += IXGBE_READ_REG(hw, IXGBE_FCOEDWTC);
	}

	if (stats == NULL)
		return;

	/* Fill out the rte_eth_stats statistics structure */
	stats->ipackets = total_qprc;
	stats->ibytes = total_qbrc;
	stats->opackets = hw_stats->gptc;
	stats->obytes = hw_stats->gotc;
	stats->imcasts = hw_stats->mprc;

	for (i = 0; i < IXGBE_QUEUE_STAT_COUNTERS; i++) {
		stats->q_ipackets[i] = hw_stats->qprc[i];
		stats->q_opackets[i] = hw_stats->qptc[i];
		stats->q_ibytes[i] = hw_stats->qbrc[i];
		stats->q_obytes[i] = hw_stats->qbtc[i];
		stats->q_errors[i] = hw_stats->qprdc[i];
	}

	/* Rx Errors */
	stats->ierrors = total_missed_rx + hw_stats->crcerrs +
		hw_stats->rlec;

	stats->oerrors  = 0;

	/* XON/XOFF pause frames */
	stats->tx_pause_xon  = hw_stats->lxontxc;
	stats->rx_pause_xon  = hw_stats->lxonrxc;
	stats->tx_pause_xoff = hw_stats->lxofftxc;
	stats->rx_pause_xoff = hw_stats->lxoffrxc;

	/* Flow Director Stats registers */
	hw_stats->fdirmatch += IXGBE_READ_REG(hw, IXGBE_FDIRMATCH);
	hw_stats->fdirmiss += IXGBE_READ_REG(hw, IXGBE_FDIRMISS);
	stats->fdirmatch = hw_stats->fdirmatch;
	stats->fdirmiss = hw_stats->fdirmiss;
}

static void
ixgbe_dev_stats_reset(struct rte_eth_dev *dev)
{
	struct ixgbe_hw_stats *stats =
			IXGBE_DEV_PRIVATE_TO_STATS(dev->data->dev_private);

	/* HW registers are cleared on read */
	ixgbe_dev_stats_get(dev, NULL);

	/* Reset software totals */
	memset(stats, 0, sizeof(*stats));
}

static void
ixgbe_dev_info_get(struct rte_eth_dev *dev, struct rte_eth_dev_info *dev_info)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	dev_info->max_rx_queues = (uint16_t)hw->mac.max_rx_queues;
	dev_info->max_tx_queues = (uint16_t)hw->mac.max_tx_queues;
	dev_info->nb_rx_fgs = 128;
	dev_info->min_rx_bufsize = 1024; /* cf BSIZEPACKET in SRRCTL register */
	dev_info->max_rx_pktlen = 15872; /* includes CRC, cf MAXFRS register */
	dev_info->max_mac_addrs = hw->mac.num_rar_entries;
	dev_info->max_hash_mac_addrs = IXGBE_VMDQ_NUM_UC_MAC;
	dev_info->max_vfs = pci_sriov_get_totalvfs(dev->pci_dev);
	dev_info->max_vmdq_pools = ETH_64_POOLS;
	dev_info->rx_offload_capa =
		DEV_RX_OFFLOAD_VLAN_STRIP |
		DEV_RX_OFFLOAD_IPV4_CKSUM |
		DEV_RX_OFFLOAD_UDP_CKSUM  |
		DEV_RX_OFFLOAD_TCP_CKSUM;
	dev_info->tx_offload_capa =
		DEV_TX_OFFLOAD_VLAN_INSERT |
		DEV_TX_OFFLOAD_IPV4_CKSUM  |
		DEV_TX_OFFLOAD_UDP_CKSUM   |
		DEV_TX_OFFLOAD_TCP_CKSUM   |
		DEV_TX_OFFLOAD_SCTP_CKSUM;
}

/* return 0 means link status changed, -1 means not changed */
static int
ixgbe_dev_link_update(struct rte_eth_dev *dev, int wait_to_complete)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_eth_link link, old;
	ixgbe_link_speed link_speed;
	int diag;
	bool link_up;

	link.link_status = 0;
	link.link_speed = 0;
	link.link_duplex = 0;
	memset(&old, 0, sizeof(old));
	old = dev->data->dev_link;

	/* check if it needs to wait to complete, if lsc interrupt is enabled */
	if (wait_to_complete == 0 || dev->data->dev_conf.intr_conf.lsc != 0)
		diag = ixgbe_check_link(hw, &link_speed, &link_up, 0);
	else
		diag = ixgbe_check_link(hw, &link_speed, &link_up, 1);
	if (diag != 0) {
		link.link_speed = ETH_LINK_SPEED_100;
		link.link_duplex = ETH_LINK_HALF_DUPLEX;
		dev->data->dev_link = link;
		if (link.link_status == old.link_status)
			return -1;
		return 0;
	}

	if (link_up == 0) {
		dev->data->dev_link = link;
		if (link.link_status == old.link_status)
			return -1;
		return 0;
	}
	link.link_status = 1;
	link.link_duplex = ETH_LINK_FULL_DUPLEX;

	switch (link_speed) {
	default:
	case IXGBE_LINK_SPEED_UNKNOWN:
		link.link_duplex = ETH_LINK_HALF_DUPLEX;
		link.link_speed = ETH_LINK_SPEED_100;
		break;

	case IXGBE_LINK_SPEED_100_FULL:
		link.link_speed = ETH_LINK_SPEED_100;
		break;

	case IXGBE_LINK_SPEED_1GB_FULL:
		link.link_speed = ETH_LINK_SPEED_1000;
		break;

	case IXGBE_LINK_SPEED_10GB_FULL:
		link.link_speed = ETH_LINK_SPEED_10000;
		break;
	}

	dev->data->dev_link = link;

	if (link.link_status == old.link_status)
		return -1;

	return 0;
}

static struct eth_dev_ops ixgbe_eth_dev_ops = {
	.dev_configure        = ixgbe_dev_configure,
	.dev_start            = ixgbe_dev_start,
	.dev_stop             = ixgbe_dev_stop,
	.dev_close            = ixgbe_dev_close,
	.promiscuous_enable   = ixgbe_dev_promiscuous_enable,
	.promiscuous_disable  = ixgbe_dev_promiscuous_disable,
	.allmulticast_enable  = ixgbe_dev_allmulticast_enable,
	.allmulticast_disable = ixgbe_dev_allmulticast_disable,
	.link_update          = ixgbe_dev_link_update,
	.stats_get            = ixgbe_dev_stats_get,
	.stats_reset          = ixgbe_dev_stats_reset,
	.queue_stats_mapping_set = ixgbe_dev_queue_stats_mapping_set,
	.dev_infos_get        = ixgbe_dev_info_get,
	.vlan_filter_set      = ixgbe_vlan_filter_set,
	.vlan_tpid_set        = ixgbe_vlan_tpid_set,
	.vlan_offload_set     = ixgbe_vlan_offload_set,
	.vlan_strip_queue_set = ixgbe_vlan_strip_queue_set,
	.rx_queue_setup       = ixgbe_dev_rx_queue_setup,
	.rx_queue_release     = ixgbe_dev_rx_queue_release,
	.tx_queue_setup       = ixgbe_dev_tx_queue_setup,
	.tx_queue_release     = ixgbe_dev_tx_queue_release,
	.dev_led_on           = ixgbe_dev_led_on,
	.dev_led_off          = ixgbe_dev_led_off,
	.flow_ctrl_set        = ixgbe_flow_ctrl_set,
	.priority_flow_ctrl_set = ixgbe_priority_flow_ctrl_set,
	.mac_addr_add         = ixgbe_add_rar,
	.mac_addr_remove      = ixgbe_remove_rar,
	.uc_hash_table_set    = ixgbe_uc_hash_table_set,
	.uc_all_hash_table_set  = ixgbe_uc_all_hash_table_set,
	.mirror_rule_set   	= ixgbe_mirror_rule_set,
	.mirror_rule_reset 	= ixgbe_mirror_rule_reset,
	.set_vf_rx_mode       = ixgbe_set_pool_rx_mode,
	.set_vf_rx            = ixgbe_set_pool_rx,
	.set_vf_tx            = ixgbe_set_pool_tx,
	.set_vf_vlan_filter   = ixgbe_set_pool_vlan_filter,
	.fdir_add_signature_filter    = ixgbe_fdir_add_signature_filter,
	.fdir_update_signature_filter = ixgbe_fdir_update_signature_filter,
	.fdir_remove_signature_filter = ixgbe_fdir_remove_signature_filter,
	.fdir_infos_get               = ixgbe_fdir_info_get,
	.fdir_add_perfect_filter      = ixgbe_fdir_add_perfect_filter,
	.fdir_update_perfect_filter   = ixgbe_fdir_update_perfect_filter,
	.fdir_remove_perfect_filter   = ixgbe_fdir_remove_perfect_filter,
	.fdir_set_masks               = ixgbe_fdir_set_masks,
	.reta_update          = ixgbe_dev_rss_reta_update,
	.reta_query           = ixgbe_dev_rss_reta_query,
};

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

static inline void ixgbe_irq_enable(struct rte_eth_dev *dev)
{
	struct ixgbe_interrupt *intr =
		IXGBE_DEV_PRIVATE_TO_INTR(dev->data->dev_private);
	struct ixgbe_hw *hw =
		IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	IXGBE_WRITE_REG(hw, IXGBE_EIMS, intr->mask);
	IXGBE_WRITE_FLUSH(hw);
}

/**
 * ixgbe_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/
static inline void ixgbe_irq_disable(struct ixgbe_hw *hw)
{
	switch (hw->mac.type) {
	case ixgbe_mac_82599EB:
		IXGBE_WRITE_REG(hw, IXGBE_EIMC, 0xFFFF0000);
		IXGBE_WRITE_REG(hw, IXGBE_EIMC_EX(0), ~0);
		IXGBE_WRITE_REG(hw, IXGBE_EIMC_EX(1), ~0);
		break;
	default:
		break;
	}
	IXGBE_WRITE_FLUSH(hw);
}

static void
ixgbe_dev_promiscuous_enable(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t fctrl;

	fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
	fctrl |= (IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
	IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);
}

static void
ixgbe_dev_promiscuous_disable(struct rte_eth_dev *dev)
{
	struct ixgbe_hw *hw = IXGBE_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t fctrl;

	fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
	fctrl &= (~IXGBE_FCTRL_UPE);
	if (dev->data->all_multicast == 1)
		fctrl |= IXGBE_FCTRL_MPE;
	else
		fctrl &= (~IXGBE_FCTRL_MPE);
	IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);
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
	netdev->pci_dev = pdev;
	netdev->dev_ops = &ixgbe_eth_dev_ops;
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
