// copied from ix/ixgbe/ixgbe_ethdev.h
#pragma once

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/netdevice.h>

#include "ixgbe_type.h"
#include "ixgbe_dcb.h"
#include <ix/ethdev.h>

/* need update link, bit flag */
#define IXGBE_FLAG_NEED_LINK_UPDATE (uint32_t)(1 << 0)
#define IXGBE_FLAG_MAILBOX          (uint32_t)(1 << 1)

/*
 * Defines that were not part of ixgbe_type.h as they are not used by the
 * FreeBSD driver.
 */
#define IXGBE_ADVTXD_MAC_1588       0x00080000 /* IEEE1588 Timestamp packet */
#define IXGBE_RXD_STAT_TMST         0x10000    /* Timestamped Packet indication */
#define IXGBE_ADVTXD_TUCMD_L4T_RSV  0x00001800 /* L4 Packet TYPE, resvd  */
#define IXGBE_RXDADV_ERR_CKSUM_BIT  30
#define IXGBE_RXDADV_ERR_CKSUM_MSK  3
#define IXGBE_ADVTXD_MACLEN_SHIFT   9          /* Bit shift for l2_len */
#define IXGBE_NB_STAT_MAPPING_REGS  32
#define IXGBE_EXTENDED_VLAN	  (uint32_t)(1 << 26) /* EXTENDED VLAN ENABLE */
#define IXGBE_VFTA_SIZE 128
#define IXGBE_VLAN_TAG_SIZE 4
#define IXGBE_MAX_RX_QUEUE_NUM	128
#ifndef NBBY
#define NBBY	8	/* number of bits in a byte */
#endif
#define IXGBE_HWSTRIP_BITMAP_SIZE (IXGBE_MAX_RX_QUEUE_NUM / (sizeof(uint32_t) * NBBY))

/* Loopback operation modes */
/* 82599 specific loopback operation types */
#define IXGBE_LPBK_82599_NONE		0x0 /* Default value. Loopback is disabled. */
#define IXGBE_LPBK_82599_TX_RX		0x1 /* Tx->Rx loopback operation is enabled. */

#define MAX_RX_QUEUES	64		// TODO I don't know if it is the right value here, copied from linux/ixgbe
#define MAX_TX_QUEUES	64

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

/*
 * Information about the fdir mode.
 */
struct ixgbe_hw_fdir_info {
	uint16_t    collision;
	uint16_t    free;
	uint16_t    maxhash;
	uint8_t     maxlen;
	uint64_t    add;
	uint64_t    remove;
	uint64_t    f_add;
	uint64_t    f_remove;
};

/* structure for interrupt relative data */
struct ixgbe_interrupt {
	uint32_t flags;
	uint32_t mask;
};

struct ixgbe_stat_mapping_registers {
	uint32_t tqsm[IXGBE_NB_STAT_MAPPING_REGS];
	uint32_t rqsmr[IXGBE_NB_STAT_MAPPING_REGS];
};

struct ixgbe_vfta {
	uint32_t vfta[IXGBE_VFTA_SIZE];
};

struct ixgbe_hwstrip {
	uint32_t bitmap[IXGBE_HWSTRIP_BITMAP_SIZE];
};

/*
 * VF data which used by PF host only
 */
#define IXGBE_MAX_VF_MC_ENTRIES		30
#define IXGBE_MAX_MR_RULE_ENTRIES	4 /* number of mirroring rules supported */
#define IXGBE_MAX_UTA                   128	

struct ixgbe_mirror_info {
	struct rte_eth_vmdq_mirror_conf mr_conf[ETH_VMDQ_NUM_MIRROR_RULE]; 
	/**< store PF mirror rules configuration*/
};

struct ixgbe_vf_info {
	uint8_t vf_mac_addresses[ETH_ADDR_LEN];
	uint16_t vf_mc_hashes[IXGBE_MAX_VF_MC_ENTRIES];
	uint16_t num_vf_mc_hashes;
	uint16_t default_vf_vlan_id;
	uint16_t vlans_enabled;
	bool clear_to_send;
	uint16_t tx_rate;
	uint16_t vlan_count;
	uint8_t spoofchk_enabled;
};

struct ixgbe_uta_info {
	uint8_t  uc_filter_type;
	uint16_t uta_in_use;
	uint32_t uta_shadow[IXGBE_MAX_UTA];
};
/*
 * Structure to store private data for each driver instance (for each port).
 */
struct ixgbe_adapter {
	unsigned long state;

	struct ixgbe_hw             hw;
	struct ixgbe_hw_stats       stats;
	struct ixgbe_hw_fdir_info   fdir;
	struct ixgbe_interrupt      intr;
	struct ixgbe_stat_mapping_registers stat_mappings;
	struct ixgbe_vfta           shadow_vfta;
	struct ixgbe_hwstrip		hwstrip;
	struct ixgbe_dcb_config     dcb_config;
	struct ixgbe_mirror_info    mr_data;
	struct ixgbe_vf_info        *vfdata;
	struct ixgbe_uta_info       uta_info;
};

#define IXGBE_DEV_PRIVATE_TO_HW(adapter)\
	(&((struct ixgbe_adapter *)adapter)->hw)

#define IXGBE_DEV_PRIVATE_TO_STATS(adapter) \
	(&((struct ixgbe_adapter *)adapter)->stats)

#define IXGBE_DEV_PRIVATE_TO_INTR(adapter) \
	(&((struct ixgbe_adapter *)adapter)->intr)

#define IXGBE_DEV_PRIVATE_TO_FDIR_INFO(adapter) \
	(&((struct ixgbe_adapter *)adapter)->fdir)

#define IXGBE_DEV_PRIVATE_TO_STAT_MAPPINGS(adapter) \
	(&((struct ixgbe_adapter *)adapter)->stat_mappings)

#define IXGBE_DEV_PRIVATE_TO_VFTA(adapter) \
	(&((struct ixgbe_adapter *)adapter)->shadow_vfta)

#define IXGBE_DEV_PRIVATE_TO_HWSTRIP_BITMAP(adapter) \
	(&((struct ixgbe_adapter *)adapter)->hwstrip)

#define IXGBE_DEV_PRIVATE_TO_DCB_CFG(adapter) \
	(&((struct ixgbe_adapter *)adapter)->dcb_config)

#define IXGBE_DEV_PRIVATE_TO_P_VFDATA(adapter) \
	(&((struct ixgbe_adapter *)adapter)->vfdata)

#define IXGBE_DEV_PRIVATE_TO_PFDATA(adapter) \
	(&((struct ixgbe_adapter *)adapter)->mr_data)

#define IXGBE_DEV_PRIVATE_TO_UTA(adapter) \
	(&((struct ixgbe_adapter *)adapter)->uta_info)

/*
 * RX/TX function prototypes
 */
void ixgbe_dev_clear_queues(struct rte_eth_dev *dev);

void ixgbe_dev_rx_queue_release(struct eth_rx_queue *rxq);

void ixgbe_dev_tx_queue_release(struct eth_tx_queue *txq);

int  ixgbe_dev_rx_queue_setup(struct rte_eth_dev *dev, int rx_queue_id,
		int numa_node, uint16_t nb_rx_desc);

int  ixgbe_dev_rx_queue_init(struct rte_eth_dev *dev, int rx_queue_id);

int  ixgbe_dev_tx_queue_setup(struct rte_eth_dev *dev, int tx_queue_id,
		int numa_node, uint16_t nb_tx_desc);

int  ixgbe_dev_tx_queue_init(struct rte_eth_dev *dev, int tx_queue_id);

int ixgbe_dev_rx_init(struct rte_eth_dev *dev);

void ixgbe_dev_tx_init(struct rte_eth_dev *dev);

void ixgbe_dev_rxtx_start(struct rte_eth_dev *dev);

int ixgbevf_dev_rx_init(struct rte_eth_dev *dev);

void ixgbevf_dev_tx_init(struct rte_eth_dev *dev);

void ixgbevf_dev_rxtx_start(struct rte_eth_dev *dev);
/*
uint16_t ixgbe_recv_pkts(void *rx_queue, struct mbuf **rx_pkts,
		uint16_t nb_pkts);

#ifdef RTE_LIBRTE_IXGBE_RX_ALLOW_BULK_ALLOC
uint16_t ixgbe_recv_pkts_bulk_alloc(void *rx_queue, struct mbuf **rx_pkts,
		uint16_t nb_pkts);
#endif

uint16_t ixgbe_recv_scattered_pkts(void *rx_queue,
		struct mbuf **rx_pkts, uint16_t nb_pkts);

uint16_t ixgbe_xmit_pkts(void *tx_queue, struct mbuf **tx_pkts,
		uint16_t nb_pkts);

uint16_t ixgbe_xmit_pkts_simple(void *tx_queue, struct mbuf **tx_pkts,
		uint16_t nb_pkts);
*/
/*
 * Flow director function prototypes
 */
int ixgbe_fdir_configure(struct rte_eth_dev *dev);

int ixgbe_fdir_add_signature_filter(struct rte_eth_dev *dev,
		struct rte_fdir_filter *fdir_filter, uint8_t queue);

int ixgbe_fdir_update_signature_filter(struct rte_eth_dev *dev,
		struct rte_fdir_filter *fdir_filter, uint8_t queue);

int ixgbe_fdir_remove_signature_filter(struct rte_eth_dev *dev,
		struct rte_fdir_filter *fdir_filter);

void ixgbe_fdir_info_get(struct rte_eth_dev *dev,
		struct rte_eth_fdir *fdir);

int ixgbe_fdir_add_perfect_filter(struct rte_eth_dev *dev,
		struct rte_fdir_filter *fdir_filter, uint16_t soft_id,
		uint8_t queue, uint8_t drop);

int ixgbe_fdir_update_perfect_filter(struct rte_eth_dev *dev,
		struct rte_fdir_filter *fdir_filter,uint16_t soft_id,
		uint8_t queue, uint8_t drop);

int ixgbe_fdir_remove_perfect_filter(struct rte_eth_dev *dev,
		struct rte_fdir_filter *fdir_filter, uint16_t soft_id);

int ixgbe_fdir_set_masks(struct rte_eth_dev *dev,
		struct rte_fdir_masks *fdir_masks);

void ixgbe_configure_dcb(struct rte_eth_dev *dev);

/*
 * misc function prototypes
 */
void ixgbe_vlan_hw_filter_enable(struct rte_eth_dev *dev);

void ixgbe_vlan_hw_filter_disable(struct rte_eth_dev *dev);

void ixgbe_vlan_hw_strip_enable_all(struct rte_eth_dev *dev);

void ixgbe_vlan_hw_strip_disable_all(struct rte_eth_dev *dev);

void ixgbe_pf_host_init(struct rte_eth_dev *eth_dev);

void ixgbe_pf_mbx_process(struct rte_eth_dev *eth_dev);

int ixgbe_pf_host_configure(struct rte_eth_dev *eth_dev);

void ixgbe_irq_enable(struct rte_eth_dev *dev);
void ixgbe_irq_disable(struct ixgbe_hw *hw);

void netdev_init(struct rte_eth_dev *netdev, struct pci_dev *pdev);
