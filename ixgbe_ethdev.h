// copied from ix/ixgbe/ixgbe_ethdev.h
#pragma once

#include <linux/types.h>

#include "ixgbe_type.h"

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

// put here temperarily
#define ETH_ADDR_LEN 6
#define ETH_VMDQ_NUM_MIRROR_RULE 4
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