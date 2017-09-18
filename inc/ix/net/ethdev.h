/**
 *   BSD LICENSE
 * 
 *   Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <linux/types.h>

#include <net/ethernet.h>

/*
#ifdef ENABLE_PCAP
#include <net/pcap.h>
#endif
*/

/* FIXME: figure out the right size for this */
#define RTE_ETHDEV_QUEUE_STAT_CNTRS	16

/**
 * A structure used to retrieve statistics for an Ethernet port.
 */
struct rte_eth_stats {
	uint64_t ipackets;  /**< Total number of successfully received packets. */
	uint64_t opackets;  /**< Total number of successfully transmitted packets.*/
	uint64_t ibytes;    /**< Total number of successfully received bytes. */
	uint64_t obytes;    /**< Total number of successfully transmitted bytes. */
	uint64_t ierrors;   /**< Total number of erroneous received packets. */
	uint64_t oerrors;   /**< Total number of failed transmitted packets. */
	uint64_t imcasts;   /**< Total number of multicast received packets. */
	uint64_t rx_nombuf; /**< Total number of RX mbuf allocation failures. */
	uint64_t fdirmatch; /**< Total number of RX packets matching a filter. */
	uint64_t fdirmiss;  /**< Total number of RX packets not matching any filter. */
	uint64_t tx_pause_xon;  /**< Total nb. of XON pause frame sent. */
	uint64_t rx_pause_xon;  /**< Total nb. of XON pause frame received. */
	uint64_t tx_pause_xoff; /**< Total nb. of XOFF pause frame sent. */
	uint64_t rx_pause_xoff; /**< Total nb. of XOFF pause frame received. */
	uint64_t q_ipackets[RTE_ETHDEV_QUEUE_STAT_CNTRS];
	/**< Total number of queue RX packets. */
	uint64_t q_opackets[RTE_ETHDEV_QUEUE_STAT_CNTRS];
	/**< Total number of queue TX packets. */
	uint64_t q_ibytes[RTE_ETHDEV_QUEUE_STAT_CNTRS];
	/**< Total number of successfully received queue bytes. */
	uint64_t q_obytes[RTE_ETHDEV_QUEUE_STAT_CNTRS];
	/**< Total number of successfully transmitted queue bytes. */
	uint64_t q_errors[RTE_ETHDEV_QUEUE_STAT_CNTRS];
	/**< Total number of queue packets received that are dropped. */
	uint64_t ilbpackets;
	/**< Total number of good packets received from loopback,VF Only */
	uint64_t olbpackets;
	/**< Total number of good packets transmitted to loopback,VF Only */
	uint64_t ilbbytes;
	/**< Total number of good bytes received from loopback,VF Only */
	uint64_t olbbytes;
	/**< Total number of good bytes transmitted to loopback,VF Only */
};

/**
 * A structure used to retrieve link-level information of an Ethernet port.
 */
struct rte_eth_link {
	uint16_t link_speed;      /**< ETH_LINK_SPEED_[10, 100, 1000, 10000] */
	uint16_t link_duplex;     /**< ETH_LINK_[HALF_DUPLEX, FULL_DUPLEX] */
	uint8_t  link_status : 1; /**< 1 -> link up, 0 -> link down */
}__attribute__((aligned(8)));     /**< aligned for atomic64 read/write */

#define ETH_LINK_SPEED_AUTONEG  0       /**< Auto-negotiate link speed. */
#define ETH_LINK_SPEED_10       10      /**< 10 megabits/second. */
#define ETH_LINK_SPEED_100      100     /**< 100 megabits/second. */
#define ETH_LINK_SPEED_1000     1000    /**< 1 gigabits/second. */
#define ETH_LINK_SPEED_10000    10000   /**< 10 gigabits/second. */

#define ETH_LINK_AUTONEG_DUPLEX 0       /**< Auto-negotiate duplex. */
#define ETH_LINK_HALF_DUPLEX    1       /**< Half-duplex connection. */
#define ETH_LINK_FULL_DUPLEX    2       /**< Full-duplex connection. */

/**
 * A structure used to configure the ring threshold registers of an RX/TX
 * queue for an Ethernet port.
 */
struct rte_eth_thresh {
	uint8_t pthresh; /**< Ring prefetch threshold. */
	uint8_t hthresh; /**< Ring host threshold. */
	uint8_t wthresh; /**< Ring writeback threshold. */
};

/**
 *  A set of values to identify what method is to be used to route
 *  packets to multiple queues.
 */
enum rte_eth_rx_mq_mode {
	ETH_MQ_RX_NONE = 0,  /**< None of DCB,RSS or VMDQ mode */

	ETH_MQ_RX_RSS,       /**< For RX side, only RSS is on */
	ETH_MQ_RX_DCB,       /**< For RX side,only DCB is on. */
	ETH_MQ_RX_DCB_RSS,   /**< Both DCB and RSS enable */

	ETH_MQ_RX_VMDQ_ONLY, /**< Only VMDQ, no RSS nor DCB */
	ETH_MQ_RX_VMDQ_RSS,  /**< RSS mode with VMDQ */
	ETH_MQ_RX_VMDQ_DCB,  /**< Use VMDQ+DCB to route traffic to queues */
	ETH_MQ_RX_VMDQ_DCB_RSS, /**< Enable both VMDQ and DCB in VMDq */
};

/**
 * for rx mq mode backward compatible 
 */
#define ETH_RSS                       ETH_MQ_RX_RSS
#define VMDQ_DCB                      ETH_MQ_RX_VMDQ_DCB
#define ETH_DCB_RX                    ETH_MQ_RX_DCB

/**
 * A set of values to identify what method is to be used to transmit 
 * packets using multi-TCs.
 */
enum rte_eth_tx_mq_mode {
	ETH_MQ_TX_NONE    = 0, 	/**< It is in neither DCB nor VT mode. */
	ETH_MQ_TX_DCB,         	/**< For TX side,only DCB is on. */
	ETH_MQ_TX_VMDQ_DCB,	/**< For TX side,both DCB and VT is on. */
	ETH_MQ_TX_VMDQ_ONLY,    /**< Only VT on, no DCB */
};

/**
 * for tx mq mode backward compatible 
 */
#define ETH_DCB_NONE                ETH_MQ_TX_NONE
#define ETH_VMDQ_DCB_TX             ETH_MQ_TX_VMDQ_DCB
#define ETH_DCB_TX                  ETH_MQ_TX_DCB

/**
 * A structure used to configure the RX features of an Ethernet port.
 */
struct rte_eth_rxmode {
	/** The multi-queue packet distribution mode to be used, e.g. RSS. */
	enum rte_eth_rx_mq_mode mq_mode;
	uint32_t max_rx_pkt_len;  /**< Only used if jumbo_frame enabled. */
	uint16_t split_hdr_size;  /**< hdr buf size (header_split enabled).*/
	uint8_t header_split : 1, /**< Header Split enable. */
		hw_ip_checksum   : 1, /**< IP/UDP/TCP checksum offload enable. */
		hw_vlan_filter   : 1, /**< VLAN filter enable. */
		hw_vlan_strip    : 1, /**< VLAN strip enable. */
		hw_vlan_extend   : 1, /**< Extended VLAN enable. */
		jumbo_frame      : 1, /**< Jumbo Frame Receipt enable. */
		hw_strip_crc     : 1; /**< Enable CRC stripping by hardware. */
};

/**
 * A structure used to configure the Receive Side Scaling (RSS) feature
 * of an Ethernet port.
 * If not NULL, the *rss_key* pointer of the *rss_conf* structure points
 * to an array of 40 bytes holding the RSS key to use for hashing specific
 * header fields of received packets.
 * Otherwise, a default random hash key is used by the device driver.
 *
 * The *rss_hf* field of the *rss_conf* structure indicates the different
 * types of IPv4/IPv6 packets to which the RSS hashing must be applied.
 * Supplying an *rss_hf* equal to zero disables the RSS feature.
 */
struct rte_eth_rss_conf {
	uint8_t  *rss_key;   /**< If not NULL, 40-byte hash key. */
	uint16_t rss_hf;     /**< Hash functions to apply - see below. */
};

#define ETH_RSS_IPV4        0x0001 /**< IPv4 packet. */
#define ETH_RSS_IPV4_TCP    0x0002 /**< IPv4/TCP packet. */
#define ETH_RSS_IPV6        0x0004 /**< IPv6 packet. */
#define ETH_RSS_IPV6_EX     0x0008 /**< IPv6 packet with extension headers.*/
#define ETH_RSS_IPV6_TCP    0x0010 /**< IPv6/TCP packet. */
#define ETH_RSS_IPV6_TCP_EX 0x0020 /**< IPv6/TCP with extension headers. */
/* Intel RSS extensions to UDP packets */
#define ETH_RSS_IPV4_UDP    0x0040 /**< IPv4/UDP packet. */
#define ETH_RSS_IPV6_UDP    0x0080 /**< IPv6/UDP packet. */
#define ETH_RSS_IPV6_UDP_EX 0x0100 /**< IPv6/UDP with extension headers. */
/* Definitions used for redirection table entry size */
#define ETH_RSS_RETA_NUM_ENTRIES 128
#define ETH_RSS_RETA_MAX_QUEUE   16  

/* Definitions used for VMDQ and DCB functionality */
#define ETH_VMDQ_MAX_VLAN_FILTERS   64 /**< Maximum nb. of VMDQ vlan filters. */
#define ETH_DCB_NUM_USER_PRIORITIES 8  /**< Maximum nb. of DCB priorities. */
#define ETH_VMDQ_DCB_NUM_QUEUES     128 /**< Maximum nb. of VMDQ DCB queues. */
#define ETH_DCB_NUM_QUEUES          128 /**< Maximum nb. of DCB queues. */

/* DCB capability defines */
#define ETH_DCB_PG_SUPPORT      0x00000001 /**< Priority Group(ETS) support. */
#define ETH_DCB_PFC_SUPPORT     0x00000002 /**< Priority Flow Control support. */ 

/* Definitions used for VLAN Offload functionality */
#define ETH_VLAN_STRIP_OFFLOAD   0x0001 /**< VLAN Strip  On/Off */
#define ETH_VLAN_FILTER_OFFLOAD  0x0002 /**< VLAN Filter On/Off */
#define ETH_VLAN_EXTEND_OFFLOAD  0x0004 /**< VLAN Extend On/Off */

/* Definitions used for mask VLAN setting */
#define ETH_VLAN_STRIP_MASK   0x0001 /**< VLAN Strip  setting mask */
#define ETH_VLAN_FILTER_MASK  0x0002 /**< VLAN Filter  setting mask*/
#define ETH_VLAN_EXTEND_MASK  0x0004 /**< VLAN Extend  setting mask*/
#define ETH_VLAN_ID_MAX       0x0FFF /**< VLAN ID is in lower 12 bits*/ 

/* Definitions used for receive MAC address   */
#define ETH_NUM_RECEIVE_MAC_ADDR  128 /**< Maximum nb. of receive mac addr. */


/* Definitions used for unicast hash  */
#define ETH_VMDQ_NUM_UC_HASH_ARRAY  128 /**< Maximum nb. of UC hash array. */

/* Definitions used for VMDQ pool rx mode setting */
#define ETH_VMDQ_ACCEPT_UNTAG   0x0001 /**< accept untagged packets. */
#define ETH_VMDQ_ACCEPT_HASH_MC 0x0002 /**< accept packets in multicast table . */
#define ETH_VMDQ_ACCEPT_HASH_UC 0x0004 /**< accept packets in unicast table. */
#define ETH_VMDQ_ACCEPT_BROADCAST   0x0008 /**< accept broadcast packets. */
#define ETH_VMDQ_ACCEPT_MULTICAST   0x0010 /**< multicast promiscuous. */

/* Definitions used for VMDQ mirror rules setting */
#define ETH_VMDQ_NUM_MIRROR_RULE     4 /**< Maximum nb. of mirror rules. . */

#define ETH_VMDQ_POOL_MIRROR    0x0001 /**< Virtual Pool Mirroring. */
#define ETH_VMDQ_UPLINK_MIRROR  0x0002 /**< Uplink Port Mirroring. */
#define ETH_VMDQ_DOWNLIN_MIRROR 0x0004 /**< Downlink Port Mirroring. */
#define ETH_VMDQ_VLAN_MIRROR    0x0008 /**< VLAN Mirroring. */

/**
 * A structure used to configure VLAN traffic mirror of an Ethernet port.
 */
struct rte_eth_vlan_mirror {
	uint64_t vlan_mask; /**< mask for valid VLAN ID. */
	uint16_t vlan_id[ETH_VMDQ_MAX_VLAN_FILTERS]; 
	/** VLAN ID list for vlan mirror. */
};

/**
 * A structure used to configure traffic mirror of an Ethernet port.
 */
struct rte_eth_vmdq_mirror_conf {
	uint8_t rule_type_mask; /**< Mirroring rule type mask we want to set */
	uint8_t dst_pool; /**< Destination pool for this mirror rule. */
	uint64_t pool_mask; /**< Bitmap of pool for pool mirroring */
	struct rte_eth_vlan_mirror vlan; /**< VLAN ID setting for VLAN mirroring */
};