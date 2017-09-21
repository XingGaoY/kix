#include <ix/ethdev.h>
#include <ix/toeplitz.h>

#include <linux/slab.h>

static uint8_t rss_key[40];

static const struct rte_eth_conf default_conf = {
        .rxmode = {
                .split_hdr_size = 0,
                .header_split   = 0, /**< Header Split disabled */
                .hw_ip_checksum = 1, /**< IP checksum offload disabled */
                .hw_vlan_filter = 0, /**< VLAN filtering disabled */
                .jumbo_frame    = 0, /**< Jumbo Frame Support disabled */
                .hw_strip_crc   = 1, /**< CRC stripped by hardware */
                .mq_mode        = ETH_MQ_RX_RSS,
        },
	.rx_adv_conf = {
		.rss_conf = {
			.rss_hf = ETH_RSS_IPV4_TCP | ETH_RSS_IPV4_UDP,
			.rss_key = rss_key,
		},
	},
        .txmode = {
                .mq_mode = ETH_MQ_TX_NONE,
        },
};

/**
 * eth_dev_alloc - allocates an ethernet device
 * @private_len: the size of the private area
 *
 * Returns an ethernet device, or NULL if failure.
 */
struct rte_eth_dev *eth_dev_alloc(size_t private_len)
{
	struct rte_eth_dev *dev;

	dev = kmalloc(sizeof(struct rte_eth_dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	dev->pci_dev = NULL;
	dev->dev_ops = NULL;

	dev->data = kmalloc(sizeof(struct rte_eth_dev_data), GFP_KERNEL);
	if (!dev->data) {
		kfree(dev);
		return NULL;
	}

	memset(dev->data, 0, sizeof(struct rte_eth_dev_data));
	dev->data->dev_conf = default_conf;
	toeplitz_get_key(rss_key, sizeof(rss_key));

	dev->data->dev_private = kmalloc(private_len, GFP_KERNEL);
	if (!dev->data->dev_private) {
		kfree(dev->data);
		kfree(dev);
		return NULL;
	}

	memset(dev->data->dev_private, 0, private_len);

	return dev;
}