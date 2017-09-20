#include <ix/ethdev.h>

/**
 * eth_dev_alloc - allocates an ethernet device
 * @private_len: the size of the private area
 *
 * Returns an ethernet device, or NULL if failure.
 */
struct rte_eth_dev *eth_dev_alloc(size_t private_len)
{
	struct rte_eth_dev *dev;

	dev = malloc(sizeof(struct rte_eth_dev));
	if (!dev)
		return NULL;

	dev->pci_dev = NULL;
	dev->dev_ops = NULL;

	dev->data = malloc(sizeof(struct rte_eth_dev_data));
	if (!dev->data) {
		free(dev);
		return NULL;
	}

	memset(dev->data, 0, sizeof(struct rte_eth_dev_data));
	dev->data->dev_conf = default_conf;
	toeplitz_get_key(rss_key, sizeof(rss_key));

	dev->data->dev_private = malloc(private_len);
	if (!dev->data->dev_private) {
		free(dev->data);
		free(dev);
		return NULL;
	}

	memset(dev->data->dev_private, 0, private_len);

	return dev;
}