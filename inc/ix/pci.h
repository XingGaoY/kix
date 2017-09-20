#pragma once

#include <linux/types.h>
#include <linux/pci.h>

#define PCI_TLB_SIZ 10

typedef struct pci_addr{
	const char *addr;
}pci_addr_t;

// TODO not a good implementation
extern pci_addr_t pci_addr_tlb[];
extern int pci_ixgbe_registed_num;