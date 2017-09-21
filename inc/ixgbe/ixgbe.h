#ifndef _IXGBE_H_
#define _IXGBE_H_

#include <linux/pci.h>

enum ixgbe_state_t {
	__IXGBE_TESTING,
	__IXGBE_RESETTING,
	__IXGBE_DOWN,
	__IXGBE_DISABLED,
	__IXGBE_REMOVING,
	__IXGBE_SERVICE_SCHED,
	__IXGBE_SERVICE_INITED,
	__IXGBE_IN_SFP_INIT,
	__IXGBE_PTP_RUNNING,
	__IXGBE_PTP_TX_IN_PROGRESS,
	__IXGBE_RESET_REQUESTED,
};

enum ixgbe_boards {
	board_82599,
};

//extern const struct ixgbe_info ixgbe_82599_info;

int ixgbe_init(void);
void ixgbe_exit(void);

#endif /* _IXGBE_H_ */