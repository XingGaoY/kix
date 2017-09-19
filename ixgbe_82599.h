#pragma once

#include "ixgbe_type.h"
// TODO should be declared in ixgbe.h as extern function
// And registered in 82599.c

int ixgbe_init_ops_82599(struct ixgbe_hw *hw);