#include "ixgbe_82599.h"
#include "ixgbe_phy.h"
#include "ixgbe_common.h"

s32 ixgbe_reset_pipeline_82599(struct ixgbe_hw *hw);
bool ixgbe_verify_lesm_fw_enabled_82599(struct ixgbe_hw *hw);

int ixgbe_init_ops_82599(struct ixgbe_hw *hw){
	int ret_val;

	printk(KERN_INFO "ixgbe_init_ops_82599");

	ret_val = ixgbe_init_phy_ops_generic(hw);

	return ret_val;
}

/**
 * ixgbe_reset_pipeline_82599 - perform pipeline reset
 *
 * @hw: pointer to hardware structure
 *
 * Reset pipeline by asserting Restart_AN together with LMS change to ensure
 * full pipeline reset.  Note - We must hold the SW/FW semaphore before writing
 * to AUTOC, so this function assumes the semaphore is held.
 **/
s32 ixgbe_reset_pipeline_82599(struct ixgbe_hw *hw)
{
	s32 ret_val;
	u32 anlp1_reg = 0;
	u32 i, autoc_reg, autoc2_reg;

	/* Enable link if disabled in NVM */
	autoc2_reg = IXGBE_READ_REG(hw, IXGBE_AUTOC2);
	if (autoc2_reg & IXGBE_AUTOC2_LINK_DISABLE_MASK) {
		autoc2_reg &= ~IXGBE_AUTOC2_LINK_DISABLE_MASK;
		IXGBE_WRITE_REG(hw, IXGBE_AUTOC2, autoc2_reg);
		IXGBE_WRITE_FLUSH(hw);
	}

	autoc_reg = IXGBE_READ_REG(hw, IXGBE_AUTOC);
	autoc_reg |= IXGBE_AUTOC_AN_RESTART;

	/* Write AUTOC register with toggled LMS[2] bit and Restart_AN */
	IXGBE_WRITE_REG(hw, IXGBE_AUTOC,
			autoc_reg ^ (0x4 << IXGBE_AUTOC_LMS_SHIFT));

	/* Wait for AN to leave state 0 */
	for (i = 0; i < 10; i++) {
		usleep_range(4000, 8000);
		anlp1_reg = IXGBE_READ_REG(hw, IXGBE_ANLP1);
		if (anlp1_reg & IXGBE_ANLP1_AN_STATE_MASK)
			break;
	}

	if (!(anlp1_reg & IXGBE_ANLP1_AN_STATE_MASK)) {
		hw_dbg(hw, "auto negotiation not completed\n");
		ret_val = IXGBE_ERR_RESET_FAILED;
		goto reset_pipeline_out;
	}

	ret_val = 0;

reset_pipeline_out:
	/* Write AUTOC register with original LMS field and Restart_AN */
	IXGBE_WRITE_REG(hw, IXGBE_AUTOC, autoc_reg);
	IXGBE_WRITE_FLUSH(hw);

	return ret_val;
}

/**
 *  ixgbe_verify_lesm_fw_enabled_82599 - Checks LESM FW module state.
 *  @hw: pointer to hardware structure
 *
 *  Returns true if the LESM FW module is present and enabled. Otherwise
 *  returns false. Smart Speed must be disabled if LESM FW module is enabled.
 **/
bool ixgbe_verify_lesm_fw_enabled_82599(struct ixgbe_hw *hw)
{
	bool lesm_enabled = false;
	u16 fw_offset, fw_lesm_param_offset, fw_lesm_state;
	s32 status;

	printk(KERN_INFO "ixgbe_verify_lesm_fw_enabled_82599");

	/* get the offset to the Firmware Module block */
	status = hw->eeprom.ops.read(hw, IXGBE_FW_PTR, &fw_offset);

	if ((status != IXGBE_SUCCESS) ||
	    (fw_offset == 0) || (fw_offset == 0xFFFF))
		goto out;

	/* get the offset to the LESM Parameters block */
	status = hw->eeprom.ops.read(hw, (fw_offset +
				     IXGBE_FW_LESM_PARAMETERS_PTR),
				     &fw_lesm_param_offset);

	if ((status != IXGBE_SUCCESS) ||
	    (fw_lesm_param_offset == 0) || (fw_lesm_param_offset == 0xFFFF))
		goto out;

	/* get the lesm state word */
	status = hw->eeprom.ops.read(hw, (fw_lesm_param_offset +
				     IXGBE_FW_LESM_STATE_1),
				     &fw_lesm_state);

	if ((status == IXGBE_SUCCESS) &&
	    (fw_lesm_state & IXGBE_FW_LESM_STATE_ENABLED))
		lesm_enabled = true;

out:
	return lesm_enabled;
}