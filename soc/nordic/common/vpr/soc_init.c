/*
 * Copyright (C) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <hal/nrf_vpr_csr.h>
#include <hal/nrf_vpr_csr_vevif.h>

#if !defined(CONFIG_NORDIC_VPR_HW_STACKING)
/*
 * Interrupt hardware stacking control field of the VPRNORDICFEATURESDISABLE
 * CSR. It is not described by all MDK versions, provide its definition when
 * missing. Hardware stacking is enabled out of reset.
 */
#if !defined(VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_Pos)
#define VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_Pos (1UL)
#define VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_Msk \
	(0x3UL << VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_Pos)
#define VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_NOAUTOSTACK (0x0UL)
#endif /* !VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_Pos */

/*
 * Disable the hardware stacking of registers on interrupt entry, and the
 * corresponding unstacking on mret. The complete interrupted context is then
 * saved and restored by the kernel in software.
 *
 * Must be called before any interrupt is taken, i.e. while interrupts are
 * still globally disabled during the kernel initialization.
 */
static void vpr_hw_stacking_disable(void)
{
	unsigned long reg = csr_read(VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE);

	reg &= ~(VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_Msk |
		 VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_NORDICKEY_Msk);
	reg |= (VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_NOAUTOSTACK <<
		VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_Pos) |
	       (VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_NORDICKEY_Enabled <<
		VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_NORDICKEY_Pos);

	csr_write(VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE, reg);

	__ASSERT((csr_read(VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE) &
		  VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_Msk) ==
		 (VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_NOAUTOSTACK <<
		  VPRCSR_NORDIC_VPRNORDICFEATURESDISABLE_INTHWSTACKING_Pos),
		 "Failed to disable VPR interrupt hardware stacking");
}
#endif /* !CONFIG_NORDIC_VPR_HW_STACKING */

static int vpr_init(void)
{
	uint32_t sleep_mode;

#if !defined(CONFIG_NORDIC_VPR_HW_STACKING)
	vpr_hw_stacking_disable();
#endif /* !CONFIG_NORDIC_VPR_HW_STACKING */

	if (IS_ENABLED(CONFIG_NORDIC_VPR_DEEPSLEEP)) {
		sleep_mode = VPRCSR_NORDIC_VPRNORDICSLEEPCTRL_SLEEPSTATE_DEEPSLEEP;
	} else if (IS_ENABLED(CONFIG_NORDIC_VPR_HIBERNATE)) {
		sleep_mode = VPRCSR_NORDIC_VPRNORDICSLEEPCTRL_SLEEPSTATE_HIBERNATE;
	} else {
		sleep_mode = VPRCSR_NORDIC_VPRNORDICSLEEPCTRL_SLEEPSTATE_SLEEP;
	}

	csr_write(VPRCSR_NORDIC_VPRNORDICSLEEPCTRL, sleep_mode);

	/* RT peripherals for VPR all share one enable.
	 * To prevent redundant calls, do it here once.
	 */
	nrf_vpr_csr_rtperiph_enable_set(true);

#if DT_NODE_HAS_PROP(DT_NODELABEL(cpu), nordic_vpr_ready_event)
	/* Notify parent core that core is ready and can accept IPC communication. */
	nrf_vpr_csr_vevif_events_set(BIT(DT_PROP(DT_NODELABEL(cpu), nordic_vpr_ready_event)));
#endif
	return 0;
}

SYS_INIT(vpr_init, PRE_KERNEL_1, 0);
