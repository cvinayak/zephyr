/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include "bsim_args_runner.h"
#include "bstests.h"

/* Use a unique advertising SID per simulated device; device 0..N-1 are broadcasters */
uint8_t iso_broadcast_sid_get(void)
{
	return (uint8_t)get_device_nbr();
}

bst_test_install_t test_installers[] = {
	NULL
};
