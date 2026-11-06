/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef CHAN_MAP_H_
#define CHAN_MAP_H_

/**
 * @brief Start the channel metrics based channel map update.
 *
 * Call after bt_enable().
 *
 * @retval 0 Success
 * @retval -errno Registering the vendor-specific event callback failed
 */
int chan_map_init(void);

#endif /* CHAN_MAP_H_ */
