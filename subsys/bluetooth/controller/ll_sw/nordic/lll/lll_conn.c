/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

static int _init_reset(void);

int lll_conn_init(void)
{
	int err;

	err = _init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int lll_conn_reset(void)
{
	int err;

	err = _init_reset();
	if (err) {
		return err;
	}

	return 0;
}

static int _init_reset(void)
{
	return 0;
}
