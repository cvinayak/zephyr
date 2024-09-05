/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/settings/settings.h>

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s %u %s\n", addr, err, bt_hci_err_to_str(err));
		return;
	}

	printk("Connected %s\n", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected from %s, reason 0x%02x %s\n", addr,
	       reason, bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/** Print data as d_0 d_1 d_2 ... d_(n-2) d_(n-1) d_(n) to show the 3 first and 3 last octets
 *
 * Examples:
 * 01
 * 0102
 * 010203
 * 01020304
 * 0102030405
 * 010203040506
 * 010203...050607
 * 010203...060708
 * etc.
 */
static void iso_print_data(uint8_t *data, size_t data_len)
{
	/* Maximum number of octets from each end of the data */
	const uint8_t max_octets = 3;
	char data_str[35];
	size_t str_len;

	str_len = bin2hex(data, MIN(max_octets, data_len), data_str, sizeof(data_str));
	if (data_len > max_octets) {
		if (data_len > (max_octets * 2)) {
			static const char dots[] = "...";

			strcat(&data_str[str_len], dots);
			str_len += strlen(dots);
		}

		str_len += bin2hex(data + (data_len - MIN(max_octets, data_len - max_octets)),
				   MIN(max_octets, data_len - max_octets),
				   data_str + str_len,
				   sizeof(data_str) - str_len);
	}

	printk("\t %s\n", data_str);
}

static void iso_recv(struct bt_iso_chan *chan, const struct bt_iso_recv_info *info,
		struct net_buf *buf)
{
	if (info->flags & BT_ISO_FLAGS_VALID) {
		printk("Incoming data channel %p len %u ts %u\n", chan, buf->len, info->ts);
		iso_print_data(buf->data, buf->len);
	}
}

static struct k_work_delayable iso_send_work;
static uint16_t seq_num;

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);

	seq_num = 0U;

	/* Start send timer */
	k_work_schedule(&iso_send_work, K_MSEC(0));
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected (reason 0x%02x)\n", chan, reason);

	k_work_cancel_delayable(&iso_send_work);
}

static struct bt_iso_chan_ops iso_ops = {
	.recv		= iso_recv,
	.connected	= iso_connected,
	.disconnected	= iso_disconnected,
};

static struct bt_iso_chan_io_qos iso_tx = {
	.sdu = CONFIG_BT_ISO_TX_MTU,
	.phy = BT_GAP_LE_PHY_2M,
	.rtn = 0,
	.path = NULL,
};

static struct bt_iso_chan_io_qos iso_rx = {
	.sdu = CONFIG_BT_ISO_TX_MTU,
	.path = NULL,
};

static struct bt_iso_chan_qos iso_qos = {
	.tx = &iso_tx,
	.rx = &iso_rx,
};

static struct bt_iso_chan iso_chan = {
	.ops = &iso_ops,
	.qos = &iso_qos,
};

static int iso_accept(const struct bt_iso_accept_info *info,
		      struct bt_iso_chan **chan)
{
	printk("Incoming request from %p\n", (void *)info->acl);

	if (iso_chan.iso) {
		printk("No channels available\n");
		return -ENOMEM;
	}

	*chan = &iso_chan;

	return 0;
}

static struct bt_iso_server iso_server = {
#if defined(CONFIG_BT_SMP)
	.sec_level = BT_SECURITY_L1,
#endif /* CONFIG_BT_SMP */
	.accept = iso_accept,
};

static uint32_t interval_us = 10U * USEC_PER_MSEC; /* 10 ms */
NET_BUF_POOL_FIXED_DEFINE(tx_pool, 5, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);
/**
 * @brief Send ISO data on timeout
 *
 * This will send an increasing amount of ISO data, starting from 1 octet.
 *
 * First iteration : 0x00
 *
 * Second iteration: 0x00 0x01
 * Third iteration : 0x00 0x01 0x02
 *
 * And so on, until it wraps around the configured ISO TX MTU (CONFIG_BT_ISO_TX_MTU)
 *
 * @param work Pointer to the work structure
 */
static void iso_timer_timeout(struct k_work *work)
{
	int ret;
	static uint8_t buf_data[CONFIG_BT_ISO_TX_MTU];
	static bool data_initialized;
	struct net_buf *buf;
	static size_t len_to_send = 1;

	if (!data_initialized) {
		for (int i = 0; i < ARRAY_SIZE(buf_data); i++) {
			buf_data[i] = (uint8_t)i;
		}

		data_initialized = true;
	}

	buf = net_buf_alloc(&tx_pool, K_NO_WAIT);
	if (buf != NULL) {
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

		net_buf_add_mem(buf, buf_data, len_to_send);

		ret = bt_iso_chan_send(&iso_chan, buf, seq_num);

		if (ret < 0) {
			printk("Failed to send ISO data (%d)\n", ret);
			net_buf_unref(buf);
		}

		len_to_send++;
		if (len_to_send > ARRAY_SIZE(buf_data)) {
			len_to_send = 1;
		}
	} else {
		printk("Failed to allocate buffer, retrying in next interval (%u us)\n",
		       interval_us);
	}

	/* Sequence number shall be incremented for each SDU interval */
	seq_num++;

	k_work_schedule(&iso_send_work, K_USEC(interval_us));
}

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	printk("Bluetooth initialized\n");

	err = bt_iso_server_register(&iso_server);
	if (err) {
		printk("Unable to register ISO server (err %d)\n", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_ONE_TIME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	printk("Advertising successfully started\n");

	k_work_init_delayable(&iso_send_work, iso_timer_timeout);

	return 0;
}
