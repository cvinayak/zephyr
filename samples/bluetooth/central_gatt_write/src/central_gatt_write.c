/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

extern int mtu_exchange(struct bt_conn *conn);
extern int write_cmd(struct bt_conn *conn);
extern struct bt_conn *conn_connected;
extern uint32_t last_write_rate;
extern uint32_t *write_countdown;
extern void (*start_scan_func)(void);

#if defined(CONFIG_BT_EXT_ADV)
#define NAME_LEN 30

static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, NAME_LEN - 1);
		(void)memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case BT_GAP_LE_PHY_NONE: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

static void scan_recv(const struct bt_le_scan_recv_info *info,
		      struct net_buf_simple *buf)
{
	char le_addr[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN];
	uint8_t data_status;
	uint16_t data_len;

	(void)memset(name, 0, sizeof(name));

	data_len = buf->len;
	bt_data_parse(buf, data_cb, name);

	data_status = BT_HCI_LE_ADV_EVT_TYPE_DATA_STATUS(info->adv_props);

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	printk("[DEVICE]: %s, AD evt type %u, Tx Pwr: %i, RSSI %i "
	       "Data status: %u, AD data len: %u Name: %s "
	       "C:%u S:%u D:%u SR:%u E:%u Pri PHY: %s, Sec PHY: %s, "
	       "Interval: 0x%04x (%u ms), SID: %u\n",
	       le_addr, info->adv_type, info->tx_power, info->rssi,
	       data_status, data_len, name,
	       (info->adv_props & BT_GAP_ADV_PROP_CONNECTABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCANNABLE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_DIRECTED) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_SCAN_RESPONSE) != 0,
	       (info->adv_props & BT_GAP_ADV_PROP_EXT_ADV) != 0,
	       phy2str(info->primary_phy), phy2str(info->secondary_phy),
	       info->interval, info->interval * 5 / 4, info->sid);
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};
#endif /* CONFIG_BT_EXT_ADV */

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	struct bt_conn *conn;
	int err;

	printk("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
	       bt_addr_le_str(addr), type, ad->len, rssi);

	/* If already connected, do nothing */
	if (conn_connected) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	/* connect only to devices in close proximity */
	if (rssi < -50) {
		return;
	}

	err = bt_le_scan_stop();
	if (err) {
		printk("%s: Stop LE scan failed (err %d)\n", __func__, err);
		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &conn);
	if (err) {
		printk("%s: Create conn failed (err %d)\n", __func__, err);
		start_scan_func();
	} else {
		bt_conn_unref(conn);
	}
}

static void start_scan(void)
{
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = BT_LE_SCAN_OPT_CODED,
		.interval   = 0x0010,
		.window     = 0x0010,
	};
	int err;

scan_start_retry:
	printk("Starting scanning...\n");
	err = bt_le_scan_start(&scan_param, device_found);
	if (err) {
		if ((scan_param.options & BT_LE_SCAN_OPT_CODED) != 0U) {
			printk("Failed to start scanning with Coded PHY (err %d), retrying "
			       "without...\n", err);

			scan_param.options &= ~BT_LE_SCAN_OPT_CODED;

			goto scan_start_retry;
		}

		printk("Start scanning failed (err %d)\n", err);

		return;
	}

	printk("success.\n");
}

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};

uint32_t central_gatt_write(uint32_t count)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0U;
	}
	printk("Bluetooth initialized\n");

	bt_gatt_cb_register(&gatt_callbacks);

	conn_connected = NULL;
	last_write_rate = 0U;
	write_countdown = &count;

	if (count != 0U) {
		printk("GATT Write countdown %u on connection.\n", count);
	} else {
		printk("GATT Write forever on connection.\n");
	}

#if defined(CONFIG_BT_USER_PHY_UPDATE)
	err = bt_conn_le_set_default_phy(BT_GAP_LE_PHY_1M, BT_GAP_LE_PHY_1M);
	if (err) {
		printk("Failed to set default PHY (err %d)\n", err);
		return 0U;
	}
#endif /* CONFIG_BT_USER_PHY_UPDATE */

#if defined(CONFIG_BT_EXT_ADV)
	bt_le_scan_cb_register(&scan_callbacks);
	printk("Registered scan callbacks\n");
#endif /* CONFIG_BT_EXT_ADV */

	start_scan_func = start_scan;
	start_scan_func();

	while (true) {
		struct bt_conn *conn = NULL;

		if (conn_connected) {
			/* Get a connection reference to ensure that a
			 * reference is maintained in case disconnected
			 * callback is called while we perform GATT Write
			 * command.
			 */
			conn = bt_conn_ref(conn_connected);
		}

		if (conn) {
			(void)write_cmd(conn);
			bt_conn_unref(conn);

			/* Passing `0` will not use GATT Write Cmd countdown.
			 * Below code block will be optimized out by the linker.
			 */
			if (count) {
				if ((count % 1000U) == 0U) {
					printk("GATT Write countdown %u\n", count);
				}

				count--;
				if (!count) {
					break;
				}
			}

			k_yield();
		} else {
			k_sleep(K_SECONDS(1));
		}
	}

	return last_write_rate;
}
