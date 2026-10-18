/*
 * Copyright (c) 2021-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/assigned_numbers.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/audio/ascs.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/bap_lc3_preset.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/toolchain.h>
#include <zephyr/types.h>

#include "stream_tx.h"

#if defined(CONFIG_USE_USB_AUDIO)
#include <lc3.h>

#include "stream_lc3.h"
#include "usb.h"
#endif /* CONFIG_USE_USB_AUDIO */

static void start_scan(void);

uint64_t unicast_audio_recv_ctr; /* This value is exposed to test code */

static struct bt_bap_unicast_client_cb unicast_client_cbs;
static struct bt_conn *default_conn;
static struct bt_bap_unicast_group *unicast_group;
static struct audio_sink {
	struct bt_bap_ep *ep;
	uint16_t seq_num;
} sinks[CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT];
static struct bt_bap_ep *sources[CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT];

static struct bt_bap_stream streams[CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SNK_COUNT +
				      CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT];
static size_t configured_sink_stream_count;
static size_t configured_source_stream_count;

#define configured_stream_count (configured_sink_stream_count + \
				 configured_source_stream_count)

/* Select codec configurations to apply that are mandatory to support by both client and server.
 * Allows this sample application to work without logic to parse the codec capabilities of the
 * server and selection of an appropriate codec configuration.
 */
#if defined(CONFIG_SINK_PRESET_48_2_1)
#define SINK_PRESET BT_BAP_LC3_UNICAST_PRESET_48_2_1
#elif defined(CONFIG_SINK_PRESET_24_2_1)
#define SINK_PRESET BT_BAP_LC3_UNICAST_PRESET_24_2_1
#else
#define SINK_PRESET BT_BAP_LC3_UNICAST_PRESET_16_2_1
#endif

#if defined(CONFIG_SOURCE_PRESET_24_2_1)
#define SOURCE_PRESET BT_BAP_LC3_UNICAST_PRESET_24_2_1
#else
#define SOURCE_PRESET BT_BAP_LC3_UNICAST_PRESET_16_2_1
#endif

#if defined(CONFIG_USE_USB_AUDIO)
#define SINK_CONTEXT   BT_AUDIO_CONTEXT_TYPE_MEDIA
#define SOURCE_CONTEXT BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL
#else
#define SINK_CONTEXT   BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED
#define SOURCE_CONTEXT BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED
#endif

/* Sink streams alternate between the left and right channel */
static const struct bt_bap_lc3_preset sink_preset_left =
	SINK_PRESET(BT_AUDIO_LOCATION_FRONT_LEFT, SINK_CONTEXT);
static const struct bt_bap_lc3_preset sink_preset_right =
	SINK_PRESET(BT_AUDIO_LOCATION_FRONT_RIGHT, SINK_CONTEXT);
static const struct bt_bap_lc3_preset source_preset =
	SOURCE_PRESET(BT_AUDIO_LOCATION_FRONT_LEFT, SOURCE_CONTEXT);

/* Codec configuration of each stream, the stream keeps a reference to it */
static struct bt_bap_lc3_preset stream_presets[ARRAY_SIZE(streams)];

static K_SEM_DEFINE(sem_connected, 0U, 1U);
static K_SEM_DEFINE(sem_disconnected, 0U, 1U);
static K_SEM_DEFINE(sem_mtu_exchanged, 0U, 1U);
static K_SEM_DEFINE(sem_security_updated, 0U, 1U);
static K_SEM_DEFINE(sem_sinks_discovered, 0U, 1U);
static K_SEM_DEFINE(sem_sources_discovered, 0U, 1U);
static K_SEM_DEFINE(sem_stream_configured, 0U, 1U);
static K_SEM_DEFINE(sem_stream_qos, 0U, ARRAY_SIZE(sinks) + ARRAY_SIZE(sources));
static K_SEM_DEFINE(sem_stream_enabled, 0U, 1U);
static K_SEM_DEFINE(sem_stream_started, 0U, 1U);
static K_SEM_DEFINE(sem_stream_connected, 0U, 1U);

#if defined(CONFIG_USE_USB_AUDIO)
#define MIC_SDU_LEN_MAX     155U
#define MIC_PCM_SAMPLES_MAX (USB_SAMPLE_RATE_HZ / 100U)
#define MIC_SDU_COUNT       4U
#define MIC_STACK_SIZE      4096U
#define MIC_PRIO            K_PRIO_PREEMPT(5)

struct mic_sdu {
	uint32_t freq_hz;
	uint32_t frame_dur_us;
	uint16_t octets_per_frame;
	uint16_t len;
	uint8_t data[MIC_SDU_LEN_MAX];
};

K_MSGQ_DEFINE(mic_sdu_q, sizeof(struct mic_sdu), MIC_SDU_COUNT, 4);
static struct bt_bap_stream *mic_stream;
static struct mic_sdu mic_cfg;
static atomic_t mic_sdu_dropped;

/* Use the first source stream as the USB microphone */
static void mic_stream_start(struct bt_bap_stream *stream)
{
	const struct bt_audio_codec_cfg *codec_cfg = stream->codec_cfg;
	int ret;

	if (mic_stream != NULL) {
		return;
	}

	ret = bt_audio_codec_cfg_get_freq(codec_cfg);
	if (ret < 0) {
		printk("Could not get source stream frequency: %d\n", ret);
		return;
	}

	mic_cfg.freq_hz = (uint32_t)bt_audio_codec_cfg_freq_to_freq_hz(ret);

	ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
	if (ret < 0) {
		printk("Could not get source stream frame duration: %d\n", ret);
		return;
	}

	mic_cfg.frame_dur_us = (uint32_t)bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);

	ret = bt_audio_codec_cfg_get_octets_per_frame(codec_cfg);
	if (ret <= 0 || ret > MIC_SDU_LEN_MAX) {
		printk("Unsupported source stream octets per frame: %d\n", ret);
		return;
	}

	mic_cfg.octets_per_frame = (uint16_t)ret;
	mic_stream = stream;

	printk("Source stream %p to USB microphone: %u Hz, %u us, %u octets\n", stream,
	       mic_cfg.freq_hz, mic_cfg.frame_dur_us, mic_cfg.octets_per_frame);
}

static void mic_stream_stop(struct bt_bap_stream *stream)
{
	if (mic_stream == stream) {
		mic_stream = NULL;
		k_msgq_purge(&mic_sdu_q);
	}
}

static void mic_sdu_recv(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info,
			 const struct net_buf *buf)
{
	struct mic_sdu sdu;

	if (stream != mic_stream) {
		return;
	}

	sdu = mic_cfg;
	sdu.len = 0U;

	/* An empty SDU makes the decoder conceal the lost frame */
	if ((info->flags & BT_ISO_FLAGS_VALID) != 0U && buf->len == sdu.octets_per_frame) {
		sdu.len = buf->len;
		(void)memcpy(sdu.data, buf->data, buf->len);
	}

	if (k_msgq_put(&mic_sdu_q, &sdu, K_NO_WAIT) != 0) {
		atomic_inc(&mic_sdu_dropped);
	}
}

/* Decode the source stream to 48 kHz and send it to the USB microphone */
static void mic_thread_func(void *p1, void *p2, void *p3)
{
	static lc3_decoder_mem_48k_t decoder_mem;
	static int16_t pcm[MIC_PCM_SAMPLES_MAX];
	static struct mic_sdu sdu;
	lc3_decoder_t decoder = NULL;
	uint32_t freq_hz = 0U;
	uint32_t frame_dur_us = 0U;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		size_t samples;
		int err;

		(void)k_msgq_get(&mic_sdu_q, &sdu, K_FOREVER);

		if (decoder == NULL || sdu.freq_hz != freq_hz || sdu.frame_dur_us != frame_dur_us) {
			decoder = lc3_setup_decoder(sdu.frame_dur_us, sdu.freq_hz,
						    USB_SAMPLE_RATE_HZ, &decoder_mem);
			if (decoder == NULL) {
				printk("Failed to setup LC3 decoder for %u Hz %u us\n",
				       sdu.freq_hz, sdu.frame_dur_us);
				continue;
			}

			freq_hz = sdu.freq_hz;
			frame_dur_us = sdu.frame_dur_us;
		}

		samples = ((size_t)frame_dur_us * USB_SAMPLE_RATE_HZ) / USEC_PER_SEC;
		if (samples > ARRAY_SIZE(pcm)) {
			continue;
		}

		err = lc3_decode(decoder, sdu.len > 0U ? sdu.data : NULL, sdu.len,
				 LC3_PCM_FORMAT_S16, pcm, 1);
		if (err < 0) {
			printk("LC3 decode failed: %d\n", err);
			continue;
		}

		usb_audio_pcm_put(pcm, samples, 1U);
	}
}

K_THREAD_DEFINE(mic_thread, MIC_STACK_SIZE, mic_thread_func, NULL, NULL, NULL, MIC_PRIO, 0, 0);
#endif /* CONFIG_USE_USB_AUDIO */

static void print_hex(const uint8_t *ptr, size_t len)
{
	while (len != 0U) {
		printk("%02x", *ptr);
		ptr++;
		len--;
	}
}

static bool print_cb(struct bt_data *data, void *user_data)
{
	const char *str = (const char *)user_data;

	printk("%s: type 0x%02x value_len %u\n", str, data->type, data->data_len);
	print_hex(data->data, data->data_len);
	printk("\n");

	return true;
}

static void print_codec_cap(const struct bt_audio_codec_cap *codec_cap)
{
	printk("codec id 0x%02x cid 0x%04x vid 0x%04x count %u\n", codec_cap->id, codec_cap->cid,
	       codec_cap->vid, codec_cap->data_len);

	if (codec_cap->id == BT_HCI_CODING_FORMAT_LC3) {
		bt_audio_data_parse(codec_cap->data, codec_cap->data_len, print_cb, "data");
	} else { /* If not LC3, we cannot assume it's LTV */
		printk("data: ");
		print_hex(codec_cap->data, codec_cap->data_len);
		printk("\n");
	}

	bt_audio_data_parse(codec_cap->meta, codec_cap->meta_len, print_cb, "meta");
}

static bool check_audio_support_and_connect(struct bt_data *data,
					    void *user_data)
{
	struct net_buf_simple ascs_svc_data;
	bt_addr_le_t *addr = user_data;
	uint8_t announcement_type;
	uint32_t audio_contexts;
	const struct bt_uuid *uuid;
	uint16_t uuid_val;
	uint8_t meta_len;
	size_t min_size;
	int err;

	printk("[AD]: %u data_len %u\n", data->type, data->data_len);

	if (data->type != BT_DATA_SVC_DATA16) {
		return true; /* Continue parsing to next AD data type */
	}

	if (data->data_len < sizeof(uuid_val)) {
		printk("AD invalid size %u\n", data->data_len);
		return true; /* Continue parsing to next AD data type */
	}

	net_buf_simple_init_with_data(&ascs_svc_data, (void *)data->data,
				      data->data_len);

	uuid_val = net_buf_simple_pull_le16(&ascs_svc_data);
	uuid = BT_UUID_DECLARE_16(sys_le16_to_cpu(uuid_val));
	if (bt_uuid_cmp(uuid, BT_UUID_ASCS) != 0) {
		/* We are looking for the ASCS service data */
		return true; /* Continue parsing to next AD data type */
	}

	min_size = sizeof(announcement_type) + sizeof(audio_contexts) + sizeof(meta_len);
	if (ascs_svc_data.len < min_size) {
		printk("AD invalid size %u\n", data->data_len);
		return false; /* Stop parsing */
	}

	announcement_type = net_buf_simple_pull_u8(&ascs_svc_data);
	audio_contexts = net_buf_simple_pull_le32(&ascs_svc_data);
	meta_len = net_buf_simple_pull_u8(&ascs_svc_data);

	err = bt_le_scan_stop();
	if (err != 0) {
		printk("Failed to stop scan: %d\n", err);
		return false; /* Stop parsing */
	}

	printk("Audio server found with type %u, contexts 0x%08x and meta_len %u; connecting\n",
	       announcement_type, audio_contexts, meta_len);

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_BAP_CONN_PARAM_RELAXED,
				&default_conn);
	if (err != 0) {
		printk("Create conn to failed (%u)\n", err);
		start_scan();
	}

	return false; /* Stop parsing */
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	if (default_conn != NULL) {
		/* Already connected */
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
	    type != BT_GAP_ADV_TYPE_EXT_ADV) {
		return;
	}

	printk("Device found: %s (RSSI %d)\n", bt_addr_le_str(addr), rssi);

	/* connect only to devices in close proximity */
	if (rssi < -50) {
		return;
	}

	bt_data_parse(ad, check_audio_support_and_connect, (void *)addr);
}

static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err != 0) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void stream_codec_configured(struct bt_bap_stream *stream,
				    const struct bt_bap_qos_cfg_pref *pref)
{
	ARG_UNUSED(pref);

	printk("Audio Stream %p configured\n", stream);

	k_sem_give(&sem_stream_configured);
}

static void stream_qos_configured(struct bt_bap_stream *stream)
{
	struct bt_iso_info info;
	int err;

	err = bt_iso_chan_get_info(stream->iso, &info);
	__ASSERT(err == 0, "Failed to get ISO chan info: %d", err);

	printk("Audio Stream %p QoS set with CIG_ID %u and CIS_ID %u\n", stream,
	       info.unicast.cig_id, info.unicast.cis_id);

	k_sem_give(&sem_stream_qos);
}

static void stream_enabled(struct bt_bap_stream *stream)
{
	printk("Audio Stream %p enabled\n", stream);

	k_sem_give(&sem_stream_enabled);
}

static bool stream_tx_can_send(const struct bt_bap_stream *stream)
{
	struct bt_bap_ep_info info;
	int err;

	if (stream == NULL || stream->ep == NULL) {
		return false;
	}

	err = bt_bap_ep_get_info(stream->ep, &info);
	if (err != 0) {
		return false;
	}

	return info.can_send;
}

static void stream_connected_cb(struct bt_bap_stream *stream)
{
	printk("Audio Stream %p connected\n", stream);

	/* Reset sequence number for sinks */
	for (size_t i = 0U; i < configured_sink_stream_count; i++) {
		if (stream->ep == sinks[i].ep) {
			sinks[i].seq_num = 0U;
			break;
		}
	}

	k_sem_give(&sem_stream_connected);
}

static void stream_started(struct bt_bap_stream *stream)
{
	printk("Audio Stream %p started\n", stream);
	unicast_audio_recv_ctr = 0U;

	/* Register the stream for TX if it can send */
	if (IS_ENABLED(CONFIG_BT_AUDIO_TX) && stream_tx_can_send(stream)) {
		const int err = stream_tx_register(stream);

		if (err != 0) {
			printk("Failed to register stream %p for TX: %d\n", stream, err);
		}
	}

#if defined(CONFIG_USE_USB_AUDIO)
	struct bt_bap_ep_info ep_info;

	if (bt_bap_ep_get_info(stream->ep, &ep_info) == 0 && ep_info.dir == BT_AUDIO_DIR_SOURCE) {
		mic_stream_start(stream);
	}
#endif /* CONFIG_USE_USB_AUDIO */

	k_sem_give(&sem_stream_started);
}

static void stream_metadata_updated(struct bt_bap_stream *stream)
{
	printk("Audio Stream %p metadata updated\n", stream);
}

static void stream_disabled(struct bt_bap_stream *stream)
{
	printk("Audio Stream %p disabled\n", stream);
}

static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
	printk("Audio Stream %p stopped with reason 0x%02X\n", stream, reason);

	/* Unregister the stream for TX if it can send */
	if (IS_ENABLED(CONFIG_BT_AUDIO_TX) && stream_tx_can_send(stream)) {
		const int err = stream_tx_unregister(stream);

		if (err != 0) {
			printk("Failed to unregister stream %p for TX: %d", stream, err);
		}
	}

#if defined(CONFIG_USE_USB_AUDIO)
	mic_stream_stop(stream);
#endif /* CONFIG_USE_USB_AUDIO */
}

static void stream_released(struct bt_bap_stream *stream)
{
	printk("Audio Stream %p released\n", stream);
}

static void stream_recv(struct bt_bap_stream *stream,
			const struct bt_iso_recv_info *info,
			struct net_buf *buf)
{
	if (info->flags & BT_ISO_FLAGS_VALID) {
		unicast_audio_recv_ctr++;

		if (CONFIG_INFO_REPORTING_INTERVAL > 0 &&
		    (unicast_audio_recv_ctr % CONFIG_INFO_REPORTING_INTERVAL) == 0U) {
			printk("Incoming audio on stream %p len %u (%" PRIu64 ")\n", stream,
			       buf->len, unicast_audio_recv_ctr);
#if defined(CONFIG_USE_USB_AUDIO)
			printk("Microphone SDUs dropped: %ld\n", atomic_get(&mic_sdu_dropped));
			usb_audio_stats_log();
#endif /* CONFIG_USE_USB_AUDIO */
		}
	}

#if defined(CONFIG_USE_USB_AUDIO)
	mic_sdu_recv(stream, info, buf);
#endif /* CONFIG_USE_USB_AUDIO */
}

static struct bt_bap_stream_ops stream_ops = {
	.codec_configured = stream_codec_configured,
	.qos_configured = stream_qos_configured,
	.enabled = stream_enabled,
	.started = stream_started,
	.metadata_updated = stream_metadata_updated,
	.disabled = stream_disabled,
	.stopped = stream_stopped,
	.released = stream_released,
	.recv = stream_recv,
	.connected = stream_connected_cb,
};

static void add_remote_source(struct bt_bap_ep *ep)
{
	for (size_t i = 0U; i < ARRAY_SIZE(sources); i++) {
		if (sources[i] == NULL) {
			printk("Source #%zu: ep %p\n", i, ep);
			sources[i] = ep;
			return;
		}
	}

	printk("Could not add source ep\n");
}

static void add_remote_sink(struct bt_bap_ep *ep)
{
	for (size_t i = 0U; i < ARRAY_SIZE(sinks); i++) {
		if (sinks[i].ep == NULL) {
			printk("Sink #%zu: ep %p\n", i, ep);
			sinks[i].ep = ep;
			return;
		}
	}

	printk("Could not add sink ep\n");
}

static void print_remote_codec_cap(const struct bt_audio_codec_cap *codec_cap,
				   enum bt_audio_dir dir)
{
	printk("codec_cap %p dir 0x%02x\n", codec_cap, dir);

	print_codec_cap(codec_cap);
}

static void discover_sinks_cb(struct bt_conn *conn, int err, enum bt_audio_dir dir)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(dir);

	if (err != 0 && err != BT_ATT_ERR_ATTRIBUTE_NOT_FOUND) {
		printk("Discovery failed: %d\n", err);
		return;
	}

	if (err == BT_ATT_ERR_ATTRIBUTE_NOT_FOUND) {
		printk("Discover sinks completed without finding any sink ASEs\n");
	} else {
		printk("Discover sinks complete: err %d\n", err);
	}

	k_sem_give(&sem_sinks_discovered);
}

static void discover_sources_cb(struct bt_conn *conn, int err, enum bt_audio_dir dir)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(dir);

	if (err != 0 && err != BT_ATT_ERR_ATTRIBUTE_NOT_FOUND) {
		printk("Discovery failed: %d\n", err);
		return;
	}

	if (err == BT_ATT_ERR_ATTRIBUTE_NOT_FOUND) {
		printk("Discover sinks completed without finding any source ASEs\n");
	} else {
		printk("Discover sources complete: err %d\n", err);
	}

	k_sem_give(&sem_sources_discovered);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0) {
		printk("Failed to connect to %s %u %s\n", bt_conn_dst_str(conn),
		       err, bt_hci_err_to_str(err));

		bt_conn_drop(&default_conn);

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", bt_conn_dst_str(conn));
	k_sem_give(&sem_connected);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != default_conn) {
		return;
	}

	printk("Disconnected: %s, reason 0x%02x %s\n", bt_conn_dst_str(conn),
	       reason, bt_hci_err_to_str(reason));

	bt_conn_drop(&default_conn);

	k_sem_give(&sem_disconnected);
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
				enum bt_security_err err)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(level);

	if (err == 0) {
		k_sem_give(&sem_security_updated);
	} else {
		printk("Failed to set security level: %s(%u)", bt_security_err_to_str(err), err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed_cb
};

static void att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	ARG_UNUSED(conn);

	printk("MTU exchanged: %u/%u\n", tx, rx);
	k_sem_give(&sem_mtu_exchanged);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = att_mtu_updated,
};

static void unicast_client_location_cb(struct bt_conn *conn,
				      enum bt_audio_dir dir,
				      enum bt_audio_location loc)
{
	ARG_UNUSED(conn);

	printk("dir %u loc %X\n", dir, loc);
}

static void supported_contexts_cb(struct bt_conn *conn, enum bt_audio_context snk_ctx,
				  enum bt_audio_context src_ctx)
{
	ARG_UNUSED(conn);

	printk("Supported snk ctx %u src ctx %u\n", snk_ctx, src_ctx);
}

static void available_contexts_cb(struct bt_conn *conn,
				  enum bt_audio_context snk_ctx,
				  enum bt_audio_context src_ctx)
{
	ARG_UNUSED(conn);

	printk("Available snk ctx %u src ctx %u\n", snk_ctx, src_ctx);
}

static void pac_record_cb(struct bt_conn *conn, enum bt_audio_dir dir,
			  const struct bt_audio_codec_cap *codec_cap)
{
	ARG_UNUSED(conn);

	print_remote_codec_cap(codec_cap, dir);
}

static void endpoint_cb(struct bt_conn *conn, enum bt_audio_dir dir, struct bt_bap_ep *ep)
{
	ARG_UNUSED(conn);

	if (dir == BT_AUDIO_DIR_SOURCE) {
		add_remote_source(ep);
	} else if (dir == BT_AUDIO_DIR_SINK) {
		add_remote_sink(ep);
	}
}

static struct bt_bap_unicast_client_cb unicast_client_cbs = {
	.location = unicast_client_location_cb,
	.supported_contexts = supported_contexts_cb,
	.available_contexts = available_contexts_cb,
	.pac_record = pac_record_cb,
	.endpoint = endpoint_cb,
};

static int init(void)
{
	int err;

	err = bt_enable(NULL);
	if (err != 0) {
		printk("Bluetooth enable failed (err %d)\n", err);
		return err;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(streams); i++) {
		streams[i].ops = &stream_ops;
	}

	bt_gatt_cb_register(&gatt_callbacks);

	if (IS_ENABLED(CONFIG_BT_AUDIO_TX)) {
		stream_tx_init();
	}

#if defined(CONFIG_USE_USB_AUDIO)
	err = usb_audio_init();
	if (err != 0) {
		printk("USB audio init failed (err %d)\n", err);
		return err;
	}

	stream_lc3_pcm_source_set(usb_audio_pcm_get);
#endif /* CONFIG_USE_USB_AUDIO */

	return 0;
}

static int scan_and_connect(void)
{
	int err;

	start_scan();

	err = k_sem_take(&sem_connected, K_FOREVER);
	if (err != 0) {
		printk("failed to take sem_connected (err %d)\n", err);
		return err;
	}

	err = k_sem_take(&sem_mtu_exchanged, K_FOREVER);
	if (err != 0) {
		printk("failed to take sem_mtu_exchanged (err %d)\n", err);
		return err;
	}

	err = bt_conn_set_security(default_conn, BT_SECURITY_L2);
	if (err != 0) {
		printk("failed to set security (err %d)\n", err);
		return err;
	}

	err = k_sem_take(&sem_security_updated, K_FOREVER);
	if (err != 0) {
		printk("failed to take sem_security_updated (err %d)\n", err);
		return err;
	}

	return 0;
}

static int discover_sinks(void)
{
	int err;

	unicast_client_cbs.discover = discover_sinks_cb;

	err = bt_bap_unicast_client_discover(default_conn, BT_AUDIO_DIR_SINK);
	if (err != 0) {
		printk("Failed to discover sinks: %d\n", err);
		return err;
	}

	err = k_sem_take(&sem_sinks_discovered, K_FOREVER);
	if (err != 0) {
		printk("failed to take sem_sinks_discovered (err %d)\n", err);
		return err;
	}

	return 0;
}

static int discover_sources(void)
{
	int err;

	unicast_client_cbs.discover = discover_sources_cb;

	err = bt_bap_unicast_client_discover(default_conn, BT_AUDIO_DIR_SOURCE);
	if (err != 0) {
		printk("Failed to discover sources: %d\n", err);
		return err;
	}

	err = k_sem_take(&sem_sources_discovered, K_FOREVER);
	if (err != 0) {
		printk("failed to take sem_sources_discovered (err %d)\n", err);
		return err;
	}

	return 0;
}

static int configure_stream(struct bt_bap_stream *stream, struct bt_bap_ep *ep,
			    const struct bt_bap_lc3_preset *preset)
{
	struct bt_bap_lc3_preset *stream_preset = &stream_presets[ARRAY_INDEX(streams, stream)];
	int err;

	*stream_preset = *preset;

	err = bt_bap_stream_config(default_conn, stream, ep, &stream_preset->codec_cfg);
	if (err != 0) {
		return err;
	}

	err = k_sem_take(&sem_stream_configured, K_FOREVER);
	if (err != 0) {
		printk("failed to take sem_stream_configured (err %d)\n", err);
		return err;
	}

	return 0;
}

static int configure_streams(void)
{
	int err;

	for (size_t i = 0U; i < ARRAY_SIZE(sinks); i++) {
		struct bt_bap_ep *ep = sinks[i].ep;
		struct bt_bap_stream *stream = &streams[i];
		const struct bt_bap_lc3_preset *preset;

		if (ep == NULL) {
			continue;
		}

		preset = (configured_sink_stream_count % 2U) == 0U ? &sink_preset_left
								   : &sink_preset_right;
		err = configure_stream(stream, ep, preset);
		if (err != 0) {
			printk("Could not configure sink stream[%zu]: %d\n",
			       i, err);
			return err;
		}

		printk("Configured sink stream[%zu]\n", i);
		configured_sink_stream_count++;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(sources); i++) {
		struct bt_bap_ep *ep = sources[i];
		struct bt_bap_stream *stream = &streams[i + configured_sink_stream_count];

		if (ep == NULL) {
			continue;
		}

		err = configure_stream(stream, ep, &source_preset);
		if (err != 0) {
			printk("Could not configure source stream[%zu]: %d\n",
			       i, err);
			return err;
		}

		printk("Configured source stream[%zu]\n", i);
		configured_source_stream_count++;
	}

	return 0;
}

static int create_group(void)
{
	const size_t params_count = MAX(configured_sink_stream_count,
					configured_source_stream_count);
	struct bt_bap_unicast_group_stream_pair_param pair_params[params_count];
	struct bt_bap_unicast_group_stream_param stream_params[configured_stream_count];
	struct bt_bap_unicast_group_param param;
	int err;

	for (size_t i = 0U; i < configured_stream_count; i++) {
		stream_params[i].stream = &streams[i];
		stream_params[i].qos = &stream_presets[i].qos;
	}

	for (size_t i = 0U; i < params_count; i++) {
		if (i < configured_sink_stream_count) {
			pair_params[i].tx_param = &stream_params[i];
		} else {
			pair_params[i].tx_param = NULL;
		}

		if (i < configured_source_stream_count) {
			pair_params[i].rx_param = &stream_params[i + configured_sink_stream_count];
		} else {
			pair_params[i].rx_param = NULL;
		}
	}

	param.params = pair_params;
	param.params_count = params_count;
	param.packing = (IS_ENABLED(CONFIG_ISO_PACKING_INTERLEAVED) ?
			 BT_ISO_PACKING_INTERLEAVED :
			 BT_ISO_PACKING_SEQUENTIAL);

	err = bt_bap_unicast_group_create(&param, &unicast_group);
	if (err != 0) {
		printk("Could not create unicast group (err %d)\n", err);
		return err;
	}

	return 0;
}

static int delete_group(void)
{
	int err;

	err = bt_bap_unicast_group_delete(unicast_group);
	if (err != 0) {
		printk("Could not create unicast group (err %d)\n", err);
		return err;
	}

	return 0;
}

static int set_stream_qos(void)
{
	int err;

	err = bt_bap_stream_qos(default_conn, unicast_group);
	if (err != 0) {
		printk("Unable to setup QoS: %d\n", err);
		return err;
	}

	for (size_t i = 0U; i < configured_stream_count; i++) {
		printk("QoS: waiting for %zu streams\n", configured_stream_count);
		err = k_sem_take(&sem_stream_qos, K_FOREVER);
		if (err != 0) {
			printk("failed to take sem_stream_qos (err %d)\n", err);
			return err;
		}
	}

	return 0;
}

static int enable_streams(void)
{
	for (size_t i = 0U; i < configured_stream_count; i++) {
		int err;

		err = bt_bap_stream_enable(&streams[i], stream_presets[i].codec_cfg.meta,
					   stream_presets[i].codec_cfg.meta_len);
		if (err != 0) {
			printk("Unable to enable stream: %d\n", err);
			return err;
		}

		err = k_sem_take(&sem_stream_enabled, K_FOREVER);
		if (err != 0) {
			printk("failed to take sem_stream_enabled (err %d)\n", err);
			return err;
		}
	}

	return 0;
}

static int connect_streams(void)
{
	for (size_t i = 0U; i < configured_stream_count; i++) {
		int err;

		k_sem_reset(&sem_stream_connected);

		err = bt_bap_stream_connect(&streams[i]);
		if (err == -EALREADY) {
			/* We have already connected a paired stream */
			continue;
		} else if (err != 0) {
			printk("Unable to start stream: %d\n", err);
			return err;
		}

		err = k_sem_take(&sem_stream_connected, K_FOREVER);
		if (err != 0) {
			printk("failed to take sem_stream_connected (err %d)\n", err);
			return err;
		}
	}

	return 0;
}

static enum bt_audio_dir stream_dir(const struct bt_bap_stream *stream)
{
	struct bt_bap_ep_info ep_info;
	int err;

	err = bt_bap_ep_get_info(stream->ep, &ep_info);
	if (err != 0) {
		printk("Failed to get ep info for %p: %d\n", stream, err);
		__ASSERT_NO_MSG(false);

		return 0;
	}

	return ep_info.dir;
}

static int start_streams(void)
{
	for (size_t i = 0U; i < configured_stream_count; i++) {
		struct bt_bap_stream *stream = &streams[i];
		int err;

		if (stream_dir(stream) == BT_AUDIO_DIR_SOURCE) {
			err = bt_bap_stream_start(&streams[i]);
			if (err != 0) {
				printk("Unable to start stream: %d\n", err);
				return err;
			}
		} /* Sink streams are started by the unicast server */

		err = k_sem_take(&sem_stream_started, K_FOREVER);
		if (err != 0) {
			printk("failed to take sem_stream_started (err %d)\n", err);
			return err;
		}
	}

	return 0;
}

static void reset_data(void)
{
	k_sem_reset(&sem_connected);
	k_sem_reset(&sem_disconnected);
	k_sem_reset(&sem_mtu_exchanged);
	k_sem_reset(&sem_security_updated);
	k_sem_reset(&sem_sinks_discovered);
	k_sem_reset(&sem_sources_discovered);
	k_sem_reset(&sem_stream_configured);
	k_sem_reset(&sem_stream_qos);
	k_sem_reset(&sem_stream_enabled);
	k_sem_reset(&sem_stream_started);
	k_sem_reset(&sem_stream_connected);

	configured_sink_stream_count = 0U;
	configured_source_stream_count = 0U;
	memset(sinks, 0, sizeof(sinks));
	memset(sources, 0, sizeof(sources));
}

int main(void)
{
	int err;

	printk("Initializing\n");
	err = init();
	if (err != 0) {
		return 0;
	}
	printk("Initialized\n");

	err = bt_bap_unicast_client_register_cb(&unicast_client_cbs);
	if (err != 0) {
		printk("Failed to register client callbacks: %d", err);
		return 0;
	}

	while (true) {
		reset_data();

		printk("Waiting for connection\n");
		err = scan_and_connect();
		if (err != 0) {
			return 0;
		}
		printk("Connected\n");

		printk("Discovering sinks\n");
		err = discover_sinks();
		if (err != 0) {
			return 0;
		}
		printk("Sinks discovered\n");

		printk("Discovering sources\n");
		err = discover_sources();
		if (err != 0) {
			return 0;
		}
		printk("Sources discovered\n");

		printk("Configuring streams\n");
		err = configure_streams();
		if (err != 0) {
			return 0;
		}

		if (configured_stream_count == 0U) {
			printk("No streams were configured\n");
			return 0;
		}

		printk("Creating unicast group\n");
		err = create_group();
		if (err != 0) {
			return 0;
		}
		printk("Unicast group created\n");

		printk("Setting stream QoS\n");
		err = set_stream_qos();
		if (err != 0) {
			return 0;
		}
		printk("Stream QoS Set\n");

		printk("Enabling streams\n");
		err = enable_streams();
		if (err != 0) {
			return 0;
		}
		printk("Streams enabled\n");

		printk("Connecting streams\n");
		err = connect_streams();
		if (err != 0) {
			return 0;
		}
		printk("Streams connected\n");

		printk("Starting streams\n");
		err = start_streams();
		if (err != 0) {
			return 0;
		}
		printk("Streams started\n");

		/* Wait for disconnect */
		err = k_sem_take(&sem_disconnected, K_FOREVER);
		if (err != 0) {
			printk("failed to take sem_disconnected (err %d)\n", err);
			return 0;
		}

		printk("Deleting group\n");
		err = delete_group();
		if (err != 0) {
			return 0;
		}
		printk("Group deleted\n");
	}
}
