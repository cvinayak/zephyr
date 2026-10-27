/*
 * Copyright (c) 2021-2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/autoconf.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/assigned_numbers.h>
#include <zephyr/bluetooth/audio/ascs.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/lc3.h>
#include <zephyr/bluetooth/audio/pacs.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/clock.h>
#include <zephyr/toolchain.h>
#include <zephyr/types.h>

#include "stream_tx.h"

#if defined(CONFIG_USE_TLV320_AUDIO)
#define TLV320_AUDIO_OUTPUT_ENABLED 1
#else
#define TLV320_AUDIO_OUTPUT_ENABLED 0
#endif

/* Sink streams are decoded to 48 kHz stereo and played on the TLV320 DAC or the USB microphone */
#if defined(CONFIG_USE_TLV320_AUDIO) || defined(CONFIG_USE_USB_AUDIO)
#define AUDIO_PLAYBACK_ENABLED 1
#else
#define AUDIO_PLAYBACK_ENABLED 0
#endif

#define SDU_METRICS_INTERVAL_SECONDS 10U

#if AUDIO_PLAYBACK_ENABLED
#include <lc3.h>

#include "stream_lc3.h"

/* Audio output is 48 kHz stereo, 16-bit samples */
#define DAC_SAMPLE_RATE_HZ     48000U
#define DAC_CHAN_CNT           2U
#define DAC_FRAME_DURATION_US  10000U
#define DAC_FRAME_SAMPLES      ((DAC_FRAME_DURATION_US * DAC_SAMPLE_RATE_HZ) / USEC_PER_SEC)

#define JITTER_BUFFER_SIZE  6U
#define JITTER_BUFFER_CHECK 4U

#define CHAN_LEFT  0U
#define CHAN_RIGHT 1U
#endif /* AUDIO_PLAYBACK_ENABLED */

#if defined(CONFIG_USE_USB_AUDIO)
#include "usb.h"
#endif /* CONFIG_USE_USB_AUDIO */

#if TLV320_AUDIO_OUTPUT_ENABLED
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>
#include <nrfx_i2s.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* NRF54L15_XXAA */

#define I2C_NODE DT_NODELABEL(tlv320)
#define I2S_NL   DT_NODELABEL(i2s20)

/* Samples per channel in each 1 ms I2S block */
#define I2S_SAMPLES_NUM        48U
#define I2S_BLOCK_SIZE         (I2S_SAMPLES_NUM * DAC_CHAN_CNT * sizeof(int16_t))
/* Number of 1 ms I2S blocks buffered between LC3 decoder and I2S */
#define BUFFER_SPACE           60U
#define RING_BUF_SIZE          (I2S_BLOCK_SIZE * BUFFER_SPACE)

static const struct gpio_dt_spec dac_rst = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

PINCTRL_DT_DEFINE(I2S_NL);
static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(NRF_I2S20);
static nrfx_i2s_config_t i2s_cfg = {
	/* Pins are configured by pinctrl. */
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_SLAVE,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.prescalers = {
		.ratio = NRF_I2S_RATIO_64X,
		.mck_setup = NRF_I2S_MCK_32MDIV2,
	},
	.sample_width = NRF_I2S_SWIDTH_16BIT,
	.channels = NRF_I2S_CHANNELS_STEREO,
};

static int16_t i2s_tx_buf_a[I2S_SAMPLES_NUM * DAC_CHAN_CNT] __aligned(4);
static int16_t i2s_tx_buf_b[I2S_SAMPLES_NUM * DAC_CHAN_CNT] __aligned(4);
static int16_t i2s_rx_buf_a[I2S_SAMPLES_NUM * DAC_CHAN_CNT] __aligned(4);
static int16_t i2s_rx_buf_b[I2S_SAMPLES_NUM * DAC_CHAN_CNT] __aligned(4);
/* The audio thread is the only producer and the I2S interrupt the only consumer of the ring
 * buffers, so no lock is needed and interrupts are not disabled while audio is copied.
 */
RING_BUF_DECLARE(i2s_tx_ring_buf, RING_BUF_SIZE);
/* I2S TX ring buffer flush request, handled by the I2S interrupt (consumer) */
static atomic_t i2s_tx_flush;
/* Last ring buffer free space in percent, reported with the SDU metrics */
static uint8_t ring_buf_free_percent;

#if defined(CONFIG_TLV320_MIC)
/* 48 kHz stereo frames received from the TLV320 ADC on I2S SDIN */
#define MIC_FRAME_SIZE        (DAC_CHAN_CNT * sizeof(int16_t))
#define MIC_RING_BUF_SIZE     (I2S_BLOCK_SIZE * 48U)
/* Mic ring buffer levels, in 1 ms I2S blocks */
#define MIC_PRIME_BLOCKS      20U
#define MIC_DRIFT_LOW_BLOCKS  5U
#define MIC_DRIFT_HIGH_BLOCKS 35U

/* Produced by the I2S interrupt, consumed by the TX thread */
RING_BUF_DECLARE(mic_ring_buf, MIC_RING_BUF_SIZE);
static bool mic_primed;
static uint32_t mic_underrun_cnt;
static uint32_t mic_overrun_cnt;
static uint32_t mic_drift_cnt;
#endif /* CONFIG_TLV320_MIC */
#endif /* TLV320_AUDIO_OUTPUT_ENABLED */

#if AUDIO_PLAYBACK_ENABLED
struct recv_pkt_info {
	uint32_t ts;
	bool bad_frame;
	uint16_t size;
	uint8_t buf[CONFIG_BT_ISO_RX_MTU];
};

K_MSGQ_DEFINE(recv_pkt_msgq_l, sizeof(struct recv_pkt_info), JITTER_BUFFER_SIZE, 4U);
K_MSGQ_DEFINE(recv_pkt_msgq_r, sizeof(struct recv_pkt_info), JITTER_BUFFER_SIZE, 4U);
static struct k_msgq *const recv_pkt_msgq[DAC_CHAN_CNT] = {&recv_pkt_msgq_l, &recv_pkt_msgq_r};

/* Sink stream playing each DAC channel, NULL when the channel is not streaming */
static struct audio_sink *chan_sink[DAC_CHAN_CNT];
/* Maximum SDU timestamp difference for left and right SDUs to be played together */
static uint32_t pair_tolerance_us;

/* LC3 decoding and DAC playback run in a dedicated thread, not in the BT RX thread */
#define AUDIO_THREAD_STACK_SIZE 4096U
#define AUDIO_THREAD_PRIO       K_PRIO_PREEMPT(4)

K_SEM_DEFINE(audio_sem, 0U, K_SEM_MAX_LIMIT);
/* Protects chan_sink, the jitter buffers and the decoders against the audio thread */
K_MUTEX_DEFINE(audio_lock);
#endif /* AUDIO_PLAYBACK_ENABLED */

#define AVAILABLE_SINK_CONTEXT  (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
				 BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
				 BT_AUDIO_CONTEXT_TYPE_MEDIA | \
				 BT_AUDIO_CONTEXT_TYPE_GAME | \
				 BT_AUDIO_CONTEXT_TYPE_INSTRUCTIONAL)

#define AVAILABLE_SOURCE_CONTEXT (BT_AUDIO_CONTEXT_TYPE_UNSPECIFIED | \
				  BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | \
				  BT_AUDIO_CONTEXT_TYPE_MEDIA | \
				  BT_AUDIO_CONTEXT_TYPE_GAME)

NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT,
			  BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

#if AUDIO_PLAYBACK_ENABLED
/* LC3 is decoded and upsampled to the 48 kHz output rate. Up to 155 octets per frame accepts the
 * 48_5 and 48_6 configurations.
 */
static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
	BT_AUDIO_CODEC_CAP_FREQ_16KHZ | BT_AUDIO_CODEC_CAP_FREQ_24KHZ |
		BT_AUDIO_CODEC_CAP_FREQ_32KHZ | BT_AUDIO_CODEC_CAP_FREQ_48KHZ,
	BT_AUDIO_CODEC_CAP_DURATION_10, BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 40u, 155u, 1u,
	(BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA));
/* The 48 kHz audio input is decimated by an integer factor for the source stream */
static const struct bt_audio_codec_cap lc3_codec_cap_src = BT_AUDIO_CODEC_CAP_LC3(
	BT_AUDIO_CODEC_CAP_FREQ_16KHZ | BT_AUDIO_CODEC_CAP_FREQ_24KHZ |
		BT_AUDIO_CODEC_CAP_FREQ_48KHZ,
	BT_AUDIO_CODEC_CAP_DURATION_10, BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 40u, 155u, 1u,
	(BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA));
#else
static const struct bt_audio_codec_cap lc3_codec_cap = BT_AUDIO_CODEC_CAP_LC3(
	BT_AUDIO_CODEC_CAP_FREQ_ANY, BT_AUDIO_CODEC_CAP_DURATION_10,
	BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 40u, 120u, 1u,
	(BT_AUDIO_CONTEXT_TYPE_CONVERSATIONAL | BT_AUDIO_CONTEXT_TYPE_MEDIA));
#define lc3_codec_cap_src lc3_codec_cap
#endif /* AUDIO_PLAYBACK_ENABLED */

static struct bt_conn *default_conn;
static struct audio_sink {
	struct bt_bap_stream stream;
	size_t recv_cnt;
#if AUDIO_PLAYBACK_ENABLED
	uint8_t chan;
	lc3_decoder_t decoder;
	lc3_decoder_mem_48k_t decoder_mem;
#endif /* AUDIO_PLAYBACK_ENABLED */
} sink_streams[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static struct audio_source {
	struct bt_bap_stream stream;
	uint16_t seq_num;
} source_streams[CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT];
static size_t configured_source_stream_count;

#if AUDIO_PLAYBACK_ENABLED
/* Accept presentation delays from 10 ms to 40 ms used by typical USB LE Audio dongles */
static const struct bt_bap_qos_cfg_pref qos_pref =
	BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_2M, 0x02U, 10U, 10000U, 40000U, 10000U, 40000U);
#else
static const struct bt_bap_qos_cfg_pref qos_pref =
	BT_BAP_QOS_CFG_PREF(true, BT_GAP_LE_PHY_2M, 0x02U, 10U, 40000U, 40000U, 40000U, 40000U);
#endif /* AUDIO_PLAYBACK_ENABLED */

struct sdu_reception_metrics {
	uint64_t received_cnt;
	uint64_t valid_cnt;
	uint64_t invalid_cnt;
	uint64_t empty_cnt;
	uint64_t error_cnt;
	uint64_t lost_cnt;
	uint64_t duplicate_ts_cnt;
	uint64_t duplicate_seq_cnt;
	uint64_t bytes_cnt;
	uint32_t last_ts;
	uint16_t last_seq;
	bool has_last;
};

static struct sdu_reception_metrics sdu_metrics[CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT];
static int64_t sdu_metrics_next_report_ms;

static K_SEM_DEFINE(sem_disconnected, 0U, 1U);

static uint8_t unicast_server_addata[] = {
	BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL), /* ASCS UUID */
	BT_AUDIO_UNICAST_ANNOUNCEMENT_TARGETED, /* Target Announcement */
	BT_BYTES_LIST_LE16(AVAILABLE_SINK_CONTEXT),
	BT_BYTES_LIST_LE16(AVAILABLE_SOURCE_CONTEXT),
	0x00U, /* Metadata length */
};

/* TODO: Expand with BAP data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL)),
	BT_DATA(BT_DATA_SVC_DATA16, unicast_server_addata, ARRAY_SIZE(unicast_server_addata)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

#if defined(CONFIG_LIBLC3) && !AUDIO_PLAYBACK_ENABLED

#include "lc3.h"

#define MAX_SAMPLE_RATE         48000U
#define MAX_FRAME_DURATION_US   10000U
#define MAX_NUM_SAMPLES         ((MAX_FRAME_DURATION_US * MAX_SAMPLE_RATE) / USEC_PER_SEC)

static lc3_decoder_t lc3_decoder;
static lc3_decoder_mem_48k_t lc3_decoder_mem;
static int frames_per_sdu;

#endif

static void sdu_metrics_maybe_log(void)
{
	const int64_t now_ms = k_uptime_get();

	if (now_ms < sdu_metrics_next_report_ms) {
		return;
	}

	printk("SDU reception metrics (%u s interval)\n", SDU_METRICS_INTERVAL_SECONDS);
	for (size_t i = 0U; i < ARRAY_SIZE(sdu_metrics); i++) {
		const struct sdu_reception_metrics *metrics = &sdu_metrics[i];

		printk("  stream[%u]: rx=%llu valid=%llu invalid=%llu empty=%llu "
		       "error=%llu lost=%llu dup_ts=%llu dup_seq=%llu bytes=%llu\n",
		       (unsigned int)i,
		       (unsigned long long)metrics->received_cnt,
		       (unsigned long long)metrics->valid_cnt,
		       (unsigned long long)metrics->invalid_cnt,
		       (unsigned long long)metrics->empty_cnt,
		       (unsigned long long)metrics->error_cnt,
		       (unsigned long long)metrics->lost_cnt,
		       (unsigned long long)metrics->duplicate_ts_cnt,
		       (unsigned long long)metrics->duplicate_seq_cnt,
		       (unsigned long long)metrics->bytes_cnt);
	}

#if TLV320_AUDIO_OUTPUT_ENABLED
	printk("  I2S ring buffer free %u %%\n", ring_buf_free_percent);
#endif /* TLV320_AUDIO_OUTPUT_ENABLED */
#if defined(CONFIG_TLV320_MIC)
	printk("  Mic underrun %u overrun %u drift corrections %u\n", mic_underrun_cnt,
	       mic_overrun_cnt, mic_drift_cnt);
#endif /* CONFIG_TLV320_MIC */
#if defined(CONFIG_USE_USB_AUDIO)
	usb_audio_stats_log();
#endif /* CONFIG_USE_USB_AUDIO */

	sdu_metrics_next_report_ms = now_ms + ((int64_t)SDU_METRICS_INTERVAL_SECONDS * MSEC_PER_SEC);
}

static void sdu_metrics_update(const struct audio_sink *sink, const struct bt_iso_recv_info *info,
			       const struct net_buf *buf)
{
	const size_t idx = ARRAY_INDEX(sink_streams, sink);
	struct sdu_reception_metrics *metrics = &sdu_metrics[idx];

	metrics->received_cnt++;
	metrics->bytes_cnt += buf->len;

	if ((info->flags & BT_ISO_FLAGS_VALID) != 0U) {
		if (buf->len == 0U) {
			metrics->empty_cnt++;
		} else {
			metrics->valid_cnt++;
		}
	} else {
		metrics->invalid_cnt++;
	}

	if ((info->flags & BT_ISO_FLAGS_ERROR) != 0U) {
		metrics->error_cnt++;
	}

	if ((info->flags & BT_ISO_FLAGS_LOST) != 0U) {
		metrics->lost_cnt++;
	}

	if (metrics->has_last) {
		if (metrics->last_ts == info->ts) {
			metrics->duplicate_ts_cnt++;
		}

		if (metrics->last_seq == info->seq_num) {
			metrics->duplicate_seq_cnt++;
		}
	}

	metrics->has_last = true;
	metrics->last_ts = info->ts;
	metrics->last_seq = info->seq_num;

	sdu_metrics_maybe_log();
}

static void sdu_metrics_reset(const struct audio_sink *sink)
{
	const size_t idx = ARRAY_INDEX(sink_streams, sink);

	(void)memset(&sdu_metrics[idx], 0, sizeof(sdu_metrics[idx]));
	sdu_metrics_next_report_ms =
		k_uptime_get() + ((int64_t)SDU_METRICS_INTERVAL_SECONDS * MSEC_PER_SEC);
}

#if TLV320_AUDIO_OUTPUT_ENABLED
static void audio_i2s_set_next_buf(int16_t *tx_buf, int16_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {
		.p_rx_buffer = (uint32_t *)rx_buf,
		.p_tx_buffer = (uint32_t *)tx_buf,
		/* Buffer size in 32-bit words, i.e. stereo 16-bit samples */
		.buffer_size = I2S_SAMPLES_NUM,
	};
	int ret;

	ret = nrfx_i2s_next_buffers_set(&i2s_inst, &i2s_buf);
	if (ret != 0) {
		printk("Failed to set next I2S buffers: %d\n", ret);
	}
}

static void i2s_fill_tx_buf(int16_t *tx_buf)
{
	uint32_t len;

	if (atomic_cas(&i2s_tx_flush, 1, 0)) {
		ring_buf_consume(&i2s_tx_ring_buf, ring_buf_size_get(&i2s_tx_ring_buf));
	}

	len = ring_buf_get(&i2s_tx_ring_buf, (uint8_t *)tx_buf, I2S_BLOCK_SIZE);

	if (len != I2S_BLOCK_SIZE) {
		/* Underrun, play silence */
		(void)memset(tx_buf, 0, I2S_BLOCK_SIZE);
	}
}

#if defined(CONFIG_TLV320_MIC)
/* Queue a received I2S block for the source stream encoder, no processing in the interrupt */
static void i2s_store_rx_buf(const int16_t *rx_buf)
{
	if (rx_buf == NULL) {
		return;
	}

	if (ring_buf_put(&mic_ring_buf, (const uint8_t *)rx_buf, I2S_BLOCK_SIZE) !=
	    I2S_BLOCK_SIZE) {
		mic_overrun_cnt++;
	}
}

/* Provide the TLV320 mono ADC output, left I2S slot, decimated to the source stream rate */
static bool mic_pcm_get(enum bt_audio_location chan_allocation, uint32_t freq_hz, int16_t *pcm,
			size_t samples)
{
	/* Only called from the TX thread; static to save stack */
	static int16_t in[(DAC_FRAME_SAMPLES + 1U) * DAC_CHAN_CNT];
	const uint32_t prime = I2S_BLOCK_SIZE * MIC_PRIME_BLOCKS;
	uint32_t ratio;
	uint32_t fill;
	size_t frames;
	size_t read;

	ARG_UNUSED(chan_allocation);

	if (freq_hz == 0U || (DAC_SAMPLE_RATE_HZ % freq_hz) != 0U ||
	    (samples * (DAC_SAMPLE_RATE_HZ / freq_hz)) > DAC_FRAME_SAMPLES) {
		(void)memset(pcm, 0, samples * sizeof(int16_t));
		return false;
	}

	ratio = DAC_SAMPLE_RATE_HZ / freq_hz;
	frames = samples * ratio;
	read = frames;

	fill = ring_buf_size_get(&mic_ring_buf);
	if (!mic_primed && fill >= MAX(prime, frames * MIC_FRAME_SIZE)) {
		mic_primed = true;
	}

	if (mic_primed && fill < frames * MIC_FRAME_SIZE) {
		mic_primed = false;
		mic_underrun_cnt++;
	}

	if (!mic_primed) {
		(void)memset(pcm, 0, samples * sizeof(int16_t));
		return false;
	}

	/* Correct the drift between the I2S and Bluetooth clocks by one sample frame */
	if (fill > I2S_BLOCK_SIZE * MIC_DRIFT_HIGH_BLOCKS) {
		read = frames + 1U;
		mic_drift_cnt++;
	} else if (fill < I2S_BLOCK_SIZE * MIC_DRIFT_LOW_BLOCKS) {
		read = frames - 1U;
		mic_drift_cnt++;
	}

	(void)ring_buf_get(&mic_ring_buf, (uint8_t *)in, read * MIC_FRAME_SIZE);

	if (read < frames) {
		in[(frames - 1U) * DAC_CHAN_CNT] = in[(frames - 2U) * DAC_CHAN_CNT];
	}

	for (size_t i = 0U; i < samples; i++) {
		int32_t sum = 0;

		for (uint32_t j = 0U; j < ratio; j++) {
			sum += in[((i * ratio) + j) * DAC_CHAN_CNT];
		}

		pcm[i] = (int16_t)(sum / (int32_t)ratio);
	}

	return true;
}
#endif /* CONFIG_TLV320_MIC */

static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	if (status != NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
		return;
	}

#if defined(CONFIG_TLV320_MIC)
	i2s_store_rx_buf((const int16_t *)released_bufs->p_rx_buffer);
#endif /* CONFIG_TLV320_MIC */

	if ((int16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_a) {
		i2s_fill_tx_buf(i2s_tx_buf_a);
		audio_i2s_set_next_buf(i2s_tx_buf_a, i2s_rx_buf_a);
	} else if ((int16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_b) {
		i2s_fill_tx_buf(i2s_tx_buf_b);
		audio_i2s_set_next_buf(i2s_tx_buf_b, i2s_rx_buf_b);
	}
}

static int audio_i2s_start(void)
{
	const nrfx_i2s_buffers_t i2s_buf = {
		.p_rx_buffer = (uint32_t *)i2s_rx_buf_a,
		.p_tx_buffer = (uint32_t *)i2s_tx_buf_a,
		.buffer_size = I2S_SAMPLES_NUM,
	};
	int ret;

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		printk("Failed to apply I2S pinctrl state: %d\n", ret);
		return ret;
	}

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_i2s_irq_handler, &i2s_inst, 0);
	irq_enable(DT_IRQN(I2S_NL));

	ret = nrfx_i2s_init(&i2s_inst, &i2s_cfg, i2s_comp_handler);
	if (ret != 0) {
		printk("Failed to initialize I2S: %d\n", ret);
		return ret;
	}

	ret = nrfx_i2s_start(&i2s_inst, &i2s_buf, 0);
	if (ret != 0) {
		printk("Failed to start I2S: %d\n", ret);
		return ret;
	}

	audio_i2s_set_next_buf(i2s_tx_buf_b, i2s_rx_buf_b);

	return 0;
}

static int clocks_start(void)
{
	const struct device *xo = DEVICE_DT_GET(DT_NODELABEL(xo));
	int err;

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* NRF54L15_XXAA */

	if (!device_is_ready(xo)) {
		printk("XO clock device not ready\n");
		return -ENXIO;
	}

	err = nrf_clock_control_request_sync(xo, NULL, K_FOREVER);
	if (err < 0) {
		printk("HF clock could not be started: %d\n", err);
		return err;
	}

	printk("HF clock started\n");

	return 0;
}

static void dac_i2c_write(uint8_t reg, uint8_t value)
{
	const uint8_t config[2] = {reg, value};
	int ret;

	ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
	if (ret != 0) {
		printk("Failed to write to I2C device address %x at reg. %x\n", dev_i2c.addr, reg);
	}
}

/* TLV320 PLL D value, D[13:8] and nominal, faster and slower D[7:0] */
#define DAC_PLL_D_MSB      0x0EU
#define DAC_PLL_D_LSB_NOM  0xB0U
#define DAC_PLL_D_LSB_FAST 0xDAU
#define DAC_PLL_D_LSB_SLOW 0x86U

static int tlv320_setup(void)
{
	int err;

	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready\n", dev_i2c.bus->name);
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&dac_rst, GPIO_OUTPUT);
	if (err != 0) {
		printk("Failed to configure TLV320 reset GPIO: %d\n", err);
		return err;
	}

	/* Reset the TLV320 */
	(void)gpio_pin_set_dt(&dac_rst, 0);
	k_sleep(K_MSEC(1000));
	(void)gpio_pin_set_dt(&dac_rst, 1);

	/* Page 0, software reset */
	dac_i2c_write(0x00, 0x00);
	dac_i2c_write(0x01, 0x01);
	k_sleep(K_MSEC(10));

	/* Clock and PLL setup */
	dac_i2c_write(0x04, 0x03);
	dac_i2c_write(0x05, (0x1 << 4) | 0x1);
	dac_i2c_write(0x06, 0x05);
	dac_i2c_write(0x07, DAC_PLL_D_MSB);
	dac_i2c_write(0x08, DAC_PLL_D_LSB_NOM);
	dac_i2c_write(0x05, BIT(7) | (0x1 << 4) | 0x1);
	k_sleep(K_MSEC(15));

	/* NDAC, MDAC, DOSR */
	dac_i2c_write(0x0B, 0x87);
	dac_i2c_write(0x0C, 0x82);
	dac_i2c_write(0x0D, 0x00);
	dac_i2c_write(0x0E, 0x80);

	/* Audio interface, BCLK, processing block */
	dac_i2c_write(0x1B, 0x0C);
	dac_i2c_write(0x1E, 0x84);
	dac_i2c_write(0x1D, 0x01);
	dac_i2c_write(0x3C, 0x01);
	dac_i2c_write(0x74, 0x00);

	/* Page 1, output drivers */
	dac_i2c_write(0x00, 0x01);
	dac_i2c_write(0x1F, 0x00);
	dac_i2c_write(0x21, (0x7 << 3) | (0x3 << 1));
	dac_i2c_write(0x23, 0x44);
	dac_i2c_write(0x24, 0x80);
	dac_i2c_write(0x25, 0x80);
	dac_i2c_write(0x28, 0x06);
	dac_i2c_write(0x29, 0x06);
	dac_i2c_write(0x1F, 0xC0);
	k_sleep(K_MSEC(300));

	/* Page 0, DAC power up, volume and unmute */
	dac_i2c_write(0x00, 0x00);
	dac_i2c_write(0x3F, 0xD4);
	dac_i2c_write(0x41, (uint8_t)-60);
	dac_i2c_write(0x42, (uint8_t)-60);
	dac_i2c_write(0x40, 0x00);

#if defined(CONFIG_TLV320_MIC)
	/* Page 1, MICBIAS 2.5 V, MIC PGA 20 dB, MIC1LP and CM through 10 kOhm */
	dac_i2c_write(0x00, 0x01);
	dac_i2c_write(0x2E, 0x02);
	dac_i2c_write(0x2F, 0x28);
	dac_i2c_write(0x30, 0x40);
	dac_i2c_write(0x31, 0x40);

	/* Page 0, NADC, MADC and AOSR as for the DAC, processing block PRB_R4 */
	dac_i2c_write(0x00, 0x00);
	dac_i2c_write(0x12, 0x87);
	dac_i2c_write(0x13, 0x82);
	dac_i2c_write(0x14, 0x80);
	dac_i2c_write(0x3D, 0x04);

	/* AGC enabled with -8 dB target and 20 dB maximum gain */
	dac_i2c_write(0x56, 0x90);
	dac_i2c_write(0x58, 0x28);

	/* ADC power up, unmute, 0 dB fine and coarse gain */
	dac_i2c_write(0x51, 0x80);
	dac_i2c_write(0x52, 0x00);
	dac_i2c_write(0x53, 0x00);
#endif /* CONFIG_TLV320_MIC */

	return 0;
}

/* Track the source clock drift by adjusting the TLV320 PLL using the ring buffer fill level */
static void dac_pll_track(void)
{
	static uint8_t d_lsb_cur = DAC_PLL_D_LSB_NOM;
	uint32_t space;
	uint8_t d_lsb;

	space = ring_buf_space_get(&i2s_tx_ring_buf);

	ring_buf_free_percent = (uint8_t)((space * 100U) / RING_BUF_SIZE);

	if (ring_buf_free_percent < 45U) {
		d_lsb = DAC_PLL_D_LSB_FAST;
	} else if (ring_buf_free_percent <= 55U) {
		d_lsb = DAC_PLL_D_LSB_NOM;
	} else {
		d_lsb = DAC_PLL_D_LSB_SLOW;
	}

	if (d_lsb != d_lsb_cur) {
		d_lsb_cur = d_lsb;
		dac_i2c_write(0x07, DAC_PLL_D_MSB);
		dac_i2c_write(0x08, d_lsb);
	}
}
#endif /* TLV320_AUDIO_OUTPUT_ENABLED */

#if AUDIO_PLAYBACK_ENABLED
static void dac_buf_reset(void)
{
	k_msgq_purge(&recv_pkt_msgq_l);
	k_msgq_purge(&recv_pkt_msgq_r);

#if TLV320_AUDIO_OUTPUT_ENABLED
	atomic_set(&i2s_tx_flush, 1);
#endif /* TLV320_AUDIO_OUTPUT_ENABLED */
}

static uint8_t chan_from_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg)
{
	enum bt_audio_location loc;
	int err;

	err = bt_audio_codec_cfg_get_chan_allocation(codec_cfg, &loc, false);
	if (err == 0 && (loc & BT_AUDIO_LOCATION_FRONT_RIGHT) != 0U &&
	    (loc & BT_AUDIO_LOCATION_FRONT_LEFT) == 0U) {
		return CHAN_RIGHT;
	}

	return CHAN_LEFT;
}

static void dac_sink_start(struct audio_sink *sink, const struct bt_bap_stream *stream)
{
	(void)k_mutex_lock(&audio_lock, K_FOREVER);

	if (chan_sink[sink->chan] != NULL && chan_sink[sink->chan] != sink) {
		/* Both streams have the same location, use the other channel */
		sink->chan = (sink->chan == CHAN_LEFT) ? CHAN_RIGHT : CHAN_LEFT;
	}

	if (chan_sink[sink->chan] == NULL) {
		chan_sink[sink->chan] = sink;
		pair_tolerance_us = stream->qos->interval / 2U;
		dac_buf_reset();
		printk("  Playing on %s channel\n", (sink->chan == CHAN_LEFT) ? "left" : "right");
	}

	(void)k_mutex_unlock(&audio_lock);
}

static void dac_sink_stop(struct audio_sink *sink)
{
	(void)k_mutex_lock(&audio_lock, K_FOREVER);

	if (chan_sink[sink->chan] == sink) {
		chan_sink[sink->chan] = NULL;
	}

	sink->decoder = NULL;
	dac_buf_reset();

	(void)k_mutex_unlock(&audio_lock);
}

static void dac_decode(struct audio_sink *sink, const struct recv_pkt_info *pkt, int16_t *pcm)
{
	int err;

	/* Passing NULL performs PLC */
	err = lc3_decode(sink->decoder, pkt->bad_frame ? NULL : pkt->buf, pkt->size,
			 LC3_PCM_FORMAT_S16, pcm, DAC_CHAN_CNT);
	if (err < 0) {
		printk("LC3 decoder failed - wrong parameters?: %d\n", err);
	}
}

/* Play one 48 kHz stereo frame on the TLV320 DAC or send it to the USB microphone */
static void dac_play(const int16_t *pcm)
{
#if TLV320_AUDIO_OUTPUT_ENABLED
	dac_pll_track();

	(void)ring_buf_put(&i2s_tx_ring_buf, (const uint8_t *)pcm,
			   DAC_FRAME_SAMPLES * DAC_CHAN_CNT * sizeof(int16_t));
#else
	usb_audio_pcm_put(pcm, DAC_FRAME_SAMPLES, DAC_CHAN_CNT);
#endif /* TLV320_AUDIO_OUTPUT_ENABLED */
}

/* Queue a received SDU for the audio thread; decoding is kept out of the BT RX thread */
static void stream_recv_tlv320(struct audio_sink *sink, const struct bt_iso_recv_info *info,
			       struct net_buf *buf)
{
	/* Only called from the BT RX thread; static to save stack */
	static struct recv_pkt_info pkt;
	static struct recv_pkt_info drop;
	struct k_msgq *msgq;

	if (sink->decoder == NULL || chan_sink[sink->chan] != sink ||
	    buf->len > sizeof(pkt.buf)) {
		return;
	}

	pkt.ts = info->ts;
	pkt.size = buf->len;
	pkt.bad_frame = (buf->len == 0U) || ((info->flags & BT_ISO_FLAGS_VALID) == 0U);
	(void)memcpy(pkt.buf, buf->data, buf->len);

	msgq = recv_pkt_msgq[sink->chan];
	if (k_msgq_put(msgq, &pkt, K_NO_WAIT) != 0) {
		printk("%c: jitter buffer full\n", (sink->chan == CHAN_LEFT) ? 'L' : 'R');
		(void)k_msgq_get(msgq, &drop, K_NO_WAIT);
		(void)k_msgq_put(msgq, &pkt, K_NO_WAIT);
	}

	k_sem_give(&audio_sem);
}

static void audio_play_mono(void)
{
	/* Only called from the audio thread; static to save stack */
	static int16_t pcm[DAC_FRAME_SAMPLES * DAC_CHAN_CNT];
	static struct recv_pkt_info pkt;
	struct audio_sink *sink;

	sink = (chan_sink[CHAN_LEFT] != NULL) ? chan_sink[CHAN_LEFT] : chan_sink[CHAN_RIGHT];
	if (sink == NULL || sink->decoder == NULL) {
		return;
	}

	/* Single stream, play the same audio on both channels */
	while (k_msgq_get(recv_pkt_msgq[sink->chan], &pkt, K_NO_WAIT) == 0) {
		dac_decode(sink, &pkt, &pcm[0]);
		for (size_t i = 0U; i < DAC_FRAME_SAMPLES; i++) {
			pcm[(i * DAC_CHAN_CNT) + 1U] = pcm[i * DAC_CHAN_CNT];
		}
		dac_play(pcm);
	}
}

static void audio_play_stereo(void)
{
	/* Only called from the audio thread; static to save stack */
	static int16_t pcm[DAC_FRAME_SAMPLES * DAC_CHAN_CNT];
	static struct recv_pkt_info pkt_l;
	static struct recv_pkt_info pkt_r;

	if (chan_sink[CHAN_LEFT]->decoder == NULL || chan_sink[CHAN_RIGHT]->decoder == NULL) {
		return;
	}

	while (k_msgq_num_used_get(&recv_pkt_msgq_l) >= JITTER_BUFFER_CHECK &&
	       k_msgq_num_used_get(&recv_pkt_msgq_r) >= JITTER_BUFFER_CHECK) {
		int32_t diff;

		(void)k_msgq_peek(&recv_pkt_msgq_l, &pkt_l);
		(void)k_msgq_peek(&recv_pkt_msgq_r, &pkt_r);

		diff = (int32_t)(pkt_l.ts - pkt_r.ts);
		if (diff > (int32_t)pair_tolerance_us) {
			(void)k_msgq_get(&recv_pkt_msgq_r, &pkt_r, K_NO_WAIT);
			printk("drop R %u (L %u)\n", pkt_r.ts, pkt_l.ts);
			continue;
		} else if (diff < -(int32_t)pair_tolerance_us) {
			(void)k_msgq_get(&recv_pkt_msgq_l, &pkt_l, K_NO_WAIT);
			printk("drop L %u (R %u)\n", pkt_l.ts, pkt_r.ts);
			continue;
		}

		(void)k_msgq_get(&recv_pkt_msgq_l, &pkt_l, K_NO_WAIT);
		(void)k_msgq_get(&recv_pkt_msgq_r, &pkt_r, K_NO_WAIT);

		dac_decode(chan_sink[CHAN_LEFT], &pkt_l, &pcm[CHAN_LEFT]);
		dac_decode(chan_sink[CHAN_RIGHT], &pkt_r, &pcm[CHAN_RIGHT]);
		dac_play(pcm);

		break;
	}
}

static void audio_thread_func(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		(void)k_sem_take(&audio_sem, K_FOREVER);

		(void)k_mutex_lock(&audio_lock, K_FOREVER);
		if (chan_sink[CHAN_LEFT] != NULL && chan_sink[CHAN_RIGHT] != NULL) {
			audio_play_stereo();
		} else {
			audio_play_mono();
		}
		(void)k_mutex_unlock(&audio_lock);
	}
}

K_THREAD_DEFINE(audio_thread, AUDIO_THREAD_STACK_SIZE, audio_thread_func, NULL, NULL, NULL,
		AUDIO_THREAD_PRIO, 0, 0);
#endif /* AUDIO_PLAYBACK_ENABLED */

void print_hex(const uint8_t *ptr, size_t len)
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

static void print_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg)
{
	printk("codec_cfg 0x%02x cid 0x%04x vid 0x%04x count %u\n", codec_cfg->id, codec_cfg->cid,
	       codec_cfg->vid, codec_cfg->data_len);

	if (codec_cfg->id == BT_HCI_CODING_FORMAT_LC3) {
		enum bt_audio_location chan_allocation;
		int ret;

		/* LC3 uses the generic LTV format - other codecs might do as well */

		bt_audio_data_parse(codec_cfg->data, codec_cfg->data_len, print_cb, "data");

		ret = bt_audio_codec_cfg_get_freq(codec_cfg);
		if (ret > 0) {
			printk("  Frequency: %d Hz\n", bt_audio_codec_cfg_freq_to_freq_hz(ret));
		}

		ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
		if (ret >= 0) {
			printk("  Frame Duration: %d us\n",
			       bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret));
		}

		ret = bt_audio_codec_cfg_get_chan_allocation(codec_cfg, &chan_allocation, false);
		if (ret == 0) {
			printk("  Channel allocation: 0x%x\n", chan_allocation);
		}

		printk("  Octets per frame: %d (negative means value not present)\n",
		       bt_audio_codec_cfg_get_octets_per_frame(codec_cfg));
		printk("  Frames per SDU: %d\n",
		       bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true));
	} else {
		print_hex(codec_cfg->data, codec_cfg->data_len);
	}

	bt_audio_data_parse(codec_cfg->meta, codec_cfg->meta_len, print_cb, "meta");
}

static void print_qos(const struct bt_bap_qos_cfg *qos)
{
	printk("QoS: interval %u framing 0x%02x phy 0x%02x sdu %u "
	       "rtn %u latency %u pd %u\n",
	       qos->interval, qos->framing, qos->phy, qos->sdu,
	       qos->rtn, qos->latency, qos->pd);
}

static enum bt_audio_dir stream_dir(const struct bt_bap_stream *stream)
{
	for (size_t i = 0U; i < ARRAY_SIZE(source_streams); i++) {
		if (stream == &source_streams[i].stream) {
			return BT_AUDIO_DIR_SOURCE;
		}
	}

	for (size_t i = 0U; i < ARRAY_SIZE(sink_streams); i++) {
		if (stream == &sink_streams[i].stream) {
			return BT_AUDIO_DIR_SINK;
		}
	}

	__ASSERT(false, "Invalid stream %p", stream);
	return 0;
}

static struct bt_bap_stream *stream_alloc(enum bt_audio_dir dir)
{
	if (dir == BT_AUDIO_DIR_SOURCE) {
		for (size_t i = 0U; i < ARRAY_SIZE(source_streams); i++) {
			struct bt_bap_stream *stream = &source_streams[i].stream;

			if (!stream->conn) {
				return stream;
			}
		}
	} else {
		for (size_t i = 0U; i < ARRAY_SIZE(sink_streams); i++) {
			struct bt_bap_stream *stream = &sink_streams[i].stream;

			if (!stream->conn) {
				return stream;
			}
		}
	}

	return NULL;
}

static int lc3_config(struct bt_conn *conn, const struct bt_bap_ep *ep, enum bt_audio_dir dir,
		      const struct bt_audio_codec_cfg *codec_cfg, struct bt_bap_stream **stream,
		      struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	printk("ASE Codec Config: conn %p ep %p dir %u\n", conn, ep, dir);

	print_codec_cfg(codec_cfg);

	*stream = stream_alloc(dir);
	if (*stream == NULL) {
		printk("No streams available\n");
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_NO_MEM, BT_BAP_ASCS_REASON_NONE);

		return -ENOMEM;
	}

	printk("ASE Codec Config stream %p\n", *stream);

	if (dir == BT_AUDIO_DIR_SOURCE) {
		configured_source_stream_count++;
	}

	*pref = qos_pref;

#if AUDIO_PLAYBACK_ENABLED
	if (dir == BT_AUDIO_DIR_SINK) {
		struct audio_sink *sink = CONTAINER_OF(*stream, struct audio_sink, stream);

		/* Nothing to free as static memory is used */
		sink->decoder = NULL;
		sink->chan = chan_from_codec_cfg(codec_cfg);
	}
#elif defined(CONFIG_LIBLC3)
	/* Nothing to free as static memory is used */
	lc3_decoder = NULL;
#endif

	return 0;
}

static int lc3_reconfig(struct bt_bap_stream *stream, enum bt_audio_dir dir,
			const struct bt_audio_codec_cfg *codec_cfg,
			struct bt_bap_qos_cfg_pref *const pref, struct bt_bap_ascs_rsp *rsp)
{
	ARG_UNUSED(dir);
	ARG_UNUSED(pref);

	printk("ASE Codec Reconfig: stream %p\n", stream);

	print_codec_cfg(codec_cfg);

#if AUDIO_PLAYBACK_ENABLED
	if (dir == BT_AUDIO_DIR_SINK) {
		/* Nothing to free as static memory is used */
		CONTAINER_OF(stream, struct audio_sink, stream)->decoder = NULL;
	}
#elif defined(CONFIG_LIBLC3)
	/* Nothing to free as static memory is used */
	lc3_decoder = NULL;
#endif

	*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED, BT_BAP_ASCS_REASON_NONE);

	/* We only support one QoS at the moment, reject changes */
	return -ENOEXEC;
}

static int lc3_qos(struct bt_bap_stream *stream, const struct bt_bap_qos_cfg *qos,
		   struct bt_bap_ascs_rsp *rsp)
{
	ARG_UNUSED(rsp);

	printk("QoS: stream %p qos %p\n", stream, qos);

	print_qos(qos);

	return 0;
}

static int lc3_enable(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
		      struct bt_bap_ascs_rsp *rsp)
{
	ARG_UNUSED(meta);

	printk("Enable: stream %p meta_len %zu\n", stream, meta_len);

#if defined(CONFIG_LIBLC3)
	if (AUDIO_PLAYBACK_ENABLED && stream_dir(stream) != BT_AUDIO_DIR_SINK) {
		/* Only sink streams are decoded */
		return 0;
	}

	{
		int frame_duration_us;
		int freq;
		int ret;

		ret = bt_audio_codec_cfg_get_freq(stream->codec_cfg);
		if (ret > 0) {
			freq = bt_audio_codec_cfg_freq_to_freq_hz(ret);
		} else {
			printk("Error: Codec frequency not set, cannot start codec.");
			*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
					       BT_BAP_ASCS_REASON_CODEC_DATA);
			return ret;
		}

		ret = bt_audio_codec_cfg_get_frame_dur(stream->codec_cfg);
		if (ret >= 0) {
			frame_duration_us = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
		} else {
			printk("Error: Frame duration not set, cannot start codec: %d", ret);
			*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
					       BT_BAP_ASCS_REASON_CODEC_DATA);
			return ret;
		}

#if AUDIO_PLAYBACK_ENABLED
		struct audio_sink *sink = CONTAINER_OF(stream, struct audio_sink, stream);
		lc3_decoder_t lc3_decoder;

		/* Upsample to the output sample rate */
		lc3_decoder = lc3_setup_decoder(frame_duration_us, freq, DAC_SAMPLE_RATE_HZ,
						&sink->decoder_mem);
		sink->decoder = lc3_decoder;
#else
		frames_per_sdu =
			bt_audio_codec_cfg_get_frame_blocks_per_sdu(stream->codec_cfg, true);

		lc3_decoder = lc3_setup_decoder(frame_duration_us,
						freq,
						0, /* No resampling */
						&lc3_decoder_mem);
#endif /* AUDIO_PLAYBACK_ENABLED */

		if (lc3_decoder == NULL) {
			printk("ERROR: Failed to setup LC3 encoder - wrong parameters?\n");
			*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_CONF_INVALID,
					       BT_BAP_ASCS_REASON_CODEC_DATA);
			return -1;
		}
	}
#else
	ARG_UNUSED(rsp);
#endif

	return 0;
}

static int lc3_start(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	ARG_UNUSED(rsp);

	printk("Start: stream %p\n", stream);

	for (size_t i = 0U; i < configured_source_stream_count; i++) {
		if (stream == &source_streams[i].stream) {
			source_streams[i].seq_num = 0U;
			break;
		}
	}

	return 0;
}

static bool data_func_cb(struct bt_data *data, void *user_data)
{
	struct bt_bap_ascs_rsp *rsp = (struct bt_bap_ascs_rsp *)user_data;

	if (!BT_AUDIO_METADATA_TYPE_IS_KNOWN(data->type)) {
		printk("Invalid metadata type %u or length %u\n", data->type, data->data_len);
		*rsp = BT_BAP_ASCS_RSP(BT_BAP_ASCS_RSP_CODE_METADATA_REJECTED, data->type);

		return -EINVAL;
	}

	return true;
}

static int lc3_metadata(struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len,
			struct bt_bap_ascs_rsp *rsp)
{
	printk("Metadata: stream %p meta_len %zu\n", stream, meta_len);

	return bt_audio_data_parse(meta, meta_len, data_func_cb, rsp);
}

static int lc3_disable(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	ARG_UNUSED(rsp);

	printk("Disable: stream %p\n", stream);

	return 0;
}

static int lc3_stop(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	ARG_UNUSED(rsp);

	printk("Stop: stream %p\n", stream);

	return 0;
}

static int lc3_release(struct bt_bap_stream *stream, struct bt_bap_ascs_rsp *rsp)
{
	ARG_UNUSED(rsp);

	printk("Release: stream %p\n", stream);

#if AUDIO_PLAYBACK_ENABLED
	if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
		/* Nothing to free as static memory is used */
		CONTAINER_OF(stream, struct audio_sink, stream)->decoder = NULL;
	}
#endif /* AUDIO_PLAYBACK_ENABLED */

	return 0;
}

static struct bt_bap_unicast_server_register_param param = {
	CONFIG_BT_ASCS_MAX_ASE_SNK_COUNT,
	CONFIG_BT_ASCS_MAX_ASE_SRC_COUNT
};

static const struct bt_bap_unicast_server_cb unicast_server_cb = {
	.config = lc3_config,
	.reconfig = lc3_reconfig,
	.qos = lc3_qos,
	.enable = lc3_enable,
	.start = lc3_start,
	.metadata = lc3_metadata,
	.disable = lc3_disable,
	.stop = lc3_stop,
	.release = lc3_release,
};


#if AUDIO_PLAYBACK_ENABLED
/* Audio is decoded and played by audio_thread_func() */
#elif defined(CONFIG_LIBLC3)

static void stream_recv_lc3_codec(struct bt_bap_stream *stream,
				  const struct bt_iso_recv_info *info,
				  struct net_buf *buf)
{
	static int16_t audio_buf[MAX_NUM_SAMPLES];

	struct audio_sink *sink_stream = CONTAINER_OF(stream, struct audio_sink, stream);
	const bool valid_data = (info->flags & BT_ISO_FLAGS_VALID) != 0;
	const int octets_per_frame = buf->len / frames_per_sdu;

	if (valid_data) {
		sink_stream->recv_cnt++;

		if (CONFIG_INFO_REPORTING_INTERVAL > 0 &&
		    (sink_stream->recv_cnt % CONFIG_INFO_REPORTING_INTERVAL) == 0U) {
			printk("Incoming audio on stream %p len %u (%zu)\n", stream, buf->len,
			       sink_stream->recv_cnt);
		}
	} else {
		printk("Bad packet: 0x%02X\n", info->flags);
	}

	if (lc3_decoder == NULL) {
		printk("LC3 decoder not setup, cannot decode data.\n");
		return;
	}

	for (int i = 0; i < frames_per_sdu; i++) {
		/* Passing NULL performs PLC */
		const int err = lc3_decode(
			lc3_decoder, valid_data ? net_buf_pull_mem(buf, octets_per_frame) : NULL,
			octets_per_frame, LC3_PCM_FORMAT_S16, audio_buf, 1);

		if (err == 1) {
			printk("[%d]: Decoder performed PLC\n", i);
		} else if (err < 0) {
			printk("[%d]: Decoder failed - wrong parameters?: %d\n", i, err);
		}
	}
}

#else

static void stream_recv(struct bt_bap_stream *stream,
			const struct bt_iso_recv_info *info,
			struct net_buf *buf)
{
	if (info->flags & BT_ISO_FLAGS_VALID) {
		struct audio_sink *sink_stream = CONTAINER_OF(stream, struct audio_sink, stream);

		sink_stream->recv_cnt++;

		if (CONFIG_INFO_REPORTING_INTERVAL > 0 &&
		    (sink_stream->recv_cnt % CONFIG_INFO_REPORTING_INTERVAL) == 0U) {
			printk("Incoming audio on stream %p len %u\n", stream, buf->len);
		}
	}
}

#endif

static void stream_recv_cb(struct bt_bap_stream *stream, const struct bt_iso_recv_info *info,
			   struct net_buf *buf)
{
	struct audio_sink *sink_stream = CONTAINER_OF(stream, struct audio_sink, stream);

	sdu_metrics_update(sink_stream, info, buf);

#if AUDIO_PLAYBACK_ENABLED
	stream_recv_tlv320(sink_stream, info, buf);
#elif defined(CONFIG_LIBLC3)
	stream_recv_lc3_codec(stream, info, buf);
#else
	stream_recv(stream, info, buf);
#endif
}

static void stream_connected_cb(struct bt_bap_stream *stream)
{
	printk("Audio Stream %p connected\n", stream);
}

static void stream_disconnected_cb(struct bt_bap_stream *stream, uint8_t reason)
{
	printk("Audio Stream %p disconnected with reason 0x%02X\n", stream, reason);
}

static void stream_stopped(struct bt_bap_stream *stream, uint8_t reason)
{
	printk("Audio Stream %p stopped with reason 0x%02X\n", stream, reason);

#if AUDIO_PLAYBACK_ENABLED
	if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
		dac_sink_stop(CONTAINER_OF(stream, struct audio_sink, stream));
	}
#endif /* AUDIO_PLAYBACK_ENABLED */

	if (IS_ENABLED(CONFIG_BT_AUDIO_TX) && stream_dir(stream) == BT_AUDIO_DIR_SOURCE) {
		const int err = stream_tx_unregister(stream);

		if (err != 0) {
			printk("Failed to register stream %p for TX: %d\n", stream, err);
		}
	}
}

static void stream_started(struct bt_bap_stream *stream)
{
	struct bt_iso_info info;
	int err;

	err = bt_iso_chan_get_info(stream->iso, &info);
	__ASSERT(err == 0, "Failed to get ISO chan info: %d", err);

	printk("Audio Stream %p started  with CIG_ID %u and CIS_ID %u\n", stream,
	       info.unicast.cig_id, info.unicast.cis_id);
	printk("  ISO interval %u us, C to P latency %u us, P to C latency %u us\n",
	       info.iso_interval * 1250U, info.unicast.central.latency,
	       info.unicast.peripheral.latency);

	if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
		struct audio_sink *sink_stream = CONTAINER_OF(stream, struct audio_sink, stream);

		sink_stream->recv_cnt = 0U;
		sdu_metrics_reset(sink_stream);

#if AUDIO_PLAYBACK_ENABLED
		dac_sink_start(sink_stream, stream);
#endif /* AUDIO_PLAYBACK_ENABLED */
	} else if (IS_ENABLED(CONFIG_BT_AUDIO_TX)) {
		const int err = stream_tx_register(stream);

		if (err != 0) {
			printk("Failed to register stream %p for TX: %d\n", stream, err);
		}
	}
}

static void stream_enabled_cb(struct bt_bap_stream *stream)
{
	/* The unicast server is responsible for starting sink ASEs after the
	 * client has enabled them.
	 */
	if (stream_dir(stream) == BT_AUDIO_DIR_SINK) {
		const int err = bt_bap_stream_start(stream);

		if (err != 0) {
			printk("Failed to start stream %p: %d", stream, err);
		}
	}
}

static struct bt_bap_stream_ops stream_ops = {
	.connected = stream_connected_cb,
	.disconnected = stream_disconnected_cb,
	.recv = stream_recv_cb,
	.stopped = stream_stopped,
	.started = stream_started,
	.enabled = stream_enabled_cb,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0) {
		printk("Failed to connect to %s %u %s\n", bt_conn_dst_str(conn),
		       err, bt_hci_err_to_str(err));

		default_conn = NULL;
		return;
	}

	printk("Connected: %s\n", bt_conn_dst_str(conn));
	default_conn = bt_conn_ref(conn);
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

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static struct bt_pacs_cap cap_sink = {
	.codec_cap = &lc3_codec_cap,
};

static struct bt_pacs_cap cap_source = {
	.codec_cap = &lc3_codec_cap_src,
};

static int set_location(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SNK_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SINK,
					   (BT_AUDIO_LOCATION_FRONT_LEFT |
					    BT_AUDIO_LOCATION_FRONT_RIGHT));
		if (err != 0) {
			printk("Failed to set sink location (err %d)\n", err);
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC_LOC)) {
		err = bt_pacs_set_location(BT_AUDIO_DIR_SOURCE,
					   BT_AUDIO_LOCATION_FRONT_CENTER);
		if (err != 0) {
			printk("Failed to set source location (err %d)\n", err);
			return err;
		}
	}

	printk("Location successfully set\n");

	return 0;
}

static int set_supported_contexts(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SNK)) {
		err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SINK,
						     AVAILABLE_SINK_CONTEXT);
		if (err != 0) {
			printk("Failed to set sink supported contexts (err %d)\n",
			       err);

			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
		err = bt_pacs_set_supported_contexts(BT_AUDIO_DIR_SOURCE,
						     AVAILABLE_SOURCE_CONTEXT);
		if (err != 0) {
			printk("Failed to set source supported contexts (err %d)\n",
			       err);

			return err;
		}
	}

	printk("Supported contexts successfully set\n");

	return 0;
}

static int set_available_contexts(void)
{
	int err;

	if (IS_ENABLED(CONFIG_BT_PAC_SNK)) {
		err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SINK,
						     AVAILABLE_SINK_CONTEXT);
		if (err != 0) {
			printk("Failed to set sink available contexts (err %d)\n", err);
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_BT_PAC_SRC)) {
		err = bt_pacs_set_available_contexts(BT_AUDIO_DIR_SOURCE,
						     AVAILABLE_SOURCE_CONTEXT);
		if (err != 0) {
			printk("Failed to set source available contexts (err %d)\n", err);
			return err;
		}
	}

	printk("Available contexts successfully set\n");
	return 0;
}


int main(void)
{
	struct bt_le_ext_adv *adv;
	const struct bt_pacs_register_param pacs_param = {
		.snk_pac = true,
		.snk_loc = true,
		.src_pac = true,
		.src_loc = true,
	};
	int err;

#if TLV320_AUDIO_OUTPUT_ENABLED
	err = clocks_start();
	if (err != 0) {
		return 0;
	}

	err = tlv320_setup();
	if (err != 0) {
		return 0;
	}

	err = audio_i2s_start();
	if (err != 0) {
		return 0;
	}

#if defined(CONFIG_TLV320_MIC)
	/* Encode the TLV320 microphone on the source stream */
	stream_lc3_pcm_source_set(mic_pcm_get);
#endif /* CONFIG_TLV320_MIC */
#endif /* TLV320_AUDIO_OUTPUT_ENABLED */

	err = bt_enable(NULL);
	if (err != 0) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");

	err = bt_pacs_register(&pacs_param);
	if (err != 0) {
		printk("Could not register PACS (err %d)\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_BT_AUDIO_TX)) {
		stream_tx_init();
	}

#if defined(CONFIG_USE_USB_AUDIO)
	err = usb_audio_init();
	if (err != 0) {
		printk("USB audio init failed (err %d)\n", err);
		return 0;
	}

	/* Encode the USB speaker audio on the source stream */
	stream_lc3_pcm_source_set(usb_audio_pcm_get);
#endif /* CONFIG_USE_USB_AUDIO */

	err = bt_bap_unicast_server_register(&param);
	if (err != 0) {
		printk("Could not register unicast server (err %d)\n", err);
		return 0;
	}

	err = bt_bap_unicast_server_register_cb(&unicast_server_cb);
	if (err != 0) {
		printk("Could not register unicast server callbacks (err %d)\n", err);
		return 0;
	}

	err = bt_pacs_cap_register(BT_AUDIO_DIR_SINK, &cap_sink);
	if (err != 0) {
		printk("Could not register sink capability (err %d)\n", err);
		return 0;
	}

	err = bt_pacs_cap_register(BT_AUDIO_DIR_SOURCE, &cap_source);
	if (err != 0) {
		printk("Could not register source capability (err %d)\n", err);
		return 0;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(sink_streams); i++) {
		bt_bap_stream_cb_register(&sink_streams[i].stream, &stream_ops);
	}

	for (size_t i = 0U; i < ARRAY_SIZE(source_streams); i++) {
		bt_bap_stream_cb_register(&source_streams[i].stream,
					    &stream_ops);
	}

	err = set_location();
	if (err != 0) {
		return 0;
	}

	err = set_supported_contexts();
	if (err != 0) {
		return 0;
	}

	err = set_available_contexts();
	if (err != 0) {
		return 0;
	}

	/* Create a connectable advertising set */
	err = bt_le_ext_adv_create(BT_BAP_ADV_PARAM_CONN_QUICK, NULL, &adv);
	if (err != 0) {
		printk("Failed to create advertising set (err %d)\n", err);
		return 0;
	}

	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err != 0) {
		printk("Failed to set advertising data (err %d)\n", err);
		return 0;
	}

	while (true) {
		err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
		if (err != 0) {
			printk("Failed to start advertising set (err %d)\n", err);
			return 0;
		}

		printk("Advertising successfully started\n");

		err = k_sem_take(&sem_disconnected, K_FOREVER);
		if (err != 0) {
			printk("failed to take sem_disconnected (err %d)\n", err);
			return 0;
		}

		/* reset data */
		configured_source_stream_count = 0U;
	}
	return 0;
}
