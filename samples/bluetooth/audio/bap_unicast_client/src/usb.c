/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* USB Audio Class 2 headset
 *
 * Speaker: 48 kHz stereo from the USB host is split into per-channel ring buffers that the LC3
 * encoder reads from, decimated to the stream sampling rate.
 *
 * Microphone: decoded 48 kHz PCM is queued in a ring buffer and sent to the USB host on each
 * USB Start of Frame.
 *
 * The clock is declared SOF-synchronized. The drift between the USB and Bluetooth clocks is
 * corrected by dropping or repeating one sample frame when a ring buffer fill level leaves its
 * window.
 *
 * Each ring buffer has a single producer and a single consumer, so no lock is needed and
 * interrupts are never disabled while audio is copied. The USB stack callbacks run in the USB
 * device thread and the encoder callback in the Bluetooth TX thread. Ring buffers are flushed
 * only by their consumer.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/usb/usb_buf.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/usb/usbd.h>

#include <sample_usbd.h>

#include "usb.h"

LOG_MODULE_REGISTER(usb_audio, LOG_LEVEL_INF);

#define USB_FRAME_SAMPLES     (USB_SAMPLE_RATE_HZ / 1000U)
/* Synchronous endpoints may carry one extra sample frame */
#define USB_MAX_FRAME_SAMPLES (USB_FRAME_SAMPLES + 1U)
#define USB_SPK_CHANNELS      2U
#define USB_MIC_CHANNELS                                                                           \
	(DT_PROP(DT_NODELABEL(mic_input), front_left) +                                            \
	 DT_PROP(DT_NODELABEL(mic_input), front_right))
#define USB_SPK_MAX_SIZE      (USB_MAX_FRAME_SAMPLES * USB_SPK_CHANNELS * sizeof(int16_t))
#define USB_MIC_FRAME_SIZE    (USB_FRAME_SAMPLES * USB_MIC_CHANNELS * sizeof(int16_t))

/* Ring buffer levels in milliseconds of audio */
#define RING_MS               48U
#define PRIME_MS              20U
#define DRIFT_LOW_MS          5U
#define DRIFT_HIGH_MS         35U
#define MS_TO_SAMPLES(ms)     ((ms) * USB_FRAME_SAMPLES)

#define SPK_RING_SIZE         (MS_TO_SAMPLES(RING_MS) * sizeof(int16_t))
#define MIC_RING_SIZE         (MS_TO_SAMPLES(RING_MS) * USB_MIC_CHANNELS * sizeof(int16_t))

#define OUT_TERMINAL_ID       UAC2_ENTITY_ID(DT_NODELABEL(out_terminal))
#define IN_TERMINAL_ID        UAC2_ENTITY_ID(DT_NODELABEL(in_terminal))

BUILD_ASSERT(USB_MIC_CHANNELS == 1U || USB_MIC_CHANNELS == 2U,
	     "USB microphone must be mono or stereo");

/* One buffer for USB to receive into and two more for rapid terminal enable/disable cycles */
K_MEM_SLAB_DEFINE_STATIC(usb_out_pool, ROUND_UP(USB_SPK_MAX_SIZE, UDC_BUF_GRANULARITY), 3U,
			 UDC_BUF_ALIGN);
K_MEM_SLAB_DEFINE_STATIC(usb_in_pool, ROUND_UP(USB_MIC_FRAME_SIZE, UDC_BUF_GRANULARITY), 4U,
			 UDC_BUF_ALIGN);

RING_BUF_DECLARE(spk_ring_left, SPK_RING_SIZE);
RING_BUF_DECLARE(spk_ring_right, SPK_RING_SIZE);
RING_BUF_DECLARE(mic_ring, MIC_RING_SIZE);

static struct ring_buf *const spk_rings[USB_SPK_CHANNELS] = {&spk_ring_left, &spk_ring_right};
/* Speaker ring buffer flush request, handled by the encoder (consumer) */
static atomic_t spk_flush;
static bool spk_primed[USB_SPK_CHANNELS];
static bool mic_primed;
static volatile bool spk_enabled;
static volatile bool mic_enabled;

static struct {
	uint32_t spk_underrun;
	uint32_t spk_overrun;
	uint32_t spk_drop;
	uint32_t spk_repeat;
	uint32_t mic_underrun;
	uint32_t mic_overrun;
	uint32_t mic_drop;
	uint32_t mic_repeat;
	uint32_t send_err;
} stats;

/* Called by the speaker ring buffer consumer only */
static void spk_flush_handle(void)
{
	if (!atomic_cas(&spk_flush, 1, 0)) {
		return;
	}

	for (size_t i = 0U; i < USB_SPK_CHANNELS; i++) {
		ring_buf_consume(spk_rings[i], ring_buf_size_get(spk_rings[i]));
		spk_primed[i] = false;
	}
}

/* Called by the microphone ring buffer consumer only, in the USB device thread */
static void mic_flush(void)
{
	ring_buf_consume(&mic_ring, ring_buf_size_get(&mic_ring));
	mic_primed = false;
}

static void terminal_update_cb(const struct device *dev, uint8_t terminal, bool enabled,
			       bool microframes, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(microframes);
	ARG_UNUSED(user_data);

	if (terminal == OUT_TERMINAL_ID) {
		spk_enabled = enabled;
		atomic_set(&spk_flush, 1);
	} else if (terminal == IN_TERMINAL_ID) {
		mic_enabled = enabled;
		mic_flush();
	}

	LOG_INF("USB %s %s", terminal == OUT_TERMINAL_ID ? "speaker" : "microphone",
		enabled ? "enabled" : "disabled");
}

static void *get_recv_buf_cb(const struct device *dev, uint8_t terminal, uint16_t size,
			     void *user_data)
{
	void *buf = NULL;

	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);
	ARG_UNUSED(user_data);

	if (!spk_enabled || size > USB_SPK_MAX_SIZE) {
		return NULL;
	}

	if (k_mem_slab_alloc(&usb_out_pool, &buf, K_NO_WAIT) != 0) {
		return NULL;
	}

	return buf;
}

static void data_recv_cb(const struct device *dev, uint8_t terminal, void *buf, uint16_t size,
			 void *user_data)
{
	/* One spare sample frame for drift correction */
	int16_t chan_pcm[USB_SPK_CHANNELS][USB_MAX_FRAME_SAMPLES + 1U];
	const int16_t *pcm = buf;
	size_t frames;
	uint32_t fill;

	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);
	ARG_UNUSED(user_data);

	if (buf == NULL) {
		return;
	}

	frames = MIN(size / (USB_SPK_CHANNELS * sizeof(int16_t)), USB_MAX_FRAME_SAMPLES);
	if (!spk_enabled || frames == 0U) {
		k_mem_slab_free(&usb_out_pool, buf);
		return;
	}

	for (size_t i = 0U; i < frames; i++) {
		for (size_t ch = 0U; ch < USB_SPK_CHANNELS; ch++) {
			chan_pcm[ch][i] = pcm[i * USB_SPK_CHANNELS + ch];
		}
	}

	k_mem_slab_free(&usb_out_pool, buf);

	fill = ring_buf_size_get(spk_rings[0]) / sizeof(int16_t);
	if (fill > MS_TO_SAMPLES(DRIFT_HIGH_MS)) {
		frames--;
		stats.spk_drop++;
	} else if (spk_primed[0] && fill < MS_TO_SAMPLES(DRIFT_LOW_MS)) {
		for (size_t ch = 0U; ch < USB_SPK_CHANNELS; ch++) {
			chan_pcm[ch][frames] = chan_pcm[ch][frames - 1U];
		}

		frames++;
		stats.spk_repeat++;
	}

	for (size_t ch = 0U; ch < USB_SPK_CHANNELS; ch++) {
		const uint32_t len = frames * sizeof(int16_t);

		if (ring_buf_put(spk_rings[ch], (const uint8_t *)chan_pcm[ch], len) != len) {
			stats.spk_overrun++;
		}
	}
}

/* Read samples of one speaker channel and decimate them by averaging */
static bool spk_read(size_t ch, uint32_t ratio, int16_t *pcm, size_t samples)
{
	const uint32_t needed = samples * ratio * sizeof(int16_t);
	const uint32_t prime = MS_TO_SAMPLES(PRIME_MS) * sizeof(int16_t);
	struct ring_buf *ring = spk_rings[ch];
	int16_t in[USB_FRAME_SAMPLES];
	size_t out = 0U;

	spk_flush_handle();

	if (!spk_primed[ch]) {
		if (ring_buf_size_get(ring) < MAX(needed, prime)) {
			return false;
		}

		spk_primed[ch] = true;
	}

	if (ring_buf_size_get(ring) < needed) {
		spk_primed[ch] = false;
		stats.spk_underrun++;
		return false;
	}

	while (out < samples) {
		const size_t chunk = MIN(samples - out, ARRAY_SIZE(in) / ratio);

		(void)ring_buf_get(ring, (uint8_t *)in, chunk * ratio * sizeof(int16_t));

		for (size_t i = 0U; i < chunk; i++) {
			int32_t sum = 0;

			for (uint32_t j = 0U; j < ratio; j++) {
				sum += in[i * ratio + j];
			}

			pcm[out++] = (int16_t)(sum / (int32_t)ratio);
		}
	}

	return true;
}

bool usb_audio_pcm_get(enum bt_audio_location chan_allocation, uint32_t freq_hz, int16_t *pcm,
		       size_t samples)
{
	const size_t size = samples * sizeof(int16_t);
	uint32_t ratio;
	bool ok;

	if (freq_hz == 0U || (USB_SAMPLE_RATE_HZ % freq_hz) != 0U || !spk_enabled) {
		(void)memset(pcm, 0, size);
		return false;
	}

	ratio = USB_SAMPLE_RATE_HZ / freq_hz;

	if (chan_allocation == BT_AUDIO_LOCATION_FRONT_LEFT) {
		ok = spk_read(0U, ratio, pcm, samples);
	} else if (chan_allocation == BT_AUDIO_LOCATION_FRONT_RIGHT) {
		ok = spk_read(1U, ratio, pcm, samples);
	} else {
		static int16_t right[USB_SAMPLE_RATE_HZ / 100U];

		ok = samples <= ARRAY_SIZE(right) && spk_read(0U, ratio, pcm, samples) &&
		     spk_read(1U, ratio, right, samples);
		if (ok) {
			for (size_t i = 0U; i < samples; i++) {
				pcm[i] = (int16_t)(((int32_t)pcm[i] + right[i]) / 2);
			}
		}
	}

	if (!ok) {
		(void)memset(pcm, 0, size);
	}

	return ok;
}

void usb_audio_pcm_put(const int16_t *pcm, size_t frames, uint8_t chan_cnt)
{
	int16_t frame[USB_MIC_CHANNELS];
	uint32_t fill;

	if (!mic_enabled || frames == 0U || (chan_cnt != 1U && chan_cnt != 2U)) {
		return;
	}

	fill = ring_buf_size_get(&mic_ring) / (USB_MIC_CHANNELS * sizeof(int16_t));
	if (fill > MS_TO_SAMPLES(DRIFT_HIGH_MS)) {
		frames--;
		stats.mic_drop++;
	}

	for (size_t i = 0U; i < frames; i++) {
		const int16_t *in = &pcm[i * chan_cnt];

		if (USB_MIC_CHANNELS == chan_cnt) {
			(void)memcpy(frame, in, sizeof(frame));
		} else if (USB_MIC_CHANNELS == 2U) {
			frame[0] = in[0];
			frame[USB_MIC_CHANNELS - 1U] = in[0];
		} else {
			frame[0] = (int16_t)(((int32_t)in[0] + in[1]) / 2);
		}

		if (ring_buf_put(&mic_ring, (const uint8_t *)frame, sizeof(frame)) !=
		    sizeof(frame)) {
			stats.mic_overrun++;
			break;
		}

		if (i == frames - 1U && mic_primed && fill < MS_TO_SAMPLES(DRIFT_LOW_MS)) {
			if (ring_buf_put(&mic_ring, (const uint8_t *)frame, sizeof(frame)) ==
			    sizeof(frame)) {
				stats.mic_repeat++;
			}
		}
	}
}

/* Called every 1 ms, sends one frame of microphone audio to the USB host */
static void sof_cb(const struct device *dev, void *user_data)
{
	void *buf;
	int err;

	ARG_UNUSED(user_data);

	if (!mic_enabled) {
		return;
	}

	if (k_mem_slab_alloc(&usb_in_pool, &buf, K_NO_WAIT) != 0) {
		stats.send_err++;
		return;
	}

	if (!mic_primed &&
	    ring_buf_size_get(&mic_ring) >= MS_TO_SAMPLES(PRIME_MS) * USB_MIC_CHANNELS *
						    sizeof(int16_t)) {
		mic_primed = true;
	}

	if (mic_primed && ring_buf_size_get(&mic_ring) >= USB_MIC_FRAME_SIZE) {
		(void)ring_buf_get(&mic_ring, buf, USB_MIC_FRAME_SIZE);
	} else {
		if (mic_primed) {
			mic_primed = false;
			stats.mic_underrun++;
		}

		(void)memset(buf, 0, USB_MIC_FRAME_SIZE);
	}

	err = usbd_uac2_send(dev, IN_TERMINAL_ID, buf, USB_MIC_FRAME_SIZE);
	if (err != 0) {
		stats.send_err++;
		k_mem_slab_free(&usb_in_pool, buf);
	}
}

static void buf_release_cb(const struct device *dev, uint8_t terminal, void *buf, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(terminal);
	ARG_UNUSED(user_data);

	k_mem_slab_free(&usb_in_pool, buf);
}

void usb_audio_stats_log(void)
{
	LOG_INF("USB spk %s: under %u over %u drop %u rep %u | mic %s: under %u over %u drop %u "
		"rep %u | send err %u",
		spk_enabled ? "on" : "off", stats.spk_underrun, stats.spk_overrun, stats.spk_drop,
		stats.spk_repeat, mic_enabled ? "on" : "off", stats.mic_underrun,
		stats.mic_overrun, stats.mic_drop, stats.mic_repeat, stats.send_err);
}

int usb_audio_init(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(uac2_headset));
	static const struct uac2_ops ops = {
		.sof_cb = sof_cb,
		.terminal_update_cb = terminal_update_cb,
		.get_recv_buf = get_recv_buf_cb,
		.data_recv_cb = data_recv_cb,
		.buf_release_cb = buf_release_cb,
	};
	struct usbd_context *usbd;
	static bool initialized;
	int err;

	if (initialized) {
		return -EALREADY;
	}

	if (!device_is_ready(dev)) {
		LOG_ERR("USB headset device not ready");
		return -EIO;
	}

	usbd_uac2_set_ops(dev, &ops, NULL);

	usbd = sample_usbd_init_device(NULL);
	if (usbd == NULL) {
		return -ENODEV;
	}

	err = usbd_enable(usbd);
	if (err != 0) {
		return err;
	}

	initialized = true;
	LOG_INF("USB headset initialized, %u channel microphone", USB_MIC_CHANNELS);

	return 0;
}
