/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef USB_H
#define USB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/audio/audio.h>

/** Sampling rate of the USB headset speaker and microphone */
#define USB_SAMPLE_RATE_HZ 48000U

/**
 * @brief Initialize and enable the USB headset
 *
 * @retval 0 Success
 * @retval -EALREADY Already initialized
 * @retval -EIO USB Audio 2 device not ready
 * @retval -ENODEV USB device could not be initialized
 */
int usb_audio_init(void);

/**
 * @brief Get speaker samples received from the USB host
 *
 * Signature matches @ref stream_lc3_pcm_get_t. Front left and front right return the respective
 * channel; any other location returns the average of both channels. The 48 kHz USB audio is
 * decimated to @p freq_hz, which must divide 48 kHz.
 *
 * @param chan_allocation Audio location of the channel to provide
 * @param freq_hz Sampling rate of the requested samples
 * @param pcm Buffer to fill with 16-bit mono samples
 * @param samples Number of samples to provide
 *
 * @retval true @p pcm filled with audio
 * @retval false No audio available, @p pcm filled with silence
 */
bool usb_audio_pcm_get(enum bt_audio_location chan_allocation, uint32_t freq_hz, int16_t *pcm,
		       size_t samples);

/**
 * @brief Queue 48 kHz samples for the USB headset microphone
 *
 * Mono input is duplicated when the microphone is stereo, stereo input is downmixed when the
 * microphone is mono.
 *
 * @param pcm Interleaved 16-bit samples at 48 kHz
 * @param frames Number of sample frames in @p pcm
 * @param chan_cnt Number of interleaved channels in @p pcm, 1 or 2
 */
void usb_audio_pcm_put(const int16_t *pcm, size_t frames, uint8_t chan_cnt);

/** @brief Log the USB audio counters */
void usb_audio_stats_log(void);

#endif /* USB_H */
