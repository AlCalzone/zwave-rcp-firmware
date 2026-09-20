#ifndef RADIO_H
#define RADIO_H

#include <stdbool.h>
#include <stdint.h>
#include "serial_api.h"

/// Radio backend interface
///
/// The serial API protocol layer (serial_api.c) drives the radio through
/// these functions only. Each supported radio library implements them:
///
/// - Silicon Labs RAIL on EFR32ZG23: app_process.c
/// - Trident IoT ZPAL on T32CZ20: trident/app/radio_zpal.c
///
/// Responses and callbacks towards the host are sent by the implementations
/// through the respond_cmd_* / callback_cmd_* / notify_receive functions
/// declared in serial_api.h.

/// @brief Report the radio library and its version for FUNC_ID_GET_FIRMWARE_INFO
void radio_get_library_info(radio_library_t *library, uint8_t *major, uint8_t *minor, uint8_t *patch);

/// @brief Change the region of the radio and describe its channels
/// @return false when the region or channel configuration is not supported
bool radio_set_region(zwave_region_t region, zwave_channel_cfg_t channel_cfg, uint8_t *num_channels, channel_info_t *channels);

/// @brief Read the radio's region, channel configuration and channel list
/// @note num_channels is left untouched when the active region is unknown
void radio_get_region(zwave_region_t *region, zwave_channel_cfg_t *channel_cfg, uint8_t *num_channels, channel_info_t *channels);

/// @brief Report the transmit power range the radio supports, in deci-dBm
void radio_get_tx_power_range(int16_t *min_deci_dbm, int16_t *max_deci_dbm);

/// @brief Queue raw data for transmission over radio
/// @param power_deci_dbm Transmit power in deci-dBm, coerced by the radio to the channel's maximum
/// @param flags Bitmask of TRANSMIT_FLAG_*
/// @param replacements Validated OFFSET | SOURCE pairs to patch into data right before the transmit
///
/// Answers FUNC_ID_TRANSMIT with respond_cmd_transmit() and reports the
/// outcome of a queued transmit with callback_cmd_transmit().
void radio_transmit(uint8_t channel, int16_t power_deci_dbm, uint8_t flags, uint8_t *data, uint32_t len, const uint8_t *replacements, uint8_t num_replacements);

/// @brief Measure the noise floor on a channel and restart RX, returning
/// NOISE_FLOOR_NOT_AVAILABLE when the radio is busy or the measurement failed
int8_t radio_measure_noise_floor_cmd(uint8_t channel);

/// @brief Start a wakeup beam that repeats data back to back for fragment_duration_ms per fragment
/// @param power_deci_dbm Transmit power in deci-dBm, coerced by the radio to the channel's maximum
/// @param fragment_period_ms Spacing between fragment starts, ignored when num_fragments is 1
/// @param channels Channel per fragment, indexed by the fragment number modulo num_channels
///
/// Answers FUNC_ID_TRANSMIT_BEAM with respond_cmd_transmit_beam() and
/// reports the end of a started beam with callback_cmd_transmit_beam().
void radio_transmit_beam(
    int16_t power_deci_dbm,
    uint8_t num_fragments,
    uint16_t fragment_duration_ms,
    uint16_t fragment_period_ms,
    uint8_t num_channels,
    const uint8_t *channels,
    const uint8_t *data,
    uint8_t data_len);

/// @brief Stop an ongoing beam and return the radio to RX
void radio_abort_beam(void);

#endif // RADIO_H
