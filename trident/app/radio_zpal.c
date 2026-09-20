/// Radio backend on the Z-Wave Platform Abstraction Layer (ZPAL) of the
/// Trident IoT SDK, for the T32CZ20 radio.
///
/// ZPAL configures the hardware from the SDK's region tables. Its raw transmit
/// API does not expose those tables. Each transmission still requires explicit
/// PHY parameters. This backend mirrors the metadata needed by the Serial API.
///
/// Implements radio.h. Everything here runs on the RCP task, except the two
/// ZPAL completion callbacks which only record the event and wake the task.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

#include <zpal_radio.h>
#include <zpal_radio_private.h>
#include <zpal_radio_utils.h>

#include "common.h"
#include "serial_api.h"
#include "radio.h"
#include "rcp_app.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

#ifndef RCP_DEFAULT_REGION
/// Region the radio starts in until the host configures one
#define RCP_DEFAULT_REGION ZWAVE_REGION_EU
#endif

#ifndef RCP_TX_MAX_POWER_DECI_DBM
/// Transmit power class of the module. The Trident IoT sample applications
/// default to the 14 dBm tables; a module set up for 20 dBm can be built
/// with RCP_TX_MAX_POWER_DECI_DBM=200.
#define RCP_TX_MAX_POWER_DECI_DBM ZW_TX_POWER_14DBM
#endif

/// Transmit power used until the host sets one, matching the initial PA
/// power of the EFR32 build (SL_RAIL_UTIL_PA_POWER_DECI_DBM)
#define RCP_INITIAL_TX_POWER_DECI_DBM 100

/// The ZPAL driver copies a frame into a 200 byte transmit buffer without a
/// bounds check, so refuse anything larger
#define RCP_TX_MAX_FRAME_LEN 200

/// Longest single beam repeat train handed to the radio. A running train
/// cannot be stopped, so this bounds the latency of FUNC_ID_ABORT_BEAM and
/// of a region change. Fragments longer than this are chained back to back.
#define BEAM_CHUNK_MAX_MS 250

/// Beam preamble lengths in bytes, G.9959 Table 7-10 "Minimum Preamble length"
/// for R2 (20 bytes) and R3 in channel configuration 3 (8 bytes), and Z-Wave
/// Long Range PHY and MAC Layer Specification Table 5-10 "Required Preamble
/// length" for LR (8 bytes / 16 symbols).
#define BEAM_PREAMBLE_BYTES_R2 20
#define BEAM_PREAMBLE_BYTES_R3 8
#define BEAM_PREAMBLE_BYTES_LR 8
/// A rate that carries no beam preamble length in the tables above
#define BEAM_PREAMBLE_BYTES_NONE 0

/// Frame preamble lengths in bytes, as the Z-Wave stack transmits them
#define PREAMBLE_BYTES_9K6 10
#define PREAMBLE_BYTES_40K 20
#define PREAMBLE_BYTES_100K_2CH 40
#define PREAMBLE_BYTES_100K_3CH 24
#define PREAMBLE_BYTES_LR 40

#define PREAMBLE_BYTE_CLASSIC 0x55
/// Spread by the LR PHY, so the transmitted symbols differ from this value
#define PREAMBLE_BYTE_LR 0x00
#define START_OF_FRAME_CLASSIC 0xF0
#define START_OF_FRAME_LR 0x5E

/// Offset of the length field in every Z-Wave MPDU header (2CH, 3CH and LR)
#define MPDU_LENGTH_OFFSET 7

// -----------------------------------------------------------------------------
// Region description
// -----------------------------------------------------------------------------

/// Channel plan of a region as the ZPAL radio driver of the T32CZ20
/// implements it. ZPAL offers no API to read the channel frequencies, so they
/// are mirrored from ZW_region_rf_settings.c of the SDK.
typedef struct
{
  uint8_t num_channels;
  /// Serial API channel index to ZPAL channel id
  uint8_t zpal_channel[RADIO_MAX_CHANNELS];
  channel_info_t channels[RADIO_MAX_CHANNELS];
  /// G.9959 channel configuration 3: three channels, all at 100 kbps
  bool is_channel_cfg_3;
  zpal_radio_lr_channel_config_t lr_channel_cfg;
} region_desc_t;

/// The region the radio is configured for
static zwave_region_t active_region = ZWAVE_REGION_UNKNOWN;
static zwave_channel_cfg_t active_channel_cfg = CHANNEL_CFG_CLASSIC;
static region_desc_t active_desc = {0};

// -----------------------------------------------------------------------------
// Radio state
// -----------------------------------------------------------------------------

static uint8_t network_home_id[4] = {0};
static zpal_radio_network_stats_t network_stats = {0};

/// A frame or beam transmit was handed to the radio and has not completed yet
static volatile bool radio_tx_busy = false;
/// The event the radio reported for the last transmit
static volatile zpal_radio_event_t last_tx_event = ZPAL_RADIO_EVENT_NONE;

/// A FUNC_ID_TRANSMIT is in flight, radio_tx_busy belongs to it
static bool frame_tx_active = false;

/// Power the radio transmits with when the host passes TX_POWER_UNCHANGED
static int8_t last_tx_power_dbm = RCP_INITIAL_TX_POWER_DECI_DBM / 10;

/// The beam the host requested with FUNC_ID_TRANSMIT_BEAM
static struct
{
  /// A beam is running, radio_tx_busy belongs to it while transmitting
  bool active;
  /// A repeat train is on air
  bool transmitting;
  /// FUNC_ID_ABORT_BEAM arrived while a train was on air
  bool abort_requested;
  uint8_t num_fragments;
  uint8_t fragment_index;
  TickType_t fragment_duration;
  TickType_t fragment_period;
  uint8_t num_channels;
  uint8_t channels[RADIO_MAX_CHANNELS];
  uint8_t data[BEAM_DATA_MAX_LEN];
  uint8_t data_len;
  int8_t power_dbm;
  /// Start of the current fragment, the reference the period counts from
  TickType_t fragment_start;
  /// End of the current fragment
  TickType_t fragment_end;
} beam = {0};

static StaticTimer_t beam_timer_buffer;
static TimerHandle_t beam_timer = NULL;

// -----------------------------------------------------------------------------
// Static function declarations
// -----------------------------------------------------------------------------

static bool describe_region(zwave_region_t region, zwave_channel_cfg_t channel_cfg, region_desc_t *desc);
static void rx_callback(zpal_radio_event_t event);
static void tx_callback(zpal_radio_event_t event);
static void region_change_callback(zpal_radio_event_t event);
static void radio_assert_callback(zpal_radio_event_t event);
static void receive_handler(zpal_radio_rx_parameters_t *rx_parameters, zpal_radio_receive_frame_t *frame);
static int8_t deci_dbm_to_dbm(int16_t deci_dbm);
static void tx_parameters_for_channel(uint8_t channel, zpal_radio_transmit_parameter_t *params);
static uint8_t beam_preamble_bytes(uint8_t channel);
static uint32_t channel_bitrate(uint8_t channel);
static tx_result_t tx_result_from_event(zpal_radio_event_t event);
static int8_t measure_noise_floor(uint8_t channel);
static uint8_t beam_fragment_channel(void);
static tx_result_t beam_start_train(void);
static void beam_arm_timer(TickType_t deadline);
static void beam_stop(void);
static void beam_end(tx_result_t result);
static void beam_timer_expired(TimerHandle_t timer);

// -----------------------------------------------------------------------------
// Init and events
// -----------------------------------------------------------------------------

void radio_zpal_init(void)
{
  beam_timer = xTimerCreateStatic("beam", 1, pdFALSE, NULL, beam_timer_expired, &beam_timer_buffer);

  region_desc_t desc;
  bool ok = describe_region(RCP_DEFAULT_REGION, CHANNEL_CFG_CLASSIC, &desc);
  (void)ok;

  zpal_radio_profile_t profile = {
      .region = (zpal_radio_region_t)RCP_DEFAULT_REGION,
      .wakeup = ZPAL_RADIO_WAKEUP_ALWAYS_LISTEN,
      .primary_lr_channel = ZPAL_RADIO_LR_CHANNEL_UNINITIALIZED,
      .lr_channel_auto_mode = false,
      .active_lr_channel_config = desc.lr_channel_cfg,
      // Lets the driver apply the LBT threshold of each region
      .listen_before_talk_threshold = ZPAL_RADIO_RSSI_NOT_AVAILABLE,
      .tx_power_max = RCP_TX_MAX_POWER_DECI_DBM,
      .tx_power_adjust = 0,
      .tx_power_max_lr = RCP_TX_MAX_POWER_DECI_DBM,
      .tx_power_min_lr = -60,
      .home_id = network_home_id,
      .rx_cb = rx_callback,
      .tx_cb = tx_callback,
      .region_change_cb = region_change_callback,
      .assert_cb = radio_assert_callback,
      .network_stats = &network_stats,
      .radio_debug_enable = false,
      .receive_handler_cb = receive_handler,
  };

  zpal_radio_init(&profile);

  active_region = RCP_DEFAULT_REGION;
  active_channel_cfg = CHANNEL_CFG_CLASSIC;
  active_desc = desc;

  // Every frame reaches the host, the radio filters nothing
  zpal_radio_network_id_filter_set(false);
  zpal_radio_start_receive();
}

/// Runs in interrupt context
static void rx_callback(zpal_radio_event_t event)
{
  switch (event)
  {
  case ZPAL_RADIO_EVENT_RX_COMPLETE:
    rcp_notify(RCP_EVENT_RF_RX);
    break;
  default:
    // Beams, aborted receptions and timeouts carry nothing for the host
    break;
  }
}

/// Runs in interrupt context
static void tx_callback(zpal_radio_event_t event)
{
  last_tx_event = event;
  radio_tx_busy = false;
  rcp_notify(RCP_EVENT_RF_TX_DONE);
}

static void region_change_callback(zpal_radio_event_t event)
{
  (void)event;
}

static void radio_assert_callback(zpal_radio_event_t event)
{
  (void)event;
}

void radio_zpal_handle_rx(void)
{
  // Pops one frame from the driver FIFO and hands it to receive_handler().
  // The driver raises another RX event while frames remain queued
  zpal_radio_get_last_received_frame();
}

/// Called by zpal_radio_get_last_received_frame() for each received frame
static void receive_handler(zpal_radio_rx_parameters_t *rx_parameters, zpal_radio_receive_frame_t *frame)
{
  uint8_t len = frame->frame_content_length;
  uint8_t *content = frame->frame_content;

  // ZPAL delivers the checksum with the frame, the EFR32 build does not.
  // The length field of every Z-Wave MPDU counts the checksum, so a frame
  // whose content is exactly as long as its length field still carries it.
  if (len > MPDU_LENGTH_OFFSET && content[MPDU_LENGTH_OFFSET] == len)
  {
    uint8_t checksum_len = (rx_parameters->speed == ZPAL_RADIO_SPEED_9600 || rx_parameters->speed == ZPAL_RADIO_SPEED_40K) ? 1 : 2;
    if (len > checksum_len)
    {
      len -= checksum_len;
    }
  }

  // Map the ZPAL channel id back to the channel index of the active region
  uint8_t channel = 0xff;
  for (uint8_t i = 0; i < active_desc.num_channels; i++)
  {
    if (active_desc.zpal_channel[i] == rx_parameters->channel_id)
    {
      channel = i;
      break;
    }
  }

  // ZPAL reports no link quality indicator
  notify_receive(content, len, rx_parameters->rssi, 0, channel);
}

void radio_zpal_handle_tx_done(void)
{
  zpal_radio_event_t event = last_tx_event;
  last_tx_event = ZPAL_RADIO_EVENT_NONE;

  if (beam.active && beam.transmitting)
  {
    beam.transmitting = false;

    if ((event & ZPAL_RADIO_EVENT_MASK) != ZPAL_RADIO_EVENT_TX_BEAM_COMPLETE)
    {
      beam_end(tx_result_from_event(event));
      return;
    }
    if (beam.abort_requested)
    {
      beam_end(TX_RESULT_ABORTED);
      return;
    }

    TickType_t now = xTaskGetTickCount();
    if ((int32_t)(beam.fragment_end - now) > 0)
    {
      // The fragment outlasts one repeat train, so start the next one
      tx_result_t result = beam_start_train();
      if (result != TX_RESULT_QUEUED)
      {
        beam_end(result);
      }
      return;
    }

    if (beam.fragment_index + 1 >= beam.num_fragments)
    {
      beam_end(TX_RESULT_COMPLETED);
      return;
    }

    // Between fragments the radio is back in RX, where the woken node's ack
    // arrives.
    // G.9959 §8.1.3.11: "The next beam fragment shall begin in the range
    // 190-200 ms measured from the beginning of the previous beam fragment."
    beam_arm_timer(beam.fragment_start + beam.fragment_period);
    return;
  }

  if (frame_tx_active)
  {
    frame_tx_active = false;
    callback_cmd_transmit(tx_result_from_event(event));
  }
  // Otherwise the train of a beam that was stopped early ran out. Nothing
  // waits for it.
}

// -----------------------------------------------------------------------------
// radio.h: information
// -----------------------------------------------------------------------------

void radio_get_library_info(radio_library_t *library, uint8_t *major, uint8_t *minor, uint8_t *patch)
{
  *library = RADIO_LIBRARY_ZPAL;
  // The Trident IoT SDK is versioned YEAR.MONTH.PATCH
  *major = (uint8_t)(RCP_TRIDENT_SDK_VERSION_YEAR % 100);
  *minor = (uint8_t)RCP_TRIDENT_SDK_VERSION_MONTH;
  *patch = (uint8_t)RCP_TRIDENT_SDK_VERSION_PATCH;
}

void radio_get_tx_power_range(int16_t *min_deci_dbm, int16_t *max_deci_dbm)
{
  int16_t min_dbm = INT16_MAX;
  int16_t max_dbm = INT16_MIN;

  // The driver keeps separate power tables per channel, report the union
  for (uint8_t i = 0; i < active_desc.num_channels; i++)
  {
    int16_t channel_min = zpal_radio_min_tx_power_get(active_desc.zpal_channel[i]);
    int16_t channel_max = zpal_radio_max_tx_power_get(active_desc.zpal_channel[i]);
    if (channel_min < min_dbm)
    {
      min_dbm = channel_min;
    }
    if (channel_max > max_dbm)
    {
      max_dbm = channel_max;
    }
  }

  if (active_desc.num_channels == 0)
  {
    min_dbm = 0;
    max_dbm = 0;
  }

  *min_deci_dbm = (int16_t)(min_dbm * 10);
  *max_deci_dbm = (int16_t)(max_dbm * 10);
}

// -----------------------------------------------------------------------------
// radio.h: region
// -----------------------------------------------------------------------------

/// @brief Describe a classic 2-channel region
static void describe_2ch(region_desc_t *desc, uint32_t freq_100k_khz, uint32_t freq_40k_9k6_khz)
{
  desc->num_channels = 3;
  desc->zpal_channel[0] = ZPAL_RADIO_ZWAVE_CHANNEL_0;
  desc->zpal_channel[1] = ZPAL_RADIO_ZWAVE_CHANNEL_1;
  desc->zpal_channel[2] = ZPAL_RADIO_ZWAVE_CHANNEL_2;
  desc->channels[0].freq = freq_100k_khz * 1000;
  desc->channels[0].baud = ZWAVE_BAUD_100k;
  desc->channels[1].freq = freq_40k_9k6_khz * 1000;
  desc->channels[1].baud = ZWAVE_BAUD_40k;
  desc->channels[2].freq = freq_40k_9k6_khz * 1000;
  desc->channels[2].baud = ZWAVE_BAUD_9k6;
  desc->is_channel_cfg_3 = false;
  desc->lr_channel_cfg = ZPAL_RADIO_LR_CH_CFG_NO_LR;
}

/// @brief Describe a 3-channel region (G.9959 channel configuration 3)
static void describe_3ch(region_desc_t *desc, uint32_t freq0_khz, uint32_t freq1_khz, uint32_t freq2_khz)
{
  desc->num_channels = 3;
  desc->zpal_channel[0] = ZPAL_RADIO_ZWAVE_CHANNEL_0;
  desc->zpal_channel[1] = ZPAL_RADIO_ZWAVE_CHANNEL_1;
  desc->zpal_channel[2] = ZPAL_RADIO_ZWAVE_CHANNEL_2;
  desc->channels[0].freq = freq0_khz * 1000;
  desc->channels[1].freq = freq1_khz * 1000;
  desc->channels[2].freq = freq2_khz * 1000;
  for (int i = 0; i < 3; i++)
  {
    desc->channels[i].baud = ZWAVE_BAUD_100k;
  }
  desc->is_channel_cfg_3 = true;
  desc->lr_channel_cfg = ZPAL_RADIO_LR_CH_CFG_NO_LR;
}

/// @brief Describe a Long Range region in one of its channel configurations
/// @return false for a channel configuration the region does not support
static bool describe_lr(region_desc_t *desc, zwave_channel_cfg_t channel_cfg, uint32_t freq_100k_khz, uint32_t freq_40k_9k6_khz, uint32_t freq_lr_a_khz, uint32_t freq_lr_b_khz)
{
  switch (channel_cfg)
  {
  case CHANNEL_CFG_CLASSIC_LR_A:
  case CHANNEL_CFG_CLASSIC_LR_B:
  {
    // Classic channels plus one LR channel. The driver numbers the primary
    // channel A as ZPAL channel 3 and the backup channel B as channel 4
    bool is_a = (channel_cfg == CHANNEL_CFG_CLASSIC_LR_A);
    describe_2ch(desc, freq_100k_khz, freq_40k_9k6_khz);
    desc->num_channels = 4;
    desc->zpal_channel[3] = is_a ? ZPAL_RADIO_ZWAVE_CHANNEL_3 : ZPAL_RADIO_ZWAVE_CHANNEL_4;
    desc->channels[3].freq = (is_a ? freq_lr_a_khz : freq_lr_b_khz) * 1000;
    desc->channels[3].baud = ZWAVE_BAUD_LR100k;
    desc->lr_channel_cfg = is_a ? ZPAL_RADIO_LR_CH_CFG1 : ZPAL_RADIO_LR_CH_CFG2;
    return true;
  }
  case CHANNEL_CFG_LR:
    // Both LR channels only, as an LR end device listens
    desc->num_channels = 2;
    desc->zpal_channel[0] = ZPAL_RADIO_ZWAVE_CHANNEL_3;
    desc->zpal_channel[1] = ZPAL_RADIO_ZWAVE_CHANNEL_4;
    desc->channels[0].freq = freq_lr_a_khz * 1000;
    desc->channels[0].baud = ZWAVE_BAUD_LR100k;
    desc->channels[1].freq = freq_lr_b_khz * 1000;
    desc->channels[1].baud = ZWAVE_BAUD_LR100k;
    desc->is_channel_cfg_3 = false;
    desc->lr_channel_cfg = ZPAL_RADIO_LR_CH_CFG3;
    return true;
  default:
    return false;
  }
}

/// @brief Look up the channel plan of a region, mirroring the ZPAL driver
static bool describe_region(zwave_region_t region, zwave_channel_cfg_t channel_cfg, region_desc_t *desc)
{
  memset(desc, 0, sizeof(*desc));

  switch (region)
  {
  case ZWAVE_REGION_EU:
    describe_2ch(desc, 869850, 868400);
    return true;
  case ZWAVE_REGION_US:
    describe_2ch(desc, 916000, 908400);
    return true;
  case ZWAVE_REGION_ANZ:
    describe_2ch(desc, 919800, 921400);
    return true;
  case ZWAVE_REGION_HK:
    describe_2ch(desc, 919800, 919800);
    return true;
  case ZWAVE_REGION_IN:
    describe_2ch(desc, 865200, 865200);
    return true;
  case ZWAVE_REGION_IL:
    describe_2ch(desc, 916000, 916000);
    return true;
  case ZWAVE_REGION_RU:
    describe_2ch(desc, 869000, 869000);
    return true;
  case ZWAVE_REGION_CN:
    describe_2ch(desc, 868400, 868400);
    return true;
  case ZWAVE_REGION_JP:
    describe_3ch(desc, 922500, 923900, 926300);
    return true;
  case ZWAVE_REGION_KR:
    describe_3ch(desc, 920900, 921700, 923100);
    return true;
  // For LR regions, the channel plan also depends on the channel configuration
  case ZWAVE_REGION_US_LR:
    return describe_lr(desc, channel_cfg, 916000, 908400, 912000, 920000);
  case ZWAVE_REGION_EU_LR:
    return describe_lr(desc, channel_cfg, 869850, 868400, 864000, 866000);
  default:
    return false;
  }
}

static void export_channel_info(const region_desc_t *desc, uint8_t *num_channels, channel_info_t *channels)
{
  *num_channels = desc->num_channels;
  for (uint8_t i = 0; i < desc->num_channels; i++)
  {
    channels[i] = desc->channels[i];
  }
}

bool radio_set_region(zwave_region_t region, zwave_channel_cfg_t channel_cfg, uint8_t *num_channels, channel_info_t *channels)
{
  region_desc_t desc;
  if (!describe_region(region, channel_cfg, &desc))
  {
    return false;
  }

  // A region change invalidates the channel list a running beam addresses, so
  // the beam must end before the new region takes effect
  if (beam.active)
  {
    beam_end(TX_RESULT_ABORTED);
  }
  // Reconfiguring the radio drops a transmit in flight without a completion
  if (frame_tx_active)
  {
    frame_tx_active = false;
    callback_cmd_transmit(TX_RESULT_ABORTED);
  }

  if (zpal_radio_change_region((zpal_radio_region_t)region, desc.lr_channel_cfg) != ZPAL_STATUS_OK)
  {
    return false;
  }

  active_region = region;
  active_channel_cfg = (desc.lr_channel_cfg == ZPAL_RADIO_LR_CH_CFG_NO_LR) ? CHANNEL_CFG_CLASSIC : channel_cfg;
  active_desc = desc;

  // The train of a stopped beam may still be on air, the radio returns to RX
  // by itself once it ran out
  radio_tx_busy = false;
  zpal_radio_network_id_filter_set(false);
  zpal_radio_start_receive();

  export_channel_info(&desc, num_channels, channels);
  return true;
}

void radio_get_region(zwave_region_t *region, zwave_channel_cfg_t *channel_cfg, uint8_t *num_channels, channel_info_t *channels)
{
  *region = active_region;
  *channel_cfg = active_channel_cfg;
  if (active_region == ZWAVE_REGION_UNKNOWN)
  {
    return;
  }
  export_channel_info(&active_desc, num_channels, channels);
}

// -----------------------------------------------------------------------------
// radio.h: transmit
// -----------------------------------------------------------------------------

/// @brief Round a deci-dBm power to whole dBm, as ZPAL expects it
static int8_t deci_dbm_to_dbm(int16_t deci_dbm)
{
  int32_t dbm = deci_dbm >= 0 ? (deci_dbm + 5) / 10 : (deci_dbm - 5) / 10;
  if (dbm > INT8_MAX)
  {
    dbm = INT8_MAX;
  }
  else if (dbm < INT8_MIN)
  {
    dbm = INT8_MIN;
  }
  return (int8_t)dbm;
}

/// @brief PHY parameters for a frame on a channel of the active region
static void tx_parameters_for_channel(uint8_t channel, zpal_radio_transmit_parameter_t *params)
{
  memset(params, 0, sizeof(*params));
  params->channel_id = (zpal_radio_zwave_channel_t)active_desc.zpal_channel[channel];
  params->repeats = 0;

  switch (active_desc.channels[channel].baud)
  {
  case ZWAVE_BAUD_9k6:
    params->speed = ZPAL_RADIO_SPEED_9600;
    params->crc = ZPAL_RADIO_CRC_8_BIT_XOR;
    params->preamble = PREAMBLE_BYTE_CLASSIC;
    params->preamble_length = PREAMBLE_BYTES_9K6;
    params->start_of_frame = START_OF_FRAME_CLASSIC;
    break;
  case ZWAVE_BAUD_40k:
    params->speed = ZPAL_RADIO_SPEED_40K;
    params->crc = ZPAL_RADIO_CRC_8_BIT_XOR;
    params->preamble = PREAMBLE_BYTE_CLASSIC;
    params->preamble_length = PREAMBLE_BYTES_40K;
    params->start_of_frame = START_OF_FRAME_CLASSIC;
    break;
  case ZWAVE_BAUD_100k:
    params->speed = ZPAL_RADIO_SPEED_100K;
    params->crc = ZPAL_RADIO_CRC_16_BIT_CCITT;
    params->preamble = PREAMBLE_BYTE_CLASSIC;
    params->preamble_length = active_desc.is_channel_cfg_3 ? PREAMBLE_BYTES_100K_3CH : PREAMBLE_BYTES_100K_2CH;
    params->start_of_frame = START_OF_FRAME_CLASSIC;
    break;
  case ZWAVE_BAUD_LR100k:
  default:
    params->speed = ZPAL_RADIO_SPEED_100KLR;
    params->crc = ZPAL_RADIO_CRC_16_BIT_CCITT;
    params->preamble = PREAMBLE_BYTE_LR;
    params->preamble_length = PREAMBLE_BYTES_LR;
    params->start_of_frame = START_OF_FRAME_LR;
    break;
  }
}

static tx_result_t tx_result_from_event(zpal_radio_event_t event)
{
  switch (event & ZPAL_RADIO_EVENT_MASK)
  {
  case ZPAL_RADIO_EVENT_TX_COMPLETE:
  case ZPAL_RADIO_EVENT_TX_BEAM_COMPLETE:
    return TX_RESULT_COMPLETED;
  case ZPAL_RADIO_EVENT_TX_FAIL_LBT:
    return TX_RESULT_CHANNEL_BUSY;
  case ZPAL_RADIO_EVENT_TX_FAIL:
  default:
    return TX_RESULT_UNKNOWN_ERROR;
  }
}

/// @brief Read the background RSSI on a channel of the active region
///
/// Returns the RSSI in dBm, clamped to the range the LR MPDU RSSI fields
/// allow, or NOISE_FLOOR_NOT_AVAILABLE when the driver has no valid reading.
/// The driver samples the RSSI on every channel while it scans for frames, so
/// this does not interrupt reception.
static int8_t measure_noise_floor(uint8_t channel)
{
  int8_t rssi = ZPAL_RADIO_INVALID_RSSI_DBM;
  if (zpal_radio_get_background_rssi(active_desc.zpal_channel[channel], &rssi) != ZPAL_STATUS_OK)
  {
    return NOISE_FLOOR_NOT_AVAILABLE;
  }
  if (rssi == ZPAL_RADIO_INVALID_RSSI_DBM || rssi == ZPAL_RADIO_RSSI_NOT_AVAILABLE)
  {
    return NOISE_FLOOR_NOT_AVAILABLE;
  }
  if (rssi < NOISE_FLOOR_MIN_DBM)
  {
    return NOISE_FLOOR_MIN_DBM;
  }
  if (rssi > NOISE_FLOOR_MAX_DBM)
  {
    return NOISE_FLOOR_MAX_DBM;
  }
  return rssi;
}

int8_t radio_measure_noise_floor_cmd(uint8_t channel)
{
  if (radio_tx_busy || frame_tx_active || beam.active)
  {
    // The reading would describe our own transmission
    return NOISE_FLOOR_NOT_AVAILABLE;
  }
  if (channel >= active_desc.num_channels)
  {
    return NOISE_FLOOR_NOT_AVAILABLE;
  }
  return measure_noise_floor(channel);
}

void radio_transmit(uint8_t channel, int16_t power_deci_dbm, uint8_t flags, uint8_t *data, uint32_t len, const uint8_t *replacements, uint8_t num_replacements)
{
  if (radio_tx_busy || frame_tx_active || beam.active)
  {
    // There is already a packet on the air, or a beam is running
    respond_cmd_transmit(TX_RESULT_BUSY);
    return;
  }
  if (len > RCP_TX_MAX_FRAME_LEN)
  {
    respond_cmd_transmit(TX_RESULT_OVERFLOW);
    return;
  }
  if (channel >= active_desc.num_channels)
  {
    respond_cmd_transmit(TX_RESULT_INVALID_CHANNEL);
    return;
  }

  if (num_replacements > 0)
  {
    // One measurement serves every noise floor replacement. A failed
    // measurement patches "RSSI not available", and the transmit still goes out
    int8_t noise = NOISE_FLOOR_NOT_AVAILABLE;
    bool noise_measured = false;
    for (uint8_t i = 0; i < num_replacements; i++)
    {
      // Sources other than the noise floor need their own measurement here
      if (replacements[2 * i + 1] != REPLACEMENT_SOURCE_NOISE_FLOOR)
      {
        continue;
      }
      if (!noise_measured)
      {
        noise = measure_noise_floor(channel);
        noise_measured = true;
      }
      data[replacements[2 * i]] = (uint8_t)noise;
    }
  }

  if (power_deci_dbm != TX_POWER_UNCHANGED)
  {
    // The driver clamps the power to the range of its power table
    last_tx_power_dbm = deci_dbm_to_dbm(power_deci_dbm);
  }

  zpal_radio_transmit_parameter_t params;
  tx_parameters_for_channel(channel, &params);

  // The driver appends the checksum the channel's PHY requires. ZPAL splits a
  // frame into header and payload only to concatenate them again, so the
  // whole frame travels as the header
  bool use_lbt = (flags & TRANSMIT_FLAG_CCA) != 0;
  frame_tx_active = true;
  radio_tx_busy = true;
  zpal_status_t status = zpal_radio_transmit(&params, (uint8_t)len, data, 0, data, use_lbt ? 1 : 0, last_tx_power_dbm);

  if (status == ZPAL_STATUS_OK)
  {
    respond_cmd_transmit(TX_RESULT_QUEUED);
  }
  else
  {
    frame_tx_active = false;
    radio_tx_busy = false;
    respond_cmd_transmit(status == ZPAL_STATUS_INVALID_ARGUMENT ? TX_RESULT_INVALID_PARAM : TX_RESULT_UNKNOWN_ERROR);
  }
}

// -----------------------------------------------------------------------------
// radio.h: beam
// -----------------------------------------------------------------------------

/// @brief Look up the beam preamble length for a channel of the active region
static uint8_t beam_preamble_bytes(uint8_t channel)
{
  switch (active_desc.channels[channel].baud)
  {
  case ZWAVE_BAUD_40k:
    return BEAM_PREAMBLE_BYTES_R2;
  case ZWAVE_BAUD_100k:
    // G.9959 Table 7-10 gives R3 a beam preamble length in channel
    // configuration 3 only. Elsewhere §8.1.3.13 puts FL nodes on the R2
    // continuous beam, so an R3 beam would reach nobody.
    return active_desc.is_channel_cfg_3 ? BEAM_PREAMBLE_BYTES_R3 : BEAM_PREAMBLE_BYTES_NONE;
  case ZWAVE_BAUD_LR100k:
    return BEAM_PREAMBLE_BYTES_LR;
  default:
    // G.9959 Table 7-10 lists no beam preamble length for R1
    return BEAM_PREAMBLE_BYTES_NONE;
  }
}

static uint32_t channel_bitrate(uint8_t channel)
{
  switch (active_desc.channels[channel].baud)
  {
  case ZWAVE_BAUD_9k6:
    return 9600;
  case ZWAVE_BAUD_40k:
    return 40000;
  default:
    return 100000;
  }
}

static uint8_t beam_fragment_channel(void)
{
  return beam.channels[beam.fragment_index % beam.num_channels];
}

/// @brief Put a repeat train on air that covers the rest of the current fragment
///
/// The driver repeats the beam frame back to back on its own, up to 255
/// times per train. A fragment longer than one train is covered by chaining
/// trains from radio_zpal_handle_tx_done().
///
/// G.9959 §8.1.3.12: "The beam frames shall be sent back to back to prevent
/// other TXs from interrupting the continuous beam."
static tx_result_t beam_start_train(void)
{
  uint8_t channel = beam_fragment_channel();

  TickType_t now = xTaskGetTickCount();
  int32_t remaining_ms = (int32_t)(beam.fragment_end - now);
  if (remaining_ms <= 0)
  {
    remaining_ms = 1;
  }
  if (remaining_ms > BEAM_CHUNK_MAX_MS)
  {
    remaining_ms = BEAM_CHUNK_MAX_MS;
  }

  // Airtime of one beam frame: preamble, start of frame and the beam data
  uint32_t frame_bits = ((uint32_t)beam_preamble_bytes(channel) + 1 + beam.data_len) * 8;
  uint32_t frame_us = (frame_bits * 1000000UL) / channel_bitrate(channel);
  if (frame_us == 0)
  {
    frame_us = 1;
  }
  uint32_t repeats = ((uint32_t)remaining_ms * 1000UL) / frame_us + 1;
  if (repeats > UINT8_MAX)
  {
    repeats = UINT8_MAX;
  }

  zpal_radio_transmit_parameter_t params;
  tx_parameters_for_channel(channel, &params);
  // Beam frames carry no checksum
  params.crc = ZPAL_RADIO_CRC_NONE;
  params.preamble_length = beam_preamble_bytes(channel);
  params.repeats = (uint8_t)repeats;

  beam.transmitting = true;
  radio_tx_busy = true;
  zpal_status_t status = zpal_radio_transmit_beam(&params, beam.data_len, beam.data, beam.power_dbm);
  if (status != ZPAL_STATUS_OK)
  {
    beam.transmitting = false;
    radio_tx_busy = false;
    return status == ZPAL_STATUS_INVALID_ARGUMENT ? TX_RESULT_INVALID_PARAM : TX_RESULT_UNKNOWN_ERROR;
  }
  return TX_RESULT_QUEUED;
}

void radio_transmit_beam(
    int16_t power_deci_dbm,
    uint8_t num_fragments,
    uint16_t fragment_duration_ms,
    uint16_t fragment_period_ms,
    uint8_t num_channels,
    const uint8_t *channels,
    const uint8_t *data,
    uint8_t data_len)
{
  if (radio_tx_busy || frame_tx_active || beam.active)
  {
    // There is already a packet on the air, or a beam is running
    respond_cmd_transmit_beam(TX_RESULT_BUSY);
    return;
  }
  if (num_fragments == 0 || num_channels > RADIO_MAX_CHANNELS || data_len == 0 || data_len > BEAM_DATA_MAX_LEN || fragment_duration_ms == 0)
  {
    respond_cmd_transmit_beam(TX_RESULT_INVALID_PARAM);
    return;
  }
  if (num_fragments > 1 && fragment_duration_ms > fragment_period_ms)
  {
    // Each fragment must end before the next one starts
    respond_cmd_transmit_beam(TX_RESULT_INVALID_PARAM);
    return;
  }

  // Every fragment must be startable, so validate every channel up front
  for (uint8_t i = 0; i < num_channels; i++)
  {
    if (channels[i] >= active_desc.num_channels || beam_preamble_bytes(channels[i]) == BEAM_PREAMBLE_BYTES_NONE)
    {
      respond_cmd_transmit_beam(TX_RESULT_INVALID_CHANNEL);
      return;
    }
  }

  beam.num_fragments = num_fragments;
  beam.fragment_index = 0;
  beam.fragment_duration = pdMS_TO_TICKS(fragment_duration_ms);
  beam.fragment_period = pdMS_TO_TICKS(fragment_period_ms);
  beam.num_channels = num_channels;
  memcpy(beam.channels, channels, num_channels);
  memcpy(beam.data, data, data_len);
  beam.data_len = data_len;
  if (power_deci_dbm != TX_POWER_UNCHANGED)
  {
    // A beam leaves the radio at the power it used, like on EFR32
    last_tx_power_dbm = deci_dbm_to_dbm(power_deci_dbm);
  }
  beam.power_dbm = last_tx_power_dbm;

  beam.active = true;
  beam.transmitting = false;
  beam.abort_requested = false;
  beam.fragment_start = xTaskGetTickCount();
  beam.fragment_end = beam.fragment_start + beam.fragment_duration;

  tx_result_t result = beam_start_train();
  if (result != TX_RESULT_QUEUED)
  {
    // The host learns about the failure from the response, so no callback follows
    beam_stop();
  }
  respond_cmd_transmit_beam(result);
}

void radio_abort_beam(void)
{
  if (!beam.active)
  {
    return;
  }
  if (beam.transmitting)
  {
    // A train on air cannot be cut short. The completion of the current train
    // ends the beam, and no further train starts
    beam.abort_requested = true;
    return;
  }
  beam_end(TX_RESULT_ABORTED);
}

/// @brief Schedule the next beam phase, treating a deadline in the past as reached
static void beam_arm_timer(TickType_t deadline)
{
  int32_t delay = (int32_t)(deadline - xTaskGetTickCount());
  if (delay <= 0)
  {
    rcp_notify(RCP_EVENT_BEAM_TIMER);
    return;
  }
  // Changing the period also starts a dormant timer
  xTimerChangePeriod(beam_timer, (TickType_t)delay, 0);
}

/// @brief Take the radio out of beam mode. The radio returns to RX on its own
static void beam_stop(void)
{
  xTimerStop(beam_timer, 0);
  beam.active = false;
  beam.transmitting = false;
  beam.abort_requested = false;
}

/// @brief Stop the beam and report the result to the host
static void beam_end(tx_result_t result)
{
  beam_stop();
  callback_cmd_transmit_beam(result);
}

/// @brief FreeRTOS timer callback, runs on the timer service task
static void beam_timer_expired(TimerHandle_t timer)
{
  (void)timer;
  rcp_notify(RCP_EVENT_BEAM_TIMER);
}

void radio_zpal_handle_beam_timer(void)
{
  if (!beam.active || beam.transmitting)
  {
    // A stale timer from a beam that already ended
    return;
  }
  if (beam.abort_requested)
  {
    beam_end(TX_RESULT_ABORTED);
    return;
  }

  // Count the period off the previous fragment's own reference, so the
  // latency of each fragment start does not accumulate over the beam
  beam.fragment_index++;
  beam.fragment_start += beam.fragment_period;
  beam.fragment_end = beam.fragment_start + beam.fragment_duration;

  tx_result_t result = beam_start_train();
  if (result != TX_RESULT_QUEUED)
  {
    beam_end(result);
  }
}
