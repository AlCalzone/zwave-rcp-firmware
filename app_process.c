/******************************************************************************
 * @file
 * @brief app_process.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include "sl_component_catalog.h"
#include <em_eusart.h>
#include "rail.h"
#include "rail_zwave.h"

#include "common.h"
#include "serial_api.h"
#include "app_init.h"
#include "app_process.h"

#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "app_task_init.h"
#endif

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
typedef enum
{
  RAILS_IDLE,
  RAILS_RX,
  RAILS_TX,
  RAILS_BEAM,
} rail_state_t;

/// Beam preamble lengths in bits, G.9959 Table 7-10 "Minimum Preamble length"
/// for R2 (20 bytes) and R3 in channel configuration 3 (8 bytes), and Z-Wave
/// Long Range PHY and MAC Layer Specification Table 5-10 "Required Preamble
/// length" for LR (8 bytes / 16 symbols).
#define BEAM_PREAMBLE_BITS_R2 (20 * 8)
#define BEAM_PREAMBLE_BITS_R3 (8 * 8)
#define BEAM_PREAMBLE_BITS_LR (8 * 8)

/// A rate that carries no beam preamble length in the tables above
#define BEAM_PREAMBLE_BITS_NONE 0

/// Beam frames must not carry the checksum the Z-Wave PHY normally appends.
/// The transmit uses exactly the fixed length configured below.
/// The beam preamble length comes from RAIL_SetTxAltPreambleLength().
/// RESEND lets every repeat reuse the frame written to the TX FIFO once.
#define BEAM_TX_OPTIONS (RAIL_TX_OPTION_REMOVE_CRC | RAIL_TX_OPTION_ALT_PREAMBLE_LEN | RAIL_TX_OPTION_RESEND)

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
static void handle_received_packet(RAIL_Handle_t rail_handle);
static void uart_handle_rx(RAIL_Handle_t rail_handle);
static void reset_rx_fifo(uint8_t new_start);
static void uart_handle_frame(RAIL_Handle_t rail_handle, frame_type_t frame_type, func_id_t func_id, uint8_t *payload, uint8_t len);
static void export_channel_info(const RAIL_ZWAVE_RegionConfig_t *region_config, uint8_t *num_channels, channel_info_t *channels);
static void rail_transmit(RAIL_Handle_t rail_handle, uint8_t *data, uint32_t len);
static uint8_t tx_result_from_events(RAIL_Events_t events);
static uint16_t beam_preamble_bits(uint8_t channel);
static uint8_t beam_fragment_channel(void);
static tx_result_t beam_start_fragment(RAIL_Handle_t rail_handle);
static void beam_arm_timer(RAIL_Handle_t rail_handle, RAIL_Time_t deadline);
static void beam_pause_radio(RAIL_Handle_t rail_handle);
static void beam_stop(RAIL_Handle_t rail_handle);
static void beam_end(RAIL_Handle_t rail_handle, tx_result_t result);
static void beam_advance(RAIL_Handle_t rail_handle);
static void beam_timer_expired(RAIL_Handle_t rail_handle);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
rail_state_t rail_state = RAILS_IDLE;

uint8_t UART_RX_FIFO[UART_RX_FIFO_SIZE] = {0};
/// The position of the write cursor in the RX FIFO
uint32_t uart_rx_pos = 0;

uint8_t UART_TX_FIFO[UART_TX_FIFO_SIZE] = {0};
uint32_t uart_tx_pos = 0;
uint32_t uart_tx_len = 0;

/// New UART RX data arrived and awaits processing
bool uart_rx_done = false;
bool uart_tx_done = false;

static uint8_t tx_channel = 0;
static int16_t tx_power_deci_dbm = 0;
static uint8_t tx_flags = 0;
/// Number of channels the currently configured region has
static uint8_t region_num_channels = 0;
/// Data rate of each channel of the currently configured region
static zwave_baudrate_t region_channel_baud[RAIL_NUM_ZWAVE_CHANNELS] = {0};
/// The currently configured region uses G.9959 channel configuration 3
static bool region_is_channel_cfg_3 = false;
static uint8_t OUT_PACKET[RAIL_FIFO_SIZE] = {0};
static uint32_t out_packet_len = 0;

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

static bool rail_packet_sent = false;

static bool rail_packet_received = false;

static bool rail_error = false;
static uint8_t tx_error = 0;

/// A transmit was handed to RAIL and has not completed yet. The radio owns
/// the TX FIFO and the PA power until then, so no other transmit may start.
static volatile bool tx_in_flight = false;

static bool start_rx = true;

/// A beam is running. TX completion events belong to its repeat train and
/// no other transmit may start
static volatile bool beam_active = false;

/// The repeat train is on air. Clear this before idling the radio, so the abort
/// that the idle causes is not mistaken for a radio failure.
static volatile bool beam_transmitting = false;

/// TX error RAIL reported for the repeat train, which stops the repetition
static volatile uint8_t beam_tx_error = 0;

/// The beam timer expired, so the main loop moves the beam to its next phase
static volatile bool beam_timer_fired = false;

/// The beam the host requested with FUNC_ID_TRANSMIT_BEAM
static struct
{
  uint8_t num_fragments;
  uint8_t fragment_index;
  uint32_t fragment_duration_us;
  uint32_t fragment_period_us;
  uint8_t num_channels;
  uint8_t channels[RAIL_NUM_ZWAVE_CHANNELS];
  uint8_t data[BEAM_DATA_MAX_LEN];
  uint8_t data_len;
  int16_t power_deci_dbm;
  /// RAIL time the current fragment is scheduled at, the reference the period counts from
  RAIL_Time_t fragment_start;
} beam = {0};

/// Repeat the beam frame until the fragment timer stops it. RAIL chains the
/// repeats in hardware, so only the PA ramp separates two frames. Each repeat
/// still raises its own completion event, which the event handler discards.
///
/// G.9959 §8.1.3.12: "The beam frames shall be sent back to back to prevent
/// other TXs from interrupting the continuous beam."
static const RAIL_TxRepeatConfig_t beam_repeat_config = {
    .iterations = RAIL_TX_REPEAT_INFINITE_ITERATIONS,
    .repeatOptions = RAIL_TX_REPEAT_OPTIONS_NONE,
    .delayOrHop = {.delay = 0},
};

/// Copy of last RAIL events to process
static RAIL_Events_t rail_last_state = RAIL_EVENTS_NONE;

static volatile RAIL_Status_t calibration_status = 0;

static volatile RAIL_RxPacketHandle_t rx_packet_handle = RAIL_RX_PACKET_HANDLE_INVALID;

/// LBT parameters for transmits that request CCA. A single check with no
/// backoff, so the host stays in charge of the retry policy.
///
/// G.9959 §7.1.2.5.4: "The PHY shall be able to perform a CCA with a threshold
/// of –80 dBm. In a given deployment, a listen before talk (LBT) operation
/// based on CCA shall comply with actual regional RF regulatory requirements,
/// e.g., listening period and threshold."
///
/// The fixed threshold and duration below are a known simplification. Deriving
/// the listening period and threshold from the active region is a tracked
/// follow-up.
static const RAIL_LbtConfig_t lbt_config = {
    .lbtMinBoRand = 0,
    .lbtMaxBoRand = 0,
    .lbtTries = 1,
    .lbtThreshold = -80,
    .lbtBackoff = 0,
    // 1 ms spans several bit periods at the slowest Z-Wave rate of 9.6 kbps
    .lbtDuration = 1000,
    .lbtTimeout = 0,
};

static __ALIGNED(RAIL_FIFO_ALIGNMENT) uint8_t rx_fifo[RAIL_FIFO_SIZE];
static __ALIGNED(RAIL_FIFO_ALIGNMENT) uint8_t tx_fifo[RAIL_FIFO_SIZE];

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------

/******************************************************************************
 * Set up the rail TX fifo for later usage
 * @param[in] rail_handle Which rail handler should be updated
 *****************************************************************************/
void set_up_tx_fifo(RAIL_Handle_t rail_handle)
{
  uint16_t allocated_tx_fifo_size = 0;
  allocated_tx_fifo_size = RAIL_SetTxFifo(rail_handle, tx_fifo, 0, RAIL_FIFO_SIZE);
}

/******************************************************************************
 * Application state machine, called infinitely
 *****************************************************************************/
void app_process_action(RAIL_Handle_t rail_handle)
{
  ///////////////////////////////////////////////////////////////////////////
  // Put your application code here!                                       //
  // This is called infinitely.                                            //
  // Do not call blocking functions from here!                             //
  ///////////////////////////////////////////////////////////////////////////

  // State machine for radio RX/TX
  if (rail_state == RAILS_IDLE)
  {
    // We're only idle at the start of the program
    // Start receiving on channel 0. RX channel hopping takes care of the other channels
    RAIL_StartRx(rail_handle, 0, NULL);
    rail_state = RAILS_RX;
  }
  else if (rail_state == RAILS_RX)
  {
    if (rail_packet_received)
    {
      rail_packet_received = false;
      handle_received_packet(rail_handle);
    }
    else if (out_packet_len > 0)
    {
      rail_state = RAILS_TX;
    }
  }
  else if (rail_state == RAILS_TX)
  {
    if (out_packet_len > 0)
    {
      rail_transmit(rail_handle, OUT_PACKET, out_packet_len);
      out_packet_len = 0;
    }
    else if (rail_packet_sent)
    {
      callback_cmd_transmit(TX_RESULT_COMPLETED);
      rail_packet_sent = false;
      rail_state = RAILS_RX;
    }
    else if (tx_error != 0)
    {
      // FIXME: Figure out what the error is
      callback_cmd_transmit((tx_result_t)tx_error);
      tx_error = 0;
      rail_state = RAILS_RX;
    }
  }
  else if (rail_state == RAILS_BEAM)
  {
    if (beam_tx_error != 0)
    {
      // The repeat train stopped early, so the beam's fixed frame length has to
      // go before the next received frame is decoded with it
      beam_end(rail_handle, (tx_result_t)beam_tx_error);
    }
    else if (beam_timer_fired)
    {
      beam_timer_fired = false;
      beam_advance(rail_handle);
    }
    else if (rail_packet_received)
    {
      // Between fragments the radio is back in RX, where the woken node's ack arrives
      rail_packet_received = false;
      handle_received_packet(rail_handle);
    }
  }

  // Handle any UART data that arrived
  if (uart_rx_done)
  {
    uart_rx_done = false;
    uart_handle_rx(rail_handle);
  }
}

/// @brief Queue raw data for transmission over UART
void uart_transmit(uint8_t *data, uint32_t len)
{
  memcpy(&UART_TX_FIFO[uart_tx_len], data, len);
  uart_tx_len += len;
  uart_tx_done = false;
  // Enable the TX interrupt that performs the transmission
  EUSART_IntEnable(EUSART0, EUSART_IEN_TXFL);
}

/// @brief Queue a frame for transmission over UART
void uart_transmit_frame(frame_type_t frame_type, func_id_t func_id, uint8_t *payload, uint32_t payload_len)
{
  uint8_t frame_len = payload_len + 3; // length, type, func_id
  uint8_t frame[frame_len + 2];        // SOF, ...rest, chksum
  uint8_t chksum = 0xff ^ frame_len ^ frame_type ^ func_id;
  uint8_t i = 0;
  frame[i++] = SOF;
  frame[i++] = frame_len;
  frame[i++] = frame_type;
  frame[i++] = func_id;
  for (int j = 0; j < payload_len; j++)
  {
    frame[i++] = payload[j];
    chksum ^= payload[j];
  }
  frame[i++] = chksum;

  uart_transmit(frame, frame_len + 2);
}

/// @brief Queue a single byte for transmission over UART
void uart_transmit_byte(uint8_t byte)
{
  UART_TX_FIFO[uart_tx_len++] = byte;
  uart_tx_done = false;
  // Enable the TX interrupt that performs the transmission
  EUSART_IntEnable(EUSART0, EUSART_IEN_TXFL);
}

/// @brief Handle a received frame over UART
static void uart_handle_rx(RAIL_Handle_t rail_handle)
{
  // Advance to the first SOF byte
  int i = 0;
  while (
      i < uart_rx_pos
      && UART_RX_FIFO[i] != SOF)
  {
    i++;
  }
  if (i == uart_rx_pos)
  {
    // The buffer contains no SOF
    reset_rx_fifo(i);
    return;
  }

  // We need SOF and LEN to validate the frame length
  uint8_t remaining = uart_rx_pos - i;
  if (remaining < 2)
  {
    return;
  }

  uint8_t chksum = 0xff;
  // Skip SOF
  i++;

  // Read the frame length, which excludes SOF and checksum
  uint8_t len = UART_RX_FIFO[i++];
  chksum ^= len;

  if (len < 3)
  {
    // LEN must cover at least the frame type, the function ID and the checksum.
    // Anything shorter underflows the payload length passed to the handlers.
    reset_rx_fifo(i);
    uart_transmit_byte(NAK);
    return;
  }

  remaining = uart_rx_pos - i;
  if (remaining < len)
  {
    // Wait for the rest of the frame to arrive
    return;
  }

  // Extract frame and update checksum
  uint8_t cmd[len - 1];
  for (int j = 0; j < len - 1; j++)
  {
    cmd[j] = UART_RX_FIFO[i++];
    chksum ^= cmd[j];
  }

  // XOR in the received checksum so a match leaves chksum at 0
  chksum ^= UART_RX_FIFO[i++];

  // Move remaining data to start of buffer
  // FIXME: Use a ringbuffer so we can avoid this
  if (i < uart_rx_pos)
  {
    reset_rx_fifo(i);
    // Trigger another processing pass for the buffered data
    uart_rx_done = true;
  }
  uart_rx_pos = 0;

  if (chksum == 0)
  {
    uart_transmit_byte(ACK);
    uart_handle_frame(rail_handle, cmd[0], cmd[1], &cmd[2], len - 3);
  }
  else
  {
    // Try to re-sync
    uart_transmit_byte(NAK);
  }
}

static void reset_rx_fifo(uint8_t new_start)
{
  if (new_start < uart_rx_pos)
  {
    memmove(UART_RX_FIFO, &UART_RX_FIFO[new_start], uart_rx_pos - new_start);
  }
  uart_rx_pos = 0;
}

static void uart_handle_frame(RAIL_Handle_t rail_handle, frame_type_t frame_type, func_id_t func_id, uint8_t *payload, uint8_t len)
{
  if (frame_type != FRAME_TYPE_REQ)
  {
    return;
  }

  switch (func_id)
  {
  case FUNC_ID_GET_FIRMWARE_INFO:
    handle_cmd_get_firmware_info(payload, len);
    break;

  case FUNC_ID_SETUP_RADIO:
    handle_cmd_setup_radio(rail_handle, payload, len);
    break;

  case FUNC_ID_TRANSMIT:
    handle_cmd_transmit(payload, len);
    break;

  case FUNC_ID_TRANSMIT_BEAM:
    handle_cmd_transmit_beam(rail_handle, payload, len);
    break;

  case FUNC_ID_ABORT_BEAM:
    handle_cmd_abort_beam(rail_handle);
    break;
  default:
    break;
  }
}

bool radio_set_region(RAIL_Handle_t rail_handle, zwave_region_t region, zwave_channel_cfg_t channel_cfg, uint8_t *num_channels, channel_info_t *channels)
{
  const RAIL_ZWAVE_RegionConfig_t *region_config;

  switch (region)
  {
  case REGION_EU:
    region_config = &RAIL_ZWAVE_REGION_EU;
    break;
  case REGION_US:
    region_config = &RAIL_ZWAVE_REGION_US;
    break;
  case REGION_ANZ:
    region_config = &RAIL_ZWAVE_REGION_ANZ;
    break;
  case REGION_HK:
    region_config = &RAIL_ZWAVE_REGION_HK;
    break;
  case REGION_IN:
    region_config = &RAIL_ZWAVE_REGION_IN;
    break;
  case REGION_IL:
    region_config = &RAIL_ZWAVE_REGION_IL;
    break;
  case REGION_RU:
    region_config = &RAIL_ZWAVE_REGION_RU;
    break;
  case REGION_CN:
    region_config = &RAIL_ZWAVE_REGION_CN;
    break;
  case REGION_JP:
    region_config = &RAIL_ZWAVE_REGION_JP;
    break;
  case REGION_KR:
    region_config = &RAIL_ZWAVE_REGION_KR;
    break;

    // For LR regions, the RAIL config also depends on the channel configuration
  case REGION_US_LR:
    switch (channel_cfg)
    {
    case CHANNEL_CFG_CLASSIC_LR_A:
      region_config = &RAIL_ZWAVE_REGION_US_LR1;
      break;
    case CHANNEL_CFG_CLASSIC_LR_B:
      region_config = &RAIL_ZWAVE_REGION_US_LR2;
      break;
    case CHANNEL_CFG_LR:
      region_config = &RAIL_ZWAVE_REGION_US_LR3;
      break;
    default:
      return false;
    }
    break;
  case REGION_EU_LR:
    switch (channel_cfg)
    {
    case CHANNEL_CFG_CLASSIC_LR_A:
      region_config = &RAIL_ZWAVE_REGION_EU_LR1;
      break;
    case CHANNEL_CFG_CLASSIC_LR_B:
      region_config = &RAIL_ZWAVE_REGION_EU_LR2;
      break;
    case CHANNEL_CFG_LR:
      region_config = &RAIL_ZWAVE_REGION_EU_LR3;
      break;
    default:
      return false;
    }
    break;

  default:
    return false;
  }

  RAIL_Status_t status = RAIL_ZWAVE_ConfigRegion(rail_handle, region_config);
  if (status != RAIL_STATUS_NO_ERROR)
  {
    return false;
  }

  radio_sync_active_region(rail_handle);

  export_channel_info(region_config, num_channels, channels);
  return true;
}

static const RAIL_ZWAVE_RegionConfig_t *resolve_active_region(RAIL_Handle_t rail_handle, zwave_region_t *region, zwave_channel_cfg_t *channel_cfg)
{
  RAIL_ZWAVE_RegionId_t rail_region = RAIL_ZWAVE_GetRegion(rail_handle);
  const RAIL_ZWAVE_RegionConfig_t *region_config;

  switch (rail_region)
  {
  case RAIL_ZWAVE_REGIONID_EU:
    *region = REGION_EU;
    region_config = &RAIL_ZWAVE_REGION_EU;
    break;
  case RAIL_ZWAVE_REGIONID_US:
    *region = REGION_US;
    region_config = &RAIL_ZWAVE_REGION_US;
    break;
  case RAIL_ZWAVE_REGIONID_ANZ:
    *region = REGION_ANZ;
    region_config = &RAIL_ZWAVE_REGION_ANZ;
    break;
  case RAIL_ZWAVE_REGIONID_HK:
    *region = REGION_HK;
    region_config = &RAIL_ZWAVE_REGION_HK;
    break;
  case RAIL_ZWAVE_REGIONID_IN:
    *region = REGION_IN;
    region_config = &RAIL_ZWAVE_REGION_IN;
    break;
  case RAIL_ZWAVE_REGIONID_JP:
    *region = REGION_JP;
    region_config = &RAIL_ZWAVE_REGION_JP;
    break;
  case RAIL_ZWAVE_REGIONID_RU:
    *region = REGION_RU;
    region_config = &RAIL_ZWAVE_REGION_RU;
    break;
  case RAIL_ZWAVE_REGIONID_IL:
    *region = REGION_IL;
    region_config = &RAIL_ZWAVE_REGION_IL;
    break;
  case RAIL_ZWAVE_REGIONID_KR:
    *region = REGION_KR;
    region_config = &RAIL_ZWAVE_REGION_KR;
    break;
  case RAIL_ZWAVE_REGIONID_CN:
    *region = REGION_CN;
    region_config = &RAIL_ZWAVE_REGION_CN;
    break;
  case RAIL_ZWAVE_REGIONID_US_LR1:
    *region = REGION_US_LR;
    *channel_cfg = CHANNEL_CFG_CLASSIC_LR_A;
    region_config = &RAIL_ZWAVE_REGION_US_LR1;
    break;
  case RAIL_ZWAVE_REGIONID_US_LR2:
    *region = REGION_US_LR;
    *channel_cfg = CHANNEL_CFG_CLASSIC_LR_B;
    region_config = &RAIL_ZWAVE_REGION_US_LR2;
    break;
  case RAIL_ZWAVE_REGIONID_US_LR3:
    *region = REGION_US_LR;
    *channel_cfg = CHANNEL_CFG_LR;
    region_config = &RAIL_ZWAVE_REGION_US_LR3;
    break;
  case RAIL_ZWAVE_REGIONID_EU_LR1:
    *region = REGION_EU_LR;
    *channel_cfg = CHANNEL_CFG_CLASSIC_LR_A;
    region_config = &RAIL_ZWAVE_REGION_EU_LR1;
    break;
  case RAIL_ZWAVE_REGIONID_EU_LR2:
    *region = REGION_EU_LR;
    *channel_cfg = CHANNEL_CFG_CLASSIC_LR_B;
    region_config = &RAIL_ZWAVE_REGION_EU_LR2;
    break;
  case RAIL_ZWAVE_REGIONID_EU_LR3:
    *region = REGION_EU_LR;
    *channel_cfg = CHANNEL_CFG_LR;
    region_config = &RAIL_ZWAVE_REGION_EU_LR3;
    break;
  default:
    *region = REGION_UNKNOWN;
    return NULL;
  }

  return region_config;
}

void radio_get_region(RAIL_Handle_t rail_handle, zwave_region_t *region, zwave_channel_cfg_t *channel_cfg, uint8_t *num_channels, channel_info_t *channels)
{
  const RAIL_ZWAVE_RegionConfig_t *region_config = resolve_active_region(rail_handle, region, channel_cfg);
  if (region_config == NULL)
  {
    return;
  }

  export_channel_info(region_config, num_channels, channels);
}

/// Cache the channel count and data rates of the region RAIL currently has
/// configured and reconfigure RX channel hopping for it. The transmit and
/// beam handlers validate against this cached state, so it must be refreshed
/// whenever the region changes.

void radio_sync_active_region(RAIL_Handle_t rail_handle)
{
  zwave_region_t region;
  zwave_channel_cfg_t channel_cfg;
  const RAIL_ZWAVE_RegionConfig_t *region_config = resolve_active_region(rail_handle, &region, &channel_cfg);

  // A region change invalidates the channel list a running beam addresses, so
  // the beam must end before the new region takes effect
  if (beam_active)
  {
    beam_end(rail_handle, TX_RESULT_ABORTED);
  }

  if (region_config == NULL)
  {
    region_num_channels = 0;
    return;
  }

  channel_info_t channels[RAIL_NUM_ZWAVE_CHANNELS] = {0};
  region_num_channels = 0;
  export_channel_info(region_config, &region_num_channels, channels);

  // G.9959 Table 7-3 gives channel configuration 3 three channels, all at
  // 100 kbps. Every other configuration mixes data rates or channel counts.
  region_is_channel_cfg_3 = (region_num_channels == 3);
  for (uint8_t i = 0; i < region_num_channels; i++)
  {
    region_channel_baud[i] = channels[i].baud;
    if (channels[i].baud != ZWAVE_BAUD_100k)
    {
      region_is_channel_cfg_3 = false;
    }
  }

  // Dwell times and the number of hopped channels differ per region
  init_rx_channel_hopping(rail_handle, region_num_channels);

  // Configuring hopping idles the radio, so return to the state that starts RX
  rail_state = RAILS_IDLE;
}

static void export_channel_info(const RAIL_ZWAVE_RegionConfig_t *region_config, uint8_t *num_channels, channel_info_t *channels)
{
  for (int i = 0; i < RAIL_NUM_ZWAVE_CHANNELS; i++)
  {
    if (region_config->frequency[i] == 0xffffffff)
    {
      break;
    }
    channels[i].freq = region_config->frequency[i];
    channels[i].baud = region_config->baudRate[i];
    (*num_channels)++;
  }
}

void radio_transmit(uint8_t channel, int16_t power_deci_dbm, uint8_t flags, uint8_t *data, uint32_t len)
{
  if (out_packet_len > 0 || tx_in_flight || rail_packet_sent || tx_error != 0 || beam_active)
  {
    // There is already a packet in the buffer or on the air, or a completed
    // transmit the state machine has not reported to the host yet
    respond_cmd_transmit(TX_RESULT_BUSY);
    return;
  }
  if (len > RAIL_FIFO_SIZE)
  {
    respond_cmd_transmit(TX_RESULT_OVERFLOW);
    return;
  }
  if (channel >= region_num_channels)
  {
    respond_cmd_transmit(TX_RESULT_INVALID_CHANNEL);
    return;
  }

  // Queue the packet. The response will be handled by `rail_transmit()`
  tx_channel = channel;
  tx_power_deci_dbm = power_deci_dbm;
  tx_flags = flags;
  memcpy(OUT_PACKET, data, len);
  out_packet_len = len;
}

static void rail_transmit(RAIL_Handle_t rail_handle, uint8_t *data, uint32_t len)
{
  if (tx_power_deci_dbm != TX_POWER_UNCHANGED)
  {
    // RAIL coerces the requested power against the channel the radio is tuned
    // to, and RX channel hopping parks it on an arbitrary channel
    RAIL_PrepareChannel(rail_handle, tx_channel);

    // RAIL clamps the power to the PA curve and the channel's maximum
    RAIL_Status_t power_status = RAIL_SetTxPowerDbm(rail_handle, (RAIL_TxPower_t)tx_power_deci_dbm);
    if (power_status != RAIL_STATUS_NO_ERROR)
    {
      // The transmit must happen at the power the host requested
      respond_cmd_transmit(TX_RESULT_INVALID_PARAM);
      rail_state = RAILS_RX;
      return;
    }
  }

  RAIL_WriteTxFifo(rail_handle, data, len, true);

  RAIL_Status_t rail_status;
  if (tx_flags & TRANSMIT_FLAG_CCA)
  {
    rail_status = RAIL_StartCcaLbtTx(rail_handle, tx_channel, RAIL_TX_OPTIONS_DEFAULT, &lbt_config, NULL);
  }
  else
  {
    rail_status = RAIL_StartTx(rail_handle, tx_channel, RAIL_TX_OPTIONS_DEFAULT, NULL);
  }

  if (rail_status == RAIL_STATUS_NO_ERROR)
  {
    tx_in_flight = true;
    respond_cmd_transmit(TX_RESULT_QUEUED);
  }
  else if (rail_status == RAIL_STATUS_INVALID_PARAMETER)
  {
    // RAIL never started, so no completion event will arrive to leave RAILS_TX
    rail_state = RAILS_RX;
    respond_cmd_transmit(TX_RESULT_INVALID_PARAM);
  }
  else
  {
    rail_state = RAILS_RX;
    // TODO: Figure out other possible errors and expose them
    respond_cmd_transmit(TX_RESULT_UNKNOWN_ERROR);
  }
}

void radio_transmit_beam(
    RAIL_Handle_t rail_handle,
    int16_t power_deci_dbm,
    uint8_t num_fragments,
    uint16_t fragment_duration_ms,
    uint16_t fragment_period_ms,
    uint8_t num_channels,
    const uint8_t *channels,
    const uint8_t *data,
    uint8_t data_len)
{
  if (out_packet_len > 0 || tx_in_flight || rail_packet_sent || tx_error != 0 || beam_active)
  {
    // There is already a packet in the buffer or on the air, or a completed
    // transmit the state machine has not reported to the host yet
    respond_cmd_transmit_beam(TX_RESULT_BUSY);
    return;
  }
  if (num_fragments == 0 || num_channels > RAIL_NUM_ZWAVE_CHANNELS || data_len == 0 || data_len > BEAM_DATA_MAX_LEN || fragment_duration_ms == 0)
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
    if (channels[i] >= region_num_channels || beam_preamble_bits(region_channel_baud[channels[i]]) == BEAM_PREAMBLE_BITS_NONE)
    {
      respond_cmd_transmit_beam(TX_RESULT_INVALID_CHANNEL);
      return;
    }
  }

  beam.num_fragments = num_fragments;
  beam.fragment_index = 0;
  beam.fragment_duration_us = (uint32_t)fragment_duration_ms * 1000;
  beam.fragment_period_us = (uint32_t)fragment_period_ms * 1000;
  beam.num_channels = num_channels;
  memcpy(beam.channels, channels, num_channels);
  memcpy(beam.data, data, data_len);
  beam.data_len = data_len;
  beam.power_deci_dbm = power_deci_dbm;

  beam_active = true;
  beam_transmitting = false;
  beam_tx_error = 0;
  beam_timer_fired = false;
  rail_state = RAILS_BEAM;

  tx_result_t result = beam_start_fragment(rail_handle);
  if (result != TX_RESULT_QUEUED)
  {
    // The host learns about the failure from the response, so no callback follows
    beam_stop(rail_handle);
  }
  respond_cmd_transmit_beam(result);
}

void radio_abort_beam(RAIL_Handle_t rail_handle)
{
  if (!beam_active)
  {
    return;
  }
  beam_end(rail_handle, TX_RESULT_ABORTED);
}

static uint8_t tx_result_from_events(RAIL_Events_t events)
{
  if (events & RAIL_EVENT_TX_ABORTED)
  {
    return TX_RESULT_ABORTED;
  }
  if (events & RAIL_EVENT_TX_BLOCKED)
  {
    return TX_RESULT_BLOCKED;
  }
  if (events & RAIL_EVENT_TX_UNDERFLOW)
  {
    return TX_RESULT_UNDERFLOW;
  }
  if (events & RAIL_EVENT_TX_CHANNEL_BUSY)
  {
    return TX_RESULT_CHANNEL_BUSY;
  }
  return TX_RESULT_UNKNOWN_ERROR;
}

/// @brief Look up the beam preamble length for a channel of the active region
static uint16_t beam_preamble_bits(uint8_t channel)
{
  switch (region_channel_baud[channel])
  {
  case ZWAVE_BAUD_40k:
    return BEAM_PREAMBLE_BITS_R2;
  case ZWAVE_BAUD_100k:
    // G.9959 Table 7-10 gives R3 a beam preamble length in channel
    // configuration 3 only. Elsewhere §8.1.3.13 puts FL nodes on the R2
    // continuous beam, so an R3 beam would reach nobody.
    return region_is_channel_cfg_3 ? BEAM_PREAMBLE_BITS_R3 : BEAM_PREAMBLE_BITS_NONE;
  case ZWAVE_BAUD_LR100k:
    return BEAM_PREAMBLE_BITS_LR;
  default:
    // G.9959 Table 7-10 lists no beam preamble length for R1
    return BEAM_PREAMBLE_BITS_NONE;
  }
}

static uint8_t beam_fragment_channel(void)
{
  return beam.channels[beam.fragment_index % beam.num_channels];
}

/// @brief Put the repeat train for the current fragment on air
static tx_result_t beam_start_fragment(RAIL_Handle_t rail_handle)
{
  uint8_t channel = beam_fragment_channel();

  // Take the radio out of RX channel hopping before reconfiguring the PHY
  RAIL_Idle(rail_handle, RAIL_IDLE_ABORT, true);

  // The woken node acks on the channel a fragment went out on, so RX must park
  // there between fragments instead of scanning the whole region
  RAIL_EnableRxChannelHopping(rail_handle, false, true);

  if (beam.power_deci_dbm != TX_POWER_UNCHANGED)
  {
    // RAIL coerces the requested power against the channel the radio is tuned
    // to, so each fragment sets it again for the channel it uses
    if (RAIL_PrepareChannel(rail_handle, channel) != RAIL_STATUS_NO_ERROR)
    {
      return TX_RESULT_UNKNOWN_ERROR;
    }

    // RAIL clamps the power to the PA curve and the channel's maximum
    if (RAIL_SetTxPowerDbm(rail_handle, (RAIL_TxPower_t)beam.power_deci_dbm) != RAIL_STATUS_NO_ERROR)
    {
      // The transmit must happen at the power the host requested
      return TX_RESULT_INVALID_PARAM;
    }
  }

  // Beam frames carry no length field, so fixed length mode makes RAIL send
  // exactly the bytes the host handed us. Fixed length also applies to RX and
  // must be restored before RX resumes.
  RAIL_SetFixedLength(rail_handle, beam.data_len);
  RAIL_SetTxAltPreambleLength(rail_handle, beam_preamble_bits(channel));

  if (RAIL_WriteTxFifo(rail_handle, beam.data, beam.data_len, true) != beam.data_len)
  {
    return TX_RESULT_UNKNOWN_ERROR;
  }
  if (RAIL_SetNextTxRepeat(rail_handle, &beam_repeat_config) != RAIL_STATUS_NO_ERROR)
  {
    // Without the repetition the fragment would carry a single beam frame
    return TX_RESULT_UNKNOWN_ERROR;
  }

  if (beam.fragment_index == 0)
  {
    beam.fragment_start = RAIL_GetTime();
  }
  else
  {
    // Count the period off the previous fragment's own reference, so the
    // latency of each fragment start does not accumulate over the beam
    beam.fragment_start += beam.fragment_period_us;
  }

  beam_transmitting = true;
  RAIL_Status_t status = RAIL_StartTx(rail_handle, channel, BEAM_TX_OPTIONS, NULL);
  if (status != RAIL_STATUS_NO_ERROR)
  {
    beam_transmitting = false;
    return status == RAIL_STATUS_INVALID_PARAMETER ? TX_RESULT_INVALID_PARAM : TX_RESULT_UNKNOWN_ERROR;
  }

  beam_arm_timer(rail_handle, beam.fragment_start + beam.fragment_duration_us);
  return TX_RESULT_QUEUED;
}

/// @brief Schedule the next beam phase, treating a deadline in the past as reached
static void beam_arm_timer(RAIL_Handle_t rail_handle, RAIL_Time_t deadline)
{
  // An absolute deadline keeps the fragment grid drift-free over a long beam.
  // RAIL rejects one that already passed, which means the phase is over.
  if (RAIL_SetTimer(rail_handle, deadline, RAIL_TIME_ABSOLUTE, beam_timer_expired) != RAIL_STATUS_NO_ERROR)
  {
    beam_timer_fired = true;
  }
}

/// @brief End the repeat train and listen on the fragment's channel for the ack
static void beam_pause_radio(RAIL_Handle_t rail_handle)
{
  // Idling clears the pending repeats and cuts any frame that is on air.
  // Receivers drop the truncated final frame.
  beam_transmitting = false;
  RAIL_Idle(rail_handle, RAIL_IDLE_ABORT, true);

  RAIL_SetFixedLength(rail_handle, RAIL_SETFIXEDLENGTH_INVALID);
  RAIL_YieldRadio(rail_handle);
  RAIL_StartRx(rail_handle, beam_fragment_channel(), NULL);
}

/// @brief Take the radio out of beam mode and back to hopped RX
static void beam_stop(RAIL_Handle_t rail_handle)
{
  RAIL_CancelTimer(rail_handle);

  // Idle while beam_active still holds, so the completion event of the aborted
  // repeat train reaches the beam branch of the event handler
  beam_transmitting = false;
  RAIL_Idle(rail_handle, RAIL_IDLE_ABORT, true);
  beam_active = false;

  RAIL_SetFixedLength(rail_handle, RAIL_SETFIXEDLENGTH_INVALID);
  RAIL_YieldRadio(rail_handle);

  RAIL_EnableRxChannelHopping(rail_handle, true, true);

  // Clear last, so an event the repeat train raised on its way out cannot leak
  // into the next FUNC_ID_TRANSMIT
  beam_timer_fired = false;
  beam_tx_error = 0;
  rail_packet_sent = false;
  tx_error = 0;
  tx_in_flight = false;

  // Enabling hopping idles the radio, so return to the state that starts RX
  rail_state = RAILS_IDLE;
}

/// @brief Stop the beam and report the result to the host
static void beam_end(RAIL_Handle_t rail_handle, tx_result_t result)
{
  beam_stop(rail_handle);
  callback_cmd_transmit_beam(result);
}

/// @brief Move the beam to its next phase after a fragment or a pause ended
static void beam_advance(RAIL_Handle_t rail_handle)
{
  if (beam_transmitting)
  {
    if (beam.fragment_index + 1 >= beam.num_fragments)
    {
      beam_end(rail_handle, TX_RESULT_COMPLETED);
      return;
    }

    beam_pause_radio(rail_handle);

    // G.9959 §8.1.3.11: "The next beam fragment shall begin in the range
    // 190-200 ms measured from the beginning of the previous beam fragment."
    beam_arm_timer(rail_handle, beam.fragment_start + beam.fragment_period_us);
    return;
  }

  beam.fragment_index++;
  tx_result_t result = beam_start_fragment(rail_handle);
  if (result != TX_RESULT_QUEUED)
  {
    beam_end(rail_handle, result);
  }
}

/// @brief RAIL timer callback, runs in interrupt context
static void beam_timer_expired(RAIL_Handle_t rail_handle)
{
  (void)rail_handle;
  beam_timer_fired = true;
}

/******************************************************************************
 * RAIL callback, called if a RAIL event occurs
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  rail_last_state = events;

  // Handle Tx events
  if ((events & RAIL_EVENTS_TX_COMPLETION) && beam_active)
  {
    // Every beam frame of the repeat train raises its own completion event,
    // which the fragment timer makes redundant. A failure is different: RAIL
    // stops the repetition on a TX error, so the fragment has to end early.
    if (beam_transmitting && !(events & RAIL_EVENT_TX_PACKET_SENT))
    {
      beam_tx_error = tx_result_from_events(events);
    }
  }
  else if (events & RAIL_EVENTS_TX_COMPLETION)
  {
    tx_in_flight = false;
    if (events & RAIL_EVENT_TX_PACKET_SENT)
    {
      rail_packet_sent = true;
    }
    else
    {
      tx_error = tx_result_from_events(events);
    }
    // Yield the radio even in single-protocol mode, where the docs call it a no-op.
    // It is required for RX to reliably receive ACKs.
    RAIL_YieldRadio(rail_handle);
  }

  // Handle Rx events
  if (events & RAIL_EVENTS_RX_COMPLETION)
  {
    if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
    {
      // Keep the packet in the radio buffer so the state machine can download it later
      rx_packet_handle = RAIL_HoldRxPacket(rail_handle);
      rail_packet_received = true;
    }
    else
    {
      rail_error = true;
    }
  }

  // Handle calibration events
  if (events & RAIL_EVENT_CAL_NEEDED)
  {
    calibration_status = RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    if (calibration_status != RAIL_STATUS_NO_ERROR)
    {
      rail_error = true;
    }
  }
#if defined(SL_CATALOG_KERNEL_PRESENT)
  app_task_notify();
#endif
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------

/**************************************************************************/ /**
                                                                              * @brief
                                                                              *    The EUSART0 receive interrupt saves incoming characters.
                                                                              *****************************************************************************/
void EUSART0_RX_IRQHandler(void)
{
  uint8_t byte = EUSART0->RXDATA;
  UART_RX_FIFO[uart_rx_pos++] = byte;
  if (uart_rx_pos == UART_RX_FIFO_SIZE)
  {
    // FIXME: We should use a ringbuffer
    uart_rx_pos = 0;
  }

  uart_rx_done = true;

  // The EUSART requires explicit clearing of the RX interrupt flag even after
  // emptying the RX FIFO
  EUSART_IntClear(EUSART0, EUSART_IF_RXFL);
}

/**************************************************************************/ /**
                                                                              * @brief
                                                                              *    The EUSART0 transmit interrupt outputs characters.
                                                                              *****************************************************************************/
void EUSART0_TX_IRQHandler(void)
{
  if (uart_tx_pos < uart_tx_len)
  {
    EUSART0->TXDATA = UART_TX_FIFO[uart_tx_pos++];

    // The EUSART requires explicit clearing of the TX FIFO interrupt flag even
    // after a write to the FIFO
    EUSART_IntClear(EUSART0, EUSART_IF_TXFL);
  }
  else
  {
    // Transmission finished, so reset the cursors
    uart_tx_done = true;
    uart_tx_pos = 0;
    uart_tx_len = 0;
    // The TX FIFO level interrupt must be disabled here, or it retriggers
    // immediately after the handler exits
    EUSART_IntDisable(EUSART0, EUSART_IEN_TXFL);
  }
}

/*******************************************************************************
 * Process the received packet (print data packet or indicate ACK)
 ******************************************************************************/
static void handle_received_packet(RAIL_Handle_t rail_handle)
{
  RAIL_RxPacketInfo_t packet_info;
  RAIL_RxPacketDetails_t packet_details;

  rx_packet_handle = RAIL_GetRxPacketInfo(rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
  if (rx_packet_handle == RAIL_RX_PACKET_HANDLE_INVALID)
  {
    return;
  }

  // The RSSI, LQI and channel the host is told about come from the details, so
  // a packet whose details are unavailable is dropped
  if (RAIL_GetRxPacketDetails(rail_handle, rx_packet_handle, &packet_details) != RAIL_STATUS_NO_ERROR)
  {
    RAIL_ReleaseRxPacket(rail_handle, rx_packet_handle);
    return;
  }

  uint16_t packet_size = packet_info.packetBytes;
  RAIL_CopyRxPacket(rx_fifo, &packet_info);

  notify_receive(rx_fifo, (uint8_t)packet_size, packet_details.rssi, packet_details.lqi, (uint8_t)packet_details.channel & 0xff);

  RAIL_ReleaseRxPacket(rail_handle, rx_packet_handle);
}
