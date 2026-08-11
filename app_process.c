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

/// The Z-Wave PHY appends a checksum that beam frames must not carry, sends
/// exactly the fixed length configured below, and takes the beam preamble
/// length from RAIL_SetTxAltPreambleLength(). RESEND lets every repeat reuse
/// the frame written to the TX FIFO once.
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
static uint16_t beam_preamble_bits(zwave_baudrate_t baud);
static tx_result_t beam_start_fragment(RAIL_Handle_t rail_handle);
static void beam_pause_radio(RAIL_Handle_t rail_handle);
static void beam_end(RAIL_Handle_t rail_handle, tx_result_t result);
static void beam_advance(RAIL_Handle_t rail_handle);
static void beam_timer_expired(RAIL_Handle_t rail_handle);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
rail_state_t rail_state = RAILS_IDLE;

// The received UART data
uint8_t UART_RX_FIFO[UART_RX_FIFO_SIZE] = {0};
// The position of the write cursor in the RX FIFO
uint32_t uart_rx_pos = 0;

uint8_t UART_TX_FIFO[UART_TX_FIFO_SIZE] = {0};
uint32_t uart_tx_pos = 0;
uint32_t uart_tx_len = 0;

/// Indicates that there is new data to be processed
bool uart_rx_done = false;
bool uart_tx_done = false;

static uint8_t tx_channel = 0;
static int16_t tx_power_deci_dbm = 0;
static uint8_t tx_flags = 0;
/// Number of channels the currently configured region has
static uint8_t region_num_channels = 0;
/// Data rate of each channel of the currently configured region
static zwave_baudrate_t region_channel_baud[RAIL_NUM_ZWAVE_CHANNELS] = {0};
static uint8_t OUT_PACKET[RAIL_FIFO_SIZE] = {0};
static uint32_t out_packet_len = 0;

// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

/// State machine flags and conditions
/// Notify end of packet transmission
static bool rail_packet_sent = false;

/// Notify reception of packet
static bool rail_packet_received = false;

/// Notify RAIL Tx or Rx error
static bool rail_error = false;
static uint8_t tx_error = 0;

/// A transmit was handed to RAIL and has not completed yet. The radio owns
/// the TX FIFO and the PA power until then, so no other transmit may start.
static volatile bool tx_in_flight = false;

/// Request start receiving
static bool start_rx = true;

/// A beam is running, so TX completion events belong to its repeat train and
/// no other transmit may start
static volatile bool beam_active = false;

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
  int8_t power_dbm;
  /// RAIL time the current fragment started, the reference the period counts from
  RAIL_Time_t fragment_start;
  /// The repeat train is on air
  bool transmitting;
} beam = {0};

/// Repeat the beam frame until the fragment timer stops it. RAIL chains the
/// repeats in hardware, so only the PA ramp separates two frames.
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

/// Contains the status of RAIL Calibration
static volatile RAIL_Status_t calibration_status = 0;

/// RAIL Rx packet handle
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

/// Receive and Send FIFO
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
  // app_assert(allocated_tx_fifo_size == RAIL_FIFO_SIZE,
  //            "RAIL_SetTxFifo() failed to allocate a large enough fifo (%d bytes instead of %d bytes)\n",
  //            allocated_tx_fifo_size,
  //            RAIL_FIFO_SIZE);
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
    // RAIL is listening for packets
    if (rail_packet_received)
    {
      // A complete packet was received, handle it
      rail_packet_received = false;
      handle_received_packet(rail_handle);
    }
    else if (out_packet_len > 0)
    {
      // There is a packet in the outgoing buffer, switch to TX state
      rail_state = RAILS_TX;
    }
  }
  else if (rail_state == RAILS_TX)
  {
    if (out_packet_len > 0)
    {
      // Queue the packet for transmit
      rail_transmit(rail_handle, OUT_PACKET, out_packet_len);
      out_packet_len = 0;
    }
    else if (rail_packet_sent)
    {
      callback_cmd_transmit(TX_RESULT_COMPLETED);
      // if it was sent, switch back to RX
      rail_packet_sent = false;
      // TODO: Notify application that packet was sent
      rail_state = RAILS_RX;
    }
    else if (tx_error != 0)
    {
      // FIXME: Figure out what the error is
      callback_cmd_transmit((tx_result_t)tx_error);
      // if there was an error, also switch back to RX
      tx_error = 0;
      // TODO: Notify application that there was an error
      rail_state = RAILS_RX;
    }
  }
  else if (rail_state == RAILS_BEAM)
  {
    if (beam_timer_fired)
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

  // Whenever a complete serial frame was received via UART, handle it
  if (uart_rx_done)
  {
    uart_rx_done = false;
    uart_handle_rx(rail_handle);
  }
}

/// @brief Queue a frame for transmission over UART
void uart_transmit(uint8_t *data, uint32_t len)
{
  // Copy the data onto the TX FIFO
  memcpy(&UART_TX_FIFO[uart_tx_len], data, len);
  uart_tx_len += len;
  uart_tx_done = false;
  // And enable the TX interrupt that handles the transmission
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
  // Copy the data onto the TX FIFO
  UART_TX_FIFO[uart_tx_len++] = byte;
  uart_tx_done = false;
  // And enable the TX interrupt that handles the transmission
  EUSART_IntEnable(EUSART0, EUSART_IEN_TXFL);
}

/// @brief Handle a received frame over UART
static void uart_handle_rx(RAIL_Handle_t rail_handle)
{
  // Skip all data that does not start with SOF
  int i = 0;
  while (
      // There is something to read
      i < uart_rx_pos
      // that is not a SOF
      && UART_RX_FIFO[i] != SOF)
  {
    i++;
  }
  if (i == uart_rx_pos)
  {
    // No SOF found
    reset_rx_fifo(i);
    return;
  }

  // We need SOF and LEN to validate the frame length
  uint8_t remaining = uart_rx_pos - i;
  if (remaining < 2)
  {
    // Not enough data to process a frame
    return;
  }

  uint8_t chksum = 0xff;
  // Skip SOF
  i++;

  // Check the length of the frame, not including SOF and checksum
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
    // Not enough data to process a frame
    return;
  }

  // Extract frame and update checksum
  uint8_t cmd[len - 1];
  for (int j = 0; j < len - 1; j++)
  {
    cmd[j] = UART_RX_FIFO[i++];
    chksum ^= cmd[j];
  }

  // Compare with actual checksum. chksum should now be 0 if it matches
  chksum ^= UART_RX_FIFO[i++];

  // Move remaining data to start of buffer
  // FIXME: Use a ringbuffer so we can avoid this
  if (i < uart_rx_pos)
  {
    reset_rx_fifo(i);
    // There is still data left to handle
    uart_rx_done = true;
  }
  uart_rx_pos = 0;

  if (chksum == 0)
  {
    // Checksum valid
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
    // We only handle requests
    return;
  }

  // Process the frame
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
    // Unknown command
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

    // For LR regions, the actual configuration depends on the channel configuration
    // which is ignored for the non-LR regions
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

  // Expose the channel information to the host
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

  // Expose the channel information to the host
  export_channel_info(region_config, num_channels, channels);
}

/// Cache the channel count and data rates of the region RAIL currently has
/// configured and reconfigure RX channel hopping for it. Transmit validation
/// reads this cached state, so it must be refreshed whenever the region
/// changes.
void radio_sync_active_region(RAIL_Handle_t rail_handle)
{
  zwave_region_t region;
  zwave_channel_cfg_t channel_cfg;
  const RAIL_ZWAVE_RegionConfig_t *region_config = resolve_active_region(rail_handle, &region, &channel_cfg);

  // A running beam addresses channels of the region it was started in
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

  for (uint8_t i = 0; i < region_num_channels; i++)
  {
    region_channel_baud[i] = channels[i].baud;
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
    // The packet is too large
    respond_cmd_transmit(TX_RESULT_OVERFLOW);
    return;
  }
  if (channel >= region_num_channels)
  {
    // Invalid channel
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
      // Transmitting at the previous power would misreport what the host asked for
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
    // TODO: Figure out oother possible errors and expose them
    respond_cmd_transmit(TX_RESULT_UNKNOWN_ERROR);
  }
}

void radio_transmit_beam(RAIL_Handle_t rail_handle, int8_t power_dbm, uint8_t num_fragments, uint16_t fragment_duration_ms, uint16_t fragment_period_ms, uint8_t num_channels, const uint8_t *channels, const uint8_t *data, uint8_t data_len)
{
  if (out_packet_len > 0 || tx_in_flight || beam_active)
  {
    // There is already a packet in the buffer or on the air
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
    // Fragments would overlap
    respond_cmd_transmit_beam(TX_RESULT_INVALID_PARAM);
    return;
  }

  // Every fragment must be startable, so reject the whole beam if any channel
  // is unknown or carries a rate that has no beam preamble length
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
  beam.power_dbm = power_dbm;
  beam.transmitting = false;

  beam_active = true;
  beam_timer_fired = false;
  rail_state = RAILS_BEAM;

  tx_result_t result = beam_start_fragment(rail_handle);
  if (result != TX_RESULT_QUEUED)
  {
    // The host learns about the failure from the response, so no callback follows
    beam_active = false;
    rail_state = RAILS_RX;
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

static uint16_t beam_preamble_bits(zwave_baudrate_t baud)
{
  switch (baud)
  {
  case ZWAVE_BAUD_40k:
    return BEAM_PREAMBLE_BITS_R2;
  case ZWAVE_BAUD_100k:
    return BEAM_PREAMBLE_BITS_R3;
  case ZWAVE_BAUD_LR100k:
    return BEAM_PREAMBLE_BITS_LR;
  default:
    // G.9959 Table 7-10 lists no beam preamble length for R1
    return BEAM_PREAMBLE_BITS_NONE;
  }
}

/// @brief Put the repeat train for the current fragment on air
static tx_result_t beam_start_fragment(RAIL_Handle_t rail_handle)
{
  uint8_t channel = beam.channels[beam.fragment_index % beam.num_channels];

  // Take the radio out of RX channel hopping before reconfiguring the PHY
  RAIL_Idle(rail_handle, RAIL_IDLE_ABORT, true);

  if (beam.power_dbm != (int8_t)TX_POWER_UNCHANGED)
  {
    // RAIL coerces the requested power against the channel the radio is tuned
    // to, so each fragment sets it again for the channel it uses
    RAIL_PrepareChannel(rail_handle, channel);

    // RAIL takes deci-dBm and clamps to the PA curve and the channel's maximum
    if (RAIL_SetTxPowerDbm(rail_handle, (RAIL_TxPower_t)beam.power_dbm * 10) != RAIL_STATUS_NO_ERROR)
    {
      // Transmitting at the previous power would misreport what the host asked for
      beam_pause_radio(rail_handle);
      return TX_RESULT_INVALID_PARAM;
    }
  }

  // Beam frames carry no length field for the PHY's variable length decoding
  // to read, so fixed length mode makes RAIL send exactly the bytes the host
  // handed us. This also applies to RX, so it is restored before RX resumes.
  RAIL_SetFixedLength(rail_handle, beam.data_len);
  RAIL_SetTxAltPreambleLength(rail_handle, beam_preamble_bits(region_channel_baud[channel]));
  RAIL_WriteTxFifo(rail_handle, beam.data, beam.data_len, true);
  RAIL_SetNextTxRepeat(rail_handle, &beam_repeat_config);

  beam.transmitting = true;
  beam.fragment_start = RAIL_GetTime();
  RAIL_Status_t status = RAIL_StartTx(rail_handle, channel, BEAM_TX_OPTIONS, NULL);
  if (status != RAIL_STATUS_NO_ERROR)
  {
    beam_pause_radio(rail_handle);
    return status == RAIL_STATUS_INVALID_PARAMETER ? TX_RESULT_INVALID_PARAM : TX_RESULT_UNKNOWN_ERROR;
  }

  // Absolute deadlines keep the fragment grid free of the drift that relative
  // timers accumulate over a long beam
  RAIL_SetTimer(rail_handle, beam.fragment_start + beam.fragment_duration_us, RAIL_TIME_ABSOLUTE, beam_timer_expired);
  return TX_RESULT_QUEUED;
}

/// @brief End the repeat train and hand the radio back to RX
static void beam_pause_radio(RAIL_Handle_t rail_handle)
{
  // Idling clears the pending repeats. It also cuts the frame that is on air
  // at that moment, so the last frame of a fragment reaches a receiver as a
  // truncated frame it drops.
  RAIL_Idle(rail_handle, RAIL_IDLE_ABORT, true);
  beam.transmitting = false;

  RAIL_SetFixedLength(rail_handle, RAIL_SETFIXEDLENGTH_INVALID);
  RAIL_YieldRadio(rail_handle);
  RAIL_StartRx(rail_handle, 0, NULL);
}

/// @brief Stop the beam, report result to the host and return to the RX state
static void beam_end(RAIL_Handle_t rail_handle, tx_result_t result)
{
  RAIL_CancelTimer(rail_handle);
  beam_timer_fired = false;
  beam_active = false;

  if (beam.transmitting)
  {
    beam_pause_radio(rail_handle);
  }

  rail_state = RAILS_RX;
  callback_cmd_transmit_beam(result);
}

/// @brief Move the beam to its next phase after a fragment or a pause ended
static void beam_advance(RAIL_Handle_t rail_handle)
{
  if (beam.transmitting)
  {
    if (beam.fragment_index + 1 >= beam.num_fragments)
    {
      beam_end(rail_handle, TX_RESULT_COMPLETED);
      return;
    }

    beam_pause_radio(rail_handle);

    // G.9959 §8.1.3.11: "The next beam fragment shall begin in the range
    // 190-200 ms measured from the beginning of the previous beam fragment."
    RAIL_SetTimer(rail_handle, beam.fragment_start + beam.fragment_period_us, RAIL_TIME_ABSOLUTE, beam_timer_expired);
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
  // Make a copy of the events
  rail_last_state = events;

  // Handle Tx events
  if ((events & RAIL_EVENTS_TX_COMPLETION) && beam_active)
  {
    // Every beam frame of the repeat train raises its own completion event.
    // The fragment timer ends the train, so the events need no bookkeeping.
  }
  else if (events & RAIL_EVENTS_TX_COMPLETION)
  {
    tx_in_flight = false;
    if (events & RAIL_EVENT_TX_PACKET_SENT)
    {
      rail_packet_sent = true;
    }
    else if (events & RAIL_EVENT_TX_ABORTED)
    {
      tx_error = TX_RESULT_ABORTED;
    }
    else if (events & RAIL_EVENT_TX_BLOCKED)
    {
      tx_error = TX_RESULT_BLOCKED;
    }
    else if (events & RAIL_EVENT_TX_UNDERFLOW)
    {
      tx_error = TX_RESULT_UNDERFLOW;
    }
    else if (events & RAIL_EVENT_TX_CHANNEL_BUSY)
    {
      tx_error = TX_RESULT_CHANNEL_BUSY;
    }
    else
    {
      tx_error = TX_RESULT_UNKNOWN_ERROR;
    }
    // Documentation indicates that it does nothing for single protocol,
    // but without it, we've frequently missed receiving ACKs that other radios
    // are picking up.
    // I've also seen it in Z-Wave controller binaries. So this seems to be required.
    RAIL_YieldRadio(rail_handle);
  }

  // Handle Rx events
  if (events & RAIL_EVENTS_RX_COMPLETION)
  {
    if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
    {
      // Keep the packet in the radio buffer, download it later at the state machine
      rx_packet_handle = RAIL_HoldRxPacket(rail_handle);
      rail_packet_received = true;
    }
    else
    {
      rail_error = true;
    }
  }

  // Perform all calibrations when needed or indicate error if failed
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
  // Get the character just received
  uint8_t byte = EUSART0->RXDATA;
  UART_RX_FIFO[uart_rx_pos++] = byte;
  // Wrap around after reaching the end of the buffer
  if (uart_rx_pos == UART_RX_FIFO_SIZE)
  {
    // FIXME: We should use a ringbuffer
    uart_rx_pos = 0;
  }

  // Signal that there is something to process
  uart_rx_done = true;

  /*
   * The EUSART differs from the USART in that explicit clearing of
   * RX interrupt flags is required even after emptying the RX FIFO.
   */
  EUSART_IntClear(EUSART0, EUSART_IF_RXFL);
}

/**************************************************************************/ /**
                                                                              * @brief
                                                                              *    The EUSART0 transmit interrupt outputs characters.
                                                                              *****************************************************************************/
void EUSART0_TX_IRQHandler(void)
{
  // Send a previously queued character
  if (uart_tx_pos < uart_tx_len)
  {
    EUSART0->TXDATA = UART_TX_FIFO[uart_tx_pos++];

    /*
     * The EUSART differs from the USART in that the TX FIFO interrupt
     * flag must be explicitly cleared even after a write to the FIFO.
     */
    EUSART_IntClear(EUSART0, EUSART_IF_TXFL);
  }
  else
  /*
   * Need to disable the transmit FIFO level interrupt in this IRQ
   * handler when done or it will immediately trigger again upon exit
   * even though there is no data left to send.
   */
  {
    // Done transmitting - reset cursors
    uart_tx_done = true;
    uart_tx_pos = 0;
    uart_tx_len = 0;
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
  RAIL_Status_t packet_status;
  /// Status indicator of the RAIL API calls
  RAIL_Status_t rail_status;

  //  - Check whether RAIL_HoldRxPacket() was successful, i.e. packet handle is valid
  //  - Copy it to the application FIFO
  //  - Free up the radio FIFO
  //  - Return to IDLE state i.e. RAIL Rx
  if (rx_packet_handle == RAIL_RX_PACKET_HANDLE_INVALID)
  {
    // app_log_error("RAIL_HoldRxPacket() error: RAIL_RX_PACKET_HANDLE_INVALID\n"
    //               "No such RAIL rx packet yet exists or rail_handle is not active");
  }
  rx_packet_handle = RAIL_GetRxPacketInfo(rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
  if (rx_packet_handle == RAIL_RX_PACKET_HANDLE_INVALID)
  {
    // app_log_error("RAIL_GetRxPacketInfo() error: RAIL_RX_PACKET_HANDLE_INVALID\n");
  }
  if (rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID)
  {
    // Get packet details to identify ACK of last Tx
    packet_status = RAIL_GetRxPacketDetails(rail_handle, rx_packet_handle, &packet_details);
    if (packet_status != RAIL_STATUS_NO_ERROR)
    {
      // app_log_error("RAIL_GetRxPacketDetails() error: %lu\n", packet_status);
    }

    uint16_t packet_size = packet_info.packetBytes;
    RAIL_CopyRxPacket(rx_fifo, &packet_info);

    // FIXME: Include info about the received package (RSSI etc.)
    notify_receive(rx_fifo, (uint8_t)packet_size, packet_details.rssi, packet_details.lqi, (uint8_t)packet_details.channel & 0xff);

    rail_status = RAIL_ReleaseRxPacket(rail_handle, rx_packet_handle);
    if (rail_status != RAIL_STATUS_NO_ERROR)
    {
      // app_log_warning("RAIL_ReleaseRxPacket() result: %lu\n", rail_status);
    }
  }
}
