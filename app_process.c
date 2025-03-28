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
#include "rail.h"
#include "rail_zwave.h"
#include "common.h"
#include <em_eusart.h>

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
} rail_state_t;

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
static void handle_received_packet(RAIL_Handle_t rail_handle);

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
rail_state_t rail_state = RAILS_IDLE;

uint8_t UART_RX_FIFO[UART_RX_FIFO_SIZE] = {0};
uint8_t UART_TX_FIFO[UART_TX_FIFO_SIZE] = {0};
uint32_t uart_rx_pos = 0;
uint32_t uart_tx_pos = 0;
uint32_t uart_tx_len = 0;
bool uart_rx_done = false;
bool uart_tx_done = false;

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

/// Request start receiving
static bool start_rx = true;

/// Copy of last RAIL events to process
static RAIL_Events_t rail_last_state = RAIL_EVENTS_NONE;

/// Contains the status of RAIL Calibration
static volatile RAIL_Status_t calibration_status = 0;

/// RAIL Rx packet handle
static volatile RAIL_RxPacketHandle_t rx_packet_handle = RAIL_RX_PACKET_HANDLE_INVALID;

/// Receive and Send FIFO
static __ALIGNED(RAIL_FIFO_ALIGNMENT) uint8_t rx_fifo[RAIL_FIFO_SIZE];
static __ALIGNED(RAIL_FIFO_ALIGNMENT) uint8_t tx_fifo[RAIL_FIFO_SIZE];

// RX channel hopping
#define CHANNEL_HOPPING_NUMBER_OF_CHANNELS RAIL_NUM_ZWAVE_CHANNELS
// The documentation explains some complicated formula to calculate the buffer size,
// but using that buffer size results in error 0x21 (RAIL_STATUS_INVALID_PARAMETER).
// It seems that RAIL wants a size of 1050
#define CHANNEL_HOPPING_BUFFER_SIZE 1050

RAIL_RxChannelHoppingConfigEntry_t channelHoppingEntries[CHANNEL_HOPPING_NUMBER_OF_CHANNELS];
uint32_t channelHoppingBuffer[CHANNEL_HOPPING_BUFFER_SIZE];

RAIL_RxChannelHoppingConfig_t channelHoppingConfig = {
  .buffer = channelHoppingBuffer,
  .bufferLength = CHANNEL_HOPPING_BUFFER_SIZE,
  .numberOfChannels = CHANNEL_HOPPING_NUMBER_OF_CHANNELS,
  .entries = channelHoppingEntries
};

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
void init_channel_hopping(RAIL_Handle_t rail_handle) {
  // Force the radio into idle mode
  RAIL_Status_t status = RAIL_Idle(rail_handle, RAIL_IDLE_ABORT, false);
  if (status != RAIL_STATUS_NO_ERROR) {
    uint16_t buf[1] = {status};
    uart_transmit((uint8_t*) buf, 2);
  }

  // Populate the channel hopping settings for Z-Wave
  status = RAIL_ZWAVE_ConfigRxChannelHopping(rail_handle, &channelHoppingConfig);
  if (status != RAIL_STATUS_NO_ERROR) {
    // app_log_error("RAIL_ZWAVE_ConfigRxChannelHopping() failed with status %d\n", status);
      uint16_t buf[1] = {status};
      uart_transmit((uint8_t*) buf, 2);
  }

  status = RAIL_EnableRxChannelHopping(rail_handle, true, true);
  if (status != RAIL_STATUS_NO_ERROR) {
    uint16_t buf[1] = {status};
    uart_transmit((uint8_t*) buf, 2);
  }
}

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

  if (rail_state == RAILS_IDLE)
  {
    RAIL_StartRx(rail_handle, 0, NULL);
    rail_state = RAILS_RX;
  }
  else if (rail_state == RAILS_RX)
  {
    if (rail_packet_received)
    {
      rail_packet_received = false;
      handle_received_packet(rail_handle);
    } else if (out_packet_len > 0) {
      // Transmit requested
      rail_state = RAILS_TX;
    }
  } else if (rail_state = RAILS_TX) {
    if (out_packet_len > 0) {
      rail_transmit(rail_handle, OUT_PACKET, out_packet_len);
      out_packet_len = 0;
    } else if (rail_packet_sent) {
      rail_packet_sent = false;
      rail_state = RAILS_IDLE;
    } else if (rail_error) {
      // TODO: Handle
      rail_error = false;
      rail_state = RAILS_IDLE;
    }
  }

  if (uart_rx_done) {
    uart_rx_done = false;
    uart_handle_rx();
  }

  // if (uart_tx_done) {
  //   // Reset cursors
  //   uart_tx_done = false;
  //   uart_tx_pos = 0;
  //   uart_tx_len = 0;
  // }
}

void uart_transmit(uint8_t *data, uint32_t len)
{
  memcpy(&UART_TX_FIFO[uart_tx_len], data, len);
  uart_tx_len += len;
  uart_tx_done = false;
  EUSART_IntEnable(EUSART0, EUSART_IEN_TXFL);
}

void uart_handle_rx()
{
  uint8_t cmd[UART_RX_FIFO_SIZE / 2] = {0};
  int cmd_len = 0;
  for (int i = 0; i < uart_rx_pos; i += 2)
  {
    char c = UART_RX_FIFO[i];
    if (c == '\r')
    {
      // End of frame
      break;
    }

    if (c >= '0' && c <= '9')
    {
      cmd[i / 2] += (c - '0') << 4;
    }
    else
    {
      cmd[i / 2] += (c - 'a' + 10) << 4;
    }
    cmd_len++;

    c = UART_RX_FIFO[i + 1];
    if (c == '\r')
    {
      // End of frame
      break;
    }

    if (c >= '0' && c <= '9')
    {
      cmd[i / 2] += c - '0';
    }
    else
    {
      cmd[i / 2] += c - 'a' + 10;
    }
  }

  // After receiving a complete frame, reset the read position
  uart_rx_pos = 0;

  memcpy(OUT_PACKET, cmd, cmd_len);
  out_packet_len = cmd_len;
}

void rail_transmit(RAIL_Handle_t rail_handle, uint8_t *data, uint32_t len)
{
  RAIL_WriteTxFifo(rail_handle, data, len, true);
  RAIL_Status_t rail_status = RAIL_StartTx(rail_handle, 0, RAIL_TX_OPTION_WAIT_FOR_ACK, NULL);
  if (rail_status != RAIL_STATUS_NO_ERROR)
  {
    // app_log_warning("RAIL_StartTx() result: %lu\n ", rail_status);
  }
}

/******************************************************************************
 * RAIL callback, called if a RAIL event occurs
 *****************************************************************************/
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  // Make a copy of the events
  rail_last_state = events;

  // Handle Tx events
  if (events & RAIL_EVENTS_TX_COMPLETION)
  {
    if (events & RAIL_EVENT_TX_PACKET_SENT)
    {
      rail_packet_sent = true;
    }
    else
    {
      rail_error = true;
    }
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

    uint8_t hex[UART_RX_FIFO_SIZE] = {0};
    for (int i = 0; i < packet_size; i++)
    {
      uint8_t nibble = rx_fifo[i] >> 4;
      if (nibble < 10)
      {
        hex[i * 2] = '0' + nibble;
      }
      else
      {
        hex[i * 2] = 'a' + (nibble - 10);
      }

      nibble = rx_fifo[i] & 0xF;
      if (nibble < 10)
      {
        hex[i * 2 + 1] = '0' + nibble;
      }
      else
      {
        hex[i * 2 + 1] = 'a' + (nibble - 10);
      }
    }
    hex[packet_size * 2] = '\n';
    uart_transmit(hex, packet_size * 2 + 1);

    rail_status = RAIL_ReleaseRxPacket(rail_handle, rx_packet_handle);
    if (rail_status != RAIL_STATUS_NO_ERROR)
    {
      // app_log_warning("RAIL_ReleaseRxPacket() result: %lu\n", rail_status);
    }
  }
}
