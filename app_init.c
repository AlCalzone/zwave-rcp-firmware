/***************************************************************************//**
 * @file
 * @brief app_init.c
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
#include "sl_rail_util_init.h"
#include "rail.h"
#include "rail_zwave.h"

#include <em_eusart.h>
#include <em_gpio.h>
#include "em_cmu.h"
#include "em_emu.h"
#include "pin_config.h"
#include "app_init.h"
#include "app_process.h"

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
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
//                                Static Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------
/******************************************************************************
 * The function is used for some basic initialization related to the app.
 *****************************************************************************/
RAIL_Handle_t app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your application init code here!                                    //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

  /// EUSART initialization
  // Enable clocks
  // CMU_ClockEnable(cmuClock_GPIO, true); // should be enabled by the stack
  CMU_ClockEnable(cmuClock_EUSART0, true);

  initGPIO();
  initEUSART0();

  // Start receiving on UART
  EUSART_IntEnable(EUSART0, EUSART_IF_RXFL);

  // Get RAIL handle, used later by the application
  RAIL_Handle_t rail_handle = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
  set_up_tx_fifo(rail_handle);
  init_rx_channel_hopping(rail_handle);

  return rail_handle;
}

void initGPIO(void)
{
  // Configure the USART TX pin to the board controller as an output
  GPIO_PinModeSet(EUSART0_TX_PORT, EUSART0_TX_PIN, gpioModePushPull, 1);
  // Configure the USART RX pin to the board controller as an input
  GPIO_PinModeSet(EUSART0_RX_PORT, EUSART0_RX_PIN, gpioModeInput, 0);
}

void initEUSART0(void)
{
  // Default asynchronous initializer (115.2 Kbps, 8N1, no flow control)
  EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_HF;

  // Route EUSART0 TX and RX to the board controller TX and RX pins
  GPIO->EUSARTROUTE[0].TXROUTE = (EUSART0_TX_PORT << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
      | (EUSART0_TX_PIN << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[0].RXROUTE = (EUSART0_RX_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
      | (EUSART0_RX_PIN << _GPIO_EUSART_RXROUTE_PIN_SHIFT);

  // Enable RX and TX signals now that they have been routed
  GPIO->EUSARTROUTE[0].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN | GPIO_EUSART_ROUTEEN_TXPEN;

  // Configure and enable EUSART0 for high-frequency (EM0/1) operation
  EUSART_UartInitHf(EUSART0, &init);

  // Enable NVIC USART sources
  NVIC_ClearPendingIRQ(EUSART0_RX_IRQn);
  NVIC_EnableIRQ(EUSART0_RX_IRQn);
  NVIC_ClearPendingIRQ(EUSART0_TX_IRQn);
  NVIC_EnableIRQ(EUSART0_TX_IRQn);
}

void init_rx_channel_hopping(RAIL_Handle_t rail_handle) {
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

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
