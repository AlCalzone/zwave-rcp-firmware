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
extern uint8_t UART_RX_FIFO[UART_RX_FIFO_SIZE];
extern uint8_t UART_TX_FIFO[UART_TX_FIFO_SIZE];
extern uint32_t uart_rx_pos;
extern uint32_t uart_tx_pos;
extern uint32_t uart_tx_len;
extern bool uart_rx_done;
extern bool uart_tx_done;

extern uint8_t RAIL_RX_FIFO[RAIL_FIFO_SIZE];
extern uint8_t RAIL_TX_FIFO[RAIL_FIFO_SIZE];

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
  init_channel_hopping(rail_handle);

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

/**************************************************************************//**
 * @brief
 *    The EUSART0 receive interrupt saves incoming characters.
 *****************************************************************************/
void EUSART0_RX_IRQHandler(void)
{
  // Get the character just received
  uint8_t byte = EUSART0->RXDATA;
  if (uart_rx_pos < UART_RX_FIFO_SIZE) {
    UART_RX_FIFO[uart_rx_pos++] = byte;
  }

  // When the command is complete or the buffer is full, handle the command
  if (byte == '\r' || uart_rx_pos == UART_RX_FIFO_SIZE) {
    uart_rx_done = true;
  }

  /*
   * The EUSART differs from the USART in that explicit clearing of
   * RX interrupt flags is required even after emptying the RX FIFO.
   */
  EUSART_IntClear(EUSART0, EUSART_IF_RXFL);
}

/**************************************************************************//**
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

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------
