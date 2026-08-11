/***************************************************************************//**
 * @file
 * @brief app_process.h
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
#ifndef APP_PROCESS_H
#define APP_PROCESS_H

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include "rail.h"
#include "serial_api.h"

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Public Function Declarations
// -----------------------------------------------------------------------------

/******************************************************************************
 * Set up the rail TX fifo for later usage
 * @param[in] rail_handle Which rail handler should be updated
 *****************************************************************************/
void set_up_tx_fifo(RAIL_Handle_t rail_handle);

/**************************************************************************//**
 * The function is used for Application logic.
 *
 * @param[in] rail_handle RAIL handle
 *
 * The function is used for Application logic.
 * It is called infinitely.
 *****************************************************************************/
void app_process_action(RAIL_Handle_t rail_handle);

/// @brief Queue raw data for transmission over UART
void uart_transmit(uint8_t *data, uint32_t len);
/// @brief Queue a frame for transmission over UART
void uart_transmit_frame(frame_type_t frame_type, func_id_t func_id, uint8_t *payload, uint32_t payload_len);
/// @brief Queue a single byte for transmission over UART
void uart_transmit_byte(uint8_t byte);

/// @brief Queue raw data for transmission over radio
/// @param power_dbm Transmit power in dBm, coerced by RAIL to the channel's maximum
/// @param flags Bitmask of TRANSMIT_FLAG_*
void radio_transmit(uint8_t channel, int8_t power_dbm, uint8_t flags, uint8_t *data, uint32_t len);
/// @brief Change the region of the radio
bool radio_set_region(RAIL_Handle_t rail_handle, zwave_region_t region, zwave_channel_cfg_t channel_cfg, uint8_t* num_channels, channel_info_t* channels);
/// @brief Adopt the region RAIL currently has configured: channel count, RX channel hopping and RX restart
void radio_sync_active_region(RAIL_Handle_t rail_handle);
/// @brief Read the region of the radio and its information
void radio_get_region(RAIL_Handle_t rail_handle, zwave_region_t* region, zwave_channel_cfg_t* channel_cfg, uint8_t* num_channels, channel_info_t* channels);

// Interrupt handlers for UART
void EUSART0_RX_IRQHandler(void);
void EUSART0_TX_IRQHandler(void);

#endif  // APP_PROCESS_H
