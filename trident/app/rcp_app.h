#ifndef RCP_APP_H
#define RCP_APP_H

#include <stdbool.h>
#include <stdint.h>
#include <zpal_uart.h>

/// Z-Wave RCP application for the Trident IoT SDK (T32CZ20)
///
/// The firmware runs as a single FreeRTOS task. Interrupt and timer contexts
/// only record what happened and wake the task with one of the event bits
/// below, so all protocol handling happens in task context, like the main
/// loop of the Silicon Labs build.

/// The radio holds at least one received frame
#define RCP_EVENT_RF_RX (1u << 0)
/// A frame or beam transmit finished, see radio_zpal_handle_tx_done()
#define RCP_EVENT_RF_TX_DONE (1u << 1)
/// The host sent bytes over UART
#define RCP_EVENT_UART_RX (1u << 2)
/// The beam fragment timer expired
#define RCP_EVENT_BEAM_TIMER (1u << 3)

/// @brief Wake the RCP task with the given event bits from any context
void rcp_notify(uint32_t events);

// --- Radio backend (radio_zpal.c) ---

/// @brief Configure the radio for the default region and start receiving
void radio_zpal_init(void);
/// @brief Hand received frames to the host, called for RCP_EVENT_RF_RX
void radio_zpal_handle_rx(void);
/// @brief Finish the transmit the radio reported, called for RCP_EVENT_RF_TX_DONE
void radio_zpal_handle_tx_done(void);
/// @brief Move a beam to its next fragment, called for RCP_EVENT_BEAM_TIMER
void radio_zpal_handle_beam_timer(void);

// --- UART backend (uart_zpal.c) ---

/// @brief Open the host UART described by the board's RCP_UART_CONFIG
void uart_zpal_init(void);
/// @brief Move received bytes into the serial link RX FIFO, called for RCP_EVENT_UART_RX
void uart_zpal_pump_rx(void);

// --- Board support (hardware/src_hw_*) ---

/// UART the host is attached to, provided by the board specific source
extern zpal_uart_config_t RCP_UART_CONFIG;

#endif // RCP_APP_H
