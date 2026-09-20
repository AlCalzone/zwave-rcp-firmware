#ifndef SERIAL_LINK_H
#define SERIAL_LINK_H

#include <stdbool.h>
#include <stdint.h>
#include "common.h"
#include "serial_api.h"

/// Serial link framing shared by all platforms
///
/// FRAME STRUCTURE
/// SOF | LEN | TYPE | FUNC_ID | DATA... | CHKSUM
///
/// The platform feeds received bytes into UART_RX_FIFO and sets uart_rx_done,
/// then calls serial_link_process_rx() from its main loop or task. Outgoing
/// bytes leave through uart_transmit() and uart_transmit_byte(), which each
/// platform implements on top of its UART driver.

/// Bytes received from the host that await parsing
extern uint8_t UART_RX_FIFO[UART_RX_FIFO_SIZE];
/// The position of the write cursor in the RX FIFO
extern volatile uint32_t uart_rx_pos;
/// New UART RX data arrived and awaits processing
extern volatile bool uart_rx_done;

/// @brief Parse the RX FIFO, acknowledge complete frames and dispatch the requests they carry
void serial_link_process_rx(void);

/// @brief Queue a frame for transmission over UART
void uart_transmit_frame(frame_type_t frame_type, func_id_t func_id, uint8_t *payload, uint32_t payload_len);

/// @brief Queue raw data for transmission over UART (platform specific)
void uart_transmit(uint8_t *data, uint32_t len);
/// @brief Queue a single byte for transmission over UART (platform specific)
void uart_transmit_byte(uint8_t byte);

#endif // SERIAL_LINK_H
