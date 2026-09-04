/// Board support for the Trident IoT DKNCZ20 development kit (DKNCZ20B20)
///
/// The host talks to the RCP over UART0, which the kit routes to its on-board
/// USB to UART bridge.

#include <zpal_uart.h>
#include <zpal_uart_gpio.h>
#include <tr_board_DKNCZ20.h>

#include "rcp_app.h"

#ifndef RCP_UART_BAUD_RATE
/// The T32CZ20 UART only offers a fixed set of baud rates and 460800, which
/// the EFR32 build uses, is not one of them. 230400 is the closest supported
/// rate that keeps up with the radio.
#define RCP_UART_BAUD_RATE 230400
#endif

// The ZPAL driver receives into this ring buffer. The task drains it on every
// receive interrupt, so it only needs to cover the interrupt latency
#define RCP_UART_RX_BUFFER_SIZE 256

static uint8_t uart_rx_buffer[RCP_UART_RX_BUFFER_SIZE] __attribute__((aligned(4)));

static const zpal_uart_config_ext_t uart_gpio_config = {
    .txd_pin = TR_BOARD_UART0_TX,
    .rxd_pin = TR_BOARD_UART0_RX,
    .cts_pin = 0, // Not used
    .rts_pin = 0, // Not used
    .uart_wakeup = false,
};

zpal_uart_config_t RCP_UART_CONFIG = {
    .id = ZPAL_UART0,
    // Transmits go straight from the application's ring buffer over DMA
    .tx_buffer = NULL,
    .tx_buffer_len = 0,
    .rx_buffer = uart_rx_buffer,
    .rx_buffer_len = RCP_UART_RX_BUFFER_SIZE,
    .baud_rate = RCP_UART_BAUD_RATE,
    .data_bits = 8,
    .parity_bit = ZPAL_UART_NO_PARITY,
    .stop_bits = ZPAL_UART_STOP_BITS_1,
    .receive_callback = NULL, // Set by uart_zpal_init()
    .ptr = &uart_gpio_config,
    .flags = 0,
};
