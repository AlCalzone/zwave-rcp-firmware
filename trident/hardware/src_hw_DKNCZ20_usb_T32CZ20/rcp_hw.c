/// Board support for the Trident IoT DKNCZ20 development kit (DKNCZ20B20)
///
/// The host talks to the RCP over UART0, which the kit routes to its on-board
/// USB to UART bridge.

#include <zpal_uart.h>
#include <zpal_uart_gpio.h>
#include <tr_board_DKNCZ20.h>

#include "rcp_app.h"

#ifndef RCP_UART_BAUD_RATE
/// The ZPAL driver offers fixed baud rates. 500000 is closest to the EFR32
/// build's 460800 baud.
#define RCP_UART_BAUD_RATE 500000
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
