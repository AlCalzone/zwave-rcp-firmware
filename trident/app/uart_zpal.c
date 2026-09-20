/// Host UART on the Trident IoT SDK, implemented on the ZPAL UART driver
///
/// The ZPAL driver receives into a ring buffer it owns and transmits straight
/// from the caller's memory with DMA, so this module keeps its own transmit
/// ring buffer and feeds the DMA one contiguous chunk at a time.

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include <zpal_uart.h>
#include <sysfun.h>
#include <Assert.h>

#include "common.h"
#include "serial_link.h"
#include "rcp_app.h"

/// Holds the UART frames waiting for the DMA. A received Z-Wave frame turns
/// into a UART frame of up to 256 + 9 bytes, and a few of those may queue up
/// while the host is slow to read
#define UART_TX_RING_SIZE 1024

static zpal_uart_handle_t uart_handle = NULL;

static uint8_t tx_ring[UART_TX_RING_SIZE];
/// Next byte the DMA has not been given yet
static volatile uint16_t tx_tail = 0;
/// Next free byte
static volatile uint16_t tx_head = 0;
/// Bytes handed to the DMA in the current chunk
static volatile uint16_t tx_chunk_len = 0;
/// A DMA chunk is on its way
static volatile bool tx_busy = false;
/// Bytes dropped because the ring was full, for debugging
static volatile uint32_t tx_dropped = 0;

static void tx_start_chunk(void);

static void tx_done_callback(zpal_uart_handle_t handle)
{
  (void)handle;
  // Runs in interrupt context
  tx_tail = (uint16_t)((tx_tail + tx_chunk_len) % UART_TX_RING_SIZE);
  tx_chunk_len = 0;
  tx_busy = false;
  tx_start_chunk();
}

/// @brief Give the DMA the bytes from the tail up to the head or the end of the ring
///
/// Must run with interrupts disabled or from the transmit done interrupt.
static void tx_start_chunk(void)
{
  if (tx_busy || tx_head == tx_tail)
  {
    return;
  }

  uint16_t chunk = (tx_head > tx_tail) ? (uint16_t)(tx_head - tx_tail) : (uint16_t)(UART_TX_RING_SIZE - tx_tail);
  tx_chunk_len = chunk;
  tx_busy = true;
  if (zpal_uart_transmit(uart_handle, &tx_ring[tx_tail], chunk, tx_done_callback) != ZPAL_STATUS_OK)
  {
    // The driver refused the chunk, so no completion follows. Drop it
    tx_tail = (uint16_t)((tx_tail + chunk) % UART_TX_RING_SIZE);
    tx_chunk_len = 0;
    tx_busy = false;
    tx_dropped += chunk;
  }
}

void uart_transmit(uint8_t *data, uint32_t len)
{
  enter_critical_section();

  uint16_t used = (uint16_t)((tx_head + UART_TX_RING_SIZE - tx_tail) % UART_TX_RING_SIZE);
  // One byte stays free so a full ring is distinguishable from an empty one
  uint32_t space = (uint32_t)(UART_TX_RING_SIZE - 1 - used);
  if (len > space)
  {
    // Dropping the tail of a frame corrupts it, dropping the whole frame
    // lets the host resynchronize on the next SOF
    tx_dropped += len;
    leave_critical_section();
    return;
  }

  for (uint32_t i = 0; i < len; i++)
  {
    tx_ring[tx_head] = data[i];
    tx_head = (uint16_t)((tx_head + 1) % UART_TX_RING_SIZE);
  }

  tx_start_chunk();

  leave_critical_section();
}

void uart_transmit_byte(uint8_t byte)
{
  uart_transmit(&byte, 1);
}

static void uart_receive_callback(zpal_uart_handle_t handle, size_t length)
{
  (void)handle;
  (void)length;
  rcp_notify(RCP_EVENT_UART_RX);
}

void uart_zpal_init(void)
{
  RCP_UART_CONFIG.receive_callback = uart_receive_callback;

  zpal_status_t status = zpal_uart_init(&RCP_UART_CONFIG, &uart_handle);
  ASSERT(status == ZPAL_STATUS_OK);
  status = zpal_uart_enable(uart_handle);
  ASSERT(status == ZPAL_STATUS_OK);
}

void uart_zpal_pump_rx(void)
{
  bool received = false;

  while (zpal_uart_get_available(uart_handle) > 0)
  {
    if (uart_rx_pos >= UART_RX_FIFO_SIZE)
    {
      // Same policy as the EFR32 interrupt handler: a FIFO the parser did
      // not drain starts over
      // FIXME: We should use a ringbuffer
      uart_rx_pos = 0;
    }

    size_t room = UART_RX_FIFO_SIZE - uart_rx_pos;
    size_t read = zpal_uart_receive(uart_handle, &UART_RX_FIFO[uart_rx_pos], room);
    if (read == 0)
    {
      break;
    }
    uart_rx_pos += read;
    received = true;
  }

  if (received)
  {
    uart_rx_done = true;
  }
}
