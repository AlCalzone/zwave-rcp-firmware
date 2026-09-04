#include <string.h>
#include "serial_link.h"
#include "serial_api.h"

uint8_t UART_RX_FIFO[UART_RX_FIFO_SIZE] = {0};
volatile uint32_t uart_rx_pos = 0;
volatile bool uart_rx_done = false;

static void reset_rx_fifo(uint8_t new_start);
static void dispatch_frame(frame_type_t frame_type, func_id_t func_id, uint8_t *payload, uint8_t len);

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

/// @brief Handle a received frame over UART
void serial_link_process_rx(void)
{
  // Advance to the first SOF byte
  int i = 0;
  while (
      i < uart_rx_pos
      && UART_RX_FIFO[i] != SOF)
  {
    i++;
  }
  if (i == uart_rx_pos)
  {
    // The buffer contains no SOF
    reset_rx_fifo(i);
    return;
  }

  // We need SOF and LEN to validate the frame length
  uint8_t remaining = uart_rx_pos - i;
  if (remaining < 2)
  {
    return;
  }

  uint8_t chksum = 0xff;
  // Skip SOF
  i++;

  // Read the frame length, which excludes SOF and checksum
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
    // Wait for the rest of the frame to arrive
    return;
  }

  // Extract frame and update checksum
  uint8_t cmd[len - 1];
  for (int j = 0; j < len - 1; j++)
  {
    cmd[j] = UART_RX_FIFO[i++];
    chksum ^= cmd[j];
  }

  // XOR in the received checksum so a match leaves chksum at 0
  chksum ^= UART_RX_FIFO[i++];

  // Move remaining data to start of buffer
  // FIXME: Use a ringbuffer so we can avoid this
  if (i < uart_rx_pos)
  {
    reset_rx_fifo(i);
    // Trigger another processing pass for the buffered data
    uart_rx_done = true;
  }
  uart_rx_pos = 0;

  if (chksum == 0)
  {
    uart_transmit_byte(ACK);
    dispatch_frame(cmd[0], cmd[1], &cmd[2], len - 3);
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

static void dispatch_frame(frame_type_t frame_type, func_id_t func_id, uint8_t *payload, uint8_t len)
{
  if (frame_type != FRAME_TYPE_REQ)
  {
    return;
  }

  switch (func_id)
  {
  case FUNC_ID_GET_FIRMWARE_INFO:
    handle_cmd_get_firmware_info(payload, len);
    break;

  case FUNC_ID_SETUP_RADIO:
    handle_cmd_setup_radio(payload, len);
    break;

  case FUNC_ID_TRANSMIT:
    handle_cmd_transmit(payload, len);
    break;

  case FUNC_ID_TRANSMIT_BEAM:
    handle_cmd_transmit_beam(payload, len);
    break;

  case FUNC_ID_ABORT_BEAM:
    handle_cmd_abort_beam();
    break;

  case FUNC_ID_MEASURE_NOISE_FLOOR:
    handle_cmd_measure_noise_floor(payload, len);
    break;
  default:
    break;
  }
}
