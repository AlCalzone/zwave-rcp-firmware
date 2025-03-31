#ifndef SERIAL_API_H
#define SERIAL_API_H

#include <stdint.h>

#define SOF 0x01
#define ACK 0x06
#define NAK 0x15

/// FRAME STRUCTURE
/// SOF | LEN | TYPE | FUNC_ID | DATA... | CHKSUM

typedef enum
{
	FUNC_ID_GET_FIRMWARE_INFO = 0x01,
	FUNC_ID_SETUP_RADIO = 0x02,
	FUNC_ID_TRANSMIT = 0x03,
	FUNC_ID_RECEIVE = 0x04,
} func_id_t;

typedef enum
{
	FRAME_TYPE_REQ = 0x00,
	FRAME_TYPE_RESP = 0x01,
	FRAME_TYPE_CALLBACK = 0x02,
} frame_type_t;

void handle_cmd_get_firmware_info(uint8_t *payload, uint8_t len);

typedef enum
{
	FW_TYPE_RCP = 0x01,
} firmware_type_t;

typedef enum
{
	// The frame was successfully queued for transmission
	TX_RESULT_QUEUED = 0x00,
	// The TX FIFO is busy, cannot queue the frame
	TX_RESULT_BUSY = 0x01,
	// The frame is too long to be transmitted
	TX_RESULT_OVERFLOW = 0x02,
	// Invalid TX channel selected
	TX_RESULT_INVALID_CHANNEL = 0x03,
	// Other invalid parameters were passed
	TX_RESULT_INVALID_PARAM = 0x04,

	// Underlying radio errors
	TX_RESULT_ABORTED = 0xf0,
	TX_RESULT_BLOCKED = 0xf1,
	TX_RESULT_UNDERFLOW = 0xf2,
	TX_RESULT_CHANNEL_BUSY = 0xf3,
	TX_RESULT_UNKNOWN_ERROR = 0xfe,

	// Transmission completed
	TX_RESULT_COMPLETED = 0xff,
} tx_result_t;

void handle_cmd_transmit(uint8_t *payload, uint8_t len);
void respond_cmd_transmit(tx_result_t result);
void callback_cmd_transmit(tx_result_t result);

void notify_receive(uint8_t *data, uint8_t len);

#endif
