#ifndef SERIAL_API_H
#define SERIAL_API_H

#include <stdint.h>
#include "rail.h"

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

typedef enum
{
  RADIO_LIBRARY_RAIL,
} radio_library_t;

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

typedef enum {
	SETUP_RADIO_CMD_SET_REGION = 0x01,
	SETUP_RADIO_CMD_GET_REGION = 0x02,
} setup_radio_cmd_t;

typedef enum {
	REGION_EU = 0,
	REGION_US,
	REGION_ANZ,
	REGION_HK,
	REGION_IN = 5,
	REGION_IL,
	REGION_RU,
	REGION_CN,
	REGION_US_LR,
	REGION_EU_LR = 11,
	REGION_JP = 32,
	REGION_KR,
	REGION_UNKNOWN = 0xfe,
} zwave_region_t;

typedef enum {
	CHANNEL_CFG_CLASSIC = 0,
	CHANNEL_CFG_CLASSIC_LR_A = 1,
	CHANNEL_CFG_CLASSIC_LR_B = 2,
	CHANNEL_CFG_LR = 3,
} zwave_channel_cfg_t;

typedef enum {
	ZWAVE_BAUD_9k6 = 0,
	ZWAVE_BAUD_40k = 1,
	ZWAVE_BAUD_100k = 2,
	ZWAVE_BAUD_LR100k = 3,
} zwave_baudrate_t;

typedef struct {
	uint32_t freq;
	zwave_baudrate_t baud;
} channel_info_t;

void handle_cmd_get_firmware_info(uint8_t *payload, uint8_t len);

void handle_cmd_setup_radio(RAIL_Handle_t rail_handle, uint8_t *payload, uint8_t len);

void handle_cmd_transmit(uint8_t *payload, uint8_t len);
void respond_cmd_transmit(tx_result_t result);
void callback_cmd_transmit(tx_result_t result);

void notify_receive(uint8_t *data, uint8_t len, int8_t rssi, uint8_t lqi, uint8_t channel);

#endif
