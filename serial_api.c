#include <stdint.h>
#include "serial_api.h"
#include "common.h"
#include "app_process.h"

void handle_cmd_get_firmware_info(uint8_t *payload, uint8_t len)
{
	// HOST -> ZW: ()
	// ZW -> HOST: FW_TYPE | VER_MAJOR | VER_MINOR | VER_PATCH | LEN_BITMASK | FUNC_ID_BITMASK
	uint8_t resp[6] = {
		FW_TYPE_RCP,
		FIRMWARE_VERSION_MAJOR,
		FIRMWARE_VERSION_MINOR,
		FIRMWARE_VERSION_PATCH,
		0x01, // LEN_BITMASK
		0 |
			(1 << (FUNC_ID_GET_FIRMWARE_INFO - 1)) |
			(1 << (FUNC_ID_SETUP_RADIO - 1)) |
			(1 << (FUNC_ID_TRANSMIT - 1)) |
			(1 << (FUNC_ID_RECEIVE - 1)) |
			0};

	uart_transmit_frame(FRAME_TYPE_RESP, FUNC_ID_GET_FIRMWARE_INFO, resp, sizeof(resp));
}

// HOST -> ZW: HOME_ID (4 bytes) | ...DATA
// ZW -> HOST: QUEUE_RESULT | [RAIL_STATUS]
// ZW -> HOST: TX_RESULT | [RAIL_STATUS]

void handle_cmd_transmit(uint8_t *payload, uint8_t len)
{
	// HOST -> ZW: Channel | HOME_ID (4 bytes) | ...DATA
	// ZW -> HOST: QUEUE_RESULT | [RAIL_STATUS]

	// For now simply forward the frame to the radio. We'll figure out if
	// we need to do anything else later.
	uint8_t channel = payload[0];
	radio_transmit(channel, &payload[1], len - 1);
}

void respond_cmd_transmit(tx_result_t result)
{
	uint8_t payload[1] = {result};
	uart_transmit_frame(
		FRAME_TYPE_RESP,
		FUNC_ID_TRANSMIT,
		payload,
		sizeof(payload));
}

void callback_cmd_transmit(tx_result_t result)
{

	uint8_t payload[1] = {result};
	uart_transmit_frame(
		FRAME_TYPE_CALLBACK,
		FUNC_ID_TRANSMIT,
		payload,
		sizeof(payload));
}

void notify_receive(uint8_t *data, uint8_t len)
{
	// ZW -> HOST: LEN | ...DATA
	uint8_t payload[len + 1];
	payload[0] = len;
	memcpy(&payload[1], data, len);

	uart_transmit_frame(
		FRAME_TYPE_CALLBACK,
		FUNC_ID_RECEIVE,
		payload,
		sizeof(payload));
}