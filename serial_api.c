#include <stdint.h>
#include "serial_api.h"
#include "common.h"
#include "app_process.h"
#include "rail.h"
#include "rail_zwave.h"

void handle_cmd_get_firmware_info(uint8_t *payload, uint8_t len)
{
	// HOST -> ZW: ()
	// ZW -> HOST: VER_MAJOR | VER_MINOR | VER_PATCH | LIB_TYPE | LIB_MAJOR | LIB_MINOR | LIB_PATCH | LEN_BITMASK | FUNC_ID_BITMASK

	RAIL_Version_t rail_version = {0};
	RAIL_GetVersion(&rail_version, false);

	uint8_t resp[9] = {
		FIRMWARE_VERSION_MAJOR,
		FIRMWARE_VERSION_MINOR,
		FIRMWARE_VERSION_PATCH,
		RADIO_LIBRARY_RAIL,
		rail_version.major,
		rail_version.minor,
		rail_version.rev,
		0x01, // LEN_BITMASK
		0 |
			(1 << (FUNC_ID_GET_FIRMWARE_INFO - 1)) |
			(1 << (FUNC_ID_SETUP_RADIO - 1)) |
			(1 << (FUNC_ID_TRANSMIT - 1)) |
			(1 << (FUNC_ID_RECEIVE - 1)) |
			0};

	uart_transmit_frame(FRAME_TYPE_RESP, FUNC_ID_GET_FIRMWARE_INFO, resp, sizeof(resp));
}

void handle_cmd_setup_radio(RAIL_Handle_t rail_handle, uint8_t *payload, uint8_t len)
{
	setup_radio_cmd_t subcmd = payload[0];
	switch (subcmd)
	{
	case SETUP_RADIO_CMD_SET_REGION:
	{
		// HOST -> ZW: SET_REGION | REGION | [CHANNEL_CFG]
		// ZW -> HOST: SET_REGION | RESULT | [NUM_CHANNELS | CH_1_FREQ (32 bit) | CH_1_BAUD | ... | CH_N_FREQ (32 bit) | CH_N_BAUD]
		zwave_region_t region = payload[1];
		zwave_channel_cfg_t channel_cfg = CHANNEL_CFG_CLASSIC;
		if (len >= 3)
		{
			channel_cfg = payload[2];
		}

		channel_info_t channels[RAIL_NUM_ZWAVE_CHANNELS] = {0};
		uint8_t num_channels = 0;
		bool result = radio_set_region(rail_handle, region, channel_cfg, &num_channels, channels);
		if (result)
		{
			uint8_t resp[3 + num_channels * 5];
			resp[0] = SETUP_RADIO_CMD_SET_REGION;
			resp[1] = 1;
			resp[2] = num_channels;
			int i = 3;
			for (int ch = 0; ch < num_channels; ch++)
			{
				resp[i++] = (channels[ch].freq >> 24) & 0xff;
				resp[i++] = (channels[ch].freq >> 16) & 0xff;
				resp[i++] = (channels[ch].freq >> 8) & 0xff;
				resp[i++] = channels[ch].freq & 0xff;
				resp[i++] = channels[ch].baud;
			}
			uart_transmit_frame(FRAME_TYPE_RESP, FUNC_ID_SETUP_RADIO, resp, sizeof(resp));
		}
		else
		{
			uint8_t resp[2] = {SETUP_RADIO_CMD_SET_REGION, 0};
			uart_transmit_frame(FRAME_TYPE_RESP, FUNC_ID_SETUP_RADIO, resp, sizeof(resp));
		}
		break;
	}
	case SETUP_RADIO_CMD_GET_REGION:
	{
		// HOST -> ZW: GET_REGION
		// ZW -> HOST: GET_REGION | REGION | CHANNEL_CFG | NUM_CHANNELS | CH_1_FREQ (32 bit) | CH_1_BAUD | ... | CH_N_FREQ (32 bit) | CH_N_BAUD
		zwave_region_t region = REGION_UNKNOWN;
		zwave_channel_cfg_t channel_cfg = CHANNEL_CFG_CLASSIC;
		channel_info_t channels[RAIL_NUM_ZWAVE_CHANNELS] = {0};
		uint8_t num_channels = 0;

		radio_get_region(rail_handle, &region, &channel_cfg, &num_channels, channels);

		uint8_t resp[4 + num_channels * 5];
		resp[0] = SETUP_RADIO_CMD_SET_REGION;
		resp[1] = region;
		resp[2] = channel_cfg;
		resp[3] = num_channels;
		int i = 4;
		for (int ch = 0; ch < num_channels; ch++)
		{
			resp[i++] = (channels[ch].freq >> 24) & 0xff;
			resp[i++] = (channels[ch].freq >> 16) & 0xff;
			resp[i++] = (channels[ch].freq >> 8) & 0xff;
			resp[i++] = channels[ch].freq & 0xff;
			resp[i++] = channels[ch].baud;
		}
		uart_transmit_frame(FRAME_TYPE_RESP, FUNC_ID_SETUP_RADIO, resp, sizeof(resp));

		break;
	}
	}
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

void notify_receive(uint8_t *data, uint8_t len, int8_t rssi, uint8_t lqi, uint8_t channel)
{
	// ZW -> HOST: LEN | ...DATA | RSSI | LQI | channel
	uint8_t payload[len + 4];
	payload[0] = len;
	memcpy(&payload[1], data, len);
	int offset = len + 1;
	payload[offset++] = rssi;
	payload[offset++] = lqi;
	payload[offset++] = channel;

	uart_transmit_frame(
		FRAME_TYPE_CALLBACK,
		FUNC_ID_RECEIVE,
		payload,
		sizeof(payload));
}
