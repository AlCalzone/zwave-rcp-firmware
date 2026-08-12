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
			(1 << (FUNC_ID_TRANSMIT_BEAM - 1)) |
			(1 << (FUNC_ID_ABORT_BEAM - 1)) |
			(1 << (FUNC_ID_MEASURE_NOISE_FLOOR - 1)) |
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
			resp[0] = subcmd;
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
			uint8_t resp[2] = {subcmd, 0};
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
		resp[0] = subcmd;
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
	case SETUP_RADIO_CMD_GET_TX_POWER_RANGE:
	{
		// HOST -> ZW: GET_TX_POWER_RANGE
		// ZW -> HOST: GET_TX_POWER_RANGE | MIN_POWER (int16 BE, deci-dBm) | MAX_POWER (int16 BE, deci-dBm)
		RAIL_TxPowerConfig_t pa_config = {0};
		RAIL_GetTxPowerConfig(rail_handle, &pa_config);

		RAIL_TxPowerMode_t mode = pa_config.mode;
		RAIL_TxPowerLevel_t min_level = 0;
		RAIL_TxPowerLevel_t max_level = 0;
		int16_t min_power = 0;
		int16_t max_power = 0;

		// RAIL fills in the levels only when it supports the mode. An
		// unsupported mode reports an empty 0 dBm range
		if (RAIL_SupportsTxPowerModeAlt(rail_handle, &mode, &max_level, &min_level))
		{
			min_power = RAIL_ConvertRawToDbm(rail_handle, mode, min_level);
			max_power = RAIL_ConvertRawToDbm(rail_handle, mode, max_level);
		}

		uint8_t resp[5] = {
			subcmd,
			(min_power >> 8) & 0xff,
			min_power & 0xff,
			(max_power >> 8) & 0xff,
			max_power & 0xff,
		};
		uart_transmit_frame(FRAME_TYPE_RESP, FUNC_ID_SETUP_RADIO, resp, sizeof(resp));

		break;
	}
	case SETUP_RADIO_CMD_GET_CAPABILITIES:
	{
		// HOST -> ZW: GET_CAPABILITIES
		// ZW -> HOST: GET_CAPABILITIES | BITMASK_LEN | ...BITMASK
		//
		// The bitmask carries the radio_capability_t values this firmware
		// implements, so the host can tell optional features apart from the
		// commands FUNC_ID_BITMASK already covers.
		uint8_t capabilities[1] = {
			0 |
				(1 << (RADIO_CAPABILITY_TRANSMIT_REPLACEMENTS - 1)) |
				0};

		uint8_t resp[2 + sizeof(capabilities)];
		resp[0] = subcmd;
		resp[1] = sizeof(capabilities);
		memcpy(&resp[2], capabilities, sizeof(capabilities));
		uart_transmit_frame(FRAME_TYPE_RESP, FUNC_ID_SETUP_RADIO, resp, sizeof(resp));

		break;
	}
	}
}

void handle_cmd_transmit(uint8_t *payload, uint8_t len)
{
	// HOST -> ZW: CHANNEL | TX_POWER (int16 BE, deci-dBm, or TX_POWER_UNCHANGED) | FLAGS | [NUM_REPLACEMENTS | (OFFSET | SOURCE)*] | ...DATA
	// ZW -> HOST: TX_RESULT
	// ZW -> HOST (callback): TX_RESULT
	//
	// Replacement arguments are present exactly when FLAGS carries
	// TRANSMIT_FLAG_REPLACEMENTS. Each one patches DATA[OFFSET] with the
	// measurement SOURCE names, taken right before the transmit. Only bytes the
	// host explicitly lists here are replaced.

	if (len < 5)
	{
		// The header takes 4 bytes, followed by at least one data byte
		respond_cmd_transmit(TX_RESULT_INVALID_PARAM);
		return;
	}

	uint8_t channel = payload[0];
	int16_t power_deci_dbm = (int16_t)(((uint16_t)payload[1] << 8) | payload[2]);
	// Undefined flags are reserved and must be ignored
	uint8_t flags = payload[3];

	const uint8_t *replacements = NULL;
	uint8_t num_replacements = 0;
	uint8_t data_start = 4;

	if (flags & TRANSMIT_FLAG_REPLACEMENTS)
	{
		num_replacements = payload[4];
		data_start = 5 + 2 * num_replacements;
		if (
			num_replacements == 0
			|| num_replacements > TRANSMIT_MAX_REPLACEMENTS
			// At least one data byte must follow the replacement list
			|| len < (uint8_t)(data_start + 1))
		{
			respond_cmd_transmit(TX_RESULT_INVALID_PARAM);
			return;
		}
		replacements = &payload[5];

		uint8_t data_len = len - data_start;
		for (uint8_t i = 0; i < num_replacements; i++)
		{
			uint8_t offset = replacements[2 * i];
			uint8_t source = replacements[2 * i + 1];
			if (offset >= data_len || source != REPLACEMENT_SOURCE_NOISE_FLOOR)
			{
				respond_cmd_transmit(TX_RESULT_INVALID_PARAM);
				return;
			}
		}
	}

	radio_transmit(channel, power_deci_dbm, flags, &payload[data_start], len - data_start, replacements, num_replacements);
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

void handle_cmd_transmit_beam(RAIL_Handle_t rail_handle, uint8_t *payload, uint8_t len)
{
	// HOST -> ZW: TX_POWER (int16 BE, deci-dBm, or TX_POWER_UNCHANGED) | NUM_FRAGMENTS | FRAGMENT_DURATION_MS (u16 BE) | FRAGMENT_PERIOD_MS (u16 BE) | NUM_CHANNELS | ...CHANNELS | ...DATA
	// ZW -> HOST: TX_RESULT
	// ZW -> HOST (callback): TX_RESULT
	//
	// The callback carries TX_RESULT_COMPLETED for a beam that ran to its last
	// fragment, TX_RESULT_ABORTED for one that FUNC_ID_ABORT_BEAM or a region
	// change stopped, and any other radio error the repeat train ran into.
	//
	// A beam leaves the radio at the power it used, so a later transmit passing
	// TX_POWER_UNCHANGED goes out at the beam's power.

	if (len < 10)
	{
		// The header takes 8 bytes, followed by at least one channel and one data byte
		respond_cmd_transmit_beam(TX_RESULT_INVALID_PARAM);
		return;
	}

	int16_t power_deci_dbm = (int16_t)(((uint16_t)payload[0] << 8) | payload[1]);
	uint8_t num_fragments = payload[2];
	uint16_t fragment_duration_ms = ((uint16_t)payload[3] << 8) | payload[4];
	uint16_t fragment_period_ms = ((uint16_t)payload[5] << 8) | payload[6];
	uint8_t num_channels = payload[7];

	if (num_channels == 0 || len < 8 + num_channels + 1)
	{
		respond_cmd_transmit_beam(TX_RESULT_INVALID_PARAM);
		return;
	}

	const uint8_t *channels = &payload[8];
	const uint8_t *data = &payload[8 + num_channels];
	uint8_t data_len = len - 8 - num_channels;

	radio_transmit_beam(
		rail_handle,
		power_deci_dbm,
		num_fragments,
		fragment_duration_ms,
		fragment_period_ms,
		num_channels,
		channels,
		data,
		data_len);
}

void respond_cmd_transmit_beam(tx_result_t result)
{
	uint8_t payload[1] = {result};
	uart_transmit_frame(
		FRAME_TYPE_RESP,
		FUNC_ID_TRANSMIT_BEAM,
		payload,
		sizeof(payload));
}

void callback_cmd_transmit_beam(tx_result_t result)
{
	uint8_t payload[1] = {result};
	uart_transmit_frame(
		FRAME_TYPE_CALLBACK,
		FUNC_ID_TRANSMIT_BEAM,
		payload,
		sizeof(payload));
}

void handle_cmd_abort_beam(RAIL_Handle_t rail_handle)
{
	// HOST -> ZW: ()
	// ZW -> HOST: 1

	// Aborting reports success even when no beam was running, so the host can
	// call this without tracking whether its beam already completed
	uint8_t resp[1] = {1};
	uart_transmit_frame(FRAME_TYPE_RESP, FUNC_ID_ABORT_BEAM, resp, sizeof(resp));

	radio_abort_beam(rail_handle);
}

void handle_cmd_measure_noise_floor(RAIL_Handle_t rail_handle, uint8_t *payload, uint8_t len)
{
	// HOST -> ZW: CHANNEL
	// ZW -> HOST: NOISE_FLOOR (int8, dBm, clamped to -120..30, or 127 when
	// no measurement could be taken: missing/invalid channel, radio busy
	// with a transmit or beam, or the measurement itself failed)

	int8_t noise = NOISE_FLOOR_NOT_AVAILABLE;
	if (len >= 1)
	{
		noise = radio_measure_noise_floor_cmd(rail_handle, payload[0]);
	}

	uint8_t resp[1] = {(uint8_t)noise};
	uart_transmit_frame(FRAME_TYPE_RESP, FUNC_ID_MEASURE_NOISE_FLOOR, resp, sizeof(resp));
}

void notify_receive(uint8_t *data, uint8_t len, int8_t rssi, uint8_t lqi, uint8_t channel)
{
	// ZW -> HOST (callback): LEN | ...DATA | RSSI | LQI | CHANNEL
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
