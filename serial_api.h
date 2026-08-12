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
	FUNC_ID_TRANSMIT_BEAM = 0x05,
	FUNC_ID_ABORT_BEAM = 0x06,
	FUNC_ID_MEASURE_NOISE_FLOOR = 0x07,
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
	// The TX FIFO is busy, so the frame was not queued
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

	TX_RESULT_COMPLETED = 0xff,
} tx_result_t;

/// TX_POWER sentinel meaning the radio keeps whatever power it is set to.
/// 3276.7 dBm is not a valid power level in the Z-Wave power tables.
#define TX_POWER_UNCHANGED 0x7fff

/// Perform a clear channel assessment before transmitting
#define TRANSMIT_FLAG_CCA 0x01

/// Replacement arguments follow the FLAGS byte of FUNC_ID_TRANSMIT
#define TRANSMIT_FLAG_REPLACEMENTS 0x02

/// Measurements FUNC_ID_TRANSMIT can patch into the frame right before transmitting
typedef enum {
	/// Noise floor on the TX channel, encoded like FUNC_ID_MEASURE_NOISE_FLOOR
	REPLACEMENT_SOURCE_NOISE_FLOOR = 0x00,
} replacement_source_t;

/// Most replacements a single FUNC_ID_TRANSMIT may carry
#define TRANSMIT_MAX_REPLACEMENTS 4

/// Z-Wave Long Range PHY and MAC Layer Specification (2023.07.03), Table 6-23
/// and Table 6-27 both define 127 as "RSSI not available"
#define NOISE_FLOOR_NOT_AVAILABLE 127

/// Z-Wave Long Range PHY and MAC Layer Specification (2023.07.03), Table 6-27:
/// an RSSI field carries an "RSSI value in dBm" between these bounds
#define NOISE_FLOOR_MIN_DBM (-120)
#define NOISE_FLOOR_MAX_DBM 30

/// Longest beam frame content the host may hand to FUNC_ID_TRANSMIT_BEAM.
/// G.9959 §8.1.3.10 beam frames carry 3 bytes, Z-Wave LR beam frames 4.
#define BEAM_DATA_MAX_LEN 8

typedef enum {
	SETUP_RADIO_CMD_SET_REGION = 0x01,
	SETUP_RADIO_CMD_GET_REGION = 0x02,
	SETUP_RADIO_CMD_GET_TX_POWER_RANGE = 0x03,
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

/// @brief Handle a request for the firmware and radio library versions
void handle_cmd_get_firmware_info(uint8_t *payload, uint8_t len);

/// @brief Handle a request to configure the radio
void handle_cmd_setup_radio(RAIL_Handle_t rail_handle, uint8_t *payload, uint8_t len);

/// @brief Handle a transmit request
void handle_cmd_transmit(uint8_t *payload, uint8_t len);
void respond_cmd_transmit(tx_result_t result);
void callback_cmd_transmit(tx_result_t result);

/// @brief Handle a request to transmit a wakeup beam
void handle_cmd_transmit_beam(RAIL_Handle_t rail_handle, uint8_t *payload, uint8_t len);
void respond_cmd_transmit_beam(tx_result_t result);
void callback_cmd_transmit_beam(tx_result_t result);

/// @brief Handle a request to stop an ongoing beam
void handle_cmd_abort_beam(RAIL_Handle_t rail_handle);

/// @brief Handle a request to measure the noise floor on a channel
void handle_cmd_measure_noise_floor(RAIL_Handle_t rail_handle, uint8_t *payload, uint8_t len);

/// @brief Report a received frame to the host
void notify_receive(uint8_t *data, uint8_t len, int8_t rssi, uint8_t lqi, uint8_t channel);

#endif
