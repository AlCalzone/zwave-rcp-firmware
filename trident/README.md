# Z-Wave RCP firmware for Trident IoT T32CZ20

This directory builds the RCP firmware for Trident IoT's T32CZ20 Z-Wave SoC,
using the Z-Wave Platform Abstraction Layer (ZPAL) radio driver of the
[Trident IoT SDK](https://github.com/tridentiot/tridentiot-sdk-releases).

The serial protocol, the protocol handlers (`serial_api.c`) and the UART
framing (`serial_link.c`) are shared with the Silicon Labs EFR32 build. Only
the platform layer differs:

| File | Purpose |
| --- | --- |
| `app/rcp_app.c` | FreeRTOS task, startup and the hooks the ZPAL library expects |
| `app/radio_zpal.c` | `radio.h` implemented on the ZPAL radio API |
| `app/uart_zpal.c` | `uart_transmit()` and RX pump on the ZPAL UART driver |
| `hardware/src_hw_<board>_T32CZ20/` | Board specific UART pins, one directory per board |

## Supported boards

| Board | Directory | Host UART |
| --- | --- | --- |
| DKNCZ20B20 development kit (`DKNCZ20`) | `hardware/src_hw_DKNCZ20_usb_T32CZ20` | UART0 (GPIO 16/17) via the on-board USB bridge |

Add a directory `hardware/src_hw_<name>_T32CZ20/` with a `zpal_uart_config_t
RCP_UART_CONFIG` to support another board. The build produces one image per
directory.

## Prerequisites

- `arm-none-eabi-gcc` (the SDK is built with 13.2.rel1), CMake >= 3.25 and
  make or ninja
- Python 3 with `jinja2`, `pyyaml`, `cryptography`, `pyelftools` and `ecdsa`
- `openssl` (generates the signing keys)
- The Trident IoT SDK, downloaded by `tools/fetch_trident_sdk.sh` into
  `trident/tridentiot-sdk/`. Using the SDK means accepting the Trident IoT
  Master Software License Agreement shipped in its `LICENSES/` directory.

The SDK's own toolchain container documents the exact versions:
`tridentiot-sdk/tools/docker/Dockerfile`.

## Building

```sh
tools/fetch_trident_sdk.sh
tools/build_trident.sh
```

Without a local ARM toolchain and Python setup, build inside the toolchain
image Trident IoT publishes for each SDK release. It is a public image, so
only docker is needed:

```sh
tools/fetch_trident_sdk.sh
tools/build_trident.sh --container
```

`tools/build_trident.sh` configures and builds the `T32CZ20.Release` preset and
copies the results to `artifact/trident/`:

| File | Contents |
| --- | --- |
| `zwave_rcp_<board>_signed_combined.hex` | Bootloader, application and keys. Flash this over SWD, for example with `elcap flash` or a J-Link |
| `zwave_rcp_<board>.ota` | Firmware update image for the Trident IoT bootloader |
| `zwave_rcp_<board>.elf`, `.map` | Debug symbols and link map |

Building by hand from `trident/`:

```sh
cmake --preset T32CZ20.Release        # or T32CZ20.Debug
cmake --build --preset T32CZ20.Release
```

The SDK signs the bootloader and the application. Without configured keys it
generates throwaway keys under `trident/keys/` on the first configure. Set the
`ZWSDK_CONFIG_*_KEY_PATH` variables in `config.cmake` to use your own.

### Build options

Pass these as `-D<name>=<value>` when configuring:

| Option | Default | Effect |
| --- | --- | --- |
| `RCP_UART_BAUD_RATE` | `500000` | Host UART baud rate. Must be supported by the Trident SDK's ZPAL driver (115200, 230400, 500000, 1000000, ...). 500000 is closest to the EFR32 build's 460800 baud. |
| `RCP_DEFAULT_REGION` | `ZWAVE_REGION_EU` | Region the radio starts in before the host configures one |
| `RCP_TX_MAX_POWER_DECI_DBM` | `140` | Power class of the module, `140` or `200`. Selects the driver's 14 dBm or 20 dBm power tables, like the SDK sample applications. |
| `TRIDENT_SDK_DIR` | `trident/tridentiot-sdk` | Location of the unpacked SDK |

## Using elcap

[elcap](https://tridentiot.github.io/elcap-cli/) is Trident IoT's project
tool. `trident.toml` describes this directory as an elcap project, so the
device commands work from `trident/` after `elcap login`:

```sh
cd trident
elcap device discover
elcap flash                  # flashes the default hex from the last build
elcap tokens read
```

`elcap build` does not work for this project. It mounts only the project
directory into its build container, and the shared protocol sources live in
the repository root. Use `tools/build_trident.sh`, which runs the same CMake
presets, or `tools/build_trident.sh --container` for the same toolchain image
elcap uses. The GitHub Actions build needs neither elcap nor a Trident account.

## Differences to the EFR32 build

- `FUNC_ID_GET_FIRMWARE_INFO` reports `LIB_TYPE` 1 (ZPAL) and the Trident IoT
  SDK version as `YY.MM.PATCH`.
- The host UART runs at 500000 baud by default, see above.
- Received frames carry an LQI of 0, the ZPAL radio reports none.
- The ZPAL driver delivers received frames including their checksum. The
  backend strips it so the host sees the same frame content as from EFR32.
- Clear channel assessment (`TRANSMIT_FLAG_CCA`) uses the LBT threshold and
  listening time the SDK defines per region, not the fixed -80 dBm the EFR32
  build uses.
- A running beam repeat train cannot be cut short. Fragments are transmitted
  in trains of at most 250 ms, so `FUNC_ID_ABORT_BEAM` and a region change take
  effect within that time. Between trains the radio is back in receive mode for
  a few hundred microseconds.
- Between beam fragments the radio scans all channels of the region instead of
  parking on the fragment's channel.

## Status

The port compiles and links against Trident IoT SDK v2026.06.00-ga. It has not
been run on hardware yet. Things worth verifying first on a DKNCZ20B20:

- Frames are received and forwarded, and the checksum stripping leaves exactly
  the MPDU.
- Transmit power and the noise floor readings are in the expected range.
- Beam transmissions wake a FLiRS device.
