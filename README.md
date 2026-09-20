# Z-Wave RCP firmware

An alternative approach to Z-Wave controller firmware, where the Z-Wave chip only provides radio functionality (RX and TX).
All protocol handling is done in the host controller.

## This is a work in progress and is not nearly ready for production use!

## Supported hardware

| Radio SoC | Radio library | Build |
| --- | --- | --- |
| Silicon Labs EFR32ZG23 | RAIL (Simplicity SDK 2024.12.1) | `zwave_rcp.slcp`, built by `tools/build.sh` |
| Trident IoT T32CZ20 (e.g. DKNCZ20B20 development kit) | ZPAL (Trident IoT SDK) | `trident/`, built by `tools/build_trident.sh`, see [trident/README.md](trident/README.md) |

The serial protocol and its handlers (`serial_api.c`, `serial_link.c`) are
shared. Each platform implements the radio backend declared in `radio.h` and
the UART transmit functions declared in `serial_link.h`.

## Serial Protocol Description

TODO