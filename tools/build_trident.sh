#!/bin/bash
# Builds the Trident IoT (T32CZ20) firmware and collects the binaries in
# artifact/trident/.
#
# Requirements: arm-none-eabi-gcc, cmake >= 3.25, ninja or make, python3 with
# jinja2, yaml, cryptography, pyelftools and ecdsa, openssl, and the Trident
# IoT SDK (tools/fetch_trident_sdk.sh).
#
# Optional env variables:
# PRESET: CMake preset to build (default: T32CZ20.Release)
# TRIDENT_SDK_DIR: SDK location when not at trident/tridentiot-sdk
#
# Pass --container to run the build inside the Trident IoT toolchain image
# instead of using the tools installed on this machine. The image is public,
# so this needs docker but no Trident account.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PRESET="${PRESET:-T32CZ20.Release}"
PROJ_NAME=zwave_rcp

# Same image elcap uses for its builds, tagged with the SDK release
CONTAINER_IMAGE="ghcr.io/tridentiot/tridentiot-sdk-releases:v2026.06.00-ga"

if [ "${1:-}" = "--container" ]; then
	# The image's entrypoint creates a user with USER_ID and runs the command
	# as that user, so the build output belongs to the caller
	exec docker run --rm \
		-e USER_ID="$(id -u)" -e GROUP_ID="$(id -g)" \
		-e PRESET="$PRESET" \
		-v "$REPO_ROOT":/trident -w /trident \
		"$CONTAINER_IMAGE" tools/build_trident.sh
fi

cd "$REPO_ROOT/trident"

CMAKE_ARGS=()
if [ -n "${TRIDENT_SDK_DIR:-}" ]; then
	CMAKE_ARGS+=("-DTRIDENT_SDK_DIR=$TRIDENT_SDK_DIR")
fi

rm -rf "build/$PRESET"
cmake --preset "$PRESET" "${CMAKE_ARGS[@]}"
cmake --build --preset "$PRESET"

# One image is built per board directory under trident/hardware
OUT_DIR="$REPO_ROOT/artifact/trident"
rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"
for elf in "build/$PRESET/app/${PROJ_NAME}_"*.elf; do
	base="${elf%.elf}"
	name="$(basename "$base")"
	cp "$elf" "$OUT_DIR/$name.elf"
	cp "$base.map" "$OUT_DIR/$name.map"
	# Bootloader, application and keys in one image for flashing over SWD
	cp "${base}_signed_combined.hex" "$OUT_DIR/${name}_signed_combined.hex"
	# Firmware update image for the Trident IoT bootloader
	cp "$base.ota" "$OUT_DIR/$name.ota"
done

ls -la "$OUT_DIR"
