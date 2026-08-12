#!/bin/bash
# Required env variables:
# SDK: Path to SDK root
#
# Optional env variables:
# SLC: Path to SLC-CLI binary (default: slc)
# COMMANDER: Path to Simplicity Commander binary (default: commander)
# TOOLCHAIN: Path to the ARM toolchain root (default: found under /opt)

set -euo pipefail

if [ -z "${SDK:-}" ]; then
	echo "ERROR: env variable SDK must be set to SDK root"
	exit 1
fi

SLC="${SLC:-slc}"
TOOLCHAIN="${TOOLCHAIN:-}"
PROJ_NAME=zwave_rcp

rm -rf build/

"$SLC" signature trust --sdk "$SDK"

# Find the toolchain unless one was provided. -print -quit stops at the first
# match without a pipe, so it can't trip pipefail.
if [ -z "$TOOLCHAIN" ]; then
	TOOLCHAIN="$(find /opt -type d -name '*arm-none-eabi*' -print -quit)"
fi
echo "Using toolchain: $TOOLCHAIN"

# --copy-sources writes a self-contained project into build/, so the generated
# makefile references the SDK and sources with paths relative to build/.
"$SLC" generate \
	--project-file "$PROJ_NAME.slcp" \
	--export-destination build/ \
	--sdk "$SDK" \
	--copy-sources \
	--toolchain toolchain_gcc \
	--output-type makefile

# slc-cli does not run the pin tool, so supply the committed pin_config.h that
# app_init.c includes.
cp config/pin_config.h build/config/pin_config.h

# Build from inside build/ so COPIED_SDK_PATH resolves, and pin OUTPUT_DIR to
# build/release so mkgbl.sh and the workflow find the binaries there.
make -C build -B -f "$PROJ_NAME.Makefile" release \
	OUTPUT_DIR=release \
	ARM_GCC_DIR="$TOOLCHAIN"

tools/mkgbl.sh
