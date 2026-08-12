#!/bin/bash
# Optional env variables:
# COMMANDER: Path to Simplicity Commander binary (default: commander)

set -euo pipefail

# Default COMMANDER so the script also works when run on its own.
COMMANDER="${COMMANDER:-commander}"

BUILD_OUTPUT=build/release/zwave_rcp.hex
OUTFILE=artifact/zwave_rcp.gbl
SIGN_KEY=keys/vendor_sign.key
ENC_KEY=keys/vendor_encrypt.key

mkdir -p artifact

# Sign and encrypt when vendor keys are present, otherwise emit a plain GBL.
if [ -f "$SIGN_KEY" ] && [ -f "$ENC_KEY" ]; then
	"$COMMANDER" gbl create "$OUTFILE" --app "$BUILD_OUTPUT" --sign "$SIGN_KEY" --encrypt "$ENC_KEY" --compress lzma
else
	"$COMMANDER" gbl create "$OUTFILE" --app "$BUILD_OUTPUT" --compress lzma
fi
