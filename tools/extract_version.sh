#!/bin/bash
# Prints the firmware version as major.minor.patch, read from common.h.

set -euo pipefail

CONFIG=common.h

read_define() {
	sed -nE "s/^#define[[:space:]]+$1[[:space:]]+([0-9]+).*/\1/p" "$CONFIG" | head -n 1
}

MAJOR=$(read_define FIRMWARE_VERSION_MAJOR)
MINOR=$(read_define FIRMWARE_VERSION_MINOR)
PATCH=$(read_define FIRMWARE_VERSION_PATCH)

if [ -z "$MAJOR" ] || [ -z "$MINOR" ] || [ -z "$PATCH" ]; then
	echo "ERROR: failed to extract firmware version from $CONFIG" >&2
	exit 1
fi

echo "$MAJOR.$MINOR.$PATCH"
