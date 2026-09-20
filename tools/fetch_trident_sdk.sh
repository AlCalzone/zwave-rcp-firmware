#!/bin/bash
# Downloads the Trident IoT SDK release the Trident build is developed against
# and unpacks it into trident/tridentiot-sdk.
#
# Optional env variables:
# TRIDENT_SDK_DIR: Where to unpack the SDK (default: trident/tridentiot-sdk)
#
# The SDK is licensed under the Trident IoT Master Software License Agreement,
# see LICENSES/LicenseRef-TridentMSLA.txt inside the archive. It is not part of
# this repository.

set -euo pipefail

SDK_VERSION="v2026.06.00-ga"
SDK_SHA256="6d9a1673a7340ebdfc31545b835a151920ea5c545ea623780fece67d91f02f10"
SDK_URL="https://github.com/tridentiot/tridentiot-sdk-releases/releases/download/${SDK_VERSION}/tridentiot-sdk-${SDK_VERSION}.tar.gz"

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TRIDENT_SDK_DIR="${TRIDENT_SDK_DIR:-$REPO_ROOT/trident/tridentiot-sdk}"

if [ -f "$TRIDENT_SDK_DIR/tridentiot-sdk.toml" ]; then
	echo "Trident IoT SDK already present at $TRIDENT_SDK_DIR"
	exit 0
fi

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

echo "Downloading Trident IoT SDK ${SDK_VERSION}..."
curl -sSfL -o "$TMP_DIR/sdk.tar.gz" "$SDK_URL"

echo "${SDK_SHA256}  $TMP_DIR/sdk.tar.gz" | sha256sum -c -

mkdir -p "$TRIDENT_SDK_DIR"
# The archive wraps everything in a tridentiot-sdk-<version>/ directory
tar -xzf "$TMP_DIR/sdk.tar.gz" -C "$TRIDENT_SDK_DIR" --strip-components=1

echo "Trident IoT SDK unpacked to $TRIDENT_SDK_DIR"
