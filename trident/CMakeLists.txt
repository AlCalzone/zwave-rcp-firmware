# Z-Wave RCP firmware for Trident IoT T32CZ20 based boards (e.g. DKNCZ20B20)
#
# This project embeds the Trident IoT SDK, which tools/fetch_trident_sdk.sh
# downloads into trident/tridentiot-sdk (or point TRIDENT_SDK_DIR elsewhere).
# Configure and build with the presets from CMakePresets.json, for example:
#
#   cmake --preset T32CZ20.Release
#   cmake --build --preset T32CZ20.Release

cmake_minimum_required(VERSION 3.23.5)

# The firmware version lives in common.h, shared with the EFR32 build
set(RCP_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
file(STRINGS "${RCP_ROOT_DIR}/common.h" _rcp_version_lines REGEX "^#define[ \t]+FIRMWARE_VERSION_(MAJOR|MINOR|PATCH)[ \t]+[0-9]+")
foreach(_line IN LISTS _rcp_version_lines)
  if(_line MATCHES "FIRMWARE_VERSION_MAJOR[ \t]+([0-9]+)")
    set(RCP_VERSION_MAJOR "${CMAKE_MATCH_1}")
  elseif(_line MATCHES "FIRMWARE_VERSION_MINOR[ \t]+([0-9]+)")
    set(RCP_VERSION_MINOR "${CMAKE_MATCH_1}")
  elseif(_line MATCHES "FIRMWARE_VERSION_PATCH[ \t]+([0-9]+)")
    set(RCP_VERSION_PATCH "${CMAKE_MATCH_1}")
  endif()
endforeach()
if(NOT DEFINED RCP_VERSION_MAJOR OR NOT DEFINED RCP_VERSION_MINOR OR NOT DEFINED RCP_VERSION_PATCH)
  message(FATAL_ERROR "Failed to read FIRMWARE_VERSION_* from ${RCP_ROOT_DIR}/common.h")
endif()

project(zwave_rcp
        VERSION ${RCP_VERSION_MAJOR}.${RCP_VERSION_MINOR}.${RCP_VERSION_PATCH}
        LANGUAGES C ASM)

set(TRIDENT_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tridentiot-sdk" CACHE PATH "Root of the unpacked Trident IoT SDK")
if(NOT EXISTS "${TRIDENT_SDK_DIR}/z-wave/CMakeLists.txt")
  message(FATAL_ERROR "Trident IoT SDK not found at ${TRIDENT_SDK_DIR}. Run tools/fetch_trident_sdk.sh or set TRIDENT_SDK_DIR.")
endif()

# Build time configuration of the SDK
include(config.cmake)

# Report the SDK version through FUNC_ID_GET_FIRMWARE_INFO
include("${TRIDENT_SDK_DIR}/tools/cmake/tr_get_sdk_version.cmake")
tr_get_sdk_version("${TRIDENT_SDK_DIR}/tridentiot-sdk.toml")

set(APP_NAME zwave_rcp)
# Output name without the SDK version and git hash, e.g. zwave_rcp_dkncz20_usb_t32cz20.elf
set(ZWAVE_RCP_TARGET_NAME_TEMPLATE "{{name}}_{{hardware}}.elf")

add_subdirectory("${TRIDENT_SDK_DIR}/z-wave" "${CMAKE_BINARY_DIR}/tridentiot-sdk/z-wave")
add_subdirectory(hardware)
add_subdirectory(app)
