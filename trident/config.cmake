# Build time configuration of the Trident IoT SDK for the Z-Wave RCP firmware.
#
# The full list of options is documented at
# https://tridentiot.github.io/tridentiot-sdk-docs/latest/z-wave/build_system_customization.html

# Link the prebuilt ZPAL library that ships with the SDK release
if(NOT ZWSDK_CONFIG_USE_SOURCES)
  set(ZWSDK_CONFIG_USE_SOURCES "OFF")
endif()

# The RCP configures its region at runtime and does not build the Z-Wave
# protocol stack. A single region keeps the SDK from generating the stack
# configuration for every region.
if(NOT DEFINED ZWSDK_CONFIG_REGION)
  set(ZWSDK_CONFIG_REGION REGION_EU)
endif()

# No command line interface, the UART belongs to the serial API
if(NOT DEFINED ZWSDK_CONFIG_USE_TR_CLI)
  set(ZWSDK_CONFIG_USE_TR_CLI "")
endif()

# Signing and encryption keys for the bootloader and firmware update images.
# When none are configured the SDK generates throwaway keys under
# trident/keys/ on the first build. Point these at real keys for production:
#
# set(ZWSDK_CONFIG_PRIVATE_ROOT_SIGNING_KEY_PATH "${CMAKE_SOURCE_DIR}/keys/root_private.pem")
# set(ZWSDK_CONFIG_PUBLIC_ROOT_SIGNING_KEY_PATH  "${CMAKE_SOURCE_DIR}/keys/root_public.der")
# set(ZWSDK_CONFIG_PRIVATE_SIGNING_KEY_PATH      "${CMAKE_SOURCE_DIR}/keys/private.pem")
# set(ZWSDK_CONFIG_PUBLIC_SIGNING_KEY_PATH       "${CMAKE_SOURCE_DIR}/keys/public.der")
# set(ZWSDK_CONFIG_ENCRYPTION_KEY_PATH           "${CMAKE_SOURCE_DIR}/keys/encryption.hex")
