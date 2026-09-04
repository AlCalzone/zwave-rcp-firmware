# Application target, built with the zw_create_app() function of the SDK

if(NOT DEFINED APP_NAME)
  set(APP_NAME "zwave_rcp")
endif()

set(RCP_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")

set(RCP_INCLUDES
  ${RCP_ROOT_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${ZW_SDK_ROOT}/platform/TridentIoT/inc
  ${ZW_SDK_ROOT}/z-wave-stack/Components/Assert
)

set(RCP_SOURCES
  # Shared serial API protocol layer
  ${RCP_ROOT_DIR}/serial_api.c
  ${RCP_ROOT_DIR}/serial_link.c
  # Trident IoT platform
  ${CMAKE_CURRENT_SOURCE_DIR}/rcp_app.c
  ${CMAKE_CURRENT_SOURCE_DIR}/radio_zpal.c
  ${CMAKE_CURRENT_SOURCE_DIR}/uart_zpal.c
)

set(RCP_DEFINITIONS
  RCP_TRIDENT_SDK_VERSION_YEAR=${TSDK_VERSION_YEAR}
  RCP_TRIDENT_SDK_VERSION_MONTH=${TSDK_VERSION_MONTH}
  RCP_TRIDENT_SDK_VERSION_PATCH=${TSDK_VERSION_PATCH}
)

if(DEFINED RCP_UART_BAUD_RATE)
  list(APPEND RCP_DEFINITIONS RCP_UART_BAUD_RATE=${RCP_UART_BAUD_RATE})
endif()
if(DEFINED RCP_DEFAULT_REGION)
  list(APPEND RCP_DEFINITIONS RCP_DEFAULT_REGION=${RCP_DEFAULT_REGION})
endif()
if(DEFINED RCP_TX_MAX_POWER_DECI_DBM)
  list(APPEND RCP_DEFINITIONS RCP_TX_MAX_POWER_DECI_DBM=${RCP_TX_MAX_POWER_DECI_DBM})
endif()

# Hold the platform sources to the same warning level as the board libraries.
# The shared protocol layer is compiled with the SDK defaults, like on EFR32.
set_source_files_properties(
  ${CMAKE_CURRENT_SOURCE_DIR}/rcp_app.c
  ${CMAKE_CURRENT_SOURCE_DIR}/radio_zpal.c
  ${CMAKE_CURRENT_SOURCE_DIR}/uart_zpal.c
  PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wconversion;-Werror"
)

if(CMAKE_BUILD_TYPE MATCHES Release OR CMAKE_BUILD_TYPE MATCHES Debug)
  zw_create_app(
    NAME "${APP_NAME}"
    SOURCES "${RCP_SOURCES}"
    INCLUDES "${RCP_INCLUDES}"
    LIBS ""
    DEFINITIONS "${RCP_DEFINITIONS}"
    # The RCP only uses the radio. No Z-Wave protocol stack, no region variants.
    NO_STACK
  )
endif()
