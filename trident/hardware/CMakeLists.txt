# Board support. One library is generated per src_hw_<board>_<platform>
# directory, and zw_create_app() builds one firmware image per library.

if(NOT DEFINED APP_NAME)
  set(APP_NAME "zwave_rcp")
endif()

tr_zw_generate_hardware_libraries(
  NAME ${APP_NAME}
  INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/../app;${CMAKE_CURRENT_SOURCE_DIR}/../.."
)
