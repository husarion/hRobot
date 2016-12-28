cmake_minimum_required (VERSION 2.6)
project(Main NONE)

set(PORT "stm32")
set(BOARD_VERSION "1.0.0")
set(BOARD_TYPE "core2")
include(${HFRAMEWORK_PATH}/hFramework.cmake)

add_component_lib("hCloudClient" "")
add_component_lib("hSensors" "")
add_component_lib("hModules" "")
add_component_lib("modules" "")

add_source_files("soft_enc.cpp" "main.cpp" "ErrorLog.cpp")

add_hexecutable(main)
