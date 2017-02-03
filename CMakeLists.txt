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

add_source_files("ParseCommand.cpp" "MotionManager.cpp" "ErrorLog.cpp" "main.cpp" "GeoMath.cpp" "Addons.cpp" "UI_Labels.cpp" "UI_Buttons.cpp" "DblMotorCtrl.cpp" "soft_enc.cpp" "ServoCtrl.cpp" "MotorManager.cpp")

add_hexecutable(main)
