cmake_minimum_required(VERSION 2.8)
project(ARoboCoreProject NONE)

include(${HFRAMEWORK_PATH}/hFramework.cmake)
enable_module(hCloudClient)

add_hexecutable(myproject main.cpp Addons.cpp DblMotorCtrl.cpp ErrorLog.cpp GeoMath.cpp GripperCtrl.cpp MotionManager.cpp MotorManager.cpp ParseCommand.cpp ServoCtrl.cpp SoftEnc.cpp UI_Buttons.cpp UI_Labels.cpp Arm.cpp CommandInput.cpp)
