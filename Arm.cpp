#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include <iostream>
#include <cstdio>
#include <vector>

#include "Arm.h"
#include "Addons.h"
#include "MotionManager.h"
#include "ErrorLog.h"

#include "GeoMath.h"
#include "UI_Buttons.h"
#include "UI_Labels.h"
#include "ParseCommand.h"
#include "MotorManager.h"

Arm::Arm(){}

bool Arm::PassInstruction(instruction_code instruction){
	return MotionManager::get().Istruction(instruction);
}

void Arm::ArmInit(){
	MotorManagerInit();
	sys.setSysLogDev(&devNull);
	sys.taskCreate(printfErrorTask);
	sys.taskCreate(ComandInputTaskSerial, 1, 600, "ComITS");
	platform.begin(&RPi);
	platform.ui.configHandler = cfgHandler;
	platform.ui.onButtonEvent = onButtonEvent;
	platform.ui.onValueChangeEvent = onValueChangeEvent;
	platform.ui.setProjectId("@@@PROJECT_ID@@@");
	sys.taskCreate(MotorManagerUpdateTask, 2, 600, "MorManU");
	sys.taskCreate(MotionTask, 2, 1000, "MotManT");
	sys.taskCreate(printOnLabelsTask, 2, 1500, "labelsT");
}
