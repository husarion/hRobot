// hRobot_0_05_09 edited by mzabinski94@gmail.com
// from hRobot_0_05_06

#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "Addons.h"
#include "GeoMath.h"
#include "ErrorLog.h"
#include "UI_Buttons.h"
#include "UI_Labels.h"
#include "ParseCommand.h"
#include "MotionManager.h"
#include "MotorManager.h"
#include "ParseCommand.h"

float current[9];

// jog, ui, recorded positions values
float target[9];
float pos1[9];
float pos2[9];
float pos3[9];
float pos4[9];
float pos5[9];
float pos6[9];
int mode = 0; // 0:UI_joints 1:UI_cartesian 2:UI_code 3:USB_code
int modeLast;
int pos_label; //switch counter for position label

// temp PID values for calibration
float tempKp = 8.4;
float tempKi = 0.35;
float tempKd = 9;

void MovementUpdateTask()
{
	for (;;) {
		if (mode != modeLast) {
			if (mode == 1) {
				Coordinates point_t = joints2cartes(Coordinates(jointsCo, target[1], target[2], target[3], target[4], target[5]));
				point_t.k1 = target[1];
				point_t.k2 = target[2];
				point_t.k3 = target[3];
				point_t.k4 = target[5];
				point_t.k5 = target[6];
			}
			if (mode == 0) {
				Coordinates point_t = cartes2joints(Coordinates(cartesianCo, target[1], target[2], target[3], target[4], target[5]), Coordinates(jointsCo, current[1], current[2], current[3], current[4], current[5]), 1);
				point_t.k1 = target[1];
				point_t.k2 = target[2];
				point_t.k3 = target[3];
				point_t.k4 = target[5];
				point_t.k5 = target[6];
			}
			modeLast = mode;
		}

		switch (mode) {
		case 0:
			MotorManagerUpdateTargetGlobal();
			sys.delay(100);
			break;
		case 1:
			MotorManagerUpdateTargetDef(cartes2joints(Coordinates(cartesianCo, target[1], target[2], target[3], target[4], target[5]), Coordinates(jointsCo, current[1], current[2], current[3], current[4], current[5]), 1));
			sys.delay(500);
			break;
		case 2:
			break;
		case 3:
			break;
		}
	}
}



void hMain()
{
	Serial.init(115200);
	sys.setLogDev(&Serial);
	MotorManagerInit();
	sys.setSysLogDev(&devNull);
	sys.taskCreate(printfErrorTask);
	sys.taskCreate(ComandInputTask, 1, 2000, "ComandInputTask");
	platform.begin(&RPi);
	platform.ui.configHandler = cfgHandler;
	platform.ui.onButtonEvent = onButtonEvent;
	//platform.ui.onValueChangeEvent = onValueChangeEvent;
	platform.ui.setProjectId("@@@PROJECT_ID@@@");

	sys.taskCreate(MotorManagerUpdateTask, 2, 400, "MotorManUpdate");

	sys.taskCreate(MotionTask);
    
	sys.taskCreate(printOnLabelsTask, 2, 2000, "labelsTask");

	for (;;) {
		//printOnLabels();
		sys.delay(1000);
		LED1.toggle();
	}
}


