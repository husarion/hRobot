// hRobot_1_00_2 edited by mzabinski94@gmail.com
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

void hMain()
{
	Serial.init(115200);
	sys.setLogDev(&Serial);
	MotorManagerInit();
	sys.setSysLogDev(&devNull);
	sys.taskCreate(printfErrorTask);
	sys.taskCreate(ComandInputTask, 1, 2000, "ComandInputTask");
	changeInputToUI();
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
