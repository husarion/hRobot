// hRobot_0_03_9
#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
//#include "UI_Buttons.h"
//#include "UI_Labels.h"
#include "Addons.h"
#include "MotorManager.h"
#include "ServoCtrl.h"

// jog, ui, recorded positions values
float target[9];
float current[9];
float offset[9];
float temp[9];
float pos1[9];
float pos2[9];
float pos3[9];
float pos4[9];
float pos5[9];
float pos6[9];
bool mode = false;
int pos_label; //switch counter for position

// temp PID values for calibration
float tempKp = 1;
float tempKi = 0;
float tempKd = 0;

void hMain()
{
    Serial.init(9600);
    
	//platform.begin(&RPi);
	//platform.ui.configHandler = cfgHandler;
	//platform.ui.onButtonEvent = onButtonEvent;
	//platform.ui.setProjectId("@@@PROJECT_ID@@@");
    
	MotorManagerInit();
    
	sys.taskCreate(printfOnConsoleInWebIDE);
    
	sys.taskCreate(MotorManagerJ1UpdateTask);
	sys.taskCreate(MotorManagerJ2UpdateTask);
	sys.taskCreate(MotorManagerJ3UpdateTask);
	sys.taskCreate(MotorManagerJ5UpdateTask);
	sys.taskCreate(MotorManagerJ6UpdateTask);
	sys.taskCreate(MotorManagerUpdateTargetGlobalTask);
	//sys.taskCreate(motor_task);
	
	for (;;) {
        
		//printOnLabels();
        
		sys.delay(500);
		LED1.toggle();
	}
}
