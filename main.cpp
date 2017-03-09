// hRobot_1_00_10 edited by mzabinski94@gmail.com
// from hRobot_1_00_09

#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "Arm.h"
#include "CommandInput.h"

float current[9];
// jog, ui, recorded positions values
float target[9];

// temp PID values for calibration
float tempKp = 8.4;
float tempKi = 0.35;
float tempKd = 9;

Arm hRobot;
CommandInput InputData(&hRobot);

char taskList[1000];

void hMain()
{
	Serial.init(115200);
	sys.setLogDev(&Serial);
	hRobot.ArmInit();

	sys.delay(3000);
    InputData.AddInstructionStream("SET P0 J; SHOWALL; PRECYSION OFF 0 3000; MOVE P0; MOVE P0 D 100; MOVE P0 D -100; MOVE P0;\n", SERIAL);

	for (;;) {
		sys.delay(1000);
		LED1.toggle();
		//sys.getTaskList(taskList);
        //Serial.printf("\r\nName\tState\tPriority\tStack\tNum\r\n******************************\r\n%s", taskList);
	}
}