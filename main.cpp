// hRobot_1_00_10 edited by mzabinski94@gmail.com
// from hRobot_1_00_09

#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "Arm.h"

float current[9];
// jog, ui, recorded positions values
float target[9];

// temp PID values for calibration
float tempKp = 8.4;
float tempKi = 0.35;
float tempKd = 9;

Arm hRobot;

void hMain()
{
	Serial.init(115200);
	sys.setLogDev(&Serial);
	hRobot.ArmInit();

	sys.delay(10000);
    //hRobot.AddInstructionStream("SET P2 J 0 -20 30 0 0; SET P1 J; SHOWALL; SHOW P1; SHOW P2;\n", SERIAL);

	for (;;) {
		sys.delay(1000);
		LED1.toggle();
	}
}