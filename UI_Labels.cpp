#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "UI_Buttons.h"

// jog, ui, recorded positions values
extern float target[9];
extern float current[9];
extern float offset[9];
extern float temp[9];
extern float pos1[9];
extern float pos2[9];
extern float pos3[9];
extern float pos4[9];
extern float pos5[9];
extern float pos6[9];
extern bool mode;
extern int pos_label; //switch counter for position

// temp PID values for calibration
extern float tempKp;
extern float tempKi;
extern float tempKd;

void printOnLabels()
{
	platform.ui.label("mode").setText("test2");
	//platform.ui.label("info").setText("time: %d", sys.getRefTime());

	switch (pos_label) {
	case 1:
		platform.ui.label("info").setText("Pos1 J1: %f, J2: %f, J3: %f, J5: %f, J6: %f", pos1[1], pos1[2], pos1[3], pos1[5], pos1[6]);
		break;
	case 2:
		platform.ui.label("info").setText("Pos2 J1: %f, J2: %f, J3: %f, J5: %f, J6: %f", pos2[1], pos2[2], pos2[3], pos2[5], pos2[6]);
		break;
	case 3:
		platform.ui.label("info").setText("Pos3 J1: %f, J2: %f, J3: %f, J5: %f, J6: %f", pos3[1], pos3[2], pos3[3], pos3[5], pos3[6]);
		break;
	case 4:
		platform.ui.label("info").setText("Pos4 J1: %f, J2: %f, J3: %f, J5: %f, J6: %f", pos4[1], pos4[2], pos4[3], pos4[5], pos4[6]);
		break;
	case 5:
		platform.ui.label("info").setText("Pos5 J1: %f, J2: %f, J3: %f, J5: %f, J6: %f", pos5[1], pos5[2], pos5[3], pos5[5], pos5[6]);
		break;
	case 6:
		platform.ui.label("info").setText("Pos6 J1: %f, J2: %f, J3: %f, J5: %f, J6: %f", pos6[1], pos6[2], pos6[3], pos6[5], pos6[6]);
		break;
	case 7:
		platform.ui.label("info").setText("Current J1: %f, J2: %f, J3: %f, J5: %f, J6: %f", current[1], current[2], current[3], current[5], current[6]);
		break;
	default:
		if (!mode) {
			platform.ui.label("info").setText("Target J1: %f, j2: %f, J3: %f, J5: %f, J6: %f", target[1], target[2], target[3], target[5], target[6]);
		} else {
			platform.ui.label("info").setText("Temp J1: %f, J2: %f, J3: %f, J5: %f, J6: %f", temp[1], temp[2], temp[3], temp[5], temp[6]);
		}
		break;
	}


	platform.ui.label("PIDinfo").setText("J5: Kp=%f, Ki=%f, Kd=%f", tempKp, tempKi, tempKd);
}
