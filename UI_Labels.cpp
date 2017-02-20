#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "UI_Buttons.h"
#include "Addons.h"
#include "UI_Labels.h"

// jog, ui, recorded positions values
extern float target[9];
extern float current[9];
extern float pos1[9];
extern float pos2[9];
extern float pos3[9];
extern float pos4[9];
extern float pos5[9];
extern float pos6[9];
extern int mode;
int modeLastLbs=-1;
extern int pos_label; //switch counter for position

// temp PID values for calibration
extern float tempKp;
extern float tempKi;
extern float tempKd;

void printOnLabelsTask()
{
	for (;;) {
		printOnLabels();
		sys.delay(1000);
	}
}

void printOnLabels()
{
	//platform.ui.label("mode").setText("test2");
	//platform.ui.label("info").setText("time: %d", sys.getRefTime());
	//UARTd(111100,pos_label);
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
	    //erco(40);
		platform.ui.label("info").setText("Current J1: %f, J2: %f, J3: %f, J5: %f, J6: %f", current[1], current[2], current[3], current[5], current[6]);
		break;
	default:
	    //erco(30);
		platform.ui.label("info").setText("Target J1: %f, j2: %f, J3: %f, J5: %f, J6: %f", target[1], target[2], target[3], target[5], target[6]);
		break;
	}

	if (mode != modeLastLbs) {
		switch (mode) {
		case 0:
		    platform.ui.label("modeMov").setText("Virtual UI_jog for movement by joints");
		    platform.ui.label("lb_mode1").setText("  J1  ");
		    platform.ui.label("lb_mode2").setText("  J2  ");
		    platform.ui.label("lb_mode3").setText("  J3  ");
		    platform.ui.label("lb_mode5").setText("  J5  ");
		    platform.ui.label("lb_mode6").setText("  J6  ");
			break;
		case 1:
		    platform.ui.label("modeMov").setText("Virtual UI_jog for movement by cartesian-Co");
		    platform.ui.label("lb_mode1").setText("  x  ");
		    platform.ui.label("lb_mode2").setText("  y  ");
		    platform.ui.label("lb_mode3").setText("  z  ");
		    platform.ui.label("lb_mode5").setText("  A  ");
		    platform.ui.label("lb_mode6").setText("  B  ");
			break;
		case 2:
		    platform.ui.label("modeMov").setText("Movement by hCode on UI");
		    platform.ui.label("lb_mode1").setText("  --  ");
		    platform.ui.label("lb_mode2").setText("  --  ");
		    platform.ui.label("lb_mode3").setText("  --  ");
		    platform.ui.label("lb_mode5").setText("  --  ");
		    platform.ui.label("lb_mode6").setText("  --  ");
			break;
		case 3:
		    platform.ui.label("modeMov").setText("Movement by hCode on serial USB");
		    platform.ui.label("lb_mode1").setText("  --  ");
		    platform.ui.label("lb_mode2").setText("  --  ");
		    platform.ui.label("lb_mode3").setText("  --  ");
		    platform.ui.label("lb_mode5").setText("  --  ");
		    platform.ui.label("lb_mode6").setText("  --  ");
			break;

		}
		modeLastLbs = mode;
	}

	//UART(111200);
	//platform.ui.label("PIDinfo").setText("J5: Kp=%f, Ki=%f, Kd=%f", tempKp, tempKi, tempKd);
}
