// hRobot_0_05_09 edited by mzabinski94@gmail.com
// from hRobot_0_05_07

#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "UI_Buttons.h"
#include "Addons.h"
#include "ErrorLog.h"
#include "Arm.h"
#include "CommandInput.h"
#include "Program.h"

extern Arm hRobot;
extern CommandInput InputData;

// jog, ui, recorded positions values
extern float target[9];
extern float temp[9];
float pos1[9];
float pos2[9];
float pos3[9];
float pos4[9];
float pos5[9];
float pos6[9];
int mode;
int last_mode = 3;
int modeLastBtn = -1;
int pos_label_mode; //switch counter for position

// jog UI steps
float step1;
float step2;
float step3;
float step4;

// temp PID values for calibration
//extern float tempKp;
//extern float tempKi;
//extern float tempKd;

char UIcon[512];
char UIcon_pass[512];

void passUIcom()
{
    for (int i = 0; i < 512; i++)
    {
	UIcon_pass[i] = UIcon[i];
    }
	InputData.AddInstruction(UIcon_pass, UI);
}

void cfgHandler()
{
    //UART(131000);
    // strona startowa
    platform.ui.loadHtml({Resource::WEBIDE, "/ui.html"});

    //UART(132000);
    // create labels
    auto info = platform.ui.label("info");
    auto modeMov = platform.ui.label("modeMov");
    auto lb_mode1 = platform.ui.label("lb_mode1");
    auto lb_mode2 = platform.ui.label("lb_mode2");
    auto lb_mode3 = platform.ui.label("lb_mode3");
    auto lb_mode5 = platform.ui.label("lb_mode5");
    auto lb_mode6 = platform.ui.label("lb_mode6");

    //UART(133000);
    // jog buttons
    auto btn11 = platform.ui.button("btn11");
    auto btn12 = platform.ui.button("btn12");
    auto btn13 = platform.ui.button("btn13");
    auto btn14 = platform.ui.button("btn14");
    auto btn15 = platform.ui.button("btn15");
    auto btn16 = platform.ui.button("btn16");
    auto btn17 = platform.ui.button("btn17");
    auto btn18 = platform.ui.button("btn18");

    auto btn21 = platform.ui.button("btn21");
    auto btn22 = platform.ui.button("btn22");
    auto btn23 = platform.ui.button("btn23");
    auto btn24 = platform.ui.button("btn24");
    auto btn25 = platform.ui.button("btn25");
    auto btn26 = platform.ui.button("btn26");
    auto btn27 = platform.ui.button("btn27");
    auto btn28 = platform.ui.button("btn28");

    auto btn31 = platform.ui.button("btn31");
    auto btn32 = platform.ui.button("btn32");
    auto btn33 = platform.ui.button("btn33");
    auto btn34 = platform.ui.button("btn34");
    auto btn35 = platform.ui.button("btn35");
    auto btn36 = platform.ui.button("btn36");
    auto btn37 = platform.ui.button("btn37");
    auto btn38 = platform.ui.button("btn38");

    auto btn51 = platform.ui.button("btn51");
    auto btn52 = platform.ui.button("btn52");
    auto btn53 = platform.ui.button("btn53");
    auto btn54 = platform.ui.button("btn54");
    auto btn55 = platform.ui.button("btn55");
    auto btn56 = platform.ui.button("btn56");
    auto btn57 = platform.ui.button("btn57");
    auto btn58 = platform.ui.button("btn58");

    // grabber buttons
    auto btn_close = platform.ui.button("btn_close");
    auto btn_open = platform.ui.button("btn_open");
    auto btn_stop = platform.ui.button("btn_stop");

    // write positions buttons
    auto btn_pos1_write = platform.ui.button("btn_pos1_write");
    auto btn_pos2_write = platform.ui.button("btn_pos2_write");
    auto btn_pos3_write = platform.ui.button("btn_pos3_write");
    auto btn_pos4_write = platform.ui.button("btn_pos4_write");
    auto btn_pos5_write = platform.ui.button("btn_pos5_write");
    auto btn_pos6_write = platform.ui.button("btn_pos6_write");

    //read positions buttons
    auto btn_pos1_read = platform.ui.button("btn_pos1_read");
    auto btn_pos2_read = platform.ui.button("btn_pos2_read");
    auto btn_pos3_read = platform.ui.button("btn_pos3_read");
    auto btn_pos4_read = platform.ui.button("btn_pos4_read");
    auto btn_pos5_read = platform.ui.button("btn_pos5_read");
    auto btn_pos6_read = platform.ui.button("btn_pos6_read");

    //show positions buttons
    auto btn_pos1_show = platform.ui.button("btn_pos1_show");
    auto btn_pos2_show = platform.ui.button("btn_pos2_show");
    auto btn_pos3_show = platform.ui.button("btn_pos3_show");
    auto btn_pos4_show = platform.ui.button("btn_pos4_show");
    auto btn_pos5_show = platform.ui.button("btn_pos5_show");
    auto btn_pos6_show = platform.ui.button("btn_pos6_show");
    auto btn_pos0_show = platform.ui.button("btn_pos0_show");
    auto btn_pos7_show = platform.ui.button("btn_pos7_show");

    //PID buttons
    //auto btn_kpp = platform.ui.button("btn_kpp");
    //auto btn_kpm = platform.ui.button("btn_kpm");
    //auto btn_kip = platform.ui.button("btn_kip");
    //auto btn_kim = platform.ui.button("btn_kim");
    //auto btn_kdp = platform.ui.button("btn_kdp");
    //auto btn_kdm = platform.ui.button("btn_kdm");

    //UI command line
    auto btn_com = platform.ui.button("btn_do");

    //Movement changing buttons
    auto btnCartesianMov = platform.ui.button("btnCartesianMov");
    auto btnJointsMov = platform.ui.button("btnJointsMov");
    auto btnCodeUSBMov = platform.ui.button("btnCodeUSBMov");
    auto btnCodeUIMov = platform.ui.button("btnCodeUIMov");

	//User program buttons
	auto prog1 = platform.ui.button("doP1");
	auto prog2 = platform.ui.button("doP2");
	auto prog3 = platform.ui.button("doP3");
	auto prog4 = platform.ui.button("doP4");
	auto prog5 = platform.ui.button("doP5");
	auto prog6 = platform.ui.button("doP6");
	auto prog7 = platform.ui.button("doP7");
	auto prog8 = platform.ui.button("doP8");

	//Home buttons
	auto homeJ1 = platform.ui.button("homeJ1");
	auto homeJ2 = platform.ui.button("homeJ2");
	auto homeJ3 = platform.ui.button("homeJ3");
	auto homeJ5 = platform.ui.button("homeJ5");
	auto homeJ6 = platform.ui.button("homeJ6");
	auto homeG1 = platform.ui.button("homeG1");

	//Prec
	auto precOn = platform.ui.button("prec_on");
	auto precOff = platform.ui.button("prec_off");

}

void onValueChangeEvent(hId id, const char *data)
{
    int i = 0;
    if (id == "hCode_line")
    {
	for (int i = 0; i < 512; i++){
	    UIcon[i] = 0;
	}
	while (data[i] != NULL)
	{
	    UIcon[i] = data[i];
	    i++;
	}
    }
}

void sendtoMotionManager()
{
    if (mode == 0)
    {
	    instruction_code code = {MOVE_JN, "", "", "", target[1], target[2], target[3], target[5], target[6]};
        InputData.AddInstruction(code, UI);
    }
    if (mode == 1)
    {
    	instruction_code code = {MOVE_CN, "", "", "", target[1], target[2], target[3], target[5], target[6]};
        InputData.AddInstruction(code, UI);
    }
}

void sendtoMotionManagerInter()
{
    if (mode == 0)
    {
        instruction_code code = {MOVE_JI, "", "", "", target[1], target[2], target[3], target[5], target[6]};
        InputData.AddInstruction(code, UI);
    }
    if (mode == 1)
    {
        instruction_code code = {MOVE_CI, "", "", "", target[1], target[2], target[3], target[5], target[6]};
        InputData.AddInstruction(code, UI);
    }
}

void onButtonEvent(hId id, ButtonEventType type)
{

    //UART(121100);
    if (type == ButtonEventType::Pressed)
    { // Pressed
	//UART(121200);
	// always
	//UI comand buttons
	//if (id == "btn_com") {}//hCode_line

	// PID buttons
	//if (id == "btn_kpp")
	//    tempKp += 0.05;
	//if (id == "btn_kpm")
	//    tempKp -= 0.05;
	//if (id == "btn_kdp")
	//    tempKd += 0.05;
	//if (id == "btn_kdm")
	//    tempKd -= 0.05;
	//if (id == "btn_kip")
	//    tempKi += 0.05;
	//if (id == "btn_kim")
	//    tempKi -= 0.05;

	if (id == "doP1"){program1_start();}
	if (id == "doP2"){program2_start();}
	if (id == "doP3"){program3_start();}
	if (id == "doP4"){program4_start();}
	if (id == "doP5"){program5_start();}
	if (id == "doP6"){program6_start();}
	if (id == "doP7"){program7_start();}
	if (id == "doP8"){program8_start();}

	if (id == "homeAll"){
		instruction_code code = {HOMEALL, "", "", "", 0, 0, 0, 0, 0};
		InputData.AddInstruction(code, UI);
	}
	if (id == "homeJ1"){
		instruction_code code = {HOMEJ1, "", "", "", 0, 0, 0, 0, 0};
		InputData.AddInstruction(code, UI);
	}
	if (id == "homeJ2"){
		instruction_code code = {HOMEJ2, "", "", "", 0, 0, 0, 0, 0};
		InputData.AddInstruction(code, UI);
		}
	if (id == "homeJ3"){
		instruction_code code = {HOMEJ3, "", "", "", 0, 0, 0, 0, 0};
		InputData.AddInstruction(code, UI);
		}
	if (id == "homeJ5"){
		instruction_code code = {HOMEJ5, "", "", "", 0, 0, 0, 0, 0};
		InputData.AddInstruction(code, UI);
		}
	if (id == "homeJ6"){
		instruction_code code = {HOMEJ6, "", "", "", 0, 0, 0, 0, 0};
		InputData.AddInstruction(code, UI);
		}
	if (id == "homeG1"){
		instruction_code code = {HOMEH1, "", "", "", 0, 0, 0, 0, 0};
		InputData.AddInstruction(code, UI);
		}

	if (id == "prec_on"){
		instruction_code code = {PRECYSION_ON, "", "", "", 100, 0, 0, 0, 0};
		InputData.AddInstruction(code, UI);
		}
	if (id == "prec_off"){
		instruction_code code = {PRECYSION_OFF, "", "", "", 0, 0, 0, 0, 0};
		InputData.AddInstruction(code, UI);
		}

	// grabber buttons
	if (id == "btn_close"){
	    instruction_code code = {H1CLOSE, "", "", "", 0, 0, 0, 0, 0};
        InputData.AddInstruction(code, UI);
	}
	if (id == "btn_open"){
	    instruction_code code = {H1OPEN, "", "", "", 0, 0, 0, 0, 0};
        InputData.AddInstruction(code, UI);
	}
	if (id == "btn_stop"){
	    instruction_code code = {H1STOP, "", "", "", 0, 0, 0, 0, 0};
        InputData.AddInstruction(code, UI);
	}

	// do button for code execution
	if (id == "btn_do"){
		passUIcom();
	}

	// show positions buttons
	if (id == "btn_pos1_show")
	{
	    pos_label_mode = 1;
	}
	if (id == "btn_pos2_show")
	{
	    pos_label_mode = 2;
	}
	if (id == "btn_pos3_show")
	{
	    pos_label_mode = 3;
	}
	if (id == "btn_pos4_show")
	{
	    pos_label_mode = 4;
	}
	if (id == "btn_pos5_show")
	{
	    pos_label_mode = 5;
	}
	if (id == "btn_pos6_show")
	{
	    pos_label_mode = 6;
	}
	if (id == "btn_pos0_show")
	{
	    pos_label_mode = 0;
	}
	if (id == "btn_pos7_show")
	{
	    pos_label_mode = 7;
	}

	if (id == "btnCartesianMov")
	{
	    mode = 1;
		InputData.changeInstructionInputType(UI, cartesianCo);
	    if (last_mode == 3 || last_mode == 2)
	    {
		//target[1] = MotionManager::get().getTarget(1);
		//target[2] = MotionManager::get().getTarget(2);
		//target[3] = MotionManager::get().getTarget(3);
		//target[4] = MotionManager::get().getTarget(4);
		//target[5] = MotionManager::get().getTarget(5);
	    }
	    last_mode = mode;
	    ErrorLogs::err().send(21);
	}
	if (id == "btnJointsMov")
	{
	    mode = 0;
		InputData.changeInstructionInputType(UI, jointsCo);
	    if (last_mode == 3 || last_mode == 2)
	    {
		//target[1] = MotionManager::get().getTarget(1);
		//target[2] = MotionManager::get().getTarget(2);
		//target[3] = MotionManager::get().getTarget(3);
		//target[4] = MotionManager::get().getTarget(4);
		//target[5] = MotionManager::get().getTarget(5);
	    }
	    last_mode = mode;
	    ErrorLogs::err().send(21);
	}
	if (id == "btnCodeUSBMov")
	{
	    mode = 3;
		InputData.changeInstructionInputType(SERIAL);
	    if (last_mode == 3 || last_mode == 2)
	    {
	    }
	    else
	    {
		//MotionManager::get().setTarget(target[1], target[2], target[3], target[4], target[5]);
	    }
	    last_mode = mode;
	    ErrorLogs::err().send(22);
	}
	if (id == "btnCodeUIMov")
	{
	    mode = 2;
		InputData.changeInstructionInputType(UI);
	    if (last_mode == 3 || last_mode == 2)
	    {
	    }
	    else
	    {
		//MotionManager::get().setTarget(target[1], target[2], target[3], target[4], target[5]);
	    }
	    last_mode = mode;
	    ErrorLogs::err().send(27);
	}

	if (mode == 0 || mode == 1)
	{
	    if (mode != modeLastBtn)
	    {
		switch (mode)
		{
		case 0:
		    step1 = 1;
		    step2 = 5;
		    step3 = 15;
		    step4 = 45;
		    break;
		case 1:
		    step1 = 0.5;
		    step2 = 1;
		    step3 = 5;
		    step4 = 10;
		    break;
		default:
		    step1 = 0;
		    step2 = 0;
		    step3 = 0;
		    step4 = 0;
		    break;
		}
		modeLastBtn = mode;
	    }

	    // write positions buttons
	    if (id == "btn_pos1_write")
	    {
		for (int k = 0; k < 9; k++)
		{
		    pos1[k] = target[k];
		}
		pos_label_mode = 0;
		char *temp;
		temp = new char[20];
		for (int i = 0; i < 20; i++)
		{
		    temp[i] = 0;
		}
		temp[0] = 80;
		temp[1] = 49;
		temp[2] = 85;
		temp[3] = 73;
		instruction_code code = {SET_J, temp, "", "", target[1], target[2], target[3], target[5], target[6]};
        InputData.AddInstruction(code, UI);
	    }
	    if (id == "btn_pos2_write")
	    {
		for (int k = 0; k < 9; k++)
		{
		    pos2[k] = target[k];
		}
		pos_label_mode = 0;
		char *temp;
		temp = new char[20];
		for (int i = 0; i < 20; i++)
		{
		    temp[i] = 0;
		}
		temp[0] = 80;
		temp[1] = 50;
		temp[2] = 85;
		temp[3] = 73;
		instruction_code code = {SET_J, temp, "", "", target[1], target[2], target[3], target[5], target[6]};
        InputData.AddInstruction(code, UI);
	    }
	    if (id == "btn_pos3_write")
	    {
		for (int k = 0; k < 9; k++)
		{
		    pos3[k] = target[k];
		}
		pos_label_mode = 0;
		char *temp;
		temp = new char[20];
		for (int i = 0; i < 20; i++)
		{
		    temp[i] = 0;
		}
		temp[0] = 80;
		temp[1] = 51;
		temp[2] = 85;
		temp[3] = 73;
	    instruction_code code = {SET_J, temp, "", "", target[1], target[2], target[3], target[5], target[6]};
        InputData.AddInstruction(code, UI);
	    }
	    if (id == "btn_pos4_write")
	    {
		for (int k = 0; k < 9; k++)
		{
		    pos4[k] = target[k];
		}
		pos_label_mode = 0;
		char *temp;
		temp = new char[20];
		for (int i = 0; i < 20; i++)
		{
		    temp[i] = 0;
		}
		temp[0] = 80;
		temp[1] = 52;
		temp[2] = 85;
		temp[3] = 73;
		instruction_code code = {SET_J, temp, "", "", target[1], target[2], target[3], target[5], target[6]};
        InputData.AddInstruction(code, UI);
	    }
	    if (id == "btn_pos5_write")
	    {
		for (int k = 0; k < 9; k++)
		{
		    pos5[k] = target[k];
		}
		pos_label_mode = 0;
		char *temp;
		temp = new char[20];
		for (int i = 0; i < 20; i++)
		{
		    temp[i] = 0;
		}
		temp[0] = 80;
		temp[1] = 53;
		temp[2] = 85;
		temp[3] = 73;
		instruction_code code = {SET_J, temp, "", "", target[1], target[2], target[3], target[5], target[6]};
        InputData.AddInstruction(code, UI);
	    }
	    if (id == "btn_pos6_write")
	    {
		for (int k = 0; k < 9; k++)
		{
		    pos6[k] = target[k];
		}
		pos_label_mode = 0;
		char *temp;
		temp = new char[20];
		for (int i = 0; i < 20; i++)
		{
		    temp[i] = 0;
		}
		temp[0] = 80;
		temp[1] = 54;
		temp[2] = 85;
		temp[3] = 73;
		instruction_code code = {SET_J, temp, "", "", target[1], target[2], target[3], target[5], target[6]};
        InputData.AddInstruction(code, UI);
	    }

	    // read positions buttons
	    if (id == "btn_pos1_read")
	    {
		for (int k = 0; k < 9; k++)
		{
		    target[k] = pos1[k];
		}
		pos_label_mode = 0;
		sendtoMotionManagerInter();
	    }
	    if (id == "btn_pos2_read")
	    {
		for (int k = 0; k < 9; k++)
		{
		    target[k] = pos2[k];
		}
		pos_label_mode = 0;
		sendtoMotionManagerInter();
	    }
	    if (id == "btn_pos3_read")
	    {
		for (int k = 0; k < 9; k++)
		{
		    target[k] = pos3[k];
		}
		pos_label_mode = 0;
		sendtoMotionManagerInter();
	    }
	    if (id == "btn_pos4_read")
	    {
		for (int k = 0; k < 9; k++)
		{
		    target[k] = pos4[k];
		}
		pos_label_mode = 0;
		sendtoMotionManagerInter();
	    }
	    if (id == "btn_pos5_read")
	    {
		for (int k = 0; k < 9; k++)
		{
		    target[k] = pos5[k];
		}
		pos_label_mode = 0;
		sendtoMotionManagerInter();
	    }
	    if (id == "btn_pos6_read")
	    {
		for (int k = 0; k < 9; k++)
		{
		    target[k] = pos6[k];
		}
		pos_label_mode = 0;
		sendtoMotionManagerInter();
	    }

	    // jog buttons target
	    if (id == "btn11")
	    {
		target[1] += step4;
		sendtoMotionManager();
	    }
	    if (id == "btn12")
	    {
		target[1] += step3;
		sendtoMotionManager();
	    }
	    if (id == "btn13")
	    {
		target[1] += step2;
		sendtoMotionManager();
	    }
	    if (id == "btn14")
	    {
		target[1] += step1;
		sendtoMotionManager();
	    }
	    if (id == "btn15")
	    {
		target[1] -= step1;
		sendtoMotionManager();
	    }
	    if (id == "btn16")
	    {
		target[1] -= step2;
		sendtoMotionManager();
	    }
	    if (id == "btn17")
	    {
		target[1] -= step3;
		sendtoMotionManager();
	    }
	    if (id == "btn18")
	    {
		target[1] -= step4;
		sendtoMotionManager();
	    }

	    if (id == "btn21")
	    {
		target[2] += step4;
		sendtoMotionManager();
	    }
	    if (id == "btn22")
	    {
		target[2] += step3;
		sendtoMotionManager();
	    }
	    if (id == "btn23")
	    {
		target[2] += step2;
		sendtoMotionManager();
	    }
	    if (id == "btn24")
	    {
		target[2] += step1;
		sendtoMotionManager();
	    }
	    if (id == "btn25")
	    {
		target[2] -= step1;
		sendtoMotionManager();
	    }
	    if (id == "btn26")
	    {
		target[2] -= step2;
		sendtoMotionManager();
	    }
	    if (id == "btn27")
	    {
		target[2] -= step3;
		sendtoMotionManager();
	    }
	    if (id == "btn28")
	    {
		target[2] -= step4;
		sendtoMotionManager();
	    }

	    if (id == "btn31")
	    {
		target[3] += step4;
		sendtoMotionManager();
	    }
	    if (id == "btn32")
	    {
		target[3] += step3;
		sendtoMotionManager();
	    }
	    if (id == "btn33")
	    {
		target[3] += step2;
		sendtoMotionManager();
	    }
	    if (id == "btn34")
	    {
		target[3] += step1;
		sendtoMotionManager();
	    }
	    if (id == "btn35")
	    {
		target[3] -= step1;
		sendtoMotionManager();
	    }
	    if (id == "btn36")
	    {
		target[3] -= step2;
		sendtoMotionManager();
	    }
	    if (id == "btn37")
	    {
		target[3] -= step3;
		sendtoMotionManager();
	    }
	    if (id == "btn38")
	    {
		target[3] -= step4;
		sendtoMotionManager();
	    }

	    if (id == "btn51")
	    {
		target[5] += step4;
		sendtoMotionManager();
	    }
	    if (id == "btn52")
	    {
		target[5] += step3;
		sendtoMotionManager();
	    }
	    if (id == "btn53")
	    {
		target[5] += step2;
		sendtoMotionManager();
	    }
	    if (id == "btn54")
	    {
		target[5] += step1;
		sendtoMotionManager();
	    }
	    if (id == "btn55")
	    {
		target[5] -= step1;
		sendtoMotionManager();
	    }
	    if (id == "btn56")
	    {
		target[5] -= step2;
		sendtoMotionManager();
	    }
	    if (id == "btn57")
	    {
		target[5] -= step3;
		sendtoMotionManager();
	    }
	    if (id == "btn58")
	    {
		target[5] -= step4;
		sendtoMotionManager();
	    }

	    if (id == "btn61")
	    {
		target[6] += step4;
		sendtoMotionManager();
	    }
	    if (id == "btn62")
	    {
		target[6] += step3;
		sendtoMotionManager();
	    }
	    if (id == "btn63")
	    {
		target[6] += step2;
		sendtoMotionManager();
	    }
	    if (id == "btn64")
	    {
		target[6] += step1;
		sendtoMotionManager();
	    }
	    if (id == "btn65")
	    {
		target[6] -= step1;
		sendtoMotionManager();
	    }
	    if (id == "btn66")
	    {
		target[6] -= step2;
		sendtoMotionManager();
	    }
	    if (id == "btn67")
	    {
		target[6] -= step3;
		sendtoMotionManager();
	    }
	    if (id == "btn68")
	    {
		target[6] -= step4;
		sendtoMotionManager();
	    }
	}
	/*else
		{
		    TODO: write positions buttons to serial movement
		}*/
	//tymczasowe
    }

    //MotorManagerUpdateTargetGlobal();
    //UART(122000);
}
