#include <cstddef>
#include <cstdint>
#include "hFramework.h"
//#include "hCloudClient.h"
#include <iostream>
#include <cstdio>
#include <vector>

#include "Arm.h"
#include "Addons.h"
#include "MotionManager.h"
#include "ErrorLog.h"

#include "GeoMath.h"
//#include "UI_Buttons.h"
//#include "UI_Labels.h"
#include "ParseCommand.h"
#include "MotorManager.h"

Arm::Arm(){}

bool Arm::PassInstruction(instruction_code instruction){
	return MotionManager::get().instruction(instruction);
}

void Arm::ArmInit(){

	motorManagerInit();
	sys.setSysLogDev(&devNull);
	sys.taskCreate(printfErrorTask);
	sys.taskCreate(comandInputTaskSerial, 1, 600, "ComITS");
	platform.begin(&RPi);
	//platform.ui.configHandler = cfgHandler;
	//platform.ui.onButtonEvent = onButtonEvent;
	//platform.ui.onValueChangeEvent = onValueChangeEvent;
	//platform.ui.setProjectId("68b5fe1f1473854f");
	sys.taskCreate(motorManagerUpdateTask, 2, 600, "MorManU");
	sys.taskCreate(MotionTask, 2, 1000, "MotManT");
	//sys.taskCreate(taskPrintOnLabels, 2, 1500, "labelsT");	
}

bool Arm::SET(char* Pt, type_co Co, float k1, float k2, float k3, float k4, float k5){
	char* temp;
	temp = new char[20];
    for(int i=0; i<20; i++){
        temp[i] = Pt[i];
    }
	if(Co == jointsCo){
		instruction_code code = {instruction_command::SET_J, temp, "", "", k1, k2, k3, k4, k5};
		return PassInstruction(code);
	}
	if(Co == cylindricalCo){
		instruction_code code = {instruction_command::SET_R, temp, "", "", k1, k2, k3, k4, k5};
		return PassInstruction(code);
	}
	if(Co == cartesianCo){
		instruction_code code = {instruction_command::SET_C, temp, "", "", k1, k2, k3, k4, k5};
		return PassInstruction(code);
	}
	return false;
}

bool Arm::MOVE(char* Pt){
	char* temp;
	temp = new char[20];
    for(int i=0; i<20; i++){
        temp[i] = Pt[i];
    }
	instruction_code code = {instruction_command::MOVE, temp, "", "", 0, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::MOVES(char* Pt){
	char* temp;
	temp = new char[20];
    for(int i=0; i<20; i++){
        temp[i] = Pt[i];
    }
	instruction_code code = {instruction_command::MOVES, temp, "", "", 0, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::DLY(float time){
	instruction_code code = {instruction_command::DELAY, "", "", "", time*1000, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::DEPART(char* Pt, float distance){
	char* temp;
	temp = new char[20];
    for(int i=0; i<20; i++){
        temp[i] = Pt[i];
    }
	instruction_code code = {instruction_command::MOVE_D, temp, "", "", distance, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::DEPARTS(char* Pt, float distance){
	char* temp;
	temp = new char[20];
    for(int i=0; i<20; i++){
        temp[i] = Pt[i];
    }
	instruction_code code = {instruction_command::MOVES_D, temp, "", "", distance, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::RESETPOINTS(){
	instruction_code code = {instruction_command::RESETPOINTS, "", "", "", 0, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::PRECYSION_ON(){
	instruction_code code = {instruction_command::PRECYSION_ON, "", "", "", 100, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::PRECYSION_OFF(){
	instruction_code code = {instruction_command::PRECYSION_OFF, "", "", "", 100, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::JOG(joint_names j, float distance){
	instruction_code code;
	switch(j){
		case J1:
			code = {instruction_command::JOG_J1, "", "", "", distance, 0, 0, 0, 0};
			return PassInstruction(code);	
		break;
		case J2:
			 code = {instruction_command::JOG_J2, "", "", "", distance, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
		case J3:
			code = {instruction_command::JOG_J3, "", "", "", distance, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
		case J4:break;
		case J5:
			code = {instruction_command::JOG_J5, "", "", "", distance, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
		case J6:
			code = {instruction_command::JOG_J6, "", "", "", distance, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
	}
	return false;
}

bool Arm::JOG(float J1, float J2, float J3, float J5, float J6){
	instruction_code code = {instruction_command::JOG_J, "", "", "", J1, J2, J3, J4, J5};
	return PassInstruction(code);
}

bool Arm::SPEED(float value){
	instruction_code code = {instruction_command::SPEED, "", "", "", value, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::CPY(char* Pn, char* Pd){
	char* temp1;
	char* temp2;
	temp1 = new char[20];
	temp2 = new char[20];
    for(int i=0; i<20; i++){
        temp1[i] = Pn[i];
		temp2[i] = Pd[i];
    }
	instruction_code code = {instruction_command::COPY, temp1, temp2, "", 0, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::CPY(char* Pn, char* Pd, type_co Co){
	instruction_code code;
	char* temp1;
	char* temp2;
	temp1 = new char[20];
	temp2 = new char[20];
    for(int i=0; i<20; i++){
        temp1[i] = Pn[i];
		temp2[i] = Pd[i];
    }
	switch(Co){
		case cartesianCo:
			code = {instruction_command::COPY_C, temp1, temp2, "", 0, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
		case cylindricalCo:
			code = {instruction_command::COPY_R, temp1, temp2, "", 0, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
		case jointsCo:
			code = {instruction_command::COPY_J, temp1, temp2, "", 0, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
		case none:
			code = {instruction_command::COPY_J, temp1, temp2, "", 0, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
	}
	return false;
}

bool Arm::TRANS(char* Pi, char* Pd){
	char* temp1;
	char* temp2;
	temp1 = new char[20];
	temp2 = new char[20];
    for(int i=0; i<20; i++){
        temp1[i] = Pi[i];
		temp2[i] = Pd[i];
    }
	instruction_code code = {instruction_command::TRANSLATE, temp1, temp2, "", 0, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::TRANS(char* Pi, char* Pd, char* Pt){
	char* temp1;
	char* temp2;
	char* temp3;	
	temp1 = new char[20];
	temp2 = new char[20];
	temp3 = new char[20];
    for(int i=0; i<20; i++){
        temp1[i] = Pi[i];
		temp2[i] = Pd[i];
		temp3[i] = Pt[i];
    }
	instruction_code code = {instruction_command::TRANSLATE_SET, temp1, temp2, temp3, 0, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::HERE(char* Pn){
	char* temp;	
	temp = new char[20];
    for(int i=0; i<20; i++){
        temp[i] = Pn[i];
    }
	instruction_code code = {instruction_command::SET_HERE_J, temp, "", "", 0, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool Arm::HERE(char* Pn, type_co Co){
	char* temp;	
	temp = new char[20];
    for(int i=0; i<20; i++){
        temp[i] = Pn[i];
    }
	instruction_code code;
	switch(Co){
	case cartesianCo:
		code = {instruction_command::SET_HERE_C, temp, "", "", 0, 0, 0, 0, 0};
		return PassInstruction(code);
	break;
	case cylindricalCo:
		code = {instruction_command::SET_HERE_R, temp, "", "", 0, 0, 0, 0, 0};
		return PassInstruction(code);
	break;
	case jointsCo:
		code = {instruction_command::SET_HERE_J, temp, "", "", 0, 0, 0, 0, 0};
		return PassInstruction(code);
	break;
	case none:
		code = {instruction_command::SET_HERE_J, temp, "", "", 0, 0, 0, 0, 0};
		return PassInstruction(code);
	break;
	}
	return false;
}

void ftoa(char* str, float number){
	float t = number;
	int steps = 0;
	do{
		steps++;
		t /= 10;
	}while(t>=1);
	steps += 3;
	int iter = steps;
	int num = (int)(number*1000);
	for(int i = 0; i < steps; i++){
		if(i == 3){
			str[iter] = 44;
			iter--;
		}
		switch(num % 10){
			case 0 : str[iter] = 48; break;
			case 1 : str[iter] = 49; break;
			case 2 : str[iter] = 50; break;
			case 3 : str[iter] = 51; break;
			case 4 : str[iter] = 52; break;
			case 5 : str[iter] = 53; break;
			case 6 : str[iter] = 54; break;
			case 7 : str[iter] = 55; break;
			case 8 : str[iter] = 56; break;
			case 9 : str[iter] = 57; break;
		}
		num /= 10;
		iter--;
	}
}

void addStrAndFloat(char* str, char* text1, size_t size1, char* text2, char* textJ1, size_t size2, float j1, char* textJ2, size_t size3, float j2,
char* textJ3, size_t size4, float j3, char* textJ4, size_t size5, float j4, char* textJ5, size_t size6, float j5, char* type, size_t size7){
	int iter = 0;
	for(size_t i = 0; i< size1; i++){
		str[iter] = text1[i];
		iter++;
	}
	for(size_t i = 0; i< 20; i++){
		if(text2[i] == 0){
			break;
		}
		str[iter] = text2[i];
		iter++;
	}
	for(size_t i = 0; i< size2; i++){
		str[iter] = textJ1[i];
		iter++;
	}
	char* temp;
	temp = new char[20];
	for(int i = 0; i< 20 ; i++){
		temp[i] = 0;
	}
	ftoa(temp, j1);
	for(size_t i = 0; i< 20; i++){
		if(text2[i] == 0){
			break;
		}
		str[iter] = temp[i];
		iter++;
	}
	for(size_t i = 0; i< size3; i++){
		str[iter] = textJ2[i];
		iter++;
	}
	for(int i = 0; i< 20 ; i++){
		temp[i] = 0;
	}
	ftoa(temp, j2);
	for(size_t i = 0; i< 20; i++){
		if(text2[i] == 0){
			break;
		}
		str[iter] = temp[i];
		iter++;
	}
	for(size_t i = 0; i< size4; i++){
		str[iter] = textJ3[i];
		iter++;
	}
	for(int i = 0; i< 20 ; i++){
		temp[i] = 0;
	}
	ftoa(temp, j3);
	for(size_t i = 0; i< 20; i++){
		if(text2[i] == 0){
			break;
		}
		str[iter] = temp[i];
		iter++;
	}
	for(size_t i = 0; i< size5; i++){
		str[iter] = textJ4[i];
		iter++;
	}
	for(int i = 0; i< 20 ; i++){
		temp[i] = 0;
	}
	ftoa(temp, j4);
	for(size_t i = 0; i< 20; i++){
		if(text2[i] == 0){
			break;
		}
		str[iter] = temp[i];
		iter++;
	}
	for(size_t i = 0; i< size6; i++){
		str[iter] = textJ5[i];
		iter++;
	}
	for(int i = 0; i< 20 ; i++){
		temp[i] = 0;
	}
	ftoa(temp, j5);
	for(size_t i = 0; i< 20; i++){
		if(text2[i] == 0){
			break;
		}
		str[iter] = temp[i];
		iter++;
	}
	for(size_t i = 0; i< size7; i++){
		str[iter] = type[i];
		iter++;
	}
	str[iter] = 9;
	str[iter+1] = '\n';
}

bool Arm::SHOW(char* Pd, type_co Co){
	Coordinates a;
	a = MotionManager::get().getPoint(Pd, Co);
	char* str;
	str = new char(64);
	for(int i = 0; i< 64; i++){
		str[i]=0;
	}
	if(a.type != none){
		switch(Co){
		case jointsCo:
			addStrAndFloat(str, "Point ", 6, Pd, "\tJ1: ", 5, a.k1, "\tJ2: ", 5 , a.k2, "\tJ3: ", 5, a.k3, "\tJ5: ", 5, a.k4, "\tJ6: ", 5, a.k5, "\tjointsCo", 9);
		break;
		case cylindricalCo:
			addStrAndFloat(str, "Point ", 6, Pd, "\tR: ", 4, a.k1, "\tH: ", 4, a.k2, "\tF: ", 4, a.k3, "\tA: ", 4, a.k4, "\tB: ", 4, a.k5, "\tcylindricalCo", 14);
			break;
		case cartesianCo:
			addStrAndFloat(str, "Point ", 6, Pd, "\tX: ", 4, a.k1, "\tY: ", 4, a.k2, "\tZ: ", 4, a.k3, "\tA: ", 4, a.k4, "\tB: ", 4, a.k5, "\tcartesianCo", 11);
			break;
		case none:
			break;
		}
		Serial.printf("%s\n", str);
		return true;
	}
	else{
		return false;
	}
}

bool Arm::HGRABBER(grabber_options option){
	instruction_code code;
	switch(option){
		case GOPEN:
			code = {instruction_command::H1OPEN, "", "", "", 0, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
		case GCLOSE:
			code = {instruction_command::H1CLOSE, "", "", "", 0, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
		case GSTOP:
			code = {instruction_command::H1STOP, "", "", "", 0, 0, 0, 0, 0};
			return PassInstruction(code);
		break;
	}
	return false;
}
