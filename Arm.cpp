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
	return MotionManager::get().instruction(instruction);
}

void Arm::ArmInit(){

	motorManagerInit();
	sys.setSysLogDev(&devNull);
	sys.taskCreate(printfErrorTask);
	sys.taskCreate(comandInputTaskSerial, 1, 600, "ComITS");
	platform.begin(&RPi);
	platform.ui.configHandler = cfgHandler;
	platform.ui.onButtonEvent = onButtonEvent;
	platform.ui.onValueChangeEvent = onValueChangeEvent;
	platform.ui.setProjectId("68b5fe1f1473854f");
	sys.taskCreate(motorManagerUpdateTask, 2, 600, "MorManU");
	sys.taskCreate(MotionTask, 2, 1000, "MotManT");
	sys.taskCreate(taskPrintOnLabels, 2, 1500, "labelsT");
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
	instruction_code code = {instruction_command::COPY, temp1, temp1, "", 0, 0, 0, 0, 0};
	return PassInstruction(code);
}

bool CPY(char* Pn, char* Pd, type_co Co){}//TODO:
