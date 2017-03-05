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
#include "ParseCommand.h"

Arm::Arm(){
    input_type = SERIAL;
    jog_type = jointsCo;
}

void Arm::ChangeInstructionInputType(instruction_input_type new_type, typeCo jog_new_type){
    input_type = new_type;
    jog_type = jog_new_type;
}

bool Arm::AddInstruction(instruction_code instruction, instruction_input_type from){
    if(from != input_type){
        return false;
    }
    else{

    }
}//TODO:

bool Arm::AddInstruction(char* instruction, instruction_input_type from){
    if(from != input_type){
        return false;
    }
    else{
        
    }
}//TODO:

bool Arm::AddInstructionStream(char* instruction, instruction_input_type from){
    if(from != input_type){
        return false;
    }
    else{
        
    }
}//TODO:

instruction_code Arm::CommandTranslation(const char* command, const  char* param1, const  char* param2, 
const  char* param3, const char* param4, const char* param5, const char* param6, const char* param7){
        
        if (strcmp(command, "MOVE") == 0)
	    {
            instruction_code code = {MOVE, param1, 0, 0, 0, 0, 0};
            return code;
	    }
	    if (strcmp(command, "MOVES") == 0)
	    {
            instruction_code code = {MOVES, param1, 0, 0, 0, 0, 0};
            return code;
	    }
	    if (strcmp(command, "SET") == 0)
	    {
		if (strcmp(param2, "J") == 0)
		{
            instruction_code code = {SET_J, param1, (float)atof(param3), (float)atof(param4), (float)atof(param5), (float)atof(param6), (float)atof(param7)};
            return code;
		}
		if (strcmp(param2, "R") == 0)
		{
            instruction_code code = {SET_R, param1, (float)atof(param3), (float)atof(param4), (float)atof(param5), (float)atof(param6), (float)atof(param7)};
            return code;
		}
		if (strcmp(param2, "C") == 0)
		{
            instruction_code code = {SET_C, param1, (float)atof(param3), (float)atof(param4), (float)atof(param5), (float)atof(param6), (float)atof(param7)};
            return code;
		}
		if (strcmp(param2, "HERE") == 0)
		{
		    if (strcmp(param3, "J") == 0)
		    {
                instruction_code code = {SET_HERE_J, param1, 0, 0, 0, 0, 0};
                return code;
		    }
		    if (strcmp(param3, "R") == 0)
		    {
                instruction_code code = {SET_HERE_R, param1, 0, 0, 0, 0, 0};
                return code;
		    }
		    if (strcmp(param3, "C") == 0)
		    {
                instruction_code code = {SET_HERE_C, param1, 0, 0, 0, 0, 0};
                return code;
		    }
		}
	    }
	    if (strcmp(command, "SHOWALL") == 0)
	    {
            instruction_code code = {SHOWALL, "", 0, 0, 0, 0, 0};
            return code;
	    }
	    if (strcmp(command, "SHOWCURRENT") == 0)
	    {
            if (strcmp(param1, "") == 0){
                instruction_code code = {SHOWCURRENT, "", 0, 0, 0, 0, 0};
                return code;
            }
            if (strcmp(param1, "J") == 0){
                instruction_code code = {SHOWCURRENT_J, "", 0, 0, 0, 0, 0};
                return code;
            }
            if (strcmp(param1, "R") == 0){
                instruction_code code = {SHOWCURRENT_R, "", 0, 0, 0, 0, 0};
                return code;
            }
            if (strcmp(param1, "C") == 0){
                instruction_code code = {SHOWCURRENT_C, "", 0, 0, 0, 0, 0};
                return code;
            }
	    }
	    if (strcmp(command, "SHOW") == 0)
	    {
		if (strcmp(param2, "") == 0)
		{
            instruction_code code = {SHOW, param1, 0, 0, 0, 0, 0};
            return code;
		}
		if (strcmp(param2, "J") == 0)
		{
            instruction_code code = {SHOW_J, param1, 0, 0, 0, 0, 0};
            return code;
		}
		if (strcmp(param2, "R") == 0)
		{
            instruction_code code = {SHOW_R, param1, 0, 0, 0, 0, 0};
            return code;
		}
		if (strcmp(param2, "C") == 0)
		{
            instruction_code code = {SHOW_C, param1, 0, 0, 0, 0, 0};
            return code;
		}
	    }
	    if (strcmp(command, "DELAY") == 0)
	    {
            instruction_code code = {SHOW_C, "", (float)atof(param1), 0, 0, 0, 0};
            return code;
	    }
	    if (strcmp(command, "PRECYSION") == 0)
	    {
		if (strcmp(param1, "ON") == 0)
		{
            instruction_code code = {PRECYSION_ON, "", (float)atof(param2), (float)atof(param3), 0, 0, 0};
            return code;
		}
		if (strcmp(param1, "OFF") == 0)
		{
            instruction_code code = {PRECYSION_OFF, "", (float)atof(param2), (float)atof(param3), 0, 0, 0};
            return code;
		}
	    }
	    if (strcmp(command, "CONFIG") == 0)
	    {
		if (strcmp(param1, "COM") == 0)
		{
		    if (strcmp(param2, "UI") == 0)
		    {
                instruction_code code = {CONFIG_COM_UI, "", 0, 0, 0, 0, 0};
                return code;
		    }
		    if (strcmp(param2, "SERIAL") == 0)
		    {
			    instruction_code code = {CONFIG_COM_SERIAL, "", 0, 0, 0, 0, 0};
                return code;
		    }
		}
		if (strcmp(command, "OFFSET") == 0)
		{
		    if (strcmp(param1, "ONPOINT") == 0)
		    {
                instruction_code code = {OFFSET_ONPOINT, "", 0, 0, 0, 0, 0};
                return code;
		    }
		    if (strcmp(param1, "INPOINT") == 0)
		    {
                instruction_code code = {OFFSET_INPOINT, param1, 0, 0, 0, 0, 0};
                return code;
		    }
		}
		if (strcmp(command, "RESETPOINTS") == 0)
		{
            instruction_code code = {RESETPOINTS, "", 0, 0, 0, 0, 0};
            return code;
		}
	    }

	    if (strcmp(command, "H1OPEN") == 0)
	    {
            instruction_code code = {H1OPEN, "", 0, 0, 0, 0, 0};
            return code;
	    }
	    if (strcmp(command, "H1CLOSE") == 0)
	    {
            instruction_code code = {H1CLOSE, "", 0, 0, 0, 0, 0};
            return code;
	    }
	    if (strcmp(command, "H1STOP") == 0)
	    {
            instruction_code code = {H1STOP, "", 0, 0, 0, 0, 0};
            return code;
	    }
    }

void Arm::ArmInit(){
	MotorManagerInit();
	sys.setSysLogDev(&devNull);
	sys.taskCreate(printfErrorTask);
	sys.taskCreate(ComandInputTaskSerial, 1, 2000, "ComandInputTS");
	sys.taskCreate(ComandInputTaskUI, 1, 2000, "ComandInputTU");
	platform.begin(&RPi);
	platform.ui.configHandler = cfgHandler;
	platform.ui.onButtonEvent = onButtonEvent;
	platform.ui.onValueChangeEvent = onValueChangeEvent;
	platform.ui.setProjectId("@@@PROJECT_ID@@@");
	sys.taskCreate(MotorManagerUpdateTask, 2, 400, "MotorManUpdate");
	sys.taskCreate(MotionTask);
	sys.taskCreate(printOnLabelsTask, 2, 2000, "labelsTask");
}
