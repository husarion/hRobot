#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include <iostream>
#include <cstdio>
#include <vector>

#include "Addons.h"

#ifndef HROBOTARM
#define HROBOTARM

class Arm{
private:
    instruction_input_type input_type;
    typeCo jog_type;
public:
    Arm();
    void ArmInit();

    bool AddInstruction(instruction_code instruction, instruction_input_type from);
    bool AddInstruction(char* instruction, instruction_input_type from);
    void AddInstructionStream(char* instruction, instruction_input_type from);

    instruction_code CommandTranslation(char* command, char* param1, char* param2, 
    char* param3, char* param4, char* param5, char* param6, char* param7);

    void ChangeInstructionInputType(instruction_input_type new_type, typeCo jog_new_type);
    void ChangeInstructionInputType(instruction_input_type new_type);
};

#endif