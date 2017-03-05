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
    bool AddInstructionStream(char* instruction, instruction_input_type from);

    instruction_code CommandTranslation(const char* command, const  char* param1, const  char* param2, 
    const  char* param3, const char* param4, const char* param5, const char* param6, const char* param7);

    void ChangeInstructionInputType(instruction_input_type new_type, typeCo jog_new_type);
};

#endif