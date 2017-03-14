#include <cstddef>
#include <cstdint>
#include "hFramework.h"

#include "Addons.h"
#include "Arm.h"

#ifndef HROBOTCOMMANDINPUT
#define HROBOTCOMMANDINPUT

class CommandInput{
private:
    instruction_input_type input_type;
    type_co jog_type;
    Arm* robot;
public:
    CommandInput(Arm* t_robot);

    bool AddInstruction(instruction_code instruction, instruction_input_type from);
    bool AddInstruction(char* instruction, instruction_input_type from);
    void AddInstructionStream(char* instruction, instruction_input_type from);

    instruction_code CommandTranslation(char* command, char* param1, char* param2, 
    char* param3, char* param4, char* param5, char* param6, char* param7);

    void changeInstructionInputType(instruction_input_type new_type, type_co jog_new_type);
    void changeInstructionInputType(instruction_input_type new_type);
};

#endif