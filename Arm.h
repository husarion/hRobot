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
public:
    Arm();
    void ArmInit();

    bool PassInstruction(instruction_code instruction);
    
};

#endif