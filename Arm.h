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
    
    bool SET(char* Pt, type_co Co, float k1, float k2, float k3, float k4, float k5);//crete or replase point values
    bool MOVE(char* Pt);//moving from current point to set point in joint interpolation
    bool MOVES(char* Pt);//moving from current point to set point in cartesian interpolation
    bool DLY(float time);//delay in seconds
    bool DEPART(char* Pt, float distance);//moving from current point to set point translate in tool z axis in joint interpolation
    bool DEPARTS(char* Pt, float distance);//moving from current point to set point translate in tool z axis in cartesian interpolation
    bool RESETPOINTS();
    bool PRECYSION_ON();
    bool PRECYSION_OFF();
    bool JOG(joint_names j, float distance);
    bool JOG(float J1, float J2, float J3, float J5, float J6);
    bool SPEED(float value);
    bool CPY(char* Pn, char* Pd);
    bool CPY(char* Pn, char* Pd, type_co Co);//TODO:
};

#endif