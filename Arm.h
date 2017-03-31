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

enum grabber_options{GOPEN, GCLOSE, GSTOP};

class Arm{
private:
public:
    Arm();
    void ArmInit();

    bool PassInstruction(instruction_code instruction);
    
    bool SET(PointCa Pt, type_co Co, float k1, float k2, float k3, float k4, float k5);//crete or replase point values
    bool MOVE(PointCa Pt);//moving from current point to set point in joint interpolation
    bool MOVES(PointCa Pt);//moving from current point to set point in cartesian interpolation
    bool DLY(float time);//delay in seconds
    bool DEPART(PointCa Pt, float distance);//moving from current point to set point translate in tool z axis in joint interpolation
    bool DEPARTS(PointCa Pt, float distance);//moving from current point to set point translate in tool z axis in cartesian interpolation
    bool RESETPOINTS();
    bool PRECYSION_ON(float t_treshold = 100, float t_time = 0);
    bool PRECYSION_OFF(float t_time = 0);
    bool JOG(joint_names j, float distance);
    bool JOG(float J1, float J2, float J3, float J5, float J6);
    bool SPEED(float value);
    bool CPY(PointCa Pn, PointCa Pd);// Create new point Pn, what is a copy of point Pd.
    bool CPY(PointCa Pn, PointCa Pd, type_co Co);// Create new point Pn, which is Pd transated to Coordinates Co.
    bool TRANS(PointCa Pi, PointCa Pd);// Translate point Pi about point Pd and save it as Pi
    bool TRANS(PointCa Pi, PointCa Pd, PointCa Pt);// Translate point Pi about point Pd and save it as Pt
    bool HERE(PointCa Pn, type_co Co = none);// saving current target set before as a point Pn in specified system.
    bool SHOW(PointCa Pd, type_co Co = none);
    bool HGRABBER(grabber_options option);
};

#endif