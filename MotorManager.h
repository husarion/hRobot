#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "Addons.h"

#ifndef __MotorManager__
#define __MotorManager__

void motor_task();
void motorManagerInit();
void motorManagerInitServos();
void motorManagerInitEncoders();

void motorManagerUpdateTask();

void motorManagerUpdateTargetGlobal();
void motorManagerUpdateTargetGlobalTask();
void motorManagerUpdateTargetDef(float j1, float j2, float j3, float j5, float j6);
void motorManagerUpdateTargetDef(Coordinates point);

void motorManagerSetOffsetDef(int t_joint, float value);
void motorManagerSetOffsetDef(Coordinates current_point);

bool checkOverRange(Coordinates *point);
bool checkRange(Coordinates *point);

void motorManagerEndswitchInit();
void endswitchRun();

void setGripperValume(int volume);

#endif //__MotorManager__
