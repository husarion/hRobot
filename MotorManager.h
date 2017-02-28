#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "Addons.h"

#ifndef __MotorManager__
#define __MotorManager__

void motor_task();
void MotorManagerInit();
void MotorManagerInitServos();
void MotorManagerInitMotor();
void MotorManagerInitEncoders();

void MotorManagerUpdateTask();

void MotorManagerUpdateTargetGlobal();
void MotorManagerUpdateTargetGlobalTask();
void MotorManagerUpdateTargetDef(float j1, float j2,  float j3,  float j5,  float j6);
void MotorManagerUpdateTargetDef(Coordinates point);

void MotorManagerSetOffsetDef(int t_joint, float value);
void MotorManagerSetOffsetDef(Coordinates current_point);

bool CheckOverRange(Coordinates* point);
bool CheckRange(Coordinates* point);

void EndSwitchInit();
void EndSwitchRun();

void setGripperValume(int volume);

#endif //__MotorManager__
