#include <cstddef>
#include <cstdint>
#include "hFramework.h"

#ifndef __MotorManager__
#define __MotorManager__

void motor_task();
void MotorManagerInit();
void MotorManagerInitServos();
void MotorManagerInitMotor();
void MotorManagerInitEncoders();

void MotorManagerJ1UpdateTask();
void MotorManagerJ2UpdateTask();
void MotorManagerJ3UpdateTask();
void MotorManagerJ5UpdateTask();
void MotorManagerJ6UpdateTask();

void MotorManagerUpdateTargetGlobal();
void MotorManagerUpdateTargetGlobalTask();
void MotorManagerUpdateTargetDef(float j1, float j2,  float j3,  float j5,  float j6);

#endif //__MotorManager__
