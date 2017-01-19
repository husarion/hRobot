#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "MotorManager.h"
#include "SoftEnc.h"
#include "ServoCtrl.h"


extern float current[9];
extern float target[9];
float cur[2];
extern float offset[9];
float jointTarget[5];

const float encoder_tics_J1 = 20;
const float encoder_tics_J21 = 72000;
const float encoder_tics_J22 = 72000;
const float encoder_tics_J3 = 20 / 4.355;
const float encoder_tics_J5 = 4.4;
const float encoder_tics_J6 = 0.2;

extern float tempKp;
extern float tempKi;
extern float tempKd;

SoftEnc enkoder1(hExt.pin1, hExt.pin2);
SoftEnc enkoder2(hSens1.pin1, hSens2.pin1);

IServo& s1 = hServoModule.servo1;
IServo& s2 = hServoModule.servo2;
IServo& s3 = hServoModule.servo3;
IServo& s4 = hServoModule.servo4;

void MotorManagerInitServos()
{
	hServoModule.enablePower();
}

void MotorManagerInitMotors()
{

}

void MotorManagerInitEncoders()
{

	//J1
	hMot1.setEncoderPolarity(Polarity::Reversed);

	//J22
	hMot3.setEncoderPolarity(Polarity::Reversed);

	//J3
	enkoder1.init();
	enkoder1.resetEncoderCnt();

	//J5
    enkoder2.init();
	hSens1.pin1.setIn_pu();
	hSens2.pin1.setIn_pu();
	enkoder2.resetEncoderCnt();

	//J6
	hMot4.setEncoderPolarity(Polarity::Reversed);


}

void MotorManagerInit()
{

	MotorManagerInitEncoders();
	MotorManagerInitServos();
	MotorManagerInitMotors();

}

void MotorManagerJ1UpdateTask()
{
    ServoCtrl J1(s1, 1125, 2, 0.9, 0.4, 0.2, 100, 20);
	for (;;) {
		current[1] = (float)hMot1.getEncoderCnt() / encoder_tics_J1 + offset[1];
        J1.update(jointTarget[0] - current[1] , sys.getRefTime());
		sys.delay(50);
	}
}

void MotorManagerJ2UpdateTask()
{
    //DblMotorCtrl J2();
    
	for (;;) {
		cur[0] = (float)hMot2.getEncoderCnt() / encoder_tics_J21 + offset[2];
		cur[1] = (float)hMot3.getEncoderCnt() / encoder_tics_J22 + offset[2];
		current[2] = (cur[0] + cur[1]) / 2;
		//J21.update(jointTarget[1] - cur[0] , sys.getRefTime());
		//J22.update(jointTarget[1] - cur[1] , sys.getRefTime());
		sys.delay(50);
	}
}

void MotorManagerJ3UpdateTask()
{
    ServoCtrl J3(s2, 1470, 0, 3.5, 0.6, 1.5, 100, 5);
    for (;;) {
		current[3] = (float)enkoder1.getEncoderCnt() / encoder_tics_J3 + offset[3];
		J3.update(-jointTarget[2] - current[3] , sys.getRefTime());
		sys.delay(50);
	}
}

void MotorManagerJ5UpdateTask()
{
    ServoCtrl J5(s3, 1730, 0, 1, 0, 0, 100, 20);
    for (;;) {
		current[5] = (float)enkoder2.getEncoderCnt() / encoder_tics_J5 + offset[5];
		J5.update(jointTarget[3] - current[5] , sys.getRefTime());
		sys.delay(50);
	}
}

void MotorManagerJ6UpdateTask()
{
    ServoCtrl J6(s4, 1125, 0, 1, 0, 0, 100, 20);
    for (;;) {
		current[6] = (float)hMot4.getEncoderCnt() / encoder_tics_J6 + offset[6];
		J6.update(jointTarget[4] - current[6] , sys.getRefTime());
		sys.delay(50);
	}
}

void MotorManagerUpdateTargetGlobal()
{
	jointTarget[0] = target[1];
	jointTarget[1] = target[2];
	jointTarget[2] = target[3];
	jointTarget[3] = target[5];
	jointTarget[4] = target[6];
}

void MotorManagerUpdateTargetGlobalTask()
{
	for (;;) {
		MotorManagerUpdateTargetGlobal();
		sys.delay(100);
	}
}

void MotorManagerUpdateTargetDef(float j1, float j2,  float j3,  float j5,  float j6)
{
	jointTarget[0] = j1;
	jointTarget[1] = j2;
	jointTarget[2] = j3;
	jointTarget[3] = j5;
	jointTarget[4] = j6;
}