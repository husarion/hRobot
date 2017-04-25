#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "MotorManager.h"
#include "SoftEnc.h"
#include "ServoCtrl.h"
#include "DblMotorCtrl.h"
#include "GripperCtrl.h"
#include "Addons.h"
#include "Config.h"

extern float current[9];
extern float target[9];
Coordinates offset(jointsCo, 0, 0, 0, 0, 0);
float jointTarget[6];

Coordinates minimum(jointsCo, J1_min, J2_min, J3_min, J4_min, J5_min);
Coordinates maximum(jointsCo, J1_max, J2_max, J3_max, J4_max, J5_max);

//EndSwitch Variables
//J1
bool stateP52;
bool stateP52last;
float stopInP52;
//J2
bool stateP53;
bool stateP53last;
float stopInP53;
//J3
bool stateP54;
bool stateP54last;
float stopInP54;
//J5
bool stateP62;
bool stateP62last;
float stopInP62;
//J6
bool stateP63;
bool stateP63last;
float stopInP63;
//H1
bool stateP64;
bool stateP64last;
float stopInP64;

SoftEnc soft_enkoder(hSens1.pin1, hSens2.pin1);

void motorManagerInitServos()
{
    hServoModule.enablePower();
}

void motorManagerInitMotors() {}

void motorManagerInitEncoders()
{
    //J1
    hMot1.setEncoderPolarity(Polarity::Reversed);
    //J22
    hMot2.setEncoderPolarity(Polarity::Reversed);
    //J3
    //	enkoder1.init();
    //	enkoder1.resetEncoderCnt();
    //J5
    soft_enkoder.init();
    hSens1.pin1.setIn_pu();
    hSens2.pin1.setIn_pu();
    soft_enkoder.resetEncoderCnt();
    //J6
    //soft_enkoder.setEncoderPolarity(Polarity::Reversed);
}

void motorManagerInit()
{
    motorManagerInitEncoders();
    motorManagerInitServos();
    motorManagerEndSwitchInit();
}

void motorManagerUpdateTask()
{
    //UART(212100);
    ServoCtrl J1(s1, J1_middle, J1_threshold, J1_kp_down, J1_ki_down,
    J1_kd_down, J1_kp_up, J1_ki_up, J1_kd_up, J1_error_saturate, 
    J1_integrator_saturate_down, J1_integrator_saturate_up);
    DblMotorCtrl J2(J2_threshold, J2_kp_down, J2_ki_down, J2_kd_down, 
    J2_kp_up, J2_ki_up, J2_kd_up, J2_error_saturate, 
    J2_integrator_saturate_down, J2_integrator_saturate_up);
    ServoCtrl J3(s2, J3_middle, J3_threshold, J3_kp_down, J3_ki_down, 
    J3_kd_down, J3_kp_up, J3_ki_up, J3_kd_up, J3_error_saturate, 
    J3_integrator_saturate_down, J3_integrator_saturate_up);
    ServoCtrl J5(s3, J5_middle, J5_threshold, J5_kp_down, J5_ki_down, 
    J5_kd_down, J5_kp_up, J5_ki_up, J5_kd_up, J5_error_saturate, 
    J5_integrator_saturate_down, J5_integrator_saturate_up);
    ServoCtrl J6(s4, J6_middle, J6_threshold, J6_kp_down, J6_ki_down, 
    J6_kd_down, J6_kp_up, J6_ki_up, J6_kd_up, J6_error_saturate, 
    J6_integrator_saturate_down, J6_integrator_saturate_up);
    GripperCrtl H1(h1);

    for (;;)
    {
        // sensor
        current[1] = (float)hMot1.getEncoderCnt() / encoder_tics_J1 + offset.k1;
        current[2] = (float)hMot2.getEncoderCnt() / encoder_tics_J2 + offset.k2;
        current[3] = ((float)hMot3.getEncoderCnt() / encoder_tics_J3 + offset.k3);
        current[5] = (float)hMot4.getEncoderCnt() / encoder_tics_J5 + offset.k4;
        current[6] = (float)soft_enkoder.getEncoderCnt() / encoder_tics_J6 + offset.k5;
        // motion

        if (endswitch_active)
        {
            endSwitchRun();
        }

        int t = sys.getRefTime();
        J1.update(-jointTarget[0] - current[1], t);
        J2.update(-(jointTarget[1] - current[2]), t);
        J3.update(-jointTarget[2] - current[3], t);
        J5.update(jointTarget[3] - current[5], t);
        J6.update(jointTarget[4] - current[6], t);
        H1.update(jointTarget[5]);

        sys.delay(50);
        LED2.toggle();

        //configuration
        //J2.set_pid_values(tempKp*10,tempKi*10,tempKd*10);
    }
}

void rangeCheck(){
    if(min_max_enable){
        if(homedP52)
            jointTarget[0] = saturateFloatUnsym(jointTarget[0], maximum.k1, minimum.k1);
        else
            jointTarget[0] = saturateFloatUnsym(jointTarget[0], 360, minimum.k1);
        
        if(homedP53)
            jointTarget[1] = saturateFloatUnsym(jointTarget[1], maximum.k2, minimum.k2);
        else
            jointTarget[1] = saturateFloatUnsym(jointTarget[1], 360, minimum.k2);

        if(homedP54)
            jointTarget[2] = saturateFloatUnsym(jointTarget[2], maximum.k3, minimum.k3);
        else
            jointTarget[2] = saturateFloatUnsym(jointTarget[2], 360, minimum.k3);

        if(homedP62)
            jointTarget[3] = saturateFloatUnsym(jointTarget[3], maximum.k4, minimum.k4);
        else
            jointTarget[3] = saturateFloatUnsym(jointTarget[3], 360, minimum.k4);

        if(homedP63)
            jointTarget[4] = saturateFloatUnsym(jointTarget[4], maximum.k5, minimum.k5);
        else
            jointTarget[4] = saturateFloatUnsym(jointTarget[4], 360, minimum.k5);
    }
}

void motorManagerUpdateTargetGlobal()
{
    jointTarget[0] = target[1];
    jointTarget[1] = target[2];
    jointTarget[2] = target[3];
    jointTarget[3] = target[5];
    jointTarget[4] = target[6];
    jointTarget[5] = target[7];
    
    rangeCheck();
}

void motorManagerUpdateTargetGlobalTask()
{
    for (;;)
    {
        motorManagerUpdateTargetGlobal();
        sys.delay(100);
    }
}

void motorManagerUpdateTargetDef(float j1, float j2, float j3, float j5, float j6)
{
    jointTarget[0] = j1;
    jointTarget[1] = j2;
    jointTarget[2] = j3;
    jointTarget[3] = j5;
    jointTarget[4] = j6;
    
    rangeCheck();
}

void motorManagerUpdateTargetDef(Coordinates point)
{
    if (point.type != jointsCo)
    {
        point.translate(jointsCo);
    }
        jointTarget[0] = point.k1;
        jointTarget[1] = point.k2;
        jointTarget[2] = point.k3;
        jointTarget[3] = point.k4;
        jointTarget[4] = point.k5;

        rangeCheck();
}

void motorManagerSetOffsetDef(int t_joint, float value)
{
    switch (t_joint)
    {
    case 1:
        offset.k1 = value;
        break;
    case 2:
        offset.k2 = value;
        break;
    case 3:
        offset.k3 = value;
        break;
    case 5:
        offset.k4 = value;
        break;
    case 6:
        offset.k5 = value;
        break;
    }
}

void motorManagerSetOffsetDef(Coordinates current_point)
{
    current_point.translate(jointsCo);
    offset = current_point;
}

bool checkIfInRange(Coordinates *point)
{
    if(min_max_enable){
    if (point->k1 != saturateFloatUnsym(point->k1, maximum.k1, minimum.k1))
        return false;
    if (point->k2 != saturateFloatUnsym(point->k2, maximum.k1, minimum.k2))
        return false;
    if (point->k3 != saturateFloatUnsym(point->k3, maximum.k1, minimum.k3))
        return false;
    if (point->k4 != saturateFloatUnsym(point->k4, maximum.k1, minimum.k4))
        return false;
    if (point->k5 != saturateFloatUnsym(point->k5, maximum.k1, minimum.k5))
        return false;
    }
    return true;
}

void setGripperValume(int volume)
{
    jointTarget[5] = volume;
}

void motorManagerEndSwitchInit()
{
    hSens5.pin2.setIn_pu(); //J1
    hSens5.pin3.setIn_pu(); //J2
    hSens5.pin4.setIn_pu(); //J3
    hSens6.pin2.setIn_pu(); //J5
    hSens6.pin3.setIn_pu(); //J6
    hSens6.pin4.setIn_pu(); //H1
}

void endSwitchRun()
{
    stateP52 = hSens5.pin2.read(); //J1
    if (stateP52 && !stateP52last)
    {
        stopInP52 = current[1];
        homedP52 = true;
    }
    if (stateP52 && jointTarget[0] > stopInP52)
    {
        jointTarget[0] = stopInP52;
    }
    stateP52last = stateP52;

    stateP53 = hSens5.pin3.read(); //J2
    if (stateP53 && !stateP53last)
    {
        stopInP53 = current[2];
        homedP53 = true;
        //
        //offset.k2 = current[2] - J2_max;
        Serial.printf("%d\n", current[2] - J2_max);
        //
    }
    if (stateP53 && jointTarget[1] > stopInP53)
    {
        jointTarget[1] = stopInP53;
    }
    stateP53last = stateP53;

    stateP54 = hSens5.pin4.read(); //J3
    if (stateP54 && !stateP54last)
    {
        stopInP54 = -current[3];
        homedP54 = true;
    }
    if (stateP54 && jointTarget[2] > stopInP54)
    {
        jointTarget[2] = stopInP54;
    }
    stateP54last = stateP54;

    stateP62 = hSens6.pin2.read(); //J5
    if (stateP62 && !stateP62last)
    {
        stopInP62 = current[5];
        homedP62 = true;
    }
    if (stateP62 && jointTarget[3] > stopInP62)
    {
        jointTarget[3] = stopInP62;
    }
    stateP62last = stateP62;

    stateP63 = hSens6.pin3.read(); //J6
    if (stateP63 && !stateP63last)
    {
        stopInP63 = current[6];
        homedP63 = true;
    }
    if (stateP63 && jointTarget[4] > stopInP63)
    {
        jointTarget[4] = stopInP63;
    }
    stateP63last = stateP63;


    stateP64 = hSens6.pin4.read(); //H1
    if (stateP64 && !stateP64last)
    {
        stopInP64 = 0;
        homedP64 = true;
    }
    if (stateP64 && jointTarget[5] > stopInP64)
    {
        jointTarget[5] = stopInP64;
    }
    stateP64last = stateP64;
}
