// hRobot_0_05_09 edited by mzabinski94@gmail.com
// from hRobot_0_05_06

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <cstdio>
#include <stdio.h>

#include "hFramework.h"
#include "MotionManager.h"
#include "Addons.h"
#include "ErrorLog.h"
#include "MotorManager.h"

extern float current[9];
extern float target[9];
extern bool homedP52;
extern bool homedP53;
extern bool homedP54;
extern bool homedP62;
extern bool homedP63;
extern bool homedP64;

Coordinates to_send;

//Interpolation setings
const int time_motion_task = 100;
const float time_iteration = 75;
const float step_mul = 10;

void MotionTask()
{
    for (;;)
    {
        sys.delay(time_motion_task);
        MotionManager::get().update();
    }
}

void MotionManager::moveCartesianInter()
{
    if (targetPoint.type != cartesianCo)
    {
        targetPoint.translate(cartesianCo);
    }
    Coordinates curentPointC = curentPoint;
    curentPointC.translate(cartesianCo);
    float dis = sqrt(pow(curentPointC.k1-targetPoint.k1, 2)+pow(curentPointC.k2-targetPoint.k2, 2)+pow(curentPointC.k3-targetPoint.k3, 2));

    float speed = 10;//mm/s
    float l = dis*speed/time_iteration;
    int iter = dis/l;

    float x_tra = (targetPoint.k1 - curentPointC.k1 )/iter;
    float y_tra = (targetPoint.k2 - curentPointC.k2 )/iter;
    float z_tra = (targetPoint.k3 - curentPointC.k3 )/iter;
    float a_tra = (targetPoint.k4 - curentPointC.k4 )/iter;
    float b_tra = (targetPoint.k5 - curentPointC.k5 )/iter;

    to_send = curentPointC;
    for(int i =0; i< iter; i++){
        to_send.k1 += x_tra;
        to_send.k2 += y_tra;
        to_send.k3 += z_tra;
        to_send.k4 += a_tra;
        to_send.k5 += b_tra;
        motorManagerUpdateTargetDef(to_send);
        sys.delay(time_iteration);
    }
    to_send = targetPoint;
    motorManagerUpdateTargetDef(to_send);
}

void MotionManager::moveCartesianNorm()
{
    if (targetPoint.type == cartesianCo)
    {
        motorManagerUpdateTargetDef(targetPoint);
    }
    else
    {
        targetPoint.translate(cartesianCo);
        motorManagerUpdateTargetDef(targetPoint);
    }
}

void MotionManager::moveJointNorm()
{
    if (targetPoint.type == jointsCo)
    {
        motorManagerUpdateTargetDef(targetPoint);
    }
    else
    {
        targetPoint.translate(jointsCo);
        motorManagerUpdateTargetDef(targetPoint);
    }
}

void MotionManager::moveJointInter()
{   
    if (targetPoint.type != jointsCo)
    {
        targetPoint.translate(jointsCo);
    }
    
    /////////
    float internal_speed = 5; //5mm/s
    float ovrd = 1;           //100% OVRD
    /////////
    //curentPoint.Translate(jointsCo);
    float dis;
    dis = pointToPointDistance(targetPoint, curentPoint);
    int steps = (int)((dis / (internal_speed * ovrd * 10)) * step_mul);
    float j1_iter_step = (targetPoint.k1 - curentPoint.k1) / steps;
    float j2_iter_step = (targetPoint.k2 - curentPoint.k2) / steps;
    float j3_iter_step = (targetPoint.k3 - curentPoint.k3) / steps;
    float j5_iter_step = (targetPoint.k4 - curentPoint.k4) / steps;
    float j6_iter_step = (targetPoint.k5 - curentPoint.k5) / steps;
    to_send = curentPoint;
    for (int i = 0; i < steps; i++)
    {
        to_send.k1 += j1_iter_step;
        to_send.k2 += j2_iter_step;
        to_send.k3 += j3_iter_step;
        to_send.k4 += j5_iter_step;
        to_send.k5 += j6_iter_step;
        motorManagerUpdateTargetDef(to_send);
        sys.delay(time_iteration);
    }
}

void MotionManager::homejoint(joint_names joint){
    Coordinates a;
    switch(joint){
        case J1 : 
            while(!homedP52){
                a.k1 = current[1] + 1;
                a.k2 = current[2];
                a.k3 = current[3];
                a.k4 = current[5];
                a.k5 = current[6];
                targetPoint = a;
                moveJointNorm();
                sys.delay(time_iteration);
            }
        break;
        case J2 : 
            while(!homedP53){
                a.k1 = current[1];
                a.k2 = current[2] + 1;
                a.k3 = current[3];
                a.k4 = current[5];
                a.k5 = current[6];
                targetPoint = a;
                moveJointNorm();
                sys.delay(time_iteration);
            }
        break;
        case J3 : 
            while(!homedP54){
                a.k1 = current[1];
                a.k2 = current[2];
                a.k3 = current[3] + 1;
                a.k4 = current[5];
                a.k5 = current[6];
                targetPoint = a;
                moveJointNorm();
                sys.delay(time_iteration);
            }
        break;
        case J4 : break;
        case J5 : 
            while(!homedP62){
                a.k1 = current[1];
                a.k2 = current[2];
                a.k3 = current[3];
                a.k4 = current[5] + 1;
                a.k5 = current[6];
                targetPoint = a;
                moveJointNorm();
                sys.delay(time_iteration);
            }
        break;
        case J6 : 
            while(!homedP63){
                a.k1 = current[1];
                a.k2 = current[2];
                a.k3 = current[3];
                a.k4 = current[5];
                a.k5 = current[6] + 1;
                targetPoint = a;
                moveJointNorm();
                sys.delay(time_iteration);
            }
        break;
        case H1 : 
            instruction_code code = {H1OPEN, "", "", "", 0, 0, 0, 0, 0};
            MotionManager::get().instruction(code);
        break;
    }
}

void MotionManager::update()
{
    curentPoint.k1 = current[1];
    curentPoint.k2 = current[2];
    curentPoint.k3 = current[3];
    curentPoint.k4 = current[5];
    curentPoint.k5 = current[6];
    curentPoint.type = jointsCo;
    //curentPoint = targetPoint;
    //curentPoint.translate(jointsCo);
    Coordinates a;
    if (motions.size() > 0)
    {
        switch(motions[0].instruction.comand){
        case NOCOMMAND: break;
        
        case SET_J:
            addPoint(motions[0].instruction.point_name, jointsCo, 
            motions[0].instruction.param1, motions[0].instruction.param2, 
            motions[0].instruction.param3, motions[0].instruction.param4, 
            motions[0].instruction.param5);
        break;
        case SET_C:
            addPoint(motions[0].instruction.point_name, cartesianCo, 
            motions[0].instruction.param1, motions[0].instruction.param2, 
            motions[0].instruction.param3, motions[0].instruction.param4, 
            motions[0].instruction.param5);break;
        case SET_R:
            addPoint(motions[0].instruction.point_name, cylindricalCo, 
            motions[0].instruction.param1, motions[0].instruction.param2, 
            motions[0].instruction.param3, motions[0].instruction.param4, 
            motions[0].instruction.param5);break;
        case SET_HERE_J:
            addPoint(motions[0].instruction.point_name, jointsCo);    
        break;
        case SET_HERE_C:
            addPoint(motions[0].instruction.point_name, cartesianCo);
        break;
        case SET_HERE_R:
            addPoint(motions[0].instruction.point_name, cylindricalCo);
        break;
        
        case SHOWALL:
            showAll();
        break;
        case SHOWCURRENT:
            showCurrent();
        break;
        case SHOWCURRENT_J:
            showCurrent(jointsCo);
        break;
        case SHOWCURRENT_C:
            showCurrent(cartesianCo);
        break;
        case SHOWCURRENT_R:
            showCurrent(cylindricalCo);
        break;
        case SHOW:
            show(motions[0].instruction.point_name);
        break;
        case SHOW_J:
            show(motions[0].instruction.point_name, jointsCo);
        break;
        case SHOW_C:
            show(motions[0].instruction.point_name, cartesianCo);
        break;
        case SHOW_R:
            show(motions[0].instruction.point_name, cylindricalCo);
        break;
        
        case MOVE:
            if (checkPoint(motions[0].instruction.point_name))
            {
                a = findPoint(motions[0].instruction.point_name);
                a.translate(jointsCo);
                targetPoint = a;
                moveJointInter();
                waitForReachingTarget();
            }
            else
            {
                ErrorLogs::err().sendPar(18, motions[0].instruction.point_name);
            }
        break;
        case MOVE_D:
            if (checkPoint(motions[0].instruction.point_name))
            {
                a = findPoint(motions[0].instruction.point_name);
                a.translate(cartesianCo);
                a.k3 = a.k3 + motions[0].instruction.param1;
                a.translate(jointsCo);
                targetPoint = a;
                moveJointInter();
                waitForReachingTarget();
            }
            else
            {
                ErrorLogs::err().sendPar(18, motions[0].instruction.point_name);
            }
        break;
        case MOVE_JI:
                a.k1 = motions[0].instruction.param1;
                a.k2 = motions[0].instruction.param2;
                a.k3 = motions[0].instruction.param3;
                a.k4 = motions[0].instruction.param4;
                a.k5 = motions[0].instruction.param5;
                a.type = jointsCo;
                targetPoint = a;
                moveJointInter();
                waitForReachingTarget();
        break;
        case MOVE_CI:
                a.k1 = motions[0].instruction.param1;
                a.k2 = motions[0].instruction.param2;
                a.k3 = motions[0].instruction.param3;
                a.k4 = motions[0].instruction.param4;
                a.k5 = motions[0].instruction.param5;
                a.type = cartesianCo;
                targetPoint = a;
                moveCartesianInter();
                waitForReachingTarget();
        break;
        case MOVE_JN:
                a.k1 = motions[0].instruction.param1;
                a.k2 = motions[0].instruction.param2;
                a.k3 = motions[0].instruction.param3;
                a.k4 = motions[0].instruction.param4;
                a.k5 = motions[0].instruction.param5;
                a.type = jointsCo;
                targetPoint = a;
                moveJointNorm();
                waitForReachingTarget();
        break;
        case MOVE_CN:
                a.k1 = motions[0].instruction.param1;
                a.k2 = motions[0].instruction.param2;
                a.k3 = motions[0].instruction.param3;
                a.k4 = motions[0].instruction.param4;
                a.k5 = motions[0].instruction.param5;
                a.type = cartesianCo;
                targetPoint = a;
                moveCartesianNorm();
                waitForReachingTarget();
        break;
        
        case MOVES:
            if (checkPoint(motions[0].instruction.point_name))
            {
                a = findPoint(motions[0].instruction.point_name);
                a.translate(cartesianCo);
                targetPoint = a;
                moveCartesianInter();
                waitForReachingTarget();
            }
            else
            {
                ErrorLogs::err().sendPar(18, motions[0].instruction.point_name);
            }
        break;
        case MOVES_D:
            if (checkPoint(motions[0].instruction.point_name))
            {
                a = findPoint(motions[0].instruction.point_name);
                a.translate(cartesianCo);
                a.k3 = a.k3 + motions[0].instruction.param1;
                targetPoint = a;
                moveCartesianInter();
                waitForReachingTarget();
            }
            else
            {
                ErrorLogs::err().sendPar(18, motions[0].instruction.point_name);
            }
        break;
        
        case JOG_J:
            a = curentPoint;
            a.translate(jointsCo);
            a.k1 += motions[0].instruction.param1;
            a.k2 += motions[0].instruction.param2;
            a.k3 += motions[0].instruction.param3;
            a.k4 += motions[0].instruction.param4;
            a.k5 += motions[0].instruction.param5;
            targetPoint = a;
            moveJointNorm();
            waitForReachingTarget();
        break;
        case JOG_R:
            a = curentPoint;
            a.translate(cylindricalCo);
            a.k1 += motions[0].instruction.param1;
            a.k2 += motions[0].instruction.param2;
            a.k3 += motions[0].instruction.param3;
            a.k4 += motions[0].instruction.param4;
            a.k5 += motions[0].instruction.param5;
            a.translate(jointsCo);
            targetPoint = a;
            moveJointNorm();
            waitForReachingTarget();
        break;
        case JOG_C:
            a = curentPoint;
            a.translate(cartesianCo);
            a.k1 += motions[0].instruction.param1;
            a.k2 += motions[0].instruction.param2;
            a.k3 += motions[0].instruction.param3;
            a.k4 += motions[0].instruction.param4;
            a.k5 += motions[0].instruction.param5;
            targetPoint = a;
            moveCartesianNorm();
            waitForReachingTarget();
        break;
        case JOG_X:
            a = curentPoint;
            a.translate(cartesianCo);
            a.k1 += motions[0].instruction.param1;
            targetPoint = a;
            moveCartesianNorm();
            waitForReachingTarget();
        break;
        case JOG_Y:
            a = curentPoint;
            a.translate(cartesianCo);
            a.k2 += motions[0].instruction.param1;
            targetPoint = a;
            moveCartesianNorm();
            waitForReachingTarget();
        break;
        case JOG_Z:
            a = curentPoint;
            a.translate(cartesianCo);
            a.k3 += motions[0].instruction.param1;
            targetPoint = a;
            moveCartesianNorm();
            waitForReachingTarget();
        break;
        case JOG_J1:
            a = curentPoint;
            a.translate(jointsCo);
            a.k1 += motions[0].instruction.param1;
            targetPoint = a;
            moveJointNorm();
            waitForReachingTarget();
        break;
        case JOG_J2:
            a = curentPoint;
            a.translate(jointsCo);
            a.k2 += motions[0].instruction.param1;
            targetPoint = a;
            moveJointNorm();
            waitForReachingTarget();
        break;
        case JOG_J3:
            a = curentPoint;
            a.translate(jointsCo);
            a.k3 += motions[0].instruction.param1;
            targetPoint = a;
            moveJointNorm();
            waitForReachingTarget();
        break;
        case JOG_J5:
            a = curentPoint;
            a.translate(jointsCo);
            a.k4 += motions[0].instruction.param1;
            targetPoint = a;
            moveJointNorm();
            waitForReachingTarget();
        break;
        case JOG_J6:
            a = curentPoint;
            a.translate(jointsCo);
            a.k5 += motions[0].instruction.param1;
            targetPoint = a;
            moveJointNorm();
            waitForReachingTarget();
        break;

        case DELAY:
            sys.delay(motions[0].instruction.param1);
        break;
        
        case H1OPEN:
            griperOpen();
        break;
        case H1CLOSE:
            griperClose();
        break;
        case H1STOP:
            griperStop();
        break;
        
        case RESETPOINTS:
            clearPoints();
        break;
        
        case PRECYSION_ON:
            setPresitionMode(true, motions[0].instruction.param1, (int)motions[0].instruction.param2);
		    ErrorLogs::err().sendPar(28, (int)motions[0].instruction.param2);
        break;
        case PRECYSION_OFF:
            setPresitionMode(false, motions[0].instruction.param1, (int)motions[0].instruction.param2);
		    ErrorLogs::err().sendPar(28, (int)motions[0].instruction.param2);
        break;

        case SPEED:
            ovrd_speed = saturateFloatUnsym(motions[0].instruction.param1, 0, 100)/100;
        break;
        case COPY:
            copyPoint(motions[0].instruction.point_1_name, motions[0].instruction.point_name);
        break;
        case COPY_J:
            copyPoint(motions[0].instruction.point_1_name, motions[0].instruction.point_name, jointsCo);
        break;
        case COPY_R:
            copyPoint(motions[0].instruction.point_1_name, motions[0].instruction.point_name, cylindricalCo);
        break;
        case COPY_C:
            copyPoint(motions[0].instruction.point_1_name, motions[0].instruction.point_name, cartesianCo);
        break;

        case TRANSLATE:
            translatePoint(motions[0].instruction.point_name, motions[0].instruction.point_1_name);
        break;
        case TRANSLATE_SET:
            translatePoint(motions[0].instruction.point_name, motions[0].instruction.point_1_name, motions[0].instruction.point_2_name);
        break;

        case ADDPOINTAUTO_J:
            addPointAutomatic(jointsCo, motions[0].instruction.param1, motions[0].instruction.param2, 
            motions[0].instruction.param3, motions[0].instruction.param4, motions[0].instruction.param5);
        break;
        case ADDPOINTAUTO_R:
            addPointAutomatic(cylindricalCo, motions[0].instruction.param1, motions[0].instruction.param2, 
            motions[0].instruction.param3, motions[0].instruction.param4, motions[0].instruction.param5);
        break;
        case ADDPOINTAUTO_C:
            addPointAutomatic(cartesianCo, motions[0].instruction.param1, motions[0].instruction.param2, 
            motions[0].instruction.param3, motions[0].instruction.param4, motions[0].instruction.param5);
        break;

        case CONFIG_COM_STRIM: break;
        case CONFIG_COM_UI: break;
        case CONFIG_COM_SERIAL: break;
        case CONFIG_COM_JOG: break;
        case CONFIG_COM_CODE: break;
        
        case OFFSET_ONPOINT:
            setOffset();
        break;
        case OFFSET_INPOINT:
            setOffset(motions[0].instruction.point_name);
        break;

        case HOMEALL:
            homejoint(H1);
            homejoint(J1);
            homejoint(J2);
            homejoint(J3);
            homejoint(J4);
            homejoint(J5);
        break;
        case HOMEJ1:
            homejoint(J1);
        break;
        case HOMEJ2:
            homejoint(J2);
        break;
        case HOMEJ3:
            homejoint(J3);
        break;
        case HOMEJ5:
            homejoint(J5);
        break;
        case HOMEJ6:
            homejoint(J5);
        break;
        case HOMEH1:
            homejoint(H1);
        break;

        case CONFIG_CLEARHOME:
            homedP52 = false;
            homedP53 = false;
            homedP54 = false;
            homedP62 = false;
            homedP63 = false;
            homedP64 = false;
        break;
        }
        
        if (motions.size() == 1)
        {
            motions.clear();
        }
        else
        {
            if (motions.size() > 1)
            {
                motions.erase(motions.begin());
            }
        }
    }

    /////////
    //printf("target %f\t %f\t %f\t %f\t %f\t\n", targetPoint.k1, targetPoint.k2, targetPoint.k3, targetPoint.k4, targetPoint.k5);
    //printf("offset %f\t %f\t %f\t %f\t %f\t\n", offsetPoint.k1, offsetPoint.k2, offsetPoint.k3, offsetPoint.k4, offsetPoint.k5);
    //printf("curent %f\t %f\t %f\t %f\t %f\t\n", curentPoint.k1, curentPoint.k2, curentPoint.k3, curentPoint.k4, curentPoint.k5);
    //printf("send %f\t %f\t %f\t %f\t %f\t\n", to_send.k1, to_send.k2, to_send.k3, to_send.k4, to_send.k5);
    //printf("tar: %f\tcur: %f\tsend: %f\t\n", targetPoint.k1, curentPoint.k1, to_send.k1);
}

void MotionManager::addMotionInst(motion_inst instruction)
{   
    motions.push_back(instruction);
}

MotionManager::MotionManager()
{
    presition_mode_time = 0;
    presition_mode = false;
    presition_mode_value = 100;
    point_iterator = 0;
}

MotionManager::MotionManager(const MotionManager &) {}

void MotionManager::copyPoint(char* name_out, char* name_in){
    if (checkPoint(name_in))
    {
        if(checkPoint(name_out)){
            Coordinates a = findPoint(name_in);
            changeCoordinates(name_out, a.type, a.k1, a.k2, a.k3, a.k4, a.k5);    
        }
        else{
            Coordinates a = findPoint(name_in);
            char *temp;
            temp = new char[20];
            for(int i = 0; i < 20; i++)temp[i] = name_out[i];
            addPoint(temp, a.type, a.k1, a.k2, a.k3, a.k4, a.k5);
        }
    }
    else
    {
        ErrorLogs::err().sendPar(20, name_in);
    }
}

void MotionManager::copyPoint(char* name_out, char* name_in, type_co type){
    if (checkPoint(name_in))
    {
        if(checkPoint(name_out)){
            Coordinates b = findPoint(name_in);
            Coordinates a = b;
            a.translate(type);
            changeCoordinates(name_out, a.type, a.k1, a.k2, a.k3, a.k4, a.k5);    
        }
        else{
            Coordinates b = findPoint(name_in);
            Coordinates a = b;
            a.translate(type);
            char *temp;
            temp = new char[20];
            for(int i = 0; i < 20; i++)temp[i] = name_out[i];
            addPoint(temp, a.type, a.k1, a.k2, a.k3, a.k4, a.k5);
        }
    }
    else
    {
        ErrorLogs::err().sendPar(20, name_in);
    }
}

void MotionManager::translatePoint(char* point, char* about){
    if (checkPoint(point))
    {
        if (checkPoint(about))
        {
            Coordinates a = findPoint(point);
            Coordinates c = findPoint(about);
            Coordinates b = c;
            b.translate(a.type);
            changeCoordinates(point, a.type, a.k1+b.k1, b.k2+a.k2, b.k3+a.k3, b.k4+a.k4, b.k5+a.k5);
        }
        else{
            ErrorLogs::err().sendPar(20, about);
        }        
    }
    else{
        ErrorLogs::err().sendPar(20, point);
    }
}

void MotionManager::translatePoint(char* point, char* about, char* name_out){
    if (checkPoint(point))
    {
        if (checkPoint(about))
        {
            Coordinates a = findPoint(point);
            Coordinates c = findPoint(about);
            Coordinates b = c;
            b.translate(a.type);
            if (checkPoint(name_out)){
                changeCoordinates(name_out, a.type, a.k1+b.k1, b.k2+a.k2, b.k3+a.k3, b.k4+a.k4, b.k5+a.k5);
            }
            else{
                Coordinates a = findPoint(point);
                Coordinates c = findPoint(about);
                Coordinates b = c;
                b.translate(a.type);
                char *temp;
                temp = new char[20];
                for(int i = 0; i < 20; i++)temp[i] = name_out[i];
                addPoint(temp, a.type, a.k1+b.k1, b.k2+a.k2, b.k3+a.k3, b.k4+a.k4, b.k5+a.k5);
            }
        }
        else{
            ErrorLogs::err().sendPar(20, about);
        }        
    }
    else{
        ErrorLogs::err().sendPar(20, point);
    }
}

void MotionManager::addPoint(char *name, type_co type, float k1, float k2, float k3)
{
    if (checkPoint(name))
    {
        changeCoordinates(name, type, k1, k2, k3, 0.0, 0.0);
    }
    else
    {
        char *temp;
        temp = new char[20];
        for(int i = 0; i < 20; i++)temp[i] = name[i];
        points_key.push_back(temp);
        points_cor.push_back(new Coordinates(type, k1, k2, k3));
    }
}

void MotionManager::addPoint(char *name, type_co type, float k1, float k2, float k3, float k4, float k5)
{
    if (checkPoint(name))
    {
        changeCoordinates(name, type, k1, k2, k3, k4, k5);
    }
    else
    {
        char *temp;
        temp = new char[20];
        for(int i = 0; i < 20; i++)temp[i] = name[i];
        points_key.push_back(temp);
        points_cor.push_back(new Coordinates(type, k1, k2, k3, k4, k5));
    }
}

void MotionManager::addPoint(char *name, type_co type)
{
    if (checkPoint(name))
    {
        Coordinates a(jointsCo, curentPoint.k1, curentPoint.k2, curentPoint.k3, curentPoint.k4, curentPoint.k5);
        a.translate(type);
        changeCoordinates(name, type, a.k1, a.k2, a.k3, a.k4, a.k5);
    }
    else
    {
        char *temp;
        temp = new char[20];
        for(int i = 0; i < 20; i++)temp[i] = name[i];
        points_key.push_back(temp);
        Coordinates a(jointsCo, curentPoint.k1, curentPoint.k2, curentPoint.k3, curentPoint.k4, curentPoint.k5);
        a.translate(type);
        points_cor.push_back(new Coordinates(type, a.k1, a.k2, a.k3, a.k4, a.k5));
    }
}

void itoa(int volume, char* str){
    int retro = 0;
    int iter = 0;
    do{
        retro = volume % 10;
        volume /= 10;
        switch(retro){
            case 0: str[iter] = 48; break;
            case 1: str[iter] = 49; break;
            case 2: str[iter] = 50; break;
            case 3: str[iter] = 51; break;
            case 4: str[iter] = 52; break;
            case 6: str[iter] = 53; break;
            case 7: str[iter] = 54; break;
            case 8: str[iter] = 55; break;
            case 9: str[iter] = 56; break;
        }
        if(retro != 0){
            char temp;
            for(int i = iter; i>0; i--){
                temp = str[iter];
                str[iter] = str[iter-1];
                str[iter-1] = temp;
            }
        }
        iter ++;
    }while(volume != 0);
}

void MotionManager::addPointAutomatic(type_co type, float k1, float k2, float k3){
    char* temp;
    char* temp2;
    temp = new char[20];
    temp2 = new char[20];
    for(int i =0; i< 20; i++){temp[i] = 0;temp2[i] = 0;}
    itoa(point_iterator, temp2);
    point_iterator++;
    temp[0] = 80;
    temp[1] = 71;
    for(int i = 2; i< 20 ; i++){
        if(temp2[i-2] > 47 && temp2[i-2] < 58){
            temp[i] = temp2[i-2];
        }
        else{
            break;
        }
    }
    points_key.push_back(temp);
    points_cor.push_back(new Coordinates(type, k1, k2, k3, 0, 0));
}

void MotionManager::addPointAutomatic(type_co type, float k1, float k2, float k3, float k4, float k5){
    char* temp;
    char* temp2;
    temp = new char[20];
    temp2 = new char[20];
    for(int i =0; i< 20; i++){temp[i] = 0;temp2[i] = 0;}
    itoa(point_iterator, temp2);
    point_iterator++;
    temp[0] = 80;
    temp[1] = 71;
    for(int i = 2; i< 20 ; i++){
        if(temp2[i-2] > 47 && temp2[i-2] < 58){
            temp[i] = temp2[i-2];
        }
        else{
            break;
        }
    }
    points_key.push_back(temp);
    points_cor.push_back(new Coordinates(type, k1, k2, k3, k4, k5));
}

void MotionManager::addPointAutomatic(type_co type){
    char* temp;
    char* temp2;
    temp = new char[20];
    temp2 = new char[20];
    for(int i =0; i< 20; i++){temp[i] = 0;temp2[i] = 0;}
    itoa(point_iterator, temp2);
    point_iterator++;
    temp[0] = 80;
    temp[1] = 71;
    for(int i = 2; i< 20 ; i++){
        if(temp2[i-2] > 47 && temp2[i-2] < 58){
            temp[i] = temp2[i-2];
        }
        else{
            break;
        }
    }
    points_key.push_back(temp);
    points_cor.push_back(new Coordinates(type, 0, 0, 0, 0, 0));
}

void MotionManager::clearPoints()
{
    point_iterator = 0;
    points_key.clear();
    points_cor.clear();
}

bool com(char *tem1, char *tem2)
{
    for (int i = 0; i < 20; i++)
    {
        if (tem1[i] != tem2[i] && tem1[1] >= 48 && tem2[1] >= 48 && tem1[i] <= 90 && tem2[i] <= 90)
            return false;
        if (tem1[i] == 0 && tem2[i] == 0)
            break;
    }
    return true;
}

Coordinates MotionManager::findPoint(char *name)
{
    for (unsigned int i = 0; i < points_key.size(); i++)
    {
        if (com(name, points_key[i]))
        {
            return *points_cor[i];
        }
    }
    Coordinates a;
    return a;
}

bool MotionManager::checkPoint(char *name)
{
    for (unsigned int i = 0; i < points_key.size(); i++)
    {
        if (com(name, points_key[i]))
        {
            return true;
        }
    }
    return false;
}

void MotionManager::changeCoordinates(char *name, type_co t_type, float t_k1, float t_k2, float t_k3, float t_k4, float t_k5)
{
    for (unsigned int i = 0; i < points_key.size(); i++)
    {
        if (com(name, points_key[i]))
        {
            points_cor[i]->type = t_type;
            points_cor[i]->k1 = t_k1;
            points_cor[i]->k2 = t_k2;
            points_cor[i]->k3 = t_k3;
            points_cor[i]->k4 = t_k4;
            points_cor[i]->k5 = t_k5;
        }
    }
}

void MotionManager::show(char *name)
{
    if (checkPoint(name))
    {
        show(name, findPoint(name).type);
    }
    else
    {
        ErrorLogs::err().sendPar(20, name);
    }
}

void MotionManager::show(char* name, type_co type)
{
    if (checkPoint(name))
    {
        Coordinates a;
        a = findPoint(name);
        a.translate(type);
        switch (type)
        {
        case cartesianCo:
            Serial.printf("Point %s : x: %f, y: %f, z: %f, A: %f, B: %f", name, a.k1, a.k2, a.k3, a.k4, a.k5);
            break;
        case cylindricalCo:
            Serial.printf("Point %s : r: %f, h: %f, alpha: %f, A: %f, B: %f", name, a.k1, a.k2, a.k3, a.k4, a.k5);
            break;
        case jointsCo:
            Serial.printf("Point %s : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f", name, a.k1, a.k2, a.k3, a.k4, a.k5);
            break;
        case none:
            Serial.printf("Point %s : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f", name, a.k1, a.k2, a.k3, a.k4, a.k5);
            break;
        }
        if (a.type == cartesianCo)
            Serial.printf(" type: cartesian\n");
        if (a.type == cylindricalCo)
            Serial.printf(" type: cylindrical\n");
        if (a.type == jointsCo)
            Serial.printf(" type: joints\n");
    }
    else
    {
        ErrorLogs::err().sendPar(20, name);
    }
}

void MotionManager::show(Coordinates point){
    switch (point.type)
        {
        case cartesianCo:
            Serial.printf("Point : x: %f, y: %f, z: %f, A: %f, B: %f", point.k1, point.k2, point.k3, point.k4, point.k5);
            break;
        case cylindricalCo:
            Serial.printf("Point : r: %f, h: %f, alpha: %f, A: %f, B: %f", point.k1, point.k2, point.k3, point.k4, point.k5);
            break;
        case jointsCo:
            Serial.printf("Point : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f", point.k1, point.k2, point.k3, point.k4, point.k5);
            break;
        case none:
            Serial.printf("Point : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f", point.k1, point.k2, point.k3, point.k4, point.k5);
            break;
        }
        if (point.type == cartesianCo)
            Serial.printf(" type: cartesian\n");
        if (point.type == cylindricalCo)
            Serial.printf(" type: cylindrical\n");
        if (point.type == jointsCo)
            Serial.printf(" type: joints\n");
}

void MotionManager::showAll()
{
    for (size_t i = 0; i < points_cor.size(); i++)
    {
        switch (points_cor[i]->type)
        {
        case cartesianCo:
            Serial.printf("Point %s : x: %f, y: %f, z: %f, A: %f, B: %f", points_key[i], points_cor[i]->k1, points_cor[i]->k2, points_cor[i]->k3, points_cor[i]->k4, points_cor[i]->k5);
            break;
        case cylindricalCo:
            Serial.printf("Point %s : r: %f, h: %f, alpha: %f, A: %f, B: %f", points_key[i], points_cor[i]->k1, points_cor[i]->k2, points_cor[i]->k3, points_cor[i]->k4, points_cor[i]->k5);
            break;
        case jointsCo:
            Serial.printf("Point %s : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f", points_key[i], points_cor[i]->k1, points_cor[i]->k2, points_cor[i]->k3, points_cor[i]->k4, points_cor[i]->k5);
            break;
        case none:
            Serial.printf("Point %s : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f", points_key[i], points_cor[i]->k1, points_cor[i]->k2, points_cor[i]->k3, points_cor[i]->k4, points_cor[i]->k5);
            break;
        }
        if (points_cor[i]->type == cartesianCo)
            Serial.printf(" type: cartesian\n");
        if (points_cor[i]->type == cylindricalCo)
            Serial.printf(" type: cylindrical\n");
        if (points_cor[i]->type == jointsCo)
            Serial.printf(" type: joints\n");
    }
}

void MotionManager::showCurrent()
{
    Serial.printf("Current Point : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f\n", curentPoint.k1, curentPoint.k2, curentPoint.k3, curentPoint.k4, curentPoint.k5);
}

void MotionManager::showCurrent(type_co t_type){
    Coordinates a = curentPoint;
    a.translate(t_type);
    Serial.printf("Current Point : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f\n", a.k1, a.k2, a.k3, a.k4, a.k5);
}

void MotionManager::setOffset(char *point)
{
    Coordinates a;
    a = findPoint(point);
    motorManagerSetOffsetDef(a);
}

void MotionManager::setOffset()
{
    Coordinates a;
    a.k1 = current[1];
    a.k2 = current[2];
    a.k3 = current[3];
    a.k4 = current[5];
    a.k5 = current[6];
    motorManagerSetOffsetDef(a);
}

void MotionManager::griperOpen()
{
    setGripperValume(15);
}

void MotionManager::griperClose()
{
    setGripperValume(-15);
}

void MotionManager::griperStop()
{
    setGripperValume(0);
}

void MotionManager::setTarget(float t_k1, float t_k2, float t_k3, float t_k4, float t_k5)
{
    targetPoint.k1 = t_k1;
    targetPoint.k2 = t_k2;
    targetPoint.k3 = t_k3;
    targetPoint.k4 = t_k4;
    targetPoint.k5 = t_k5;
}

void MotionManager::setTarget(Coordinates *point)
{
    targetPoint.k1 = point->k1;
    targetPoint.k2 = point->k2;
    targetPoint.k3 = point->k3;
    targetPoint.k4 = point->k4;
    targetPoint.k5 = point->k5;
}

float MotionManager::getTarget(int t_joint)
{
    switch (t_joint)
    {
    case 1:
        return targetPoint.k1;
        break;
    case 2:
        return targetPoint.k2;
        break;
    case 3:
        return targetPoint.k3;
        break;
    case 4:
        return targetPoint.k4;
        break;
    case 5:
        return targetPoint.k5;
        break;
    }
    return 0;
}

void MotionManager::setPresitionMode(bool precysion, float t_volume, int t_time)
{
    presition_mode = precysion;
    presition_mode_time = abs(t_time);
    presition_mode_value = abs(t_volume);
}

void MotionManager::waitForReachingTarget()
{
    if (presition_mode)
    {
        float dis;
        do
        {
            Coordinates a(jointsCo, current[1], current[2], current[3], current[5], current[6]);
            dis = pointToPointDistanceJointMax(targetPoint, a);
            sys.delay(10);
        } while (dis > presition_mode_value);
        if (presition_mode_time > 0)
            sys.delay(presition_mode_time);
    }
    else
    {
        if (presition_mode_time > 0)
            sys.delay(presition_mode_time);
    }
}

bool MotionManager::instruction(instruction_code instruction){
    motion_inst a;
    a.instruction.comand = instruction.comand;
    a.instruction.param1 = instruction.param1;
    a.instruction.param2 = instruction.param2;
    a.instruction.param3 = instruction.param3;
    a.instruction.param4 = instruction.param4;
    a.instruction.param5 = instruction.param5;
    for(int i=0; i< 20; i++){
        a.instruction.point_name = instruction.point_name;
        a.instruction.point_1_name = instruction.point_1_name;
        a.instruction.point_2_name = instruction.point_2_name;
    }
    if(instruction.point_name != ""){
        a.point = findPoint(a.instruction.point_name);
    }
    addMotionInst(a);
    return true;
}

Coordinates MotionManager::getPoint(char* name){
    char* temp;
    temp = new char[20];
    for(int i=0; i<20; i++) temp[i]=name[i];
    if(checkPoint(temp)){
        return findPoint(temp);
    }
    else{
        ErrorLogs::err().sendPar(20, temp);
        Coordinates a;
        return a;
    }
}

Coordinates MotionManager::getPoint(char* name, type_co type){
    char* temp;
    temp = new char[20];
    for(int i=0; i<20; i++)temp[i]=name[i];
    if(checkPoint(temp)){
        Coordinates a;
        a = findPoint(temp);
        a.translate(type);
        return a;
    }
    else{
        ErrorLogs::err().sendPar(20, temp);
        Coordinates a;
        return a;
    }
}
