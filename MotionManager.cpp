// hRobot_0_05_09 edited by mzabinski94@gmail.com
// from hRobot_0_05_06

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <cstdio>
#include <stdio.h>
#include <string.h>
#include <cstring>
#include <map>

#include "hFramework.h"
#include "MotionManager.h"
#include "Addons.h"
#include "ErrorLog.h"
#include "MotorManager.h"

extern float current[9];
extern float target[9];

const int time_motion_task = 100;
float time_iteration = 100;
int step_mul = 10;

Coordinates to_send;

void MotionTask(){
    
    for(;;){
        sys.delay(time_motion_task);
        MotionManager::get().update();
    }
}

void MotionManager::MoveCartesianInter(){
    Serial.printf("30\n");
}//TODO:

void MotionManager::MoveCartesianNorm(){
    if(targetPoint.type == cartesianCo){
        MotorManagerUpdateTargetDef(targetPoint);
    }
    else{
        targetPoint.Translate(cartesianCo);
        MotorManagerUpdateTargetDef(targetPoint);
    }
}

void MotionManager::MoveJointNorm(){
    if(targetPoint.type == jointsCo){
        MotorManagerUpdateTargetDef(targetPoint);
    }
    else{
        targetPoint.Translate(jointsCo);
        MotorManagerUpdateTargetDef(targetPoint);
    }
}

void MotionManager::MoveJointInter(){
    /////////
    float internal_speed = 5;//5mm/s
    float ovrd = 1;//100% OVRD
    /////////
    
    float dis = pointToPointDistance(targetPoint, curentPoint);
    int steps = (int)(dis/(internal_speed*ovrd*10))*step_mul;
    
    
    float j1_iter_step = (targetPoint.k1 - curentPoint.k1) / steps;
    float j2_iter_step = (targetPoint.k2 - curentPoint.k2) / steps;
    float j3_iter_step = (targetPoint.k3 - curentPoint.k3) / steps;
    float j5_iter_step = (targetPoint.k4 - curentPoint.k4) / steps;
    float j6_iter_step = (targetPoint.k5 - curentPoint.k5) / steps;
    to_send = curentPoint;
    
    for(int i =0; i<steps; i++){
        to_send.k1 += j1_iter_step;
        to_send.k2 += j2_iter_step;
        to_send.k3 += j3_iter_step;
        to_send.k4 += j5_iter_step;
        to_send.k5 += j6_iter_step;
        
        MotorManagerUpdateTargetDef(to_send);
        
        sys.delay(time_iteration); 
    }
}

void MotionManager::update(){
    //curentPoint.k1 = current[1];
    //curentPoint.k2 = current[2];
    //curentPoint.k3 = current[3];
    //curentPoint.k4 = current[5];
    //curentPoint.k5 = current[6];
    curentPoint = targetPoint;
    
    if(motions.size()>0){
        switch(motions[0].type){
            case cartesianInter:
                targetPoint = motions[0].point;
                MoveCartesianInter();
            break;
            case cartesianNorm:
                targetPoint = motions[0].point;
                MoveCartesianNorm();
            break;
            case jointsInter:
                targetPoint = motions[0].point;
                MoveJointInter();
            break;
            case jointsNorm:
                targetPoint = motions[0].point;
                MoveJointNorm();
            break;
            case Delay:
                sys.delay(motions[0].point.k1);
            break;
        }
        if(motions.size()==1){
            motions.clear();
        }
        else{
            if(motions.size()>1){
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

void MotionManager::addMotionInst(Coordinates point, motion_type movment_type){
    motion_inst a;
    a.point = point;
    a.type = movment_type;
    motions.push_back(a);
}

void MotionManager::addMotionInst(float t_k1, float t_k2, float t_k3, float t_k4, float t_k5, motion_type movment_type){
    Coordinates t_point(jointsCo, t_k1, t_k2, t_k3, t_k4, t_k5);
    motion_inst a;
    a.point = t_point;
    a.type = movment_type;
    motions.push_back(a);
}

MotionManager::MotionManager(){}

MotionManager::MotionManager(const MotionManager&){}

int MotionManager::Move(motion_type mode, char* point_name){
    switch(mode){
        case jointsInter:
            if(checkPoint(point_name)){
                Coordinates a;
                a = findPoint(point_name);
                a.Translate(jointsCo);
                addMotionInst(a, jointsInter);
            }
            else{
                ErrorLogs::Err().sendPar(18, point_name);
            }
        break;
        case cartesianInter:
            if(checkPoint(point_name)){
                Coordinates a;
                a = findPoint(point_name);
                a.Translate(jointsCo);
                addMotionInst(a, cartesianInter);
            }
            else{
                ErrorLogs::Err().sendPar(18, point_name);
            }
        break;
    }
    return 26;
}

void MotionManager::addPoint(char* name, typeCo type, float k1, float k2, float k3){
    if(checkPoint(name)){
        changeCoordinates(name, type, k1, k2, k3, 0.0, 0.0);
    }
    else{
        char* temp;
        temp = new char[20];
        for(int i =0; i<20; i++)temp[i]=name[i];
        points_key.push_back(temp);
        points_cor.push_back(new Coordinates(type, k1, k2, k3));
    }
}

void MotionManager::addPoint(char* name, typeCo type, float k1, float k2, float k3, float k4, float k5){
    if(checkPoint(name)){
        changeCoordinates(name, type, k1, k2, k3, k4, k5);
    }
    else{
        char* temp;
        temp = new char[20];
        for(int i =0; i<20; i++)temp[i]=name[i];
        points_key.push_back(temp);
        points_cor.push_back(new Coordinates(type, k1, k2, k3, k4, k5));
    }
}

void MotionManager::addPoint(char* name, typeCo type){
    if(checkPoint(name)){
        changeCoordinates(name, type, curentPoint.k1, curentPoint.k2, curentPoint.k3, curentPoint.k4, curentPoint.k5);
    }
    else{
        char* temp;
        temp = new char[20];
        for(int i =0; i<20; i++)temp[i]=name[i];
        points_key.push_back(temp);
        points_cor.push_back(new Coordinates(type, curentPoint.k1, curentPoint.k2, curentPoint.k3, curentPoint.k4, curentPoint.k5));
    }
}

void MotionManager::clearPoints(){
    points_key.clear();
    points_cor.clear();
}

bool com(char* tem1, char* tem2){
    for(int i = 0; i<20; i++){
        if(tem1[i]!=tem2[i] && tem1[1]>=48 && tem2[1]>=48 && tem1[i]<=90 && tem2[i]<=90)
        return false;
    }
    return true;
}

Coordinates MotionManager::findPoint(char* name){
    for(unsigned int i=0; i< points_key.size(); i++){
        if(com(name, points_key[i])){
            return *points_cor[i];
        }
    }
    Coordinates a;
    return a;
}

bool MotionManager::checkPoint(char* name){
    for(unsigned int i=0; i< points_key.size(); i++){
        if(com(name, points_key[i])){
            return true;
        }
    }
    return false;
}

void MotionManager::changeCoordinates(char* name, typeCo t_type, float t_k1, float t_k2, float t_k3, float t_k4, float t_k5){
    for(unsigned int i=0; i< points_key.size(); i++){
        if(com(name, points_key[i])){
            points_cor[i]->type=t_type;
            points_cor[i]->k1=t_k1;
            points_cor[i]->k2=t_k2;
            points_cor[i]->k3=t_k3;
            points_cor[i]->k4=t_k4;
            points_cor[i]->k5=t_k5;
        }
    }
}

void MotionManager::show(char* name){
    if(checkPoint(name)){
        show(name, findPoint(name).type);
    }
    else{
        ErrorLogs::Err().sendPar(20, name);
    }
}

void MotionManager::show(char* name, typeCo type){
    if(checkPoint(name)){
        Coordinates a;
        a = findPoint(name);
        a.Translate(type);
        switch (type){
            case cartesianCo:
                Serial.printf("Point %s : x: %f, y: %f, z: %f, A: %f, B: %f", name, a.k1, a.k2, a.k3, a.k4, a.k5);
                break;
            case cylindricalCo:
                Serial.printf("Point %s : r: %f, h: %f, alpha: %f, A: %f, B: %f", name, a.k1, a.k2, a.k3, a.k4, a.k5);
                break;
            case jointsCo:
                Serial.printf("Point %s : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f", name, a.k1, a.k2, a.k3, a.k4, a.k5);
                break;
        }
        if(a.type == cartesianCo)Serial.printf(" type: cartesian\n");
        if(a.type == cylindricalCo)Serial.printf(" type: cylindrical\n");
        if(a.type == jointsCo)Serial.printf(" type: joints\n");
    }
    else{
        ErrorLogs::Err().sendPar(20, name);
    }
}

void MotionManager::showAll(){
    for(size_t i = 0; i<points_cor.size(); i++){
        switch (points_cor[i]->type){
            case cartesianCo:
                Serial.printf("Point %s : x: %f, y: %f, z: %f, A: %f, B: %f", points_key[i], points_cor[i]->k1, points_cor[i]->k2, points_cor[i]->k3, points_cor[i]->k4, points_cor[i]->k5);
                break;
            case cylindricalCo:
                Serial.printf("Point %s : r: %f, h: %f, alpha: %f, A: %f, B: %f", points_key[i], points_cor[i]->k1, points_cor[i]->k2, points_cor[i]->k3, points_cor[i]->k4, points_cor[i]->k5);
                break;
            case jointsCo:
                Serial.printf("Point %s : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f", points_key[i], points_cor[i]->k1, points_cor[i]->k2, points_cor[i]->k3, points_cor[i]->k4, points_cor[i]->k5);
                break;
        }
        if(points_cor[i]->type == cartesianCo)Serial.printf(" type: cartesian\n");
        if(points_cor[i]->type == cylindricalCo)Serial.printf(" type: cylindrical\n");
        if(points_cor[i]->type == jointsCo)Serial.printf(" type: joints\n");
    }
}

void MotionManager::showCurrent(){
    Serial.printf("Current Point : j1: %f, j2: %f, j3: %f, j5: %f, j6: %f\n", curentPoint.k1, curentPoint.k2, curentPoint.k3, curentPoint.k4, curentPoint.k5);
}

void MotionManager::setOffset(char* point){
    Coordinates a;
    a = findPoint(point);
    MotorManagerSetOffsetDef(a);
}

void MotionManager::setOffset(){
    Coordinates a;
    a.k1 = current[1];
    a.k2 = current[2];
    a.k3 = current[3];
    a.k4 = current[5];
    a.k5 = current[6];
    MotorManagerSetOffsetDef(a);
}

void MotionManager::GriperOpen(){
    setGripperValume(15);
}

void MotionManager::GriperClose(){
    setGripperValume(-15);
}

void MotionManager::GriperStop(){
    setGripperValume(0);
}

void MotionManager::setTarget(float t_k1, float t_k2, float t_k3, float t_k4, float t_k5){
    targetPoint.k1 = t_k1;
    targetPoint.k2 = t_k2;
    targetPoint.k3 = t_k3;
    targetPoint.k4 = t_k4;
    targetPoint.k5 = t_k5;
}

void MotionManager::setTarget(Coordinates* point){
    targetPoint.k1 = point->k1;
    targetPoint.k2 = point->k2;
    targetPoint.k3 = point->k3;
    targetPoint.k4 = point->k4;
    targetPoint.k5 = point->k5;
}

float MotionManager::getTarget(int t_joint){
    switch(t_joint){
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

void MotionManager::setPrecysionMode(int precysion){
    precysion_mode = abs(precysion);
}

void MotionManager::waitForReachingTarget(){
    if(precysion_mode == 0){}
    else{
        sys.delay(precysion_mode);
    }
}