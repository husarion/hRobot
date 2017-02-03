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

MotionManager::MotionManager(){
    target = new float[5];
    curent = new float[5];
    for(int i =0; i<5; i++){
        target[i]=0.0;
        curent[i]=0.0;
    }
}

MotionManager::MotionManager(const MotionManager&){}

MotionManager::~MotionManager(){
    delete(target);
    delete(curent);
}

int MotionManager::Move(movment_mode mode, char* point_name){
    switch(mode){
        case Normal:
            if(checkPoint(point_name)){
                Coordinates a;
                a = findPoint(point_name);
                a.Translate(jointsCo);
                target[0]=a.k1;
                target[1]=a.k2;
                target[2]=a.k3;
                target[3]=a.k4;
                target[4]=a.k5;
            }
            else{
                ErrorLogs::Err().sendPar(18, point_name);
            }
        break;
        case Cartesian:
            if(checkPoint(point_name)){
                Coordinates a;
                a = findPoint(point_name);
                a.Translate(jointsCo);
                target[0]=a.k1;
                target[1]=a.k2;
                target[2]=a.k3;
                target[3]=a.k4;
                target[4]=a.k5;
            }
            else{
                ErrorLogs::Err().sendPar(18, point_name);
            }
        break;
    }
}

void MotionManager::addPoint(char* name, typeCo type, float k1, float k2, float k3){
    if(checkPoint(name)){
        ErrorLogs::Err().sendPar(19, name);
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
        ErrorLogs::Err().sendPar(19, name);
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
        ErrorLogs::Err().sendPar(19, name);
    }
    else{
        char* temp;
        temp = new char[20];
        for(int i =0; i<20; i++)temp[i]=name[i];
        points_key.push_back(temp);
        points_cor.push_back(new Coordinates(type, curent[0], curent[1], curent[2], curent[3], curent[4]));
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
    for(int i=0; i< points_key.size(); i++){
        if(com(name, points_key[i])){
            return *points_cor[i];
        }
    }
}

bool MotionManager::checkPoint(char* name){
    for(int i=0; i< points_key.size(); i++){
        if(com(name, points_key[i])){
            return true;
        }
    }
    return false;
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
                printf("Point %s : x: %f, y: %f, z: %f, A: %f, B: %f", name, a.k1, a.k2, a.k3, a.k4, a.k5);
                break;
            case cylindricalCo:
                printf("Point %s : r: %f, h: %f, alpha: %f, A: %f, B: %f", name, a.k1, a.k2, a.k3, a.k4, a.k5);
                break;
            case jointsCo:
                printf("Point %s : j1: %f, j2: %f, j3: %f, j4: %f, j5: %f", name, a.k1, a.k2, a.k3, a.k4, a.k5);
                break;
        }
        if(a.type == cartesianCo)printf(" type: cartesian\n");
        if(a.type == cylindricalCo)printf(" type: cylindrical\n");
        if(a.type == jointsCo)printf(" type: joints\n");
    }
    else{
        ErrorLogs::Err().sendPar(20, name);
    }
}
