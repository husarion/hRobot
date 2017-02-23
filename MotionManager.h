// hRobot_0_05_09 edited by mzabinski94@gmail.com
// from hRobot_0_05_06

#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include <iostream>
#include <cstdio>
#include <stdio.h>
#include <string.h>
#include <map>
#include <vector>

#include "hFramework.h"
#include "Addons.h"

#ifndef HROBOTMOTIONMANAGER
#define HROBOTMOTIONMANAGER

enum joint_names{J1, J2, J3, J4, J5, J6};
enum motion_type{cartesianInter, cartesianNorm, jointsInter, jointsNorm, Delay};

struct motion_inst{
    Coordinates point;
    motion_type type;
};

void MotionTask();

class MotionManager{
private:
    Coordinates targetPoint;
    Coordinates curentPoint;
    Coordinates offsetPoint;
    std::vector<char*> points_key;
    std::vector<Coordinates*> points_cor;
    std::vector<motion_inst> motions;
    int precysion_mode;
    
    bool checkRangeJ(Coordinates* point);
    void waitForReachingTarget();
    
    MotionManager();
    MotionManager(const MotionManager&);
public:
    static MotionManager & get(){
        static MotionManager singleton;
        return singleton;
    }
    
    int Move(motion_type mode, char* point_name);
    void addPoint(char* name, typeCo type, float k1, float k2, float k3);
    void addPoint(char* name, typeCo type, float k1, float k2, float k3, float k4, float k5);
    void addPoint(char* name, typeCo type);
    void clearPoints();
    Coordinates findPoint(char* name);
    void changeCoordinates(char* name, typeCo t_type, float t_k1, float t_k2, float t_k3, float t_k4, float t_k5);
    bool checkPoint(char* name);
    void show(char* name);
    void show(char* name, typeCo type);
    void showAll();
    void showCurrent();
    void setOffset(float t_k1, float t_k2, float t_k3);
    void setOffset(char* point);
    void setOffset();
    bool checkRange(Coordinates* point);
    void setMinMax(joint_names joint, float t_min, float t_max);
    void setTarget(float t_k1, float t_k2, float t_k3, float t_k4, float t_k5);
    void setTarget(Coordinates* point);
    float getTarget(int t_joint);
    void setPrecysionMode(int precysion);

    void GriperOpen();
    void GriperClose();
    void GriperStop();
    
    void update();
    void MoveJointInter();
    void MoveJointNorm();
    void MoveCartesianInter();
    void MoveCartesianNorm();
    
    void addMotionInst(Coordinates point, motion_type movment_type);
    void addMotionInst(float t_k1, float t_k2, float t_k3, float t_k4, float t_k5, motion_type movment_type);
};

#endif