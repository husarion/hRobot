// hRobot_0_05_09 edited by mzabinski94@gmail.com
// from hRobot_0_05_06

#include <cstddef>
#include <cstdint>
#include "hCloudClient.h"
#include <iostream>
#include <cstdio>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "hFramework.h"
#include "Addons.h"

#ifndef HROBOTMOTIONMANAGER
#define HROBOTMOTIONMANAGER

struct motion_inst
{
    instruction_code instruction;
    Coordinates point;
};

void MotionTask();

class MotionManager
{
  private:
    Coordinates targetPoint;
    Coordinates curentPoint;
    Coordinates offsetPoint;
    std::vector<char *> points_key;
    std::vector<Coordinates *> points_cor;
    std::vector<motion_inst> motions;
    bool presition_mode;
    int presition_mode_time;
    float presition_mode_value;

    float ovrd_speed;

    bool checkRangeJ(Coordinates *point);
    void waitForReachingTarget();

    MotionManager();
    MotionManager(const MotionManager &);
    
    Coordinates findPoint(char *name);
    bool checkPoint(char *name);
    bool checkRange(Coordinates *point);
    
    void showAll();
    void showCurrent();
    void showCurrent(type_co t_type);
    void setOffset(float t_k1, float t_k2, float t_k3);
    void setOffset(char *point);
    void setOffset();
    void setPresitionMode(bool presition, float t_volume, int t_time);
    void addPoint(char *name, type_co type, float k1, float k2, float k3);
    void addPoint(char *name, type_co type, float k1, float k2, float k3, float k4, float k5);
    void addPoint(char *name, type_co type);
    void copyPoint(char* name_out, char* name_in);
    void copyPoint(char* name_out, char* name_in, type_co type);
    void clearPoints();
    void changeCoordinates(char *name, type_co t_type, float t_k1, float t_k2, float t_k3, float t_k4, float t_k5);
    void addMotionInst(motion_inst instruction);
    void moveJointInter();
    void moveJointNorm();
    void moveCartesianInter();
    void moveCartesianNorm();
    void griperOpen();
    void griperClose();
    void griperStop();

  public:
    static MotionManager & get()
    {
        static MotionManager singleton;
        return singleton;
    }
    
    void show(char* name);
    void show(char* name, type_co type);
    void show(Coordinates point);
    
    void setMinMax(joint_names joint, float t_min, float t_max);
    void setTarget(float t_k1, float t_k2, float t_k3, float t_k4, float t_k5);
    void setTarget(Coordinates *point);
    float getTarget(int t_joint);
        
    bool instruction(instruction_code instruction);

    void update();
};

#endif