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

enum movment_mode{Normal, Cartesian};

class MotionManager{
private:
    float* target;
    float* curent;
    std::vector<char*> points_key;
    std::vector<Coordinates*> points_cor;
    
    MotionManager();
    MotionManager(const MotionManager&);
public:
    ~MotionManager();
    static MotionManager & get(){
        static MotionManager singleton;
        return singleton;
    }
    
    int Move(movment_mode mode, char* point_name);
    void addPoint(char* name, typeCo type, float k1, float k2, float k3);
    void addPoint(char* name, typeCo type, float k1, float k2, float k3, float k4, float k5);
    void addPoint(char* name, typeCo type);
    void clearPoints();
    Coordinates findPoint(char* name);
    bool checkPoint(char* name);
    void show(char* name);
    void show(char* name, typeCo type);
};

#endif