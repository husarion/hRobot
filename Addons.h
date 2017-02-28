#include <cstddef>
#include <cstdint>
#include "hFramework.h"

// ver 5.

#ifndef __Addons__
#define __Addons__

enum typeCo
{
    cartesianCo,
    cylindricalCo,
    jointsCo
};

class Coordinates
{
  private:
  public:
    typeCo type;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
    Coordinates(const Coordinates &t);
    Coordinates();
    Coordinates(typeCo type, float k1, float k2, float k3);
    Coordinates(typeCo type, float k1, float k2, float k3, float k4, float k5);
    void Translate(typeCo t_type);
};

float pointToPointDistance(Coordinates from, Coordinates to);
float pointToPointDistanceJointMax(Coordinates from, Coordinates to);

float saturateFloat(float val, float bord);
float saturateFloatUnsym(float val, float max, float min);
float thresholdFloat(float val, float th);
float circleFloat(float val);
float deg2rad(float deg);
float rad2deg(float rad);
float sq(float val);
void erco(int code);

#endif //__Addons__