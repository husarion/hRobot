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

enum instruction_input_type{UI, SERIAL, JOG, CODE};

enum instruction_command{
                        NOCOMMAND,
                        SET_J, SET_C, SET_R, SET_HERE_J, SET_HERE_C, SET_HERE_R,
                        SHOWALL,
                        SHOWCURRENT, SHOWCURRENT_J, SHOWCURRENT_C, SHOWCURRENT_R,
                        SHOW, SHOW_J, SHOW_C, SHOW_R,
                        MOVE,
                        MOVES,
                        DELAY,
                        H1OPEN, H1CLOSE, H1STOP,
                        RESETPOINTS,
                        PRECYSION_ON, PRECYSION_OFF,
                        CONFIG_COM_STRIM, CONFIG_COM_UI, CONFIG_COM_SERIAL, CONFIG_COM_JOG, CONFIG_COM_CODE,
                        OFFSET_ONPOINT, OFFSET_INPOINT
                        };

struct instruction_code{
    instruction_command comand;
    const char* point_name;
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
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
float sind(float deg);
float cosd(float deg);

#endif //__Addons__