#include <cstddef>
#include <cstdint>
#include "hFramework.h"

// ver 5.

#ifndef __Addons__
#define __Addons__

enum type_co
{
    cartesianCo,
    cylindricalCo,
    jointsCo,
    none
};

enum joint_names
{
    J1,
    J2,
    J3,
    J4,
    J5,
    J6
};

enum instruction_input_type{UI, SERIAL, JOG, CODE, STREAM};

enum instruction_command{
                        NOCOMMAND,
                        SET_J, SET_C, SET_R, SET_HERE_J, SET_HERE_C, SET_HERE_R,
                        ADDPOINTAUTO_J, ADDPOINTAUTO_R, ADDPOINTAUTO_C,
                        SHOWALL,
                        SHOWCURRENT, SHOWCURRENT_J, SHOWCURRENT_C, SHOWCURRENT_R,
                        SHOW, SHOW_J, SHOW_C, SHOW_R,
                        MOVE, MOVE_D, MOVE_JI, MOVE_CI, MOVE_JN, MOVE_CN,
                        MOVES, MOVES_D,
                        JOG_J, JOG_R, JOG_C, JOG_X, JOG_Y, JOG_Z, JOG_J1, JOG_J2, JOG_J3, JOG_J5, JOG_J6,
                        DELAY,
                        H1OPEN, H1CLOSE, H1STOP,
                        RESETPOINTS,
                        PRECYSION_ON, PRECYSION_OFF,
                        SPEED,
                        COPY, COPY_J, COPY_R, COPY_C,
                        TRANSLATE_SET, TRANSLATE,
                        CONFIG_COM_STRIM, CONFIG_COM_UI, CONFIG_COM_SERIAL, CONFIG_COM_JOG, CONFIG_COM_CODE,
                        OFFSET_ONPOINT, OFFSET_INPOINT
                        };

struct instruction_code{
    instruction_command comand;
    char* point_name;
    char* point_1_name;
    char* point_2_name;
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
    type_co type;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
    Coordinates(const Coordinates &t);
    Coordinates();
    Coordinates(type_co type, float k1, float k2, float k3);
    Coordinates(type_co type, float k1, float k2, float k3, float k4, float k5);
    void translate(type_co t_type);
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