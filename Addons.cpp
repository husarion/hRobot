#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "Addons.h"
#include "ErrorLog.h"
#include "GeoMath.h"

// ver 6.

float saturateFloat(float val, float bord){ // limits float val symetricly between -bord and bord.
    if (val<-bord){ val=-bord; } else if (val>bord) val=bord;
    return val;
}
float saturateFloatUnsym(float val, float max, float min){ // limits float val unsymetricly between -min and max.
    if (val<-min){ val=-min; } else if (val>max) val=max;
    return val;
}
float thresholdFloat(float val, float th){ // cuts to zero if in threshold
    if (val>-th && val<0){ val=0; } else if (val<th && val>0) val=0;
    return val;
}
float circleFloat(float val){  // sub or add value to stay between -180 and 180
    if (val>=180){ val-=360; } else if (val<-180) val+=360;
    return val;
}

float deg2rad(float deg){
    return deg/180*3.14159265;
}

float rad2deg(float rad){
    return rad/3.14159265*180;
}

float sq(float val){
    return pow(val, 2);
}

void erco(int code){
    ErrorLogs::Err().sendPar(17, code);
    //printf("-----------------------> Code: %d\r\n",code);
}

Coordinates::Coordinates(const Coordinates & t){
    this->k1 = t.k1;
    this->k2 = t.k2;
    this->k3 = t.k3;
    this->k4 = t.k4;
    this->k5 = t.k5;
    this->type = t.type;
}

Coordinates::Coordinates(){}

Coordinates::Coordinates(typeCo type, float k1, float k2, float k3){
    this->k1 = k1;
    this->k2 = k2;
    this->k3 = k3;
    this->type = type;
}

Coordinates::Coordinates(typeCo type, float k1, float k2, float k3, float k4, float k5){
    this->k1 = k1;
    this->k2 = k2;
    this->k3 = k3;
    this->k4 = k4;
    this->k5 = k5;
    this->type = type;
}

void Coordinates::Translate(typeCo t_type){
    switch(type){
        case cartesianCo :
        if(t_type == cylindricalCo){*this = cartes2cylin(*this);}
        if(t_type == jointsCo){Coordinates c(jointsCo, 0, 0, 0); *this = cartes2joints(*this, c, 0.01);}
        break;
        case cylindricalCo :
        if(t_type == cartesianCo){*this = cylin2cartes(*this);}
        if(t_type == jointsCo){Coordinates c(jointsCo, 0, 0, 0); *this = cylin2joints(*this, c, 0.01);}
        break;
        case jointsCo:
        if(t_type == cartesianCo){*this = joints2cartes(*this);}
        if(t_type == cylindricalCo){*this = joints2cylin(*this);}
        break;
    }
}