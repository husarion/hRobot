#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "Addons.h"
#include "ErrorLog.h"
#include "GeoMath.h"

// ver 7.
float sind(float deg){
    return sin(deg2rad(deg));
}

float cosd(float deg){
    return cos(deg2rad(deg));
}

float saturateFloat(float val, float bord){ // limits float val symetricly between -bord and bord.
    if (val<-bord){ val=-bord; } else if (val>bord) val=bord;
    return val;
}
float saturateFloatUnsym(float val, float max, float min){ // limits float val unsymetricly between -min and max.
    if (val<min){ val=min; } else if (val>max) val=max;
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
    return val*val;

}

void erco(int code){
    ErrorLogs::err().sendPar(17, code);
}

void ftoa(char* str, float number){
	float t = number;
	int steps = 0;
	do{
		steps++;
		t /= 10;
	}while(t>=1);
	steps += 3;
	int iter = steps;
	int num = (int)(number*1000);
	for(int i = 0; i < steps; i++){
		if(i == 3){
			str[iter] = 44;
			iter--;
		}
		switch(num % 10){
			case 0 : str[iter] = 48; break;
			case 1 : str[iter] = 49; break;
			case 2 : str[iter] = 50; break;
			case 3 : str[iter] = 51; break;
			case 4 : str[iter] = 52; break;
			case 5 : str[iter] = 53; break;
			case 6 : str[iter] = 54; break;
			case 7 : str[iter] = 55; break;
			case 8 : str[iter] = 56; break;
			case 9 : str[iter] = 57; break;
		}
		num /= 10;
		iter--;
	}
}

Coordinates::Coordinates(const Coordinates & t){
    this->k1 = t.k1;
    this->k2 = t.k2;
    this->k3 = t.k3;
    this->k4 = t.k4;
    this->k5 = t.k5;
    this->type = t.type;
}

Coordinates::Coordinates(){
    this->k1 = 0.0;
    this->k2 = 0.0;
    this->k3 = 0.0;
    this->k4 = 0.0;
    this->k5 = 0.0;
    this->type = none;
}

Coordinates::Coordinates(type_co type, float k1, float k2, float k3){
    this->k1 = k1;
    this->k2 = k2;
    this->k3 = k3;
    this->k4 = 0;
    this->k5 = 0;
    this->type = type;
}

Coordinates::Coordinates(type_co type, float k1, float k2, float k3, float k4, float k5){
    this->k1 = k1;
    this->k2 = k2;
    this->k3 = k3;
    this->k4 = k4;
    this->k5 = k5;
    this->type = type;
}

void Coordinates::translate(type_co t_type){
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
        case none: break;
    }
}

PointCa::PointCa(){
    name = new char[20];
    for(int i = 0; i< 20 ; i++)name[i] = 0;
}

PointCa::PointCa(char* t_name){
    name = new char[20];
    for(int i = 0; i< 20 ; i++)name[i] = 0;
    for(int i = 0; i< 20 ; i++){
        if(t_name[i] > 31){
            name[i] = t_name[i];
        }
        else{
            break;
        }
    }
}

PointCa::PointCa(const PointCa &t){
    name = new char[20];
    for(int i = 0; i< 20 ; i++)name[i] = 0;
    for(int i = 0; i< 20 ; i++){
        if(t.name[i] > 31){
            name[i] = t.name[i];
        }
        else{
            break;
        }
    }
}

float pointToPointDistance(Coordinates from, Coordinates to){
    from.translate(cartesianCo);
    to.translate(cartesianCo);
    float x = pow(to.k1-from.k1, 2);
    float y = pow(to.k2-from.k2, 2);
    float z = pow(to.k3-from.k3, 2);
    float s = x+y+z;
    float t = sqrt(s);
    return t;
}

float greaterThan(float temp1, float temp2){
    if(temp1 > temp2){return temp1;}else{return temp2;}
}

float pointToPointDistanceJointMax(Coordinates from, Coordinates to){
    float dis1 = abs(from.k1-to.k1);
    float dis2 = abs(from.k2-to.k2);
    float dis3 = abs(from.k3-to.k3);
    float dis4 = abs(from.k4-to.k4);
    float dis5 = abs(from.k5-to.k5);
    
    return greaterThan(dis1, greaterThan(dis2, greaterThan(dis3, greaterThan(dis4, dis5))));
}