#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "Addons.h"
#include "GeoMath.h"

// internal data
	
//float step=deg2rad(45);

Coordinates  joints2cylin(Coordinates temp)
{
	float k1 = d1 + d2 * cosd(temp.k2 + angle90) + d4 * cosd(temp.k3 + temp.k2) + d5 * cosd(temp.k4 + temp.k3 + temp.k2 - angle90);
	float k2 = a1 + d2 * sind(temp.k2 + angle90) + d4 * sind(temp.k3 + temp.k2) + d5 * sind(temp.k4 + temp.k3 + temp.k2 - angle90);
	float k3 = temp.k1;
	float k4 = temp.k4 - temp.k2 - temp.k3;
	return Coordinates(cylindricalCo, k1, k2, k3, k4, temp.k5);
}

Coordinates  cylin2cartes(Coordinates temp)
{
	float k1 = temp.k1 * cosd(temp.k3);
	float k2 = temp.k1 * sind(temp.k3);
	float k3 = temp.k2;
	return Coordinates(cartesianCo, k1, k2, k3, temp.k4, temp.k5);
}

Coordinates  joints2cartes(Coordinates temp)
{
	return cylin2cartes(joints2cylin(temp));
}

Coordinates  cartes2cylin(Coordinates temp)
{
	float k1 = sqrt(sq(temp.k1) + sq(temp.k2));
	float k2 = temp.k3;
	float k3 = rad2deg(atan2(temp.k2, temp.k1));
	return Coordinates(cylindricalCo, k1, k2, k3, temp.k4, temp.k5);
}

float  convergenceCartesian(Coordinates temp1, Coordinates temp2)
{
	return sqrt(sq(temp1.k1 - temp2.k1) + sq(temp1.k2 - temp2.k2) + sq(temp1.k3 - temp2.k3)) + sqrt(sq(((temp1.k4 - temp2.k4)))) / 5;
}

float  convergenceCylindrical(Coordinates temp1, Coordinates temp2)
{
	return sqrt(sq(temp1.k1 - temp2.k1) + sq(temp1.k2 - temp2.k2)) + abs(temp1.k4 - temp2.k4) / 5;
}

Coordinates  cartes2joints(Coordinates cartes_target, Coordinates joints, float accuracy)
{
	Coordinates cylin_target = cartes2cylin(cartes_target);
	Coordinates joints_t = cylin2joints(cylin_target, joints, accuracy);
	joints_t.k1 = cylin_target.k3;
	joints_t.k5 = cartes_target.k5;
	return joints_t;
}

Coordinates  cylin2joints(Coordinates cylin_target, Coordinates joints, float accuracy)
{

	Coordinates joints_temp = joints;
	joints_temp.k1 = cylin_target.k3;
	Coordinates cylin_temp = joints2cylin(joints_temp);
	float convergence = convergenceCylindrical(cylin_target, cylin_temp);
	float st = convergence*5; //float st=step;
	float convergenceLast = convergence;
	float convergenceLastLast = convergenceLast;
	bool done = false;
	while(!done) {
		//k2 modifk2
		joints_temp = joints;	// make work object
		joints_temp.k2 += st;	// modifk2 up
		cylin_temp = joints2cylin(joints_temp); // make material to compare
		convergence = convergenceCylindrical(cylin_target, cylin_temp); // compare Coordinatess
		if (convergence < convergenceLast) { // if better
			convergenceLast = convergence;
			joints = joints_temp; // save new Coordinates
			//printf("1");
		} else {
			joints_temp.k2 -= st * 2;	// modifk2 up
			cylin_temp = joints2cylin(joints_temp); // make material to compare
			convergence = convergenceCylindrical(cylin_target, cylin_temp); // compare cylin-cosd
			if (convergence < convergenceLast) { // if better
				convergenceLast = convergence;
				joints = joints_temp; // save new Coordinates
				//printf("2");
			}
		}
		//k3 modifk2
		joints_temp = joints;	// make work object
		joints_temp.k3 += st;	// modifk2 up
		cylin_temp = joints2cylin(joints_temp); // make material to compare
		convergence = convergenceCylindrical(cylin_target, cylin_temp); // compare Coordinatess
		if (convergence < convergenceLast) { // if better
			convergenceLast = convergence;
			joints = joints_temp; // save new Coordinates
			//printf("3");
		} else {
			joints_temp.k3 -= st * 2;	// modifk2 up
			cylin_temp = joints2cylin(joints_temp); // make material to compare
			convergence = convergenceCylindrical(cylin_target, cylin_temp); // compare cylin-cosd
			if (convergence < convergenceLast) { // if better
				convergenceLast = convergence;
				joints = joints_temp; // save new Coordinates
				//printf("4");
			}
		}
		//k4 modifk2
		joints_temp = joints;	// make work object
		joints_temp.k4 += st;	// modifk2 up
		cylin_temp = joints2cylin(joints_temp); // make material to compare
		convergence = convergenceCylindrical(cylin_target, cylin_temp); // compare Coordinatess
		if (convergence < convergenceLast) { // if better
			convergenceLast = convergence;
			joints = joints_temp; // save new Coordinates
			//printf("5");
		} else {
			joints_temp.k4 -= st * 2;	// modifk2 up
			cylin_temp = joints2cylin(joints_temp); // make material to compare
			convergence = convergenceCylindrical(cylin_target, cylin_temp); // compare cylin-cosd
			if (convergence < convergenceLast) { // if better
				convergenceLast = convergence;
				joints = joints_temp; // save new Coordinates
				//printf("6");
			}
		}
		cylin_temp = joints2cylin(joints); // make material to compare
		convergence = convergenceCylindrical(cylin_target, cylin_temp);
		//printf("convergence: %f, target k1: %f, ta rget k2: %f, temp k1: %f, temp k2: %f\k1\n", convergenceLast, cylin_target.k1, cylin_target.k2, cylin_temp.k1, cylin_temp.k2);
		//printf("current calculated Coordinates: k1=%f, k2=%f, k3=%f, k4=%f, k5=%f\k1\n", rad2deg(joints.k1), rad2deg(joints.k2), rad2deg(joints.k3), rad2deg(joints.k4), rad2deg(joints.k5));
		if (convergence >= convergenceLastLast) { // if tk2e re was ank2 imp rovement, ck2ange step.
			st = st / 2;
			if (st <= (accuracy / 10000000000000)) { // fell in extremum trap
				//->printf("fell in extremum trap\t\n");
				done = true;
				//joints=Coordinates(-0,-0,-0,-0,-0); //  TODO:  no idea what to return...
			}
		}
		if (convergenceLast <= accuracy) { // better tk2en  requi red?
			//->printf("good job\t\n");
			//->printf("convergence: %f\t\n", convergenceLast);
			done = true;
		}
		convergenceLastLast = convergenceLast;
	}
	joints.k5 = cylin_target.k5;
	return joints;
}

