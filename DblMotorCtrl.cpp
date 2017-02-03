#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "DblMotorCtrl.h"
#include "Addons.h"
#include "Addons.h"

hPIDRegulator pidReg, pidReg2;

DblMotorCtrl::DblMotorCtrl(float kp_t, float ki_t, float kd_t)
{
    //UART(231000);
	kp = kp_t;
	ki = ki_t;
	kd = kd_t;
	
	pidReg.setScale(1);
	pidReg.setKP(kp_t);
	pidReg.setKI(ki_t);
	pidReg.setKD(1000);
	pidReg.dtMs = 5;
	pidReg.stableRange = 10;
	pidReg.stableTimes = 3;
	pidReg2 = pidReg;
	hMot2.attachPositionRegulator(pidReg);
	hMot3.attachPositionRegulator(pidReg2);
	hMot3.setEncoderPolarity(Polarity::Reversed);
	hMot3.setMotorPolarity(Polarity::Reversed);
	
}

int DblMotorCtrl::update(float error1, float error2, float t_time)
{

	//UART(232000);
	hMot2.rotRel(error1);
	hMot3.rotRel(error2);
	
}

void DblMotorCtrl::set_pid_values(float kp_t, float ki_t, float kd_t)
{
	pidReg.setKP(kp_t);
	pidReg.setKI(ki_t);
	pidReg.setKD(kd_t);
	pidReg2 = pidReg;
}

void DblMotorCtrl::set_kp(float kp_t) {pidReg.setKP(kp_t); pidReg2 = pidReg;}
void DblMotorCtrl::set_ki(float ki_t) {pidReg.setKP(ki_t); pidReg2 = pidReg;}
void DblMotorCtrl::set_kd(float kd_t) {pidReg.setKP(kd_t); pidReg2 = pidReg;}

float DblMotorCtrl::get_kp() {return kp;}
float DblMotorCtrl::get_ki() {return ki;}
float DblMotorCtrl::get_kd() {return kd;}