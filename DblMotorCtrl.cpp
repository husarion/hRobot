#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "DblMotorCtrl.h"
#include "Addons.h"
#include "Addons.h"
DblMotorCtrl::DblMotorCtrl(float threshold_t,
                     float kp_t, float ki_t, float kd_t, float error_saturate_t,
                     float integrator_saturate_t)
{
    //UART(231000);
    threshold_t = threshold_t;
    kp = kp_t;
    ki = ki_t;
    kd = kd_t;
    error_saturate = error_saturate_t;
    integrator_saturate = integrator_saturate_t;
}

int DblMotorCtrl::update(float error1, float t_time)
{
    //UART(232000);
    error1 = thresholdFloat(error1, threshold);
    error1 = saturateFloat(error1, error_saturate);
    error1_integrator += error1;
    error1_deviator = (error1 - error1_last);//(t_time-time_last);
    error1_integrator = saturateFloat(error1_integrator, integrator_saturate);
    output1 = error1 * kp + error1_integrator * ki + error1_deviator * kd;
    error1_last = error1;
    
    time_last = t_time;
    
    make_output(output1);
}

void DblMotorCtrl::make_output(float o1)
{
    hMot2.setPower(saturateFloat(-o1, 100)*7);
}

void DblMotorCtrl::set_pid_values(float kp_t, float ki_t, float kd_t)
{
    kp = kp_t;
    ki = ki_t;
    kd = kd_t;
}

void DblMotorCtrl::set_kp(float kp_t){
	kp = kp_t;
}

void DblMotorCtrl::set_ki(float ki_t){
	ki = ki_t;
}

void DblMotorCtrl::set_kd(float kd_t){
	kd = kd_t;
}

void DblMotorCtrl::set_error_saturate(float error_saturate_t){
	error_saturate = error_saturate_t;
}

void DblMotorCtrl::set_integrator_saturate(float integrator_saturate_t){
	integrator_saturate = integrator_saturate_t;
}

void DblMotorCtrl::set_output_saturate(float output_saturate_t){
	output_saturate = output_saturate_t;
}

void DblMotorCtrl::set_threshold(float threshold_t){
	threshold = threshold_t;
}

float DblMotorCtrl::get_kp(){
	return kp;
}

float DblMotorCtrl::get_ki(){
	return ki;
}

float DblMotorCtrl::get_kd(){
	return kd;
}

float DblMotorCtrl::get_error_saturate(){
	return error_saturate;
}

float DblMotorCtrl::get_integrator_saturate(){
	return integrator_saturate;
}

float DblMotorCtrl::get_output_saturate(){
	return output_saturate;
}

float DblMotorCtrl::get_threshold(){
	return threshold;
}
