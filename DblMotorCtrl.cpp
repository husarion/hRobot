#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "DblMotorCtrl.h"
#include "Addons.h"
#include "Addons.h"

DblMotorCtrl::DblMotorCtrl(float threshold_t,
                     float kp_down_t, float ki_down_t, float kd_down_t, float kp_up_t, float ki_up_t, float kd_up_t, float error_saturate_t,
                     float integrator_saturate_down_t, float integrator_saturate_up_t)
{
    threshold_t = threshold_t;

	kp_down = kp_down_t;
	ki_down = ki_down_t;
	kd_down = kd_down_t;
	
	kp_up = kp_up_t;
	ki_up = ki_up_t;
	kd_up = kd_up_t;

    error_saturate = error_saturate_t;
    integrator_saturate_up = integrator_saturate_up_t;
    integrator_saturate_down = integrator_saturate_down_t;
}

int DblMotorCtrl::update(float error1, float t_time)
{
	if (error1 < 0) {
		updateDown(error1, t_time);
	} else {
		updateUp(error1, t_time);
	}
}

int DblMotorCtrl::updateUp(float error1, float t_time)
{
    error1 = thresholdFloat(error1, threshold);
    error1 = saturateFloat(error1, error_saturate);
    error1_integrator += error1;
    error1_deviator = (error1 - error1_last);//(t_time-time_last);
    error1_integrator = saturateFloat(error1_integrator, integrator_saturate_up);
    output1 = error1 * kp_up + error1_integrator * ki_up + error1_deviator * kd_up;
    make_output(output1);
    error1_last = error1;
    time_last = t_time;
}

int DblMotorCtrl::updateDown(float error1, float t_time)
{
    error1 = thresholdFloat(error1, threshold);
    error1 = saturateFloat(error1, error_saturate);
    error1_integrator += error1;
    error1_deviator = (error1 - error1_last);//(t_time-time_last);
    error1_integrator = saturateFloat(error1_integrator, integrator_saturate_down);
    output1 = error1 * kp_down + error1_integrator * ki_down + error1_deviator * kd_down;
    make_output(output1);
    error1_last = error1;  
    time_last = t_time;
}

void DblMotorCtrl::make_output(float val)
{
	if (val == 0) {
		hMot2.setPower(0);
	} else {
		val = saturateFloat(val, 100);
        val=val*10;                       
        val = saturateFloat(val, output_saturate);   // 70% max power
		hMot2.setPower(val);
	}
}

void DblMotorCtrl::set_pid_valuesUp(float kp_t, float ki_t, float kd_t)
{
    kp_up = kp_t;
    ki_up = ki_t;
    kd_up = kd_t;
}


void DblMotorCtrl::set_pid_valuesDown(float kp_t, float ki_t, float kd_t)
{
    kp_down = kp_t;
    ki_down = ki_t;
    kd_down = kd_t;
}

void DblMotorCtrl::set_output_saturate(float output_saturate_t){
	output_saturate = output_saturate_t;
}

float DblMotorCtrl::get_output_saturate(){
	return output_saturate;
}
