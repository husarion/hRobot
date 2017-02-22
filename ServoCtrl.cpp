#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "ServoCtrl.h"
#include "Addons.h"
#include "Addons.h"

ServoCtrl::ServoCtrl(IServo& servo_t, int servo_center_t, float threshold_t,
                     float kp_down_t, float ki_down_t, float kd_down_t, float kp_up_t, float ki_up_t, float kd_up_t, float error_saturate_t,
                     float integrator_saturate_down_t, float integrator_saturate_up_t)
{
	servo = &servo_t;
	servo_center = servo_center_t;
	servo->calibrate(-100, servo_center - 700, 100, servo_center + 700);
	threshold_t = threshold_t;
	
	kp_down = kp_down_t;
	ki_down = ki_down_t;
	kd_down = kd_down_t;
	
	kp_down = kp_down_t;
	ki_down = ki_down_t;
	kd_down = kd_down_t;
	
	error_saturate = error_saturate_t;
	integrator_saturate_up = integrator_saturate_up_t;
	integrator_saturate_down = integrator_saturate_down_t;
}
int ServoCtrl::update(float error, float t_time)
{
	if (error < 0) {
		updateDown(error, t_time);
	} else {
		updateUp(error, t_time);
	}
}

int ServoCtrl::updateUp(float error, float t_time)
{
	error = thresholdFloat(error, threshold);
	error = saturateFloat(error, error_saturate);
	error_integrator += error;
	error_deviator = (error - error_last);//(t_time-time_last);
	error_integrator = saturateFloat(error_integrator, integrator_saturate_down);
	output = error * kp_down + error_integrator * ki_down + error_deviator * kd_down;
	make_output(output);
	error_last = error;
	time_last = t_time;
}

int ServoCtrl::updateDown(float error, float t_time)
{
	error = thresholdFloat(error, threshold);
	error = saturateFloat(error, error_saturate);
	error_integrator += error;
	error_deviator = (error - error_last);//(t_time-time_last);
	error_integrator = saturateFloat(error_integrator, integrator_saturate_down);
	output = error * kp_down + error_integrator * ki_down + error_deviator * kd_down;
	make_output(output);
	error_last = error;
	time_last = t_time;
}

void ServoCtrl::make_output(float val)
{
	if (val == 0) {
		servo->rotAbs(-1000);
	} else {
		val = saturateFloat(val, 100);
		servo->rotAbs(val);
	}
}

void ServoCtrl::set_error_saturate(float error_saturate_t) {error_saturate = error_saturate_t;}
void ServoCtrl::set_output_saturate(float output_saturate_t) {output_saturate = output_saturate_t;}
void ServoCtrl::set_threshold(float threshold_t) {threshold = threshold_t;}

float ServoCtrl::get_error_saturate() {return error_saturate;}
float ServoCtrl::get_output_saturate() {return output_saturate;}
float ServoCtrl::get_threshold() {return threshold;}
