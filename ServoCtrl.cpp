#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "ServoCtrl.h"
#include "Addons.h"
#include "Addons.h"

ServoCtrl::ServoCtrl(IServo& servo_t, int servo_center_t, float threshold_t,
                     float kp_t, float ki_t, float kd_t, float error_saturate_t,
                     float integrator_saturate_t)
{
    //UART(231000);
	servo = &servo_t;
	servo_center = servo_center_t;
	servo->calibrate(-100, servo_center - 700, 100, servo_center + 700);
	servo->calibrate(-100, servo_center - 700, 100, servo_center + 700);
	threshold_t = threshold_t;
	kp = kp_t;
	ki = ki_t;
	kd = kd_t;
	error_saturate = error_saturate_t;
	integrator_saturate = integrator_saturate_t;
}

void ServoCtrl::update(float error, float t_time)
{

	//UART(232000);
	error = thresholdFloat(error, threshold);
	error = saturateFloat(error, error_saturate);
	error_integrator += error;
	error_deviator = (error - error_last);//(t_time-time_last);
	error_integrator = saturateFloat(error_integrator, integrator_saturate);
	output = error * kp + error_integrator * ki + error_deviator * kd;
	make_output(output);
	error_last = error;
	time_last = t_time;
}

void ServoCtrl::make_output(float val)
{
    //UART(233000);
	if (val == 0) {
		servo->rotAbs(-1000);
	} else {
		output = saturateFloat(val, 100);
		servo->rotAbs(val);
	}
}

void ServoCtrl::set_pid_values(float kp_t, float ki_t, float kd_t)
{
	kp = kp_t;
	ki = ki_t;
	kd = kd_t;
}

void ServoCtrl::set_kp(float kp_t) {kp = kp_t;}
void ServoCtrl::set_ki(float ki_t) {ki = ki_t;}
void ServoCtrl::set_kd(float kd_t) {kd = kd_t;}

void ServoCtrl::set_error_saturate(float error_saturate_t) {error_saturate = error_saturate_t;}
void ServoCtrl::set_integrator_saturate(float integrator_saturate_t) {integrator_saturate = integrator_saturate_t;}
void ServoCtrl::set_output_saturate(float output_saturate_t) {output_saturate = output_saturate_t;}
void ServoCtrl::set_threshold(float threshold_t) {threshold = threshold_t;}

float ServoCtrl::get_kp() {return kp;}
float ServoCtrl::get_ki() {return ki;}
float ServoCtrl::get_kd() {return kd;}

float ServoCtrl::get_error_saturate() {return error_saturate;}
float ServoCtrl::get_integrator_saturate() {return integrator_saturate;}
float ServoCtrl::get_output_saturate() {return output_saturate;}
float ServoCtrl::get_threshold() {return threshold;}
