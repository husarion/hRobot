#include <cstddef>
#include <cstdint>
#include "hFramework.h"

#ifndef DblMotorCtrlClass
#define DblMotorCtrlClass

class DblMotorCtrl {
private:
	float threshold;
	float kp;
	float ki;
	float kd;
	float error_saturate;
	float integrator_saturate;

	float error1_integrator;
	float error1_deviator;
	float error1_last;
	float output1;
	float time_last;
	float output_saturate;
public:

	DblMotorCtrl(float threshold_t, float kp_t, float ki_t, float kd_t,
				float error_saturate_t, float integrator_saturate_t);
	int update(float error1, float t_time);
	void make_output(float o1);
	void set_pid_values(float kp_t, float ki_t, float kd_t);
	void set_kp(float kp_t);
	void set_ki(float ki_t);
	void set_kd(float kd_t);
	float get_kp();
	float get_ki();
	float get_kd();
	void set_error_saturate(float error_saturate_t);
	void set_integrator_saturate(float integrator_saturate_t);
	void set_output_saturate(float output_saturate_t);
	void set_threshold(float threshold_t);
	float get_error_saturate();
	float get_integrator_saturate();
	float get_output_saturate();
	float get_threshold();
};

#endif
