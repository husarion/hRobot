#include <cstddef>
#include <cstdint>
#include "hFramework.h"

#ifndef DblMotorCtrlClass
#define DblMotorCtrlClass

class DblMotorCtrl {
private:
	float kp;
	float ki;
	float kd;
public:

	DblMotorCtrl(float kp_t, float ki_t, float kd_t);
	int update(float error1, float t_time);
	void set_pid_values(float kp_t, float ki_t, float kd_t);
	void set_kp(float kp_t);
	void set_ki(float ki_t);
	void set_kd(float kd_t);
	float get_kp();
	float get_ki();
	float get_kd();
};

#endif
