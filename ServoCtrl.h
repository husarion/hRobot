/*Przykładowa implementacja {
	ServoCtrl J1(s1, 1125, 4, 1.5, 0.1, 2, 100, 20);

	hMot1.setEncoderPolarity(Polarity::Reversed);
	hServoModule.enablePower();

	for (;;)
	{
		J1.update(target[1] - (float)hMot1.getEncoderCnt() / encoder_tics_J1 , sys.getRefTime());
	}
}
*/
#include <cstddef>
#include <cstdint>
#include "hFramework.h"

#ifndef ServoCrtlClass
#define ServoCrtlClass

class ServoCtrl {
private:
	IServo* servo;
	float threshold;
	float kp;
	float ki;
	float kd;
	float error_saturate;
	float integrator_saturate;
	float output_saturate;
	float error_last;
	float error_integrator;
	float error_deviator;
	float time_last;
	int servo_center;
	float output;

	void make_output(float val);
public:

	ServoCtrl(IServo& servo_t, int servo_center_t, float threshold_t,
	                               float kp_t, float ki_t, float kd_t, float error_saturate_t,
	                               float integrator_saturate_t);
	int update(float error, float t_time);
	void set_pid_values(float kp_t, float ki_t, float kd_t);
	void set_kp(float kp_t);
	void set_ki(float ki_t);
	void set_kd(float kd_t);
	float get_kp();
	float get_ki();
	float get_kd();
	void set_error_saturate(float error_saturate_t);
	float get_error_saturate();
	void set_integrator_saturate(float integrator_saturate_t);
	float get_integrator_saturate();
	void set_output_saturate(float output_saturate_t);
	float get_output_saturate();
	void set_threshold(float threshold_t);
	float get_threshold();
};

#endif
