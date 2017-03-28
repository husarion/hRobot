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

class ServoCtrl
{
  private:
    IServo *servo;
    float threshold;
    float kp_up;
    float ki_up;
    float kd_up;
    float kp_down;
    float ki_down;
    float kd_down;
    float error_saturate;
    float integrator_saturate_up;
    float integrator_saturate_down;
    float output_saturate;
    float error_last;
    float error_integrator;
    float error_deviator;
    float time_last;
    int servo_center;
    float output;

    void makeOutput(float val);

  public:
    ServoCtrl(IServo &servo_t, int servo_center_t, float threshold_t,
	      float kp_down_t, float ki_down_t, float kd_down_t, float kp_up_t, float ki_up_t, float kd_up_t, float error_saturate_t,
	      float integrator_saturate_down_t, float integrator_saturate_up_t);
    void update(float error, float t_time);
    void updateDown(float error, float t_time);
    void updateUp(float error, float t_time);
    void setErrorSaturate(float error_saturate_t);
    float getErrorSaturate();
    void setOutputSaturate(float output_saturate_t);
    float getOutputSaturate();
    void setThreshold(float threshold_t);
    float getThreshold();
};

#endif
