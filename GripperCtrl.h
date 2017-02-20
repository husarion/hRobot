#include <cstddef>
#include <cstdint>
#include "hFramework.h"

#ifndef GripperCrtlClass
#define GripperCrtlClass

class GripperCrtl {
private:
	IServo* servo;
	float threshold;
	int servo_center;

	void make_output(float val);
public:

	GripperCrtl(IServo& servo_t, int servo_center_t, float threshold_t);
	int update(float volume);
	void set_threshold(float threshold_t);
	float get_threshold();
};

#endif