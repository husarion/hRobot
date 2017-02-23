#include <cstddef>
#include <cstdint>
#include "hFramework.h"

#ifndef GripperCrtlClass
#define GripperCrtlClass

class GripperCrtl {
private:
	IServo* servo;
	void make_output(float val);
public:
	GripperCrtl(IServo& servo_t);
	int update(float volume);
};

#endif