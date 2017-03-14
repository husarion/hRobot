#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "GripperCtrl.h"
#include "Addons.h"
#include "Addons.h"

GripperCrtl::GripperCrtl(IServo &servo_t)
{
    servo = &servo_t;
    servo->calibrate(-100, 10, 100, 2000);
}

void GripperCrtl::update(float volume)
{
    make_output(volume);
}

void GripperCrtl::make_output(float val)
{
    if (val < -5)
    {
	servo->rotAbs(-100);
    }
    else if (val > 5)
    {
	servo->rotAbs(100);
    }
    else
    {
	servo->rotAbs(-1000);
    }
}
