#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "GripperCtrl.h"
#include "Addons.h"
#include "Addons.h"

GripperCrtl::GripperCrtl(IServo& servo_t, int servo_center_t, float threshold_t)
{
    //UART(231000);
	servo = &servo_t;
	servo_center = servo_center_t;
	servo->calibrate(-100, servo_center - 500, 100, servo_center + 500);
	threshold_t = threshold_t;
}

int GripperCrtl::update(float volume)
{
	//UART(232000);
	make_output(volume);
}

void GripperCrtl::make_output(float val)
{
    //UART(233000);
	if(val>5){
		    servo->rotAbs(-100);
		}
		else if(val<-5){
		    servo->rotAbs(100);
		}
		else{
		    servo->rotAbs(0);
		}
}

void GripperCrtl::set_threshold(float threshold_t) {threshold = threshold_t;}

float GripperCrtl::get_threshold() {return threshold;}
