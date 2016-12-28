#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "hMotor.h"
#include "soft_enc.h"
//#include "ErrorLog.h"

soft_enc enkoder1(hExt.pin1, hExt.pin2);

hPIDRegulator pidReg, pidReg2;

bool rath_control = true;
int curent_joint = 1;

float encoder_J1 = 0;
float encoder_J21 = 0;
float encoder_J22 = 0;
float encoder_J3 = 0;
float encoder_J5 = 0;
float encoder_J6 = 0;

float set_pozytion_J1 = 0;
float set_pozytion_J21 = 0;
float set_pozytion_J22 = 0;
float set_pozytion_J3 = 0;
float set_pozytion_J5 = 0;
float set_pozytion_J6 = 0;

const int encoder_tics_J1 = 20;
const int encoder_tics_J21 = 4000;
const int encoder_tics_J22 = 4000;
const int encoder_tics_J3 = 20;
const int encoder_tics_J5 = 20;
const int encoder_tics_J6 = 20;

float servo_center_J1 = 70;//110-test
float servo_center_J3 = 320;
float servo_center_J5 = -980;
float servo_center_J6 = 110;//225-robot

void encoder_task()
{
    enkoder1.init();
	enkoder1.resetEncoderCnt();
    float temp;
    for (;;)
    {
        temp = (float)hMot1.getEncoderCnt()/(float)encoder_tics_J1;
        if(temp > 360) temp = 360;
        if(temp < 0) temp = 0;
        encoder_J1 = temp;
        
        temp = (float)hMot2.getEncoderCnt()/(float)encoder_tics_J21;
        if(temp > 360) temp = 360;
        if(temp < 0) temp = 0;
        encoder_J21 = temp;
        temp = (float)hMot3.getEncoderCnt()/(float)encoder_tics_J22;
        if(temp > 360) temp = 360;
        if(temp < 0) temp = 0;
        encoder_J22 = temp;
        
        temp = (float)hMot4.getEncoderCnt()/(float)encoder_tics_J3;
        if(temp > 360) temp = 360;
        if(temp < 0) temp = 0;
        encoder_J3 = temp;
        
        temp = (float)enkoder1.getEncoderCnt()/(float)encoder_tics_J6;
        if(temp > 360) temp = 360;
        if(temp < 0) temp = 0;
        encoder_J6 = temp;
        
        LED3.toggle();
        sys.delay(10);
    }
}

void motor_task(){
    pidReg.setScale(1);
    pidReg.setKP(40.0);
    pidReg.setKI(0.05);
    pidReg.setKD(1000);
    pidReg.dtMs = 5;
    pidReg.stableRange = 10;
    pidReg.stableTimes = 3;
    pidReg2 = pidReg;
    hMot2.attachPositionRegulator(pidReg);
    hMot3.attachPositionRegulator(pidReg2);
    hMot3.setEncoderPolarity(Polarity::Reversed);
    hMot3.setMotorPolarity(Polarity::Reversed);
    
    hMot1.setEncoderPolarity(Polarity::Reversed);
    hMot4.setEncoderPolarity(Polarity::Reversed);
    
    hServoModule.enablePower();
    IServo& s1 = hServoModule.servo1;
    s1.calibrate(-360, 700, 360, 1500);
    IServo& s2 = hServoModule.servo2;
    s2.calibrate(-360, 700, 360, 1500);
    IServo& s3 = hServoModule.servo3;
    s3.calibrate(-360, 700, 360, 1500);
    IServo& s4 = hServoModule.servo4;
    s4.calibrate(-360, 700, 360, 1500);
    float t_set_pozytion_J1 = 0;
    float t_set_pozytion_J21 = 0;
    float t_set_pozytion_J22 = 0;
    float t_set_pozytion_J3 = 0;
    float t_set_pozytion_J5 = 0;
    float t_set_pozytion_J6 = 0;
    for(;;){
        t_set_pozytion_J1 = set_pozytion_J1;
        if(t_set_pozytion_J1>360)t_set_pozytion_J1=360;
        if(t_set_pozytion_J1<0)t_set_pozytion_J1=0;
        t_set_pozytion_J21 = set_pozytion_J21;
        if(t_set_pozytion_J21>360)t_set_pozytion_J21=360;
        if(t_set_pozytion_J21<0)t_set_pozytion_J21=0;
        t_set_pozytion_J22 = set_pozytion_J22;
        if(t_set_pozytion_J22>360)t_set_pozytion_J22=360;
        if(t_set_pozytion_J22<0)t_set_pozytion_J22=0;
        t_set_pozytion_J3 = set_pozytion_J3;
        if(t_set_pozytion_J3>360)t_set_pozytion_J3=360;
        if(t_set_pozytion_J3<0)t_set_pozytion_J3=0;
        t_set_pozytion_J5 = set_pozytion_J5;
        if(t_set_pozytion_J5>360)t_set_pozytion_J5=360;
        if(t_set_pozytion_J5<0)t_set_pozytion_J5=0;
        t_set_pozytion_J6 = set_pozytion_J6;
        if(t_set_pozytion_J6>360)t_set_pozytion_J6=360;
        if(t_set_pozytion_J6<0)t_set_pozytion_J6=0;
        
        hMot2.rotAbs(t_set_pozytion_J21*encoder_tics_J21/1000);
        hMot3.rotAbs(t_set_pozytion_J22*encoder_tics_J22/1000);
        
        s1.rotAbs(((t_set_pozytion_J1 - encoder_J1)*20+servo_center_J1));
        //s2.rotAbs(((t_set_pozytion_J3 - encoder_J3)*20+servo_center_J3));
        //s3.rotAbs(((t_set_pozytion_J3 - encoder_J3)*20+servo_center_J3));
        s4.rotAbs(((t_set_pozytion_J1 - encoder_J1)*20+servo_center_J1));
        
        LED2.toggle();
        sys.delay(50);
    }
}

void printfOnConsoleInWebIDE()
{
	//for (;;) {
	//	if (ErrorLogs::getErrorLoger().getSize() > 0 && (int)sys.getRefTime() > 6000){
	//		ErrorLogs::getErrorLoger().translateError(ErrorLogs::getErrorLoger().getLastError());
	//		sys.delay(100);
	//	}
	//	else{
	//	sys.delay(1000);
	//	}
	//}
}

void cfgHandler()
{
	platform.ui.loadHtml({Resource::WEBIDE, "/ui.html"});
	//auto l1 = platform.ui.label("l1");
	//auto lb_bat = platform.ui.label("lb_bat");
	//auto l2 = platform.ui.label("l2");

}

void onKeyEvent(KeyEventType type, KeyCode code)
{
	if (code == KeyCode::Up && type == KeyEventType::Pressed) {
		if (curent_joint == 1){
		    if(!rath_control)
		        set_pozytion_J1 += 1;
		    else
		        set_pozytion_J1 += 5;
		}
		if (curent_joint == 2) {
			if(!rath_control)
		        {set_pozytion_J21 += 1;set_pozytion_J22 += 1;}
		    else
		        {set_pozytion_J21 += 5;set_pozytion_J22 += 5;}
		}
		if (curent_joint == 3){
			if(!rath_control)
		        set_pozytion_J3 += 1;
		    else
		        set_pozytion_J3 += 5;
		}
	}
	if (code == KeyCode::Down && type == KeyEventType::Pressed) {
		if (curent_joint == 1){
			if(!rath_control)
		        set_pozytion_J1 -= 1;
		    else
		        set_pozytion_J1 -= 5;
	    }
		if (curent_joint == 2){
			if(!rath_control)
		        {set_pozytion_J21 -= 1;set_pozytion_J22 -= 1;}
		    else
		        {set_pozytion_J21 -= 5;set_pozytion_J22 -= 5;}
        }	        
		if (curent_joint == 3){
			if(!rath_control)
		        set_pozytion_J3 -= 1;
		    else
		        set_pozytion_J3 -= 5;
        }
	}
	if (code == KeyCode::Left && type == KeyEventType::Pressed) {
		if (curent_joint == 0) {
			curent_joint = 10;
		} else
			curent_joint--;
	}
	if (code == KeyCode::Right && type == KeyEventType::Pressed) {
		if (curent_joint == 10) {
			curent_joint = 0;
		} else
			curent_joint++;
	}
	if (code == KeyCode::Key_0 && type == KeyEventType::Pressed)
		curent_joint = 10;
	if (code == KeyCode::Key_1 && type == KeyEventType::Pressed)
		curent_joint = 1;
	if (code == KeyCode::Key_2 && type == KeyEventType::Pressed)
		curent_joint = 2;
	if (code == KeyCode::Key_3 && type == KeyEventType::Pressed)
		curent_joint = 3;
	if (code == KeyCode::Key_4 && type == KeyEventType::Pressed)
		curent_joint = 4;
	if (code == KeyCode::Key_5 && type == KeyEventType::Pressed)
		curent_joint = 5;
	if (code == KeyCode::Key_6 && type == KeyEventType::Pressed)
		curent_joint = 6;
	if (code == KeyCode::Key_7 && type == KeyEventType::Pressed)
		curent_joint = 7;
	if (code == KeyCode::Key_8 && type == KeyEventType::Pressed)
		curent_joint = 8;
	if (code == KeyCode::Key_9 && type == KeyEventType::Pressed)
		curent_joint = 9;
	if (code == KeyCode::Key_F && type == KeyEventType::Pressed) {
		if (rath_control) {
			rath_control = false;
		} else {
			rath_control = true;
		}
	}
}

void onButtonEvent(hId id, ButtonEventType type)
{
	/*
	static int cnt = 0;
	if (id == "btn1")
	{
	    UiButton b = platform.ui.button("btn1");
	    if (type == ButtonEventType::Pressed) {
	        b.setText("pressed %u", cnt++);
	    } else {
	        b.setText("released %u", cnt++);
	    }
	    LED1.toggle();
	}*/
}
void hMain()
{
    platform.begin(&RPi);
    platform.ui.configHandler = cfgHandler;
    platform.ui.onKeyEvent = onKeyEvent;
    platform.ui.onButtonEvent = onButtonEvent;
    platform.ui.setProjectId("@@@PROJECT_ID@@@");
 
    sys.taskCreate(printfOnConsoleInWebIDE);
    sys.taskCreate(encoder_task);
    sys.taskCreate(motor_task);
    
    
    //ErrorLogs::getErrorLoger().addError(0);
	//ErrorLogs::getErrorLoger().addError(2);
	//ErrorLogs::getErrorLoger().addError(3);
	//ErrorLogs::getErrorLoger().addError(4);
	//ErrorLogs::getErrorLoger().addError(5);

	for (;;) {
		sys.delay(500);
		platform.ui.label("l1").setText("uptime %u", (unsigned int)sys.getRefTime());
		platform.ui.label("lb_bat").setText("%f [V]", sys.getSupplyVoltage());
		platform.ui.label("lb_joint").setText("Joint selected : J%d", curent_joint);
		if (rath_control) {
			platform.ui.label("lb_rath").setText("Rath control");
		} else {
			platform.ui.label("lb_rath").setText("Fine control");
		}
		platform.ui.label("lb_j1").setText("Set: %f deg\tCurent: %f deg\n", 
		set_pozytion_J1, encoder_J1);
		
		platform.ui.label("lb_j2").setText("Set: %f deg\t%f deg\tCurent: %f deg\t%f deg\n", 
		set_pozytion_J21, set_pozytion_J22, encoder_J21, encoder_J22);
		
		platform.ui.label("lb_j3").setText("Set: %f deg\tCurent: %f deg\n", 
		set_pozytion_J3, encoder_J3);
		
		platform.ui.label("lb_j6").setText("Set: %f deg\tCurent: %f deg\n", 
		set_pozytion_J6, encoder_J6);
		
		LED1.toggle();
	}
}