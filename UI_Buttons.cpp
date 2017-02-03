#include <cstddef>
#include <cstdint>
#include "hFramework.h"
#include "hCloudClient.h"
#include "UI_Buttons.h"
#include "Addons.h"

// jog, ui, recorded positions values
extern float target[9];
extern float temp[9];
extern float pos1[9];
extern float pos2[9];
extern float pos3[9];
extern float pos4[9];
extern float pos5[9];
extern float pos6[9];
extern bool mode;
extern int pos_label; //switch counter for position

// jog UI steps
const float step1=1;
const float step2=5;
const float step3=15;
const float step4=45;

// temp PID values for calibration
extern float tempKp;
extern float tempKi;
extern float tempKd;


void cfgHandler()
{	
    //UART(131000);
	// strona startowa
    platform.ui.loadHtml({Resource::WEBIDE, "/ui.html"});
    
	//UART(132000);
	// labels
	auto info = platform.ui.label("info");
	auto PIDinfo = platform.ui.label("PIDinfo");
	
	//UART(133000);
	// mode buttons
	auto btnmode = platform.ui.button("btnmode");
	auto btnsend = platform.ui.button("btnsend");
	// jog buttons
	auto btn10 = platform.ui.button("btn10");
	auto btn11 = platform.ui.button("btn11");
	auto btn12 = platform.ui.button("btn12");
	auto btn13 = platform.ui.button("btn13");
	auto btn14 = platform.ui.button("btn14");
	auto btn15 = platform.ui.button("btn15");
	auto btn16 = platform.ui.button("btn16");
	auto btn17 = platform.ui.button("btn17");
	auto btn18 = platform.ui.button("btn18");
	
	auto btn20 = platform.ui.button("btn20");
	auto btn21 = platform.ui.button("btn21");
	auto btn22 = platform.ui.button("btn22");
	auto btn23 = platform.ui.button("btn23");
	auto btn24 = platform.ui.button("btn24");
	auto btn25 = platform.ui.button("btn25");
	auto btn26 = platform.ui.button("btn26");
	auto btn27 = platform.ui.button("btn27");
	auto btn28 = platform.ui.button("btn28");
	
	auto btn30 = platform.ui.button("btn30");
	auto btn31 = platform.ui.button("btn31");
	auto btn32 = platform.ui.button("btn32");
	auto btn33 = platform.ui.button("btn33");
	auto btn34 = platform.ui.button("btn34");
	auto btn35 = platform.ui.button("btn35");
	auto btn36 = platform.ui.button("btn36");
	auto btn37 = platform.ui.button("btn37");
	auto btn38 = platform.ui.button("btn38");
	
	auto btn40 = platform.ui.button("btn40");
	
	auto btn50 = platform.ui.button("btn50");
	auto btn51 = platform.ui.button("btn51");
	auto btn52 = platform.ui.button("btn52");
	auto btn53 = platform.ui.button("btn53");
	auto btn54 = platform.ui.button("btn54");
	auto btn55 = platform.ui.button("btn55");
	auto btn56 = platform.ui.button("btn56");
	auto btn57 = platform.ui.button("btn57");
	auto btn58 = platform.ui.button("btn58");
	
	auto btn60 = platform.ui.button("btn60");
	auto btn61 = platform.ui.button("btn61");
	auto btn62 = platform.ui.button("btn62");
	auto btn63 = platform.ui.button("btn63");
	auto btn64 = platform.ui.button("btn64");
	auto btn65 = platform.ui.button("btn65");
	auto btn66 = platform.ui.button("btn66");
	auto btn67 = platform.ui.button("btn67");
	auto btn68 = platform.ui.button("btn68");
	
	// grabber buttons
	auto btn_close = platform.ui.button("btn_close");
	auto btn_open = platform.ui.button("btn_open");
	auto btn_stop = platform.ui.button("btn_stop");
	
	// write positions buttons
	auto btn_pos1_write = platform.ui.button("btn_pos1_write");
	auto btn_pos2_write = platform.ui.button("btn_pos2_write");
	auto btn_pos3_write = platform.ui.button("btn_pos3_write");
	auto btn_pos4_write = platform.ui.button("btn_pos4_write");
	auto btn_pos5_write = platform.ui.button("btn_pos5_write");
	auto btn_pos6_write = platform.ui.button("btn_pos6_write");
	
	//read positions buttons
	auto btn_pos1_read = platform.ui.button("btn_pos1_read");
	auto btn_pos2_read = platform.ui.button("btn_pos2_read");
	auto btn_pos3_read = platform.ui.button("btn_pos3_read");
	auto btn_pos4_read = platform.ui.button("btn_pos4_read");
	auto btn_pos5_read = platform.ui.button("btn_pos5_read");
	auto btn_pos6_read = platform.ui.button("btn_pos6_read");
	
	//show positions buttons
	auto btn_pos1_show = platform.ui.button("btn_pos1_show");
	auto btn_pos2_show = platform.ui.button("btn_pos2_show");
	auto btn_pos3_show = platform.ui.button("btn_pos3_show");
	auto btn_pos4_show = platform.ui.button("btn_pos4_show");
	auto btn_pos5_show = platform.ui.button("btn_pos5_show");
	auto btn_pos6_show = platform.ui.button("btn_pos6_show");
	auto btn_pos0_show = platform.ui.button("btn_pos0_show");
	auto btn_pos7_show = platform.ui.button("btn_pos7_show");
	
	// PID buttons
	auto btn_kpp = platform.ui.button("btn_kpp");
	auto btn_kpm = platform.ui.button("btn_kpm");
	auto btn_kip = platform.ui.button("btn_kip");
	auto btn_kim = platform.ui.button("btn_kim");
	auto btn_kdp = platform.ui.button("btn_kdp");
	auto btn_kdm = platform.ui.button("btn_kdm");
	
}

void onButtonEvent(hId id, ButtonEventType type)
{  
    //UART(121100);
	if(type == ButtonEventType::Pressed){ // Pressed
	    //UART(121200);
			// always
		// PID buttons
		if (id == "btn_kpp") tempKp+=0.05;
		if (id == "btn_kpm") tempKp-=0.05;
		if (id == "btn_kdp") tempKd+=0.05;
		if (id == "btn_kdm") tempKd-=0.05;
		if (id == "btn_kip") tempKi+=0.05;
		if (id == "btn_kim") tempKi-=0.05;
		// grabber buttons
		if (id == "btn_close") target[7]=5;
		if (id == "btn_open") target[7]=-5;
		if (id == "btn_stop") target[7]=0;
		// write positions buttons
		if (id == "btn_pos1_write") {for(int k=0;k<9;k++){pos1[k]=target[k];} pos_label=0;}
		if (id == "btn_pos2_write") {for(int k=0;k<9;k++){pos2[k]=target[k];} pos_label=0;}
		if (id == "btn_pos3_write") {for(int k=0;k<9;k++){pos3[k]=target[k];} pos_label=0;}
		if (id == "btn_pos4_write") {for(int k=0;k<9;k++){pos4[k]=target[k];} pos_label=0;}
		if (id == "btn_pos5_write") {for(int k=0;k<9;k++){pos5[k]=target[k];} pos_label=0;}
		if (id == "btn_pos6_write") {for(int k=0;k<9;k++){pos6[k]=target[k];} pos_label=0;}
		// read positions buttons
		if (id == "btn_pos1_read") {for(int k=0;k<9;k++){target[k]=pos1[k];} pos_label=0;}
		if (id == "btn_pos2_read") {for(int k=0;k<9;k++){target[k]=pos2[k];} pos_label=0;}
		if (id == "btn_pos3_read") {for(int k=0;k<9;k++){target[k]=pos3[k];} pos_label=0;}
		if (id == "btn_pos4_read") {for(int k=0;k<9;k++){target[k]=pos4[k];} pos_label=0;}
		if (id == "btn_pos5_read") {for(int k=0;k<9;k++){target[k]=pos5[k];} pos_label=0;}
		if (id == "btn_pos6_read") {for(int k=0;k<9;k++){target[k]=pos6[k];} pos_label=0;}
		// show positions buttons
		if (id == "btn_pos1_show") {pos_label=1;}
		if (id == "btn_pos2_show") {pos_label=2;}
		if (id == "btn_pos3_show") {pos_label=3;}
		if (id == "btn_pos4_show") {pos_label=4;}
		if (id == "btn_pos5_show") {pos_label=5;}
		if (id == "btn_pos6_show") {pos_label=6;}
		if (id == "btn_pos0_show") {pos_label=0;}
		if (id == "btn_pos7_show") {pos_label=7;}
			// conditionals
		if (mode) {						// (set&send)
		    //UART(121210);
			// mode buttons
			if (id == "btnmode") mode=!mode;
			if (id == "btnsend") {for(int k=0;k<9;k++){target[k]=temp[k];}}
			// jog buttons temp
			if (id == "btn11") temp[1]+=step4;
			if (id == "btn12") temp[1]+=step3;
			if (id == "btn13") temp[1]+=step2;
			if (id == "btn14") temp[1]+=step1;
			if (id == "btn15") temp[1]-=step1;
			if (id == "btn16") temp[1]-=step2;
			if (id == "btn17") temp[1]-=step3;
			if (id == "btn18") temp[1]-=step4;
			
			if (id == "btn21") temp[2]+=step4;
			if (id == "btn22") temp[2]+=step3;
			if (id == "btn23") temp[2]+=step2;
			if (id == "btn24") temp[2]+=step1;
			if (id == "btn25") temp[2]-=step1;
			if (id == "btn26") temp[2]-=step2;
			if (id == "btn27") temp[2]-=step3;
			if (id == "btn28") temp[2]-=step4;
			
			if (id == "btn31") temp[3]+=step4;
			if (id == "btn32") temp[3]+=step3;
			if (id == "btn33") temp[3]+=step2;
			if (id == "btn34") temp[3]+=step1;
			if (id == "btn35") temp[3]-=step1;
			if (id == "btn36") temp[3]-=step2;
			if (id == "btn37") temp[3]-=step3;
			if (id == "btn38") temp[3]-=step4;
			
			if (id == "btn51") temp[5]+=step4;
			if (id == "btn52") temp[5]+=step3;
			if (id == "btn53") temp[5]+=step2;
			if (id == "btn54") temp[5]+=step1;
			if (id == "btn55") temp[5]-=step1;
			if (id == "btn56") temp[5]-=step2;
			if (id == "btn57") temp[5]-=step3;
			if (id == "btn58") temp[5]-=step4;
			
			if (id == "btn61") temp[6]+=step4;
			if (id == "btn62") temp[6]+=step3;
			if (id == "btn63") temp[6]+=step2;
			if (id == "btn64") temp[6]+=step1;
			if (id == "btn65") temp[6]-=step1;
			if (id == "btn66") temp[6]-=step2;
			if (id == "btn67") temp[6]-=step3;
			if (id == "btn68") temp[6]-=step4;
		} else {						// (contuinous sending)
		    //UART(121220);
			if (id == "btnmode") {mode=!mode; for(int k=0;k<9;k++){temp[k]=target[k];}}
			// jog buttons target
			if (id == "btn11") target[1]+=step4;
			if (id == "btn12") target[1]+=step3;
			if (id == "btn13") target[1]+=step2;
			if (id == "btn14") target[1]+=step1;
			if (id == "btn15") target[1]-=step1;
			if (id == "btn16") target[1]-=step2;
			if (id == "btn17") target[1]-=step3;
			if (id == "btn18") target[1]-=step4;
			
			if (id == "btn21") target[2]+=step4;
			if (id == "btn22") target[2]+=step3;
			if (id == "btn23") target[2]+=step2;
			if (id == "btn24") target[2]+=step1;
			if (id == "btn25") target[2]-=step1;
			if (id == "btn26") target[2]-=step2;
			if (id == "btn27") target[2]-=step3;
			if (id == "btn28") target[2]-=step4;
			
			if (id == "btn31") target[3]+=step4;
			if (id == "btn32") target[3]+=step3;
			if (id == "btn33") target[3]+=step2;
			if (id == "btn34") target[3]+=step1;
			if (id == "btn35") target[3]-=step1;
			if (id == "btn36") target[3]-=step2;
			if (id == "btn37") target[3]-=step3;
			if (id == "btn38") target[3]-=step4;
			
			if (id == "btn51") target[5]+=step4;
			if (id == "btn52") target[5]+=step3;
			if (id == "btn53") target[5]+=step2;
			if (id == "btn54") target[5]+=step1;
			if (id == "btn55") target[5]-=step1;
			if (id == "btn56") target[5]-=step2;
			if (id == "btn57") target[5]-=step3;
			if (id == "btn58") target[5]-=step4;
			
			if (id == "btn61") target[6]+=step4;
			if (id == "btn62") target[6]+=step3;
			if (id == "btn63") target[6]+=step2;
			if (id == "btn64") target[6]+=step1;
			if (id == "btn65") target[6]-=step1;
			if (id == "btn66") target[6]-=step2;
			if (id == "btn67") target[6]-=step3;
			if (id == "btn68") target[6]-=step4;
		}
	}
	//UART(122000);
}