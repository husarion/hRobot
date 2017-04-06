#define NO_MIN_MAX
#define END_SWITCH_ENABLE

#ifndef HROBOTCONFIG
#define HROBOTCONFIG

//min max volues for soft stops and ofset
#ifdef NO_MIN_MAX
const float J1_min = -360.0;
const float J2_min = -360.0;
const float J3_min = -360.0;
const float J4_min = -360.0;
const float J5_min = -360.0;

const float J1_max = 360.0;
const float J2_max = 360.0;
const float J3_max = 360.0;
const float J4_max = 360.0;
const float J5_max = 360.0;
#else
const float J1_min = -180.0;
const float J2_min = -120.0;
const float J3_min = -55.0;
const float J4_min = -150.0;
const float J5_min = -180.0;

const float J1_max = 180.0;
const float J2_max = 30.0;
const float J3_max = 170.0;
const float J4_max = 105.0;
const float J5_max = 180.0;
#endif

//Interpolation setings
const int time_motion_task = 100;
const float time_iteration = 75;
const float step_mul = 10;

//enkoder tics for revolution scale
const float encoder_tics_J1 = 20;
const float encoder_tics_J2 = 18 * 4;
const float encoder_tics_J3 = 20 / 4.355;
const float encoder_tics_J5 = 4.4;
const float encoder_tics_J6 = 0.2;

//endswitch enable
#ifdef END_SWITCH_ENABLE
bool endswitch_active = true;
#else
bool endswitch_active = false;
#endif

//Servos names and settings
IServo &s1 = hServoModule.servo1;
IServo &s2 = hServoModule.servo2;
IServo &s3 = hServoModule.servo3;
IServo &s4 = hServoModule.servo4;
IServo &h1 = hServoModule.servo5;

const float J1_middle = 1125;
const float J3_middle = 1600;
const float J5_middle = 1730;
const float J6_middle = 1470;

const float J1_threshold = 2;
const float J2_threshold = 0;
const float J3_threshold = 0;
const float J5_threshold = 0;
const float J6_threshold = 7.5;

const float J1_kp_down = 0.9;
const float J1_ki_down = 0.4;
const float J1_kd_down = 0.2;
const float J1_kp_up = 0.9;
const float J1_ki_up = 0.4;
const float J1_kd_up = 0.2;
const float J1_error_saturate = 100;
const float J1_integrator_saturate_down = 20;
const float J1_integrator_saturate_up = 20;

const float J2_kp_down = 15;
const float J2_ki_down = 1;
const float J2_kd_down = 9.5;
const float J2_kp_up = 4.25;
const float J2_ki_up = 0.8;
const float J2_kd_up = 11;
const float J2_error_saturate = 100;
const float J2_integrator_saturate_down = 20;
const float J2_integrator_saturate_up = 17;

const float J3_kp_down = 3;
const float J3_ki_down = 1.8;
const float J3_kd_down = 1.5;
const float J3_kp_up = 2;
const float J3_ki_up = 1.1;
const float J3_kd_up = 2;
const float J3_error_saturate = 100;
const float J3_integrator_saturate_down = 4;
const float J3_integrator_saturate_up = 4;

const float J5_kp_down = 2.4;
const float J5_ki_down = 0.2;
const float J5_kd_down = 0.8;
const float J5_kp_up = 2.8;
const float J5_ki_up = 0.2;
const float J5_kd_up = 0.5;
const float J5_error_saturate = 100;
const float J5_integrator_saturate_down = 20;
const float J5_integrator_saturate_up = 20;

const float J6_kp_down = 2;
const float J6_ki_down = 0;
const float J6_kd_down = 0.5;
const float J6_kp_up = 2;
const float J6_ki_up = 0;
const float J6_kd_up = 0.5;
const float J6_error_saturate = 100;
const float J6_integrator_saturate_down = 20;
const float J6_integrator_saturate_up = 20;

#endif