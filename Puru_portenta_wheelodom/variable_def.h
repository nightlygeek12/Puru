/**********************************************************
*VARIABLE DEFINITION
************************************************************/


volatile long pulsesA, pulsesB;
volatile long previous_pulseA, previous_pulseB;
volatile int pulse_per_periodA, pulse_per_periodB;
int period = 10;
float resolutionRight = 3955; // Encoder 2965
float resolutionLeft = 3955; // Encoder  3955
float period_to_minute = (1000/period) * 60;
float rpm_left = 0;
float rpm_right = 0;
float speed_wheel_left = 0.0;
float speed_wheel_right= 0.0;
float wheel_rad=0.06;
float elapsedTime, timePrev, timeNow, timeCurrent;
float velocity=0.0;
//float time_y=millis();
float deltaT_Left, deltaT_Right,prevT_Left,prevT_Right;
float currT;
double PPS_Left, PPS_Right;

/*******************************************************
 * Joystick variables
 *******************************************************/
float x_axis;
float z_axis;
int vel,turn;

/*******************************************************\
 * Motor Variable declaration
\*******************************************************/

float speedR,speedL,pwmR,pwmL;

/*****************************************************\
 * PI definition
\*****************************************************/
#define PI 3.142857
