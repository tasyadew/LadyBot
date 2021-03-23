//Robot vacuum controlled via Bluetooth
//Arduino Robot Car Wireless Control using the HC-05 Bluetooth
//Include libraries
#include <math.h> 

//Defining pins
#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 6
#define in4 7
////////////PINS////////////////
//Battery Voltage input
const int battery = 4;

//Fan output
const int fanmotor =  12;      // the number of the LED pin

// Motor1 Right
const int motor1Pin1 = 3;
const int motor1Pin2 = 5;

// Motor2 Left
const int motor2Pin1 = 6;
const int motor2Pin2 = 9;

//Bumper
const int bumper1 = 10;
const int bumper2 = 11;

///////////////Constants////////////////
const float voltageBatCharged = 12.68; // Voltage measured when battery fully charged //Change this
//PWM for the micro metal motors
const int pwmMax = 160;// for 12V  pwmMAx = 170, for 10V output  pwmMax = 140
const int pwmMin = 70;;// for 9V  pwmMin = 128
//Mínimun distance of the sensor
const int minSharp = 30;

// Variables will change:
int bumperState = 0;  // variable for reading the pushbutton status
boolean control = true;
int counter = 0; //   Prevents from being stuck
