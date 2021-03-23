
#include <math.h> 
#include <Wire.h>


////////////PINS////////////////

//Battery Voltage input
const int battery = 4;         //analog 4

//Fan output
const int fanmotor =  12;      // the number of the LED pin

// Motor1 Right
const int motor1Pin1 = 4;
const int motor1Pin2 = 5;

// Motor2 Left
const int motor2Pin1 = 6;
const int motor2Pin2 = 7;

//EnA and EnB
const int enA = 9;
const int enB = 10;

//Bumper
const int bumper1 = 8;
const int bumper2 = 2;


///////////////Constants////////////////
const float voltageBatCharged = 9.68; // Voltage measured when battery fully charged //Change this
//PWM for the micro metal motors
const int pwmMax = 160;// for 12V  pwmMAx = 170, for 10V output  pwmMax = 140
const int pwmMin = 70;;// for 9V  pwmMin = 128


// Variables will change:
int bumperState = 0;  // variable for reading the pushbutton status
boolean control = true;
int counter2 = 0; //   Prevents from being stuck
int bumperState2 = 0; // variable for reading the pushbutton status
boolean control2 = true;
int counter3 = 0; // Prevents from being stuck
int bumperState3 = 0; // variable for reading the pushbutton status
boolean control3 = true;
int counter4 = 0; // Prevents from being stuck
int bumperState4 = 0; // variable for reading the pushbutton status
boolean control4 = true;
int counter5 = 0; // Prevents from being stuck


//////////////CODE/////////////
void setup() {
  //Initialize outputs and inputs
  //Fan motor as output
  pinMode(fanmotor, OUTPUT);
  //Motor1
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  //Motor2
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  
  //INPUTS
  // initialize the pushbutton inputs 
  //Bumper
  pinMode(bumper1, INPUT_PULLUP); 
  pinMode(bumper2, INPUT_PULLUP); 
  
  //Battery
  pinMode(battery, INPUT);
  // Initialize serial
  Serial.begin(9600);    
  Wire.begin();
  
  
  ///////////////////////////////Wait////////////////////////////////////////
  //Wait about 5 s and initialize fan if voltage ok

  if(readBattery(battery)>9.1){
    digitalWrite(fanmotor, HIGH); //Turn the Fan ON
    delay(1000); //For 1000ms
  }
  else {
    //do nothing Convention
    }
} 
//////////Functions To Use //////////


void forwardMotors(int moveTime){  
  //Manipulate direction according the desired movement of the motors
   analogWrite(motor1Pin1, pwmMin); 
   analogWrite(motor1Pin2, 0); //PWM value wher 0 = 0% and 255 = 100%
   analogWrite(motor2Pin1, pwmMin); 
   analogWrite(motor2Pin2, 0); 
   delay(moveTime);
}
void rightMotors(int moveTime){ 
   analogWrite(motor1Pin1, 0); 
   analogWrite(motor1Pin2, pwmMin); 
   analogWrite(motor2Pin1, pwmMin);
   analogWrite(motor2Pin2, 0); 
   delay(moveTime);
}
void leftMotors(int moveTime){ 
   analogWrite(motor1Pin1, pwmMin); 
   analogWrite(motor1Pin2, 0); 
   analogWrite(motor2Pin1, 0);
   analogWrite(motor2Pin2, pwmMin+20); 
   delay(moveTime);
}
void backwardMotors(int moveTime){
   analogWrite(motor1Pin1, 0); 
   analogWrite(motor1Pin2, pwmMin+20);
   analogWrite(motor2Pin1, 0); 
   analogWrite(motor2Pin2, pwmMin+20); 
   delay(moveTime);
}
void stopMotors(){ 
   analogWrite(motor1Pin1, 0);
   analogWrite(motor1Pin2, 0); 
   analogWrite(motor2Pin1, 0); 
   analogWrite(motor2Pin2, 0); 
}
float  readBattery(int input){
  int readInput;
  float voltage;
  readInput = analogRead(input);
  voltage = (((readInput*4.9)/1000)*voltageBatCharged ) / 5; // resolution of analog input = 4.9mV per Voltage 
  Serial.print(" Battery= ");
  Serial.print(voltage);
  return voltage;
  } 
void batteryControl(int input){
  //Turn everything off in case the battery is low
  float v_battery;
  v_battery = readBattery(input);
  if(v_battery<=11.6){ //battery limit of discharge, Don't put this limit lower than  11.1V or you can kill the battery
    control = false;
    }
  else {
    //Do nothing Convention
    }
}

////////////////////////MAIN CODE///////////////////////////

void loop(){
   
   if(BluetoothData=='1'){   // if manual button push
   void manual = TRUE ;
   }
  if (BluetoothData=='0'){// if aoto button
  void auto += TRUE
  
  }
}
}




