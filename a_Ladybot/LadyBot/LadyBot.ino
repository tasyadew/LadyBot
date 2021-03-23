//Code: VaccumCode
//Version: 2.0.1
//Author: Cesar Nieto refer to ces.nietor@gmail.com
//Last change: 19/05/2017
//Changes: Documentation added
#include <math.h>
////////////PINS////////////////

#define pin1 2 // motor #1 +
#define pin2 3 // motor #1 –
#define pw1 9 // motor #1 pwm
#define pin3 4 // motor #2 +
#define pin4 5 // motor #2 –
#define pw2 6 // motor #2 pwm
void setup() {

  //Stair Sensor
  const int

  //Battery Voltage input
  const int battery = 4;

  //IndicatorLED
  const int led = 13;

  //Fan output
  const int fanmotor =  12;      

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

  //////////////CODE/////////////
  void setup() {
    //Initialize outputs and inputs
    //Fan motor as output
    pinMode(fanmotor, OUTPUT);

    //left motor
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);

    //right motor
    pinMode(pin3, OUTPUT);
    pinMode(pin4, OUTPUT);

    //control motor speed
    pinMode(pw1, OUTPUT);
    pinMode(pw2, OUTPUT);

    Serial.begin(9600);
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, LOW);
    digitalWrite(pin4, LOW);
    analogWrite(pw1, 50);
    analogWrite(pw2, 50);
    //LED
    pinMode(led, OUTPUT);
    //INPUTS
    // initialize the pushbutton inputs
    //Bumper
    pinMode(bumper1, INPUT_PULLUP);
    pinMode(bumper2, INPUT_PULLUP);
    //Sensor
    pinMode(SD1, INPUT);
    pinMode(SD2, INPUT);
    pinMode(SD3, INPUT);
    pinMode(SD4, INPUT);
    //Batt
    pinMode(battery, INPUT);
    // Initialize serial
    Serial.begin(9600);
    ///////////////////////////////Wait////////////////////////////////////////
    //Wait about 5 s and initialize fan if voltage ok
    waitBlinking(5, 1); //5 seconds at 1 Hz
    //Crank (initialize the fan because the voltage drops when cranking)
    if (readBattery(battery) > 12.1) {
      digitalWrite(fanmotor, HIGH); //Turn the Fan ON
      delay(1000); //For 1000ms
    }
    else {
      //do nothing Convention
    }
  }
  //////////Functions To Use //////////
  void waitBlinking(int n, int frequency) {
    //blink for n seconds at frequency hz
    for (int i = 1; i <= n; i++) {
      for (int j = 1; j <= frequency; j++) {
        digitalWrite(led, HIGH);
        delay((1000 / frequency) / 2); //Half time on
        digitalWrite(led, LOW);
        delay((1000 / frequency) / 2); //Half time off
      }
    }
  }
  double sdSHARP(int Sensor) {
    //Returns the distance in cm
    double dist = pow(analogRead(Sensor), -0.857); // x to power of y
    return (dist * 1167.9);
  }
  void forwardMotors(int moveTime) {
    //Manipulate direction according the desired movement of the motors
    analogWrite(motor1Pin1, pwmMin);
    analogWrite(motor1Pin2, 0); //PWM value wher 0 = 0% and 255 = 100%
    analogWrite(motor2Pin1, pwmMin);
    analogWrite(motor2Pin2, 0);
    delay(moveTime);
  }
  void rightMotors(int moveTime) {
    analogWrite(motor1Pin1, 0);
    analogWrite(motor1Pin2, pwmMin);
    analogWrite(motor2Pin1, pwmMin);
    analogWrite(motor2Pin2, 0);
    delay(moveTime);
  }
  void leftMotors(int moveTime) {
    analogWrite(motor1Pin1, pwmMin);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor2Pin2, pwmMin + 20);
    delay(moveTime);
  }
  void backwardMotors(int moveTime) {
    analogWrite(motor1Pin1, 0);
    analogWrite(motor1Pin2, pwmMin + 20);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor2Pin2, pwmMin + 20);
    delay(moveTime);
  }
  void stopMotors() {
    analogWrite(motor1Pin1, 0);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor2Pin2, 0);
  }
  float  readBattery(int input) {
    int readInput;
    float voltage;
    readInput = analogRead(input);
    voltage = (((readInput * 4.9) / 1000) * voltageBatCharged ) / 5; // resolution of analog input = 4.9mV per Voltage
    Serial.print(" Battery= ");
    Serial.print(voltage);
    return voltage;
  }
  void batteryControl(int input) {
    //Turn everything off in case the battery is low
    float v_battery;
    v_battery = readBattery(input);
    if (v_battery <= 11.6) { //battery limit of discharge, Don't put this limit lower than  11.1V or you can kill the battery
      control = false;
    }
    else {
      //Do nothing Convention
    }
  }
  /////////////////////////////////////////////////MAIN CODE//////////////////////////////
  void loop() {
  
    bumperState = digitalRead(bumper1);
    //Keep the control of the battery automatically turn the fan off
    //If control = true the battery level is ok, otherwise the battery is low.
    batteryControl(battery); //modifies the variable control of the battery is low

    void loop() {
      bumperState = digitalRead(bumper1);
      bumperState2 = digitalRead(bumper2);

      else if (bumperState == 0) {            //BUMPER1
        counter = 0;
        backwardMotors(1500); //backward delay of 500ms
        leftMotors(400);
        Serial.print(" Turn Left ");
      }

      else if (bumperState2 == 0) {           //BUMPER2
        counter2 = 0;
        backwardMotors(1500); //backward delay of 500ms
        rightMotors(400);
        Serial.print(" Turn Right ");
      }

      if (Serial.available() >= 2 ){
        unsigned int a = Serial.read();
        unsigned int b = Serial.read();
        unsigned int val = (b * 256) + a;

        if (val == 100){ // motor 1 reverse
          digitalWrite(pin1, LOW);
          digitalWrite(pin2, HIGH);

        }
        else if (val == 200){          // motor #1 stop
          digitalWrite(pin1, LOW);
          digitalWrite(pin2, LOW);
        }
        else if (val == 300){          // motor #1 forward
          digitalWrite(pin1, HIGH);
          digitalWrite(pin2, LOW);
        }
        else if (val == 400){          // motor #2 reverse
          digitalWrite(pin3, LOW);
          digitalWrite(pin4, HIGH);
        }
        else if (val == 500){          // motor #2 stop    
          digitalWrite(pin3, LOW);
          digitalWrite(pin4, LOW);
        }
        else if (val == 600){          // motor #2 forward
          digitalWrite(pin3, HIGH);
          digitalWrite(pin4, LOW);
        }
        else if (val >= 1000 && val <= 1255){        
          analogWrite (pw1, val – 1000);
        }
        else if (val >= 2000 && val <= 2255){
          analogWrite (pw2, val – 2000);
        }
      }
    }
  }
  else if (!control) {
    //If the battery is low, turn everything off
    digitalWrite(fanmotor, LOW); //Turn the Fan OFF
    stopMotors();
    Serial.print(" Low Battery! ");
    Serial.println();
    waitBlinking(1, 3); //blink as warning 3hz in a loop
  }
  Serial.println();
}
