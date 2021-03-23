  void manual(){
  /*  
  Serial.print("SD1= ");
  Serial.print(sdSHARP(SD1));
  Serial.println();
  Serial.print("  SD2= ");
  Serial.print(sdSHARP(SD2));
  Serial.println();
  delay(200);*/
  bumperState = digitalRead(bumper1);
  bumperState2 = digitalRead(bumper2);
  bumperState3 = digitalRead(bumper3);
  bumperState4 = digitalRead(bumper4);
  //Keep the control of the battery automatically turn the fan off
  //If control = true the battery level is ok, otherwise the battery is low.
  batteryControl(battery); //modifies the variable control of the battery is low
    
  if (control){
    
      if (counter2 ==2){ 
        // prevent of being stuck on corners
        counter2 = 0;
        }
      else {
        //Do nothing Convention
      }
      forwardMotors(100); // approach a bit
      backwardMotors(500); // backward delay of 500ms
      leftMotors(300);
      counter2 = counter2 + 2;
      Serial.print("  Turn Left s1");
      }
    else if (sdSHARP(SD2)<=4.3){ 
      //If the distance between an object and the right front sensor is less than 4.3 cm, it will move to the right
      if (counter2 ==1){
        counter2 = 0;
        }
      else{
        //Do nothing Convention
      }
      forwardMotors(100);
      backwardMotors(500);
      rightMotors(300);
      counter2++;
      Serial.print("  Turn Right s2");
      }
    else if (bumperState==0){//BUMPER1
      counter2 = 0;
      backwardMotors(1500); //backward delay of 500ms
      leftMotors(400);
      Serial.print("  Turn Left b1");
      }
    else if (bumperState2==0){//BUMPER2
      counter3 = 0;
      backwardMotors(1600); //backward delay of 500ms
      rightMotors(450);
      Serial.print("  Turn Right b2");
      }
      
    else if (bumperState3==0){//BUMPER3
      counter4 = 0;
      backwardMotors(1550); //backward delay of 500ms
      leftMotors(480);
      Serial.print("  Turn Left b3");
      }
    else if (bumperState2==0){//BUMPER4
      counter2 = 4;
      backwardMotors(1590); //backward delay of 500ms
      rightMotors(490);
      Serial.print("  Turn Right b4");
      }
    else {
      if(counter2==3){ //Corner
        leftMotors(1000);
        counter2 = 0;
        }
      else {
        forwardMotors(0);
      }
      Serial.print("  Move Forward");
      }
  

mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  vx = (gx+300)/200;  // "+300" because the x axis of gyroscope give values about -350 while it's not moving. Change this value if you get something different using the TEST code, chacking if there are values far from zero.
  vy = -(gz-100)/200; // same here about "-100"
  
    Serial.print("gx = ");
  Serial.print(gx);
  Serial.print(" | gz = ");
  Serial.print(gz);
  
  Serial.print("        | X = ");
  Serial.print(vx);
  Serial.print(" | Y = ");
  Serial.println(vy);

  
if ( (vx_prec-20)<=vx && vx<=vx_prec+20 && (vy_prec-20)<=vy && vy<=vy_prec+20) { // checking the pointer doesn't move too much from its actual position: (in this case a 10 pixel square)
    count++;                                                                  
    if(count == 100){ // the click will happen after 2 seconds the pointer has stopped in the 10px square: 20ms of delay 100 times it's 2000ms = 2s
        backwardMotors(1200);
        leftMotors(500);
        count = 0;
        Serial.print("  Turn Left gpu");
      
    }
  }
  else {
    vx_prec = vx; // updating values to check the position of the pointer and allow the click
    vy_prec = vy;
    count = 0;
    }
    delay(20);
  }
  else if (!control){
    //If the battery is low, turn everything off
    digitalWrite(fanmotor, LOW); //Turn the Fan OFF
    stopMotors();
    Serial.print(" Low Battery! ");
    Serial.println();
    waitBlinking(1,3);  //blink as warning 3hz in a loop
    }
  Serial.println();

  }
}
