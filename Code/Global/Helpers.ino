//void turnLeft(double deg){
//  double rad = deg*PI/180;
//
//  ActiveLeftMotor->setPos(0);
//  ActiveRightMotor->setPos(0);
//  
//  ActiveLeftMotor->PIDPos(kp_Lpos, ki_Lpos, kd_Lpos, 0);
//  ActiveRightMotor->PIDPos(kp_Rpos, ki_Rpos, kd_Rpos, 2*PI*wheelSpacing*rad);
//}

void turnLeft(double speed){
  ActiveRightMotor->PIDVel(kp_Lvel, ki_Lvel, kd_Lvel, speed);
}
//void turnRight(double deg){
//  double rad = deg*PI/180;
//  
//  ActiveLeftMotor->setPos(0);
//  ActiveRightMotor->setPos(0);
//  
//  ActiveLeftMotor->PIDPos(kp_Lpos, ki_Lpos, kd_Lpos, 2*PI*wheelSpacing*rad);
//  ActiveRightMotor->PIDPos(kp_Rpos, ki_Rpos, kd_Rpos, 0);
//}

void turnRight(double speed){
   ActiveLeftMotor->PIDVel(kp_Lvel, ki_Lvel, kd_Lvel, speed);
}

void moveForward(double speed){
//  ActiveLeftMotor->setPos(0);
//  ActiveRightMotor->setPos(0);
  
  ActiveLeftMotor->PIDVel(kp_Lvel*0.8, ki_Lvel, kd_Lvel, speed);
  ActiveRightMotor->PIDVel(kp_Rvel*0.8, ki_Rvel, kd_Rvel, speed);
}

void moveBackward(double speed){
//  ActiveLeftMotor->setPos(0);
//  ActiveRightMotor->setPos(0);
  
  ActiveLeftMotor->PIDVel(kp_Lvel*0.8, ki_Lvel, kd_Lvel, -speed);
  ActiveRightMotor->PIDVel(kp_Rvel*0.8, ki_Rvel, kd_Rvel, -speed);
}

void stopMoving(){
  ActiveLeftMotor->pause();
  ActiveRightMotor->pause();
  BLinAct.pause();
  FLinAct.pause();
}

void actuateUp(){
  BLinAct.goUp();
  FLinAct.goUp();
}

void actuateDown(){
  BLinAct.goDown();
  FLinAct.goDown();
}

bool isStairInFront(){
  int USreadDiff=BackUS.read()-FrontMiddleUS.read();
  int USreadDiff2=Upper2US.read()-FrontMiddleUS.read();
  bool stair;
  if(USreadDiff <= 60 && USreadDiff >=10 && FrontMiddleUS.read() < 50){
    stair = true;
  }else if(USreadDiff2 <= 60 && USreadDiff2 >=10 && FrontMiddleUS.read() < 50){
    stair = true;
  }else{
    stair = false;
  }
  return stair;
}

void startStairClimb(){
 
  Serial.println("in climb function");
  if(isStairInFront()){
     calibrate();
  phaseTime=millis();
//    ActiveLeftMotor->goForwardUntil(80, &FrontMiddleUS, "<", 5);
//    ActiveRightMotor->goForwardUntil(60, &FrontMiddleUS, "<", 5);
    Serial.println("starting climb");
    setActiveMotor("F");
    delay(100);
    ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, 0.1);
    ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, 0.1);
//  ActiveLeftMotor->PIDVel(5000,0.1,0, 0.1); //0.25 is max speed on back
//  ActiveRightMotor->PIDVel(3000,0.1,0, 0.1);
    stairMode=1;
    delay(1);
//    phaseShift(0);
  }else{
    //do nothing
    Serial.println("not climbing");
  }
}

int readCompass(){
    if(millis()-compassLastTime > 200){
      compass.readHeading();
      compassLastTime=millis();
    }
    return compass.pitch;
}

int readCompassHeading(){
   if(millis()-compassLastTime > 200){
      compass.readHeading();
      compassLastTime=millis();
    }
    return compass.heading;
}

void levelLowerPlatform(int maxDegTenths){
  int pitch = -readCompass();
  if(pitch < -1*maxDegTenths){
    FLinAct.pause();
  }else{
    FLinAct.goDown();
  }
  if(pitch > maxDegTenths){
      BLinAct.pause();
    }else{
      BLinAct.goDown();
    }
}

void levelRaisePlatform(int maxDegTenths){
  int pitch = -readCompass();
  if(pitch > maxDegTenths){
    FLinAct.pause();
  }else{
    FLinAct.goUp();
  }
  if(pitch < -1*maxDegTenths){
      BLinAct.pause();
    }else{
      BLinAct.goUp();
    }
}

void levelPlatformByLowering(){
  bool flag=false;
  FLinAct.turnOn();
  BLinAct.turnOn();
  while(!flag){
    int pitch = -readCompass();
    Serial.println(pitch);
    if(pitch>0){
      FLinAct.goDown();
    }else if(pitch<0){
      BLinAct.goDown();
    }else{
      flag=true;
    }
    FLinAct.turnOff();
  BLinAct.turnOn();
  }
}

void checkFullRaise(){
  if(!BackLeftLinUpperTrip.read() && !BackRightLinUpperTrip.read()){
    BLinAct.pause();
    BLinAct.turnOff();
    Serial.println("left fully raised");
  }
  if(!FrontLeftLinUpperTrip.read() && !FrontRightLinUpperTrip.read()){
    FLinAct.pause();
    FLinAct.turnOff();
    Serial.println("right fully raised");
  }
}

void checkFullLower(){
  if(BackLinLowerTrip.read()){
    BLinAct.pause();
    BLinAct.turnOff();
  }
  if(FrontLinLowerTrip.read()){
    FLinAct.pause();
    FLinAct.turnOff();
  }
}

void orientOnStair(){
// int targetHeading=stairHeading;
  int targetHeading=905;
  setActiveMotor("F");
  bool flag = false;
  bool flag2 = false;
  while(abs(readCompassHeading()-targetHeading)>3){
//    Serial.println(BackLeftIR.read());
    FLMotor.internalUpdate();
    FRMotor.internalUpdate();
    if(readCompassHeading()>targetHeading){ // we're too far clockwise
      if(FrontLeftIR.read() <12 && FrontRightIR.read() <12){
//        BackLeftIR.read()<12 &&
        if(!flag){
          ActiveRightMotor->pause();
          ActiveLeftMotor->PIDVel(kp_Lvel, ki_Lvel, kd_Lvel, -0.12);
//          Serial.println("moving left");
          
        }else{
          flag=true;
          ActiveLeftMotor->pause();
          if(!flag2){
            ActiveRightMotor->PIDVel(kp_Rvel/2,0,kd_Rvel/2, 0.08);
            flag2=true;
//            Serial.println("moving right");
          }
          Serial.println(FRMotor.getVel());
        }
      }
    }

  }
      turnOffAll();
}

void orientOnStair2(){
  int targetHeading=905;
  calibrate();
  BLinAct.turnOn();
  while(!BackGroundTrip.read()){
    BLinAct.goUp();
  }
  BLinAct.pause();
//  FLinAct.turnOn();
//  while(!Front2GroundTrip.read()){
//    FLinAct.goUp();
//  }
  FLinAct.pause();
  FLinAct.turnOff();
  BLinAct.turnOff();
  setActiveMotor("B");
  FRMotor.turnOn();
  FLMotor.turnOn();
  while(FrontLeftIR.read() <12 && FrontRightIR.read() <12 && (!FrontLeftWallTrip.read() && !FrontRightWallTrip.read())){
    BLMotor.internalUpdate();
    BRMotor.internalUpdate();
    FRMotor.internalUpdate();
    FLMotor.internalUpdate();
    ActiveLeftMotor->PIDVel(kp_Lvel, ki_Lvel, kd_Lvel, 0.08);
    ActiveRightMotor->PIDVel(kp_Rvel, ki_Rvel, kd_Rvel, 0.08);
    FRMotor.PIDVel(kp_FRvel, ki_FRvel, kd_FRvel, 0.08);
    FLMotor.PIDVel(kp_FLvel, ki_FLvel, kd_FLvel, 0.10);
  }
  ActiveLeftMotor->pause();
  ActiveRightMotor->pause();
  FRMotor.pause();
  FLMotor.pause();
 delay(1000);
 phaseTime=millis();
  while(abs(readCompassHeading()-targetHeading)>3){
    Serial.println(readCompassHeading()-targetHeading);
    BLMotor.internalUpdate();
    BRMotor.internalUpdate();
    FRMotor.internalUpdate();
//     Serial.println(BackLeftIR.read());
    if(BackGroundTrip.read()){
      if(readCompassHeading()>targetHeading){ // we're too far clockwise
//          if(1){
            ActiveLeftMotor->PIDVel(kp_Lvel, ki_Lvel, kd_Lvel, -0.08);
            ActiveRightMotor->PIDVel(kp_Rvel, ki_Rvel, kd_Rvel, 0.08);
            FRMotor.PIDVel(kp_FRvel, ki_FRvel, kd_FRvel, 0.08);
            FRMotor.PIDVel(kp_FRvel, ki_FRvel, kd_FRvel, 0.08);
//            FRMotor.pause();
//          }
//          else{
//            ActiveRightMotor->PIDVel(kp_Rvel, ki_Rvel, kd_Rvel, 0.1);
//            FRMotor.PIDVel(kp_FRvel, ki_FRvel, kd_FRvel, 0.1);
//            ActiveLeftMotor->pause();
//          }
      }else{
//        Serial.println("straigtening");
         ActiveLeftMotor->PIDVel(kp_Lvel, ki_Lvel, kd_Lvel, 0.08);
            ActiveRightMotor->PIDVel(kp_Rvel, ki_Rvel, kd_Rvel, -0.08);
            FRMotor.PIDVel(kp_FRvel, ki_FRvel, kd_FRvel, -0.08);
            FLMotor.PIDVel(kp_FLvel+300, ki_FLvel, kd_FLvel, 0.1);
      }
//    }else{
//      setActiveMotor("F");
//    }
    }else{
      break;
    }
    if((abs(readCompassHeading()-targetHeading)<20) && millis()-phaseTime > 10000){
      break;
    }
  }
//  Serial.println(readCompassHeading()-targetHeading);
  turnOffAll();
  delay(1000);
  phaseShift(11);
}

void calibrate(){
  stairHeading=readCompassHeading();
  BLinAct.turnOn();
  FLinAct.turnOn();
//  Serial.println("switches");
//  Serial.println(!BackLinLowerTrip.read());
//  Serial.println(!FrontLinLowerTrip.read());
//  Serial.println(!BackLinLowerTrip.read() || !FrontLinLowerTrip.read());
//  Serial.println("");
  while(!BackLinLowerTrip.read() || !FrontLinLowerTrip.read()){
    checkFullLower();
    levelLowerPlatform(15);
  }
    BLinAct.turnOff();
  FLinAct.turnOff();
}

void readIrRemoteSafe(){
  //if no command in last IR_cutoff_interval, stop moving
//  Serial.println("in IR");
  if (millis() - lastTime > IR_cutoff_interval) {
//    stopMoving();
  }
  if(irrecv.decode()){
    lastTime=millis();
    int command = irrecv.decodedIRData.command;
      Serial.println(command);
    switch(command){
      case 69: //E-stop
        killEverything();
        break;
    }
    irrecv.resume();
  }
}

void readIrRemote(){
  //if no command in last IR_cutoff_interval, stop moving
//  Serial.println("in IR");
//Serial.println(millis() - lastTime);
  if (millis() - lastTime > IR_cutoff_interval && IR_action_has_timeout) {
    stopMoving();
    turnOffAll();
//    Serial.println("cutoff");
    delay(100);
  }
//    if(millis() - lastTime2 > IR_cutoff_interval*3){
////    lastTime=millis();
//    turnOffAll();
//    Serial.println("safety cutoff");
//    delay(100);
//      }
//  Serial.println(millis()-lastTime);
  if(irrecv.decode()){
    
    int command = irrecv.decodedIRData.command;
    Serial.println(command);
//    if(command != 0){
      lastTime=millis();
//    }else{
      if(command !=0){
        lastTime2=millis();
      }

//    }
    switch(command){
      case 69: //E-stop
        killEverything();
        IR_action_has_timeout=false;
        break;
      case 70: //Go Forward
      turnOffAll();
        setActiveMotor("B");
        moveForward(0.12);
        IR_action_has_timeout=true;
        IRLoopMode=1;
        break;
      case 67: //Go Right
      turnOffAll();
      setActiveMotor("B");
        turnRight(0.12);
        IR_action_has_timeout=true;
        IRLoopMode=2;
        break;
      case 68: //Go Left
      turnOffAll();
      setActiveMotor("B");
        turnLeft(0.12);
        IR_action_has_timeout=true;
        IRLoopMode=3;
        break;
      case 21: //Go Backwards
      turnOffAll();
        setActiveMotor("B");
        moveBackward(0.12);
        IR_action_has_timeout=true;
        IRLoopMode=4;
        break;
      case 64: //Hold action
      turnOffAll();
        stopMoving();
        IR_action_has_timeout=true;
        break;
      case 7:
      turnOffAll();
        startStairClimb();
        IR_action_has_timeout=false;
        IRLoopMode=0;
        break;
      case 9: 
      turnOffAll();
        orientOnStair2();
        IR_action_has_timeout=false;
        IRLoopMode=0;
        break;
      case 11://Actuate up
      turnOffAll();
        FLinAct.turnOn();
        FLinAct.goUp();
        BLinAct.turnOn();
        BLinAct.goUp();
        levelRaisePlatform(15);
        break;
      case 71: //Actuate down
      turnOffAll();
        FLinAct.turnOn();
        FLinAct.goDown();
        BLinAct.turnOn();
        BLinAct.goDown();
        levelLowerPlatform(15);
        break;
//      case 78: //Lower Front
//        FLinAct.goDown();
//        break;
//      case 88: //Lower Back
//        BLinAct.goDown();
//        break;
    }
    irrecv.resume();
  }
  switch(IRLoopMode){
    case 0: //nothing
      break;
    case 1: //front movement
      if(FrontMiddleUS.read()<6){
        turnOffAll();
        Serial.print("Mid");
        Serial.println(FrontMiddleUS.read());
      }
      if(FrontRightUS.read()<6){
        turnOffAll();
        Serial.print("Right");
        Serial.println(FrontRightUS.read());
      }
      if(FrontLeftUS.read()<6){
        turnOffAll();
        Serial.print("left");
        Serial.println(FrontLeftUS.read());
      }
      if(FrontLeftIR.read()>12){
        turnOffAll();
      }
      if(FrontRightIR.read()>12){
        turnOffAll();
      }
//Serial.println("IR 1");
      break;
    case 2: //right movement
      if(FrontMiddleUS.read()<6){
        turnOffAll();
      }
      if(FrontRightUS.read()<6){
        turnOffAll();
      }
      if(FrontLeftIR.read()>12){
        turnOffAll();
      }
      if(FrontRightIR.read()>12){
        turnOffAll();
      }
      break;
    case 3: //left movement
      if(FrontMiddleUS.read()<6){
        turnOffAll();
      }
      if(FrontLeftUS.read()<6){
        turnOffAll();
      }
      if(FrontLeftIR.read()>12){
        turnOffAll();
      }
      if(FrontRightIR.read()>12){
        turnOffAll();
      }
      break;
    case 4: //backwards movement
      if(BackLeftIR.read()>12){
        turnOffAll();
      }
      if(BackRightIR.read()>12){
        turnOffAll();
      }
      break;
    case 5: //upwards movement
      checkFullRaise();
      levelRaisePlatform(15);
      break;
    case 6: //downwards movement
      levelLowerPlatform(15);
      checkFullLower();
      break;
  }
}
void setActiveMotor(String mode){
  if(mode.compareTo("F")==0){
    ActiveLeftMotor = &FLMotor;
    ActiveRightMotor = &FRMotor;

    BLMotor.turnOff();
    BRMotor.turnOff();
    FLMotor.turnOn();
    FRMotor.turnOn();
    
    //changed gains by active wheels
    //Left
    kp_Lpos = kp_FLpos;
    ki_Lpos = ki_FLpos;
    kd_Lpos = kd_FLpos;
    
    kp_Lvel = kp_FLvel;
    ki_Lvel = ki_FLvel;
    kd_Lvel = kd_FLvel;
    
    //Right
    kp_Rpos = kp_FRpos;
    ki_Rpos = ki_FRpos;
    kd_Rpos = kd_FRpos;
    
    kp_Rvel = kp_FRvel;
    ki_Rvel = ki_FRvel;
    kd_Rvel = kd_FRvel;

  }else if(mode.compareTo("B")==0){
    ActiveLeftMotor = &BLMotor;
    ActiveRightMotor = &BRMotor;

    //changed gains by active wheels
    //Left
    kp_Lpos = kp_BLpos;
    ki_Lpos = ki_BLpos;
    kd_Lpos = kd_BLpos;
    
    kp_Lvel = kp_BLvel;
    ki_Lvel = ki_BLvel;
    kd_Lvel = kd_BLvel;
    
    //Right
    kp_Rpos = kp_BRpos;
    ki_Rpos = ki_BRpos;
    kd_Rpos = kd_BRpos;
    
    kp_Rvel = kp_BRvel;
    ki_Rvel = ki_BRvel;
    kd_Rvel = kd_BRvel;

    FLMotor.turnOff();
    FRMotor.turnOff();
    BLMotor.turnOn();
    BRMotor.turnOn();
  }
}

void resetMotorPositions(){
  BLMotor.setPos(0);
  BRMotor.setPos(0);
  FLMotor.setPos(0);
  FRMotor.setPos(0);
}

void turnOffAll(){
  BLMotor.turnOff();
  BRMotor.turnOff();
  FLMotor.turnOff();
  FRMotor.turnOff();
  BLinAct.turnOff();
  FLinAct.turnOff();
//  Serial.println("All off");
}

void killEverything(){
  turnOffAll();
  noInterrupts();
  while(true){
    //do nothing
  }
}
