//Don't use delays in this file

void phaseShift(int newPhase){
//  turnOffAll();
//  resetMotorPositions();
phase=newPhase;
  Serial.print("shifting to ");
  Serial.println(phase);
  switch(newPhase){
    case 0:
      setupPhaseZero();
      break;
    case 1:
      setupPhaseOne();
      break;
    case 2:
      setupPhaseTwo();
      break;
    case 3:
      setupPhaseThree();
      break;
    case 4:
      setupPhaseFour();
      break;
    case 5:
      setupPhaseFive();
      break;
    case 6:
      setupPhaseSix();
      break;
    case 7:
      setupPhaseSeven();
      break;
    case 8:
      setupPhaseEight();
      break;
    case 11:
      setupPhaseEleven();
      break;
    case 12:
      setupPhaseTwelve();
      break;
    case 13:
      setupPhaseThirteen();
      break;
    case 14:
      setupPhaseFourteen();
      break;
    case 15:
      setupPhaseFifteen();
      break;
    case 16:
      setupPhaseSixteen();
      break;
    case 17:
      setupPhaseSeventeen();
      break;
    case 18:
      setupPhaseEighteen();
      break;
  }
}

void loopPhaseCheck(){

  switch(phase){
    case 0:
      loopPhaseZero();
      readIrRemote();
      break;
    case 1:
      loopPhaseOne();
      readIrRemoteSafe();
      break;
    case 2:
      loopPhaseTwo();
      readIrRemoteSafe();
      break;
    case 3:
      loopPhaseThree();
      readIrRemoteSafe();
      break;
    case 4:
      loopPhaseFour();
      readIrRemoteSafe();
      break;
    case 5:
      loopPhaseFive();
      readIrRemoteSafe();
      break;
    case 6:
      loopPhaseSix();
      readIrRemoteSafe();
      break;
    case 7:
      loopPhaseSeven();
      readIrRemoteSafe();
      break;
    case 8:
      loopPhaseEight();
      readIrRemoteSafe();
      break;
    case 11:
      loopPhaseEleven();
      readIrRemoteSafe();
      break;
    case 12:
      loopPhaseTwelve();
      readIrRemoteSafe();
      break;
    case 13:
      loopPhaseThirteen();
      readIrRemoteSafe();
      break;
    case 14:
      loopPhaseFourteen();
      readIrRemoteSafe();
      break;
    case 15:
      loopPhaseFifteen();
      readIrRemoteSafe();
      break;
    case 16:
      loopPhaseSixteen();
      readIrRemoteSafe();
      break;
    case 17:
      loopPhaseSeventeen();
      readIrRemoteSafe();
      break;
    case 18:
      loopPhaseEighteen();
      readIrRemoteSafe();
      break;
  }
}

// Phase 0 - WIP

void setupPhaseZero(){
//  setActiveMotor("F");
  if(stairMode==0){
    BLinAct.turnOn();
    while(!BackGroundTrip.read()){
      BLinAct.goUp();
    }
    BLinAct.pause();
    delay(200);
    FLinAct.turnOn();
    while(!Front2GroundTrip.read()){
      FLinAct.goUp();
    }
    FLinAct.pause();
    FLinAct.turnOn();
    BLinAct.turnOn();
    unsigned long timer = millis();
    while(millis()-timer < 500){
      levelRaisePlatform(15);
    }
    turnOffAll();
    setActiveMotor("B");

//    setActiveMotor("F");

  }else if(stairMode == 1){
      phaseTime=millis();
  }
}

void loopPhaseZero(){
  if(stairMode==0){
    //do nothing for now
  }else if(stairMode == 1){
//    Serial.println("here check");
    if(FrontMiddleUS.read() < 5){
//      Serial.println("breaking to two");
      ActiveLeftMotor->pause();
      ActiveRightMotor->pause();
      delay(200);
      ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, 0.1);
      ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, 0.1);
      stairMode=2;
      phaseTime=millis();
    }
  }else if(stairMode = 2){
    if(FrontLeftWallTrip.read()){
      ActiveLeftMotor->pause();
    }
    if(FrontRightWallTrip.read()){
      ActiveRightMotor->pause();
    }
    if((FrontLeftWallTrip.read() && FrontRightWallTrip.read()) || millis()-phaseTime >5000){
      phaseShift(1);
      
    }
  }
}
// Phase 1

void setupPhaseOne(){
  //set the correct motors on/off
  turnOffAll();
  BLinAct.turnOn();
  FLinAct.turnOn();
//  setActiveMotor("F");
  
  //LinActAction
//  BLinAct.goUp();
//  FLinAct.goUp();

//  //Motor Action
//  double kp = 1;
//  double ki = 0;
//  double kd = 0;
//  double target = 0;

//    ActiveLeftMotor->PIDPos(kp_Lpos,ki_Lpos,kd_Lpos,0);
//    ActiveRightMotor->PIDPos(kp_Rpos,ki_Rpos,kd_Rpos,0);
  
}

void loopPhaseOne(){
  //phase parameters
  double maxDeg = 10;

  // If we hit complet top, stop (shouldn't happen in phase 1, but safety net)
  checkFullRaise();

  //raise platform 1.5 degrees tolerance
  levelRaisePlatform(15);

  if(!FrontGroundTrip.read()){
    phaseShift(2);
  }
}

//Phase 2

void setupPhaseTwo(){
  //set the correct motors on/off
  turnOffAll();
  BLinAct.turnOn();
  FLinAct.turnOn();
//  setActiveMotor("B");
  
  //LinActAction
  BLinAct.goUp();
  FLinAct.goUp();

//  //Motor Action
//  double kp = 1;
//  double ki = 0;
//  double kd = 0;
//  double target = 0;
  
//    ActiveLeftMotor->PIDPos(kp_Lpos,ki_Lpos,kd_Lpos,0);
//    ActiveRightMotor->PIDPos(kp_Rpos,ki_Rpos,kd_Rpos,0);
}

void loopPhaseTwo(){
  //phase parameters
  double maxDeg = 10;

  // If we hit complete top, stop
  checkFullRaise();

  //raise platform 1.5 degrees tolerance
  levelRaisePlatform(15);

  //check for next phase
  if(!BackLeftLinUpperTrip.read() && !BackRightLinUpperTrip.read() && !FrontLeftLinUpperTrip.read() && !FrontRightLinUpperTrip.read()){
//    delay(200);
//    Serial.println(-readCompass());
    while(-readCompass() > 5){
      FLinAct.turnOn();
      FLinAct.goDown();
    }
    FLinAct.turnOff();
//    delay(200);
//    while(-readCompass() < -5){
//      BLinAct.turnOn();
//      BLinAct.goDown();
//    }
//    BLinAct.turnOff();
    phaseShift(3);
  }
}

//Phase 3

void setupPhaseThree(){
  turnOffAll();
  setActiveMotor("B");
  ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, 0.1);
  ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, 0.1);
  Serial.println("phase 3 started");
  phaseTime=millis();
  delay(1);
}

void loopPhaseThree(){
  if(FrontLeft2WallTrip.read()){
    ActiveLeftMotor->pause();
    ActiveLeftMotor->turnOff();
  }
  if(FrontRight2WallTrip.read()){
    ActiveRightMotor->pause();
    ActiveRightMotor->turnOff();
  }
  if((FrontLeft2WallTrip.read() && FrontRight2WallTrip.read()) || millis() - phaseTime >5000){
    phaseShift(4);
  }
}

//Phase 4

void setupPhaseFour(){
  turnOffAll();
  BLinAct.turnOn();
  FLinAct.turnOn();

  BLinAct.goDown();
  FLinAct.goDown();
}

void loopPhaseFour(){
  levelLowerPlatform(5);

  if(FrontGroundTrip.read()){
    BLinAct.turnOff();
    FLinAct.turnOff();
    phaseShift(5);
  }
}

//Phase 5

void setupPhaseFive(){
  turnOffAll();
  isNextStair=isStairInFront();
  FLinAct.turnOn();
  FLinAct.goDown();
}

void loopPhaseFive(){
  if(FrontLinLowerTrip.read()){
    FLinAct.pause();
    FLinAct.turnOff();
    BLinAct.turnOn();
    Serial.println("shutting of front lin");
    while(-readCompass()>0){
      Serial.println("leveling in 5");
      levelRaisePlatform(0);
    }
    phaseShift(6);
  }
}

//Phase 6

void setupPhaseSix(){
  turnOffAll();
  setActiveMotor("B");
  ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, 0.1);
  ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, 0.1);
  phaseTime=millis();
}

void loopPhaseSix(){
  if(BackWallTrip.read() || millis() - phaseTime > 5000){
    ActiveLeftMotor->pause();
    ActiveLeftMotor->turnOff();
    ActiveRightMotor->pause();
    ActiveRightMotor->turnOff();
    phaseShift(7);
  }
}

//Phase 7

void setupPhaseSeven(){
  turnOffAll();
  phaseTime=millis();
   setActiveMotor("F");
   ActiveLeftMotor->PIDVel(kp_Lvel,0,kd_Lvel, 0.1);
   ActiveRightMotor->PIDVel(kp_Rvel-300,0,kd_Rvel, 0.1);
   BLinAct.turnOn();
   BLinAct.goDown();
}

void loopPhaseSeven(){
  
  if(BackLinLowerTrip.read()){
    BLinAct.pause();
    BLinAct.turnOff();
    if(isNextStair){
      phaseShift(8);
    }else{
      //make sure it clears the last bit of the stair before going to driving mode
      if((BackLeftIR.read() <=10 && BackRightIR.read() <=10) || FrontLeftIR.read()>10 || FrontRightIR.read()>10 ){
        stairMode=0;
        phaseShift(0);

        Serial.println("ending at top");
        turnOffAll();
//        orientOnStair2();
      }
    }
  }
}

//Phase 8

void setupPhaseEight(){
    phaseTime=millis();
    setActiveMotor("F");
    ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, 0.1);
    ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, 0.1);
}

void loopPhaseEight(){
  if(FrontLeftWallTrip.read()){
      ActiveLeftMotor->pause();
    }
    if(FrontRightWallTrip.read()){
      ActiveRightMotor->pause();
    }
    if((FrontLeftWallTrip.read() && FrontRightWallTrip.read()) || millis() - phaseTime >5000){
      
      Serial.println("looping to 1");
      phaseShift(1);
    }
}
