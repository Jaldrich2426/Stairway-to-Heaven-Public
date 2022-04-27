//Phase 11

void setupPhaseEleven(){
  turnOffAll();
  BLinAct.pause();
  setActiveMotor("B");
  ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, -0.08);
  ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, -0.08);
  FRMotor.turnOn();
  FLMotor.turnOn();
  FRMotor.PIDVel(kp_FRvel,ki_FRvel,kd_FRvel, -0.05);
  FLMotor.PIDVel(kp_FLvel,ki_FLvel,kd_FLvel, -0.05);
  Serial.println("phase 11 started");
  phaseTime=millis();
  delay(10);
}

void loopPhaseEleven(){
  if(!BackGroundTrip.read()){
    turnOffAll();
    setActiveMotor("B");
    ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, -0.08);
    ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, -0.08);
    delay(1000);
    phaseShift(12);
  }
  if(millis()-phaseTime > 5000){
    FRMotor.PIDVel(kp_FRvel,ki_FRvel,kd_FRvel, -0.1);
    FLMotor.PIDVel(kp_FLvel,ki_FLvel,kd_FLvel, -0.1);
  }
}

// phase 12

void setupPhaseTwelve(){
  turnOffAll();
  setActiveMotor("B");
  BLinAct.turnOn();
  ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, -0.05);
  ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, -0.05);
  BLinAct.goUp();
  phaseTime=millis();
}

void loopPhaseTwelve(){
  if(millis()-phaseTime>2000){
    ActiveLeftMotor->turnOff();
    ActiveRightMotor->turnOff();
    FLMotor.PIDVel(kp_FLvel,ki_FLvel,kd_FLvel, 0.05);
    FRMotor.PIDVel(kp_FRvel,ki_FRvel,kd_FRvel, 0.05);
  }
  if(BackGroundTrip.read()){
    turnOffAll();
    phaseShift(13);
  }
}

// phase 13

void setupPhaseThirteen(){
  turnOffAll();
  FLinAct.turnOn();
  while(!Front2GroundTrip.read()){
    FLinAct.goUp();
  }
  FLinAct.turnOff();
  setActiveMotor("B");
  ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, -0.1);
  ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, -0.1);
  phaseTime=millis();
}

void loopPhaseThirteen(){
  if(!Front2GroundTrip.read()){
    turnOffAll();
    phaseShift(14);
  }
}

// phase 14

void setupPhaseFourteen(){
  turnOffAll();
  FLinAct.turnOn();
  FLinAct.goUp();
}

void loopPhaseFourteen(){
  if(Front2GroundTrip.read()){
    turnOffAll();
    phaseShift(15);
  }
}

//phase 15

void setupPhaseFifteen(){
    turnOffAll();
  BLinAct.turnOn();
  FLinAct.turnOn();

  BLinAct.goUp();
  FLinAct.goUp();
}

void loopPhaseFifteen(){
  levelRaisePlatform(15);
  checkFullRaise();

    if(!BackLeftLinUpperTrip.read() && !BackRightLinUpperTrip.read() && !FrontLeftLinUpperTrip.read() && !FrontRightLinUpperTrip.read()){
//
//    while(-readCompass() > 5){
//      FLinAct.turnOn();
//      FLinAct.goDown();
//    }
//    FLinAct.turnOff();
//    delay(200);
//    while(-readCompass() < -5){
//      BLinAct.turnOn();
//      BLinAct.goDown();
//    }
//    BLinAct.turnOff();
    turnOffAll();
    phaseShift(16);
    }
}

// phase 16

void setupPhaseSixteen(){
  phaseIRreading=(FrontLeftIR.read()+FrontRightIR.read())/2;
  turnOffAll();
  setActiveMotor("B");
    ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, -0.1);
  ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, -0.1);
}

void loopPhaseSixteen(){
  int IrDif=((FrontLeftIR.read()+FrontRightIR.read())/2) - phaseIRreading;
  if(IrDif > 15){
    turnOffAll();
    phaseShift(17);
  }
}

//phase 17
void setupPhaseSeventeen(){
  turnOffAll();
  BLinAct.turnOn();
  FLinAct.turnOn();

  BLinAct.goDown();
  FLinAct.goDown();
}

void loopPhaseSeventeen(){
    levelLowerPlatform(15);

  checkFullLower();
  if( BackLinLowerTrip.read() && FrontLinLowerTrip.read()){
    turnOffAll();
    phaseShift(18);
  }
}

//phase 18

void setupPhaseEighteen(){
  phaseTime=millis();
  turnOffAll();
  setActiveMotor("F");
  ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, -0.1);
  ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, -0.1);
}

void loopPhaseEighteen(){
  if(BackLeftIR.read() > 12 || BackRightIR.read() >12){
    turnOffAll();
    orientOnStair2();
  }
  if(millis()-phaseTime > 2000){
    turnOffAll();
    stairMode=0;
    phaseShift(0);
  }
}
