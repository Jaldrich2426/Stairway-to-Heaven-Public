void setup() {
  //Individual Sensor/Motor Testing
Serial.begin(9600);
delay(1000);
  Wire.begin();
  Serial.println("here1");
  compass.init();
  irrecv.enableIRIn();
  irrecv.blink13(true);
  calibrate();
  delay(1000);
  phaseShift(0);
  Serial.println("here");

    delay(1000);

//    calibrate();
    delay(1000);
//    setActiveMotor("F");
//    startStairClimb();

    //bellow is all test
//      setActiveMotor("F");
//      orientOnStair2();
//  ActiveLeftMotor->PIDVel(kp_Lvel,ki_Lvel,kd_Lvel, -0.1);
//  ActiveRightMotor->PIDVel(kp_Rvel,ki_Rvel,kd_Rvel, -0.1);
}

void loop() {
  FLMotor.internalUpdate();
  FRMotor.internalUpdate();
  BLMotor.internalUpdate();
  BRMotor.internalUpdate();
//  BLinAct.turnOn();
//  BLinAct.goUp();
//  setActiveMotor("B");
//  ActiveRightMotor->PID(
  loopPhaseCheck();

//below is all test
//readIrRemote();
//Serial.println("in loop");
//FLinAct.turnOn();
//BLinAct.turnOn();
//levelRaisePlatform(15);
//Serial.println(BackLeftIR.read());
//Serial.println(FrontRightIR.read());
//Serial.println(FrontLinLowerTrip.read());

}
