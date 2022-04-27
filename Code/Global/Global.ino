// -------------- Librarires to include --------------//
#include <ActuatorsAndSensors.h>
#include <Wire.h>
#include "SFE_HMC6343.h"
#include <IRremote.hpp>

// -------------- Object information --------------//
#define BL_EncA   2
#define BR_EncA   3
#define BL_MotEn   4
#define BR_MotEn   5
#define B_LinEn   6
#define FL_MotEn   7
#define FR_MotEn   8
#define F_LinEn   9

#define FL_EncA   18
#define FL_EncB   42
#define FR_EncA   19
#define FR_EncB   43

#define BL_EncB   22
#define BR_EncB   23
#define BL_MotIN1   25
#define BL_MotIN2   24
#define BR_MotIN1   27
#define BR_MotIN2   26
#define B_LinIN1   29
#define B_LinIN2   28
#define FL_MotIN1   31
#define FL_MotIN2   30
#define FR_MotIN1   32
#define FR_MotIN2   33
#define F_LinIN1   34
#define F_LinIN2   35

#define B_US_Echo   10
#define FM_US_Echo   11
#define FL_US_Echo   12
#define FR_US_Echo   40
#define B_US_Trig   36
#define FM_US_Trig   37
#define FL_US_Trig   38
#define FR_US_Trig   41

#define B_WTr   48

//#define B_GTr   44
#define B_GTr A4


//Upper is the upper limit of actuation, lower the mimimal actuation
//#define BL_LinUppTr   46

//#define BR_LinUppTr   48
//#define FL_LinUppTr   50

//#define FR_LinUppTr   52

#define BR_LinUppTr A8
#define FR_LinUppTr A9
#define BL_LinUppTr A10
#define FL_LinUppTr A11
#define F_LinLowTr   A12
#define B_LinLowTr   A13

#define FL_WTr   A14
#define FR_WTr   A15
#define F_GTr   44
#define FL2_WTr 45
#define FR2_WTr 47
#define F2_GTr  46

#define BL_IR   A0
#define BR_IR   A1
#define FL_IR   A2
#define FR_IR   A3

//Lin Acts
LinearActuator FLinAct(F_LinIN1 ,F_LinIN2, F_LinEn);
LinearActuator BLinAct(B_LinIN1 ,B_LinIN2, B_LinEn);

//Encoders for Motors
Encoder FLEnc(FL_EncA, FL_EncB);
Encoder FREnc(FR_EncA, FR_EncB);
Encoder BLEnc(BL_EncA, BL_EncB);
Encoder BREnc(BR_EncA, BR_EncB);

//Motor Parameters
int pwmUpper=255;
int pwmLower=0;
double countPerCm150=9600/80/PI;
double countPerCm50=3200/80/PI;

//Motors
Motor FLMotor(FL_MotIN2, FL_MotIN1, FL_MotEn, pwmLower, pwmUpper, &FLEnc, countPerCm50,-1);
Motor FRMotor(FR_MotIN1, FR_MotIN2, FR_MotEn, pwmLower, pwmUpper, &FREnc, countPerCm50,1);
Motor BLMotor(BL_MotIN1, BL_MotIN2, BL_MotEn, pwmLower, pwmUpper, &BLEnc, countPerCm150,1);
Motor BRMotor(BR_MotIN1, BR_MotIN2, BR_MotEn, pwmLower, pwmUpper, &BREnc, countPerCm150,-1);

Motor *ActiveLeftMotor;
Motor *ActiveRightMotor;

//Wall and ground bump switches
BumpSwitch BackWallTrip(B_WTr, "NC");
BumpSwitch FrontLeftWallTrip(FL_WTr, "NC");
BumpSwitch FrontRightWallTrip(FR_WTr, "NC");
BumpSwitch BackGroundTrip(B_GTr, "NC");
BumpSwitch FrontGroundTrip(F_GTr, "NC");
BumpSwitch FrontLeft2WallTrip(FL2_WTr, "NC");
BumpSwitch FrontRight2WallTrip(FR2_WTr, "NC");
BumpSwitch Front2GroundTrip(F2_GTr, "NC");

//Lin bump switches
BumpSwitch  BackLeftLinUpperTrip(BL_LinUppTr, "NC");
//BumpSwitch  BackLeftLinLowerTrip(BL_LinLowTr, "NC");
BumpSwitch  BackRightLinUpperTrip(BR_LinUppTr, "NC");
BumpSwitch  BackLinLowerTrip(B_LinLowTr, "NC");
BumpSwitch  FrontLeftLinUpperTrip(FL_LinUppTr, "NC");
//BumpSwitch  FrontLeftLinLowerTrip(FL_LinLowTr, "NC");
BumpSwitch  FrontRightLinUpperTrip(FR_LinUppTr, "NC");
BumpSwitch  FrontLinLowerTrip(F_LinLowTr, "NC");

//IR
IrSensor BackLeftIR(BL_IR);
IrSensor BackRightIR(BR_IR);
IrSensor FrontLeftIR(FL_IR);
IrSensor FrontRightIR(FR_IR);
//IrSensor UpperIR(A4);

//US
UltrasonicSensor Upper2US(51,50);
UltrasonicSensor BackUS(B_US_Trig, B_US_Echo);
UltrasonicSensor FrontMiddleUS(FM_US_Trig, FM_US_Echo);
UltrasonicSensor FrontLeftUS(FL_US_Trig, FL_US_Echo);
UltrasonicSensor FrontRightUS(FR_US_Trig, FR_US_Echo);

SFE_HMC6343 compass;

#define RECV_PIN  52
IRrecv irrecv(RECV_PIN);

//--------------- Static global variables ---------------//

//Geometry (cm)
double wheelSpacing = 30;

//Motor gains - B/F=Back/Front, L/R=Left/Right, pos/vel=position/velocity

//Front Left
double kp_FLpos = 0;
double ki_FLpos = 0;
double kd_FLpos = 0;

double kp_FLvel = 1200;
double ki_FLvel = 0.0001;
double kd_FLvel = 0;

//Front Right
double kp_FRpos = 0;
double ki_FRpos = 0;
double kd_FRpos = 0;

double kp_FRvel = 1500;
double ki_FRvel = 0.0001;
double kd_FRvel = 0;

//ActiveLeftMotor->PIDVel(5000,0.1,0, 0.1); //0.25 is max speed on back
//ActiveRightMotor->PIDVel(3000,0.1,0, 0.1);
//    ActiveLeftMotor->PIDPos(2.5,0.0001,100,50);
//    ActiveRightMotor->PIDPos(1.5,0.0001,100,50);
//Back Left
double kp_BLpos = 2.5;
double ki_BLpos = 0.0001;
double kd_BLpos = 100;

double kp_BLvel = 5000;
double ki_BLvel = 0.1;
double kd_BLvel = 0;

//Back Right
double kp_BRpos = 1.5;
double ki_BRpos = 0.0001;
double kd_BRpos = 100;

double kp_BRvel = 3000;
double ki_BRvel = 0.1;
double kd_BRvel = 0;

// -------------- Dynamic global variables --------------//
int phase=0;
String wheelMode="F";
int stairMode=0; // 0=Driving, 1=up stairs, -1 = down stairs
bool isNextStair=false;
int stairHeading;
int phaseIRreading;
bool IR_action_has_timeout=false;
int IRLoopMode=0;

unsigned long compassLastTime = millis();
unsigned long phaseTime = millis();
//changed gains by active wheels
//Left
double kp_Lpos = kp_FLpos;
double ki_Lpos = ki_FLpos;
double kd_Lpos = kd_FLpos;

double kp_Lvel = kp_FLvel;
double ki_Lvel = ki_FLvel;
double kd_Lvel = kd_FLvel;

//Right
double kp_Rpos = kp_FRpos;
double ki_Rpos = ki_FRpos;
double kd_Rpos = kd_FRpos;

double kp_Rvel = kp_FRvel;
double ki_Rvel = ki_FRvel;
double kd_Rvel = kd_FRvel;

// ------ IR Variables ------//
unsigned long lastTime = millis();
unsigned long lastTime2=millis();
int IR_cutoff_interval = 500;
