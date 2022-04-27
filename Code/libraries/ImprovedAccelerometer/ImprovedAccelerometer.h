#ifndef ImprovedAcceleromter_h
#define ImprovedAcceleromter_h

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

class Accelerometer : public Adafruit_LIS3DH{
  public:
    Accelerometer()
      : 
      Adafruit_LIS3DH(){};
    double getRoll();
    double getPitch();
  private:
    double _roll;
    double _pitch;
};

#endif