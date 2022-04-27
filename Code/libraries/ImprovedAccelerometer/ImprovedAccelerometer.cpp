#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "ImprovedAccelerometer.h"

double Accelerometer::getRoll(){
  sensors_event_t event;
  this->getEvent(&event);
  _roll=atan2(-event.acceleration.x,event.acceleration.z)*180/PI;
  return _roll;
}

double Accelerometer::getPitch(){
  sensors_event_t event;
  this->getEvent(&event);
  double X = event.acceleration.x;
  double Y = event.acceleration.y;
  double Z = event.acceleration.z;
  _pitch=atan2(Y,sqrt(X*X+Z*Z))*180/PI;
  return _pitch;
}