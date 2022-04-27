# Accelerometer Improvemnts
Derived Class for Adafruit's LIS3DH

Example Code of improvements:

```c++
#include <ImprovedAccelerometer.h>

//construct accelerometer
Accelerometer Acc = Accelerometer();

void setup() {
  Serial.begin(9600);

  //initialize connection to the sensor
  Acc.begin();
}

void loop() {
  // returns roll and pitch in degrees
  Serial.print("Roll: ");
  Serial.println(Acc.getRoll());
  Serial.print("Pitch: ");
  Serial.println(Acc.getPitch());
  delay(200);
}
```
