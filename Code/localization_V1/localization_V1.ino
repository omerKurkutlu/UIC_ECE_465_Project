#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#include <MadgwickAHRS.h>

void setup() {
  Serial.begin(115200);
  if(!IMU.begin()){ Serial.println("IMU init failed"); while(1); }
  if(!BLE.begin()){ Serial.println("BLE init failed"); while(1); }
  BLE.advertise();
  Serial.println("Mini OK");
}

void loop() {
  delay(500);
}
