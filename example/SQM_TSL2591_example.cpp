/*
  SQM_example
  Uses a TSL2591 sensor as a Sky Quality Meter

  This example code is in the public domain.

  Gabe Shaughnessy
  Oct 21 2017
 */


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "SQM_TSL2591.h"

SQM_TSL2591 sqm = SQM_TSL2591(2591);
void readSQM(void);



void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  if (sqm.begin()) {

    Serial.println("Found SQM (TSL) sensor");
    sqm.config.gain = TSL2591_GAIN_LOW;
    sqm.config.time = TSL2591_INTEGRATIONTIME_200MS;
    sqm.configSensor();
    sqm.showConfig();
    sqm.setCalibrationOffset(0.0);
  } else{
    Serial.println("SQM sensor not found");
 }


}

void loop() {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  sqm.takeReading();
  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)

  Serial.print("full:   "); Serial.println(sqm.full);
  Serial.print("ir:     "); Serial.println(sqm.ir);
  Serial.print("vis:    "); Serial.println(sqm.vis);
  Serial.print("mpsas:  "); Serial.print(sqm.mpsas);
  Serial.print(" +/- "); Serial.println(sqm.dmpsas);

  Serial.println("======================================");

  delay(5000);
}
