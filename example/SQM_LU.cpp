#include "SQM_TSL2591.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <avr/wdt.h>

#define SEALEVELPRESSURE_HPA (1013.25)

SQM_TSL2591 sqm = SQM_TSL2591(2591);

void readSQM(void);

String PROTOCOL_NUMBER = "00000002";
String MODEL_NUMBER = "00000003";
String FEATURE_NUMBER = "00000001";
String SERIAL_NUMBER = "00000000";

bool LED_ON = HIGH;
bool LED_OFF = LOW;

void setup_tsl() {
  pinMode(13, OUTPUT);
  if (sqm.begin()) {
    sqm.config.gain = TSL2591_GAIN_LOW;
    sqm.config.time = TSL2591_INTEGRATIONTIME_200MS;
    sqm.configSensor();
    sqm.setCalibrationOffset(0.0);
    sqm.verbose = false;
  } else {
    Serial.println("SQM sensor not found");
  }
  delay(1000);
}

// Setup temperature sensor.  This example uses the BME280 sensor.
Adafruit_BME280 bme; // I2C
void setup_temperature() {
  bool status;
  // default settings
  status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
}
float get_temperature() { return bme.readTemperature(); }

void setup() {

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_OFF);

  // Start watchdog setup
  wdt_enable(WDTO_8S);

  setup_temperature();

  setup_tsl();
}

void loop() {
  String temp_string;
  float temp;

  wdt_reset();

  String response = "";
  bool new_data = false;

  while (Serial.available() > 0) {

    temp = get_temperature();
    sqm.takeReading();
    digitalWrite(LED_BUILTIN, LED_ON);
    char c = Serial.read();

    if (c == 'i') {
      response = String(response + "i," + PROTOCOL_NUMBER + "," + MODEL_NUMBER +
                        "," + FEATURE_NUMBER + "," + SERIAL_NUMBER);
      new_data = true;
    }
    if (c == 'r') {
      if (sqm.mpsas < 10.) {
        response = "r,  ";
      } else if (sqm.mpsas < 0) {
        response = "r,";
      } else {
        response = "r, ";
      }

      if (temp < 0.) {
        temp_string = ",  ";
      } else if (temp < 10.) {
        temp_string = ",   ";
      } else {
        temp_string = ",  ";
      }

      temp_string = String(temp_string + String(temp, 1) + "C");

      response =
          String(response + String(sqm.mpsas, 2) +
                 "m,0000005915Hz,0000000000c,0000000.000s" + temp_string);
      new_data = true;
    }

    if (new_data) {
      Serial.println(response);
      new_data = false;
    }

    digitalWrite(LED_BUILTIN, LED_OFF);

    delay(100);
  }
}
