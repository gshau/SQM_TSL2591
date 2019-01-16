#include "ESP8266WiFi.h"
#include "SQM_TSL2591.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.25)

SQM_TSL2591 sqm = SQM_TSL2591(2591);

void readSQM(void);

String PROTOCOL_NUMBER = "00000002";
String MODEL_NUMBER = "00000003";
String FEATURE_NUMBER = "00000001";
String SERIAL_NUMBER = "00000000";

// ESP8266 has polarity switched
bool LED_ON = LOW;
bool LED_OFF = HIGH;

const char *ssid = "WIFI_SSID";
const char *password = "WIFI_PASSWORD";

WiFiServer wifiServer(10001);

void setup_tsl() {
  pinMode(13, OUTPUT);
  if (sqm.begin()) {

    Serial.println("Found SQM (TSL) sensor");
    sqm.config.gain = TSL2591_GAIN_LOW;
    sqm.config.time = TSL2591_INTEGRATIONTIME_200MS;
    sqm.configSensor();
    sqm.showConfig();
    sqm.setCalibrationOffset(0.0);
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

void setup_wifi() {
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting..");
  }

  Serial.print("Connected to WiFi. IP:");
  Serial.println(WiFi.localIP());

  wifiServer.begin();
}

String IpAddress2String(const IPAddress &ipAddress) {
  return String(ipAddress[0]) + String(".") + String(ipAddress[1]) +
         String(".") + String(ipAddress[2]) + String(".") +
         String(ipAddress[3]);
}

void setup() {

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_OFF);

  // Start watchdog setup
  wdt_enable(WDTO_8S);

  setup_temperature();

  setup_wifi();

  setup_tsl();
}

void loop() {
  String temp_string;
  float temp;

  wdt_reset();
  WiFiClient client = wifiServer.available();

  if (client) {

    while (client.connected()) {
      String response = "";

      while (client.available() > 0) {
        temp = get_temperature();
        sqm.takeReading();
        delay(10);
        digitalWrite(LED_BUILTIN, LED_ON);
        char c = client.read();

        String remote_ip = String(IpAddress2String(client.remoteIP()));
        Serial.println(String("Request received from ") + remote_ip);

        if (c == 'i') {
          client.println(String(response + "i," + PROTOCOL_NUMBER + "," +
                                MODEL_NUMBER + "," + FEATURE_NUMBER + "," +
                                SERIAL_NUMBER));
        }
        if (c == 'r') {
          Serial.println(String("Sending data: ") + String(sqm.mpsas, 3));
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
          client.println(response);
        }
      }

      delay(10);
    }

    client.stop();
    Serial.println("Client disconnected");
    digitalWrite(LED_BUILTIN, LED_OFF);

    delay(100);
  }
}
