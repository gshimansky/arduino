#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <HTTPClient.h>
#else
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#endif
#include "settings.hpp"

#ifdef BME280_ADDRESS
#undef BME280_ADDRESS
#define BME280_ADDRESS 0x76
#endif

//#define BMP_SCK 13
//#define BMP_MISO 12
//#define BMP_MOSI 11
//#define BMP_CS 10

float t, h, p, pmm, dp;
char temperatureString[6];
char humidityString[6];
char pressureString[7];
char dpString[6];

Adafruit_BME280 bme; // I2C

/**************************
 *   S E T U P
 **************************/
// only runs once on boot
void setup() {
  // Initializing serial port for debugging purposes
  Serial.begin(115200);
  delay(10);
#if defined(ARDUINO_ARCH_ESP32)
  // SDA - PIN 21
  // SCL - PIN 22
  Wire.begin(SDA, SCL, BME280_ADDRESS);
#else
  // Default settings
  // D2 - SDA
  // D1 - SCL
  Wire.begin(D2, D1, BME280_ADDRESS);
//  Wire.setClock(100000);
#endif
  Serial.println(F("BME280 test"));

  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    ESP.restart();
  }

  // Connecting to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Printing the ESP IP address
  Serial.println(WiFi.localIP());
}

/**************************
 *  L O O P
 **************************/
void loop() {
    h = bme.readHumidity();
    t = bme.readTemperature();
    p = bme.seaLevelForAltitude(ALTITUDE, bme.readPressure()); // Pressure in pascals
    pmm = p * 0.0075f;  // Convert pascals to mmHg

//    t = t*1.8+32.0;  // Convert to Farengeith
//    dp = t-0.36*(100.0-h);

    dtostrf(t, 5, 1, temperatureString);
    dtostrf(h, 5, 1, humidityString);
    dtostrf(pmm, 6, 1, pressureString);

//    dtostrf(dp, 5, 1, dpString);
    delay(10000);

    Serial.print("Temperature = ");
    Serial.println(temperatureString);
    Serial.print("Humidity = ");
    Serial.println(humidityString);
    Serial.print("Pressure = ");
    Serial.println(pressureString);
//    Serial.print("Dew Point = ");
//    Serial.println(dpString);

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      bool connected = http.begin("http://api.thingspeak.com/update");
      if (connected) {
        String postStr((char *)0);
        postStr += apiKey;
        postStr +="&field1=";
        postStr += temperatureString;
        postStr +="&field2=";
        postStr += humidityString;
        postStr +="&field3=";
        postStr += pressureString;
        postStr += "\r\n\r\n";

        http.addHeader("X-THINGSPEAKAPIKEY", apiKey);
        int result = http.POST((uint8_t *)postStr.c_str(), postStr.length());
        Serial.print("Server result = ");
        Serial.println(result);
      } else {
        Serial.println("Failed to connect to server api.thingspeak.com");
      }
    } else {
      Serial.println("Not connected to WiFi");
    }
    //every 5 Min
    delay(300000);
}
