#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <ThingSpeak.h>

#include "settings.hpp"

#ifdef BME280_ADDRESS
#undef BME280_ADDRESS
#define BME280_ADDRESS 0x76
#endif

WiFiClient client;
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

  reconnect();
}

/**************************
 *  L O O P
 **************************/
void loop() {
  float t, h, p;
  h = bme.readHumidity();
  t = bme.readTemperature();
  p = bme.seaLevelForAltitude(ALTITUDE, bme.readPressure()) * 0.0075f;  // Convert pascals to mmHg

//    t = t*1.8+32.0;  // Convert to Farengeith
//    dp = t-0.36*(100.0-h);

  Serial.print("Temperature = ");
  Serial.println(t);
  Serial.print("Humidity = ");
  Serial.println(h);
  Serial.print("Pressure = ");
  Serial.println(p);
//    Serial.print("Dew Point = ");
//    Serial.println(dpString);

  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
  }

  ThingSpeak.begin(client);
  ThingSpeak.setField(1, t);
  ThingSpeak.setField(2, h);
  ThingSpeak.setField(3, p);

  int x = ThingSpeak.writeFields(channelID, apiKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  //every 5 Min
  delay(300000);
}

void reconnect() {
  // Connecting to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long start_time = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start_time > 5 * 60 * 1000) {
      Serial.println("No connection in 5 minutes, trying to restart");
      ESP.restart();
    }
  }
  Serial.println("Connected!");
  // Printing the ESP IP address
  Serial.println(WiFi.localIP());
}
