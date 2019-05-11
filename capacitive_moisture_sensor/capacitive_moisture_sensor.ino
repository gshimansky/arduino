#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include "ThingSpeak.h"
#include "settings.hpp"

const int SENSOR_READ_PIN = PIN_A0;
const int SENSOR_POWER_PIN = D5;

WiFiClient client;

void setup() {
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  //Serial Port begin
//  Serial.begin(115200);
  Serial.begin(74880);
  delay(10);

  // put your setup code here, to run once:
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.status() != WL_CONNECTED) {
  Serial.print("Attempting to connect to SSID: ");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected");
  }

  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(100);
  int val;
  val = analogRead(SENSOR_READ_PIN); //connect sensor to Analog 0
  Serial.println(val); //print the value to serial port
  digitalWrite(SENSOR_POWER_PIN, LOW);

  ThingSpeak.begin(client);
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeField(channelID, 4, val, apiKey.c_str());
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  Serial.println("Going into deep sleep for 5 minutes");
  ESP.deepSleep(uint64_t(300) * 1000 * 1000); // 5 minutes
  delay(500);
}

void loop() {
  Serial.println("Should never get here");
}
