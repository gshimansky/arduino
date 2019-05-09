#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include "ThingSpeak.h"
#include "settings.hpp"

const int SENSOR_PIN PIN_A0;

WiFiClient client;

void setup() {
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

  // put your main code here, to run repeatedly:
  int val;
  val = analogRead(0); //connect sensor to Analog 0
  Serial.println(val); //print the value to serial port

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
}

void loop() {
  Serial.println("Should never get here");
}
