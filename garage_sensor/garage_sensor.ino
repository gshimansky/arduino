#include "settings.hpp"
#include <UniversalTelegramBot.h>

#include <Wire.h>
#include <ESP8266WiFi.h>
#include <AddrList.h>
#include <WiFiClientSecure.h>

WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

bool door_open = false;

void setup() {
  // Workaround for Arduino SSL bug
  // From here https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot/issues/100#issuecomment-450145611
  client.setFingerprint("BB DC 45 2A 07 E3 4A 71 33 40 32 DA BE 81 F7 72 6F 4A 2B 6B");

  // Initialize sensor digital pin
  pinMode(D1, INPUT);
  //attachInterrupt(D1, sensorInterrupt, CHANGE);

  // Initializing serial port for debugging purposes
  Serial.begin(9600);
  delay(10);

  // Connecting to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  for (bool configured = false; !configured;) {
    for (auto addr : addrList)
      if ((configured = !addr.isLocal()
                        // && addr.isV6() // uncomment when IPv6 is mandatory
                        // && addr.ifnumber() == STATION_IF
          )) {
        break;
      }
    Serial.print('.');
    delay(500);
  }

  Serial.println("WiFi connected");

  // Printing the ESP IP address
  Serial.println(WiFi.localIP());
}

void loop() {
//  sendTelegramMessage(door_open);

  int dv = digitalRead(D1);
  if (dv == HIGH) {
    Serial.println("D: MAGNET!");
  } else {
    Serial.println("D: no magnet");
  }

  int av = analogRead(A0);
  Serial.print("A: ");
  Serial.println(av);

  // every 0.5 second
  delay(500);
}

void sensorInterrupt() {
  Serial.print("INTERRUPT! ");
  int dv = digitalRead(D1);
  Serial.println(dv);
}

void sendTelegramMessage(bool state) {
  String message = "Door is:  \n";
  message.concat(state ? "OPEN\n" : "CLOSED\n");

  if (bot.sendMessage(USER_ID, message, "Markdown")) {
    Serial.println("TELEGRAM Successfully sent");
  } else {
    Serial.println("TELEGRAM message failed");
  }
}
