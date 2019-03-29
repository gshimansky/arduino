#include "settings.hpp"
#include <UniversalTelegramBot.h>

#include <Wire.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <WiFiClientSecure.h>

const int QMC5883L_ADDRESS = 0x0D;
// const int HMC5883_ADDRESS = 0x1e;
#if defined(ARDUINO_ARCH_ESP32)
const int inputPin = 4;  // PIN 4
const int SDAPin = SDA;  // PIN 21
const int SCLPin = SCL;  // PIN 22
#else
const int inputPin = D1;
const int SDAPin = D5;
const int SCLPin = D4;
#endif

WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

bool door_open = false;

void setup() {
  // Initializing serial port for debugging purposes
  Serial.begin(115200);
  delay(10);

#if defined(ARDUINO_ARCH_ESP8622)
  // Workaround for Arduino SSL bug
  // From here https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot/issues/100#issuecomment-450145611
  client.setFingerprint("BB DC 45 2A 07 E3 4A 71 33 40 32 DA BE 81 F7 72 6F 4A 2B 6B");
#endif

  // Initialize sensor digital pin
  pinMode(inputPin, INPUT);
  //attachInterrupt(inputPin, sensorInterrupt, CHANGE);
  Wire.begin(SDAPin, SCLPin);

  Wire.beginTransmission(HMC5883_ADDRESS); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883_ADDRESS); //start talking
  Wire.write(0x01); // Set the gain Register
  Wire.write(0x00); // Set minimal gain
  Wire.endTransmission();

  // Connecting to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Printing the ESP IP address
  Serial.println(WiFi.localIP());
}

void loop() {
  //  sendTelegramMessage(door_open);
  /*
    int dv = digitalRead(inputPin);
    if (dv == HIGH) {
      Serial.println("D: MAGNET!");
    } else {
      Serial.println("D: no magnet");
    }
  */
  /*
    int av = analogRead(A0);
    Serial.print("A: ");
    Serial.println(av);
  */
  int x, y, z; //triple axis data

  // Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(HMC5883_ADDRESS);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission();

  // Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(HMC5883_ADDRESS, 6);
  if (6 <= Wire.available()) {
    x = Wire.read() << 8; //MSB  x
    x |= Wire.read(); //LSB  x
    z = Wire.read() << 8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read() << 8; //MSB y
    y |= Wire.read(); //LSB y

    // Show Values
    Serial.print("X Value: ");
    Serial.println(x);
    Serial.print("Y Value: ");
    Serial.println(y);
    Serial.print("Z Value: ");
    Serial.println(z);
    Serial.println();
  } else {
    Serial.println("Data is not available yet");
  }

  // every 0.5 second
  delay(500);
}

void sensorInterrupt() {
  Serial.print("INTERRUPT! ");
  int dv = digitalRead(inputPin);
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
