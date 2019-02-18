#include "settings.hpp"
#include <UniversalTelegramBot.h>

#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <EEPROM.h>

int distance_threshold = 140; // Distance in cm to turn on red light
const int distance_threshold_eeprom_address = 0;
const int distance_interval = 10; // Allow this many cm to stay from threshold in either direction
const int trigPin = D0;
const int echoPin = D1;
const int redpin = D3;
const int greenpin = D2;

const int measure_delay_ms = 250;
const int telegram_check_delay_ms = 2000;
const int car_presence_delay_ms = 30000;

bool threshold_enter_mode;
// Current car presence state
enum {
  CAR_UNKNOWN = 0,
  CAR_ABSENT = 1,
  CAR_PRESENT = 2
} new_car_presence, last_changed_car_presence = CAR_UNKNOWN, car_presence = CAR_UNKNOWN;
const String car_state_strings[] = {"UNKNOWN", "ABSENT", "PRESENT"};
long car_state_change_millis = 0, last_distance_change = 0;

WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

// Latest measured distance
long duration, cm;
long telegram_bot_lasttime;
long days, hours, minutes, seconds;

void setup() {
  // Workaround for Arduino SSL bug
  // From here https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot/issues/100#issuecomment-450145611
  client.setFingerprint("BB DC 45 2A 07 E3 4A 71 33 40 32 DA BE 81 F7 72 6F 4A 2B 6B");

  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);

  EEPROM.begin(sizeof(distance_threshold));
  EEPROM.get(distance_threshold_eeprom_address, distance_threshold);

  //Serial Port begin
  Serial.begin (9600);
  delay(10);

  // Connecting to WiFi network
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
  sendStatusMessage(USER_ID);
}

void getTime(long all_seconds, long &days, long &hours, long &minutes, long &seconds) {
  seconds = all_seconds % 60;
  all_seconds /= 60;
  minutes = all_seconds % 60;
  all_seconds /= 60;
  hours = all_seconds % 24;
  days = all_seconds / 24;
}

void sendStatusMessage(String chat_id) {
  getTime(millis() / 1000, days, hours, minutes, seconds);

  String reply = "Bot has been running for " + String(days) + " days " +
      String(hours) + ":" + String(minutes) + ":" + String(seconds) + ".\n"
      "Car is " + car_state_strings[car_presence] + ".\n"
      "Current threshold is " + String(distance_threshold) + " centimeters.\n";

  if (car_state_change_millis != 0) {
    getTime((millis() - car_state_change_millis) / 1000, days, hours, minutes, seconds);
    reply += "Last change of state was " + String(days) + " days " +
      String(hours) + ":" + String(minutes) + ":" + String(seconds) + " ago.\n";
  }
  reply += "Last distance was " + String(cm) + " cm.";
  bot.sendMessage(chat_id, reply, "");
}

void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    String text = bot.messages[i].text;

    String from_name = bot.messages[i].from_name;
    if (from_name == "")
      from_name = "Guest";

    if (text == "/status") {
      sendStatusMessage(chat_id);
      threshold_enter_mode = false;
    } else if (text == "/threshold") {
      bot.sendMessage(chat_id, "Enter threshold number in centimeters", "");
      threshold_enter_mode = true;
    } else if (text == "/start" || text == "/help") {
      String keyboardJson = "[[\"/status\", \"/threshold\"]]";
      String welcome = "Welcome to garage distance meathuring bot, " + from_name + ".\n"
        "/status : to get current distance\n"
        "/threshold : to set threshold when to light red light\n";
      bot.sendMessageWithReplyKeyboard(chat_id, welcome, "", keyboardJson, true);
      threshold_enter_mode = false;
    } else if (threshold_enter_mode) {
      int new_threshold;
      int ints = sscanf(text.c_str(), "%d", &new_threshold);
      if (ints == 1) {
        if (chat_id == USER_ID) {
          bot.sendMessage(chat_id,
            "New threshold is " + String(new_threshold) + " centimeters.", "");
          distance_threshold = new_threshold;
          EEPROM.put(distance_threshold_eeprom_address, distance_threshold);
          EEPROM.commit();
        } else {
          bot.sendMessage(chat_id, "You are not my master, " + from_name +
            " so you cannot change threshold.", "");
        }
      } else {
        bot.sendMessage(chat_id, "Entered text could not be parsed: " + text, "");
      }
      threshold_enter_mode = false;
    } else {
      bot.sendMessage(chat_id, "Command not understood: " + text, "");
    }
  }
}

void loop() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  // Convert the time into a distance
  cm = (duration / 2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  int compare_cm = cm;

  // Adjust distance to interval
  switch (car_presence) {
    case CAR_PRESENT:
      compare_cm -= distance_interval;
      break;
    case CAR_ABSENT:
      compare_cm += distance_interval;
      break;
  }

  if (compare_cm < distance_threshold) {
    // Show red
    analogWrite(redpin, 0);
    analogWrite(greenpin, 1024);
    new_car_presence = CAR_PRESENT;
  } else {
    // Show green
    analogWrite(redpin, 1024);
    analogWrite(greenpin, 0);
    new_car_presence = CAR_ABSENT;
  }

  if (last_changed_car_presence != new_car_presence) {
    last_distance_change = millis();
    last_changed_car_presence = new_car_presence;
  } else {
    long cur_time = millis();
    if ((cur_time - last_distance_change > car_presence_delay_ms) &&
      (new_car_presence != car_presence)) {
      getTime((millis() - car_state_change_millis) / 1000, days, hours, minutes, seconds);
      String timestr = " after " + String(days) + " days " +
        String(hours) + ":" + String(minutes) + ":" + String(seconds) +
        " of being " + car_state_strings[car_presence] + ".";

      car_presence = new_car_presence;
      car_state_change_millis = cur_time;

      Serial.println("Changing car status to " + car_state_strings[car_presence] + timestr);
      bot.sendMessage(USER_ID, "Car is now " + car_state_strings[car_presence] + timestr +
        " Car distance is " + String(cm) + " cm.", "");
    }
  }

  Serial.print(cm);
  Serial.println("cm");

  if (millis() > telegram_bot_lasttime + telegram_check_delay_ms)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while(numNewMessages) {
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    telegram_bot_lasttime = millis();
  }

  delay(measure_delay_ms);
}
