#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <EEPROM.h>
#include <LiquidCrystal_PCF8574.h>
#include <ThingSpeak.h>

#include "settings.hpp"
#include "message_queue.hpp"

//////////////////////////////////////
// BME sensor section
//////////////////////////////////////
#ifdef BME280_ADDRESS
#undef BME280_ADDRESS
#define BME280_ADDRESS 0x76
#endif

#if defined(ARDUINO_ARCH_ESP32)
const int SDAPin = SDA;  // PIN 21
const int SCLPin = SCL;  // PIN 22
#else
const int SDAPin = D5;
const int SCLPin = D4;
#endif

unsigned long bme_sensor_lasttime;
const int bme_check_delay_ms = 300000;
WiFiClient client;
Adafruit_BME280 bme; // I2C

//////////////////////////////////////
// Distance measurement section
//////////////////////////////////////

int distance_threshold = 140; // Distance in cm to turn on red light
const int distance_threshold_eeprom_address = 0;
const int presence_addition = 20; // Detect car at this distance plus threshold

#if defined(ARDUINO_ARCH_ESP32)
const int trigPin = A4;  // PIN 32
const int echoPin = A5;  // PIN 33
const int redpin = 4;    // PIN  4
const int greenpin = 5;  // PIN  5
#else
const int trigPin = D0;
const int echoPin = D1;
const int redpin = D3;
const int greenpin = D2;
#endif

#if defined(ARDUINO_ARCH_ESP32)
const int signal_channel_number = 0;
const int channel_frequency = 5000;
const int channel_bits = 10;
#endif

const int measure_delay_ms = 250;
const int car_presence_delay_ms = 30000;

bool threshold_enter_mode;
// Current car presence state
enum {
  CAR_UNKNOWN = 0,
  CAR_ABSENT = 1,
  CAR_PRESENT = 2
} new_car_presence, last_changed_car_presence = CAR_UNKNOWN, car_presence = CAR_UNKNOWN;
const String car_state_strings[] = {"UNKNOWN", "ABSENT", "PRESENT"};
unsigned long car_state_change_millis = 0, last_distance_change = 0;

//////////////////////////////////////
// Telegram communication section
//////////////////////////////////////

// Last time telegram server was checked for incoming messages
unsigned long telegram_bot_lasttime;
const int telegram_check_delay_ms = 2000;

WiFiClientSecure secure_client;
UniversalTelegramBot bot(BOT_TOKEN, secure_client);
TelegramMessageQueue tmq(bot);

//////////////////////////////////////
// LCD display
//////////////////////////////////////

const int LCD_Address = 0x27;
LiquidCrystal_PCF8574 lcd(LCD_Address);  // set the LCD address to 0x27

void setup() {
  //Serial Port begin
  Serial.begin(115200);
  delay(10);

#if defined(ARDUINO_ARCH_ESP8622)
  // Workaround for Arduino SSL bug
  // From here https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot/issues/100#issuecomment-450145611
  secure_client.setFingerprint("BB DC 45 2A 07 E3 4A 71 33 40 32 DA BE 81 F7 72 6F 4A 2B 6B");
#endif

  // Set up I2C device pins
  Wire.begin(SDAPin, SCLPin, BME280_ADDRESS);

  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    ESP.restart();
  }

  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

#if defined(ARDUINO_ARCH_ESP32)
/*
  ledcSetup(signal_channel_number, channel_frequency, channel_bits);
  ledcAttachPin(redpin, signal_channel_number);
  ledcAttachPin(greenpin, signal_channel_number);
*/
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
#endif

  EEPROM.begin(sizeof(distance_threshold));
  EEPROM.get(distance_threshold_eeprom_address, distance_threshold);

  reconnect();

  // initialize the lcd
  lcd.begin(20, 4);
  lcd.noAutoscroll();
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.noCursor();
  lcd.display();

  // Printing the ESP IP address
  Serial.println(WiFi.localIP());
  sendStatusMessage(USER_ID, 0);
}

void getTime(unsigned long all_seconds, unsigned long &days, unsigned long &hours, unsigned long &minutes, unsigned long &seconds) {
  seconds = all_seconds % 60;
  all_seconds /= 60;
  minutes = all_seconds % 60;
  all_seconds /= 60;
  hours = all_seconds % 24;
  days = all_seconds / 24;
}

void sendStatusMessage(String &&chat_id, int cm) {
  unsigned long days, hours, minutes, seconds;
  getTime(millis() / 1000, days, hours, minutes, seconds);

  String reply((char *)0);
  reply.reserve(256);
  reply += "Distance bot has been running for ";
  reply += days;
  reply += " days ";
  reply += hours;
  reply += ":";
  reply += minutes;
  reply += ":";
  reply += seconds;
  reply += ".\nCar is ";
  reply += car_state_strings[car_presence];
  reply += ".\nCurrent threshold is ";
  reply += distance_threshold;
  reply += " centimeters.\n";

  if (car_state_change_millis != 0) {
    getTime((millis() - car_state_change_millis) / 1000, days, hours, minutes, seconds);

    reply += "Last change of state was ";
    reply += days;
    reply += " days ";
    reply += hours;
    reply += ":";
    reply += minutes;
    reply += ":";
    reply += seconds;
    reply += " ago.\n";
  }

  reply += "Last distance was ";
  reply += cm;
  reply += " cm.";
  tmq.enqueue(std::move(chat_id), std::move(reply));
}

void handleNewMessages(int numNewMessages, long cm) {
  Serial.print("handleNewMessages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = bot.messages[i].chat_id;
    const String &text = bot.messages[i].text;
    const String &from_name = bot.messages[i].from_name == "" ? "Guest" : bot.messages[i].from_name;

    if (text == "/status") {
      sendStatusMessage(std::move(chat_id), cm);
      threshold_enter_mode = false;
    } else if (text == "/threshold") {
      tmq.enqueue(std::move(chat_id), "Enter threshold number in centimeters");
      threshold_enter_mode = true;
    } else if (text == "/start" || text == "/help") {
      String keyboardJson = "[[\"/status\", \"/threshold\"]]";
      String welcome((char *)0);
      welcome.reserve(256);
      welcome += "Welcome to garage distance meathuring bot, ";
      welcome += from_name;
      welcome += ".\n/status : to get current distance\n/threshold : to set threshold when to light red light\n";

      tmq.enqueue(std::move(chat_id), std::move(welcome), std::move(keyboardJson), true);
      threshold_enter_mode = false;
    } else if (threshold_enter_mode) {
      int new_threshold;
      int ints = sscanf(text.c_str(), "%d", &new_threshold);
      if (ints == 1) {
        if (chat_id == USER_ID) {
          String msg((char *)0);
          msg.reserve(256);
          msg += "New threshold is ";
          msg += new_threshold;
          msg += " centimeters.";
          tmq.enqueue(std::move(chat_id), std::move(msg));

          distance_threshold = new_threshold;
          EEPROM.put(distance_threshold_eeprom_address, distance_threshold);
          EEPROM.commit();
        } else {
          String msg((char *)0);
          msg.reserve(256);
          msg += "You are not my master, ";
          msg += from_name;
          msg += " so you cannot change threshold.";
          tmq.enqueue(std::move(chat_id), std::move(msg));
        }
      } else {
        String msg((char *)0);
        msg.reserve(256);
        msg += "Entered text could not be parsed: ";
        msg += text;
        tmq.enqueue(std::move(chat_id), std::move(msg));
      }
      threshold_enter_mode = false;
    } else {
      String msg((char *)0);
      msg.reserve(256);
      msg += "Command not understood: ";
      msg += text;
      tmq.enqueue(std::move(chat_id), std::move(msg));
    }
  }
}

void loop() {
  long duration;
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
  long cm = (duration / 2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  Serial.print(cm);
  Serial.println("cm");

  // Update information on LCD display
  updateLCD(cm);

  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
  }

  // Invoke car detection logic
  carPresenceProcessing(cm);

  // Telegram inpue message queue processing
  if (millis() - telegram_bot_lasttime > telegram_check_delay_ms) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while(numNewMessages) {
      handleNewMessages(numNewMessages, cm);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    telegram_bot_lasttime = millis();
  }

  // Telegram output messages queue processing
  tmq.send_one();

  // Temperature message processing
  if (millis() - bme_sensor_lasttime > bme_check_delay_ms) {
    bmeSensorProcessing();
    bme_sensor_lasttime = millis();
  }

  delay(measure_delay_ms);
}

void carPresenceProcessing(long cm) {
  // Show signal where to stop
#if defined(ARDUINO_ARCH_ESP32)
/*
  if (cm < distance_threshold) {
    // Show red
    ledcWrite(redpin, 0);
    ledcWrite(greenpin, 1023);
  } else {
    // Show green
    ledcWrite(redpin, 1023);
    ledcWrite(greenpin, 0);
  }
*/
  if (cm < distance_threshold) {
    // Show red
    digitalWrite(redpin, HIGH);
    digitalWrite(greenpin, LOW);
  } else {
    // Show green
    digitalWrite(redpin, LOW);
    digitalWrite(greenpin, HIGH);
  }
#else
  if (cm < distance_threshold) {
    // Show red
    analogWrite(redpin, 0);
    analogWrite(greenpin, 1023);
  } else {
    // Show green
    analogWrite(redpin, 1023);
    analogWrite(greenpin, 0);
  }
#endif

  // Detect car presence
  if (cm < distance_threshold + presence_addition) {
    new_car_presence = CAR_PRESENT;
  } else {
    new_car_presence = CAR_ABSENT;
  }

  if (last_changed_car_presence != new_car_presence) {
    last_distance_change = millis();
    last_changed_car_presence = new_car_presence;
  } else {
    unsigned long cur_time = millis();
    if ((cur_time - last_distance_change > car_presence_delay_ms) &&
      (new_car_presence != car_presence)) {
      unsigned long days, hours, minutes, seconds;
      getTime((cur_time - car_state_change_millis) / 1000, days, hours, minutes, seconds);
      String timestr((char *)0);
      timestr.reserve(256);
      timestr += " after ";
      timestr += days;
      timestr += " days ";
      timestr += hours;
      timestr += ":";
      timestr += minutes;
      timestr += ":";
      timestr += seconds;
      timestr += " of being ";
      timestr += car_state_strings[car_presence];
      timestr += ".";

      car_presence = new_car_presence;
      car_state_change_millis = cur_time;

      String debugMsg((char *)0);
      debugMsg.reserve(256);
      debugMsg += "Changing car status to ";
      debugMsg += car_state_strings[car_presence];
      debugMsg += timestr;
      Serial.println(debugMsg);

      String msg((char *)0);
      msg.reserve(256);
      msg += "Car is now ";
      msg += car_state_strings[car_presence];
      msg += timestr;
      msg += " Car distance is ";
      msg += cm;
      msg += " cm.";
      tmq.enqueue(USER_ID, std::move(msg));
    }
  }
}

void bmeSensorProcessing() {
  float t, h, p;
  h = bme.readHumidity();
  t = bme.readTemperature();
  p = bme.seaLevelForAltitude(ALTITUDE, bme.readPressure()) * 0.0075f;  // Convert pascals to mmHg

  Serial.print("Temperature = ");
  Serial.println(t);
  Serial.print("Humidity = ");
  Serial.println(h);
  Serial.print("Pressure = ");
  Serial.println(p);

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
}

void updateLCD(long cm) {
  lcd.clear();

  // Show WiFi signal information
  lcd.setCursor(0, 0);
  auto ws = WiFi.status();
  if (ws != WL_CONNECTED) {
    lcd.print("WiFi: OFF(");
    lcd.print(ws);
    lcd.print(")");
  } else {
    lcd.print("WiFi: ON(");
    long rssi = WiFi.RSSI();
    lcd.print(rssi);
    lcd.print("dBm)");
  }

  // Show distance
  lcd.setCursor(0, 1);
  lcd.print("Car ");
  lcd.print(car_state_strings[car_presence]);
  lcd.print(": ");
  lcd.print(cm);
  lcd.print("cm");

  // Show time
  lcd.setCursor(0, 2);
  lcd.print("Up: ");
  unsigned long days, hours, minutes, seconds;
  unsigned long mi = millis();
  getTime(mi / 1000, days, hours, minutes, seconds);
  lcd.print(days);
  lcd.print("d, ");
  lcd.print(hours);
  lcd.print(":");
  lcd.print(minutes);
  lcd.print(":");
  lcd.print(seconds);

  lcd.setCursor(0, 3);
  lcd.print("Millis: ");
  lcd.print(mi);
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

  WiFi.setAutoReconnect(true);
}
