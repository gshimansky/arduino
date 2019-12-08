#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <EEPROM.h>
#include <LiquidCrystal_PCF8574.h>
#include <ThingSpeak.h>
#include "esp_system.h"

#include "settings.hpp"
#include "message_queue.hpp"

//////////////////////////////////////
// EEPROM
//////////////////////////////////////

const int eeprom_address = 0;
struct eeprom_saved_state {
  int distance_threshold;
  int last_state;
} eeprom_data = {140, 0};

//////////////////////////////////////
// Watchdog
//////////////////////////////////////

#define TIMER_80MHZ_DIVIDER 80
#define RESTART_TIMEOUT_US (30 * 1000 * 1000)
#define WIFI_RESTART_TIMEOUT_MS (60 * 1000)
hw_timer_t *watchdog_timer = NULL;
void IRAM_ATTR resetModule();

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
enum car_status {
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
struct lcd_data {
  wl_status_t wifi_status;
  long wifi_rssi;
  long cm;
  car_status car_presence;
  unsigned long days, hours, minutes, seconds;
  float temp, hum;
};
lcd_data previous_data = {WL_IDLE_STATUS, 0,
  0, CAR_UNKNOWN,
  0, 0, 0, 0,
  0.0f, 0.0f};
#define DEGREE_SYMBOL 1
int degree_symbol[] = {7, 5, 7, 0, 0, 0, 0, 0};
void updateLCD(lcd_data &new_data);

void setup() {
  // Enable watchdog
  watchdog_timer = timerBegin(0, TIMER_80MHZ_DIVIDER, true);
  timerAttachInterrupt(watchdog_timer, resetModule, true);
  timerAlarmWrite(watchdog_timer, RESTART_TIMEOUT_US, false);
  timerAlarmEnable(watchdog_timer);

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

  EEPROM.begin(sizeof(eeprom_data));
  EEPROM.get(eeprom_address, eeprom_data);

  reconnect();

  // initialize the lcd
  lcd.begin(20, 4);
  lcd.noAutoscroll();
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.noCursor();
  lcd.display();
  lcd.createChar(DEGREE_SYMBOL, degree_symbol);

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
  reply += eeprom_data.distance_threshold;
  reply += " centimeters.\nLast state = ";
  reply += eeprom_data.last_state;

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

          eeprom_data.distance_threshold = new_threshold;
          EEPROM.put(eeprom_address, eeprom_data);
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
  eeprom_data.last_state = 1;
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
  }

  long duration;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  eeprom_data.last_state = 2;
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  eeprom_data.last_state = 3;
  // Convert the time into a distance
  long cm = (duration / 2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  Serial.print(cm);
  Serial.println("cm");

  eeprom_data.last_state = 4;
  // Temperature message processing
  if (millis() - bme_sensor_lasttime > bme_check_delay_ms) {
    bmeSensorProcessing();
    bme_sensor_lasttime = millis();
  }

  eeprom_data.last_state = 5;
  // Invoke car detection logic
  carPresenceProcessing(cm);

  eeprom_data.last_state = 6;
  unsigned long days, hours, minutes, seconds;
  unsigned long mi = millis();
  getTime(mi / 1000, days, hours, minutes, seconds);
  lcd_data d = {
      WiFi.status(),
      WiFi.RSSI(),
      cm,
      car_presence,
      days, hours, minutes, seconds,
      bme.readTemperature(),
      bme.readHumidity(),
  };
  // Update information on LCD display
  updateLCD(d);

  eeprom_data.last_state = 7;
  // Telegram inpue message queue processing
  if (millis() - telegram_bot_lasttime > telegram_check_delay_ms) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    eeprom_data.last_state = 8;

    while(numNewMessages) {
      handleNewMessages(numNewMessages, cm);
      eeprom_data.last_state = 9;
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
      eeprom_data.last_state = 10;
    }

    telegram_bot_lasttime = millis();
  }

  eeprom_data.last_state = 11;
  // Telegram output messages queue processing
  tmq.send_one();

  eeprom_data.last_state = 12;
  // Reset watchdog
  timerWrite(watchdog_timer, 0);

  eeprom_data.last_state = 13;
  delay(measure_delay_ms);
}

void carPresenceProcessing(long cm) {
  // Show signal where to stop
#if defined(ARDUINO_ARCH_ESP32)
/*
  if (cm < eeprom_data.distance_threshold) {
    // Show red
    ledcWrite(redpin, 0);
    ledcWrite(greenpin, 1023);
  } else {
    // Show green
    ledcWrite(redpin, 1023);
    ledcWrite(greenpin, 0);
  }
*/
  if (cm < eeprom_data.distance_threshold) {
    // Show red
    digitalWrite(redpin, HIGH);
    digitalWrite(greenpin, LOW);
  } else {
    // Show green
    digitalWrite(redpin, LOW);
    digitalWrite(greenpin, HIGH);
  }
#else
  if (cm < eeprom_data.distance_threshold) {
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
  if (cm < eeprom_data.distance_threshold + presence_addition) {
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

void updateLCD(lcd_data &new_data) {
  // Show WiFi signal information
  if (new_data.wifi_rssi != previous_data.wifi_rssi || new_data.wifi_status != previous_data.wifi_status) {
    lcd.setCursor(0, 0);
    if (new_data.wifi_status != WL_CONNECTED) {
      lcd.print("WiFi: OFF(");
      lcd.print(new_data.wifi_status);
      lcd.print(")         ");
    } else {
      lcd.print("WiFi: ON(");
      lcd.print(new_data.wifi_rssi);
      lcd.print("dBm)");
    }
    previous_data.wifi_status = new_data.wifi_status;
    previous_data.wifi_rssi = new_data.wifi_rssi;
  }

  // Show distance
  if (new_data.cm != previous_data.cm || new_data.car_presence != previous_data.car_presence) {
    lcd.setCursor(0, 1);
    lcd.printf("Car %7s: %4dcm", car_state_strings[new_data.car_presence], new_data.cm);
    previous_data.car_presence = new_data.car_presence;
    previous_data.cm = new_data.cm;
  }

  // Show time
  if (new_data.seconds != previous_data.seconds ||
    new_data.minutes != previous_data.minutes ||
    new_data.hours != previous_data.hours ||
    new_data.days != previous_data.days) {
    lcd.setCursor(0, 2);
    lcd.printf("Up: %dd, %02d:%02d:%02d", new_data.days, new_data.hours, new_data.minutes, new_data.seconds);
    previous_data.days = new_data.days;
    previous_data.hours = new_data.hours;
    previous_data.minutes = new_data.minutes;
    previous_data.seconds = new_data.seconds;
  }

  if (new_data.temp != previous_data.temp || new_data.hum != previous_data.hum) {
    lcd.setCursor(0, 3);
    lcd.printf("T %+2.1f", new_data.temp);
    lcd.write(DEGREE_SYMBOL);
    lcd.write('C');
    lcd.setCursor(11, 3);
    lcd.printf("H %3.1f%%", new_data.hum);
    previous_data.temp = new_data.temp;
    previous_data.hum = new_data.hum;
  }
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
    if (millis() - start_time > WIFI_RESTART_TIMEOUT_MS) {
      Serial.println("No connection in 5 minutes, trying to restart");
      ESP.restart();
    }
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");
  // Printing the ESP IP address
  Serial.println(WiFi.localIP());

  WiFi.setAutoReconnect(true);
}

void IRAM_ATTR resetModule(){
    Serial.print("reboot, last state = \n");
    Serial.println(eeprom_data.last_state);
    EEPROM.put(eeprom_address, eeprom_data);
    EEPROM.commit();
    ESP.restart();
}
