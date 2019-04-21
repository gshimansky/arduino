#include <Wire.h>
#include <QMC5883L.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif
#include <WiFiClientSecure.h>
#include <SH1106Wire.h>
#include <OLEDDisplay.h>

#include "settings.hpp"
#include "message_queue.hpp"

#if defined(ARDUINO_ARCH_ESP32)
const int SDAPin = SDA;  // PIN 21
const int SCLPin = SCL;  // PIN 22
#else
const int SDAPin = D5;
const int SCLPin = D4;
#endif

// configure the compass as reqiured
#define OSR 0b00               // over sampling rate set to 512. 0b01 is 256, 0b10 is 128 and 0b11 is 64
#define RNG 0b00               // Full Scale set to +/- 2 Gauss, 0b01 is +/- 8.
#define ODR 0b00               // output data rate set to 10Hz, 0b01 is 50Hz, 0b10 is 100Hz, 0b11 is 200Hz
#define MODE 0b01              // continuous measurement mode, 0b00 is standby
#define CR2 0b00000000          // control register 2: disable soft reset and pointer rollover, interrupt pin not enabled
#define RESETPERIOD 0b00000001  // datasheet recommends this be 1, not sure why!

// QMC5883L Address is 0x0D
QMC5883L compass;

short magnetic_threshold = 0; // Raw magnetic value below which door is considered open
const String door_state_strings[] = {"UNKNOWN", "OPEN", "CLOSED"};
const int door_state_delay_ms = 5000;
enum {
  DOOR_UNKNOWN = 0,
  DOOR_OPEN = 1,
  DOOR_CLOSED = 2
} new_door_state, last_changed_door_state = DOOR_UNKNOWN, door_state = DOOR_UNKNOWN;
long door_state_change_millis = 0, last_state_change = 0;

//////////////////////////////////////
// Telegram communication section
//////////////////////////////////////

// Last time telegram server was checked for incoming messages
long telegram_bot_lasttime;
const int telegram_check_delay_ms = 2000;

WiFiClientSecure secure_client;
UniversalTelegramBot bot(BOT_TOKEN, secure_client);
TelegramMessageQueue tmq(bot);

//////////////////////////////////////
// OLED display
//////////////////////////////////////

// OLED display address is 0x3c
SH1106Wire display(0x3c, 21, 22);

void setup() {
  // Initializing serial port for debugging purposes
  Serial.begin(115200);
  delay(10);

#if defined(ARDUINO_ARCH_ESP8622)
  // Workaround for Arduino SSL bug
  // From here https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot/issues/100#issuecomment-450145611
  client.setFingerprint("BB DC 45 2A 07 E3 4A 71 33 40 32 DA BE 81 F7 72 6F 4A 2B 6B");
#endif

  Wire.begin(SDAPin, SCLPin);

  Serial.println("Constructing new QMC5883L");
  compass = QMC5883L(); // Construct a new QMC5883 compass.
  int error = 0;

  // Check that a device responds at the compass address - don't continue if it doesn't - 
  do {
      delay(100);
      Wire.beginTransmission(QMC5883L_Address);
      error = Wire.endTransmission();
      if (error)
        Serial.println("Can't find compass - is it connected and powered up?");
  } while (error);

  // configure the control registers using static settings above
  // compass autoranges, but starts in the mode given
  compass.dataRegister.OSR_RNG_ODR_MODE = (OSR << 6) | (RNG << 4) | (ODR << 2) | MODE;
  compass.dataRegister.CR2_INT_ENABLE = CR2;
  compass.dataRegister.SET_RESET_PERIOD = RESETPERIOD;

  Serial.println("Configuring QMC5883L - OSR 512, range +/-2 Gauss, ODR 10, Continuous");
  error = compass.Configure(compass.dataRegister); // use static settings from above - can access register data directly if required..
  if(error != 0) // If there is an error, print it out, although no way to get error with this sensor....
    Serial.println(compass.GetErrorText(error));

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

  WiFi.setAutoReconnect(true);

  // Printing the ESP IP address
  Serial.println(WiFi.localIP());

  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);  // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high

  display.init();
  display.setFont(ArialMT_Plain_10);
}

void getTime(long all_seconds, long &days, long &hours, long &minutes, long &seconds) {
  seconds = all_seconds % 60;
  all_seconds /= 60;
  minutes = all_seconds % 60;
  all_seconds /= 60;
  hours = all_seconds % 24;
  days = all_seconds / 24;
}

void sendStatusMessage(String &&chat_id, int mag) {
  long days, hours, minutes, seconds;
  getTime(millis() / 1000, days, hours, minutes, seconds);

  String reply((char *)0);
  reply.reserve(256);
  reply += "Door bot has been running for ";
  reply += days;
  reply += " days ";
  reply += hours;
  reply += ":";
  reply += minutes;
  reply += ":";
  reply += seconds;
  reply += ". Door is ";
  reply += door_state_strings[door_state];
  reply += ".\nCurrent threshold is ";
  reply += magnetic_threshold;
  reply += ".\n";

  if (door_state_change_millis != 0) {
    getTime((millis() - door_state_change_millis) / 1000, days, hours, minutes, seconds);

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

  reply += "Last raw magnetic value was ";
  reply += mag;
  reply += ".";
  tmq.enqueue(std::move(chat_id), std::move(reply));
}

void handleNewMessages(int numNewMessages, short mag) {
  Serial.print("handleNewMessages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = bot.messages[i].chat_id;
    const String &text = bot.messages[i].text;
    const String &from_name = bot.messages[i].from_name == "" ? "Guest" : bot.messages[i].from_name;

    if (text == "/status") {
      sendStatusMessage(std::move(chat_id), mag);
    } else if (text == "/start" || text == "/help") {
      String keyboardJson = "[[\"/status\", \"/threshold\"]]";
      String welcome((char *)0);
      welcome.reserve(256);
      welcome += "Welcome to garage door status bot, ";
      welcome += from_name;
      welcome += ".\n/status : to get current status.\n";

      tmq.enqueue(std::move(chat_id), std::move(welcome), std::move(keyboardJson), true);
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
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis(&compass.dataRegister);
  short x = static_cast<short>(raw.XAxis);
  short y = static_cast<short>(raw.YAxis);
  short z = static_cast<short>(raw.ZAxis);

  Serial.print("Raw (X,Y,Z): (");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(z);
  Serial.println(")");

  // Invoke door state detection logic
  doorStateProcessing(x);

  // Update information on OLED display
  updateOLED(x, y, z);

  // Telegram inpue message queue processing
  if (millis() - telegram_bot_lasttime > telegram_check_delay_ms) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while(numNewMessages) {
      handleNewMessages(numNewMessages, x);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    telegram_bot_lasttime = millis();
  }

  // Telegram output messages queue processing
  tmq.send_one();

  // every 0.5 second
  delay(500);
}

void doorStateProcessing(short mag) {
  // Detect car presence
  if (mag < magnetic_threshold) {
    new_door_state = DOOR_OPEN;
  } else {
    new_door_state = DOOR_CLOSED;
  }

  if (last_changed_door_state != new_door_state) {
    last_state_change = millis();
    last_changed_door_state = new_door_state;
  } else {
    long cur_time = millis();
    if ((cur_time - last_state_change > door_state_delay_ms) &&
      (new_door_state != door_state)) {
      long days, hours, minutes, seconds;
      getTime((millis() - door_state_change_millis) / 1000, days, hours, minutes, seconds);
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
      timestr += door_state_strings[door_state];
      timestr += ".";

      door_state = new_door_state;
      door_state_change_millis = cur_time;

      String debugMsg((char *)0);
      debugMsg.reserve(256);
      debugMsg += "Changing door state to ";
      debugMsg += door_state_strings[door_state];
      debugMsg += timestr;
      Serial.println(debugMsg);

      String msg((char *)0);
      msg.reserve(256);
      msg += "Door is now ";
      msg += door_state_strings[door_state];
      msg += timestr;
      msg += " Magnetic value is ";
      msg += mag;
      msg += ".";
      tmq.enqueue(USER_ID, std::move(msg));
    }
  }
}

void updateOLED(short x, short y, short z) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  String msgWiFi((char *)0);
  msgWiFi.reserve(64);
  if (WiFi.status() != WL_CONNECTED) {
    msgWiFi += "WiFi: OFF(";
    msgWiFi += WiFi.status();
    msgWiFi += ")";
  } else {
    msgWiFi += "WiFi: ON(";
    long rssi = WiFi.RSSI();
    msgWiFi += rssi;
    msgWiFi += "dbm)";
  }
  display.drawString(0, 0, msgWiFi);

  String msgDoor((char *)0);
  msgDoor.reserve(64);
  msgDoor += "Door ";
  msgDoor += door_state_strings[door_state];
  display.drawString(0, 12, msgDoor);

  String msgTime((char *)0);
  msgTime.reserve(64);
  long days, hours, minutes, seconds;
  getTime((millis() - door_state_change_millis) / 1000, days, hours, minutes, seconds);
  msgTime += "for ";
  msgTime += days;
  msgTime += "d ";
  msgTime += hours;
  msgTime += ":";
  msgTime += minutes;
  msgTime += ":";
  msgTime += seconds;
  msgTime += ".";
  display.drawString(0, 24, msgTime);

  String msgUp((char *)0);
  msgUp.reserve(64);
  getTime(millis() / 1000, days, hours, minutes, seconds);
  msgUp += "Uptime: ";
  msgUp += days;
  msgUp += "d ";
  msgUp += hours;
  msgUp += ":";
  msgUp += minutes;
  msgUp += ":";
  msgUp += seconds;
  msgUp += ".";
  display.drawString(0, 36, msgUp);

  String msgMag2((char *)0);
  msgMag2.reserve(64);
  msgMag2 += x;
  msgMag2 += ", ";
  msgMag2 += y;
  msgMag2 += ", ";
  msgMag2 += z;
  display.drawString(0, 48, msgMag2);

  display.display();
}
