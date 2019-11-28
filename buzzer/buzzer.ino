#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecureBearSSL.h>

#include "settings.hpp"
#include "message_queue.hpp"

#define WIFI_RESTART_TIMEOUT_MS (60 * 1000)

//////////////////////////////////////
// Buzzer
//////////////////////////////////////

bool buzzer_on = false;
int buzzerPin = D1;

//////////////////////////////////////
// Telegram communication section
//////////////////////////////////////

// Last time telegram server was checked for incoming messages
unsigned long telegram_bot_lasttime;
const int telegram_check_delay_ms = 5000;
const uint8_t api_telegram_org_fingerprint[20] = {0xBB, 0xDC, 0x45, 0x2A, 0x07, 0xE3, 0x4A, 0x71, 0x33, 0x40, 0x32, 0xDA, 0xBE, 0x81, 0xF7, 0x72, 0x6F, 0x4A, 0x2B, 0x6B};
BearSSL::WiFiClientSecure secure_client;
UniversalTelegramBot bot(BOT_TOKEN, secure_client);
TelegramMessageQueue tmq(bot);

const int loop_delay_ms = 500;

void setup() {
  //Serial Port begin
  Serial.begin(115200);
  delay(10);

  pinMode (buzzerPin, OUTPUT);
  buzzer_on = false;

  secure_client.setFingerprint(api_telegram_org_fingerprint);

  reconnect();
}

void buzzer_start(void) {
  Serial.println("Starting buzzer");
  buzzer_on = true;
}

void buzzer_stop(void) {
  Serial.println("Stopping buzzer");
  buzzer_on = false;
}

void handleNewMessages(int numNewMessages) {
  Serial.print("handleNewMessages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = bot.messages[i].chat_id;
    const String &text = bot.messages[i].text;
    const String &from_name = bot.messages[i].from_name == "" ? "Guest" : bot.messages[i].from_name;
    const String keyboardJson = "[[\"/start\", \"/stop\", \"/status\", \"/help\"]]";

    if (text == "/status") {
      String reply((char *)0);
      reply.reserve(256);
      reply += "Buzzer is ";
      if (buzzer_on)
        reply += "on\n";
      else
        reply += "off\n";

      tmq.enqueue(std::move(chat_id), std::move(reply), std::move(keyboardJson), true);
    } else if (text == "/start") {
      buzzer_start();
      tmq.enqueue(std::move(chat_id), "Starting buzzer\n", std::move(keyboardJson), true);
    } else if (text == "/stop") {
      buzzer_stop();
      tmq.enqueue(std::move(chat_id), "Stopping buzzer\n", std::move(keyboardJson), true);
    } else if (text == "/help") {
      String welcome((char *)0);
      welcome.reserve(256);
      welcome += "Welcome to home buzzer, ";
      welcome += from_name;
      welcome += ".\n/start : to start buzzing\n"
        "/stop : to stop buzzin\n"
        "/status : to see buzzer status\n"
        "/help : to read this message\n";

      tmq.enqueue(std::move(chat_id), std::move(welcome), std::move(keyboardJson), true);
    } else {
      String msg((char *)0);
      msg.reserve(256);
      msg += "Command not understood: ";
      msg += text;

      tmq.enqueue(std::move(chat_id), std::move(msg), std::move(keyboardJson), true);
    }
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
  }

  // Telegram inpue message queue processing
  if (millis() - telegram_bot_lasttime > telegram_check_delay_ms) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    Serial.print("Got messages ");
    Serial.println(numNewMessages);

    while(numNewMessages) {
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    telegram_bot_lasttime = millis();
  }

  // Telegram output messages queue processing
  tmq.send_one();

  if (buzzer_on) {
    for (int i = 0; i < 3; i++) {
      digitalWrite (buzzerPin, HIGH);
      delay(100);
      digitalWrite (buzzerPin, LOW);
      delay(100);
    }
    digitalWrite (buzzerPin, HIGH);
    delay(300);
    digitalWrite (buzzerPin, LOW);
    delay(100);
    for (int i = 0; i < 3; i++) {
      digitalWrite (buzzerPin, HIGH);
      delay(100);
      digitalWrite (buzzerPin, LOW);
      delay(100);
    }
  }

  delay(loop_delay_ms);
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
