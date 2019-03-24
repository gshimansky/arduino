#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "settings.hpp"

#ifdef BME280_ADDRESS
#undef BME280_ADDRESS
#define BME280_ADDRESS 0x76
#endif

//#define BMP_SCK 13
//#define BMP_MISO 12
//#define BMP_MOSI 11
//#define BMP_CS 10

float t, h, p, pmm, dp;
char temperatureString[6];
char humidityString[6];
char pressureString[7];
char dpString[6];

Adafruit_BME280 bme; // I2C

const char* server = "api.thingspeak.com";
WiFiClient client;

/**************************
 *   S E T U P
 **************************/
// only runs once on boot
void setup() {
  // Initializing serial port for debugging purposes
  Serial.begin(115200);
  delay(10);
  // Default settings
  // D1 - SCL
  // D2 - SDA
//  Wire.begin(D3, D4); // Make sure you have D3 & D4 hooked up to the BME280
//  Wire.setClock(100000);
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
  Serial.println(F("BME280 test"));

  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

/**************************
 *  L O O P
 **************************/
void loop() {
    h = bme.readHumidity();
    t = bme.readTemperature();
    p = bme.seaLevelForAltitude(ALTITUDE, bme.readPressure()); // Pressure in pascals
    pmm = p * 0.0075f;  // Convert pascals to mmHg

//    t = t*1.8+32.0;  // Convert to Farengeith
//    dp = t-0.36*(100.0-h);

    dtostrf(t, 5, 1, temperatureString);
    dtostrf(h, 5, 1, humidityString);
    dtostrf(pmm, 6, 1, pressureString);

//    dtostrf(dp, 5, 1, dpString);
    delay(10000);

    Serial.print("Temperature = ");
    Serial.println(temperatureString);
    Serial.print("Humidity = ");
    Serial.println(humidityString);
    Serial.print("Pressure = ");
    Serial.println(pressureString);
//    Serial.print("Dew Point = ");
//    Serial.println(dpString);

    if (client.connect(server,80))  // "184.106.153.149" or api.thingspeak.com
    {
        String postStr = apiKey;
        postStr +="&field1=";
        postStr += String(temperatureString);
        postStr +="&field2=";
        postStr += String(humidityString);
        postStr +="&field3=";
        postStr += String(pressureString);
        postStr += "\r\n\r\n";

        client.print("POST /update HTTP/1.1\n");
        client.print("Host: api.thingspeak.com\n");
        client.print("Connection: close\n");
        client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
        client.print("Content-Type: application/x-www-form-urlencoded\n");
        client.print("Content-Length: ");
        client.print(postStr.length());
        client.print("\n\n");
        client.print(postStr);
    }
    client.stop();
    //every 5 Min
    delay(300000);
}
