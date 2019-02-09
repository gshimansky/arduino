#include "settings.hpp"
#include <Wire.h>

const int DISTANCE = 140; // Distance in cm to turn on red light
const int trigPin = D0;
const int echoPin = D1;
const int redpin = D3; // select the pin for the red LED
const int greenpin = D2; // select the pin for the green LED

long duration, cm;
int val;

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  delay(10);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
}

/*
void loop() {
  for(val = 255; val > 0; val--)
  {
    analogWrite(redpin, val);
    analogWrite(bluepin, 255 - val);
    analogWrite(greenpin, 128 - val);

    Serial.println(val, DEC);
    delay(5); 
  }
  for(val = 0; val < 255; val++)
  {
    analogWrite(redpin, val);
    analogWrite(bluepin, 255 - val);
    analogWrite(greenpin, 128 - val);
    
    Serial.println(val, DEC);
    delay(5); 
  }
}
*/

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
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343

  if (cm < DISTANCE) {
    // Show red
    analogWrite(redpin, 0);
    analogWrite(greenpin, 1024);
  } else {
    // Show green
    analogWrite(redpin, 1024);
    analogWrite(greenpin, 0);
  }

  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  
  delay(250);
}
