/*
QMC5883L_Example.ino - Example sketch for integration with an QMC5883L triple axis magnetometer.
Copyright (C) 2017 Andy Barnard based on an original for the QMC5883L by Love Electronics (C) 2011

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

See <http://www.gnu.org/licenses/>.

*/

// Reference the I2C Library
#include <Wire.h>
// Reference the QMC5883L Compass Library
#include <QMC5883L.h>

// configure the compass as reqiured
#define OSR 0b00               // over sampling rate set to 512. 0b01 is 256, 0b10 is 128 and 0b11 is 64
#define RNG 0b00               // Full Scale set to +/- 2 Gauss, 0b01 is +/- 8.
#define ODR 0b00               // output data rate set to 10Hz, 0b01 is 50Hz, 0b10 is 100Hz, 0b11 is 200Hz
#define MODE 0b01              // continuous measurement mode, 0b00 is standby
#define CR2 0b00000000          // control register 2: disable soft reset and pointer rollover, interrupt pin not enabled
#define RESETPERIOD 0b00000001  // datasheet recommends this be 1, not sure why!

// Store our compass as a variable.
QMC5883L compass;
// Record any errors that may occur in the compass.
int error = 0;

// Out setup routine, here we will configure the microcontroller and compass.
void setup()
{
  // Initialize the serial port.
  Serial.begin(115200);

  Serial.println("Starting the I2C interface.");
  Wire.begin(SDA, SCL); // Start the I2C interface.
//  TWBR = 12;    //Set the I2C clock speed to 400kHz - only works with Arduino UNO

  Serial.println("Constructing new QMC5883L");
  compass = QMC5883L(); // Construct a new HMC5883 compass.

  // Check that a device responds at the compass address - don't continue if it doesn't - 
  do {
      delay(100);
      Wire.beginTransmission(QMC5883L_Address);
      error = Wire.endTransmission();
      if (error) Serial.println("Can't find compass - is it connected and powered up?");
  } while (error);

  // configure the control registers using static settings above
  // compass autoranges, but starts in the mode given
  compass.dataRegister.OSR_RNG_ODR_MODE = (OSR << 6) |(RNG << 4)  | (ODR <<2) |  MODE;
  compass.dataRegister.CR2_INT_ENABLE = CR2;
  compass.dataRegister.SET_RESET_PERIOD = RESETPERIOD;
    
  Serial.println("Configuring QMC5883L - OSR 512, range +/-2 Gauss, ODR 10, Continuous");
  error = compass.Configure(compass.dataRegister); // use static settings from above - can access register data directly if required..
  if(error != 0) // If there is an error, print it out, although no way to get error with this sensor....
    Serial.println(compass.GetErrorText(error));
  
}

// Our main program loop.
void loop()
{
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis(&compass.dataRegister);
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis(&compass.dataRegister);
  
  // Values are accessed like so:
  int Gauss_OnThe_XAxis = scaled.XAxis;     // (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  // relative to a magnetometer mounted with 'Y' axis pointing in the direction to be measured
  // and with 'X' pointing to the right from the perspective of the operator facing in the +Y direction
  // heading (degrees): 0 = +X, 90 = +Y, 180 = -X, 270 = -Y
  float heading = atan2(scaled.YAxis, scaled.XAxis);
 
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Example is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  // float declinationAngle = 0.0457;
  float declinationAngle = 0;  // compass reads magnetic north for those who remember how to read maps and adjust for declination
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Output the data via the serial port.
  Output(raw, scaled, heading, headingDegrees);

  // Normally we would either:
  // 1. delay the application by 100ms to allow the loop to run at 10Hz (default bandwidth for the QMC5883L)
  // 2. poll the dataready flag in the dataRegister.OVL_DRDY register
  // 3. set the interrupt flat and set a hardware interrupt on the DRDY pin 
  // The first of these options is the easiest.
  delay(300);
}

// Output the data down the serial port.
void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
{
   Serial.print("Raw (X,Y,Z): (");
   Serial.print((short)raw.XAxis);
   Serial.print(", ");   
   Serial.print((short)raw.YAxis);
   Serial.print(", ");   
   Serial.print((short)raw.ZAxis);
   Serial.println(")");
/*
   Serial.print(")\tScaled (X,Y,Z): (");
   Serial.print(scaled.XAxis, 4);
   Serial.print(", ");   
   Serial.print(scaled.YAxis, 4);
   Serial.print(", ");   
   Serial.print(scaled.ZAxis, 4);
   Serial.println(")");
*/
/*
   Serial.print("Magnitude (0.25 to 0.6 on Earth surface): ");
   Serial.print(sqrt(scaled.XAxis * scaled.XAxis + scaled.YAxis * scaled.YAxis + scaled.ZAxis * scaled.ZAxis));
   Serial.print(" Heading: ");
   Serial.print(headingDegrees);
   Serial.print(" Bearing: ");
   Serial.print(bearingDegrees(headingDegrees));
   Serial.println(" (Degrees)");
*/
}

  // Cacluate bearing from heading.
  // relative to a magnetometer mounted with 'Y' axis pointing in the direction to be measured
  // and with 'X' pointing to the right from the perspective of the operator facing in the +Y direction
  // bearing 0 = Y pointing North, 90 = Y pointing E, 180 = Y pointing S, 270 = Y pointing W
  float bearingDegrees(float headingDegrees) {
     
     float bearing = headingDegrees - 90;
     if (bearing < 0)
     {
      bearing += 360;
     }
     return bearing;
  }
