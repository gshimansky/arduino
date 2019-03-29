/*
QMC5883L.cpp - Class file for the QMC5883L Triple Axis Magnetometer Arduino Library.

Copyright (C) 2017 Andy Barnard based on an original 2011 for the HMC5883L by Love Electronics

 WARNING: THE QMC5883L IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for QMC5883L:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/QMC5883L_3-Axis_Digital_Compass_IC.pdf

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.


*/

#include <Arduino.h> 
#include "QMC5883L.h"

QMC5883L::QMC5883L()
{
  
}

MagnetometerRaw QMC5883L::ReadRawAxis(QMC5883L_ODR* dataRegister)
{
  // also read DOR/OVL/DRDY and temperature data whilst we are about it...
  Read(DataRegisterBegin, 9, dataRegister);
  MagnetometerRaw raw = MagnetometerRaw();
  // QMC5883L is the other way around and comes X,Y,Z
  // answer is in 2's complement becuase XAxis etc are ints...
  raw.XAxis = (dataRegister->X_MSB << 8) | (dataRegister->X_LSB);
  raw.YAxis = (dataRegister->Y_MSB << 8) | (dataRegister->Y_LSB);
  raw.ZAxis = (dataRegister->Z_MSB << 8) | (dataRegister->Z_LSB);
return raw;
}

MagnetometerScaled QMC5883L::ReadScaledAxis(QMC5883L_ODR* dataRegister)
{
  MagnetometerRaw raw = ReadRawAxis(dataRegister);
  MagnetometerScaled scaled = MagnetometerScaled();
// for debugging
/*
Serial.print("dataRegister->X_MSB ");
Serial.print(dataRegister->X_MSB, BIN);
Serial.print(" dataRegister->X_LSB ");
Serial.print(dataRegister->X_LSB, BIN);
Serial.print(" raw.Xaxis ");
Serial.print(raw.XAxis, BIN);
Serial.println(" ");
Serial.print("dataRegister->Y_MSB ");
Serial.print(dataRegister->Y_MSB, BIN);
Serial.print(" dataRegister->Y_LSB ");
Serial.print(dataRegister->Y_LSB, BIN);
Serial.print(" raw.Yaxis ");
Serial.print(raw.YAxis, BIN);
Serial.println(" ");
Serial.print("dataRegister->Z_MSB ");
Serial.print(dataRegister->Z_MSB, BIN);
Serial.print(" dataRegister->Z_LSB ");
Serial.print(dataRegister->Z_LSB, BIN);
Serial.print(" raw.Zaxis ");
Serial.print(raw.ZAxis, BIN);
Serial.println(" ");
*/
// end of bit for debugging
  scaled.XAxis = raw.XAxis * m_Scale;
  scaled.YAxis = raw.YAxis * m_Scale;
  scaled.ZAxis = raw.ZAxis * m_Scale;
  return scaled;
}

int QMC5883L::Configure(QMC5883L_ODR dataRegister)
{
	Write(ControlRegister1, dataRegister.OSR_RNG_ODR_MODE);
	Write(ControlRegister2, dataRegister.CR2_INT_ENABLE);
	Write(SetResetPeriodRegister, dataRegister.SET_RESET_PERIOD);

// as its 2's complement, fullsscale range is either 2 or 8 gauss per 15 bits (32768) 
switch (dataRegister.OSR_RNG_ODR_MODE & 0b00010000) {
 case 0b00010000: 
     m_Scale = (float) 8 / (float) 32768;
     break;
  default:
     m_Scale = (float) 2 / (float) 32768;
  break;
}

     return 0;
}


void QMC5883L::Write(int address, int data)
{
  Wire.beginTransmission(QMC5883L_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

void QMC5883L::Read(int address, int length, QMC5883L_ODR* dataRegister)
{
  Wire.beginTransmission(QMC5883L_Address);
  Wire.write(address);
  Wire.endTransmission();
  

  Wire.beginTransmission(QMC5883L_Address);
  Wire.requestFrom(QMC5883L_Address, length);

  byte *reg = &(dataRegister->X_LSB);  // write into the 
  
  if(Wire.available() == length)
  {
 for(int i = 0; i < length; i++)
	  {
		reg[i] = Wire.read();
// for debugging to see what came from the registers...
/*
Serial.print("read ");
Serial.print(reg[i], BIN);
Serial.print(" from address ");
Serial.println(address + i, HEX);
*/
	  }
  }
  Wire.endTransmission();

  return;
}

String QMC5883L::GetErrorText(int errorCode)
{
	return String("Error code "+String(errorCode));
}