/*
QMC5883L.h - Header file for the QMC5883L Triple Axis Magnetometer Arduino Library.
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

#ifndef QMC5883L_h
#define QMC5883L_h

#include <Arduino.h>
#include <Wire.h>



#define QMC5883L_Address 0x0D
#define ControlRegister1 0x09
#define ControlRegister2 0x0A
#define SetResetPeriodRegister 0x0B
#define DataRegisterBegin 0x00  // QMC5883 from 00H to 05H
#define TempRegisterBegin 0x07H // QMC5883 from 07H to 08H

// mode configured along with scale, oversampling rate and output data rate
//#define Measurement_Continuous 0x00
//#define Measurement_SingleShot 0x01
//#define Measurement_Idle 0x03

struct QMC5883L_ODR
{
byte X_LSB;
byte X_MSB;
byte Y_LSB;
byte Y_MSB;
byte Z_LSB;
byte Z_MSB;
byte OVL_DRDY;   // indicates overload and data ready
byte TOUT_LSB;   // measures temperature
byte TOUT_MSB;
byte OSR_RNG_ODR_MODE;
byte CR2_INT_ENABLE;
byte SET_RESET_PERIOD;
};


struct MagnetometerScaled
{
	float XAxis;
	float YAxis;
	float ZAxis;
};

struct MagnetometerRaw
{
	int XAxis;
	int YAxis;
	int ZAxis;
};

class QMC5883L
{
	public:
	  QMC5883L();

       QMC5883L_ODR dataRegister;

	  MagnetometerRaw ReadRawAxis(QMC5883L_ODR* dataRegister);
	  MagnetometerScaled ReadScaledAxis(QMC5883L_ODR* dataRegister);
 
	  int Configure(QMC5883L_ODR dataRegister);

	  String GetErrorText(int errorCode);
       

	protected:
	  void Write(int address, int byte);
	  void Read(int address, int length, QMC5883L_ODR* dataRegister);

	private:
	  float m_Scale;
};
#endif