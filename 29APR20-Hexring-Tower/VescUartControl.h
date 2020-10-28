/*
	Copyright 2015 - 2016 Andreas Chaitidis Andreas.Chaitidis@gmail.com
	Copyright 2016 Tobias Sachs Tobias.Sachs@onlinehome.de

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 created by:	Andreas Chaitidis
 derived by:	Tobias Sachs
 Name:			VescUartControl.h
 Created:		22/09/2016
 Author:		TS
*/

#ifndef VescUartControl_h
#define VescUartControl_h

//Only comment out if you use VESC with patched firmware for the Command COMM_GET_LIMITS
//#define USE_PATCHED_VESC_FW_2_18

#include "Arduino.h"
#include "datatypes.h"
#include "local_datatypes.h"

class VescController
{
  public:
    VescController();
    ~VescController();

    void begin(HardwareSerial *serialIn);

    void UartSetCurrent(float current);
    void UartSetCurrentBrake(float brakeCurrent);
	
	bool UartGetValue(bldcMeasure& values);

#if defined (USE_PATCHED_VESC_FW_2_18)
	bool UartGetLimits(bldcLimits& limits);
#endif //if defined (USE_PATCHED_VESC_FW_2_18)

  private:
    HardwareSerial *_Serial;
	HardwareSerial *_debugSerial;
	
	int16_t PackSendPayload(uint8_t* payload, int lenPay);
	
	int16_t ReceiveUartMessage(uint8_t* payloadReceived);
	bool UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay);
	bool ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len);

#if defined (USE_PATCHED_VESC_FW_2_18)
	bool ProcessReadPacketLimits(uint8_t* message, bldcLimits& limits, int len);
#endif //if defined (USE_PATCHED_VESC_FW_2_18)
};

#endif
