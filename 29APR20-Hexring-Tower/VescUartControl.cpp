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
 Name:			VescUartControl.cpp
 Created:		22/09/2016
 Author:		TS
*/

//#if defined(__arm__) && (defined(__MK20DX128__) || defined(__MK20DX256__))
// ******************************************************************************
// Teensy 3.0 implementation, using Programmable Delay Block
// ******************************************************************************

#include "VescUartControl.h" //include the declaration for this class
#include "buffer.h"
#include "crc.h"

/** Default constructor, uses default serial port.
 * @see serial
 */
VescController::VescController()
{
  _Serial = NULL;
}

/** Class destructor to free up memory */
VescController::~VescController()
{
	// Do nothing
}

/** Specific address constructor.
 * @param serialIn defines the serial used by the instance of the class
 */
void VescController::begin(HardwareSerial *serialIn)
{
  _Serial =serialIn;
  _Serial->begin(115200);
}

/** Setpoint for the current controller
 * @param current defines the driving current the controller should set
 */
void VescController::UartSetCurrent(float current)
{
	int32_t index = 0;
	uint8_t payload[5];
		
	payload[index++] = COMM_SET_CURRENT ;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	PackSendPayload(payload, 5);
}

/** Setpoint for the current controller
 * @param current defines the braking current the controller should set
 */
void VescController::UartSetCurrentBrake(float brakeCurrent)
{
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);
	PackSendPayload(payload, 5);
}

int16_t VescController::PackSendPayload(uint8_t* payload, int lenPay)
{
	uint16_t crcPayload = crc16(payload, lenPay);
	int16_t count = 0; //TODO original int
	uint8_t messageSend[256];
	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}
	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = (uint8_t)NULL;
	//Sending package
	_Serial->write(messageSend, count);
	_Serial->flush();
	//serial.flush() needed to wait until serial send buffer is written

	//Returns number of send bytes
	return count;
}

bool VescController::UartGetValue(bldcMeasure& values)
{
	uint8_t command[1] = { COMM_GET_VALUES };
	uint8_t payload[256];
	PackSendPayload(command, 1);
	delay(100); //needed, otherwise data is not read it is needed in addition to _Serial.flush()
	int lenPayload = ReceiveUartMessage(payload);

	if (lenPayload > 55) {
		bool read = ProcessReadPacket(payload, values, lenPayload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

#if defined (USE_PATCHED_VESC_FW_2_18)
bool VescController::UartGetLimits(bldcLimits& limits)
{
	uint8_t command[1] = { COMM_GET_LIMITS };
	uint8_t payload[256];
	PackSendPayload(command, 1);
	delay(100); //needed, otherwise data is not read it is needed in addition to _Serial.flush()
	int lenPayload = ReceiveUartMessage(payload);

	if (lenPayload > 56)
	{
		bool read = ProcessReadPacketLimits(payload, limits, lenPayload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}
#endif //if defined (USE_PATCHED_VESC_FW_2_18)

int16_t VescController::ReceiveUartMessage(uint8_t* payloadReceived)
{
	//Messages <= 255 start with 2. 2nd byte is length
	//Messages >255 start with 3. 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF
	 
	uint16_t counter = 0; //TODO original uint8_t
	int16_t endMessage = 256; //TODO original int
	bool messageRead = false;
	uint8_t messageReceived[256];
	int16_t lenPayload = 0; //TODO original int
	while (_Serial->available())
	{
		messageReceived[counter++] = _Serial->read();

		if (counter == 2)
		{//case if state of 'counter' with last read 1

			switch (messageReceived[0])
			{
			case 2:
				lenPayload = messageReceived[1];
				endMessage = lenPayload + 5; //Payload size + 2 for size + 3 for CRC and End.
				break;
			case 3:
				//ToDo: Add Message Handling > 255 (starting with 3)
				break;
			default:
				break;
			}

		}

		if (counter >= sizeof(messageReceived))
		{
			break;
		}

		if (counter == endMessage && messageReceived[endMessage - 1] == 3)
		{//+1: Because of counter++ state of 'counter' with last read = "endMessage"
			messageReceived[endMessage] = 0;		
			messageRead = true;
			break; //Exit if end of message is reached, even if there is still more data in buffer. 
		}
	}

	bool unpacked = false;
	if (messageRead)
	{
		unpacked = UnpackPayload(messageReceived, endMessage, payloadReceived, lenPayload);
	}
	if (unpacked)
	{
		return lenPayload; //Message was read

	}
	else
	{
		return 0; //No Message Read
	}
}

bool VescController::UnpackPayload(uint8_t* message, int lenMes, uint8_t* payload, int lenPay)
{
	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;
	//Rebuild src:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	//Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if (crcPayload == crcMessage)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool VescController::ProcessReadPacket(uint8_t* message, bldcMeasure& values, int len)
{
	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++;//Eliminates the message id
	len--;

	switch (packetId)
	{
	case COMM_GET_VALUES:
		ind = 2; //skip 2byte float16 ADC_IND_TEMP_MOS1
		ind += 2; //skip 2byte float16 ADC_IND_TEMP_MOS2
		ind += 2; //skip 2byte float16 ADC_IND_TEMP_MOS3
		ind += 2; //skip 2byte float16 ADC_IND_TEMP_MOS4
		ind += 2; //skip 2byte float16 ADC_IND_TEMP_MOS5
		ind += 2; //skip 2byte float16 ADC_IND_TEMP_MOS6
		values.tempPcb = buffer_get_float16(message, 10.0, &ind);
		values.avgMotorCurrent = buffer_get_float32(message, 100.0, &ind);
		values.avgInputCurrent = buffer_get_float32(message, 100.0, &ind);
		values.dutyCycleNow = buffer_get_float16(message, 1000.0, &ind);
		values.rpm = buffer_get_float32(message, 1.0, &ind);
		values.inpVoltage = buffer_get_float16(message, 10.0, &ind);
		values.ampHours = buffer_get_float32(message, 10000.0, &ind);
		values.ampHoursCharged = buffer_get_float32(message, 10000.0, &ind);
		ind += 4; //skip 4byte float32 watt_hours
		ind += 4; //Skip 4byte float32 watt_hours_charged
		values.tachometer = buffer_get_int32(message, &ind);
		values.tachometerAbs = buffer_get_int32(message, &ind);
		return true;
		break;

	default:
		return false;
		break;
	}

}

#if defined (USE_PATCHED_VESC_FW_2_18)
bool VescController::ProcessReadPacketLimits(uint8_t* message, bldcLimits& limits, int len)
{
	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];

	message++;//Eliminates the message id
	len--;

	switch (packetId)
	{
	case COMM_GET_LIMITS:
		limits.currentMax=buffer_get_float32(message, 1000.0, &ind);
		limits.currentMin=buffer_get_float32(message, 1000.0, &ind);
		limits.inCurrentMax=buffer_get_float32(message, 1000.0, &ind);
		limits.inCurrentMin=buffer_get_float32(message, 1000.0, &ind);
		limits.absCurrentMax=buffer_get_float32(message, 1000.0, &ind);
		limits.minVin=buffer_get_float32(message, 1000.0, &ind);
		limits.maxVin=buffer_get_float32(message, 1000.0, &ind);
		limits.batteryCutStart=buffer_get_float32(message, 1000.0, &ind);	
		limits.batteryCutEnd=buffer_get_float32(message, 1000.0, &ind);

		limits.tempFetStart=buffer_get_float32(message, 1000.0, &ind);
		limits.tempFetEnd=buffer_get_float32(message, 1000.0, &ind);
		limits.tempMotorStart=buffer_get_float32(message, 1000.0, &ind);
		limits.tempMotorEnd=buffer_get_float32(message, 1000.0, &ind);
		limits.max_Duty=buffer_get_float32(message, 1000000.0, &ind);
		
		return true;
		break;

	default:
		return false;
		break;
	}
}
#endif //if defined (USE_PATCHED_VESC_FW_2_18)

//#endif defined(__arm__) && (defined(__MK20DX128__) || defined(__MK20DX256__))
