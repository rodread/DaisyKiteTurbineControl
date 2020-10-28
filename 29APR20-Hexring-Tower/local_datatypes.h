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
 Name:			local_datatypes.h
 Created:		22/09/2016
 Author:		TS
*/

#ifndef LOCAL_DATATYPES_H_
#define LOCAL_DATATYPES_H_

// Added by AC to store measured values
struct bldcMeasure
{
	float tempPcb;
	float avgMotorCurrent;
	float avgInputCurrent;
	float dutyCycleNow;
	long rpm;
	//int32_t rpm;
	float inpVoltage;
	float ampHours;
	float ampHoursCharged;
	int32_t tachometer;
	int32_t tachometerAbs;
};

#if defined (USE_PATCHED_VESC_FW_2_18)
// Added by AC to store measured values
struct bldcLimits
{
	float currentMax;
	float currentMin;
	float inCurrentMax;
	float inCurrentMin;
	float absCurrentMax;
	float minVin;
	float maxVin;
	float batteryCutStart;
	float batteryCutEnd;
	float tempFetStart;
	float tempFetEnd;
	float tempMotorStart;
	float tempMotorEnd;
	float max_Duty;
};
#endif //if defined (USE_PATCHED_VESC_FW_2_18)

#endif
