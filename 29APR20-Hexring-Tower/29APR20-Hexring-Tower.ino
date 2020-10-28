
/*
Rod Reads Ground Station control software for Rotary Kite Network Airborne Wind Energy by Windswept and Interesting Ltd

Looking to up the speed of the main loop() by removing print parts etc into object oriented...
still need to do to this and change emergency overdrive into an interrupt and flag change.

A max amount of regen current is decided by dial, 
then the amount actually demanded is decided via addition of 3 calculated quotient parts 
Speed trend, TSR and TTR

(TSR)Tip Speed Ratio dial set point will be compared to rotor speed and wind speed data.

Similarly there's a (TTR) Tension Torque Ratio setpoint using tension readings and regen current data

VESC speed trend data will also be taken into account


Problem with VESC interface software   ---   only works on old VESC firmware ~V 3.8   Latest VESC fw > v 6

  Original VESC UART interface work :Copyright 2016 Tobias Sachs Tobias.Sachs@onlinehome.de

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

 created by:  Tobias Sachs 
 messed with by ROD READ for kite ground station controller
 Name:        
 Created:   22/09/2016  messed with again until now
 Author:    TS then Rod R

  by David A. Mellis  <dam@mellis.org>
  modified 9 Apr 2012
  by Tom Igoe
  http://www.arduino.cc/en/Tutorial/Smoothing


//has emergency drive forward override This is going up to 8 amps from 2 - 3 dec 2019  ---  MAKES no difference to VESC SPeed though

//You have to configure the VESC to APP "UART" and set the baudrate to  115200 
// 
//You have to uncommend the line "#define USE_PATCHED_VESC_FW_2_18" in the file VescUartControl.h
// I got rid of the getLimits parts
//
//Could do with getting a more relevant comms set to and from the VESC... I think there are other VESC commands available
//A firmware upgrade for the VESC is available now too...

 Setup your scale and start the sketch WITHOUT a weight on the scale
 Once readings are displayed place the weight on the scale
 Press +/- or a/z to adjust the calibration_factor until the output readings match the known weight
 Arduino pin 6 -> HX711 CLK
 Arduino pin 5 -> HX711 DOUT
 Arduino pin 5V -> HX711 VCC
 Arduino pin GND -> HX711 GND 
*/

#include "VescUartControl.h"

VescController Vesc1;

struct bldcMeasure measuredVESC1;

uint16_t counts = 0;  //unsure... something to do with VESC comms

#include "HX711.h"

float calibration_factor = 10; // this calibration factor is adjusted according to my load cell
float tensionunits;              // approx = grams

float TSRsetpoint;
//float TSRactual;
float TSRaveraged;
float TSRQuota =0.1;

float TTRsetpoint;
float TTRactual;
float TTRMaxValue = 20.0;
float TTRQuota =0.1;

float tipspeed;
float windspeed = 2.0;

float SpeedTrendQuota =0.1;

HX711 scale(5, 6);


const int EMbuttonPin = 8;     // the pushbutton pin for emergency drive forward       DOUBLE CHECK YOU HAVE THIS WIRED
int EM_FW_buttonState = 0;         // variable for reading the pushbutton status

const int StartButtonPin = 2;     // the pushbutton pin for Brake at start
int StartButtonState = 0;         // variable for reading the Brake at start pushbutton status

int sensorPin1 = A0;    // select the input pin for brakecurrent ramp up average compare setpoint
float sensorValue1TSR = 5.0;  // variable to store the value coming from the sensor

int sensorPin2 = A5;    // select the input pin for the brakecurrent ramp up rate
float sensorValue2TTR = 5.0;  // variable to store the value coming from the sensor

int sensorPin7 = A7;    // select the input pin for the brakecurrent max level
float sensorValue7 = 0.1;  // variable to store the value coming from the sensor

float maxbrakecurrent = 0.5; // A limit value for the maximum allowable current
int StartBrakeCurrent = 16.0; // The initial hold brake current before release to generate up from 14


const int numReadings = 4;  //size of array for smoothing rpm readings 
const int numReadingsB = 15;  //size of long array for smoothing rpm readings 

const int numReadings2 = 4;  //size of array for smoothing tension readings 
const int numReadings3 = 3;  //size of array for smoothing windspeed readings 

float readings[numReadings];      // the rpm array
int readIndex = 0;              // the index of the current rpm reading
float total = 0.0;                  // the running total rpm
float averageSpd1 = 0.0;                // the average rpm

float readingsB[numReadingsB];      // the Long rpm array
int readIndexB = 0;              // the index of the current rpm reading
float totalB = 0.0;                  // the running total rpm
float averageSpdLong = 0.0;                // the average rpm

float readings2[numReadings2];      // the tension array
int readIndex2 = 0;              // the index of the current tension reading
float total2 = 0.0;                  // the running total tension
float averageTens = 0.0;                // the average tension

float readings3[numReadings3];      // the winspeed array
int readIndex3 = 0;              // the index of the current windspeed reading
float total3 = 0.0;                  // the running total windspeed
float averageWind = 0.0;                // the average windpseed

#define DEBUG
unsigned long count;   //not needed? Maybe to do with VESC comms
unsigned long count2;  //not needed? Maybe to do with VESC comms

float turnrate;          //erpm converted into approx rpm below
float RAD_Per_Sec;       //and then probably miles off a radpersec value

float brakeCurrent;      //desired braking level at moment

float TSRstepUP = 0.02;   //amount higher we raise brake current per itteration if TSR is too high
float TSRstepDOWN = 0.2;   //amount higher we lower brake current per itteration if TSR is too low

float TTRstepUP = 0.02;   //amount higher we raise brake current per itteration if TTR is too high
float TTRstepDOWN = 0.2;   //amount higher we lower brake current per itteration if TTR is too low


String inputString = "";         // a String to hold incoming wind data on serial 3
bool stringComplete = false;  // whether the wind data string is complete



void setup()
{
  Serial.setTimeout(5);
  
  pinMode(StartButtonPin, INPUT);  // initialize the pushbutton pin as an input: 
  
  Serial.begin(115200); //begin Serial Console
  delay(100);
  Serial2.begin(115200); //begin VESC Serial Console
  delay(200);
  Serial3.begin(115200); //begin wind data Serial port
  delay(100);

  // reserve 200 bytes for the inputString: wind data
  inputString.reserve(200);

  Vesc1.begin(&Serial2);
  Vesc1.UartSetCurrent(0.0);
  Vesc1.UartSetCurrentBrake(0.02);
  delay(50);
 
  scale.tare();  //Reset the scale to 0

    delay(30);

  long zero_factor = scale.read_average(5); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
  
  delay(30);


  scale.set_scale(calibration_factor); //Adjust to this calibration factor

  
  delay(30);
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
                          readings[thisReading] = 0;} //clear rpm smooth array 
   delay(10); 
  
  for (int thisReadingB = 0; thisReadingB < numReadingsB; thisReadingB++) {
                          readingsB[thisReadingB] = 0;} //clear longer rpm smooth array 
   delay(10); 
   
  for (int thisReading2 = 0; thisReading2 < numReadings2; thisReading2++) {
                          readings2[thisReading2] = 0;} //clear tension smooth array   
   delay(10);
  for (int thisReading3 = 0; thisReading3 < numReadings3; thisReading3++) {
                          readings3[thisReading3] = 0;} //clear the windspeed smooth array    
    delay(50);

}

void loop()
{
 
EM_FW_buttonState = digitalRead(EMbuttonPin); // read the state of the emergency forward pushbutton value:      

  // check if the emergency drive forward pushbutton is pressed. If pressed the buttonState is LOW: end of loop routine (long loop = really hard to read program sorry)
if (EM_FW_buttonState == HIGH) 
  { 

  
  StartButtonState = digitalRead(StartButtonPin); // read the state of the Brake at start pushbutton value:
  // check if the StartBrakeCurrent pushbutton is pressed. If it is, the buttonState is HIGH:
 if (StartButtonState == HIGH) 
    {
     Vesc1.UartSetCurrentBrake(StartBrakeCurrent);   //It's startup routine ... brakes high... Not time for spin up & regen yet
     windspeed = Serial3.parseFloat();     //Lets look at wind first Then get load cell readings

    tensionunits = scale.get_units(2), 10; // these roughly = gramms equivalent 2000 ~=2kg pull

    if (tensionunits < 0.0)
    {
      tensionunits = (tensionunits * -1.0);
    }    
              //Serial.print("erpm_VESC1: "); Serial.print(",");
              Serial.print("Startup Brake on ");Serial.print(",");
              Serial.print(" rpm = ");Serial.print(",");
              Serial.print(turnrate);Serial.print(",");
              Serial.print(" Wind_tip_TSR_TSRset ");Serial.print(",");
              Serial.print(windspeed);Serial.print(",");
              Serial.print(tipspeed);Serial.print(",");
              Serial.print(TSRaveraged);Serial.print(",");
              Serial.print(TSRsetpoint);Serial.print(",");
              Serial.print(" Ten-g_genA_TTR_TTRset ");Serial.print(",");
              Serial.print(tensionunits);Serial.print(",");
              Serial.print(StartBrakeCurrent);Serial.print(",");
              Serial.print(TTRactual);Serial.print(",");
              Serial.print(TTRsetpoint);Serial.print(",");
              Serial.print(" Amax ");Serial.print(",");
              Serial.print(maxbrakecurrent);Serial.print(",");
              Serial.print(" SpdQ TTRQ TSRQ ");Serial.print(",");
              Serial.print(SpeedTrendQuota);Serial.print(",");
              Serial.print(TTRQuota);Serial.print(",");
              Serial.println(TSRQuota);Serial.print(",");
            delay (300);
           loop (); 
            }
else
{
    // OK we're out of startup, let's roll with it.   Lets get wind speed data first.  Then lets get load cell readings.
 
    windspeed = Serial3.parseFloat();
 
              // Lets find the average windspeed
               total3 = total3 - readings3[readIndex3];// subtract the last windpseed reading:
                readings3[readIndex3] = windspeed;// read windspeed from this program:
                total3 = total3 + readings3[readIndex3];// add the reading to the total:
                readIndex3 = readIndex3 + 1;// advance to the next position in the array:
              
                if (readIndex3 >= numReadings3) { // if we're at the end of the array...wrap around to the beginning:
                   readIndex3 = 0;             }
                   delay(1);        // delay in between reads for stability
                   
                averageWind = (total3/numReadings3); // calculate a recent average windspeed     
     
    scale.set_scale(calibration_factor); //Adjust to this calibration factor
    tensionunits = scale.get_units(), 10; // these roughly = gramms equivalent 2000 ~=2kg pull
    if (tensionunits < 0.0)
    {
      tensionunits = (tensionunits * -1.0);
    }
 
              // Lets find the average tension
               total2 = total2 - readings2[readIndex2];// subtract the last tension reading:
                readings2[readIndex2] = tensionunits;// read tension from this program:
                total2 = total2 + readings2[readIndex2];// add the reading to the total:
                readIndex2 = readIndex2 + 1;// advance to the next position in the array:
              
                if (readIndex2 >= numReadings2) { // if we're at the end of the array...wrap around to the beginning:
                   readIndex2 = 0;             }
                   delay(1);        // delay in between reads for stability
                   
                averageTens = (total2/numReadings2); // calculate a recent averageSpd1 tension      
    

  //not worrying about including tension calibration now lets get knob set points

          sensorValue1TSR = analogRead(sensorPin1);   // read the value from rotary knob sensor 1:
          TSRsetpoint = (map (sensorValue1TSR, 0,1023,250,550)) / 100.0; //TSR setpoint map limit TSR min to 2.5 top possible setpoint value to ~5.5, 
                                                                       
          delay (2);
          sensorValue2TTR = analogRead(sensorPin2);   // read the value from the sensor 2:
          TTRsetpoint = (map (sensorValue2TTR, 0,1023,10,2000)) / 100.0; // TTR (kg pull / amps braking) want it to be ~ around ~ 0.7 I guess, max here is 20
                                                                       
          delay (2);
          sensorValue7 = analogRead(sensorPin7);   // read the value from the sensor 7: for max current
          maxbrakecurrent = (map (sensorValue7, 0,1023,1,1300)) / 100.0; // dial on A7 limiting max current from 0.1 to 13.0
          delay (2);
        


          
          if (Vesc1.UartGetValue(measuredVESC1))
              {              
              turnrate = (measuredVESC1.rpm / (920000*3.8)); //20khz x 46 poles  (23 pairs) why 3.8? -fudge factor to get the rpm roughly calibrated (odd because gear ratio is 47:22)
 
 
              // Lets find the short average turn rate
               total = total - readings[readIndex];// subtract the last rpm reading:
                readings[readIndex] = turnrate;// read rpm from this program:
                total = total + readings[readIndex];// add the reading to the total:
                readIndex = readIndex + 1;// advance to the next position in the array:
                if (readIndex >= numReadings) { // if we're at the end of the array...wrap around to the beginning:
                   readIndex = 0;             }
                   delay(1);        // delay in between reads for stability
                averageSpd1 = (total/numReadings); // calculate a recent average turnrate 

              // and a longer average turn rate
               totalB = totalB - readingsB[readIndexB];// subtract the last rpm reading:
                readingsB[readIndexB] = turnrate;// read rpm from this program:
                totalB = totalB + readingsB[readIndexB];// add the reading to the total:
                readIndexB = readIndexB + 1;// advance to the next position in the array:
                if (readIndexB >= numReadingsB) { // if we're at the end of the array...wrap around to the beginning:
                   readIndexB = 0;             }
                   delay(1);        // delay in between reads for stability
                averageSpdLong = (totalB/numReadingsB); // calculate a longer recent average turnrate
                                              
              //Tip Speed Ratio
              RAD_Per_Sec = (averageSpd1*0.104719755) ;
              tipspeed = (RAD_Per_Sec * 2.25); //rotor tip outer radius 2.25m 
              TSRaveraged = (tipspeed/averageWind); //tipspeed uses rpm average ... averageWind is windspeed av
              
                
              TTRactual = ((averageTens/1000.0) / brakeCurrent);
              if (TTRactual > TTRMaxValue){
                TTRactual = TTRMaxValue; }
                
  
                      
                      //slow states check
                      if (averageSpd1 < 65.0)
                      {
                           if (averageTens > 50000){    //Worried this could be runaway overtwist because turn rate is low tension is high
                              Vesc1.UartSetCurrentBrake(0.0);  //No braking
                              Vesc1.UartSetCurrent(8.0);    //unwind for a while
                              Serial.println("Emergency forward software");    // WARNING THIS MAY KICKSTART A ROTOR IN READY TO GEN Condition with strong lift tension
                              delay(1200);  
                              brakeCurrent = 0.02;
                              Vesc1.UartSetCurrent(0.0);
                              Vesc1.UartSetCurrentBrake(0.02);  //Start from very low again
                            }
                            else{                        
                      Vesc1.UartSetCurrentBrake(0.02);  //turn rate is too slow now lets not try braking
                      brakeCurrent = 0.02;
                          Serial.print("Too Slow 4 gen");Serial.print(",");
                          Serial.print(" rpm = ");Serial.print(",");
                          Serial.print(turnrate);Serial.print(",");
                          Serial.print(" Wind_tip_TSR_TSRset ");Serial.print(",");
                          Serial.print(windspeed);Serial.print(",");
                          Serial.print(tipspeed);Serial.print(",");
                          Serial.print(TSRaveraged);Serial.print(",");
                          Serial.print(TSRsetpoint);Serial.print(",");
                          Serial.print(" Ten-g_genA_TTR_TTRset ");Serial.print(",");
                          Serial.print(tensionunits);Serial.print(",");
                          Serial.print(brakeCurrent);Serial.print(",");
                          Serial.print(TTRactual);Serial.print(",");
                          Serial.print(TTRsetpoint);Serial.print(",");
                          Serial.print(" Amax ");Serial.print(",");
                          Serial.print(maxbrakecurrent);Serial.print(",");
                          Serial.print(" SpdQ TTRQ TSRQ ");Serial.print(",");
                          Serial.print(SpeedTrendQuota);Serial.print(",");
                          Serial.print(TTRQuota);Serial.print(",");
                          Serial.println(TSRQuota);Serial.print(",");
                          delay(1);  
                            }
                          }
              else
              //fast enough              
              { 
                                          
                      if (averageTens < 1800){  //a 1.8kg minimum pull on the line (because readings are gippy but the linkage is gippy at best)
                          brakeCurrent = 0.1;
                          Vesc1.UartSetCurrentBrake(brakeCurrent);
                          Serial.print("Weak lift tension");Serial.print(",");
                          Serial.print(" rpm = ");Serial.print(",");
                          Serial.print(turnrate);Serial.print(",");
                          Serial.print(" Wind_tip_TSR_TSRset ");Serial.print(",");
                          Serial.print(windspeed);Serial.print(",");
                          Serial.print(tipspeed);Serial.print(",");
                          Serial.print(TSRaveraged);Serial.print(",");
                          Serial.print(TSRsetpoint);Serial.print(",");
                          Serial.print(" Ten-g_genA_TTR_TTRset ");Serial.print(",");
                          Serial.print(tensionunits);Serial.print(",");
                          Serial.print(brakeCurrent);Serial.print(",");
                          Serial.print(TTRactual);Serial.print(",");
                          Serial.print(TTRsetpoint);Serial.print(",");
                          Serial.print(" Amax ");Serial.print(",");
                          Serial.print(maxbrakecurrent);Serial.print(",");
                          Serial.print(" SpdQ TTRQ TSRQ ");Serial.print(",");
                          Serial.print(SpeedTrendQuota);Serial.print(",");
                          Serial.print(TTRQuota);Serial.print(",");
                          Serial.println(TSRQuota);Serial.print(",");
                        }



                      
                      if (averageTens > 1500){
                        //TSR first
                        if (TSRaveraged > (TSRsetpoint * 1.1)){
                          TSRQuota = (TSRQuota + 0.01);}
                        if (TSRaveraged < (TSRsetpoint * 0.85)){
                          TSRQuota = (TSRQuota - 0.02);}
                          if (TSRQuota > 1.0){
                            TSRQuota = 1.0;}
                          if (TSRQuota <0.1){
                            TSRQuota = 0.1;}


                          
                          //TTR next
                        if (TTRactual > (TTRsetpoint * 1.1)){
                          TTRQuota = (TTRQuota + 0.01);}
                        if (TTRactual < (TTRsetpoint * 0.85)){
                         TTRQuota = (TTRQuota - 0.04);}
                          if (TTRQuota > 1.0){
                            TTRQuota = 1.0;}
                          if (TTRQuota <0.1){
                            TTRQuota = 0.1;}


                        if (averageSpd1 > (averageSpdLong * 1.1)){
                          SpeedTrendQuota = (SpeedTrendQuota + 0.01);}
                        if (averageSpd1 < (averageSpdLong * 0.85)){
                          SpeedTrendQuota = (SpeedTrendQuota - 0.04);}
                          if (SpeedTrendQuota > 1.0){
                            SpeedTrendQuota = 1.0;}
                          if (SpeedTrendQuota <0.1){
                            SpeedTrendQuota = 0.1;}


                            


 brakeCurrent =    maxbrakecurrent * ((SpeedTrendQuota*0.3)+(TTRQuota*0.2)+(TSRQuota*0.5));                  //ammount contribution of max brake allowed from each component .3x spd trend + .2x TTR + .5x TSR



                       if (brakeCurrent > maxbrakecurrent){
                         brakeCurrent = maxbrakecurrent;}
                       if (brakeCurrent < 0.15){
                         brakeCurrent = 0.15;}
                         

                       
                    Vesc1.UartSetCurrentBrake(brakeCurrent);
                          
                          Serial.print("OK to Gen");Serial.print(",");
                          Serial.print(" rpm = ");Serial.print(",");
                          Serial.print(turnrate);Serial.print(",");
                          Serial.print(" Wind_tip_TSR_TSRset ");Serial.print(",");
                          Serial.print(windspeed);Serial.print(",");
                          Serial.print(tipspeed);Serial.print(",");
                          Serial.print(TSRaveraged);Serial.print(",");
                          Serial.print(TSRsetpoint);Serial.print(",");
                          Serial.print(" Ten-g_genA_TTR_TTRset ");Serial.print(",");
                          Serial.print(tensionunits);Serial.print(",");
                          Serial.print(brakeCurrent);Serial.print(",");
                          Serial.print(TTRactual);Serial.print(",");
                          Serial.print(TTRsetpoint);Serial.print(",");
                          Serial.print(" Amax ");Serial.print(",");
                          Serial.print(maxbrakecurrent);Serial.print(",");
                          Serial.print(" SpdQ TTRQ TSRQ ");Serial.print(",");
                          Serial.print(SpeedTrendQuota);Serial.print(",");
                          Serial.print(TTRQuota);Serial.print(",");
                          Serial.println(TSRQuota);Serial.print(",");
                            }  
                           }                    
               }
              else
              {
              Serial.print("No data from VESC1!");Serial.print(",");Serial.print(",");Serial.print(",");Serial.print(",");
              Serial.print("windspeed ");Serial.print(",");
              Serial.print(windspeed);;Serial.print(",");Serial.print(",");Serial.print(",");Serial.print(",");
              Serial.print("Tension ");Serial.print(",");
              Serial.println(tensionunits);
              }             
          } 
    }          
                
 else
                 {
    Vesc1.UartSetCurrentBrake(0.0); // drive forward button pressed
    brakeCurrent = 0.0;
    Vesc1.UartSetCurrent(16.0);
    TTRQuota = 0.1;
    SpeedTrendQuota = 0.1;
    TSRQuota = 0.1;
    Serial.println("Emergency forward button pressed");
    delay(150); 
              }
}
