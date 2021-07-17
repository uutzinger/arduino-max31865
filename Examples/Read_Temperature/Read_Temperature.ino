/**************************************************************************
 * MAX31865 Basic Example
 *
 * Copyright (C) 2015 Ole Wolf <wolf@blazingangles.com>
 * PTD100 LUT Copyright (c) 2017, drhaney
 *
 * Example code that reads the temperature from an MAX31865 and outputs
 * it on the serial line.
 * 
 * Wire the circuit as follows, assuming that level converters have been
 * added for the 3.3V signals:
 *
 *    Arduino Uno   -->  MAX31865
 *    Teensy       
 *    ---------------------------
 *    CS:   pin 10  -->  CS
 *    MOSI: pin 11  -->  SDI (must not be changed for hardware SPI)
 *    MISO: pin 12  -->  SDO (must not be changed for hardware SPI)
 *    SCK:  pin 13  -->  SCLK (must not be changed for hardware SPI)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Modifications Urs Utzinger 2021:
 * Rewrote SPI handling
 * Added functions to set configuration bits and clear faults
 * Added function to read only rtd register
 * Incorporated LUT temperature conversion from Daniel R. Haney
 **************************************************************************/

#include <SPI.h>
#include <MAX31865.h>

#define RTD_CS_PIN   10

#define TEMP_INTERVAL        1000000    // 1 second between tempreature readings
#define LEDON_INTERVAL        100000    // interval in microseconds between turning LED on/off for status report
#define LEDOFF_INTERVAL       900000    // interval in microseconds between turning LED on/off for status report
unsigned long currentTime;              //
unsigned long nextLEDCheck;             //
unsigned long nextTempRead;             //
const int ledPin = 2;                   // Check on https://www.pjrc.com/teensy/pinout.html; pin can not be on pins needed for the SPI sensor
bool ledStatus = false;                 // Led should be off at start up

MAX31865_RTD rtd( MAX31865_RTD::RTD_PT100, RTD_CS_PIN );

void setup()
{

  pinMode(ledPin, OUTPUT);    // sets the led pin to output

  Serial.begin( 115200 );

  /* Initialize SPI communication. */
  SPI.begin( );

  /* Allow the MAX31865 to warm up. */
  delay( 100 );

  /* Configure:
   * Single Shot: turn on V_BIAS before reading then turn off to reduce power dissipation 
   * Conversion Mode: If true, continuouse conversion at 50 or 60Hz 
   * If auto conversion is true, V_BIAS needs to be on all the time 
   * If auto conversion is off, start conversion by setting 1-shot to 1 and trigger measurement with CS going high 
   * If V_BIAS is off it takes 10.5 * time constants to charge input RC network (about 10ms)
   */

  // turn off vbias
  // no auto convert
  // set wire
  rtd.configure_control( 
    false,     // V_BIAS enable
    false,     // auto conversion
    false,     // 1-shot, start conversion when CS goes high 
    false,     // 3-wire enable
    MAX31865_FAULT_DETECTION_NONE, // fault detection 
    false,      // fault status auto clear
    false);     // true = 50Hz filter, false = 60Hz

  // clear faults
  rtd.clearFaults(); 

  /* Set Threshold:
   * Low Thershold and High Threshold
   */

  rtd.configure_thresholds(
    0x0000,    // Low Thresh 0x0000
    0xffff );  // High Thresh 0xffff
    // (uint16_t)((deg_C + 256.) * 32.) << 1

  nextLEDCheck = micros();
  nextTempRead = micros();
}

void loop() 
{
  // Time Keeper
  currentTime = micros();

  if (currentTime > nextTempRead) {
    nextTempRead = currentTime + TEMP_INTERVAL;
  
    rtd.enableBias(true);  // enable V_BIAS
    delay(1);              // wait until RC network has setteled, no difference if set to 0 or 50
    rtd.oneShot();         // trigger measurement
    delay(65);             // wait until conversion is complete, 65ms from spec sheet but can go down to 45

    //Read the MAX31865 registers in the following order:  Configuration, RTD, High Fault, Low Fault, Fault Status
    uint8_t status = rtd.read_all( );

    rtd.enableBias(false);  // disable V_BIAS

    // Report registers and data
    Serial.print( " T = ");                            Serial.print(   rtd.temperature(), 2);                               Serial.println(" deg C" );
    Serial.print( " T_prec = ");                       Serial.print(   rtd.ohmsX100_to_celsius(rtd.resistance()*100.), 2 ); Serial.println(" deg C" );
    Serial.print( " R = ");                            Serial.print(   rtd.resistance(), 1 );                               Serial.println(" Ohms" );
    Serial.print(" Low  Threshold = ");                Serial.println( rtd.low_threshold(),HEX );
    Serial.print(" High Threshold = ");                Serial.println( rtd.high_threshold(),HEX ); 
    Serial.print(" Raw Resistance = ");                Serial.println( rtd.raw_resistance(),HEX ); 
    Serial.print(" RTD fault register: " );            Serial.print( status );  
    Serial.print( ": " );
    if( status == 0)                                 { Serial.println( "No faults" ); }
    else if( status & MAX31865_FAULT_HIGH_THRESHOLD ){ Serial.println( "RTD high threshold exceeded" ); }
    else if( status & MAX31865_FAULT_LOW_THRESHOLD ) { Serial.println( "RTD low threshold exceeded" ); }
    else if( status & MAX31865_FAULT_REFIN )         { Serial.println( "REFIN- > 0.85 x V_BIAS" ); }
    else if( status & MAX31865_FAULT_REFIN_FORCE )   { Serial.println( "REFIN- < 0.85 x V_BIAS, FORCE- open" ); }
    else if( status & MAX31865_FAULT_RTDIN_FORCE )   { Serial.println( "RTDIN- < 0.85 x V_BIAS, FORCE- open" ); }
    else if( status & MAX31865_FAULT_VOLTAGE )       { Serial.println( "Overvoltage/undervoltage fault" ); }
    else                                             { Serial.println( "Unknown fault; check connection" ); }

    // clear faults
    if (status != 0) { rtd.clearFaults(); }
    
  }
  
  // Blink LED
  if (currentTime > nextLEDCheck) {
    if (ledStatus) {
      // LED is ON
      ledStatus = false; 
      digitalWriteFast(ledPin, ledStatus);                        // turn off
      nextLEDCheck = currentTime + LEDOFF_INTERVAL;
    } else {
      // LED is OFF
      ledStatus = true;
      digitalWriteFast(ledPin, ledStatus);                        // turn on
      nextLEDCheck = currentTime + LEDON_INTERVAL;
    }
  }

}
