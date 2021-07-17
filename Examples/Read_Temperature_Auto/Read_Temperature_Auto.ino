/**************************************************************************
 * MAX31865 Example for Autoconversion
 *
 * Copyright (C) 2015 Ole Wolf <wolf@blazingangles.com>
 * PTD100 LUT Copyright (c) 2017, drhaney
 * 
 * Observe data in serial plotter
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
 * Added fucntions to set configuration bits and clear faults
 * Added function to read only rtd register
 * Incorporated LUT temperature conversion from Daniel R. Haney
 **************************************************************************/

#include <SPI.h>
#include <MAX31865.h>

#define RTD_CS_PIN   10

MAX31865_RTD rtd( MAX31865_RTD::RTD_PT100, RTD_CS_PIN );

void setup()
{
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

  // turn on vbias
  // auto convert
  // set wire
  rtd.configure_control( 
    true,     // V_BIAS enable
    true,     // auto conversion
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

}

void loop() 
{
  //Read the MAX31865 rtd register
  bool status = rtd.read_rtd( );

  // Report data
  // View in Serial Plotter
  Serial.print( " T = ");  Serial.print(   rtd.ohmsX100_to_celsius(rtd.resistance()*100.), 2 ); Serial.println(" deg C" );

  // clear faults
  if (status == true) { 
    Serial.println(" RTD fault occured " );  
    rtd.clearFaults(); 
}
  
  delay( 17 );  // minimum of 2-17ms for 60Hz, 2-20ms for 50Hz
}
