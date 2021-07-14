/**************************************************************************
 * MAX31865 Basic Example
 *
 * Copyright (C) 2015 Ole Wolf <wolf@blazingangles.com>
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
   * Single Shot turn on V_BIAS before reading then turn off to reduce power dissipation 
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

}

void loop() 
{

  // clear faults
  rtd.clearFaults(); // might not be needed

  // enable V_BIAS
  rtd.enableBias(true);
  
  // wait until RC network has setteled
  delay(10);
  
  // trigger measurement
  rtd.oneShot();

  // wait until conversion is complete
  delay(65);
  
  //Read the MAX31865 registers in the following order:
  //     Configuration register
  //     RTD register
  //     High Fault Threshold register 
  //     Low Fault Threshold register
  //     Fault Status register
  uint8_t status = rtd.read_all( );

  // might want to read only RTD and fault registers
  // uint8_t status = rtd.read_rtd( );

  // might want to read only RTD
  // if (rtd.read_rtd( )) {} // dont care about fault register

  // disable V_BIAS
  rtd.enableBias(false);

  // Report registers and data
  double temperature = rtd.temperature( );
  Serial.print( " T = ");
  Serial.print( temperature, 1 );
  Serial.println(" deg C" );
  double resistance = rtd.resistance( );
  Serial.print( " R = ");
  Serial.print( resistance, 1 );
  Serial.println(" Ohms" );
  uint16_t low_thresh = rtd.low_threshold();
  Serial.print(" Low Threshold = "); Serial.println(low_thresh,HEX);
  uint16_t high_thresh = rtd.high_threshold();
  Serial.print(" Low Threshold = "); Serial.println(high_thresh,HEX);
  uint16_t raw_resistance = rtd.raw_resistance();
  Serial.print(" Raw Resistance = "); Serial.println(raw_resistance,HEX);
  Serial.print( "RTD fault register: " );
  Serial.print( status );
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
  if (status != 0) {
    rtd.clearFaults();
  }
  
  delay( 1000 );
}
