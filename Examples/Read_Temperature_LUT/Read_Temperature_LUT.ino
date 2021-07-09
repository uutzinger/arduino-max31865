/**************************************************************************
 * MAX31865 Basic Example
 *
 * Copyright (C) 2015 Ole Wolf <wolf@blazingangles.com>
 *
 *
 * Example code that reads the temperature from an MAX31865 and outputs
 * it on the serial line.
 * 
 * Wire the circuit as follows, assuming that level converters have been
 * added for the 3.3V signals:
 *
 *    Arduino Uno   -->  MAX31865
 *    ---------------------------
 *    CS: pin 10    -->  CS
 *    MOSI: pin 11  -->  SDI (must not be changed for hardware SPI)
 *    MISO: pin 12  -->  SDO (must not be changed for hardware SPI)
 *    SCK: pin 13   -->  SCLK (must not be changed for hardware SPI)
 *
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

// You need https://github.com/drhaney/pt100rtd library for this
#include <pt100rtd.h>
pt100rtd PT100 = pt100rtd() ;

void setup()
{
  Serial.begin( 115200 );

  /* Initialize SPI communication. */
  SPI.begin( );

  /* Allow the MAX31865 to warm up. */
  delay( 100 );

  /* Configure:  */
  /* Single Shot turn on V_BIAS before reading then turn off to reduce power dissipation */
  /* Conversion Mode: If true, continouse conversion 50/60Hz
  /* If auto conversion is true, continous conversion, V_BIAS needs to be on all the time */
  /* If auto conversion is off, start conversion by setting 1-shot and trigger measurement with CS */
  /* 1-Shot starts conversion when CS goes high */
  /* If V_BIAS is off it takes 10.5 time constants to charge input RC network (Adafruit waits 10ms)*/

  rtd.configure_all( false, // V_BIAS enable
                 false,     // auto conversion
                 false,     // 1-shot, start conversion when CS goes high 
                 false,     // 3-wire enable
                 MAX31865_FAULT_DETECTION_NONE, // fault detection automatic delay
                 true,      // fault status auto clear
                 false,     // true = 50Hz filter, false = 60Hz
                 0x0000,    // Low Thresh 0x0000
                 0x7fff );  // High Thresh 0x7fff
}



void loop() 
{

  // enable V_BIAS 
  rtd.configure( true,     // V_BIAS enable
                 false,    // auto conversion
                 false,    // 1-shot, start conversion when CS goes high 
                 false,    // 3-wire enable
                 MAX31865_FAULT_DETECTION_NONE, // fault detection automatic delay
                 true,     // fault status auto clear
                 false);   // true = 50Hz filter, false = 60Hz
  // wait until RC network has setteled
  delay(10);
  // enable enable one shot, need to keep other settings same
  rtd.configure( true,     // V_BIAS enable
                 false,    // auto conversion
                 true,     // 1-shot, start conversion when CS goes high 
                 false,    // 3-wire enable
                 MAX31865_FAULT_DETECTION_NONE, // fault detection automatic delay
                 true,     // fault status auto clear
                 false);   // true = 50Hz filter, false = 60Hz

  uint8_t status = rtd.read_all( );

  // disable V_BIAS
  rtd.configure( false,    // V_BIAS enable
                 false,    // auto conversion
                 false,    // 1-shot, start conversion when CS goes high 
                 false,    // 3-wire enable
                 MAX31865_FAULT_DETECTION_NONE, // fault detection automatic delay
                 true,     // fault status auto clear
                 false);   // true = 50Hz filter, false = 60Hz

  if( status == 0 )
  {
    double temperature = rtd.temperature( );
    Serial.print( " T = ");
    Serial.print( temperature, 1 );
    Serial.println(" deg C" );

    double resistance = rtd.resistance( );
    Serial.print( " R = ");
    Serial.print( resistance, 1 );
    Serial.println(" Ohms" );

    // Need to check this
	//dummy = ((uint32_t)(rtd << 1)) * 100 * ((uint32_t) floor(RREF)) ;
	//dummy >>= 16 ;
	//ohmsx100 = (uint16_t) (dummy & 0xFFFF) ;
    //double ohmsx100 = rtd.raw_resistance() * 100.0;
    double ohmsx100 = rtd.resistance() * 100.0;
    Serial.print( " T precise = ");
    Serial.print( PT100.celsius(uint16_t(ohmsx100)), 1 );
    Serial.println(" deg C" );

  }
  else 
  {
    Serial.print( "RTD fault register: " );
    Serial.print( status );
    Serial.print( ": " );
    if( status & MAX31865_FAULT_HIGH_THRESHOLD )
    {
      Serial.println( "RTD high threshold exceeded" );
    }
    else if( status & MAX31865_FAULT_LOW_THRESHOLD )
    {
      Serial.println( "RTD low threshold exceeded" );
    }
    else if( status & MAX31865_FAULT_REFIN )
    {
      Serial.println( "REFIN- > 0.85 x V_BIAS" );
    }
    else if( status & MAX31865_FAULT_REFIN_FORCE )
    {
      Serial.println( "REFIN- < 0.85 x V_BIAS, FORCE- open" );
    }
    else if( status & MAX31865_FAULT_RTDIN_FORCE )
    {
      Serial.println( "RTDIN- < 0.85 x V_BIAS, FORCE- open" );
    }
    else if( status & MAX31865_FAULT_VOLTAGE )
    {
      Serial.println( "Overvoltage/undervoltage fault");
    }
    else
    {
      Serial.println( "Unknown fault; check connection" );
    }
  }
  
  delay( 1000 );
}