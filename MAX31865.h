/**************************************************************************
 * Arduino driver library for the MAX31865.
 *
 * Libary Copyright (C) 2015 Ole Wolf <wolf@blazingangles.com>
 * PTD100 LUT Copyright (c) 2017, drhaney
 *
 * Wire the circuit as follows, assuming that level converters have been
 * added for the 3.3V signals:
 *
 *    Arduino Uno            -->  MAX31865
 *    ------------------------------------
 *    CS: any available pin  -->  CS
 *    MOSI: pin 11           -->  SDI  (mandatory for hardware SPI)
 *    MISO: pin 12           -->  SDO  (mandatory for hardware SPI)
 *    SCK:  pin 13           -->  SCLK (mandatory for hardware SPI)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
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

#ifndef _MAX31865_H
#define _MAX31865_H

#include <stdint.h>

#define MAX31865_FAULT_HIGH_THRESHOLD  ( 1 << 7 )
#define MAX31865_FAULT_LOW_THRESHOLD   ( 1 << 6 )
#define MAX31865_FAULT_REFIN           ( 1 << 5 )
#define MAX31865_FAULT_REFIN_FORCE     ( 1 << 4 )
#define MAX31865_FAULT_RTDIN_FORCE     ( 1 << 3 )
#define MAX31865_FAULT_VOLTAGE         ( 1 << 2 )

#define MAX31865_FAULT_DETECTION_NONE      ( 0x00 << 2 )
#define MAX31865_FAULT_DETECTION_AUTO      ( 0x01 << 2 )
#define MAX31865_FAULT_DETECTION_MANUAL_1  ( 0x02 << 2 )
#define MAX31865_FAULT_DETECTION_MANUAL_2  ( 0x03 << 2 )

/* RTD data, RTD current, and measurement reference
   voltage. The ITS-90 standard is used; other RTDs
   may have coefficients defined by the DIN 43760 or
   the U.S. Industrial (American) standard. */

#define RTD_A_MAXIM         3.9083e-3
#define RTD_B_MAXIM        -5.775e-7
#define RTD_C_MAXIM        -4.18301e-12
#define RTD_A_ITS90         3.9080e-3
#define RTD_B_ITS90        -5.870e-7
#define RTD_A_USINDUSTRIAL  3.9692e-3
#define RTD_B_USINDUSTRIAL -5.8495e-7
#define RTD_A_DIN43760      3.9848e-3
#define RTD_B_DIN43760     -5.8019e-7

/* RTD coefficient C is required only for temperatures
   below 0 deg. C.  The selected RTD coefficient set
   is specified below. */

#define SELECT_RTD_HELPER(x) x
#define SELECT_RTD(x) SELECT_RTD_HELPER(x)
#define RTD_A         SELECT_RTD(RTD_A_MAXIM)
#define RTD_B         SELECT_RTD(RTD_B_MAXIM)
#define RTD_C         SELECT_RTD(RTD_C_MAXIM)

/*
 * The reference resistor on the hardware; see the MAX31865 datasheet
 * for details.  The values 400 and 4000 Ohm are recommended values for
 * the PT100 and PT1000.
 */
#define RTD_RREF_PT100         430 /* Ohm Adafruit:  430, Maxim:  400 */ 
#define RTD_RREF_PT1000       4300 /* Ohm Adafruit: 4300, Maxim: 4000 */

/*
 * The RTD resistance at 0 degrees Celcius.  For the PT100, this is 100 Ohm;
 * for the PT1000, it is 1000 Ohm.
 */
#define RTD_RESISTANCE_PT100   100 /* Ohm */
#define RTD_RESISTANCE_PT1000 1000 /* Ohm */

#define RTD_ADC_RESOLUTION  ( 1u << 15 ) /* 15 bits or 32768*/

/* See the main (MAX31865.cpp) file for documentation of the class methods. */
class MAX31865_RTD
{
public:
  enum ptd_type { RTD_PT100, RTD_PT1000 };

  MAX31865_RTD( ptd_type type, uint8_t cs_pin );
  void configure_all        ( bool v_bias, bool conversion_mode, bool one_shot, bool three_wire,
                              uint8_t fault_cycle, bool fault_clear, bool filter_50hz,
                              uint16_t low_threshold, uint16_t high_threshold );
  void configure_control    ( bool v_bias, bool conversion_mode, bool one_shot, bool three_wire,
                              uint8_t fault_cycle, bool fault_clear, bool filter_50hz);
  void configure_thresholds ( uint16_t low_threshold, uint16_t high_threshold );
  uint8_t           read_all( ); // sends 9 bytes to sensor
  uint8_t     read_rtd_fault( ); // sends 5 bytes to sensor
  bool              read_rtd( ); // sends 3 bytes to sensor
  void            enableBias( bool bias ); // turns bias on or off
  void           clearFaults( void ); // clears the fault status
  void               oneShot( void ); // set one shot
  double         temperature( ) const;
  float ohmsX100_to_celsius ( uint16_t ohmsX100 );
  uint8_t             status( ) const { return( measured_status ); }
  uint16_t     low_threshold( ) const { return( measured_low_threshold ); }
  uint16_t    high_threshold( ) const { return( measured_high_threshold ); }
  uint16_t    raw_resistance( ) const { return( measured_resistance ); }
  double          resistance( ) const
  {
    const double rtd_rref =
      ( this->type == RTD_PT100 ) ? (double)RTD_RREF_PT100 : (double)RTD_RREF_PT1000;
    return( (double)raw_resistance( ) * rtd_rref / (double)RTD_ADC_RESOLUTION );
  }

private:
  /* Our configuration. */
  uint8_t  cs_pin;
  ptd_type type;
  uint8_t  configuration_control_bits;
  uint16_t configuration_low_threshold;
  uint16_t configuration_high_threshold;
  void reconfigure_settings( );
  void reconfigure_thresholds( );
  int find_index(uint16_t ohmsX100);

  /* Values read from the device. */
  uint8_t  measured_configuration;
  uint16_t measured_resistance;
  uint16_t measured_high_threshold;
  uint16_t measured_low_threshold;
  uint8_t  measured_status;
};

#endif /* _MAX31865_H */
