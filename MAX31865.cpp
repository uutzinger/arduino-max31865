/**************************************************************************
 * Arduino driver library for the MAX31865.
 *
 * Copyright (C) 2015 Ole Wolf <wolf@blazingangles.com>
 *
 *
 * Wire the circuit as follows, assuming that level converters have been
 * added for the 3.3V signals:
 *
 *    Arduino Uno            -->  MAX31865
 *    ------------------------------------
 *    CS: any available pin  -->  CS
 *    MOSI: pin 11           -->  SDI (mandatory for hardware SPI)
 *    MISO: pin 12           -->  SDO (mandatory for hardware SPI)
 *    SCK: pin 13            -->  SCLK (mandatory for hardware SPI)
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


/**
 * The constructor for the MAX31865_RTD class registers the CS pin and
 * configures it as an output.
 *
 * @param [in] cs_pin Arduino pin selected for the CS signal.
 */
MAX31865_RTD::MAX31865_RTD( ptd_type type, uint8_t cs_pin )
{
  /* Set the type of PTD. */
  this->type = type;

  /* CS pin for the SPI device. */
  this->cs_pin = cs_pin;
  pinMode( this->cs_pin, OUTPUT );

  /* Pull the CS pin high to avoid conflicts on SPI bus. */
  digitalWrite( this->cs_pin, HIGH );
}



/**
 * Configure the MAX31865.  The parameters correspond to Table 2 in the MAX31865
 * datasheet.  The parameters are combined into a control bit-field that is stored
 * internally in the class for later reconfiguration, as are the fault threshold values.
 *
 * @param [in] v_bias Vbias enabled (@a true) or disabled (@a false).
 * @param [in] conversion_mode Conversion mode auto (@a true) or off (@a false).
 * @param [in] one_shot 1-shot measurement enabled (@a true) or disabled (@a false).
 * @param [in] three_wire 3-wire enabled (@a true) or 2-wire/4-wire (@a false).
 * @param [in] fault_detection Fault detection cycle control (see Table 3 in the MAX31865
 *             datasheet).
 * @param [in] fault_clear Fault status auto-clear (@a true) or manual clear (@a false).
 * @param [in] filter_50hz 50 Hz filter enabled (@a true) or 60 Hz filter enabled
 *             (@a false).
 * @param [in] low_threshold Low fault threshold.
 * @param [in] high_threshold High fault threshold.
*/

/*
Configuration register
======================
read 00h write 80

[D7, D6, D5, D4, D3, D2, D1, D0]
D7 VBIAS 1=On 0=Off 
D6 Conversion Mode 1=Auto 0=Normally Off 
D5 1-Shot 1=1-shot (then auto clear) 
D4 3-wire, 1=3-wire, 0=2 or 4 wire rtd sensor
D3, D2:
XXXX00XXb write: no Action                                 read: fault detection finished
100X010Xb write: fault detection with automatic delay,     read: automatic fault detection still running
100X100Xb write: run fault detection with manula delay,    read: manual cycle 1 still running, waiting for user to write 11
100X110Xb write: finish fault detection with manual delay, read: manucal cycle 2 still running
D1 Fault Status Clear, 1=clear (then auto clear)
D0 50/60Hz filter, 1=50Hz, 0=60Hz

High Fault Threshold
====================
MSB read 03h write 83h
LSB read 05h write 84h
[MSB, D6, D5, D4, D3, D2, D1, D0][D7, D6, D5, D4, D3, D2, LSB, X]

Low Fault Threshold
===================
read 05h write 85h
read 06h write 86h

Fault Status
============
[D7, D6, D5, D4, D3, D2, D1, D0]
D7 High Threshold
D6 Low Threhsold
D5 REFIN > 0.85VBIAS
D4 FORCE OPEN
D3 FORCE CLOSE
D2 Undervoltage fault
D1 dont care
D0 dont care

Data Registers
==============
[D7 D6 D5 D4 D3 D2 D1 D0][D7 D6 D5 D4 D3 D2 D1  D0]
MSB -  -  -  -  -  -  -   -  -  -  -  -  -  LSB Fault (any)
*/


void MAX31865_RTD::configure_all ( bool v_bias, bool conversion_mode, bool one_shot,
                                   bool three_wire, uint8_t fault_cycle, bool fault_clear,
                                   bool filter_50hz, uint16_t low_threshold,
                                   uint16_t high_threshold )
{
  uint8_t control_bits = 0;

  /* Assemble the control bit mask. */
  control_bits |= (          v_bias ? 0x80 : 0 );
  control_bits |= ( conversion_mode ? 0x40 : 0 );
  control_bits |= (        one_shot ? 0x20 : 0 );
  control_bits |= (      three_wire ? 0x10 : 0 );
  control_bits |=       fault_cycle & 0b00001100;
  control_bits |= (     fault_clear ? 0x02 : 0 );
  control_bits |= (     filter_50hz ? 0x01 : 0 );

  /* Store the control bits and the fault threshold limits for reconfiguration
     purposes. */
  this->configuration_control_bits   = control_bits;
  this->configuration_low_threshold  = low_threshold;
  this->configuration_high_threshold = high_threshold;

  /* Perform an initial "reconfiguration." */
  reconfigure_settings();
  reconfigure_thresholds();
}

void MAX31865_RTD::configure_thresholds ( uint16_t low_threshold, uint16_t high_threshold )
{
  this->configuration_low_threshold  = low_threshold;
  this->configuration_high_threshold = high_threshold;

  /* Perform an initial "reconfiguration." */  reconfigure_settings();
  reconfigure_thresholds();
}

void MAX31865_RTD::reconfigure_thresholds( )
{
  /* Write the threshold values. */
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite( this->cs_pin, LOW );
  SPI.transfer( 0x83 );
  SPI.transfer( ( this->configuration_high_threshold >> 8 ) & 0x00ff );
  SPI.transfer(   this->configuration_high_threshold        & 0x00ff );
  SPI.transfer( ( this->configuration_low_threshold >> 8 )  & 0x00ff );
  SPI.transfer(   this->configuration_low_threshold         & 0x00ff );
  digitalWrite( this->cs_pin, HIGH );
  SPI.endTransaction();
}

void MAX31865_RTD::configure_control ( bool v_bias, bool conversion_mode, bool one_shot,
                               bool three_wire, uint8_t fault_cycle, bool fault_clear,
                               bool filter_50hz)
{
  uint8_t control_bits = 0;

  /* Assemble the control bit mask. */
  control_bits |= (          v_bias ? 0x80 : 0 );
  control_bits |= ( conversion_mode ? 0x40 : 0 );
  control_bits |= (        one_shot ? 0x20 : 0 );
  control_bits |= (      three_wire ? 0x10 : 0 );
  control_bits |=       fault_cycle & 0b00001100;
  control_bits |= (     fault_clear ? 0x02 : 0 );
  control_bits |= (     filter_50hz ? 0x01 : 0 );

  /* Store the control bits and the fault threshold limits for reconfiguration
     purposes. */
  this->configuration_control_bits   = control_bits;

  /* Perform an initial "reconfiguration." */
  reconfigure_settings( );
}

void MAX31865_RTD::reconfigure_settings( )
{
  /* Write the configuration to the MAX31865. */
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite( this->cs_pin, LOW );
  SPI.transfer( 0x80 );
  SPI.transfer( this->configuration_control_bits );
  digitalWrite( this->cs_pin, HIGH );
  SPI.endTransaction();
}

/**
 * Read all settings and measurements from the MAX31865 and store them
 * internally in the class.
 *
 * @return Fault status byte
 *
*/

uint8_t MAX31865_RTD::read_all( )
{
  uint16_t combined_bytes;

  /* Start the read operation. */
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite( this->cs_pin, LOW );

  /* Read the MAX31865 registers in the following order:
       Configuration
       RTD
       High Fault Threshold
       Low Fault Threshold
       Fault Status */

  /* Tell the MAX31865 that we want to read, starting at register 0. */
  SPI.transfer( 0x00 );

  //00h
  this->measured_configuration = SPI.transfer( 0x00 );

  //01h
  combined_bytes  = SPI.transfer( 0x00 ) << 8;
  combined_bytes |= SPI.transfer( 0x00 );
  this->measured_resistance = combined_bytes >> 1;

  // 03h
  combined_bytes  = SPI.transfer( 0x00 ) << 8;
  combined_bytes |= SPI.transfer( 0x00 );
  this->measured_high_threshold = combined_bytes >> 1;
  
  // 05h
  combined_bytes  = SPI.transfer( 0x00 ) << 8;
  combined_bytes |= SPI.transfer( 0x00 );
  this->measured_low_threshold = combined_bytes >> 1;

  // 07h
  this->measured_status = SPI.transfer( 0x00 );

  digitalWrite( this->cs_pin, HIGH );
  SPI.endTransaction();

  return( status( ) );
}


/**
 * Apply the Callendar-Van Dusen equation to convert the RTD resistance
 * to temperature:
 *
 * for T>=0
 *  T(r) = (Z1 + sqrt(Z2 + Z3*r)) / Z4
 *  with Z1 = -A
 *  with Z2 = A^2 -4*B
 *  with Z3 = 4*B/R0
 *  with Z4 = 2*B
 *
 * A and B are the RTD coeffients
 *
 * for T < 0
 *  T(r) = –242.02 2.2228*r + 2.5859e-3*r^2 – 4.8260r^3 – 2.8183e-8*r^4 +1.5243e-10*r^5
 *
 * For more information on measuring with an RTD, see:
 * https://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
 *
 * This should result in less than 0.01 deg C error
 *
 * @param [in] resistance The measured RTD resistance.
 * @return Temperature in degrees Celcius.
 */
double MAX31865_RTD::temperature( ) const
{
  
  double Rt = double(resistance());

  double Z1, Z2, Z3, Z4, temp, rpoly;

  const double rtdNominal = ( this->type == RTD_PT100 ) ? RTD_RESISTANCE_PT100 : RTD_RESISTANCE_PT1000;

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / rtdNominal; 
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0) return temp;

  Rt /= rtdNominal;
  Rt *= 100; // normalize to 100 ohm

  rpoly  = Rt; // linear
  temp   = -242.02;
  temp  +=  2.2228 * rpoly;
  rpoly *= Rt; // square
  temp  +=  2.5859e-3 * rpoly ;
  rpoly *= Rt; // ^3
  temp  -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp  -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp  += 1.5243e-10 * rpoly;

  return temp;

}
