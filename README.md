
# MAXIM 31865 Driver

The files in this folder provide an Arduino driver for the **MAX31865** RTD chip
and example code that illustrates how to use it.  Move the folder contents
to your Arduino/libraries folder.

This library was updated to work on the **Teensy** (tested on 4.0) and to use current SPI setup routines.
The Lookup Table temperature conversion from from [drhaney](https://github.com/drhaney/pt100rtd) has been included.

It has been tested with [Adafruit Breakout Board](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier)

The PT-100 sensor is rated from -200C to 550C.

There are two examples:

- Read_Temperature.ino which is an example for reading temperature intermittently.
- Read_Temperature_Auto.ino which measures at maximum repetition rate. Only the temperature register is read.

The readout process for intermittent reading is
```
Loop:  
  - enable V_BIAS 
  - wait until RC network has setteled, there appears to be no difference in reading when waiting 0 or 50ms, so this might not be needed
  - enable one shot, which initites a conversion
  - rtd.read_all( ); reads all registers
  - disable V_BIAS
  - clear faults if necessary
  - long delay
```
The readout process for fast reading is
```
Setup:   
  - enable V_BIAS 
  - wait until RC network has setteled
  - enable auto conversion
Loop:  
  - rtd.read_rtd( ); reads only rtd registers
  - clear faults if necessary
  - short delay
```

*Original library is from Ole Wolf <wolf@blazingangles.com> Copyright (C) 2015*  
*Updates by Urs Utzinger Summer, 2021*
