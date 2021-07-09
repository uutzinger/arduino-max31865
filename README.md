
# MAXIM 31865 Driver

The files in this folder provide an Arduino driver for the **MAX31865** RTD chip
and example code that illustrates how to use it.  Move the folder contents
to your Arduino/libraries folder.

This library was updated to work on **Teensy** (tested on 4.0) and to use current SPI setup routines.

It has been tested with [Adafruit Breakout Board](https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier)

The PT-100 sensor is rated from -200C to 550C.

There are two examples:

- ReadTemperature.ino which uses adafruit resistance to temperature conversion
- ReadTemperatureLUT.ino which uses the library from [drhaney](https://github.com/drhaney/pt100rtd)

The readout process is
```
  - enable V_BIAS 
  - wait until RC network has setteled, delay(10);
  - enable enable one shot, need to keep other settings same
  - rtd.read_all( );
  - disable V_BIAS
```

*Original library is from Ole Wolf <wolf@blazingangles.com> Copyright (C) 2015*  
*Updates by Urs Utzinger Summer, 2021*
