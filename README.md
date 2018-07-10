### ISE Probe Interface

> Use any Ion Specific Electrode
* measure pH with automatic temperature compensation
* measure ORP and Eh
* raw mV
* temperature in Celsius
* library can be easily extended for any probe

#### What it is
An I2C slave device that can interface with any voltage based sensor such as a pH or ORP probe. There are pin headers available to connect a DS18B20 temperature sensor for automatic temperature compensation. The firmware on the device provides two calibration options, single or dual point.

#### Using it
There is extensive [documentation](http://ufire.co/ISE_Probe/) on the use and setup of the device. The library to use it is in the Arduino IDE, Particle.io IDE and a python implementation for Raspberry Pi and MicroPython is also available.

~~~
#include "uFire_ISE.h"
ISE_Probe mv;
mv.measuremV();
~~~
~~~
#include "ISE_pH.h"
ISE_pH ph;
ph.measurepH();
~~~
~~~
#include "ISE_ORP.h"
ISE_ORP orp;
orp.measureORP();
~~~

#### Compiling
This is a [PlatformIO](http://platformio.org/) project. Download and install it, import this repo, and it should download all the required tools for you. It expects a USBTiny device to upload the firmware.
