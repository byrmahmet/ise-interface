[![Build Status](https://travis-ci.org/u-fire/ph-probe-interface.svg)](https://travis-ci.org/u-fire/ph-probe-interface)
[![Codacy grade](https://img.shields.io/codacy/grade/7cbfbad007d84cd59d40fc76ddc700a0.svg)]()

### pH Probe Interface

> Measure pH easily
* pH
* pOH
* mV
* temperature in Celsius

Available on [Tindie](https://www.tindie.com/products/ufire/ph-probe-interface/)

Make one with the [source code](https://github.com/u-fire/ph-probe-interface) and [schematics](https://upverter.com/ufire/a3c74dc5b3b942a5/pH-Probe-Interface-2018a/).  

#### What it is
An ATTiny85 programmed as an I2C slave, a DS18B20 waterproof temperature probe, and a pH probe. It measures pH in the full range of 0 - 14 and optionally compensates for temperature.

#### Using it
An Arduino-compatible [library](https://github.com/u-fire/pHProbe) is provided to make using the probe easy and there is extensive [documentation](http://ufire.co/pHProbe/) on the use and setup of the device.

~~~
#include <pHProbe.h>
pH_Probe _pH;

pH = _pH.measurepH();
~~~

#### Compiling
This is a [PlatformIO](http://platformio.org/) project. Download and install it, import this repo, and it should download all the required tools for you. It expects a USBTiny device to upload the firmware.
