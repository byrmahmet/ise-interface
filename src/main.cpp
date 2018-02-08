// Copyright (c) 2017 Justin Decker

//
// MIT License
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

/*!
   \file main.cpp
   \brief ISE Probe firmware ver 1

   ufire.co for links to documentation, examples, and libraries
   github.com/u-fire/ec-salinity-probe for feature requests, bug reports, and  questions
   questions@ufire.co to get in touch with someone

 */

#include <main.h>

double readADC(int channel)
{
  uint32_t total       = 0UL;
  uint16_t sampleCount = 4096;

  for (uint16_t i = 0; i < sampleCount; i++) {
    total += analogRead(channel);
  }

  total = total >> 6;
  double proportional = (total * 1.0) / (0b00000001 << 6);
  return proportional;
}

void requestEvent()
{
  TinyWireS.send(*((uint8_t *)&i2c_register + reg_position));
  reg_position++;
}

void receiveEvent(uint8_t howMany)
{
  if ((howMany < 1) || (howMany > 16))
  {
    return;
  }

  reg_position = TinyWireS.receive();
  howMany--;

  if (!howMany)
  {
    return;
  }

  while (howMany--)
  {
    *((uint8_t *)&i2c_register + reg_position) = TinyWireS.receive();

    if (reg_position == ISE_TASK_REGISTER)
    {
      if (i2c_register.TASK == ISE_MEASURE_MV) runmV = true;
      if (i2c_register.TASK == ISE_MEASURE_TEMP) runTemp = true;
      if (i2c_register.TASK == ISE_CALIBRATE_SINGLE) runCalibrateSingle = true;
      if (i2c_register.TASK == ISE_CALIBRATE_LOW) runCalibrateLow = true;
      if (i2c_register.TASK == ISE_CALIBRATE_HIGH) runCalibrateHigh = true;
      if (i2c_register.TASK == ISE_I2C) runI2CAddress = true;
      if (i2c_register.TASK == ISE_MEMORY_WRITE) runWriteEEPROM = true;
      if (i2c_register.TASK == ISE_MEMORY_READ) runReadEEPROM = true;
    }

    if (reg_position == (ISE_CALIBRATE_SINGLE_REGISTER + 3)) runEEPROM = true;
    if (reg_position == (ISE_CALIBRATE_REFHIGH_REGISTER + 3)) runEEPROM = true;
    if (reg_position == (ISE_CALIBRATE_REFLOW_REGISTER + 3)) runEEPROM = true;
    if (reg_position == (ISE_CALIBRATE_READHIGH_REGISTER + 3)) runEEPROM = true;
    if (reg_position == (ISE_CALIBRATE_READLOW_REGISTER + 3)) runEEPROM = true;
    if (reg_position == (ISE_CONFIG_REGISTER)) EEPROM.put(ISE_CONFIG_REGISTER, i2c_register.CONFIG);
    reg_position++;
  }
}

void setup()
{
  timer1_disable();

  ds18.setResolution(TEMP_12_BIT);

  // set ADC prescaler for oversampling
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  EEPROM.get(ISE_I2C_ADDRESS_REGISTER,        ISE_Probe);
  EEPROM.get(ISE_CALIBRATE_SINGLE_REGISTER,   i2c_register.single);
  EEPROM.get(ISE_CALIBRATE_REFHIGH_REGISTER,  i2c_register.referenceHigh);
  EEPROM.get(ISE_CALIBRATE_REFLOW_REGISTER,   i2c_register.referenceLow);
  EEPROM.get(ISE_CALIBRATE_READHIGH_REGISTER, i2c_register.readingHigh);
  EEPROM.get(ISE_CALIBRATE_READLOW_REGISTER,  i2c_register.readingLow);
  EEPROM.get(ISE_CONFIG_REGISTER,             i2c_register.CONFIG);

  // if the EEPROM was blank, the user hasn't changed the i2c address, make it
  // the default address.
  if (ISE_Probe == 0xff)
  {
    ISE_Probe = ISE_PROBE_DEFAULT_ADDRESS;
  }

  // first time used, eeprom will be all 1's
  if (i2c_register.CONFIG.buffer == 0b111111)
  {
    i2c_register.CONFIG.useTempCompensation = 0;
    i2c_register.CONFIG.useDualPoint        = 0;
    i2c_register.CONFIG.buffer              = 0;
    EEPROM.put(ISE_CONFIG_REGISTER, i2c_register.CONFIG);
  }

  i2c_register.version = VERSION;

  TinyWireS.begin(ISE_Probe);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
}

void loop()
{
  low_power();

  TinyWireS_stop_check();
  if (runTemp)
  {
    ds18.requestTemperatures();
    i2c_register.tempC = ds18.getTempCByIndex(0);
    runTemp            = false;
  }

  TinyWireS_stop_check();
  if (runCalibrateSingle)
  {
    calibrateSingle();
    runCalibrateSingle = false;
  }

  TinyWireS_stop_check();
  if (runCalibrateHigh)
  {
    calibrateHigh();
    runCalibrateHigh = false;
  }

  TinyWireS_stop_check();
  if (runCalibrateLow)
  {
    calibrateLow();
    runCalibrateLow = false;
  }

  TinyWireS_stop_check();
  if (runI2CAddress)
  {
    setI2CAddress();
    runI2CAddress = false;
  }

  TinyWireS_stop_check();
  if (runEEPROM)
  {
    saveEEPROM();
    runEEPROM = false;
  }

  TinyWireS_stop_check();
  if (runmV)
  {
    measuremV();
    runmV = false;
  }

  TinyWireS_stop_check();
  if (runReadEEPROM)
  {
    readEEPROM();
    runReadEEPROM = false;
  }

  TinyWireS_stop_check();
  if (runWriteEEPROM)
  {
    writeEEPROM();
    runWriteEEPROM = false;
  }
}

float writeBias(float requestedVoltage, float inputV, uint8_t duration) {
  uint8_t analogWriteValue = requestedVoltage / (inputV / 255);

  pinMode(PWM_POWER, OUTPUT);
  digitalWrite(PWM_POWER, LOW);
  tws_delay(250);

  analogWrite(PWM_POWER, analogWriteValue);
  tws_delay(duration);

  return (inputV / 255) * analogWriteValue;
}

float measuremV() {
  double differentialmV;
  float  probeIn, probeRef, inputV, adcResolution;

  // see if the probe is positive or negative and see what
  // reference we can use for the best accuracy
  analogReference(DEFAULT);
  inputV = getVin();
  writeBias(2.50, inputV, 250);
  probeIn  = ((readADC(PROBE_IN) * inputV) / 1024.0);
  probeRef = ((readADC(PROBE_REF) * inputV) / 1024.0);
  analogWrite(PWM_POWER, 0);
  differentialmV = (probeIn - probeRef);

  if (differentialmV <= -1.1)
  {
    analogReference(INTERNAL2V56);
    adcResolution = 2.56;
    writeBias(2.50, inputV, 250);
  }
  else if (differentialmV <= -0)
  {
    analogReference(INTERNAL);
    adcResolution = 1.1;
    writeBias(1.1, inputV, 250);
  }
  else if (differentialmV <= 1.1)
  {
    analogReference(INTERNAL);
    adcResolution = 1.1;
  }
  else if (differentialmV <= 2.56)
  {
    analogReference(INTERNAL2V56);
    adcResolution = 2.56;
  }

  probeIn  = ((readADC(PROBE_IN) * adcResolution) / 1024.0);
  probeRef = ((readADC(PROBE_REF) * adcResolution) / 1024.0);

  digitalWrite(PWM_POWER, LOW);
  pinMode(PWM_POWER, INPUT);

  // turn V into mV
  differentialmV = (probeIn - probeRef) * 1000;

  // Use single point adjustment, ignoring if NaN
  if (i2c_register.single == i2c_register.single)
  {
    differentialmV = differentialmV - (differentialmV * i2c_register.single);
  }

  // Use dual point adjustment, if configured
  if (i2c_register.CONFIG.useDualPoint)
  {
    float referenceRange = i2c_register.referenceHigh - i2c_register.referenceLow;
    float readingRange   = i2c_register.readingHigh - i2c_register.readingLow;

    differentialmV = (((differentialmV - i2c_register.readingLow) * referenceRange) / readingRange) + i2c_register.referenceLow;
  }

  i2c_register.mV = differentialmV;
  return differentialmV;
}

void calibrateSingle()
{
  i2c_register.single = NAN;
  float v = measuremV();
  i2c_register.readingHigh = v;
  i2c_register.single      = (v - i2c_register.solution) / v;
  EEPROM.put(ISE_CALIBRATE_SINGLE_REGISTER, i2c_register.single);
}

void calibrateLow()
{
  i2c_register.readingLow   = measuremV();
  i2c_register.referenceLow = i2c_register.solution;
  EEPROM.put(ISE_CALIBRATE_REFLOW_REGISTER,  i2c_register.referenceLow);
  EEPROM.put(ISE_CALIBRATE_READLOW_REGISTER, i2c_register.readingLow);
}

void calibrateHigh()
{
  i2c_register.readingHigh   = measuremV();
  i2c_register.referenceHigh = i2c_register.solution;
  EEPROM.put(ISE_CALIBRATE_REFHIGH_REGISTER,  i2c_register.referenceHigh);
  EEPROM.put(ISE_CALIBRATE_READHIGH_REGISTER, i2c_register.readingHigh);
}

void setI2CAddress()
{
  // for convenience, the solution register is used to send the address
  EEPROM.put(ISE_I2C_ADDRESS_REGISTER, i2c_register.solution);
  TinyWireS.begin(i2c_register.solution);
}

void saveEEPROM()
{
  EEPROM.put(ISE_CALIBRATE_SINGLE_REGISTER,   i2c_register.single);
  EEPROM.put(ISE_CALIBRATE_REFHIGH_REGISTER,  i2c_register.referenceHigh);
  EEPROM.put(ISE_CALIBRATE_REFLOW_REGISTER,   i2c_register.referenceLow);
  EEPROM.put(ISE_CALIBRATE_READHIGH_REGISTER, i2c_register.readingHigh);
  EEPROM.put(ISE_CALIBRATE_READLOW_REGISTER,  i2c_register.readingLow);
}

void readEEPROM()
{
  // get the information stored in EEPROM at the address provided in ISE_SOLUTION_REGISTER
  // and put it into the register buffer (up to 4 bytes)
  EEPROM.get(static_cast<int>(i2c_register.solution), i2c_register.buffer);
}

void writeEEPROM()
{
  EEPROM.put(i2c_register.solution, i2c_register.buffer);
}
