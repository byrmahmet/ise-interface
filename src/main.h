#include <Arduino.h>
#include <TinyWireS.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/sleep.h>
#include <EEPROM.h>

#define VERSION 0x1a
#define ISE_PROBE_DEFAULT_ADDRESS  0x3f
#define PROBE_IN 3
#define PROBE_REF 2
#define PWM_POWER 1

uint8_t ISE_Probe;

#define ISE_MEASURE_MV 80
#define ISE_MEASURE_TEMP 40
#define ISE_CALIBRATE_SINGLE 20
#define ISE_CALIBRATE_LOW 10
#define ISE_CALIBRATE_HIGH 8
#define ISE_MEMORY_WRITE 4
#define ISE_MEMORY_READ 2
#define ISE_I2C 1

#define ISE_VERSION_REGISTER 0             /*!< version */
#define ISE_MV_REGISTER 1                  /*!< pH */
#define ISE_TEMP_REGISTER 5                /*!< temperature in C */
#define ISE_CALIBRATE_SINGLE_REGISTER 9    /*!< calibration offset */
#define ISE_CALIBRATE_REFHIGH_REGISTER 13  /*!< reference high calibration */
#define ISE_CALIBRATE_REFLOW_REGISTER 17   /*!< reference low calibration */
#define ISE_CALIBRATE_READHIGH_REGISTER 21 /*!< reading high calibration */
#define ISE_CALIBRATE_READLOW_REGISTER 25  /*!< reading low calibration */
#define ISE_SOLUTION_REGISTER  29          /*!< reference ISE solution */
#define ISE_BUFFER_REGISTER 33             /*!< buffer */
#define ISE_CONFIG_REGISTER 37             /*!< config register */
#define ISE_TASK_REGISTER 38               /*!< task register */

#define ISE_I2C_ADDRESS_REGISTER 200

#define PROBE_MV_TO_PH 0.0592
#define TEMP_CORRECTION_FACTOR 0.03

#define adc_disable() (ADCSRA &= ~(1 << ADEN)) // disable ADC (before power-off)
#define adc_enable() (ADCSRA |=  (1 << ADEN))  // re-enable ADC
#define ac_disable() ACSR    |= _BV(ACD);      // disable analog comparator
#define ac_enable() ACSR     &= _BV(ACD)       // enable analog comparator
#define timer1_disable() PRR |= _BV(PRTIM1)    // disable timer1_disable
#define INTERNAL2V56_NO_CAP (6)

#define DS18_PIN 5
OneWire oneWire(DS18_PIN);
DallasTemperature ds18(&oneWire);

#ifndef cbi
# define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif // ifndef cbi
#ifndef sbi
# define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif // ifndef sbi

float measuremV();
void  calibrateSingle();
void  calibrateLow();
void  calibrateHigh();
void  setI2CAddress();
void  saveEEPROM();
void  readEEPROM();
void  writeEEPROM();
float writeBias(float   requestedVoltage,
                float   inputV,
                uint8_t duration);

struct config
{
  uint8_t useDualPoint        : 1;
  uint8_t useTempCompensation : 1;
  uint8_t buffer              : 6;
};

struct rev1_register {
  uint8_t version;       // 0
  float   mV;            // 1-4
  float   tempC;         // 5-8
  float   single;        // 9-12
  float   referenceHigh; // 13-16
  float   referenceLow;  // 17-20
  float   readingHigh;   // 21-24
  float   readingLow;    // 25-28
  float   solution;      // 29-32
  float   buffer;        // 33-36
  config  CONFIG;        // 37
  uint8_t TASK;          // 38
} i2c_register;

volatile uint8_t reg_position;
const uint8_t    reg_size = sizeof(i2c_register);

// bool runpH              = false;
bool runTemp            = false;
bool runCalibrateSingle = false;
bool runCalibrateHigh   = false;
bool runCalibrateLow    = false;
bool runI2CAddress      = false;
bool runEEPROM          = false;
bool runmV              = false;
bool runReadEEPROM      = false;
bool runWriteEEPROM     = false;


void inline low_power()
{
  set_sleep_mode(SLEEP_MODE_IDLE);
  adc_disable();
  ac_disable();
  sleep_enable();
  sleep_mode();
  sleep_disable();
  adc_enable();
  ac_enable();
}

float getVin()
{
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || \
  defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || \
  defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || \
  defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
  #else // if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) ||
  // defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif // if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) ||
  // defined(__AVR_ATmega2560__)

  tws_delay(2);
  ADCSRA |= _BV(ADSC);

  while (bit_is_set(ADCSRA, ADSC));

  uint8_t low  = ADCL;
  uint8_t high = ADCH;

  long result = (high << 8) | low;

  result = 1125300L / result;
  return (double(result) - 0) * (6 - 0) / (6000 - 0) + 0;
}
