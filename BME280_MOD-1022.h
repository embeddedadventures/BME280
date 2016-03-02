/*

Copyright (c) 2015, Embedded Adventures
All rights reserved.

Contact us at source [at] embeddedadventures.com
www.embeddedadventures.com

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

- Neither the name of Embedded Adventures nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.

*/

// BME280 MOD-1022 weather multi-sensor Arduino library
// Written originally by Embedded Adventures


#ifndef __BME280_MOD-1022_H
#define __BME280_MOD-1022_H

#include <inttypes.h>

// register defines

#define regCalibStart     0x88
#define regChipID         0xd0
#define regReset          0xe0
#define regCalibStart2    0xe1
#define regCtrlHum        0xf2
#define regStatus         0xf3
#define regCtrlMeas       0xf4
#define regConfig         0xf5
#define regMeasurementsStart      0xf7

// address of BME280 on the MOD-1022 board
#define addrBME280        0x76

// integer types for BME code

#define BME280_S32_t      int32_t
#define BME280_U32_t      uint32_t
#define BME280_S64_t      int64_t

// order of the data we pull back from the sensor

enum dataOrder_e {
  press_msb,
  press_lsb,
  press_xlsb,
  temp_msb,
  temp_lsb,
  temp_xlsb,
  hum_msb,
  hum_lsb,
};

// t_sb standby options - effectively the gap between automatic measurements
// when in "normal" mode

enum standbySettings_e {
  tsb_0p5ms,
  tsb_62p5ms,
  tsb_125ms,
  tsb_250ms,
  tsb_500ms,
  tsb_1000ms,
  tsb_10ms,
  tsb_20ms
};

// sensor modes, it starts off in sleep mode on power on
// forced is to take a single measurement now
// normal takes measurements reqularly automatically

enum mode_e {
  smSleep,
  smForced,
  smNormal = 3,
};

// Filter coefficients
// higher numbers slow down changes, such as slamming doors

enum filterCoefficient_e {
  fc_off,
  fc_2,
  fc_4,
  fc_8,
  fc_16
};

// Oversampling options
// Oversampling reduces the noise from the sensor

enum oversampling_e {
  osSkipped,
  os1x,
  os2x,
  os4x,
  os8x,
  os16x,
};

// Structure to hold the compensation parameters necessary for the calculations

typedef struct _compParams_ts {
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;

  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;

  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  uint8_t  dig_H6;

} compParams_ts;

// union to make it easier to slurp up the first chunk of measurements

union compParams_u
{
   uint8_t        compArray[28];
   compParams_ts  compStruct;
};

class BME280Class
{
private:
  uint16_t readSensor(uint8_t command);
  uint8_t  readRegister(uint8_t register);
  void     writeRegister(uint8_t register, uint8_t data);

  // routines from the BME datasheet
  int32_t  BME280_compensate_T_int32(BME280_S32_t adc_T);
  uint32_t BME280_compensate_P_int64(BME280_S32_t adc_P);
  uint32_t BME280_compensate_P_int32(BME280_S32_t adc_P);
  uint32_t BME280_compensate_H_int32(BME280_S32_t adc_H);
  double   BME280_compensate_T_double(BME280_S32_t adc_T);
  double   BME280_compensate_P_double(BME280_S32_t adc_P);
  double   BME280_compensate_H_double(BME280_S32_t adc_H);

  union compParams_u compParams;

  // t_fine used in calculations to compensate based on temperature
  BME280_S32_t t_fine;

  // store the raw adc readings
  BME280_S32_t adc_t;
  BME280_S32_t adc_p;
  BME280_S32_t adc_h;

public:

  void     writeMode(mode_e m);	// set the mode (forced, normal, sleep)
  uint8_t  isMeasuring(void);	// is the BME280 still doing measurements?
  uint8_t  doingIMUpdate(void);	// Is the BME280 still updating parameters?
  uint8_t  readChipId(void);	// read the chip ID
  void     writeOversamplingPressure(oversampling_e os);	// set oversampling for pressure, must be >0 to get any readings at all
  void     writeOversamplingTemperature(oversampling_e os);	// set oversampling for temperature, must be >0 to get any readings at all
  void     writeOversamplingHumidity(oversampling_e os);	// set oversampling for humidity, must be >0 to get any readings at all
  void     writeStandbyTime(standbySettings_e t_sb);	// set time inbetween readings in normal mode
  void     writeFilterCoefficient(filterCoefficient_e fc);	// set filter coefficient (see datasheet)
  void     readCompensationParams(void);  // read and store the compensation parameters
  void     readMeasurements(void);	// pull the raw ADC measurements from the chip - next use getTemperature etc to get the compensated value
  uint8_t  readCtrlMeas(void);  // read the CtrlMeas register
  float    getTemperature(void); // get the compensated temperature
  double   getTemperatureMostAccurate(void);	// get the most accurate compensated temperature value (more flash/code required)
  float    getHumidity(void);	// get the compensated humidity value
  double   getHumidityMostAccurate(void);	// get the most accurate compensated humidity value (more flash/code required)
  float    getPressure(void); 	// get the compensated pressure value
  float    getPressureMoreAccurate(void); 	// get more accurate pressure value (more flash/code required)
  double   getPressureMostAccurate(void);	// get the most accurate pressure value (even more flash/ code required)

};

extern BME280Class BME280;

#endif
