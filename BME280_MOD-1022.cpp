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

#include "BME280_MOD-1022.h"
#include "Wire.h"

// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
// t_fine carries fine temperature as global value

double BME280Class::BME280_compensate_T_double(BME280_S32_t adc_T) {

double var1, var2, T;

  var1 = (((double)adc_T)/16384.0 - ((double)compParams.compStruct.dig_T1)/1024.0) * ((double)compParams.compStruct.dig_T2);
  var2 = ((((double)adc_T)/131072.0 - ((double)compParams.compStruct.dig_T1)/8192.0) *
  (((double)adc_T)/131072.0 - ((double) compParams.compStruct.dig_T1)/8192.0)) * ((double)compParams.compStruct.dig_T3);
  t_fine = (BME280_S32_t)(var1 + var2);
  T = (var1 + var2) / 5120.0;
  return T;
}

// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double BME280Class::BME280_compensate_P_double(BME280_S32_t adc_P) {
  
double var1, var2, p;

  var1 = ((double)t_fine/2.0) - 64000.0;
  var2 = var1 * var1 * ((double)compParams.compStruct.dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)compParams.compStruct.dig_P5) * 2.0;
  var2 = (var2/4.0)+(((double)compParams.compStruct.dig_P4) * 65536.0);
  var1 = (((double)compParams.compStruct.dig_P3) * var1 * var1 / 524288.0 + ((double)compParams.compStruct.dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0)*((double)compParams.compStruct.dig_P1);
  if (var1 == 0.0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576.0 - (double)adc_P;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double)compParams.compStruct.dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double)compParams.compStruct.dig_P8) / 32768.0;
  p = p + (var1 + var2 + ((double)compParams.compStruct.dig_P7)) / 16.0;
  return p;
}

// Returns humidity in %rH as as double. Output value of “46.332” represents 46.332 %rH
double BME280Class::BME280_compensate_H_double(BME280_S32_t adc_H) {

 double var_H;
 
  var_H = (((double)t_fine) - 76800.0);
  var_H = (adc_H - (((double)compParams.compStruct.dig_H4) * 64.0 + ((double)compParams.compStruct.dig_H5) / 16384.0 * var_H)) *
          (((double)compParams.compStruct.dig_H2) / 65536.0 * (1.0 + ((double)compParams.compStruct.dig_H6) / 67108864.0 * var_H *
          (1.0 + ((double)compParams.compStruct.dig_H3) / 67108864.0 * var_H)));
  var_H = var_H * (1.0 - ((double)compParams.compStruct.dig_H1) * var_H / 524288.0);
  if (var_H > 100.0) {
    var_H = 100.0;
  } else if (var_H < 0.0) {
    var_H = 0.0;
  }  
  return var_H;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
BME280_U32_t BME280Class::BME280_compensate_P_int32(BME280_S32_t adc_P)
{
BME280_S32_t var1, var2;
BME280_U32_t p;
  var1 = (((BME280_S32_t)t_fine)>>1) - (BME280_S32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((BME280_S32_t)compParams.compStruct.dig_P6);
  var2 = var2 + ((var1*((BME280_S32_t)compParams.compStruct.dig_P5))<<1);
  var2 = (var2>>2)+(((BME280_S32_t)compParams.compStruct.dig_P4)<<16);
  var1 = (((compParams.compStruct.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((BME280_S32_t)compParams.compStruct.dig_P2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((BME280_S32_t)compParams.compStruct.dig_P1))>>15);
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = (((BME280_U32_t)(((BME280_S32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((BME280_U32_t)var1);
  } else {
    p = (p / (BME280_U32_t)var1) * 2;
  }
  var1 = (((BME280_S32_t)compParams.compStruct.dig_P9) * ((BME280_S32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((BME280_S32_t)(p>>2)) * ((BME280_S32_t)compParams.compStruct.dig_P8))>>13;
  p = (BME280_U32_t)((BME280_S32_t)p + ((var1 + var2 + compParams.compStruct.dig_P7) >> 4));
  return p;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value

int32_t BME280Class::BME280_compensate_T_int32(BME280_S32_t adc_T)
{
BME280_S32_t var1, var2, T;

  var1 = ((((adc_T>>3) -((BME280_S32_t)compParams.compStruct.dig_T1<<1))) * ((BME280_S32_t)compParams.compStruct.dig_T2)) >> 11;
  var2 = (((((adc_T>>4) - ((BME280_S32_t)compParams.compStruct.dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)compParams.compStruct.dig_T1))) >> 12) * ((BME280_S32_t)compParams.compStruct.dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280Class::BME280_compensate_P_int64(BME280_S32_t adc_P)
{
BME280_S64_t var1, var2, p;

  var1 = ((BME280_S64_t)t_fine) - 128000;
  var2 = var1 * var1 * (BME280_S64_t)compParams.compStruct.dig_P6;
  var2 = var2 + ((var1*(BME280_S64_t)compParams.compStruct.dig_P5)<<17);
  var2 = var2 + (((BME280_S64_t)compParams.compStruct.dig_P4)<<35);
  var1 = ((var1 * var1 * (BME280_S64_t)compParams.compStruct.dig_P3)>>8) + ((var1 * (BME280_S64_t)compParams.compStruct.dig_P2)<<12);
  var1 = (((((BME280_S64_t)1)<<47)+var1))*((BME280_S64_t)compParams.compStruct.dig_P1)>>33;
  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576-adc_P;
  p = (((p<<31)-var2)*3125)/var1;
  var1 = (((BME280_S64_t)compParams.compStruct.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((BME280_S64_t)compParams.compStruct.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)compParams.compStruct.dig_P7)<<4);
  return (BME280_U32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
BME280_U32_t BME280Class::BME280_compensate_H_int32(BME280_S32_t adc_H)
{
BME280_S32_t v_x1_u32r;
  v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)compParams.compStruct.dig_H4) << 20) - (((BME280_S32_t)compParams.compStruct.dig_H5) * v_x1_u32r)) +
    ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)compParams.compStruct.dig_H6)) >> 10) * (((v_x1_u32r *
    ((BME280_S32_t)compParams.compStruct.dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) *
    ((BME280_S32_t)compParams.compStruct.dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)compParams.compStruct.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (BME280_U32_t)(v_x1_u32r>>12);
}


float  BME280Class::getTemperature(void) {
  return (float)BME280_compensate_T_int32(adc_t) / 100;
}

double BME280Class::getTemperatureMostAccurate(void) {
  return BME280_compensate_T_double(adc_t);
}


float BME280Class::getHumidity(void) {
  return (float)BME280_compensate_H_int32(adc_h) / 1024;
}

double BME280Class::getHumidityMostAccurate(void) {
  return BME280_compensate_H_double(adc_h);
}

float BME280Class::getPressure(void) {
  return (float)BME280_compensate_P_int32(adc_p) / 100;
}

float BME280Class::getPressureMoreAccurate(void) {
  return (float)BME280_compensate_P_int64(adc_p) / 256 / 100;
} 

double BME280Class::getPressureMostAccurate(void) {
  return BME280_compensate_P_double(adc_p) / 100;
}


void BME280Class::writeStandbyTime(standbySettings_e t_sb) {

uint8_t conf;

  conf = readRegister(regConfig);  // Status is hidden in here
  // We want to change osrs_p which is bits 4,3,2
  conf = conf & 0b00011111; // mask out the bits we care about
  conf = conf | (t_sb << 5); // Set the magic bits

  writeRegister(regConfig, conf);

}   

void BME280Class::writeFilterCoefficient(filterCoefficient_e fc) {

uint8_t conf;

  conf = readRegister(regConfig);  // Status is hidden in here
  // We want to change osrs_p which is bits 4,3,2
  conf = conf & 0b11100011; // mask out the bits we care about
  conf = conf | (fc << 2); // Set the magic bits

  writeRegister(regConfig, conf);

} 

void BME280Class::writeOversamplingPressure(oversampling_e os) {
 uint8_t ctrlMeas;
  ctrlMeas = readRegister(regCtrlMeas);  // Status is hidden in here
  // We want to change osrs_p which is bits 4,3,2
  ctrlMeas = ctrlMeas & 0b11100011; // mask out the bits we care about
  ctrlMeas = ctrlMeas | (os << 2); // Set the magic bits

  writeRegister(regCtrlMeas, ctrlMeas);

}  

void BME280Class::writeOversamplingTemperature(oversampling_e os) {
 uint8_t ctrlMeas;

  ctrlMeas = readRegister(regCtrlMeas);  // osrs_t is in CtrlMeas
  // We want to change osrs_t which is bits 7,6,5
  ctrlMeas = ctrlMeas & 0b00011111; // mask out the bits we care about
  ctrlMeas = ctrlMeas | (os << 5); // Set the magic bits

  writeRegister(regCtrlMeas, ctrlMeas);

} 

void BME280Class::writeOversamplingHumidity(oversampling_e os) {
 
  // We want to change osrs_h which is bits 2,1,0 - there are no other bits though, so we can just se it.

  writeRegister(regCtrlHum, os);

}

void BME280Class::readCompensationParams(void) {
  
  uint8_t count;
  uint16_t regE5, regE6;
  
  Wire.beginTransmission(addrBME280);
  Wire.write(regCalibStart);
  Wire.endTransmission();
  Wire.requestFrom(addrBME280, 24); 
  for (count = 0; count < 28; count++) {  // first 28 bytes we can process like this
   compParams.compArray[count] = Wire.read();
  }
  // then they go a bit strangely

  compParams.compStruct.dig_H1 = readRegister(0xA1);

  Wire.beginTransmission(addrBME280);
  Wire.write(regCalibStart2);
  Wire.endTransmission();
  Wire.requestFrom(addrBME280, 7);

  compParams.compStruct.dig_H2 = Wire.read();
  compParams.compStruct.dig_H2 |= (uint16_t)Wire.read() << 8;
  compParams.compStruct.dig_H3 = Wire.read();
  compParams.compStruct.dig_H4 = (uint16_t)Wire.read() << 4; // bits 11:4
  regE5 = Wire.read();
  compParams.compStruct.dig_H4 |= regE5 & 0b00001111; // bits 11:4
  compParams.compStruct.dig_H5 = regE5 >> 4;
  regE6 = Wire.read();
  compParams.compStruct.dig_H5 |= regE6 << 4;
  compParams.compStruct.dig_H6 = Wire.read();
  
 
}  

uint8_t BME280Class::readRegister(uint8_t reg) {

  uint8_t result;

  Wire.beginTransmission(addrBME280);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addrBME280, 1); 
  result = Wire.read();

  return result;
}

void BME280Class::writeRegister(uint8_t reg, uint8_t data) {

  Wire.beginTransmission(addrBME280);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}    

void BME280Class::readMeasurements(void) {

  uint8_t data[8];
  uint8_t count;

  Wire.beginTransmission(addrBME280);
  Wire.write(regMeasurementsStart);
  Wire.endTransmission();
  Wire.requestFrom(addrBME280, 8); 
  for (count = 0; count < 8; count++) {
    data[count] = Wire.read();
  }
  adc_h = data[hum_lsb];
  adc_h |= (uint32_t)data[hum_msb] << 8;
  
  adc_t  = (uint32_t)data[temp_xlsb] >> 4;
  adc_t |= (uint32_t)data[temp_lsb] << 4;
  adc_t |= (uint32_t)data[temp_msb] << 12;
    
  adc_p  = (uint32_t)data[press_xlsb] >> 4;
  adc_p |= (uint32_t)data[press_lsb] << 4;
  adc_p |= (uint32_t)data[press_msb] << 12;
}


void BME280Class::writeMode(mode_e m) {

  uint8_t ctrlMeas;

  ctrlMeas = readRegister(regCtrlMeas);  // Status is hidden in here
  // We want to change mode to 01
  ctrlMeas = ctrlMeas & 0b11111100; // mask out the bits we care about
  ctrlMeas = ctrlMeas | m; // Set the magic bits

  writeRegister(regCtrlMeas, ctrlMeas);

}  

uint8_t  BME280Class::readChipId(void) {
  return readRegister(regChipID);  // Status is hidden in here
}    

uint8_t BME280Class::readCtrlMeas(void) {
   return readRegister(regCtrlMeas);  // Status is hidden in here
}
uint8_t BME280Class::isMeasuring(void) {

  if (readRegister(regStatus) & 0b00001000) {  // Measuring is hidden in here
    return 1;
  } 
  else {
    return 0;
  }
}

uint8_t BME280Class::doingIMUpdate(void) {

  if (readRegister(regStatus) & 0b00000001) {  // Measuring is hidden in here
    return 1;
  } 
  else {
    return 0;
  }
}

BME280Class BME280;
