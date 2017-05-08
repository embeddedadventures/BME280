/*

Copyright (c) 2017, Embedded Adventures
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

// BME280 MOD-1022 weather multi-sensor Arduino demo using a SAMD21 board
// Written originally by Embedded Adventures

#include <avr/dtostrf.h>
#include <BME280_MOD-1022.h>
#include <Wire.h>

void setup() {
  SerialUSB.begin(115200);
  Wire.begin();
  SerialUSB.println("Welcome to the MOD1023 (BME280 & iAQ) test sketch");
  SerialUSB.println("Embedded Adventures (www.embeddedadventures.com");
  uint8_t chipID = BME280.readChipId();
  SerialUSB.print("BME280 Chip ID: 0x");
  SerialUSB.println(chipID, HEX);

  bme280_forcedSample();
}

void loop() {
  bme280_indoorSample();
  SerialUSB.println("--------------------------------------");
  delay(1500);
}

//Numbers printed cleanly
void printFormattedFloat(float x, uint8_t precision) {
  char buffer[10];
  dtostrf(x, 7, precision, buffer);
  SerialUSB.print(buffer);
}

void printCompensatedMeasurements(void) {
  float temp, humidity,  pressure, pressureMoreAccurate;
  double tempMostAccurate, humidityMostAccurate, pressureMostAccurate;
  char buffer[80];

  temp      = BME280.getTemperature();
  humidity  = BME280.getHumidity();
  pressure  = BME280.getPressure();
  
  pressureMoreAccurate = BME280.getPressureMoreAccurate();  // t_fine already calculated from getTemperaure() above
  tempMostAccurate     = BME280.getTemperatureMostAccurate();
  humidityMostAccurate = BME280.getHumidityMostAccurate();
  pressureMostAccurate = BME280.getPressureMostAccurate();

  SerialUSB.println("\t\tGood\t\tBetter\t\tBest");
  SerialUSB.print("Temperature\t");
  printFormattedFloat(temp, 2);
  SerialUSB.print("\t\t-\t\t");
  printFormattedFloat(tempMostAccurate, 2);
  SerialUSB.println();
  
  SerialUSB.print("Humidity\t");
  printFormattedFloat(humidity, 2);
  SerialUSB.print("\t\t-\t\t");
  printFormattedFloat(humidityMostAccurate, 2);
  SerialUSB.println();

  SerialUSB.print("Pressure\t");
  printFormattedFloat(pressure, 2);
  SerialUSB.print("\t\t");
  printFormattedFloat(pressureMoreAccurate, 2);
  SerialUSB.print("\t\t");
  printFormattedFloat(pressureMostAccurate, 2);
  SerialUSB.println();
}

// example of a forced sample.  After taking the measurement the chip goes back to sleep
void bme280_forcedSample() {
  // need to read the NVM compensation parameters
  BME280.readCompensationParams();
  
  // Need to turn on 1x oversampling, default is os_skipped, which means it doesn't measure anything
  BME280.writeOversamplingPressure(os1x);  // 1x over sampling (ie, just one sample)
  BME280.writeOversamplingTemperature(os1x);
  BME280.writeOversamplingHumidity(os1x);
  
  
  BME280.writeMode(smForced);
  SerialUSB.println("BME280 Forced Sample Reading");
  SerialUSB.print("Measuring");
  while (BME280.isMeasuring()) {
    SerialUSB.print(".");
    delay(100);
  }
  SerialUSB.println("Done!");
  
  // read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();
  SerialUSB.print("Temperature =\t");
  SerialUSB.println(BME280.getTemperature());  // must get temp first
  SerialUSB.print("Humidity =\t");
  SerialUSB.println(BME280.getHumidity());
  SerialUSB.print("Pressure =\t");
  SerialUSB.println(BME280.getPressure());
  SerialUSB.println("\tMore Accurate Reading");
  SerialUSB.print("Pressure =\t");
  SerialUSB.println(BME280.getPressureMoreAccurate());  // use int64 calculcations
  SerialUSB.println("\tMost Accuracy Reading");
  SerialUSB.print("Temperature =\t");
  SerialUSB.println(BME280.getTemperatureMostAccurate());  // use double calculations
  SerialUSB.print("Humidity =\t");
  SerialUSB.println(BME280.getHumidityMostAccurate()); // use double calculations
  SerialUSB.print("Pressure =\t");
  SerialUSB.println(BME280.getPressureMostAccurate()); // use double calculations
  SerialUSB.println();
}

// Example for "indoor navigation"
// We'll switch into normal mode for regular automatic samples
void bme280_indoorSample() {
  
  
  BME280.writeStandbyTime(tsb_0p5ms);        // tsb = 0.5ms
  BME280.writeFilterCoefficient(fc_16);      // IIR Filter coefficient 16
  BME280.writeOversamplingPressure(os16x);    // pressure x16
  BME280.writeOversamplingTemperature(os2x);  // temperature x2
  BME280.writeOversamplingHumidity(os1x);     // humidity x1
  
  BME280.writeMode(smNormal);
  SerialUSB.println("BME280 Normal Mode Reading");
  //Do nothing while measuring
  while (BME280.isMeasuring()) {    }
    
  // read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();
  printCompensatedMeasurements();
  SerialUSB.println();
}



