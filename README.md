# BME280
Arduino Source for the Bosch BME280 MOD-1022 weather multi-sensor
# Using the Library

 // need to read the NVM compensation parameters
 
  BME280.readCompensationParams();
  
  // Need to turn on 1x oversampling, default is os_skipped, which means it doesn't measure anything
  
  BME280.writeOversamplingPressure(os1x);  // 1x over sampling (ie, just one sample)
  BME280.writeOversamplingTemperature(os1x);
  BME280.writeOversamplingHumidity(os1x);
  
  // example of a forced sample.  After taking the measurement the chip goes back to sleep
  
  BME280.writeMode(smForced);
  while (BME280.isMeasuring()) {
    Serial.println("Measuring...");
    delay(50);
  }
  Serial.println("Done!");
  
  // read out the data - must do this before calling the getxxxxx routines
  
  BME280.readMeasurements();
  Serial.print("Temp=");
  Serial.println(BME280.getTemperature());  // must get temp first
  Serial.print("Humidity=");
  Serial.println(BME280.getHumidity());
  Serial.print("Pressure=");
  Serial.println(BME280.getPressure());
  Serial.print("PressureMoreAccurate=");
  Serial.println(BME280.getPressureMoreAccurate());  // use int64 calculcations
  Serial.print("TempMostAccurate=");
  Serial.println(BME280.getTemperatureMostAccurate());  // use double calculations
  Serial.print("HumidityMostAccurate=");
  Serial.println(BME280.getHumidityMostAccurate()); // use double calculations
  Serial.print("PressureMostAccurate=");
  Serial.println(BME280.getPressureMostAccurate()); // use double calculations
