#include <Arduino.h>
#include <Wire.h>
#include "BME280_I2C.h"

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while(!Serial) {} // Wait
  bme.begin();
  Serial.println("Starting BME library test...");
  delay(2000);
}

void loop() {
  Serial.println("Testing the basic methods");
  temps = bme.getTemperature();
  Serial.println(temps);
    delay(1000);
  presus = bme.getPressure();
  Serial.println(presus);
    delay(1000);
  humids = bme.getHumidity();
  Serial.println(humids);
  delay(2000);
  
  temps, presus, humids = 0.0;

  Serial.println("Testing all data struct get method:");
  bme.read();
  Serial.println(bme.temperature);
  Serial.println(bme.pressure);
  Serial.println(bme.humidity);
  

  delay(5000);
}