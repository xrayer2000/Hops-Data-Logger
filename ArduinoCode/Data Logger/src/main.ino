#include <Arduino.h>
#include <BH1750.h>
#include <BME280I2C.h>
#include <Wire.h>

BH1750 lightMeter;
BME280I2C bme;  
float lux;    

void setup()
{
  Serial.begin(115200);
  while(!Serial) {} 
  Wire.begin();
  lightMeter.begin();

  while(!bme.begin()){Serial.println("Could not find BME280 sensor!"); delay(1000);}
}

void loop()
{
  printBME280Data(&Serial);
}

void printBME280Data()
{
  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);
  lux = lightMeter.readLightLevel();

  print("Temp: ");
  print(temp);
  print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
  print("\tHumidity: ");
  print(hum);
  print("% RH");
  print("\tPressure: ");
  print(pres);
  print("Pa, ");
  Serial.print("\tLight: ");
  Serial.print(lux);
  Serial.println(" lx");

   delay(1000);
}

