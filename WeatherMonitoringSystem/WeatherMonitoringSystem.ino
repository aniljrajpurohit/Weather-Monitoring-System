#include <Arduino.h>
#include <Wire.h>
#include <SFE_BMP180.h>
#include <LiquidCrystal_I2C.h>
#include <HCSR04.h>
#include "DHT.h"
#define I2C_ADDRESS 0x77
#define DHTTYPE DHT11   // DHT 11
#define SERIAL_DEBUG
#define dht_dpin 14     //pin d5 of NodeMCU
DHT dht(dht_dpin, DHTTYPE);   //creating the bohect of DHT class
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
SFE_BMP180 bmp180;
UltraSonicDistanceSensor distanceSensor(13, 15); //13 - d7Trig, 15,d8 Echo

float UVdata, humidity, temperature, Pressure;


void setup()
{
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
#endif
  Wire.begin();
  lcd.init();                      // initialize the lcd
  dht.begin();
  lcd.backlight();

  bool success = bmp180.begin();

  if (success) {
    Serial.println("BMP180 init success");
  }


  Serial.println("Humidity and temperature\n\n");
  lcd.setCursor(7, 0);
  lcd.print("Welcome");
  lcd.setCursor(9, 1);
  lcd.print("To");
  lcd.setCursor(1, 2);
  lcd.print("Weather Monitoring");
  lcd.setCursor(7, 3);
  lcd.print("System");

  delay(3000);
  lcd.clear();

}

void loop() {
  rainFallData();
  getDHT11_data();
  UVdata = UVData();
  char status;
  double T, P;
  bool success = false;

  status = bmp180.startTemperature();

  if (status != 0) {
    delay(1000);
    status = bmp180.getTemperature(T);

    if (status != 0) {
      status = bmp180.startPressure(3);

      if (status != 0) {
        delay(status);
        status = bmp180.getPressure(P, T);

        if (status != 0) {
          //          Serial.print("Pressure: ");
          //          Serial.print(P);
          //          Serial.println(" hPa");
          //
          //          Serial.print("Temperature: ");
          //          Serial.print(T);
          //          Serial.println(" C");
        }
      }
    }
  }


  Serial.print("UV Index = ");
  Serial.print(UVdata);
  Serial.print(" mW/cm^2");
  Serial.print(" | ");
  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.print(" | ");
  Serial.print("Temperature = ");
  Serial.print(T);
  Serial.print(" C");
  Serial.print(" | ");
  Serial.print("Pressure = ");
  Serial.print(P);
  Serial.println(" hPa");
  Serial.print(" | ");
  Serial.print("Rainfall = ");
  Serial.print(rainFallData());
  Serial.println(" mm");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("UV Idx=");
  lcd.print(UVdata);
  lcd.print(" mW/cm^2");

  lcd.setCursor(0, 1);
  lcd.print("Humidity = ");
  lcd.print(humidity);

  lcd.setCursor(0, 2);
  lcd.print("Temp = ");
  lcd.print(T);
  lcd.print(" C");

  lcd.setCursor(0, 3);
  lcd.print("Press = ");
  lcd.print(P);
  lcd.print(" hPa");
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Rainfall = ");
  lcd.print(rainFallData());
  lcd.print(" mm");
  delay(2000);

}

float UVData() {
  float data = analogRead(A0);
  float UVdata = (3.3 / 1024) * data;
  float UVindex = mapfloat(UVdata, 0.99, 2.8, 0.0, 15.0);
  return UVindex;
}

void getDHT11_data() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
}

float rainFallData() {
  float distance = distanceSensor.measureDistanceCm();
  float raindata = mapfloat(distance, 13, 3, 0, 10);
  if (raindata <= 0)
    raindata = 0;
  raindata *= 10;
  return raindata;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
