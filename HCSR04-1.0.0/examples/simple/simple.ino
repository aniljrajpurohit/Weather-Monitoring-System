#include <HCSR04.h>

UltraSonicDistanceSensor distanceSensor(13, 15);  // Initialize sensor that uses digital pins 13 and 12.

void setup () {
  Serial.begin(9600);  // We initialize serial connection so that we could print values from sensor.
}

void loop () {
  // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
  float distance = distanceSensor.measureDistanceCm();
  float abc = mapfloat(distance, 13, 3, 0, 10);
  if (abc <= 0)
    abc = 0;
  abc*=10;
  Serial.println(abc);
  delay(500);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
