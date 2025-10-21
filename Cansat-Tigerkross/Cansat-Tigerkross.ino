#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(115200)

}

void loop() {
  float temperature = bmp.readTemperature();  
  int32_t pressure = bmp.readPressure();      
}

