#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SFE_BMP180.h>

#define RF69_FREQ   433.0
#define RFM69_CS     1
#define RFM69_INT   24
#define RFM69_RST   25
#define LED         10

RH_RF69 rf69(RFM69_CS, RFM69_INT);

Adafruit_MPU6050 mpu;
SFE_BMP180 bmp180;

//const bool USE_BMP_TEMP = false;
//
//void Blink(byte pin, byte delay_ms, byte loops) {
//  while (loops--) { digitalWrite(pin, HIGH); delay(delay_ms); digitalWrite(pin, LOW); delay(delay_ms); }
//}



void setup() {
  Serial.begin(9600);

  if (!mpu.begin()) {
    Serial.println("Hittar inte MPU6050! Kontrollera I2C-koppling.");
    while (1) { delay(1000); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

}

void loop() {

  sensors_event_t a, g, t_mpu;
  mpu.getEvent(&a, &g, &t_mpu);

  float accelX     = a.acceleration.x;
  float accelY     = a.acceleration.y;
  float accelZ     = a.acceleration.z;

  Serial.println(String((accelX),2) + " " + String((accelY),2) + " " + String((accelZ),2));

  delay(500); // ~10 Hz
}
