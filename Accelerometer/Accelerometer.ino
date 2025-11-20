#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;

float angleAccX;
float angleAccY;  
float angleAccZ;

float angleVelX = 0;
float angleVelY = 0;  
float angleVelZ = 0;

float angleX = 0;
float angleY = 0;  
float angleZ = 0;

void setup() {
  Serial.begin(9600);

  pinMode(10,INPUT);

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

  //if (digitalRead(10) == HIGH) 
  //{
    //Serial.println(String((Xcal),2) + " " + String((Ycal),2) + " " + String((Zcal),2));
  //}

  accelX += 0.438985;
  accelY -= 0.275097;
  accelZ -= 2.280243;

  accelX = (1.076808 * accelX) - (0.075704 * accelY) - (0.031436 * accelZ);
  accelY = (-0.075704 * accelX) + (1.057946 * accelY) + (0.026019 * accelZ);
  accelZ = (-0.031436 * accelX) + (0.026019 * accelY) + (1.036761 * accelZ);
  


  Serial.println("Acc:" + String((accelX),2) + " " + String((accelY),2) + " " + String((accelZ),2));

  angleAccX = round(100*(g.gyro.x+0.07))/100;
  angleAccY = round(100*(g.gyro.y+0.02))/100;  
  angleAccZ = round(100*(g.gyro.z+0.02))/100;



  angleX += (angleAccX)/2;
  angleY += (angleAccY)/2;
  angleZ += (angleAccZ)/2;

  Serial.println("Angle:" + String((angleX),2) + " " + String((angleY),2) + " " + String((angleZ),2));

  delay(10);
}