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

const bool USE_BMP_TEMP = false;

void Blink(byte pin, byte delay_ms, byte loops) {
  while (loops--) { digitalWrite(pin, HIGH); delay(delay_ms); digitalWrite(pin, LOW); delay(delay_ms); }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  pinMode(RFM69_INT, OUTPUT);
  pinMode(RFM69_CS, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  digitalWrite(RFM69_RST, HIGH); delay(10);
  digitalWrite(RFM69_RST, LOW);  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1) { delay(1); }
  }
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  rf69.setTxPower(20, true);
  Serial.print("RFM69 @ "); Serial.print(RF69_FREQ); Serial.println(" MHz");

  if (!mpu.begin()) {
    Serial.println("Hittar inte MPU6050! Kontrollera I2C-koppling.");
    while (1) { delay(1000); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  if (bmp180.begin()) {
    Serial.println("BMP180 OK");
  } else {
    Serial.println("BMP180 hittades inte");
  }

  Serial.println("Setup klar.");

}

void loop() {
  double T,P;
  char status;
  bool success = false;

  status = bmp180.startTemperature();

  if (status != 0) {
    delay(1000);
    status = bmp180.getTemperature(T); 
    if (status != 0) {
      status = bmp180.startPressure(3);
      if (status != 0) {
        delay(status);
        status = bmp180.getPressure(P,T);
      }
    }
  }

  
  unsigned long time_ms = millis()/1000;

  sensors_event_t a, g, t_mpu;
  mpu.getEvent(&a, &g, &t_mpu);

  float accelX     = a.acceleration.x;
  float accelY     = a.acceleration.y;
  float accelZ     = a.acceleration.z;

  Serial.println(String((accelX),2) + " " + String((accelY),2) + " " + String((accelZ),2));

  String msg = String(time_ms) + ";" +
               String(T, 1) + ";" +
               String(P, 1) + ";" +
               String(accelX, 2) + ";" +
               String(accelY, 2) + ";" +
               String(accelZ, 2) + ";";

  
  const uint8_t MAXLEN = 60;
  if (msg.length() >= MAXLEN) {
    
    msg = String(time_ms) + ";" +
          String(T, 1) + ";" +
          String(P, 0) + ";" +
          String(accelX, 1) + ";" +
          String(accelY, 1) + ";" +
          String(accelZ, 1) + ";\n";
  }

  
  char radiopacket[MAXLEN];
  size_t nCopy = min((size_t)(MAXLEN - 1), (size_t)msg.length());
  msg.toCharArray(radiopacket, nCopy + 1); 

  Serial.print("Sending: ");
  Serial.print(radiopacket); 

  rf69.send((uint8_t*)radiopacket, (uint8_t)strlen(radiopacket));
  rf69.waitPacketSent();


  uint8_t buf[60];
  uint8_t len = sizeof(buf);
  if (rf69.waitAvailableTimeout(500)) {
    if (rf69.recv(buf, &len)) {
      if (len < sizeof(buf)) buf[len] = '\0'; else buf[sizeof(buf) - 1] = '\0';
      Serial.print("Got a reply: ");
      Serial.println((char*)buf);
      Blink(LED, 40, 2);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is another RFM69 listening?");
  }

  delay(500); // ~10 Hz
}
