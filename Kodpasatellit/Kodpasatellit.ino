//Kod för satelliten. Skicka alla mätvärden till Groundstation.
//Cs-pin för antennen: 1 och SDkort: 9

#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <SFE_BMP180.h>
#include <TinyGPSPlus.h>

#define RF69_FREQ   433.0
#define RFM69_CS    13
#define RFM69_INT   24
#define RFM69_RST   25
#define LED         10

RH_RF69 rf69(RFM69_CS, RFM69_INT);

SFE_BMP180 bmp180;
TinyGPSPlus gps;
File myFile;

const int chipSelect = 9;
int16_t packetnum = 0;  

void Blink(byte pin, byte delay_ms, byte loops) {
  while (loops--) { 
    digitalWrite(pin, HIGH); 
    delay(delay_ms); 
    digitalWrite(pin, LOW); 
    delay(delay_ms); 
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  while (!Serial) { delay(1); }

  //Börjar initiera radion.

  pinMode(chipSelect,OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  pinMode(RFM69_INT, OUTPUT);
  pinMode(RFM69_CS, OUTPUT);

  digitalWrite(chipSelect,HIGH);
  digitalWrite(RFM69_CS, LOW);

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

  //uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
  //                  0x01, 0x03, 0x02, 0x04, 0x05, 0x06, 0x07, 0x08};
  //rf69.setEncryptionKey(key);

  //rf69.setModemConfig(RH_RF69::FSK_Rb9_6Fd19_2); // kanske ha kva

  //Sen initieras SD-kortet

  digitalWrite(RFM69_CS,HIGH);
  digitalWrite(chipSelect,LOW);

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this serial monitor after fixing your issue!");
    while (1);
  }

  Serial.println("initialization done.");

  if (SD.exists("example.txt")) {
    Serial.println("example.txt exists. Removing it");
    SD.remove("example.txt");
    Serial.println("Creating example.txt...");
    myFile = SD.open("example.txt", FILE_WRITE);
    myFile.println("Start");
    myFile.close();
  } 

  digitalWrite(chipSelect,HIGH);

  //Sen initieras tryck och temperatur mätaren

  if (bmp180.begin()) {
    Serial.println("BMP180 OK");
  } else {
    Serial.println("BMP180 hittades inte");
  }

  Serial.println("Setup klar.");

}

void loop() {
  delay(100);

  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
    gps.encode(c);
  }

  //Tryck och temperatur

  double T,P;
  char status;
  bool success = false;

  status = bmp180.startTemperature();

  if (status != 0) {
    delay(status);
    status = bmp180.getTemperature(T); 
    if (status != 0) {
      status = bmp180.startPressure(3);
      if (status != 0) {
        delay(status);
        status = bmp180.getPressure(P,T);
      }
    }
  }

  //Tiden sen programmet startade?
  
  unsigned long time_ms = millis()/1000;

  //Gpsen

  double lat = NAN;
  double lng = NAN;

  if (gps.location.isValid())  { // använda && (gps.location.isUpdated())?
    lat = gps.location.lat();
    lng = gps.location.lng();
  }

  //Lägg ihop allt till meddelandet som ska skickas

  String msg = String(time_ms) + ";" +
               String(T, 1) + ";" +
               String(P, 1) + ";" +
               String(lat ,5) + ";" +
               String(lng ,5) + ";" +"\n";

  //Skriv det i SD-kortet som backup

  digitalWrite(RFM69_CS,HIGH);
  digitalWrite(chipSelect,LOW);
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.println(msg);
  myFile.close();
  digitalWrite(RFM69_CS,LOW);
  digitalWrite(chipSelect,HIGH);

  const uint8_t MAXLEN = 60;
  if (msg.length() >= MAXLEN) {
    // Format: tid;T;P;ax;ay;az;lat;lng;
    msg = String(time_ms) + ";" +
          String(T, 1) + ";" +
          String(P, 0) + ";" +
          String(lat ,3) + ";" +
          String(lng ,3) + ";" +"\n";
  }

  //Skicka det till markstationen via radion.
  
  char radiopacket[MAXLEN];
  size_t nCopy = min((size_t)(MAXLEN - 1), (size_t)msg.length());
  msg.toCharArray(radiopacket, nCopy + 1); 

  Serial.print("Sending: ");
  Serial.print(radiopacket); 

  rf69.send((uint8_t*)radiopacket, (uint8_t)strlen(radiopacket));
  rf69.waitPacketSent();
}
