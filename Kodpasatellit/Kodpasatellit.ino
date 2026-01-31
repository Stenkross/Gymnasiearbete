//Kod för satelliten. Skicka alla mätvärden till Groundstation.
//Cs-pin för antennen: 1 och SDkort: 9

#include <SPI.h>
#include <RH_RF69.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <SFE_BMP180.h>
#include <TinyGPSPlus.h>

#define RF69_FREQ   868.0
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


//Vilken Error kod
enum ErrorCode {
  ERROR_RADIO = 2,
  ERROR_SD    = 3,
  ERROR_BMP   = 4
};

void BigError(uint8_t code) {
  pinMode(LED, OUTPUT);

  digitalWrite(LED, LOW);

  while (true) {
    for (uint8_t i = 0; i < code; i++) {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
    }
    delay(3000);
  }
}

void okbeat() {
  static unsigned long lastBeat = 0;
  static bool inPulse = false;
  unsigned long now = millis();

  const unsigned long PERIOD = 5000;
  const unsigned long PULSE  = 800; 

  if (!inPulse) {
    if (now - lastBeat >= PERIOD) {
      inPulse = true;
      lastBeat = now;
      digitalWrite(LED, HIGH);
    }
  } else {
    if (now - lastBeat >= PULSE) {
      inPulse = false;
      digitalWrite(LED, LOW);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  delay(1000);

  //Börjar initiera radion.

  pinMode(chipSelect,OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(RFM69_RST, OUTPUT);
  pinMode(RFM69_INT, OUTPUT);
  pinMode(RFM69_CS, OUTPUT);

  digitalWrite(chipSelect,HIGH);
  digitalWrite(RFM69_CS, LOW);

  digitalWrite(RFM69_RST, HIGH); delay(10);
  digitalWrite(RFM69_RST, LOW);  delay(10);

  digitalWrite(LED, HIGH); delay(500);
  digitalWrite(LED, LOW);  delay(500);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    BigError(ERROR_RADIO);
  }
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }
  rf69.setTxPower(20, true);
  Serial.print("RFM69 @ "); Serial.print(RF69_FREQ); Serial.println(" MHz");

  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x03, 0x02, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  rf69.setModemConfig(RH_RF69::FSK_Rb9_6Fd19_2); // kanske ha kva

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
    BigError(ERROR_SD);
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
    BigError(ERROR_BMP);
  }

  Serial.println("Setup klar.");
// lys i början och i slutet av setup så vet man om något gick fel i. blinka 10 gånger och sedan delay
}

void loop() {
  //delay(100);

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

  double gpsAlt = NAN;
  if (gps.altitude.isValid()) {
    gpsAlt = gps.altitude.meters();
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
               String(lng ,5) + ";" +
               String(gpsAlt, 1) + "\n";

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
    // Format: tid;T;P;lat;lng;alt;
    msg = String(time_ms) + ";" +
          String(T, 1) + ";" +
          String(P, 0) + ";" +
          String(lat ,3) + ";" +
          String(lng ,3) + ";" +
          String(gpsAlt, 0) + "\n";
          
  }

  //Skicka det till markstationen via radion.
  
  char radiopacket[MAXLEN];
  size_t nCopy = min((size_t)(MAXLEN - 1), (size_t)msg.length());
  msg.toCharArray(radiopacket, nCopy + 1); 

  Serial.print("Sending: ");
  Serial.print(radiopacket); 

  rf69.send((uint8_t*)radiopacket, (uint8_t)strlen(radiopacket));
  rf69.waitPacketSent();

  okbeat();
}