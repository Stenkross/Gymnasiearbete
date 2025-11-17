//Kod för markstation
//Måste lägga till SDkortet här och ha olika cs-pins. Just nu är antennen på 1 och SDkortet på 1

#include <SPI.h>
#include <RH_RF69.h>
#include <SD.h>
#define RF69_FREQ 433.0
#define RFM69_CS   1 
#define RFM69_INT  24 
#define RFM69_RST  25 
#define LED        11

RH_RF69 rf69(RFM69_CS, RFM69_INT);
const int chipSelect = 9;
File myFile;
int16_t packetnum = 0;  

void setup() {
  pinMode(chipSelect,OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  pinMode(RFM69_INT,OUTPUT);
  pinMode(RFM69_CS,OUTPUT);

  Serial.begin(115200);
  while (!Serial) delay(1); 

  digitalWrite(chipSelect,HIGH);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }

  Serial.println("RFM69 radio init OK!");

  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf69.setTxPower(20, true); 
  rf69.setModemConfig(RH_RF69::FSK_Rb9_6Fd19_2); // kanske ha kvar

  //uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
  //                  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  //rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");


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
}

void loop() {
  //Ta bort för transmitter/satellit och välj delay på receiver/groundstation
  delay(500);

  char radiopacket[20] = "Hello World";
  Serial.print("Sending "); Serial.println(radiopacket);

  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf69.waitPacketSent();

  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf69.waitAvailableTimeout(500)) {
    if (rf69.recv(buf, &len)) {
      Serial.print("Got a reply: ");
      Serial.println((char*)buf);
      digitalWrite(RFM69_CS,HIGH);
      digitalWrite(chipSelect,LOW);
      myFile = SD.open("example.txt", FILE_WRITE);
      myFile.println((char*)buf);
      myFile.close();
      digitalWrite(RFM69_CS,LOW);
      digitalWrite(chipSelect,HIGH);
      Blink(LED, 50, 3); // blink LED 3 times, 50ms between blinks
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is another RFM69 listening?");
  }
}

void Blink(byte pin, byte delay_ms, byte loops) {
  while (loops--) {
    digitalWrite(pin, HIGH);
    delay(delay_ms);
    digitalWrite(pin, LOW);
    delay(delay_ms);
  }
}
