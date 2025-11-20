#include <TinyGPSPlus.h>

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Startar GPS-test...");
  Serial1.begin(9600);
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
    gps.encode(c);
  }

  static unsigned long last = 0;
  if (millis() - last > 2000) {
    last = millis();
    if (gps.location.isValid()) {
      Serial.println();
      Serial.println("=== POSITION ===");
      Serial.print("Latitud:  ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitud: ");
      Serial.println(gps.location.lng(), 6);
      Serial.println("===============");
    } else {
      Serial.println();
    }
  }
}
