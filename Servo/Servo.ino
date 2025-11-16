#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;

void setup() {
  myservo.attach(11); 
  delay(1000);
}

void loop() { 
  myservo.write(pos); 
  pos += 1;
}