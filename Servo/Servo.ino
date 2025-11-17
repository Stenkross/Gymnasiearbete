#include <Servo.h>

Servo myservo; 

void setup() {
  //Sätter på servon. Kommer göra att servoarmen börjar snurra även om man inte har en myservo.write()
  myservo.attach(11); 

  //Graderna 0-26: Roterar som klockan där 0 är snabbast och 26 är långsammast
  //Graderna 27-37: Servon är stilla
  //Graderna 38-180: Roterar motsatt klockan där 180 är snabbast och 38 är långsammast
  myservo.write(180); 
  delay(5000);

  //Stänger av servon
  myservo.detach();
}

void loop() { 
  
}