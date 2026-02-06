float pi = 3.141592653;

float startx = 0;
float starty = 0;
float oldcoordx = 0;
float oldcoordy = -1;
float coordx = -1;
float coordy = 0;
float x;
float y;

float speedx;
float speedy;

float anglefromposition;
float anglefromspeed;
float angleturn;

void setup() {
  Serial.begin(9600);
  while(!Serial){}

  x = coordx - startx;
  y = coordy - starty;

  speedx = coordx - oldcoordx;
  speedy = coordy - oldcoordy;

  if(x<0)
  {
    anglefromposition = atan(y/x) * (180/pi);
  }
  else
  {
    anglefromposition = atan(y/x) * (180/pi) + 180;
  }

  if(speedx<0){
    anglefromspeed = atan(speedy/speedx) * (180/pi) + 180;
  }
  else
  {
    anglefromspeed = atan(speedy/speedx) * (180/pi);
  }

  angleturn = anglefromspeed - anglefromposition;

  if (angleturn < -180) {
    angleturn += 360;
  }
  else if (angleturn > 180) 
  {
    angleturn -= 360;
  }

  Serial.println(String(x) + " " + String(y) + " " + String(speedx) + " " + String(speedy));
  Serial.println(String(angleturn) + " " + String(anglefromposition) + " " + String(anglefromspeed));

}

void loop() {
  // put your main code here, to run repeatedly:

}
