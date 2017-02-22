int pinEnable = 10;
int pinRev1 = 7;
int pinRev2 = 8;

void setMotorSpeed(int mSpeed);

void setup() {
  // put your setup code here, to run once:
  pinMode(pinEnable,OUTPUT);
  pinMode(pinRev1,OUTPUT);
  pinMode(pinRev2,OUTPUT);
}

void loop() {
  for(int i = 0; i<255;i++){
      setMotorSpeed(i);
      delay(20);
  }

  for(int i = 255; i>-255;i--){
      setMotorSpeed(i);
      delay(20);
  }

  for(int i = -255; i<0;i++){
      setMotorSpeed(i);
      delay(20);
  }
}

void setMotorSpeed(int mSpeed){
  bool rev = mSpeed > 0;
  digitalWrite(pinRev1, rev);
  digitalWrite(pinRev2, !rev);
  analogWrite(pinEnable,abs(mSpeed));
}
