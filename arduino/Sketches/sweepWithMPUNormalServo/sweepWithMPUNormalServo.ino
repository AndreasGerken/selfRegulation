#include "Wire.h"
#include "MPU6050.h"
#include "Servo.h"

MPU6050 accelgyro;
Servo servo;

bool ledMode = false;
int LED_PIN = 13;

float servoPos = 90;
float servoDir = 1;

long lastUpdateMillis = 0;
long interval = 10;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

void setup() {
  // put your setup code here, to run once:
  Wire.begin();

  Serial.begin(115200);

  pinMode(LED_PIN,OUTPUT);

  servo.attach(4);
  servo.write(90);
  delay(200);
  
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  Serial.println("Testing device connections");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successfull" : "MPU6050 connection failed");

  Serial.println("Updating internal sensor offsets...");
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0

  accelgyro.setXGyroOffset(accelgyro.getXAccelOffset());
  accelgyro.setYGyroOffset(accelgyro.getYAccelOffset());
  accelgyro.setZGyroOffset(accelgyro.getZAccelOffset());  
}

void loop() {
  
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz); Serial.print("\t");
  Serial.print(servoPos); Serial.print("\t");
  Serial.println(servoDir);

  if(lastUpdateMillis + interval < millis()){
    lastUpdateMillis = millis();

    ledMode = !ledMode;
    digitalWrite(LED_PIN, ledMode);

    if(servoPos >= 170.0 || servoPos <= 10.0){
      servoDir *= float(random(5,20)) / -10.0;

      if(abs(servoDir) > 2){
        servoDir *= 0.7;
      }
      if(abs(servoDir) < 0.2){
        servoDir *= 1.3;
      }
      
      while(servoPos >= 170 || servoPos <= 10.0){
        servoPos += servoDir;
      }
    }
    servoPos += servoDir;
    
    servo.write(servoPos);
  }
  
}
