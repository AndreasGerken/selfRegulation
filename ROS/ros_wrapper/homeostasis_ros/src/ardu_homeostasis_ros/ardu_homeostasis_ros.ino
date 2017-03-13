// FIXME: WORK IN PROGESS!

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif



// Typedefinitions
#define Float32 float // Floats on Arduino have 32 bits

// General variables
#define dim_m 4 // Legs
#define dim_s 6 // Gyro acceleration and rotation

Float32 motorValues[dim_m];
Float32 sensorValues[dim_s];

// ROS variables
ros::NodeHandle nh;
std_msgs::Float32MultiArray messageSensors;
int16_t seq;

// MPU6050 variables
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Servo variables
int16_t servoPin[dim_m] = {5, 6, 9, 10};
bool servoRevert[dim_m] = {false, false, true, true};
Servo   servos[dim_m];
//Servo servo9;


void receiveMessage( const std_msgs::Float32MultiArray message){

}

ros::Subscriber<std_msgs::Float32MultiArray> sub("homeostasis_motor", &receiveMessage);
ros::Publisher pub("homeostasis_sensor", &messageSensors);

void writeServoPosition(int servoIndex, int position){
  int realPosition = position;

  if(servoRevert[servoIndex]){
    realPosition = 180 - position;
  }

  servos[servoIndex].write(realPosition);
}

void readSensorValues(){
  // read sensors

  // MPU 6050
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  sensorValues[0]= ax;
  sensorValues[1]= ay;
  sensorValues[2]= az;
  sensorValues[3]= gx;
  sensorValues[4]= gy;
  sensorValues[5]= gz;
}

void sendSensorValues(){
  messageSensors.data = sensorValues;
  //messageSensors.layout.dim_length = 1;
  messageSensors.data_length = dim_s;
  pub.publish(&messageSensors);
}

void setup(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  nh.initNode();

  nh.subscribe(sub);
  nh.advertise(pub);

  // Initialize MPU6050
  accelgyro.initialize();

  seq = 0;

  //while(!nh.connected()) nh.spinOnce();
  nh.loginfo("ROS Startup complete");

  nh.spinOnce();

  // MPU 6050 initialization
  nh.loginfo(accelgyro.testConnection() ? "MPU6050 connection successfull" : "MPU6050 connection failed");

  // Attach Servos
  for(int i = 0; i < dim_m; i++){
    servos[i].attach(servoPin[i]);
  }
}

void loop(){



  for(int angle = 60; angle < 120; angle++){
    //readSensorValues();

    //sendSensorValues();

    for(int i = 0; i < dim_m; i++){
      writeServoPosition(i, angle);
    }

    nh.spinOnce();
    delay(20);
  }

  for(int angle = 120; angle > 60; angle--){
    //readSensorValues();

    //sendSensorValues();

    for(int i = 0; i < dim_m; i++){
      writeServoPosition(i, angle);
    }

    nh.spinOnce();
    delay(5);
  }
}
