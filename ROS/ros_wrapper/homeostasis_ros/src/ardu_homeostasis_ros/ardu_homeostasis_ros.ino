// FIXME: WORK IN PROGESS!

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#include "Wire.h"
#include "MPU6050.h"
#include "Servo.h"

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

// MPU6050 variables
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Servo variables
int16_t servoPin[dim_m] = {5, 6, 9, 10}
Servo   servos[dim_m];


void receiveMessage( const std_msgs::Float32MultiArray message){

}

ros::Subscriber<std_msgs::Float32MultiArray> sub("homeostasis_motor", &receiveMessage);
ros::Publisher pub("homeostasis_sensor", &messageSensor);

void setup(){
  // ROS Initialization
  nh.initNode();

  nh.subscribe(sub);
  nh.advertise(pub);

  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("ROS Startup complete");

  // MPU 6050 initialization
  Wire.begin();

  // Attach Servos
  for(int i = 0; i < dim_m; i++){
    servos[i].attach(servoPin[dim_m]);
  }
}

void loop(){

}
