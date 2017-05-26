// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;
double ax_m, ay_m, az_m, gx_m, gy_m, gz_m;
double accel_alpha = 0.7; // Experimental Values
double gyro_alpha = 0.7;

#include <ros.h>
#include <ros/time.h>
#include <tiny_msgs/tinyVector.h>
#include <tiny_msgs/tinyIMU.h>
#include <std_msgs/Float32MultiArray.h>

#include <Servo.h>

// General variables
#define dim_m 4 // Legs
#define dim_s 6 // Gyro acceleration and rotation

#define servoMin 50
#define servoMax 130


// Prototypes
void writeServoPosition(int servoIndex, int position);
void receiveMessage( const std_msgs::Float32MultiArray& message);
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max);

ros::NodeHandle  nh;

float servoPos[dim_m] = {90, 90, 90, 90};

tiny_msgs::tinyIMU imu_msg;
ros::Publisher imu_pub("tinyImu", &imu_msg);

ros::Subscriber<std_msgs::Float32MultiArray> subscriber("homeostasis_motor", receiveMessage);

uint32_t seq;
unsigned long currentMillis;

unsigned long imuPrevious = 0;
unsigned long publishPrevious = 0;

unsigned long imuInterval = 5;
unsigned long publishInterval = 50;

// Servo variables
int16_t servoPin[dim_m] = {5, 6, 9, 10};
bool servoRevert[dim_m] = {true, false, true, false};
int16_t servoOffset[dim_m] = {0, 0, 0, 0};
Servo   servos[dim_m];

int16_t lastReceivedMessage;


void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  nh.initNode();
  nh.advertise(imu_pub);
  nh.subscribe(subscriber);
  
  nh.loginfo("ROS Setup finished, starting MPU6050");
  
  accelgyro.initialize();

  // MPU 6050 initialization
  nh.loginfo(accelgyro.testConnection() ? "MPU6050 connection successfull" : "MPU6050 connection failed");

  seq = 0;

  // Attach Servos
  for(int i = 0; i < dim_m; i++){
    servos[i].attach(servoPin[i]);
    servos[i].write(servoPos[i]);
  }

  // init last received message
  lastReceivedMessage =0;
}

void loop()
{
  currentMillis = millis();

  if(currentMillis - publishPrevious >= publishInterval){
    seq ++;
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = 0;
    imu_msg.header.seq = seq;
  
    imu_msg.accel.x = (int16_t)ax_m;
    imu_msg.accel.y = (int16_t)ay_m;
    imu_msg.accel.z = (int16_t)az_m;
    imu_msg.gyro.x = (int16_t)gx_m;
    imu_msg.gyro.y = (int16_t)gy_m;
    imu_msg.gyro.z = (int16_t)gz_m; //mapfloat(gz, -32768, 32768, 0, 1);
  
    imu_pub.publish( &imu_msg );
  
    nh.spinOnce();
  
    // write Servos
    for(int i = 0; i < dim_m; i++){
      writeServoPosition(i, servoPos[i]);
    }
    
    publishPrevious = currentMillis;
    
  }else if(currentMillis - imuPrevious >= imuInterval) {    
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    ax_m = (double)ax * accel_alpha + ax_m * (1-accel_alpha);
    ay_m = (double)ay * accel_alpha + ay_m * (1-accel_alpha);
    az_m = (double)az * accel_alpha + az_m * (1-accel_alpha);
    gx_m = (double)gx * accel_alpha + gx_m * (1-gyro_alpha);
    gy_m = (double)gy * accel_alpha + gy_m * (1-gyro_alpha);
    gz_m = (double)gz * accel_alpha + gz_m * (1-gyro_alpha);

    imuPrevious = currentMillis;
  }
}


void writeServoPosition(int servoIndex, int position){
  int realPosition = position + servoOffset[servoIndex];

  if(servoRevert[servoIndex]){
    realPosition = 180 - realPosition;
  }

  servos[servoIndex].write(realPosition);
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
 if( x > in_max) x = in_max;
 if (x < in_min) x = in_min;
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}


void receiveMessage( const std_msgs::Float32MultiArray& message){
  lastReceivedMessage = millis();
  servoPos[0] = mapfloat(message.data[0], -32768, 32768, servoMin, servoMax);;
  servoPos[1] = mapfloat(message.data[1], -32768, 32768, servoMin, servoMax);;
  servoPos[2] = mapfloat(message.data[2], -32768, 32768, servoMin, servoMax);;
  servoPos[3] = mapfloat(message.data[3], -32768, 32768, servoMin, servoMax);;
}

