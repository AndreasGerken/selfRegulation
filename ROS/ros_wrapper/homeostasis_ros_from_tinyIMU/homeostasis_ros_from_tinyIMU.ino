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

#include <ros.h>
#include <ros/time.h>
#include <tiny_msgs/tinyVector.h>
#include <tiny_msgs/tinyIMU.h>
#include <std_msgs/Float32MultiArray.h>

#include <Servo.h>

// General variables
#define dim_m 4 // Legs
#define dim_s 6 // Gyro acceleration and rotation

#define servoMin 60
#define servoMax 110


// Prototypes
void writeServoPosition(int servoIndex, int position);
void receiveMessage( const std_msgs::Float32MultiArray& message);

ros::NodeHandle  nh;

int servoPos[dim_m] = {90, 90, 90, 90};
int servoDir = 1;

tiny_msgs::tinyIMU imu_msg;
ros::Publisher imu_pub("tinyImu", &imu_msg);

ros::Subscriber<std_msgs::Float32MultiArray> subscriber("homeostasis_motor", receiveMessage);

uint32_t seq;

// Servo variables
int16_t servoPin[dim_m] = {5, 6, 9, 10};
bool servoRevert[dim_m] = {false, false, true, true};
Servo   servos[dim_m];


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
}

void loop()
{
  seq++;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = 0;
  imu_msg.header.seq = seq;

  imu_msg.accel.x = ax;
  imu_msg.accel.y = ay;
  imu_msg.accel.z = az;
  imu_msg.gyro.x = gx;
  imu_msg.gyro.y = gy;
  imu_msg.gyro.z = gz;

  imu_pub.publish( &imu_msg );

  nh.spinOnce();
  
  // might be enough time for the answer and the callback
  delay(10);

  if (servoPos[0] > servoMax || servoPos[0] < servoMin) servoDir *= -1;

  // Attach Servos
  for(int i = 0; i < dim_m; i++){
    servoPos[i] += servoDir;
    writeServoPosition(i, servoPos[i]);
  }


}


void writeServoPosition(int servoIndex, int position){
  int realPosition = position;

  if(servoRevert[servoIndex]){
    realPosition = 180 - position;
  }

  servos[servoIndex].write(realPosition);
}

void receiveMessage( const std_msgs::Float32MultiArray& message){
  servoPos[0] = message.data[0];
  servoPos[1] = message.data[1];
  servoPos[2] = message.data[2];
  servoPos[3] = message.data[3];  
}
