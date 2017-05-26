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

#include <Servo.h>

#include <ros.h>
#include <std_msgs/Int32.h>

// General variables
#define dim_m 4 // Legs
#define dim_s 6 // Gyro acceleration and rotation

#define servoMin 50
#define servoMax 130


// Prototypes
void writeServoPosition(int servoIndex, int position);
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max);

float servoPos[dim_m] = {90, 90, 90, 90};
float sin_speed = 0.5;
float sin_phase = 0;

uint32_t seq;

// Servo variables
int16_t servoPin[dim_m] = {5, 6, 9, 10};
bool servoRevert[dim_m] = {true, false, true, false};
int16_t servoOffset[dim_m] = {0, 0, 0, 0};
Servo   servos[dim_m];


void messageCb( const std_msgs::Int32& messageSub){
    sin_phase += sin_speed;
    
    
    // write Servos
    for(int i = 0; i < dim_m; i++){

      servoPos[i] = (int)(sin(sin_phase) * 5. + 115.);
      //Serial.println(sin(sin_phase));
      writeServoPosition(i, servoPos[i]);
    }
    //sin_speed += 0.0005;
}

ros::Subscriber<std_msgs::Int32> sub("comTestPub", &messageCb );
ros::NodeHandle nh;


void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  accelgyro.initialize();

  Serial.begin(9600);
  // MPU 6050 initialization
  Serial.print(accelgyro.testConnection() ? "MPU6050 connection successfull" : "MPU6050 connection failed");
  nh.initNode();
  nh.subscribe(sub);
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

  ax_m = (double)ax * accel_alpha + ax_m * (1-accel_alpha);
  ay_m = (double)ay * accel_alpha + ay_m * (1-accel_alpha);
  az_m = (double)az * accel_alpha + az_m * (1-accel_alpha);
  gx_m = (double)gx * accel_alpha + gx_m * (1-gyro_alpha);
  gy_m = (double)gy * accel_alpha + gy_m * (1-gyro_alpha);
  gz_m = (double)gz * accel_alpha + gz_m * (1-gyro_alpha);
  
  if(seq % 10 == 0){
    Serial.print((int16_t)ax_m);
    Serial.print("\t");
    Serial.print((int16_t)ay_m);
    Serial.print("\t");
    Serial.print((int16_t)az_m);
    Serial.print("\t");
    Serial.print((int16_t)gx_m);
    Serial.print("\t");
    Serial.print((int16_t)gy_m);
    Serial.print("\t");
    Serial.print((int16_t)gz_m); //mapfloat(gz, -32768, 32768, 0, 1);
    Serial.print("\t");
    Serial.print(sin_speed);
    Serial.print("\n");


    
  }
  nh.spinOnce();
  // results in ~20Hz publishing
  delay(3);

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
