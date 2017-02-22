/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int32.h>

#define LED_PIN 10
ros::NodeHandle nh;

void messageCb( const std_msgs::Int32& valueMsg){
  analogWrite(LED_PIN, (int)valueMsg.data);   // blink the led
}

ros::Subscriber<std_msgs::Int32> sub("comTestPub", &messageCb );

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
