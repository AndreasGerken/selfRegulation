/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

#define LED_PIN LED_BUILTIN
ros::NodeHandle nh;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_PIN, HIGH-digitalRead(LED_PIN));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("comTest", &messageCb );

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
