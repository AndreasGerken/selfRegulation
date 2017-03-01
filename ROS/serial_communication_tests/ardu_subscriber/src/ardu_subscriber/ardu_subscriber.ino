#include <ros.h>
#include <std_msgs/Int32.h>

#define LED_PIN 10
ros::NodeHandle nh;

void messageCb( const std_msgs::Int32& messageSub){
  analogWrite(LED_PIN, (int)messageSub.data);
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
