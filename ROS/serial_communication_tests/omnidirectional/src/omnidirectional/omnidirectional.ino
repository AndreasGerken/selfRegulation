#include <ros.h>
#include <std_msgs/Int32.h>

#define LED_PIN 10
#define SENSOR_PIN A0

ros::NodeHandle nh;

std_msgs::Int32 messagePub;

void messageCb( const std_msgs::Int32& messageSub){
  analogWrite(LED_PIN, (int)messageSub.data);
}

ros::Subscriber<std_msgs::Int32> sub("comTestPub", &messageCb );
ros::Publisher pub("comTestSub", &messagePub);

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  nh.initNode();

  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop()
{
  messagePub.data = analogRead(SENSOR_PIN);
  pub.publish(&messagePub);
  nh.spinOnce();
  delay(10);
}
