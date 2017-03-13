# include <ros.h>
//# include <std_msgs/Float32MultiArray.h>

//#define Float32 float // Floats on Arduino have 32 bits

ros::NodeHandle nh;
//std_msgs::Float32MultiArray messageSensors;

//void receiveMessage( const std_msgs::Float32MultiArray message){

//}

//ros::Subscriber<std_msgs::Float32MultiArray> sub("homeostasis_motor", &receiveMessage);
//ros::Publisher pub("homeostasis_sensor", &messageSensors);

void setup(){
  // ROS Initialization
  nh.initNode();

  //nh.subscribe(sub);
  //nh.advertise(pub);

  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("ROS Startup complete");
}

void loop(){
  nh.loginfo("Test");
  nh.spinOnce();
  delay(1000);
}
