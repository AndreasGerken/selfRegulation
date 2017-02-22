Blinky ROS Serial:

This Code connects a ROS node via a rosserial node to an arduino.
The python program will connect to ros and send a empty message with the topic /comTest every second.
The rosserial node sends it to the arduino and there the code subscribes to it.

Do those tutorials first:
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

To use the code:
1:  Load the ardu_ros_serial_test.ino to an Arduino
2:  Start roscore
3:  Start rosserial with
    rosrun rosserial_python serial_node.py /dev/ttyUSB0
4:  Run the python script with python2
    python ros_serial_test.py

The Builtin LED should toggle with a frequency of 1Hz
