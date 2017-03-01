<h1>Blinky ROS Serial</h1>

This Code connects a ROS node via a rosserial node to an arduino.
The python program will connect to ros and send a empty message with the topic /comTest every second.
The rosserial node sends it to the arduino and there the code subscribes to it.
The python code was not inserted in a ros package but is just a standalone script.

Do those tutorials first:
* http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

To use the code:
*  Load the ardu_ros_serial_test.ino to an Arduino
*  Atach an LED with a fitting resistor to pin 10. To just test the serial connection, you can also use LED_BUILTIN (13) which can't do analogWrite)
*  Start roscore
*  Start rosserial with: rosrun rosserial_python serial_node.py /dev/ttyUSB0
*  Run the python script with python2: python ros_serial_test.py

The LED on Pin 10 should Dim from dark to bright and back.
