<h1>Omnidirectional ros serial communication test</h1>

This Code connects a ROS node via a rosserial node to an arduino.
The python program will connect to ros and receive a "sensor" message, modify it a bit and publish it in another topic
The rosserial node manages the serial communication between ros and the arduino
The python code was not inserted in a ros package but is just a standalone script.

Do those tutorials first:
* http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

To use the code:
*  Load the ardu_ros_serial_test.ino to an Arduino
*  Atach an LED with a fitting resistor to pin 10. To just test the serial connection, you can also use LED_BUILTIN (13) which can't do analogWrite)
*  Attach an analog sensor to pin A0 like the photoresistor in this tutorial https://www.arduino.cc/en/Tutorial/AnalogInput ( with 10kOhm resistor between pin A0 and Ground). I used a force resistive sensor (fsr).
*  Start roscore
*  Start rosserial with: rosrun rosserial_python serial_node.py /dev/ttyUSB0
*  Run the python script with python2: python ros_serial_test.py

The LED should be dimmed in respect to the resistance of the analog sensor. In my example, the LED is brighter, the more force is on the sensor.
