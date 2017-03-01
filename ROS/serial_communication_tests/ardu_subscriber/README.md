<h1>Arduino as subscriber</h1>

This Code connects a ROS node via a rosserial node to an Arduino.
The python program will connect to ros and send a int32 message with the topic /comTestPub with a sweeping value from 0 to 255.
The rosserial node sends it to the Arduino and there the code subscribes to it. The sent Value will be written analogously to the LED 10.

The Python code was not inserted in a ros package but is just a standalone script.

Do those tutorials first:
* http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* http://wiki.ros.org/rosserial_Arduino/Tutorials/Arduino%20IDE%20Setup

To use the code:
*  Load the ardu_subscriber.ino to an Arduino
*  Atach an LED with a fitting resistor to pin 10. To just test the serial connection, you can also use LED_BUILTIN (13) which can't do analogWrite.
* `roscore`
*  `rosrun rosserial_python serial_node.py /dev/ttyUSB0`
*  `python src/ardu_subscriber.py`

The LED on Pin 10 should Dim from dark to bright and back.
