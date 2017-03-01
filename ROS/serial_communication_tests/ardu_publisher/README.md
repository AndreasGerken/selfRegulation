<h1>Arduino as Publisher</h1>

This code example is from [rosserial- HelloWorld](http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World)

Upload the sketch to an Arduino and run in three consoles

* `roscore`
* `rosrun rosserial_python serial_node.py /dev/ttyUSB0`
* `rostopic echo chatter`


It should print HelloWorld with 1Hz.
