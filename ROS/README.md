This folder contains projects, which connect an Arduino via [ROS](http://www.ros.org/) and it's [rosserial](http://wiki.ros.org/rosserial_Arduino) node to a python program. The goal is to control a robot with the Arduino and have a fast sensorimotor loop. The python program should do online learning on the sensor and motor data and provide new data to the Arduino. This loop does not have to be as fast.

The ROS code is not put into a node but can be executed without catkin or building anything.

All Projects use the structure of [PlatformIO](http://platformio.org/) in the [Atom](http://atom.io) editor since in this way, Python and Arduino code can be coded and executed in the same editor.
