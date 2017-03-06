<h1>ROS Wrapper</h1>

The ROS wrappers add the ros layer to some python classes. The wrapped classes can be called without building them with catkin and are not in any package.

<h2>homeostasis_ros</h2>
Wrapper for homeostasis_c which subscribes to sensor messages and publishes motor messages. To test return in multiple terminals (folder homeostasis_ros/src)

* `roscore`
* `python homeostasis_ros.py -v`
* `rostopic echo /homeostasis_motor`
* ```
rostopic pub /homeostasis_sensor std_msgs/Float32MultiArray "layout:
    dim: []
    data_offset: 0
data: [0.0, 1.0]"```

The python program should print, that it received a sensor message and that it sent a motor message. The motor message should pop up in the rostopic echo terminal.

Example output:
```
initialization finished
SensorData received: 19:12:32
[[ 0.]
 [ 1.]]
MotorData sent: 19:12:32
[-0.05436079474147093]
```
