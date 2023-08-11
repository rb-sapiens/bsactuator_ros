# bsactuator
ROS package for manipulating Bambooshoot Actuator.

## How to execute launch file
```
roslaunch bsactuator_ros bsactuator.launch sudopass:="your sudo password" --screen
```

## Subscribers
#### /set_length
Sets the length of BambooshootActuator.

ex.
```
rostopic pub -1 /set_length std_msgs/Int16 300
```

#### /hold
Turn on the motor power.

ex.
```
rostopic pub -1 /hold std_msgs/String "data: 'true'"
```

#### /release
Turn off the motor power.

ex.
```
rostopic pub -1 /hold std_msgs/String "data: 'true'"
```

## Publishers
#### /length
Publishes the current length of BambooshootActuator.