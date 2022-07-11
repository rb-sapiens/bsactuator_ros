# bsactuator
ROS package for manipulating Bambooshoot Actuator.

## How to execute launch file
```
roslaunch bamboobot bamboobot.launch sudopass:="your sudo password" --screen
```

## Subscribers
#### /set_length
Sets the length of BambooshootActuator.

ex.
```
rostopic pub -1 /set_length std_msgs/Int16 300
```

## Publishers
#### /length
Publishes the current length of BambooshootActuator.