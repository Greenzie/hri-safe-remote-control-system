## HRI Safety Sense
### Installing the ROS Node
This node is now compatible with ROS Melodic, and builds in catkin. It launches a wrapper node C Driver that 
exposes a serial interface to the VSC. 

Installation requires either this folder (named hri_safety_sense) or the cloned git repo "hri-safe-remote-control-system"
to be in the 'src' directory of your ROS workspace.

Build the package with either

```bash
catkin_make
``` 
or
```bash
catkin_build
```
if you have catkin tools installed. Install with: `sudo apt-get install python-catkin-tools`

## Launching the ROS Node
This package has an included launch file (located in hri_safety_sense/launch/) that can be used to launch
the node. The command to use this file is
```bash
roslaunch hri_safety_sense vsc_interface.launch safety_port:=${port} vsc_interface_enable:=${yN}
```

There are two arguments listed in the launch file:
* safety_port: the serial port that the device is on (default is /dev/ttyACM0)
* vsc_interface_enable: a predicate that stops the node from launching if set false (default is true).

Another method of launching the ROS node is by using the command:
```bash
rosrun hri_safety_sense safe_remote_control
```
Using the launch file is recommended, however, because it is setup to auto-restart the node should it die 
unexpectedly.

## ROS Topics:
Running this node exposes the following topics:
* /safety/emergency_stop

## Updates to Build System
As of July 2019, this node is updated to be compatible with catkin. 

The functionality of this package is also 
exported as a library that can be imported by other catkin packages in a ROS environment, enabling other nodes 
to talk to the VSC.

## Testing
This node has been tested with a point-to-point configuration with a wired EStop Button, as well as a Wireless EStop Device.

Further testing with multi-point configurations, and configurations with a SRC are needed.

####Authorship
In addition to [HRI/Fort Robotics](https://hriwiki.atlassian.net/wiki/spaces/DOC/overview) Robotics, this node was contributed to and maintained by [Jackson Stanhope](js@greenzie.co) with support from [Greenzie](www.greenzie.com).

    
