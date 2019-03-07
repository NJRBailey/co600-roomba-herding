# ROS

Python code relating to the ROS module

* Receive data from drone
* Send data to drone

## Module descriptions:

### keyboardTeleop

```keyboardTeleop``` allows for control of the drone with the keyboard. When run ```keyboardTeleop``` overides the keyboard, in order to end the process ```esc``` must be pressed.

#### Controls

* Up - move drone forward
* Down - move drone backwards
* Left - move drone left
* Right - move drone right
* U - move drone up
* J - move drone down
* H - rotate drone left
* L - rotate drone right
* Enter - takeoff drone
* Backspace - land drone
* Esc - Exit process and land drone

### movementApi

```movementApi``` initialises a single ROS message publisher, and can be called to send movement commands to the drone.

### hover

```hover``` ensures the drone stays at a set hieght, and will hover over any marker it is able to identify via the drone's camera feed.

### NavdataCoordinator

```NavdataCoordinator``` subscribes to the navdata messages published by the drone. This module contains a ROS service ```HieghtOffset``` that when called returns the amount the drone is offset from the hieght we want it to maintain. __NOTE:__ A guide to calling ros services is at the bottom of this document.

### ImageSub

```ImageSub``` subscribes to the image messages published by the drone. This module contains a ROS service ```GetLatestImage```that when called returns the latest image recieved from the drone. __NOTE:__ A guide to calling ros services is at the bottom of this document.

## Calling ROS services:

The following code must be included in order to call ROS services:

* Both rospy and the service message definitions must be imported:

```
import rospy
from co600_proj.srv import HeightOffset, GetLatestImage
```

* Wait for the services to be initialised/started:

```
rospy.wait_for_service('height_offset_srv')
```

* Declare service:

```
self.HeightOffsetSrv = rospy.ServiceProxy('height_offset_srv', HieghtOffset)
```

* Call service:

```
response = self.HeightOffsetSrv()
```

## Running ROS:

In order to run the ROS project ROS kinetic must be installed.

* In a new terminal window use ```roscore``` to launch ROS

* In a new terminal either use ```roslaunch cvg_sim_gazebo ardrone_4x4invert.launch ``` to launch the simulated world or use ```rosrun ardrone_autonomy ardrone_driver```to launch the ardrone driver (must be connected to ardrone wifi)

* In a new terminal use ```rosrun co600_proj ImageSub.py``` to launch the image subscriber service.

* In a new terminal use ```rosrun co600_proj NavdataCoordinator.py``` to launch the navdata subscriber service.

* In the final terminal window use ```rosrun co600_proj master.py``` to launch the master node, this will wait 5 seconds, then start the search and return process.

* If you want to overide the drone with the keyboard use ```python keyboardTeleop```.