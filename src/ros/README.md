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



