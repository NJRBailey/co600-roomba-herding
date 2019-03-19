# Simulator versus Real World

As of sprint 7 we have decided to almost exclusively focus on developing for the simulated world instead of the real world. A quick comparison of the two environment can be found below, with more in depth reasons for this choice.

## Comparison

### Real world

* Minimal to no rotational drift of the drone.
* Only able to test drone outside for the majority of the project
    - Testing drone outside is weather dependent and time consuming.
* Massive drift in horizontal and vertical drift due to external factors and drone. (Ardrone 2 had significantly more drift than Tello drone).
* Image analysis significantly more complex, even with the noise reduction methods we applied.
* Much greater latency in information sent by drone to us.
* Can use the higher resolution (720p) drone camera via mirror.
* No suitable area to test arena and roomba code.
* Roomba can only be used inside
* Serial used to communicate with Roomba

### Simulator

* Massive rotational and vertical drift due to simulation calculation errors.
* Testing can be done at any time.
* No external influence on the drone, however simulation errors produce a similar effect.
* Simple environments mean less overhead for image analysis component.
* Can only use low resolution bottom camera (360p) from the drone.
* Roomba is controlled via ROS.
* Different ROS message queues are required to communicate with both the drone and roomba.
* A namespace has to be defined for each robot, to ensure that they do not interfere with each other.
* Easy to test and modify arena.

## Focusing on the simulator: why?

We have now decided to work exclusively on the simulator, there are a few major issues with developing for these two different environments that we have discovered as work has progressed: The work for each user story is significantly increased as the two environments present different problems (Examples of this are the different types of drift the drone experiences, or the complexity of the image analysis), our inability to test the drone and roomba in real life whenever we want meant that we get a significant amount of work done and by the time we get to test it with the drone we find many bugs that arent present in the simulator meaning we have to delay current work to fix those bugs, two versions of the code base have to be maintained as environment variables and other parts of the code are environment specific.
