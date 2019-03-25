# Running our project

## Dependencies

* ROS kinetic and ROSpy must be installed.
* 'ardrone_autonomy' must be installed.
* A catkin workspace must be set up and configured correctly, and our simulator/ROS code should be in this directory.

## Running the project

1. Run 'roscore' to start ROS master node.
2. Run 'roslaunch cvg_sim_gazebo ardrone_4x4invert' to launch simulated world.
3. Run 'rosrun co600_proj NavdataCoordinator.py' to launch the 'rotation_srv' and 'height_srv' ROS services.
4. Run 'rosrun co600_proj ImageSub.py' to launch the 'image_srv' ROS service.
5. Run 'rosrun co600_proj roombaIdle.py' to launch the 'roomba_movement_srv' ROS service.
6. Run 'rosrun co600_proj master.py' to launch the master node of our project.