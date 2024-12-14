# ROS 2 Package Installation and Usage Instructions
__ README.md is intended to be a high level overview of our project. This doucment is intended as a more technical documentation for convenience and instruction of the user__
## Initial Setup
Before we get to any instruction/recommendations for using our code, we need to clone the repo in ros2_ws/src. 

Firstly, follow the instructions to build a ucsd_robocar Docker container given in [this document](https://docs.google.com/document/d/1Onft0sIWhEd9UH7fItJ0atC1hKnxpTxKKmUFHtqC-sA/edit?tab=t.0).

In the Docker container:
```bash
source_ros2
cd src
git clone https://github.com/UCSD-ECEMAE-148/fall-2024-final-project-team-1
```
This will provide you with both our improved lane_detection_node2.py, and with our ball_vision_package.

## Lane Detection Setup
lane_detection_node2.py is our modified lane detection node. It was not utilized in our final project, but is being submitted together with it.

Due to extreme latency in our camera (for an unknown reason), we completed the lane following assignment by modifying the lane detection node with custom error handling. When no lane is detected, the previous known error value is published.

In order to not modify code within the UCSD Robocar Hub in the ROS 2 Workspace, move this file into the lane detection package:

In your terminal (assuming you have cloned our repo in the ros2_ws/src directory):
```bash
mv lane_detection_node2.py ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg
cd ucsd_robocar_hub2/ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg
mv lane_detection_node.py lane_detection_node3.py  # renames original node to lane_detection_node3.py
mv lane_detection_node2.py lane_detection_node.py  # renames new node to lane_detection_node.py
build_ros2
```

Now, when launching ucsd_robocar_nav2_pkg all_nodes.launch.py, you will use our new node with error handling capabilities. To reverse this change, reverse the filename changes.

## Ball Vision Package Instruction:
__Note: Our package uses GUI features. As such, you must enable X forwarding on the Jetson, but outside the docker container. Before entering the container, type ```xhost +``` in your terminal. You may need to change the value of the display variable in the container, using ```export DISPLAY=localhost:XX.0```, where ```XX``` is some 2 digit number.__

__ANOTHER IMPORTANT NOTE: You must clone the ```depthai-python``` repository in order to use the YOLO model upon which our tracking node is built.__
In your Docker container's terminal, and in the ```projects``` directory:
```bash
git clone https://github.com/luxonis/depthai-python
```

Before launching the package, one change must be made to the VESC twist node

In your terminal:
```bash
gedit uscd_robocar_hub2/uscd_robocar_actuator2_pkg/uscd_robocar_actuator2_pkg/vesc_twist_node.py
```
In gedit, change the value of  self.default_straight_steering from 0.5 to 1.0:
```
 self.default_straight_steering = 1.0
 ``` 
 Now, compile this change:
 ```
 build_ros2
 ```

We're ready to launch the ```ball_vision-pkg``` now! After running ```build_ros2```, you shoule be back in ```ros2_ws```.

In your terminal:
```
ros2 launch ball_vision_package ball_tracking.launch.py
```

If you'd like to change performance characteristics:

Increasing the parameter passed to ```spatialDetectionNetwork.setConfidenceThreshold()``` will make it more difficult to track a ball, but will reduce the chance of false positives. Decreasing the parameter will have the opposite effect. By default, it is set to 0.05.

Upon measuring consecutive, decreasing ball depth values, the node will publish a throttle "burst." The variable  ```self.MAX_THROTTLE_DURATION``` controls the duration of this burst. The variable ```self.frames_required``` determines how many consecutive frames of decreasing depth are required to trigger a throttle. ```self.movement_tolerance``` determines the amount by which depth must decrease in a frame for that frame to be registered as a "decreasing" frame.

The variable ```throttle``` is passed a float. We did not have a chance to calibrate it, but it is a multiplicative coefficient to the default rpm value found in ```vesc_twist_node.py```. A value of ```5.0``` is a moderate speed, capable of deflecting slow or moderately fast kicks. We have tested it in a range from ```3.0``` to ```10.0```. The variable ```Kp``` is a proportional control term, multiplying the measured angle between the ball and car before it is published as ```cmd_vel_msg.angular.z```.

Compile any changes you make by running the following command in ```ros2_ws```
```bash
colcon build --packages-select ball_vision_package
```


