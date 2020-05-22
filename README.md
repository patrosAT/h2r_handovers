# ROS node for real-time bodyparts detection #

This ROS node provides a driver for obejct-independent human-to-robot handovers using robotic vision. The approach requires only one RGBD camera and can therefore be used in a variety of use cases without the need for artificial setups like markers or external cameras. The object-independent grasp selection approach ([GGCNN](https://github.com/patrosAT/ggcnn_humanseg_ros.git)) ensures general applicability even in cluttered environments. To ensure save handovers, the approach uses two NN to segment [body parts](https://github.com/patrosAT/bodyparts_ros) and [hands](https://github.com/patrosAT/egohands_ros).
 
The movements are based on input of the three packages [Bodyparts](https://github.com/patrosAT/bodyparts_ros), [Egohands](https://github.com/patrosAT/egohands_ros) and [GGCNN](https://github.com/patrosAT/ggcnn_humanseg_ros.git).

**Input**
* **Depth Image:** [sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)
* **Bodyparts:** [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)
* **Egohands:** [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)
* **GGCNN:** [ggcnn_humanseg_ros/GraspPrediction](https://github.com/patrosAT/ggcnn_humanseg_ros/blob/master/msg/GraspPrediction.msg)
* **Arm State:** [rv_msgs/ManipulatorState](https://github.com/RoboticVisionOrg/rv_msgs/blob/master/msg/ManipulatorState.msg)

**Output**
* **Servo Pose:** [rv_msgs/ServoToPose](https://github.com/RoboticVisionOrg/rv_msgs/blob/master/action/ServoToPose.action)
* **Named Pose:** [rv_msgs/MoveToNamedPose](https://github.com/RoboticVisionOrg/rv_msgs/blob/master/action/MoveToNamedPose.action)
* **Gripper:** [rv_msgs/ActuateGripper](https://github.com/RoboticVisionOrg/rv_msgs/blob/master/action/ActuateGripper.action)


## Examples ##

### Example Video ###
[This youtube video](https://youtu.be/GjPDilJO4F0) shows the handover of 20 household objects from a frontal and a lateral perspective.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=GjPDilJO4F0" target="_blank"><img src="http://img.youtube.com/vi/GjPDilJO4F0/0.jpg" 
alt="YouTube H2R Handovers" width="560" height="415" border="10" /></a>

### Example RVIZ ###
This image shows the handover of a banana as seen by the robot. The visualization was done in RVIZ.

<img src="./imgs/graspPoint.png" width="560"/>


## Getting Started ##

### Dependencies ###

The models have been tested with Python 2.7.

### Hardware Requirements ###

* Depth camera *(for this project an [realsense D435](https://www.intelrealsense.com/depth-camera-d435/) was used)*
* GPU >= 4 GB
 
### Software Requirements ###

**ATTENTION: This package requires the [ROS](https://www.ros.org/) operating system!**

* Python: see [requirements.txt](requirements.txt)
* ROS packages:
```
rospy
actionlib
std_msgs
geometry_msgs
tf
tf2_ros
tf2_geometry_msgs
rv_msgs
rv_manipulation_driver
```

### ROS 3rd party packages ###

* [Bodyparts_ros](https://github.com/patrosAT/bodyparts_ros.git)
* [Egohands_ros](https://github.com/patrosAT/egohands_ros.git)
* [GGCNN](https://github.com/patrosAT/ggcnn_humanseg_ros.git)

**Note:** To enable real-time processing, it might be necessary to distribute these packages across several computers. We recommend using [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html) to keep the network usage on a reasonable level.

### Launch ###

Before launching the package, make sure that the camera and the 3rd party ROS packages are up and running. 

The ROS package contains a launch file:
* **[Closed-loop publisher](launch/handover_closed_loop.launch):** Interacts with the robot at a 0.1Hz rate.

**Input**
* **Depth Image:** [sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)
* **Bodyparts:** [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)
* **Egohands:** [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)
* **GGCNN:** [ggcnn_humanseg_ros/GraspPrediction](https://github.com/patrosAT/ggcnn_humanseg_ros/blob/master/msg/GraspPrediction.msg)
* **Arm State** [rv_msgs/ManipulatorState](https://github.com/RoboticVisionOrg/rv_msgs/blob/master/msg/ManipulatorState.msg)

**Output**
* **Servo Pose:** [rv_msgs/ServoToPose](https://github.com/RoboticVisionOrg/rv_msgs/blob/master/action/ServoToPose.action)
* **Named Pose:** [rv_msgs/MoveToNamedPose](https://github.com/RoboticVisionOrg/rv_msgs/blob/master/action/MoveToNamedPose.action)
* **Gripper:** [rv_msgs/ActuateGripper](https://github.com/RoboticVisionOrg/rv_msgs/blob/master/action/ActuateGripper.action)


## Configuration ##

The initial setup can be changed by adapting the [handover.yaml](cfg/handover.yaml) file:

**Camera:**
* **depth:** Rostopic the publisher is subscribing to (depth image).

**Subscription:**
* **bodyparts:** Rostopic the node is subcribing to (bodyparts).
* **egohands:** Rostopic the node is subcribing to (egohands).

**GGCNN:**
* **topic:** Rostopic the node is subcribing to (GGCNN).
* **window:** Number of windows that are combined to make the picking point estimation more robust.
* **deviation_position:** Maximal deviation in x, y and z form the window's mean. Estimations with a larger deviation in one of these directions are dropped and not considered for calculating the grasp point. 
* **deviation_orientation:** Maximal deviation in x, y, z, and w (orientation) form the window's mean. Estimations with a larger deviation in one of these orientations are dropped and not considered for calculating the grasp point.

**Robot:**
* **arm_state:** Rostopic the node is subcribing to (arm state).
* **arm_servo_pose:** Rostopic the node is publishing to (servo pose, see [rv_manipulation_driver](https://github.com/RoboticVisionOrg/rv_manipulation_driver)).
* **arm_named_pose:** Rostopic the node is publishing to (named pose, see [rv_manipulation_driver](https://github.com/RoboticVisionOrg/rv_manipulation_driver)).
* **arm_gripper:** Rostopic the node is publishing to (gripper, see [rv_manipulation_driver](https://github.com/RoboticVisionOrg/rv_manipulation_driver)).

**Movement:**
* **dist_ggcnn:** Distance to object until which the GGCNN will update the picking point *(future feature, not implemented yet)*.
* **dist_final:** Distance to object until which the system will monitor the object to detect deviations or unforseen events. Such cases will lead the system to abort the handover.
* **speed_approach:** Speed during setup and object transfer to dropping location (see [rv_manipulation_driver](https://github.com/RoboticVisionOrg/rv_manipulation_driver)).
* **scaling_handover:** Speed during the handover (see [rv_manipulation_driver](https://github.com/RoboticVisionOrg/rv_manipulation_driver)).

**Gripper:**
* **gripper_open:** Width of the opened gripper.
* **gripper_closed:** Width of the closed gripper.

**Visualization:** The visualization node publishes the picking point to be displayed in RVIZ.
* **topic:** Rostopic the node is publishing to (visualization).
* **activated:** Turn on/off visualization: *use keywords **"True"** or **"False"** only*.


## Acknowledgments ##

The ROS node interacts with the [Franka Emika](https://frankaemika.github.io/) robot arm using the high level API [rv_manipulation_driver](https://github.com/RoboticVisionOrg/rv_manipulation_driver) provided by the [Australian Center for Robotic Vision (ACRV)](http://roboticvision.org).


## License ##

The project is licensed under the BSD 4-Clause License.

## Disclaimer ##

Please keep in mind that no system is 100% fault tolerant and that this demonstrator is focused on pushing the boundaries of innovation. Careless interaction with robots can lead to serious injuries, always use appropriate caution!

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
