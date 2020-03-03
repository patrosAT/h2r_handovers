# ROS node for real-time bodyparts detection #

This is a ROS wrapper of the [ggcnn network](https://github.com/dougsm/ggcnn).

#### Input ####

* **Depth image:**
* **:**
* **:**
* **:**
* **:**

#### Output ####

Based on inputs of the bodyparts_ros, egohands_ros, and darknet_ros The package pub a grasp map and the best picking point (location, gripper orientation, gripper width & quality) that is adapted to the users and makes sure the robot does not pick a human body part.

* **Input:** Depth image: [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)
* **Output 1:** Grasp map: [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html) indicating the picking quality of each pixel.
* **Output:** Best picking point: [](): location, gripper orientation, gripper width & quality

## Getting Started ##

### Dependencies ###

The models have been tested with Python 2.7 and 3.6.

#### Hardware ####

* RGBD Camera
* GPU > 4000MiB
 
#### Python3 / pip3 ####
```
numpy
cv2
```
#### Ros ####
```
rospy
sensor_msgs
cv_bridge
```
#### Ros 3rd party packages ###
* [Bodyparts_ros](https://github.com/patrosAT/bodyparts_ros.git)
* [Egohands_ros](https://github.com/patrosAT/egohands_ros.git)
* [Darknet_ros](https://github.com/leggedrobotics/darknet_ros)

**Note:** To enable real-time processing it might be necessary to distribute these packages across several computers. We recommend using [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)s to keep the network usage on a reasonable level.

### Bilding ###

To maximize performance, use the 'release' build mode:
```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Configuration ###

The initial setup can be changed by adapting the [bodyparts.yaml](cfg/bodyparts.yaml) file:

* **Topic camera**
    * **rgb:** Camera topic the publisher node is subcribing to.
* **Topic interface**
    * **topic:** Topic the publisher node is publishing to.
    * **service:** Topic the service node is subcribing & publishing to.
    * **action:** Topic the action node is subcribing & publishing to.
    * **visualization:** Topic visualizing the output (requires *visualization: True*).
* **visualization:** 
* **gpu:** Number of the gpu.
* **model:** Number of NN-layers. Possible options are '50', '101', and '152'.

### Launch

Before launching the package, make sure that the rgbd-camera and the 

The ros package contains 3 launch files:
* **Publisher:** The [publisher](launch/bodyparts_publisher.launch) launch file starts a ros node that published a new mask every time a new rgb image is published.
* **Serivce:** The [serivce](launch/bodyparts_service.launch) launch file starts a ros service. 
* **Action:** The [action](launch/bodyparts_action.launch) launch file starts a ros action server.

### Visualization

If the visualization is set 'True', a [sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) is published to the corresponding topic. 


## Acknowledgments

The ROS node is powered by the pytorch implementation of [DrSleep](https://github.com/DrSleep). For more information on RefineNet please refer to the original [paper](https://arxiv.org/abs/1611.06612) or the following [github repository](https://github.com/DrSleep/light-weight-refinenet)

## License

* **Academic:** The project is licensed under the 3-clause BSD License.
* **Commercial:** Please contact the author.
