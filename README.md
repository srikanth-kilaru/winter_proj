# Touch sensor assisted grasping
ME499- Robotics Winter Project (individual) 


## Overview

Robots rely on Computer Vision technologies like depth sensing or point cloud information obtained from RBGD cameras to perceive the size and location of objects they have to grasp and manipulate. However many challenges remain in perceiving accurate location and size of the objects due to factors like unfavorable lighting conditions, occlusions, noisy data, objects not being perfectly still etc.

Humans rely on their sense of touch to perform fine-grained localization and grasping of an object once their hands reach a location based on coarse grained information provided by their eyesight.

The goal of this project is to use a similar approach in localizing and grasping an object once the robot is commanded to move its end-effector (gripper) to the object's location provided by the 3D point cloud data.

To this end, I used touch sensors from [Robotic Materials](http://roboticmaterials.com/rm/product/smart-gripper-pads-for-robotiq/) that were mounted on both the gripper fingers. I used the [ASUS XtionPRO LIVE](https://www.asus.com/us/3D-Sensor/Xtion_PRO_LIVE/) for vision sensing.
![Image1](images/gripped.png?raw=true)

This project was implemented in ROS using the python programming language and it consists of three main software functional blocks:
1. A sensor reading node from [Robotic Materials](https://github.com/RoboticMaterials/finger-sensors-ros) that reads touch sensor data coming in through a USB port and publishes this data on ROS topics '/left_finger/sai' and '/right_finger/sai' software.
2. A point cloud processing node that depends on the ROS packages [perception_pcl](https://github.com/ros-perception/perception_pcl) and [openni2_camera](https://github.com/ros-drivers/openni2_camera).
3. A touch sensing and grasping node that interfaces with the ROS package [intera_sdk](https://github.com/RethinkRobotics/intera_sdk/) to control Sawyer's end effector position and for closing / opening the gripper.


## Description

### Touch Sensors
Each of the two fingers of Sawyer's grippers is attached to a touch sensor pad. Each sensor pad has a total of eight sensors, four on the inner surface and two on the outer surface as shown in the pictures below -

![Image2](images/robotiq_drawing_sensor_num.png?raw=true)
![Image3](images/finger_closeup.png?raw=true)


Each of the eight touch sensors on each of the two fingers has a different sensitivity and noise profiles and therefore has to be calibrated individually to detect whether it has touched an object or not. In addition the concept of touch is not a binary notion since the sensor values goes up as the object comes closer to the sensor even before a complete contact is made.
See the picture below to get an idea of the different levels of sensor values and noise levels for the two fingers without any object in contact.
![Image4](images/Sensor-graphics.png?raw=true)

### Computer Vision
An ASUS XtionPRO LIVE is used to view objects that need to be grasped. Point cloud location and the centroid of the object to be grasped is published to a topic that the touch_sense grasping node subscribes to. This part of the project relies heavily on the perception_pcl ROS package to compute point cloud of the object on a flat surface. The launch file reads in raw point cloud data from the XtionPRO and filters it to a more manageable dataset using filters. A cluster extractor node takes the filtered point cloud data and extracts point clusters. Finally, the centroid, height, width, and width:height ratio are computed, with the centroid values transformed to Sawyer's frame of reference.

![Image4](images/point_clouds.png?raw=true)

### Touch Sensing and Grasping node
This is the main node where the touch sense based localization and grasping logic is implemented.
This node subscribes to ROS topics that publish the sensory data coming in from the sensor reading node and the pointcloud information (PCL data) coming in from the computer vision node.
The PCL data gives the centroid location of the object to be grasped and this is the location in Cartesian coordinates of Sawyer's base frame. Sawyer is commanded to move the end effector to this location. Once this location is reached, the grippers close in small increments, both to prevent deforming delicate objects as well as notice any touch contact. If there is partial contact on any of the sensors (either the inner or the outer), the end effector and the gripper move in such a way as to deepen the grasp. If there is zero contact at the initially commanded location, the end effector is commanded to the next search location within close proximity of the initial location and it repeats the whole search process by slowly closing the grippers. In this way the gripper iterates through a series of search locations in and around the location initially identified as the object's centroid location by the PCL data.

### Kinematics
Sawyer's movements are controlled using the intera_sdk ROS package developed by [Rethink Robotics](http://sdk.rethinkrobotics.com/intera/Main_Page). Specifically the [intera_motion_interface](http://sdk.rethinkrobotics.com/intera/Robot_Interface) from this package is used by the touch sensing node to command Sawyer to take the end effector to a specific cartesian location and to open / close the grippers. The intera_motion_interface does the path planning, including collision detection, to reach the goal.


## Implementation Details
## Package Structure Overview
contains the packages - `tgrasp`, `tgrasp_msgs`

`tgrasp` package contains -
`src`: contains - `touch_search.py`, `pcl_extractor.py` `l_sensor_graphics.py` `r_sensor_graphics.py`
`launch`: contains `tgrasp.launch`
`rviz`: contains `rvizConfig.rviz`

`tgrasp_msgs` package contains -
`msg`: contains `PclData.msg`

## Overview of Functionality
### ROS nodes written by me:
1. `touch_search.py`  
    - Sub: /left_finger/sai, /right_finger/sai, tgrasp/pclData2  
    - Pub: None
2. `l_sensor_graphics.py`   
    - Sub: /left_finger/sai, /right_finger/sai
    - Pub: None
3. `r_sensor_graphics.py`   
    - Sub: /left_finger/sai, /right_finger/sai
    - Pub: None
4. `cluster_extracter.cpp`   
    - Sub: none  
    - Pub: pclData    
5. `pcl_transform.py`    
    - Sub: pclData   
    - Pub: tgrasp/pclData2


### Relevant topics:   
1. /left_finger/sai, /right_finger/sai   - FingerSAI.msg   
2. pclData   -PclData.msg   
3. inspector/pclData2   - PclData.msg   

### Relevant Msgs:
1. FingerSAI.msg (of format: float32 sensor1, float32 sensor2, float32 sensor3, float32 sensor4, float32 sensor5, float32 sensor6, float32 sensor7, float32 sensor8
2. PclData.msg (of format: point32 centroids, float32 heights, float32 widths, float32 obj_id)   

### Packages Used
* [`perception_pcl`](https://github.com/ros-perception/perception_pcl.git) (kinetic-devel branch)
* [`openni2_camera`](https://github.com/ros-drivers/openni2_camera) (indigo-devel branch)
* [`rqt_reconfigure`](https://github.com/ros-visualization/rqt_reconfigure.git) (optional, used mostly for debugging)
* This [Github repository](https://github.com/NU-MSR/nodelet_pcl_demo) was used as a framework to build the CV components of this project.*
* [finger_sensors](https://github.com/RoboticMaterials/finger-sensors-ros) (required for reading sensor data)
* [intera_sdk](https://github.com/RethinkRobotics/intera_sdk/) (required for controlling Sawyer)


#### Instructions for launching 
The launch file [tgrasp.launch](tgrasp/launch/tgrasp.launch) runs the touch_sense node
```
$ roslaunch tgrasp tgrasp.launch
```
The launch file `finger_sensors.launch` runs the finger_sensors node.
```
$ roslaunch finger_sensors finger_sensors.launch
```
Once the above mentioned packages are installed, these are the only two launch files required for running the touch sense assisted grasping program.

*Make sure all the required packages are properly installed and the XtionPRO LIVE and the touch sensors are pluged in to the USB port of your computer!*

The scripts `l_sensor_graphics.py` and `r_sensor_graphics.py` are purely for visualizing the sensor readings in a GUI and can be run in a separate window. No launch file is required. 

### Algorithm used by touch_search.py
The algorithm employed by the touch_search node is fairly simple yet quite robust.
The object location information contained in the PclData centroid attributes is used as the initial coordinates to command the end effector (EE) to.

This node constructs two search boxes on either sides (total of four) of the object centroid along the x axis. The depth of each search box is set to be the depth of the gripper. The starting search box is the initial location of the gripper as received from the PCL data. In each search box, the gripper slowly closes until the touch sensors sense a touch. If there is no touch sensed on any of the eight sensors on either of the fingers, the EE is commanded to the next search box. If in any search box, if the outer sensors (sensor 4 & 5) sense touch, this is taken as an indication of outer contact. If this outer sensors are located on the left finger, the EE is commanded to move left in small increments equal to the finger width until there is no outer touch. This then is taken as an indication to move 'in' (along the y-axis) to make contact with the inner sensors. Once the inner sensors have made contact the grasp is slowly closed and the object is raised to a fixed height to claim a successful grasp!. The logic mentioned for the left gripper is applicable to the right finger as well, except that instead of moving left, it moves to the right.
Once touch is sensed on the inner sensors, the algorithm checks if the object grasp can be deepened before the gripper is closed. By deepening we mean that if only the shallow sensors are touching, then we will move 'in' to make contact with the inner middle sensors. For curved objects it is very possible that only the middle sensors register a contact and that is considered as the best possible grasp depth.
A key aspect in this implementation is to account for the noisy sensors and tune the definition of what sensor values correspond to a 'touch' or 'contact'.
