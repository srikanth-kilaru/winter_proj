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


#### Overall logic and flow implemented in this node

##### Camera Pcl data collection

1K samples of the PclData are received initially and we calculate the average of these samples and store the centroid location and object dimensions.

At this point grasping with the aid of touch sensing process begins. We first calculate the sequence of bounding boxes. See section on search process below for information on the search / bounding boxes.


##### Gathering touch sensor data and the notion of touch / contact:

We listen to the SAI (Slow Acting Indicator) messages being sent in by the finger_sensors node (@1KHz).
 It is required that we keep all the 16 sensors clear of any contact during the first four seconds of starting up the program. During this period, the program gathers the first 50 samples and takes their average to create a set of baseline values for the no contact scenario.
 
Upon experimentation, it is found that each sensor behaves differently to touch / proximity in that the amount by which the SAI values spike up are different.

The tol[] array stores the least delta value we want see between the observed SAI values and the baseline to establish whether a contact is made or not.

SAI readings for each sensor are stored in an array and for every 50 samples, the mean is calculated and a comparison is made against the baseline. If the mean of these 50 values exceeds the baseline values by the tolerance associated with a particular sensor, then we establish that a contact has been made with that sensor.

NOTE: Since these are IR sensors, the object does not need to be in full physical contact with the object, but only needs to be very close to it.

When the search logic calls functions to check if a/any sensor is in contact or not, the function call sleeps for two seconds, as sometimes we might have just made contact and it takes a second or two for the new SAI mean to reflect the rise above the no-contact baseline.



##### Search process

Starting from the x,y,z location received from camera as the first bounding / search box, we calculate a set of sequential search boxes in 3D space.

Before we talk about how we align the search boxes, we have to figure out the best gripper orientation. If the object is a tall object (i.e. above a certain height threshold, we calculate the gripper orientation to be in a plane perpendicular to the z-axis of Sawyer.

If the the object is a short object (i.e. below a certain height threshold, we calculate the gripper orientation to be either in a plane perpendicular to the x-axis or y-axis of Sawyer. The orientation in this case is dependent upon the width of the object as seen by the camera. If the camera sees a wide object, i.e. whose width is greater than or equal to the width of the fully open gripper, then we chose the orientation of the EE to be in a plane perpendicular to the x-axis, else the EE orientation will be in a plane perpendicular to the y-axis of Sawyer.


###### Allocating the search boxes:
In the EE orientation where it is in a plane perpendicular to Sawyer's z-axis, the Search / Bounding boxes follow a sliding window approach first in the +x axes direction and then in the -x axes direction at constant fixed increments. For each of these x locations, the window extends along the y-axes for a certain fixed amount.

If the EE orientation is in a plane perpendicular to Sawyer's x or y-axis, the Search / Bounding boxes follow a sliding window approach first along the -z axes direction and then along the y axes direction followed by along the x-axis. There are equal numbers of search boxes on either side of the initial search box (centered at the camera given location), along the x & y axes.

If the object is not found in the current search box, search moves on to the next search box and if the object is not found in any search box, then the search and grasp fails When starting scan in a bounding box, the gripper is fully opened and is closed in small increments to gain touch perception. Once any finger makes the contact, the grasp is refined using the following algorithm -

If one of the outer sensors is touching:
1. Move along x and y in such a way as to take the object into the jaw
2. If the object is making contact with the outer sensors of both fingers that is a case we do not handle, i.e. the object is too wide to grasp
3. If only the shallow (inner) sensors are touching, we move inside along the (-y axes or the -z axes depending upon the EE orientation) to deepen the grasp.
4. If the object is touching both the inner and mid sensors then we will try to move in along the -y axes or the -z axes, depending upon the EE orientation, to make contact with the mid and deep sensors.
5. If only the mid sensors and nothing else is touching, then it is a narrow object and we do not try to deepen the grasp.
6. If it is touching the inner sensors, then we do not try to deepen the grasp, we already have the best grasp.
7. Once the grasp depth is optimal (as defined above), we check to make sure both fingers are touching and we slowly close the gripper in small increments till contact is made with both the fingers.
8. At this point the search and grasp has concluded and we lift the object up to a safe known cartesian location.


#### ROS Package structure
`tgrasp` package contains -
`src`: contains - `touch_search.py`, `pcl_extractor.py` `l_sensor_sai_graphics.py` `r_sensor_sai_graphics.py`
`launch`: contains `tgrasp.launch`
`rviz`: contains `rvizConfig.rviz`

`tgrasp_msgs` package contains -
`msg`: contains `PclData.msg`

## Overview of Functionality
### ROS nodes written by me:
1. `touch_search.py`  
    - Sub: /left_finger/sai, /right_finger/sai, tgrasp/pclData2  
    - Pub: None
2. `l_sensor_sai_graphics.py`   
    - Sub: /left_finger/sai, /right_finger/sai
    - Pub: None
3. `r_sensor_sai_graphics.py`   
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

