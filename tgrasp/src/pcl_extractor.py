#!/usr/bin/env python
import rospy
import numpy as np
from math import pi, sin, cos
from tgrasp_msgs.msg import PclData
from geometry_msgs.msg import Point

def trans_points(data):
    pub = rospy.Publisher("tgrasp/pclData2", PclData, queue_size = 10)
    #initialziations, rotation matrices, and kinect location
    out = PclData()
    cent = Point()
    rotx = np.array([[1,0,0],[0,cos(pi/2),-sin(pi/2)],[0,sin(pi/2),cos(pi/2)]])
    rotz = np.array([[cos(pi),-sin(pi),0],[sin(pi),cos(pi),0],[0,0,1]])
    
    #x,y,z of camera center from sawyer's base frame origin
    kin_loc = np.array([[0.0],[1.524],[-0.114]])

    #do the transformation
    exact = data.centroid
    bot_loc = np.array([[float(exact.x)],[float(exact.y)],[float(exact.z)]])
    kinect_to_bottle = rotx.dot(rotz.dot(bot_loc))
    baxter_to_bottle = kinect_to_bottle + kin_loc

    #store the transformed information into a new message
    cent.x = float(baxter_to_bottle[0])
    cent.y = float(baxter_to_bottle[1])
    cent.z = float(baxter_to_bottle[2])
    out.centroid = cent
    out.height = data.height
    out.width = data.width
    out.ratio = data.ratio

    #publish the message
    pub.publish(out)

if __name__ == '__main__':
    #initializations 
    rospy.init_node("pcl_transform")
    rospy.Subscriber("/pclData",PclData, trans_points)
    rospy.spin()

