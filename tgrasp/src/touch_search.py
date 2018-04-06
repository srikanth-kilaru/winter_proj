#!/usr/bin/env python

#########################################
# Srikanth Kilaru
# Initial version: March 2018
# MS Robotics, Northwestern University
# Evanston, IL
# srikanthkilaru2018@u.northwestern.edu
##########################################
import sys
import rospy
import numpy as np
from math import ceil
import argparse
import actionlib
import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
from tf_conversions import posemath
from intera_interface import Limb
from tgrasp_msgs.msg import PclData
from geometry_msgs.msg import Point
from finger_sensor_msgs.msg import FingerFAI, FingerSAI, FingerTouch


###################################################################
# Overall logic and flow
####################################################################

####################################################################
# Camera Pcl data collection
####################################################################
# 1K samples of the PclData are received initially and we calculate
# the average of these samples and store the centroid location and object
# dimensions
# At this point grasping with the aid of touch sensing process begins.
# We first calculate the sequence of bounding boxes. See section on search
# process below for information on the search / bounding boxes
#
#####################################################################
# Gathering touch sensor data and the notion of touch / contact:
#####################################################################
# We listen to the sai messages being sent in by the finger_sensors node (@1KHz)
# It is required to keep all the 16 sensors clear of any contact during the
# first four seconds of starting up the program.
# We gather the first 50 samples and take their average to create a baseline
# for no contact
# Upon experimentation, it is found that each sensor behaves differently to
# touch / proximity in that the amount by which the SAI values spike up are
# different.
# The tol[] array stores the least delta value we want see between the
# observed SAI values and the baseline to establish whether a contact is made
# or not.
#
# SAI readings for each sensor are stored in an array and for every 50 samples,
# the mean is calculated and a comparison is made against the baseline.
# If the mean of these 50 values exceeds the baseline values by the tolerance
# associated with a particular sensor, then we establish that a contact has
# been made with that sensor.
#
# NOTE: Since these are IR sensors, the object does not need to be in full
# physical contact with the object, but only needs to be very close to it.
#
# When the search logic calls functions to check if a/any sensor is in contact
# or not, the function call sleeps for two seconds, as sometimes we might
# have just made contact and it takes a second or two for the new SAI mean to
# reflect the rise above the No-contact baseline
#
####################################################################
# Search process
####################################################################
# Starting from the x,y,z location received from camera as the first
# bounding / search box, we calculate a set of sequential search boxes in
# 3D space. Before we talk about how we align the search boxes, we have to
# figure out the best gripper orientation.
# If the object is a tall object (i.e. above a certain height threshold,
# we calculate the gripper orientation to be in a plane perpendicular to the
# z-axis of Sawyer.
# If the the object is a short object (i.e. below a certain height threshold,
# we calculate the gripper orientation to be either in a plane perpendicular to
# the x-axis or y-axis of Sawyer. The orientation in this case is dependent
# upon the width of the object as seen by the camera.
# If the camera sees a wide object, i.e. whose width is greater than or equal to
# the width of the fully open gripper, then we chose the orientation of the EE
# to be in a plane perpendicular to the x-axis, else the EE orientation will be
# in a plane perpendicular to the y-axis of Sawyer.
#
# Allocating the search boxes:
#
# In the EE orientation where it is in a plane perpendicular to Sawyer's z-axis,
# the Search / Bounding boxes follow a sliding window approach first
# in the +x axes direction and then in the -x axes direction at constant fixed
# increments. For each of these x locations, the window extends along the
# y-axes for a certain fixed amount.
# 
# If the EE orientation is in a plane perpendicular to Sawyer's x or y-axis,
# the Search / Bounding boxes follow a sliding window approach first
# along the -z axes direction and then along the y axes direction followed by
# along the x-axis. There are equal numbers of search boxes on either side of
# the initial search box (centered at the camera given location), along the
# x & y axes.
# 
# If the object is not found in the current search box, search moves on
# to the next search box and if the object is not found in any search box,
# then the search and grasp fails
# When starting scan in a bounding box, the gripper is fully opened and is
# closed in small increments to gain touch perception
# Once any finger makes the contact, the grasp is refined using the following
# algorithm -
#
# If one of the outer sensors is touching:
# 1. Move along x and y in such a way as to take the object into the jaw
# 2. If the object is making contact with the outer sensors of both fingers
# that is a case we do not handle, i.e. the object is too wide to grasp
# 3. If only the shallow (inner) sensors are touching, we move inside
# along the (-y axes or the -z axes depending upon the EE orientation)
# to deepen the grasp.
# 4. If the object is touching both the inner and mid sensors then we will
# try to move in along the -y axes or the -z axes, depending upon the
# EE orientation, to make contact with the mid and deep sensors.
# 5. If only the mid sensors and nothing else is touching, then it is a
# narrow object and we do not try to deepen the grasp.
# 6. If it is touching the inner sensors, then we do not try to deepen the
# grasp, we already have the best grasp.
# 7. Once the grasp depth is optimal (as defined above), we check to make sure
# both fingers are touching and we slowly close the gripper in small
# inrements till contact is made with both the fingers.
# 8. At this point the search and grasp has concluded and we lift the
# object up to a safe known cartesian location.
#######################################################################


class TouchSearch(object):
    def __init__(self):

        self.l_data_index = 0
        self.r_data_index = 0
        self.max_data_index = 50
        
        self.cam_x = 0
        self.cam_y = 0
        self.cam_z = 0

        self.obj_height = 0
        self.obj_width = 0

        # current EE position
        self.cur_x = 0
        self.cur_y = 0
        self.cur_z = 0

        # x-axis is the normal to the plane running through the centers
        # of the two fingers
        self.x_axis = 0 
        # y-axis is the normal to the plane running through the centers
        # of the two fingers
        self.y_axis = 1
        # z-axis is the normal to the plane running through the centers
        # of the two fingers
        self.z_axis = 2

        # default settings for orientation
        self.orientation_x = 0.5
        self.orientation_y = -0.5
        self.orientation_z = 0.5
        self.orientation_w = 0.5
        # current orientation indicator of the gripper
        self.orientation_axis = self.z_axis
        
        self.search_boxes = []
        self.cur_search_box = 0
        
        self.l_sensors_on = np.full((8), False, dtype=bool)
        self.r_sensors_on = np.full((8), False, dtype=bool)

        self.l_sensors_data = np.zeros((8,self.max_data_index))
        self.r_sensors_data = np.zeros((8,self.max_data_index))

        self.l_sensors_calibrated = False
        self.r_sensors_calibrated = False

        # Initialized to some hand calibrated values.
        # These will be overwritten during initialization
        self.l_touch_thresh = [110.0, 118.0, 94.0, 123.0, 192.0, 137.0, 128.0, 100.0]
        self.r_touch_thresh = [100.0, 93.0, 108.0, 40.0, 65.0, 120.0, 120.0, 90.0]

        # These margins are individualized per sensor based on experimentation
        self.l_tol = [15.0, 20.0, 10.0, 20.0, 10.0, 20.0, 20.0, 15.0]
        self.r_tol = [10.0, 20.0, 20.0, 10.0, 15.0, 30.0, 15.0, 20.0]
        
        self.scan_steps = 10
        self.z_min = -0.025415

        self.pcl_height = []
        self.pcl_width = []
        self.pcl_centroid_x = []
        self.pcl_centroid_y = []
        self.pcl_centroid_z = []
        
        self.finger_width = 0.012 # metres
        self.finger_depth = 0.038
        self.finger_gap_close = 0.044 # metres
        self.finger_gap_open = 0.088 # metres
        self.in_sensor_spacing = self.finger_depth/2.0 # metres

        self.low_obj_height = 0.0 # metres
        self.wide_obj_width = self.finger_gap_open
        
        self.MOVE_ERROR = 0
        self.MOVE_SUCCESS = 1
        self.FULL_GRASP = 2
        self.NO_GRASP = 3
        self.COL_SCAN_COMPLETE = 4
        self.UNKNOWN_ERR = 5
        self.ZERO_TOUCH = 6
        self.SHALLOW = 7
        self.MID_N_SHALLOW = 8
        self.MID_N_DEEP = 9
        self.BEST_GRASP_DEPTH = 10

        
    def set_object_camera_info(self, cam_x, cam_y, cam_z,
                               obj_height, obj_width):
        self.cam_x = cam_x
        self.cam_y = cam_y
        self.cam_z = cam_z
        self.obj_height = obj_height
        self.obj_width = obj_width
        self.cur_x = cam_x
        self.cur_y = cam_y
        self.cur_z = cam_z
        self.calc_gripper_orientation()

    def get_object_camera_xyz(self):
        return self.cam_x, self.cam_y, self.cam_z

    def get_obj_dimensions(self):
        return self.obj_height, self.obj_width
        
    def count_l_sensors_touched(self):
        # returns two values
        # - number of outer sensors and number of inner sensors in contact
        in_sensors = 0
        out_sensors = 0
        
        rospy.sleep(2.0)

        for s in range(8):
            if s == 3 or s == 4:
                if self.l_sensors_on[s]:
                    out_sensors += 1
            else:
                if self.l_sensors_on[s]:
                    in_sensors += 1
        return in_sensors, out_sensors
        
    def count_r_sensors_touched(self):
        # returns two values
        # - number of outer sensors and number of inner sensors in contact
        in_sensors = 0
        out_sensors = 0

        rospy.sleep(2.0)

        for s in range(8):
            if s == 3 or s == 4:
                if self.r_sensors_on[s]:
                    out_sensors += 1
            else:
                if self.r_sensors_on[s]:
                    in_sensors += 1
        return in_sensors, out_sensors

    def str_l_sensor_state(self):
        str = "\n"
        for i in range(8):
            str += "l_sensor[{}]={}\n".format(i+1,self.l_sensors_on[i])
        return(str)
        
    def str_r_sensor_state(self):
        str = "\n"
        for i in range(8):
            str += "r_sensor[{}]={}\n".format(i+1,self.r_sensors_on[i])
        return(str)
        
    def inner_l_sensors_touched(self):
        li, lo = self.count_l_sensors_touched()
        return(li >= 1)
        
    def inner_r_sensors_touched(self):
        ri, ro = self.count_r_sensors_touched()
        return(ri >= 1)
    
    def deep_l_sensors_touching(self):
        rospy.sleep(2.0)
        return self.l_sensors_on[0] or self.l_sensors_on[7]
    
    def deep_r_sensors_touching(self):
        rospy.sleep(1.0)
        return self.r_sensors_on[0] or self.r_sensors_on[7]
    
    def mid_l_sensors_touching(self):
        rospy.sleep(2.0)
        return self.l_sensors_on[1] or self.l_sensors_on[6]
    
    def mid_r_sensors_touching(self):
        rospy.sleep(2.0)
        return self.r_sensors_on[1] or self.r_sensors_on[6]
    
    def shallow_l_sensors_touching(self):
        rospy.sleep(2.0)
        return self.l_sensors_on[2] or self.l_sensors_on[5]
        
    def shallow_r_sensors_touching(self):
        rospy.sleep(2.0)
        return self.r_sensors_on[2] or self.r_sensors_on[5]

    def open_jaw_full(self):
        gripper = intera_interface.Gripper('right_gripper')
        gripper.set_position(gripper.MAX_POSITION)
                
    def close_jaw_full(self):
        gripper = intera_interface.Gripper('right_gripper')
        gripper.set_position(gripper.MIN_POSITION)
        
    def open_jaw_incr(self, offset_pos):
        gripper = intera_interface.Gripper('right_gripper')
        cmd_pos = max(min(gripper.get_position() + offset_pos,
                          gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        print("commanded position set to {} m".format(cmd_pos))
        
    def close_jaw_incr(self, offset_pos):
        gripper = intera_interface.Gripper('right_gripper')
        cmd_pos = max(min(gripper.get_position() - offset_pos,
                          gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        print("commanded position set to {} m".format(cmd_pos))
        
    def calc_gripper_orientation(self):
        # This function is called after the object height has been
        # correctly estimated from PCL data and stored
        # t calculates the orientation of the gripper in quarternion
        if self.obj_height <= self.low_obj_height:
            if self.obj_width >= self.wide_obj_width:
                # hope this orientation encounters less width
                self.orientation_x = 0.0
                self.orientation_y = 1.0
                self.orientation_z = 0.0
                self.orientation_w = 0.0
                self.orientation_axis = self.x_axis
            else:
                self.orientation_x = -0.707
                self.orientation_y = 0.707
                self.orientation_z = 0.02
                self.orientation_w = 0.0
                self.orientation_axis = self.y_axis
        else:
            self.orientation_x = 0.5
            self.orientation_y = -0.5
            self.orientation_z = 0.5
            self.orientation_w = 0.5
            self.orientation_axis = self.z_axis
    
    def goto_cartesian(self, x, y, z):
        print("goto_cartesian called with x={}, y={}, z={}".format(x,y,z))
        limb = Limb()
        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

        wpt_opts = MotionWaypointOptions()
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_names = limb.joint_names()
        endpoint_state = limb.tip_state('right_hand')
        if endpoint_state is None:
            print('Endpoint state not found')
            return self.MOVE_ERROR
        pose = endpoint_state.pose
        
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = self.orientation_x
        pose.orientation.y = self.orientation_y
        pose.orientation.z = self.orientation_z
        pose.orientation.w = self.orientation_w
        poseStamped = PoseStamped()
        poseStamped.pose = pose
        waypoint.set_cartesian_pose(poseStamped, 'right_hand', [])

        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory()
        if result is None:
            print("Trajectory FAILED to send")
            return self.MOVE_ERROR

        if result.result:
            print('Motion controller successfully finished the trajectory!')
            self.cur_x = x
            self.cur_y = y
            self.cur_z = z
            return self.MOVE_SUCCESS
        else:
            print('Motion controller failed to complete the trajectory %s',
                  result.errorId)
            return self.MOVE_ERROR
            
    def move_l_and_in(self):
        # called when L outer sensor is in contact
        # call to this is followed by call to deepen_grasp()

        # Move till the left outer sensor is no longer in touch
        num_iter = int(ceil(self.finger_gap_open/self.finger_width))
        for i in range(num_iter):
            li, lo = self.count_l_sensors_touched()
            print("In move_l_and_in: li={}, lo={}".format(li,lo))
            if lo > 0:
                if self.orientation_axis == self.x_axis:
                    status = self.goto_cartesian(self.cur_x,
                                                 self.cur_y - self.finger_width,
                                                 self.cur_z+0.007)
                else:
                    if self.orientation_axis == self.y_axis:
                        status = self.goto_cartesian(self.cur_x + self.finger_width,
                                                     self.cur_y,
                                                     self.cur_z+0.007)
                    else:
                        status = self.goto_cartesian(self.cur_x + self.finger_width,
                                                     self.cur_y,
                                                     self.cur_z)
                if status != self.MOVE_SUCCESS:
                    return status
            else:
                break
            
        # move in now
        for i in range(num_iter):
            li, lo = self.count_l_sensors_touched()
            print("In move_l_and_in: li={}, lo={}".format(li,lo))
            if li == 0:
                if self.orientation_axis == self.z_axis:
                    status = self.goto_cartesian(self.cur_x,
                                                 self.cur_y - self.in_sensor_spacing/2.0,
                                                 self.cur_z) 
                else:
                    status = self.goto_cartesian(self.cur_x,
                                                 self.cur_y,
                                                 max(self.z_min, self.cur_z - self.in_sensor_spacing/2.0))
                    
                if status != self.MOVE_SUCCESS:
                    return status
            else:
                break

    def move_r_and_in(self):
        # called when R outer sensor is in contact
        # call to this is followed by call to deepen_grasp()
        
        # Move till the right outer sensor is no longer in touch
        num_iter = int(ceil(self.finger_gap_open/self.finger_width))
        for i in range(num_iter):
            ri, ro = self.count_r_sensors_touched()
            print("In move_r_and_in: ri={}, ro={}".format(ri,ro))
            if ro > 0:
                if self.orientation_axis == self.x_axis:
                    status = self.goto_cartesian(self.cur_x,
                                                 self.cur_y + self.finger_width,
                                                 self.cur_z+0.007)
                else:
                    if self.orientation_axis == self.y_axis:
                        status = self.goto_cartesian(self.cur_x - self.finger_width,
                                                     self.cur_y,
                                                     self.cur_z+0.007)
                    else:
                        status = self.goto_cartesian(self.cur_x - self.finger_width,
                                                     self.cur_y,
                                                     self.cur_z)
                if status != self.MOVE_SUCCESS:
                    return status
            else:
                break

        # move in now
        for i in range(num_iter):
            ri, ro = self.count_r_sensors_touched()
            print("In move_r_and_in: ri={}, ro={}".format(ri,ro))
            if ri == 0:
                if self.orientation_axis == self.z_axis:
                    status = self.goto_cartesian(self.cur_x,
                                                 self.cur_y - self.in_sensor_spacing/2.0,
                                                 self.cur_z)
                else:
                    status = self.goto_cartesian(self.cur_x,
                                                 self.cur_y,
                                                 max(self.z_min, self.cur_z - self.in_sensor_spacing/2.0))
                if status != self.MOVE_SUCCESS:
                    return status
            else:
                break

    def deepen_grasp(self):
        print("In deepen_grasp")
        # this function is/should be called only after some contact is made
        # with the inside sensors of one or more fingers
        max_tries = 4
        gripper = intera_interface.Gripper('right_gripper')

        # open jaw so that we dont topple object
        # the reason why we open all the way is because the jaws dont open
        # in increment sometimes when they are shut all the way
        cur_pos = gripper.get_position()
        self.open_jaw_full()
        if self.orientation_axis == self.z_axis:
            status = self.goto_cartesian(self.cur_x,
                                         self.cur_y - self.in_sensor_spacing,
                                         self.cur_z)
        else:
            status = self.goto_cartesian(self.cur_x,
                                         self.cur_y,
                                         max(self.z_min, self.cur_z-self.in_sensor_spacing))
        # close EE back to previous position
        gripper.set_position(cur_pos)
        
        if status != self.MOVE_SUCCESS:
            return status

        if self.deep_l_sensors_touching() or self.deep_r_sensors_touching():
            print("one or both of deep sensors are touching")
            return self.BEST_GRASP_DEPTH
        
        if self.mid_l_sensors_touching() or self.mid_r_sensors_touching():
            if self.shallow_l_sensors_touching() or self.shallow_r_sensors_touching():
                # touching mid and shallow, so we can in theory go in one more
                # step.
                # open EE slightly so that we dont topple object
                cur_pos = gripper.get_position()
                self.open_jaw_full()
                if self.orientation_axis == self.z_axis:
                    status = self.goto_cartesian(self.cur_x,
                                                 self.cur_y-self.in_sensor_spacing,
                                                 self.cur_z)
                else:
                    status = self.goto_cartesian(self.cur_x,
                                                 self.cur_y,
                                                 max(self.z_min, self.cur_z-self.in_sensor_spacing))
                # close EE back to previous position
                gripper.set_position(cur_pos)
                if status != self.MOVE_SUCCESS:
                    return status
                return self.BEST_GRASP_DEPTH
            else:
                # touching mid but not shallow which means it could be touching
                # deep sensors or not, in either case this is the best
                # or good enough
                return self.BEST_GRASP_DEPTH
        else:
            # this would be a strange case, we should have atleast a mid grasp
            # by now, but what the heck lets move in one last time before we
            # are done.
            # open EE slightly so that we dont topple object
            print("Neither of the mid or deep sensors are touching")
            for i in range(max_tries):
                cur_pos = gripper.get_position()
                self.open_jaw_full()
                if self.orientation_axis == self.z_axis:
                    status = self.goto_cartesian(self.cur_x,
                                                 self.cur_y-self.in_sensor_spacing,
                                                 self.cur_z)
                else:
                    status = self.goto_cartesian(self.cur_x,
                                                 self.cur_y,
                                                 max(self.z_min, self.cur_z-self.in_sensor_spacing))
                # close EE back to previous position
                gripper.set_position(cur_pos)
                if status != self.MOVE_SUCCESS:
                    return status
                if self.deep_l_sensors_touching() or self.deep_r_sensors_touching() or self.mid_l_sensors_touching() or self.mid_r_sensors_touching():
                    return self.BEST_GRASP_DEPTH
            return self.NO_GRASP
        
    def iterate_touch_sense(self):
        li, lo = self.count_l_sensors_touched()
        ri, ro = self.count_r_sensors_touched()
        l_cnt = li + lo
        r_cnt = ri + ro
        
        print("Left sensor state {}".format(self.str_l_sensor_state()))
        print("Right sensor state {}".format(self.str_r_sensor_state()))

        if l_cnt > 0 and r_cnt > 0:
            # both fingers have touched, this is a case of the object
            # touching both fingers with only the inner sensors
            # We do not deal w/ cases where only the outer two sensors for each
            # finger are touching as the objects handled fit within the
            # max grasp depth and therefore cannot simultaneously touch outer
            # sensors of both fingers
            print("both fingers have touched")
            if self.deep_l_sensors_touching() or self.deep_r_sensors_touching():
                print("we have the complete grasp")
                return (self.BEST_GRASP_DEPTH)
            elif self.mid_l_sensors_touching() or self.mid_r_sensors_touching():
                if self.shallow_l_sensors_touching() or self.shallow_r_sensors_touching():
                    # middle and shallow inner sensors are touching 
                    print("try deepening the grasp from mid")
                    return self.deepen_grasp()
                else:
                    # this is the best case as it is a thin object and wouldnt
                    # touch more than 2 sensors at a time
                    print("best contact is touching middle inner sensors only")
                    return self.BEST_GRASP_DEPTH
            elif ro > 0 and lo > 0:
                if ri == 0 and li == 0:
                    print("Both the outer sensors are touching, must be the table or a object wider than what we can handle")
                    return self.ZERO_TOUCH            
                else:
                    return self.BEST_GRASP_DEPTH
            else:
                print("try deepening the grasp from shallow")
                return self.deepen_grasp()
            
        elif l_cnt > 0 and r_cnt == 0:
            # only the left finger has touched
            print("only left finger has touched, li={}, lo={}".format(li,lo))
            if li == 0 and lo > 0:
                # case when only left outer sensors is touching 
                self.move_l_and_in()
                return self.deepen_grasp()
            elif li > 0 and lo == 0:
                # case when only left inner sensors are touching
                return self.deepen_grasp()
            elif li > 0 and lo > 0:
                # case when both left sensors are touching
                print("reached an impossible case")
                return self.UNKNOWN_ERR
        elif l_cnt == 0 and r_cnt > 0:
            # only the right finger has touched
            print("only right finger has touched, ri={}, ro={}".format(ri,ro))
            if ri == 0 and ro > 0:
                # case when only right outer sensors is touching
                self.move_r_and_in()
                return self.deepen_grasp()
            elif ri > 0 and ro == 0:
                # case when only right inner sensors are touching
                return self.deepen_grasp()
            elif ri > 0 and ro > 0:
                # case when both right sensors are touching
                print("reached an impossible case")
                return self.UNKNOWN_ERR
        else:
            print("No finger sensors are touching at {}".format(self.cur_z))
            return self.ZERO_TOUCH
        
    def scan_cur_search_box(self):
        offset_pos = 0
        gripper = intera_interface.Gripper('right_gripper')
        num_steps = self.scan_steps
        percent_delta = 1.0 / num_steps
        position_increment = (gripper.MAX_POSITION - gripper.MIN_POSITION) * percent_delta
        status = None
        print("num_steps = {}".format(num_steps))
        for i in range(num_steps):
            print("scan_cur_search_box iteration = {}".format(i))
            # close by small increments
            offset_pos += position_increment
            self.close_jaw_incr(offset_pos)
            status = self.iterate_touch_sense()
            if status == self.BEST_GRASP_DEPTH:
                # tighten grasp till both fingers are just touching
                # only one finger is touching
                #return self.full_grip(offset_pos, num_steps - (i+1), position_increment)
                self.close_jaw_full()
                return self.FULL_GRASP
            else:
                return status
    
    def goto_search_box(self):
        self.open_jaw_full()
        #the iterate boxes call before calling this function increments the
        # current box number, so we have to subtract by 1
        return self.goto_cartesian(self.search_boxes[self.cur_search_box-1][0],
                                   self.search_boxes[self.cur_search_box-1][1],
                                   self.search_boxes[self.cur_search_box-1][2])
    
    def iterate_search_boxes(self):
        num_boxes = len(self.search_boxes)
        # 0 based box numbering
        if self.cur_search_box+1 >= num_boxes:
            return False
        else:
            self.cur_search_box += 1
            return True
        
    def create_search_boxes(self):
        # the very first one is the camera given x,y
        #self.search_boxes.append([self.cam_x, self.cam_y, self.cam_z])
        if self.obj_height <= self.low_obj_height: # short object
            x_inc = self.finger_width
            y_inc = self.finger_width
            z_inc = self.in_sensor_spacing
            num_y = int(self.finger_gap_open/y_inc)-1
            num_x = int(self.finger_gap_open/x_inc)-1
            num_z = int(self.finger_depth/z_inc)

            print("num_x = {}, x_inc = {}".format(num_x, x_inc))
            print("num_y = {}, y_inc = {}".format(num_y, y_inc))
            print("num_z = {}, z_inc = {}".format(num_z, z_inc))
            
            '''
            for k in range(num_z):
                # first search along the -ve z-axis from given camera position
                self.search_boxes.append([self.cam_x, self.cam_y,
                                          self.cam_z-(k+1)*z_inc])
            '''
            
            for i in range(num_x):
                for j in range(num_y):
                    for k in range(num_z):
                        # first search along the z-axis, then y and then x 
                        if i < num_x/2:
                            x_inc_mult = -i
                        else:
                            x_inc_mult = i
                        if j < num_y/2:
                            y_inc_mult = -j
                        else:
                            y_inc_mult = j
                        self.search_boxes.append([self.cam_x+x_inc_mult*x_inc,
                                                  self.cam_y+y_inc_mult*y_inc,
                                                  self.cam_z-k*z_inc])
            
        else: # Tall object
            x_inc = self.finger_width
            num_x =  int(self.finger_gap_open/x_inc)-1
            y_inc = self.in_sensor_spacing
            num_y = int(self.finger_depth/y_inc)
            self.search_boxes.append([self.cam_x, self.cam_y, self.cam_z])
            # first search along the x-axis and then the y-axis
            for i in range(num_x):
                if i < num_x/2:
                    x_inc_mult = -(i+1)
                else:
                    x_inc_mult = (i+1)
                for j in range(num_y):
                    self.search_boxes.append([self.cam_x+x_inc_mult*x_inc,
                                              self.cam_y-(j+1)*y_inc,
                                              self.cam_z])
        print("Search boxes:\n")
        print(self.search_boxes)
                
    def pickup_object(self):
        # Lift it to a safe know location to claim success
        success = self.goto_cartesian(self.cur_x, self.cur_y, self.cur_z+0.2)
        if not success:
            print("goto_cartesian failed for {},{},{}".format(self.cur_x,
                                                              self.cur_y,
                                                              self.cur_z+0.2))
    def search_and_grasp(self):
        #This is the true starting point of the touch search and grasp algo
        self.create_search_boxes()
        while self.iterate_search_boxes():
            if self.goto_search_box() == self.MOVE_ERROR:
                print("Move failed") 
                exit()
            status = self.scan_cur_search_box()
            if status == self.FULL_GRASP:
                self.pickup_object()
                exit()

    def touch_l_sai_update(self, msg):
        self.l_sensors_data[0,self.l_data_index] = msg.sensor1
        self.l_sensors_data[1,self.l_data_index] = msg.sensor2
        self.l_sensors_data[2,self.l_data_index] = msg.sensor3
        self.l_sensors_data[3,self.l_data_index] = msg.sensor4
        self.l_sensors_data[4,self.l_data_index] = msg.sensor5
        self.l_sensors_data[5,self.l_data_index] = msg.sensor6
        self.l_sensors_data[6,self.l_data_index] = msg.sensor7
        self.l_sensors_data[7,self.l_data_index] = msg.sensor8
        self.l_data_index += 1
        if self.l_data_index == self.max_data_index:
            # Do the one time initial baseline calibration with no contact.
            # Ensure there is NOTHING close to the sensors when this code runs
            if not self.l_sensors_calibrated:
                self.l_sensors_calibrated = True
                self.l_touch_thresh[0] = self.mean(self.l_sensors_data[0,:])
                self.l_touch_thresh[1] = self.mean(self.l_sensors_data[1,:])
                self.l_touch_thresh[2] = self.mean(self.l_sensors_data[2,:])                
                self.l_touch_thresh[3] = self.mean(self.l_sensors_data[3,:])
                self.l_touch_thresh[4] = self.mean(self.l_sensors_data[4,:])
                self.l_touch_thresh[5] = self.mean(self.l_sensors_data[5,:])
                self.l_touch_thresh[6] = self.mean(self.l_sensors_data[6,:])
                self.l_touch_thresh[7] = self.mean(self.l_sensors_data[7,:])
                
            self.l_data_index = 0
            self.l_sensors_on[0] = self.mean(self.l_sensors_data[0,:]) > self.l_touch_thresh[0]+self.l_tol[0]
            self.l_sensors_on[1] = self.mean(self.l_sensors_data[1,:]) > self.l_touch_thresh[1]+self.l_tol[1]
            self.l_sensors_on[2] = self.mean(self.l_sensors_data[2,:]) > self.l_touch_thresh[2]+self.l_tol[2]
            self.l_sensors_on[3] = self.mean(self.l_sensors_data[3,:]) > self.l_touch_thresh[3]+self.l_tol[3]
            self.l_sensors_on[4] = self.mean(self.l_sensors_data[4,:]) > self.l_touch_thresh[4]+self.l_tol[4]
            self.l_sensors_on[5] = self.mean(self.l_sensors_data[5,:]) > self.l_touch_thresh[5]+self.l_tol[5]
            self.l_sensors_on[6] = self.mean(self.l_sensors_data[6,:]) > self.l_touch_thresh[6]+self.l_tol[6]
            self.l_sensors_on[7] = self.mean(self.l_sensors_data[7,:]) > self.l_touch_thresh[7]+self.l_tol[7]
 
            #rospy.loginfo("Average values of Left sensors: %f %f %f %f %f %f %f %f", self.mean(self.l_sensors_data[0,:]), self.mean(self.l_sensors_data[1,:]), self.mean(self.l_sensors_data[2,:]), self.mean(self.l_sensors_data[3,:]), self.mean(self.l_sensors_data[4,:]), self.mean(self.l_sensors_data[5,:]), self.mean(self.l_sensors_data[6,:]), self.mean(self.l_sensors_data[7,:]))


    def touch_r_sai_update(self, msg):
        self.r_sensors_data[0,self.r_data_index] = msg.sensor1
        self.r_sensors_data[1,self.r_data_index] = msg.sensor2
        self.r_sensors_data[2,self.r_data_index] = msg.sensor3
        self.r_sensors_data[3,self.r_data_index] = msg.sensor4
        self.r_sensors_data[4,self.r_data_index] = msg.sensor5
        self.r_sensors_data[5,self.r_data_index] = msg.sensor6
        self.r_sensors_data[6,self.r_data_index] = msg.sensor7
        self.r_sensors_data[7,self.r_data_index] = msg.sensor8
        self.r_data_index += 1

        if self.r_data_index == self.max_data_index:
            # Do the one time initial baseline calibration with no contact.
            # Ensure there is NOTHING close to the sensors when this code runs
            if not self.r_sensors_calibrated:
                self.r_sensors_calibrated = True
                self.r_touch_thresh[0] = self.mean(self.r_sensors_data[0,:])
                self.r_touch_thresh[1] = self.mean(self.r_sensors_data[1,:])
                self.r_touch_thresh[2] = self.mean(self.r_sensors_data[2,:])                
                self.r_touch_thresh[3] = self.mean(self.r_sensors_data[3,:])
                self.r_touch_thresh[4] = self.mean(self.r_sensors_data[4,:])
                self.r_touch_thresh[5] = self.mean(self.r_sensors_data[5,:])
                self.r_touch_thresh[6] = self.mean(self.r_sensors_data[6,:])
                self.r_touch_thresh[7] = self.mean(self.r_sensors_data[7,:])
                
            self.r_data_index = 0
            self.r_sensors_on[0] = self.mean(self.r_sensors_data[0,:]) > self.r_touch_thresh[0]+self.r_tol[0]
            self.r_sensors_on[1] = self.mean(self.r_sensors_data[1,:]) > self.r_touch_thresh[1]+self.r_tol[1]
            self.r_sensors_on[2] = self.mean(self.r_sensors_data[2,:]) > self.r_touch_thresh[2]+self.r_tol[2]
            self.r_sensors_on[3] = self.mean(self.r_sensors_data[3,:]) > self.r_touch_thresh[3]+self.r_tol[3]
            self.r_sensors_on[4] = self.mean(self.r_sensors_data[4,:]) > self.r_touch_thresh[4]+self.r_tol[4]
            self.r_sensors_on[5] = self.mean(self.r_sensors_data[5,:]) > self.r_touch_thresh[5]+self.r_tol[5]
            self.r_sensors_on[6] = self.mean(self.r_sensors_data[6,:]) > self.r_touch_thresh[6]+self.r_tol[6]
            self.r_sensors_on[7] = self.mean(self.r_sensors_data[7,:]) > self.r_touch_thresh[7]+self.r_tol[7]
    
            #rospy.loginfo("Average values of Right sensors: %f %f %f %f %f %f %f %f", self.mean(self.r_sensors_data[0,:]), self.mean(self.r_sensors_data[1,:]), self.mean(self.r_sensors_data[2,:]), self.mean(self.r_sensors_data[3,:]), self.mean(self.r_sensors_data[4,:]), self.mean(self.r_sensors_data[5,:]), self.mean(self.r_sensors_data[6,:]), self.mean(self.r_sensors_data[7,:]))


    def mean(self, numbers):
        return float(sum(numbers)) / max(len(numbers), 1)
    
    def pclData_update(self, msg):
        if len(self.pcl_height) < 1000:
            self.pcl_height.append(msg.height)
            self.pcl_width.append(msg.width)
            self.pcl_centroid_x.append(msg.centroid.x)
            self.pcl_centroid_y.append(msg.centroid.y)
            self.pcl_centroid_z.append(msg.centroid.z)
        else:
            self.set_object_camera_info(self.mean(self.pcl_centroid_x),
                                        self.mean(self.pcl_centroid_y),
                                        self.mean(self.pcl_centroid_z),
                                        self.mean(self.pcl_height),
                                        self.mean(self.pcl_width))
            self.search_and_grasp()
        

def main():
    rospy.init_node('touch_search')
    ts = TouchSearch()
    rospy.Subscriber('/left_finger/sai', FingerSAI, ts.touch_l_sai_update)
    rospy.Subscriber('/right_finger/sai', FingerSAI, ts.touch_r_sai_update)

    rate = rospy.Rate(1000)

    rospy.Subscriber('tgrasp/pclData2', PclData, ts.pclData_update)

    '''
    # temporary setting for quick testing for tall objects
    ts.set_object_camera_info(-0.341847607721, -0.576748373933, -0.0107231368966,
                              0.8, ts.wide_obj_width-0.04)
    
    '''
    # temporary setting for quick testing for short and wide objects
    ts.set_object_camera_info(-0.3379, -0.7296, ts.low_obj_height,
                              ts.low_obj_height, ts.wide_obj_width)

    '''
    ts.set_object_camera_info(-0.3379, -0.7296, ts.low_obj_height,
                              ts.low_obj_height, ts.wide_obj_width-0.04)
    '''
    
    ts.search_and_grasp()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")
