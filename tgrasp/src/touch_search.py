#!/usr/bin/env python

import sys
import rospy
import numpy as np
import modern_robotics as mr
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

class TouchSearch(object):
    def __init__(self, camera_x = None, camera_y= None, camera_z = None):

        self.camera_x = camera_x
        self.camera_y = camera_y
        self.camera_z = camera_z

        self.obj_height = obj_height
        self.obj_width = obj_width

        self.cur_x = camera_x
        self.cur_y = camera_y
        self.cur_z = camera_z

        self.search_boxes = []
        self.ee_orientation = 0
        self.cur_search_box = 0
        
        self.l_sensors_on =
        [False, False, False, False, False, False, False, False]
        self.r_sensors_on =
        [False, False, False, False, False, False, False, False]

        self.z_increment = 0.01 # scan step size in metres
        self.OBJ_HEIGHT_LOW = 0.1 # metres
        self.scan_steps = 20.0
        self.PclData = None
        self.finger_width = 0.014 # metres
        self.finger_depth = 0.034 
        self.finger_gap_close = 0.0508 # metres
        self.finger_gap_open = 0.0762 # metres
        self.in_sensor_spacing = self.finger_depth/2.0 # metres
        
        self.MOVE_ERROR = 0
        self.MOVE_SUCCESS = 1
        self.FULL_GRASP = 2
        self.NO_GRASP = 3
        self.COL_SCAN_COMPLETE = 4
        self.UNKNOWN_ERR = 5
        self.ZERO_TOUCH = 6
        
    def set_object_camera_info(self, camera_x, camera_y, camera_z,
                               obj_height, obj_width):
        self.camera_x = camera_x
        self.camera_y = camera_y
        self.camera_z = camera_z
        self.obj_height = obj_height
        self.obj_width = obj_width
        self.cur_x = camera_x
        self.cur_y = camera_y
        self.cur_z = camera_z
        
    def get_object_camera_xyz(self):
        return self.camera_x, self.camera_y, self.camera_z

    def get_obj_dimensions(self):
        return self.obj_height, self.obj_width
        
    def count_l_sensors_touched(self):
        # returns two values
        # - number of nail sensors and number of inner sensors in contact
        out_sensors = 0
        in_sensors = 0
        s = 0
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
        # - number of nail sensors and number of inner sensors in contact
        in_sensors = 0
        out_sensors = 0
        s = 0
        for s in range(8):
            if s == 3 or s == 4:
                if self.r_sensors_on[s]:
                    in_sensors += 1
            else:
                if self.r_sensors_on[s]:
                    out_sensors += 1
        return in_sensors, out_sensors
    
    def open_jaw_full(self):
        gripper = intera_interface.Gripper('right_gripper')
        offset_pos = gripper.MAX_POSITION
        cmd_pos = max(min(gripper.get_position() + offset_pos,
                          gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        print("commanded position set to {0} m".format(cmd_pos))
                
    def close_jaw_full(self ):
        gripper = intera_interface.Gripper('right_gripper')
        offset_pos = gripper.MIN_POSITION
        cmd_pos = max(min(gripper.get_position() + offset_pos,
                          gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        print("commanded position set to {0} m".format(cmd_pos))
        
        
    def open_jaw_incr(self, offset_pos):
        gripper = intera_interface.Gripper('right_gripper')
        cmd_pos = max(min(gripper.get_position() + offset_pos,
                          gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        print("commanded position set to {0} m".format(cmd_pos))
        
        
    def close_jaw_incr(self, offset_pos):
        gripper = intera_interface.Gripper('right_gripper')
        cmd_pos = max(min(gripper.get_position() - offset_pos,
                          gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        print("commanded position set to {0} m".format(cmd_pos))
        
    def get_fingers_touched(self, ):
        l1, l2 = self.count_l_sensors_touched()
        r1, r2 = self.count_r_sensors_touched()
        # value is 0, means the finger hasnt touched, and is positive otherwise
        return l1+l2, r1+r2
    
    def calc_gripper_orientation(self, obj_height):
        if obj_height <= self.OBJ_HEIGHT_LOW:
            self.ee_orientation = 1
        else:
            self.ee_orientation = 0
        return self.ee_orientation
       
    def deepen_grasp(self):
        
        
    def move_l_and_in(self):
        # called when L outer sensor is in contact
        # call to this is followed by call to deepen_grasp()
        status = self.goto_cartesian(self.cur_x + self.finger_width,
                                     self.cur_y, self.cur_z)
        if status != self.MOVE_SUCCESS:
            return status
        status = self.goto_cartesian(self.cur_x,
                                     self.cur_y + self.in_sensor_spacing/2.0,
                                     self.cur_z)
        if status != self.MOVE_SUCCESS:
            return status
        
    def move_r_and_in(self):
        # called when R outer sensor is in contact
        status = self.goto_cartesian(self.cur_x - self.finger_width,
                                     self.cur_y, self.cur_z)
        if status != self.MOVE_SUCCESS:
            return status
        status = self.goto_cartesian(self.cur_x,
                                     self.cur_y + self.in_sensor_spacing/2.0,
                                     self.cur_z)
        if status != self.MOVE_SUCCESS:
            return status
        
    def iterate_sense_touch(self):
        l1, l2 = self.count_l_sensors_touched()
        r1, r2 = self.count_r_sensors_touched()
        l_cnt = l1 + l2
        r_cnt = r1 + r2
        print("Left sensor state {}".format(self.str_l_sensor_state))
        print("Right sensor state {}".format(self.str_r_sensor_state))
        if l_cnt > 0 and r_cnt > 0:
            # both fingers have touched, this is a case of the object
            # touching both fingers with only the inner sensors
            print("both fingers have touched")
            if self.deep_l_sensors_touching() or self.deep_r_sensors_touching():
                print("we have the complete grasp")
                return (self.FULL_GRASP)
            elif self.mid_l_sensors_touching() or self.mid_r_sensors_touching():
                if self.shallow_l_sensors_touching() or
                self.shallow_r_sensors_touching():
                    # middle and shallow inner sensors are touching 
                    print("try deepening the grasp from mid")
                    return self.deepen_grasp()
                else:
                    # this is the best case as it is a thin object and wouldnt
                    # touch more than 2 sensors at a time
                    print("best contact is touching middle inner sensors only")
                    return self.FULL_GRASP
            else:
                print("try deepening the grasp from shallow")
                return self.deepen_grasp()
            
        elif l_cnt > 0 and r_cnt == 0:
            # only the left finger has touched
            print("only left finger has touched")
            if l1 > 0 and l2 == 0:
                # case when only left outer sensors is touching 
                self.move_l_and_in()
                return self.deepen_grasp()
            elif l1 == 0 and l2 > 0:
                # case when only left inner sensors are touching
                return self.deepen_grasp()
            elif l1 > 0 and l2 > 0:
                # case when both left sensors are touching
                print("reached an impossible case")
                return self.UNKNOWN_ERR
        elif l_cnt == 0 and r_cnt > 0:
            # only the right finger has touched
            print("only right finger has touched")
            if r1 > 0 and r2 == 0:
                # case when only right outer sensors is touching
                self.move_r_and_in()
                return self.deepen_grasp()
            elif r1 == 0 and r2 > 0:
                # case when only right inner sensors are touching
                return self.deepen_grasp()
            elif r1 > 0 and r2 > 0:
                # case when both right sensors are touching
                print("reached an impossible case")
                return self.UNKNOWN_ERR
        else:
            print("No finger sensors are touching at {}".format(zd))
            return self.ZERO_TOUCH
        
    def goto_cartesian(self, x, y, z):
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
        poseStamped = PoseStamped()
        poseStamped.pose = pose
        waypoint.set_cartesian_pose(poseStamped, 'right_hand', [])

        print('Sending waypoint: \n%s', waypoint.to_string())

        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=args.timeout)
        if result is None:
            print('Trajectory FAILED to send')
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
            
            
    def scan_vertical_incr(self, zd):
        # Srikanth: Remember to build case for tightening only to obj_width
        i = 1
        gripper = intera_interface.Gripper('right_gripper')
        num_steps = self.scan_steps
        percent_delta = 1.0 / num_steps
        position_increment =
        (gripper.MAX_POSITION - gripper.MIN_POSITION) * percent_delta

        while i <= num_steps:
            offset_pos = i*position_increment
            self.close_jaw_incr(offset_pos)
            if iterate_sense_touch() == self.FULL_GRASP:
                return self.FULL_GRASP
            i += 1
        return self.NO_GRASP
    
    def scan_column(self):
        i = 1
        while True:
            zd = self.cur_z - i*(self.z_increment)
            if zd < 0:
                return self.COL_SCAN_COMPLETE
            self.open_jaw_full()
            success = self.goto_cartesian(self.cur_x, self.cur_y, zd)
            if not success:
                print("goto_cartesian failed for {},{},{}".format(self.cur_x,
                                                                  self.cur_y,
                                                                  zd))
            else:
                self.cur_z = zd
            status = self.scan_vertical_incr(zd)
            # handle failure(?)
            if status == self.FULL_GRASP:
                return self.FULL_GRASP
            i += 1

    def goto_search_box():
        return self.goto_cartesian(self.search_boxes[self.cur_search_box][0],
                                   self.search_boxes[self.cur_search_box][1],
                                   self.camera_z)
    
    def iterate_search_boxes(self):
        num_boxes = len(self.search_boxes)
        if self.cur_search_box+1 >= num_boxes:
            return False
        else:
            self.cur_search_box += 1
            return True
        
    def create_search_boxes(self):
        lateral_incr = self.finger_gap_open/self.finger_width
        num_lateral = 2 * int(lateral_incr)
        depth_incr = self.finger_depth/self.in_sensor_spacing
        num_depth = 2 * int(depth_incr)
        # the very first one is the camera given x,y
        self.search_boxes.append([self.camera_x, self.camera_y])

        for i in range(num_lateral/2):
            for j in range(num_depth/2):
                # first move to the decrement side of x-axis
                self.search_boxes.append([self.camera_x - i*lateral_incr, self.camera_y - j*depth_incr])
        for i in range(num_lateral/2):
            for j in range(num_depth/2):
                # then move to the increment side of x-axis
                self.search_boxes.append([self.camera_x + i*lateral_incr, self.camera_y - j*depth_incr])
        
    def grasp(self):
        #This is the true starting point of the touch search and grasp algo
        self.create_search_boxes()
        while self.iterate_search_boxes():
            if self.goto_search_box() == self.MOVE_ERROR:
                print("Move failed") 
                exit() # think about what else could be done instead of quitting
            status = self.scan_column()
            if status == self.FULL_GRASP:
                self.pickup_object()
                exit()
        
            
    def touch_l_update(self, msg):
        self.l_sensors_on[0] = msg.sensor1
        self.l_sensors_on[1] = msg.sensor2
        self.l_sensors_on[2] = msg.sensor3
        self.l_sensors_on[3] = msg.sensor4
        self.l_sensors_on[4] = msg.sensor5
        self.l_sensors_on[5] = msg.sensor6
        self.l_sensors_on[6] = msg.sensor7
        self.l_sensors_on[7] = msg.sensor8
        
    def touch_r_update(self, msg):
        self.r_sensors_on[0] = msg.sensor1
        self.r_sensors_on[1] = msg.sensor2
        self.r_sensors_on[2] = msg.sensor3
        self.r_sensors_on[3] = msg.sensor4
        self.r_sensors_on[4] = msg.sensor5
        self.r_sensors_on[5] = msg.sensor6
        self.r_sensors_on[6] = msg.sensor7
        self.r_sensors_on[7] = msg.sensor8

    def pclData_update(self, msg):
        if self.PclData is None: # store only once for the very first time
            self.set_object_camera_info(msg.centroid.x,
                                        msg.centroid.y,
                                        msg.centroid.z,
                                        msg.height, msg.width)
            self.PclData = True 
            self.grasp()
            
        
# the starting point of the x_camera, y_camera, z_camera should be
# iterate_sense_touch and if that fails then it should start vertical scan
# and if that fails then do adjacent box level iteration

def main():
    rospy.init_node('touch_search')
    ts = TouchSearch()
    rospy.Subscriber('/left_finger/touch', FingerTouch, ts.touch_l_update)
    rospy.Subscriber('/right_finger/touch', FingerTouch, ts.touch_r_update)
    rospy.Subscriber('tgrasp/pclData2', PclData, ts.pclData_update)
    
    '''
    rospy.Subscriber('/left_finger/sai', FingerSAI, ts.sai_l_callback)
    rospy.Subscriber('/right_finger/sai', FingerSAI, ts.sai_r_callback)
    rospy.Subscriber('/left_finger/fai', FingerFAI, ts.fai_l_callback)
    rospy.Subscriber('/right_finger/fai', FingerFAI, ts.fai_r_callback)
    '''
    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")
        
