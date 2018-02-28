#!/usr/bin/env python

import sys
import rospy
import numpy as np
import modern_robotics as mr
import argparse
import actionlib


from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
from tf_conversions import posemath
from intera_interface import Limb
from math import cos, sin, radians

s10 = sin(radians(10)) 
c10 = cos(radians(10))

Blist = np.array([[s10, -c10, 0., -1.0155*c10, -1.0155*s10, -0.1603],
                  [-c10, -s10, 0., -0.9345*s10, 0.9345*c10, 0.],
                  [0. , 0., 1., -0.0322*s10, 0.0322*c10, 0.],
                  [-c10, -s10, 0., -0.5345*s10, 0.5345*c10, 0.],
                  [0., 0., 1., 0.1363*s10, -0.1363*c10, 0.],
                  [-c10, -s10, 0., -0.1345*s10, 0.1345*c10, 0.],
                  [0., 0., 1., 0., 0., 0.]])
Blist = Blist.T


M = np.array([[0., 0., 1., 1.0155],
              [-c10, -s10, 0., 0.1603],
              [s10, -c10, 0., 0.317],
              [0., 0., 0., 1.]])

def ee_xyz(x, y, z):
    limb = Limb()
    #print("LIMB\n", limb)

    traj_options = TrajectoryOptions()
    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
    traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
    #print("TRAJ\n", traj)

    wpt_opts = MotionWaypointOptions()
    #print("WAYPOINT_OPTIONS\n", wpt_opts)
    
    waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
    #print("WAYPOINT\n", waypoint)
    

    
    
def main():
    global M, Blist
    
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        "-p", "--position", type=float,
        nargs='+',
        help="Desired end position: X, Y, Z")
    parser.add_argument(
        "-q", "--joint_angles", type=float,
        nargs='+',
        help="A list of joint angles, one for each of the 7 joints, J0...J6")
    
    args = parser.parse_args(rospy.myargv()[1:])
    
    rospy.init_node('ee_control')

    if args.position is not None and len(args.position) == 3:
        ee_xyz(args.position[0], args.position[1], args.position[2])
    elif args.joint_angles is not None and len(args.joint_angles) == 7:
        ee_angles(args.joint_angles)
        
    limb = Limb()
    #print("LIMB\n", limb)
    
    traj = MotionTrajectory(limb = limb)
    #print("TRAJ\n", traj)

    wpt_opts = MotionWaypointOptions()
    #print("WAYPOINT_OPTIONS\n", wpt_opts)
    
    waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
    #print("WAYPOINT\n", waypoint)

    
    joint_angles = limb.joint_ordered_angles()
    waypoint.set_joint_angles(joint_angles = joint_angles)
    traj.append_waypoint(waypoint.to_msg())
    print("joint_angles=",joint_angles)
    
    if args.joint_angles is not None and len(args.joint_angles) == len(joint_angles):
        print("I am here")
        waypoint.set_joint_angles(joint_angles = args.joint_angles)
    elif args.position is not None and len(args.position) == 3:
        #cartesian corddinates of tip are specified
        '''
        x = -5
        y = 4
        z = 1.6858
        '''
        x = args.position[0]
        y = args.position[1]
        z = args.position[2]

        print("x,y,z=",x,y,z)
        
        T = [[0, 1, 0, x], [1, 0, 0, y], [0, 0, -1, z], [0, 0, 0, 1]]
        thetalist0 = [0, 0.99, 0.314, 1.57, -2.99, 0.9, 0.01]
        eomg = 0.01
        ev = 0.001
        joint_angles, success = mr.IKinBody(Blist,M,T,thetalist0,eomg,ev)
        if not success:
            print("status of call to IkinBody failed")
            exit()
        else:
            waypoint.set_joint_angles(joint_angles = joint_angles)
    else:
        print("wrong input\n")
        
    traj.append_waypoint(waypoint.to_msg())
        
    result = traj.send_trajectory(timeout=None)
    if result is None:
        rospy.logerr('Trajectory FAILED to send')
        return
    
    if result.result:
        rospy.loginfo('Motion controller successfully finished the trajectory!')
    else:
        rospy.logerr('Motion controller failed to complete the trajectory with error %s', result.errorId)
        
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")
