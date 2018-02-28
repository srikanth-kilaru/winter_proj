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


def main():
    
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        "-p", "--position", type=float,
        nargs='+',
        help="Desired end position: X, Y, Z")
    
    args = parser.parse_args(rospy.myargv()[1:])
    
    rospy.init_node('ee_control')

    limb = Limb()
    print("LIMB\n", limb)
    
    traj_options = TrajectoryOptions()
    print("TRAJ_OPTIONS\n", traj_options)
    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN

    traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
    print("TRAJ\n", traj)

    wpt_opts = MotionWaypointOptions()
    print("WAYPOINT_OPTIONS\n", wpt_opts)
    
    waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
    print("WAYPOINT\n", waypoint)
    
    joint_names = limb.joint_names()
    endpoint_state = limb.tip_state('right_hand')
    print("ENDPOINT_STATE\n", endpoint_state)

    pose = endpoint_state.pose
    pose.position.x = args.position[0]
    pose.position.y = args.position[1]
    pose.position.z = args.position[2]
    print("POSE\n", pose)

    poseStamped = PoseStamped()
    poseStamped.pose = pose

    waypoint.set_cartesian_pose(poseStamped, 'right_hand', [])
    
    rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())
    
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
