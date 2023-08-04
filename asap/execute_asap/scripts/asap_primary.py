#!/usr/bin/env python
"""
# asap_primary.py

Python interface to set options for primary Astrobee.

Keenan Albee and Charles Oestreich, 2021
MIT Space Systems Laboratory
"""

import time
import rospy
import rospkg
import math
import argparse
from std_msgs.msg import String

rospack = rospkg.RosPack()
DATA_PATH = rospack.get_path("data") + "/"

def primary_execute_test( bee_topic_prefixes, test_number,ground='false', sim='false',primary_robot_name="",secondary_robot_name="" ,goal="0,0,0"):
    """
    Run a primary test.
    """
    # Set baseline parameters
    rospy.set_param('/asap/ground', ground)  # options are: ['false', 'true']
    rospy.set_param('/asap/sim', sim)  # options are: ['false', 'true']
    #primary_robot_name="" #'/queen'
    #secondary_robot_name="" #'/bumble'
    if primary_robot_name!="" and primary_robot_name!="/":
        primary_robot_name="/"+primary_robot_name
    else:
        primary_robot_name=""

    if secondary_robot_name!="" and secondary_robot_name!="/":
        secondary_robot_name="/"+secondary_robot_name
    else:
        secondary_robot_name=""

    rospy.set_param('/asap/primary_robot_name',primary_robot_name)
    rospy.set_param('/asap/secondary_robot_name',secondary_robot_name)
    goal_str=goal.split(",")

    if (ground == "true"):
        x=1.0
        y=1.0
        z=1.0
        r_RI = [0.0, 0.0, 0.0]  # the test volume reference frame (TVR) wrt INERTIAL frame
        r_CR = [0.0, 0.6, -0.7]  # primary position wrt TVR frame
        q_CR = [0.0, 0.0, -0.7071068, 0.7071068]  # quaternion [qx qy qz qw wrt TVR frame]
        r_TR = [0.0, -0.5, -0.7]  # target position [x y z] wrt TVR frame
        set_params_IC(x,y,z, q_CR)  # an example of setting initial conditions
    else:
        x=float(goal_str[0])
        y=float(goal_str[1])
        z=float(goal_str[2])
        # r_RI_ISS = [10.9, -6.65, 4.9]  # the test volume reference frame (TVR) wrt INERTIAL frame
        r_RI_ISS = [10.8, -9.75, 4.8]  # Point A (TVR)
        r_CR = [0.0, 0.0, 0.0]  # primary position wrt TVR frame
        q_CR = [0, 0, -0.7071068, 0.7071068]  # quaternion [qx qy qz qw wrt TVR frame]
        r_TR = [0.0, 0.0, 0.0]  # target position [x y z] wrt TVR frame
        set_params_IC(x,y,z,q_CR)  # an example of setting initial conditions


def set_params_IC(x, y, z,q_CR):
    # wrt INERTIAL frame
    rospy.set_param('/asap/primary/x_start', x)  # x
    rospy.set_param('/asap/primary/y_start', y)  # y
    rospy.set_param('/asap/primary/z_start', z)  # z
    rospy.set_param('/asap/primary/qx_start', q_CR[0])  # x
    rospy.set_param('/asap/primary/qy_start', q_CR[1])  # y
    rospy.set_param('/asap/primary/qz_start', q_CR[2])  # z
    rospy.set_param('/asap/primary/qw_start', q_CR[3])  # w