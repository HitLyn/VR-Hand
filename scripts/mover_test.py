#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Author     : Lin Cong
# E-mail     : cong@informatik.uni-hamburg.de
# Description: TAMS f329 arm motion planning server
# Date       : 15/03/2022: 14:19
# File Name  : mover_test

import rospy

import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from vr_hand.srv import MoverService, MoverServiceRequest, MoverServiceResponse
from vr_hand.msg import ArmJoints

joint_names = [
 'arm_shoulder_pan_joint',
 'arm_shoulder_lift_joint',
 'arm_elbow_joint',
 'arm_wrist_1_joint',
 'arm_wrist_2_joint',
 'arm_wrist_3_joint']

# initial_joint_angles = [0., 0., 0., 0., 0., 0.]
initial_joint_angles = [1.57, -1.5707, 2.4127, -0.8727, 1.6709, 2.1416]
target_joint_angles = [1.57, -1.5707, 2.4127, -0.8727, 1.6709, 3.1416]


def moveit_server_test():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_moveit_server_test')
    rospy.wait_for_service('arm_moveit', timeout = 5.0)
    get_moveit_server = rospy.ServiceProxy("arm_moveit", MoverService)

    move_group = moveit_commander.MoveGroupCommander('arm')

    initial_arm_joints = ArmJoints()
    target_arm_joints = ArmJoints()
    initial_arm_joints.joints = initial_joint_angles
    target_arm_joints.joints = target_joint_angles

    request = MoverServiceRequest()
    request.joints_input = initial_arm_joints
    request.joints_target = target_arm_joints

    response = get_moveit_server(request)





if __name__ == '__main__':
    moveit_server_test()
