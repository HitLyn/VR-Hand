#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Author     : Lin Cong
# E-mail     : cong@informatik.uni-hamburg.de
# Description: TAMS f329 arm motion planning server
# Date       : 12/03/2022: 20:25
# File Name  : mover

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

joint_names = [
 'arm_shoulder_pan_joint',
 'arm_shoulder_lift_joint',
 'arm_elbow_joint',
 'arm_wrist_1_joint',
 'arm_wrist_2_joint',
 'arm_wrist_3_joint']


# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan


def plan(move_group, target_joint_angles, start_joint_angles):
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_joint_value_target(target_joint_angles) # JointStateMsg
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(target_joint_angles, target_joint_angles)
        raise Exception(exception_str)

    return planCompat(plan)

def plan_trajectory(req):



    response = plan()

    return response



def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_moveit_server')

    s = rospy.Service('arm_moveit', MoverService, plan_trajectory)
    print("Ready to plan for arm")
    rospy.spin()



if __name__ == '__main__':
    moveit_server()
