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
from IPython import embed

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


def display_trajectory(robot, response):
    # embed();exit()
    display = moveit_msgs.msg.DisplayTrajectory()
    display.trajectory_start = robot.get_current_state()
    display.trajectory.append(response.trajectory)
    display_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,
                                        latch=True, queue_size=10)
    display_publisher.publish(display)


def plan(move_group, target_joint_angles, start_joint_angles):
    # current robot state
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    # target joint state
    target_joint_state = JointState()
    target_joint_state.name = joint_names
    target_joint_state.position = target_joint_angles
    move_group.set_joint_value_target(target_joint_state) # JointStateMsg

    # plan
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(target_joint_angles, target_joint_angles)
        raise Exception(exception_str)

    return planCompat(plan)

def plan_trajectory(req):
    response = MoverServiceResponse()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    robot = moveit_commander.RobotCommander()

    current_robot_joint_configuration = req.joints_input.joints
    target_robot_joint_configuration = req.joints_target.joints

    robot_trajectory = plan(move_group, target_robot_joint_configuration, current_robot_joint_configuration)
    print('Successfully planned to target joint state')

    response.trajectory = robot_trajectory

    # visualization trajectory
    display_trajectory(robot, response)

    return response



def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_moveit_server')

    s = rospy.Service('arm_moveit', MoverService, plan_trajectory)
    print("Ready to plan for arm")
    rospy.spin()



if __name__ == '__main__':
    moveit_server()
