#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Author     : Lin Cong
# E-mail     : cong@informatik.uni-hamburg.de
# Description: Online finger tip tracking controller
# Date       : 07/03/2022: 20:23
# File Name  : tracking

import sys
import rospy
import rospy
import bio_ik_msgs.srv
import bio_ik_msgs.msg
import moveit_msgs.msg
import moveit_commander
import trajectory_msgs.msg
import sensor_msgs.msg
from vr_hand.msg import HandTipPose, HandFullPose
from visualization_msgs.msg import Marker, MarkerArray
import matplotlib.pyplot as plt


import numpy as np
from IPython import embed

MOVE_GROUP = "arm_and_hand"
LH_TIP_MOVEGROUP = ["lh_thtip", "lh_fftip", "lh_mftip", "lh_rftip", "lh_lftip", "lh_wrist"] # ROS movegroup links
ARM_INITIAL_POSE = [1.57, -1.5707, 2.4127, -0.8727, 1.6709, 3.1416]
ARM_JOINT_NAME = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_joint", "arm_wrist_1_joint", "arm_wrist_2_joint", "arm_wrist_3_joint"]
FINGER_NAME_UNITY= {0: "thumb_tip", 1: "index_tip", 2: "middle_tip", 3: "ring_tip", 4: "pinky_tip", 5: "wrist"} # key points on Unity side
FREQUENCY = 50
COLOR_MAP = plt.get_cmap("rainbow")

class HandIK():
    """
        Hand IK solution with velocity limitations,
        Given target key points, return target joints value
        for the next time step, according to the current robot state.
        Used as the solution in the online controller.
        Provide the joint targets for only next time step.
    """
    def __init__(self):
        # initialize moveit stuff
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.arm_move_group = moveit_commander.MoveGroupCommander("arm")
        # embed();exit()
        self.get_bio_ik = rospy.ServiceProxy("/bio_ik/get_bio_ik", bio_ik_msgs.srv.GetIK)
        self.joint_state_publisher_unity = rospy.Publisher('/unity_hand_control', sensor_msgs.msg.JointState, queue_size = 1)
        self.joint_state_publisher_rviz = rospy.Publisher('/move_group/fake_controller_joint_states', sensor_msgs.msg.JointState, queue_size = 1)

        self.target_mapping_links = LH_TIP_MOVEGROUP
        self.mapping_weights = 1.*np.array([1., 1., 1., 1., 1., 1.])
        self.joint_position_weights = 0.9*np.array([1, 1, 1, 1, 1, 1])
        self.joints_target = None
        self.finger_tip_pose = list()
        # call back for HandIK
        self.sub = rospy.Subscriber("hand_pose", HandFullPose, self.call_ik)

        rospy.spin()

    def call_ik(self, HandFullPose):
        # call IK and update joints target
        # embed();exit()
        if len(self.finger_tip_pose) > 0:
            del self.finger_tip_pose[:]

        for i in range(len(self.target_mapping_links)):
            finger_pose = getattr(HandFullPose, FINGER_NAME_UNITY[i])
            self.finger_tip_pose.append(finger_pose)

        # IK solution
        request = bio_ik_msgs.msg.IKRequest()
        request.group_name = "arm_and_hand"
        request.timeout.secs = 0.1
        request.approximate = True
        request.robot_state = self.robot.get_current_state()

        for i, link_name in enumerate(self.target_mapping_links):

            # position_goal
            position_goal = bio_ik_msgs.msg.PositionGoal()
            position_goal.link_name = link_name
            position_goal.position = self.finger_tip_pose[i].position
            position_goal.weight = self.mapping_weights[i]
            request.position_goals.append(position_goal)


        request.minimal_displacement_goals.append(bio_ik_msgs.msg.MinimalDisplacementGoal(1.0, False))
        # request.avoid_collisions = True
        response = self.get_bio_ik(request).ik_response

        # publish joints target
        target_joint_state = response.solution.joint_state
        self.joint_state_publisher_unity.publish(target_joint_state)
        # self.joint_state_publisher_rviz.publish(target_joint_state)


    def get_joints_target(self):

        return self.joints_target


    @staticmethod
    def display_trajectory(response):
        display = moveit_msgs.msg.DisplayTrajectory()
        display.trajectory_start = response.solution
        display.trajectory.append(moveit_msgs.msg.RobotTrajectory())
        display.trajectory[0].joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
        display.trajectory[0].joint_trajectory.points[-1].time_from_start.secs = 0
        display.trajectory[0].joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
        display.trajectory[0].joint_trajectory.points[-1].time_from_start.secs = 1
        display_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,
                                            latch=True, queue_size=10)
        display_publisher.publish(display)


def tracking():
    # ros node and service
    rospy.init_node("hand_control")
    rospy.wait_for_service("/bio_ik/get_bio_ik", timeout = 5.0)

    # controll parameters initialization and HandIK
    hand_ik = HandIK()


if __name__ == '__main__':
    tracking()
