#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Author     : Lin Cong
# E-mail     : cong@informatik.uni-hamburg.de
# Description: synchronize the robot from unity
# Date       : 18/03/2022: 14:32
# File Name  : synchronize unity

import sys
import rospy
import rospy
import bio_ik_msgs.srv
import bio_ik_msgs.msg
import moveit_msgs.msg
import moveit_commander
import trajectory_msgs.msg
import sensor_msgs.msg
from vr_hand.msg import HandTipPose, HandFullPose, JointPositions
from visualization_msgs.msg import Marker, MarkerArray
import matplotlib.pyplot as plt


import numpy as np
from IPython import embed

MOVE_GROUP = "arm_and_hand"
name_list = ["arm_shoulder_pan_joint", "arm_shoulder_lift_joint", "arm_elbow_joint", "arm_wrist_1_joint",
  "arm_wrist_2_joint", "arm_wrist_3_joint", "lh_WRJ2", "lh_WRJ1",
  "lh_FFJ4", "lh_FFJ3", "lh_FFJ2", "lh_FFJ1",
  "lh_LFJ5", "lh_LFJ4", "lh_LFJ3", "lh_LFJ2", "lh_LFJ1",
  "lh_MFJ4", "lh_MFJ3", "lh_MFJ2", "lh_MFJ1",
  "lh_RFJ4", "lh_RFJ3", "lh_RFJ2", "lh_RFJ1",
  "lh_THJ5", "lh_THJ4", "lh_THJ3", "lh_THJ2", "lh_THJ1"]

class Synchronize():
    """
        synchronize the robot in rviz from unity
    """
    def __init__(self):
        # moveit_commander.roscpp_initialize(sys.argv)
        # self.robot = moveit_commander.RobotCommander()
        # self.arm_move_group = moveit_commander.MoveGroupCommander("arm_and_hand")
        self.joint_state_pub = rospy.Publisher('/move_group/fake_controller_joint_states', sensor_msgs.msg.JointState, queue_size = 1)
        self.sub = rospy.Subscriber("/joint_positions", JointPositions, self.synchronize, queue_size = 1)
        rospy.spin()

    def synchronize(self, joint_positions):
        # synchronize
        joint_state = sensor_msgs.msg.JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.position = joint_positions.position
        # embed();exit()
        joint_state.name = name_list
        # embed();exit()

        self.joint_state_pub.publish(joint_state)
        # print('pub')




def synchronize():
    rospy.init_node("synchronize_unity")
    synchronization = Synchronize()





if __name__ == '__main__':
    synchronize()
