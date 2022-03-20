#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Author     : Lin Cong
# E-mail     : cong@informatik.uni-hamburg.de
# Description: Online finger tip tracking controller
# Date       : 10/03/2020: 20:23
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
# FREQUENCY = 50
COLOR_MAP = plt.get_cmap("rainbow")

class VisualizeTip():
    """
        Hand IK solution with velocity limitations,
        Given target key points, return target joints value
        for the next time step, according to the current robot state.
        Used as the solution in the online controller.
        Provide the joint targets for only next time step.
    """
    def __init__(self):
        self.target_mapping_links = LH_TIP_MOVEGROUP
        self.finger_tip_pose = list()
        # finger tip visualization
        self.finger_tip_marker_array_pub = rospy.Publisher('hand_marker_array', MarkerArray, queue_size = 1)
        # Previous Request
        self.sub = rospy.Subscriber("hand_pose", HandFullPose, self.visualize)

        rospy.spin()


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

    def visualize(self, HandFullPose):
        # visualization
        marker_array = MarkerArray()
        for i in range(len(LH_TIP_MOVEGROUP)):
            # target marker visualization
            marker = self.make_marker(HandFullPose, FINGER_NAME_UNITY[i])
            color_depth = (i + 1)/float(len(LH_TIP_MOVEGROUP))
            marker.color.a = 1
            marker.color.r = COLOR_MAP(color_depth)[0]
            marker.color.g = COLOR_MAP(color_depth)[1]
            marker.color.b = COLOR_MAP(color_depth)[2]
            marker.id = i
            marker_array.markers.append(marker)

        self.finger_tip_marker_array_pub.publish(marker_array)


    @staticmethod
    def make_marker(HandFullPoseMsg, finger_tip):
        marker = Marker()
        marker.action = marker.ADD
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.get_rostime()
        marker.lifetime = rospy.Duration(5)
        marker.type = 2
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.pose = getattr(HandFullPoseMsg, finger_tip)

        return marker






def tracking():
    # ros node and service
    # controll parameters initialization and HandIK
    rospy.init_node("finger_tip_visualization")
    v = VisualizeTip()





if __name__ == '__main__':
    tracking()
