#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Author     : Lin Cong
# E-mail     : cong@informatik.uni-hamburg.de
# Description: Online finger tip tracking controller
# Date       : 30/12/2020: 20:23
# File Name  : tracking

import sys
import rospy
import rospy
import bio_ik_msgs.srv
import bio_ik_msgs.msg
import moveit_msgs.msg
import moveit_commander
import trajectory_msgs.msg
from vr_hand.msg import HandTipPose
from visualization_msgs.msg import Marker, MarkerArray

from vr_hand.msg import HandTipPose

import numpy as np
from IPython import embed

LH_TIP_MOVEGROUP = ["lh_thtip", "lh_fftip", "lh_mftip", "lh_rftip", "lh_lftip", "lh_wrist"] # ROS movegroup links
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
        self.target_mapping_links = LH_TIP_MOVEGROUP
        self.mapping_weights = [1., 1., 1., 1., 1.]
        self.joints_target =
        # finger tip visualization
        self.finger_tip_marker_array_pub = rospy.Publisher('finger_tip_marker_array', MarkerArray, queue_size = 10)

        # call back for visualization and IK
        self.sub = rospy.Subscriber("finger_tip_pose", HandTipPose, self.visualize_and_call_ik)


    def visualize_and_call_ik(self, HandTipPose):

        marker_array = MarkerArray()
        for i in range(len(LH_TIP_MOVEGROUP)):
            # target marker visualization
            marker = self.make_marker(HandTipPoseMsg, FINGER_NAME_UNITY[i])
            color_depth = (i + 1)/5.
            marker.color.a = 1
            marker.color.r = COLOR_MAP(color_depth)[0]
            marker.color.g = COLOR_MAP(color_depth)[1]
            marker.color.b = COLOR_MAP(color_depth)[2]
            marker.id = i
            marker_array.markers.append(marker)

            # compute target vector and call IK
            link_current_pose

        self.finger_tip_marker_array_pub.publish(marker_array)

        # call IK and update joints target



        # publish joints target



        rospy.sleep(1./FREQUENCY)





    def get_joints_target(self):

        return self.joints_target



    @staticmethod
    def make_marker(HandTipPoseMsg, finger_tip):
        marker = Marker()
        marker.action = marker.ADD
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.get_rostime()
        marker.lifetime = rospy.Duration(5)
        marker.type = 2
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.pose = getattr(HandTipPoseMsg, finger_tip)

        return marker









def tracking():
    # ros node and service
    rospy.init_node("hand_tracking")
    rospy.wait_for_service("/bio_ik/get_bio_ik", timeout = 5.0)

    # controll parameters initialization and HandIK
    hand_ik = HandIK()











if __name__ == '__main__':
    tracking()
