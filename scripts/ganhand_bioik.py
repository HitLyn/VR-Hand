#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang 
# E-mail     : liang@informatik.uni-hamburg.de
# Description: 
# Date       : 30/12/2020: 20:23
# File Name  : bioik
from __future__ import print_function
import rospy
import bio_ik_msgs.srv
import bio_ik_msgs.msg
import moveit_msgs.msg
from vr_hand.msg import HandTipPose
import trajectory_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from IPython import embed


"""
key_points[0]             : hand base position
key_points[1, 2, 3, 4]    : thumb
key_points[5, 6, 7, 8]    : first finger
key_points[9, 10, 11, 12] : middle finger
key_points[13, 14, 15, 16]: ring finger
key_points[17, 18, 19, 20]: little finger
*******************************************
*            [8]  [12]  [16]  [20]        *
*                                         *
*            [7]  [11]  [15]  [19]        *
*                                         *
*            [6]  [10]  [14]  [18]        *
* [4]                                     *
*   [3]      [5]  [9 ]  [13]  [17]        *
*     [2]    [0000000000000000000]        *
*       [1]  [000 right hand  000]        *
*            [0000000000000000000]        *
*******************************************
"""
LH_TIP_MOVEGROUP = ["lh_thtip", "lh_fftip", "lh_mftip", "lh_rftip", "lh_lftip"]
FINGER_NAME_UNITY= {0: "thumb_tip", 1: "index_tip", 2: "middle_tip", 3: "ring_tip", 4: "pinky_tip"}



class BioIK:
    def __init__(self):
        rospy.init_node("bio_ik_service_client")
        self.finger_tip_pose = []
        rospy.wait_for_service("/bio_ik/get_bio_ik")
        self.get_bio_ik = rospy.ServiceProxy("/bio_ik/get_bio_ik", bio_ik_msgs.srv.GetIK)
        self.map_position_links = LH_TIP_MOVEGROUP
        self.map_position_weights = [1, 1, 1, 1, 1]
        # self.map_direction_weights = [0.1, 0.1, 0.1, 0.1, 0.1] * 3
        self.ik_result = []
        # subscribe to finger tip points
        self.sub = rospy.Subscriber("finger_tip_pose", HandTipPose, self.call_ik)
        rospy.spin()


    def show_marker_array(self, finger_tip_pose, scale):
        pub = rospy.Publisher("key_points_vis", MarkerArray, queue_size=1)
        marker_array = MarkerArray()
        for key_point in finger_tip_pose:
            pose = np.hstack([key_point, np.array([0, 0, 0, 1])])
            self.make_marker(marker_array, pose, scale=scale, color=[1, 0, 0], lifetime=100, frame_id="world")
        marker_id = 0
        for m in marker_array.markers:
            m.id = marker_id
            marker_id += 1
        pub.publish(marker_array)

    def call_ik(self, HandTipPoseMsg):
        # read finger tip positions and update self.key_points
        # clear previous data
        embed();exit()
        if len(self.finger_tip_pose) > 0:
            self.finger_tip_pose.clear()
            for i in range(5):
                finger_pose = getattr(HandTipPoseMsg, FINGER_NAME_UNITY[i])
                self.finger_tip_pose.append(finger_pose)
            
            # IK solution
            request = bio_ik_msgs.msg.IKRequest()
            request.group_name = "arm_and_hand"
            request.timeout.secs = 0.2
            request.approximate = True
            # request.fixed_joints = ["rh_WRJ2", "rh_WRJ1"]
            for i, link_name in enumerate(self.map_position_links):
                goal = bio_ik_msgs.msg.PositionGoal()
                goal.link_name = link_name
                goal.position = self.finger_tip_pose[i].position
                # goal.position.x = self.finger_tip_pose[i].position
                # goal.position.y = self.key_point[1]
                # goal.position.z = self.key_point[2]
                goal.weight = self.map_position_weights[i]
                request.position_goals.append(goal)
            # directions = []
            # for i in range(5):
            #     directions.append(key_points[RH_MIDDLE_ID[i]] - key_points[RH_PROXIMAL_ID[i]])

            # for i, link_name in enumerate(self.map_direction_links):
            #     goal = bio_ik_msgs.msg.DirectionGoal()
            #     goal.link_name = link_name
            #     goal.direction.x = directions[i][0]
            #     goal.direction.y = directions[i][1]
            #     goal.direction.z = directions[i][2]
            #     goal.weight = self.map_direction_weights[i]
            #     request.direction_goals.append(goal)
            request.avoid_collisions = True
            response = self.get_bio_ik(request).ik_response
            # i = 0

            if VIS_IK:
                self.display_trajectory(response)
            joints = np.array(response.solution.joint_state.position)[2:]  # remove wrist
            # ik order: wrist (2), ff(4), lf(5), mf(4), rf(4), th(5)
            # order in synergy:    ff(4), mf(4), rf(4), lf(5), th(5), need to change order!
            joints_new = np.copy(joints)
            self.ik_result.append(joints_new)
        else:
            print('waiting for finger tip poses')
                

    # @staticmethod
    # def make_marker(marker_array, pose, scale, color, lifetime, frame_id):
    #     marker = Marker()
    #     marker.header.frame_id = frame_id
    #     marker.header.stamp = rospy.Time.now()
    #     marker.type = marker.SPHERE
    #     marker.action = marker.ADD
    #     marker.pose = numpy_to_ros_pose(pose)
    #     marker.lifetime = rospy.Duration.from_sec(lifetime)
    #     marker.scale.x = scale
    #     marker.scale.y = scale
    #     marker.scale.z = scale
    #     marker.color.a = 0.5
    #     marker.color.r = color[0]
    #     marker.color.g = color[1]
    #     marker.color.b = color[2]
    #     marker_array.markers.append(marker)

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


if __name__ == "__main__":
    VIS_IK = True
    BioIK()
