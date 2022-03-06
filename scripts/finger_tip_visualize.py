#!/usr/bin/env python2
import rospy
import geometry_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from vr_hand.msg import HandTipPose
import numpy as np
import matplotlib.pyplot as plt
from IPython import embed

MARKER_DICT = {0: "thumb_tip", 1: "index_tip", 2: "middle_tip", 3: "ring_tip", 4: "pinky_tip"}
COLOR_MAP = plt.get_cmap("rainbow")
finger_tip_marker_array_pub = None

def finger_tip_read_and_show(HandTipPoseMsg):
    # get marker array
    global finger_tip_marker_array_pub
    marker_array = MarkerArray()
    for i in range(5):
        marker = make_marker(HandTipPoseMsg, MARKER_DICT[i])
        color_depth = (i + 1)/5.

        marker.color.a = 1
        marker.color.r = COLOR_MAP(color_depth)[0]
        marker.color.g = COLOR_MAP(color_depth)[1]
        marker.color.b = COLOR_MAP(color_depth)[2]
        marker.id = i
        marker_array.markers.append(marker)

    # publish marker array
    # embed();exit()
    finger_tip_marker_array_pub.publish(marker_array)


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

    # embed()
    marker.pose = getattr(HandTipPoseMsg, finger_tip)

    return marker

def make_target_marker(HandTipPoseMsg, finger_tip):
    marker = Marker()
    marker.action = marker.ADD
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.get_rostime()
    marker.lifetime = rospy.Duration(5)
    marker.type = 2
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03

    # embed()
    marker.pose.orientation = getattr(HandTipPoseMsg, finger_tip).orientation
    marker.pose.position.x = getattr(HandTipPoseMsg, finger_tip).position.x + 1.3
    marker.pose.position.y = getattr(HandTipPoseMsg, finger_tip).position.y - 1.4
    marker.pose.position.z = getattr(HandTipPoseMsg, finger_tip).position.z - 0.7

    return marker

class FingerTipVision():
    def __init__(self):
        rospy.init_node('finger_tip_visualize')
        self.finger_tip_marker_array_pub = rospy.Publisher('finger_tip_marker_array', MarkerArray, queue_size = 10)
        self.finger_target_marker_array_pub = rospy.Publisher('finger_tip_target_marker_array', MarkerArray, queue_size = 10)
        self.sub = rospy.Subscriber("finger_tip_pose", HandTipPose, self.finger_tip_read_and_show)
        rospy.spin()


    def finger_tip_read_and_show(self, HandTipPoseMsg):
        # get marker array
        marker_array = MarkerArray()
        target_marker_array = MarkerArray()
        for i in range(5):
            marker = make_marker(HandTipPoseMsg, MARKER_DICT[i])
            target_marker = make_target_marker(HandTipPoseMsg, MARKER_DICT[i])

            color_depth = (i + 1)/5.

            # real hand marker
            marker.color.a = 1
            marker.color.r = COLOR_MAP(color_depth)[0]
            marker.color.g = COLOR_MAP(color_depth)[1]
            marker.color.b = COLOR_MAP(color_depth)[2]
            marker.id = i
            marker_array.markers.append(marker)
            # target marker
            target_marker.color.a = 1
            target_marker.color.r = COLOR_MAP(color_depth)[0]
            target_marker.color.g = COLOR_MAP(color_depth)[1]
            target_marker.color.b = COLOR_MAP(color_depth)[2]
            target_marker.id = i
            target_marker_array.markers.append(target_marker)

        self.finger_tip_marker_array_pub.publish(marker_array)
        self.finger_target_marker_array_pub.publish(target_marker_array)


def main():
    # global finger_tip_marker_array_pub
    # rospy.init_node('finger_tip_visualize')
    # finger_tip_marker_array_pub = rospy.Publisher('finger_tip_marker_array', MarkerArray, queue_size = 10)
    # rospy.Subscriber("finger_tip_pose", HandTipPose, finger_tip_read_and_show)
    # rospy.spin()

    finger_tip_vision = FingerTipVision()


    # BioIK




if __name__ == '__main__':
    main()
