#!/usr/bin/env python
#-*- coding: utf-8 -*-

import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class Odom(object):
    def __init__(self, name, child_frame_id, frame_id="map"):
        self.pub = rospy.Publisher(name, Odometry, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.name = name
        self.child_frame_id = child_frame_id
        self.frame_id = frame_id
        self.seq = 0

    def publish(self, x, y, z, yaw):
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        #self.odom_broadcaster.sendTransform((x, y, z), odom_quat, curr_time, "base_link", "odom")

        odom = Odometry()
        odom.header.seq = self.seq
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = "base_link"
        odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))
        self.pub.publish(odom)
