#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import tf
import itertools

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from separation_axis_theorem import separating_axis_theorem, get_vertice_rect

class collision_check_marker():
    def __init__(self):
        self.object_data = {}
        self.marker_msg = {}
        self.markerarray_pub = rospy.Publisher("cars", MarkerArray, queue_size=1)
        rospy.Subscriber("driving", Marker, self.callback)
        rospy.Subscriber("parking", MarkerArray, self.callback_parking)

    def callback(self, msg):
        id = str(msg.id)
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        yaw = tf.transformations.euler_from_quaternion(quat)[2]
        self.object_data[id] = (msg.pose.position.x, msg.pose.position.y, yaw, msg.scale.x, msg.scale.y)
        self.marker_msg[id] = msg

    def callback_parking(self, msgs):
        for msg in msgs.markers:
            self.callback(msg)

    def collision_check_and_publish(self):
        if len(self.object_data) >= 2 and "1" in self.object_data:
            driving_object_vertices = get_vertice_rect(self.object_data["1"])
            for id in self.object_data:
                if id != "1":
                    parking_object_vertices = get_vertice_rect(self.object_data[id])
                    if separating_axis_theorem(driving_object_vertices, parking_object_vertices):
                        self.marker_msg["1"].color = ColorRGBA(192/255.0, 57/255.0,43/255.0, 0.97)
                        self.marker_msg[id].color = ColorRGBA(192/255.0, 57/255.0,43/255.0, 0.97)
                        break

        self.markerarray_pub.publish(MarkerArray(self.marker_msg.values()))


if __name__ == "__main__":

    rospy.init_node("collision_checking_marker_node")
    r = rospy.Rate(30)

    collision_check = collision_check_marker()
    while not rospy.is_shutdown():
        collision_check.collision_check_and_publish()
        r.sleep()
