#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import tf
import itertools

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from separation_axis_theorem import separating_axis_theorem, get_vertice_rect

class collision_check_marker():
    def __init__(self, num_objects = None):
        self.object_data = {}
        self.marker_msg = {}
        self.num_objects = num_objects
        self.markerarray_pub = rospy.Publisher("cars", MarkerArray, queue_size=1)
        for i in range(self.num_objects):
            rospy.Subscriber("car" + str(i+1), Marker, self.callback)

    def callback(self, msg):
        id = str(msg.id)
        quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        yaw = tf.transformations.euler_from_quaternion(quat)[2]
        self.object_data[id] = (msg.pose.position.x, msg.pose.position.y, yaw, msg.scale.x, msg.scale.y)
        self.marker_msg[id] = msg

    def collision_check_and_publish(self):
        if len(self.object_data) >= 2:
            for id1, id2 in itertools.combinations(self.object_data, 2):
                first_object_vertices = get_vertice_rect(self.object_data[id1])
                second_object_vertices = get_vertice_rect(self.object_data[id2])
                is_collide = separating_axis_theorem(first_object_vertices, second_object_vertices)
                if is_collide:
                    self.marker_msg[id1].color = ColorRGBA(192/255.0, 57/255.0,43/255.0, 0.97)
                    self.marker_msg[id2].color = ColorRGBA(192/255.0, 57/255.0,43/255.0, 0.97)

        self.markerarray_pub.publish(MarkerArray(self.marker_msg.values()))


if __name__ == "__main__":
    rospy.init_node("collision_checking_marker_node")
    num_objects = 3
    collision_check = collision_check_marker(num_objects)

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        collision_check.collision_check_and_publish()
        r.sleep()
