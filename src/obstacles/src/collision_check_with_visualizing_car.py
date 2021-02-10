#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import tf
import itertools
import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Point
from object_msgs.msg import Object
from separation_axis_theorem import separating_axis_theorem, get_vertice_rect


class collision_check_marker():
    def __init__(self, num_objects = None):
        self.object_msg = {}
        self.marker_pub = {}
        self.marker_msg = {}
        self.collision_check = {}
        self.num_objects = num_objects

        for i in range(self.num_objects):
            rospy.Subscriber("/objects/car_" + str(i+1), Object, self.callback_object)
            self.marker_pub[str(i + 1)] = rospy.Publisher("/objects/marker/car_" + str(i+1), Marker, queue_size=1)

    def callback_object(self, msg):
        id = str(msg.id)
        yaw = msg.yaw
        center_x = msg.x + 1.3 * math.cos(yaw)
        center_y = msg.y + 1.3 * math.sin(yaw)
        self.object_msg[id] = (center_x, center_y, yaw, msg.L, msg.W)

    def get_marker_msg(self, msg_tuple, id, is_collide):

        x = msg_tuple[0]
        y = msg_tuple[1]
        yaw = msg_tuple[2]
        L = msg_tuple[3]
        W = msg_tuple[4]

        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        m = Marker()
        m.header.frame_id = "/map"
        m.header.stamp = rospy.Time.now()
        m.id = int(id)
        m.type = m.CUBE

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.75
        m.pose.orientation = Quaternion(*quat)

        m.scale.x = L
        m.scale.y = W
        m.scale.z = 1.645

        if is_collide:
            m.color.r = 192 / 255.0
            m.color.g = 57 / 255.0
            m.color.b = 43 / 255.0
            m.color.a = 0.97
        else:
            m.color.r = 93 / 255.0
            m.color.g = 122 / 255.0
            m.color.b = 177 / 255.0
            m.color.a = 0.97

        return m

    def get_sphere_marker_list_msg(self, vertice_all):
        m = Marker()
        m.header.frame_id = "/map"
        m.header.stamp = rospy.Time.now()
        m.id = 5
        m.type = m.POINTS

        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.5

        m.color.r = 0
        m.color.g = 1
        m.color.b = 0
        m.color.a = 0.97
        for vertice_subset in vertice_all:
            for vertex in vertice_subset:
                point = Point()
                point.x = vertex[0]
                point.y = vertex[1]
                m.points.append(point)
        return m


    def collision_check_and_publish(self):
        ids = self.object_msg.keys()
        for id in ids:
            self.collision_check[id] = False

        if len(ids) >= 2:
            ids_com = itertools.combinations(ids, 2)
            for pair in ids_com:
                first_id = pair[0]
                second_id = pair[1]
                first_object_msg = self.object_msg[first_id]
                second_object_msg = self.object_msg[second_id]
                first_object_vertices = get_vertice_rect(first_object_msg)
                second_object_vertices = get_vertice_rect(second_object_msg)
                is_collide = separating_axis_theorem(first_object_vertices, second_object_vertices)
                if is_collide:
                    self.collision_check[first_id] = True
                    self.collision_check[second_id] = True

        else:
            print("Initialized object is not sufficient")

        for id in ids:
            self.marker_msg[id] = self.get_marker_msg(self.object_msg[id], id, self.collision_check[id])
            self.marker_pub[id].publish(self.marker_msg[id])


if __name__ == "__main__":
    rospy.init_node("collision_checking_marker_node")
    num_objects = 3
    collision_check = collision_check_marker(num_objects)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        collision_check.collision_check_and_publish()
        r.sleep()
