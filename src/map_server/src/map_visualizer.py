#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import rospkg
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import pickle


class Converter(object):
    def __init__(self, file_, r, g, b, a, scale, start_id):
        self.file = file_
        self.r = r
        self.g = g
        self.b = b
        self.a = a
        self.scale = scale
        self.start_id = start_id
        with open(file_, "rb") as f:
            self.waypoints = pickle.load(f)
        self.ma = self.make_marker_array()

    def make_marker_array(self):
        ma = MarkerArray()
        draw_step = 1
        if isinstance(self.waypoints, dict):
            # for A2_LINK_smoothed_near0.pkl
            self.waypoints = self.waypoints.values()
            draw_step = 3

        for i, waypoint in enumerate(self.waypoints):
            m = Marker()
            m.id = self.start_id + i
            m.header.frame_id = "/map"
            m.type = m.LINE_STRIP
            m.action = m.ADD

            m.scale.x = self.scale

            m.color.r = self.r
            m.color.g = self.g
            m.color.b = self.b
            m.color.a = self.a

            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1

            m.points = []
            for j in range(len(waypoint["x"])):
                if j % draw_step == 0 or j == len(waypoint["x"])-1:
                    p = Point()
                    p.x = waypoint["x"][j]
                    p.y = waypoint["y"][j]
                    m.points.append(p)

            ma.markers.append(m)
        return ma


if __name__ == "__main__":
    rospy.init_node("map_rviz_visualizer")
    rospack = rospkg.RosPack()
    path = rospack.get_path("map_server")

    # TODO: AS CONFIGURATION FILE
    link_file = path + "/src/route.pkl"
    linemark_file = path + "/src/surface.pkl"

    link_cv = Converter(link_file, r=255/255.0, g=236/255.0, b=139/255.0, a=0.8, scale=0.1, start_id=2000)
    lanemark_cv = Converter(linemark_file, r=228 / 255.0, g=233 / 255.0, b=237 / 255.0, a=0.4, scale=0.1, start_id=3000)

    link_pub = rospy.Publisher("/rviz/lane_links", MarkerArray, queue_size=1, latch=True)
    lanemark_pub = rospy.Publisher("/rviz/lane_markings", MarkerArray, queue_size=1, latch=True)

    rospy.sleep(1)
    while not rospy.is_shutdown():
        link_pub.publish(link_cv.ma)
        lanemark_pub.publish(lanemark_cv.ma)
        rospy.sleep(1)
