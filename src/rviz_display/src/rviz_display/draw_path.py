#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class PathDrawer:
    def __init__(self, frame_id="/map"):
        self.pub = rospy.Publisher("path", Marker, queue_size=1) 
        self.frame_id = frame_id
    def draw(self, x, y):
        marker = Marker(
                    ns = "path",
                    type=Marker.LINE_STRIP,
                    id=1,
                    lifetime = rospy.Duration(1.5),
                    scale=Vector3(0.4, 0, 0),
                    header=Header(frame_id = self.frame_id),
                    color=ColorRGBA(1.0, 0.0, 0.0, 0.8))
        for _x, _y in zip(x,y):
            point = Point(_x,_y, 0)
            marker.points.append(point)
        self.pub.publish(marker)

class PointDrawer:
    def __init__(self, frame_id="/map"):
        self.pub = rospy.Publisher("point", Marker, queue_size=1) 
        self.frame_id = frame_id

    def draw(self, x , y, z=0):
        marker = Marker(
                    ns = "obstacles",
                    type=Marker.SPHERE,
                    id=1,
                    lifetime = rospy.Duration(1.5),
                    pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(1, 1, 1),
                    header=Header(frame_id=self.frame_id),
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        self.pub.publish(marker)

class ArrowDrawer:
    def __init__(self,  frame_id="/map"):
        self.pub = rospy.Publisher("arrow", Marker, queue_size=1) 
        self.frame_id = frame_id

    def draw(self, x1 , y1, z1, x2, y2, z2):
        marker = Marker(
                    ns = "orientation",
                    id = 1,
                    type = Marker.ARROW,
                    lifetime = rospy.Duration(1.5),
                    points = [Point(x1, y1, z1), Point(x2, y2, z2)],
                    scale = Vector3(0.1, 0.5, 0.3),
                    header=Header(frame_id=self.frame_id),
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        self.pub.publish(marker)