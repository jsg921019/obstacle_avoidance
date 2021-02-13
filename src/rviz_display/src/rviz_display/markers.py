#!/usr/bin/env python

import rospy
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


class PathMarker:
    def __init__(self, frame_id="/map"):
        self.frame_id=frame_id
    def convert(self, path):
        marker = Marker(
                    ns = "path",
                    type=Marker.LINE_STRIP,
                    id=1,
                    lifetime = rospy.Duration(0.2),
                    scale=Vector3(0.3, 0, 0),
                    header=Header(frame_id = self.frame_id),
                    color=ColorRGBA(0.0, 8.0, 1.0, 0.8))
        marker.points = [Point(_x,_y, 0.1) for _x, _y in zip(path.x, path.y)]
        return marker

class PathsMarkerArray:
    def __init__(self, frame_id="/map"):
        #self.pub = rospy.Publisher("paths", MarkerArray, queue_size=1) 
        self.frame_id = frame_id

    def convert(self, paths):
        ma = []
        for i, path in enumerate(paths):
            m= Marker(
                ns = "paths",
                type=Marker.LINE_STRIP,
                id = i,
                lifetime = rospy.Duration(0.2),
                scale=Vector3(0.15, 0, 0),
                header=Header(frame_id = self.frame_id),
                color=ColorRGBA(0, 0.6, 0.5, 0.7))
            m.points = [Point(_x,_y, 0) for _x, _y in zip(path.x, path.y)]
            ma.append(m)

        return ma

class PointDrawer:
    def __init__(self, frame_id="/map"):
        self.pub = rospy.Publisher("point", Marker, queue_size=1) 
        self.frame_id = frame_id

    def draw(self, x , y, z=0):
        marker = Marker(
                    ns = "lookahead",
                    type=Marker.SPHERE,
                    id=1,
                    lifetime = rospy.Duration(1.5),
                    pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(1, 1, 1),
                    header=Header(frame_id=self.frame_id),
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        self.pub.publish(marker)
        #return marker

class ArrowDrawer:
    def __init__(self,  frame_id="/map"):
        self.pub = rospy.Publisher("arrow", Marker, queue_size=1) 
        self.frame_id = frame_id

    def draw(self, x1 , y1, z1, x2, y2, z2):
        marker = Marker(
                    ns = "lookahead",
                    id = 1,
                    type = Marker.ARROW,
                    lifetime = rospy.Duration(1.5),
                    points = [Point(x1, y1, z1), Point(x2, y2, z2)],
                    scale = Vector3(0.3, 1, 0.5),
                    header=Header(frame_id=self.frame_id),
                    color=ColorRGBA(1.0, 1.0, 0.0, 0.8))
        self.pub.publish(marker)

class TextMarker:
    def __init__(self):
        pass

    def convert(self, text, size, id=0, color=(1.0, 1.0, 1.0, 1.0)):
        marker = Marker(
            ns = "ui",
            type=Marker.TEXT_VIEW_FACING,
            id=id,
            text = text,
            pose = Pose(Point(35, 10, 0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(1, 1, size),
            header=Header(frame_id="map"),
            color=ColorRGBA(*color))
        return marker

class CubeMarker:
    def __init__(self, frame_id="map", ns="parked", scale=(4.7, 1.9, 1.4), color=(93/255.0, 122/255.0, 177/255.0, 0.97)):
        self.frame_id = frame_id
        self.ns = ns
        self.scale = Vector3(*scale)
        self.color = ColorRGBA(*color)

    def convert(self, x, y, z, yaw, id):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = self.ns
        m.id = id
        m.type = m.CUBE
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        m.pose = Pose(Point(x, y, z), Quaternion(*quat))
        m.scale = self.scale
        m.color = self.color
        return m