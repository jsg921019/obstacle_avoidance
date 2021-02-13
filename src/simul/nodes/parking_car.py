#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import rospkg
import pickle
from visualization_msgs.msg import MarkerArray
from rviz_display.markers import CubeMarker

path = rospkg.RosPack().get_path("obstacles")

with open(path + "/src/obstacles.pkl", "rb") as f:
    obstacles = pickle.load(f)

rospy.init_node("parking_car")
rate = rospy.Rate(1)
pub = rospy.Publisher("parking", MarkerArray, queue_size=1)
cube2marker = CubeMarker()

ma = []
for i in range(len(obstacles)):
    x, y, yaw = obstacles[i]
    ma.append(cube2marker.convert(x, y, 0.7, yaw, i+2))

while not rospy.is_shutdown():
    pub.publish(ma)
    rate.sleep()