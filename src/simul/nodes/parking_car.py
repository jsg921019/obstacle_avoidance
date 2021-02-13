#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import MarkerArray
from rviz_display.markers import CubeMarker

obstacles = [[-30.458553086311177, 14.414126077364653, -0.53125179508589049], [-3.0498906544701483, 2.544585769133195, -0.28522847787702166], [16.517578547291212, 21.377199222248414, 1.0650195975940742], [37.964664882119735, 36.035169960021058, -0.37898259996841843], [61.461619252942874, 18.741150237288814, -1.2338908994573341], [52.224132065398798, -8.4852862022935458, -2.0868517894874943], [26.550675034402591, -10.530933162455529, 2.6357276099486535], [-0.80380752675549849, -4.477030648168701, -2.7048020772870105], [-18.959061589166485, -27.32287936690695, -2.0781794125158246]]

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