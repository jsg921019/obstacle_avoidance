#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
from model.vehicle import CarParked

rospy.init_node("parking_car")
r = rospy.Rate(1)
cp1 = CarParked(x=45.4, y=31.7, yaw=-0.51, id=2)
cp2 = CarParked(x=25.578, y=-9.773, yaw=2.65, id=3)

while not rospy.is_shutdown():
    cp1.to_ros()
    cp2.to_ros()
    r.sleep()
