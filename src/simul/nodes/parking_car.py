#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
from model.vehicle import Vehicle

rospy.init_node("parking_car")
rate = rospy.Rate(1)

cp1 = Vehicle(x=45.4, y=31.7, yaw=-0.51)
cp1.init_marker_pub(topic="car2", frame_id="map", ns="parked", id=2)

cp2 = Vehicle(x=25.578, y=-9.773, yaw=2.65)
cp2.init_marker_pub(topic="car3", frame_id="map", ns="parked", id=3)

while not rospy.is_shutdown():
    cp1.publish_marker()
    cp2.publish_marker()
    rate.sleep()
