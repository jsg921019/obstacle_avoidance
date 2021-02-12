#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import rospkg
import pickle
import numpy as np

from model.vehicle import KinematicBicycle
from control.pid import PID_Controller
from control.stanley import Stanley
from pathfinding.frenet import Frenet
from rviz_display.draw_path import PathDrawer, PointDrawer

rospy.init_node("car")

######## Constants ########

car_id = rospy.get_param("~id")
start_ind = 100
target_speed = 20.0 / 3.6
obstacles = [[45.4, 31.7], [25.578, -9.773], [22.578, -8.273]]



######## load reference path ########

rospack = rospkg.RosPack()
path = rospack.get_path("map_server")

with open(path + "/src/ref_path.pkl", "rb") as f:
    ref_path = pickle.load(f)



######## Publishers ########

path_drawer = PathDrawer()
#point_drawer = PointDrawer()



######## Instances ########

car = KinematicBicycle(x=ref_path["x"][start_ind], y=ref_path["y"][start_ind], yaw=ref_path["yaw"][start_ind], v=0.1)
car.init_marker_pub(topic="car1", frame_id="map", ns="driving", id=car_id)
car.init_odom_pub(name="odom", child_frame_id="car1", frame_id="map")

longitudinal_controller = PID_Controller(3.5, 0, 0.00001)
lateral_controller = Stanley(0.8, 0.5, 0, 2.875)

path_finder = Frenet(ref_path, ref_path["x"][start_ind], ref_path["y"][start_ind], ref_path["yaw"][start_ind])



######## Main ########
rate = rospy.Rate(10)
while not rospy.is_shutdown():

    # find optimal path from Frenet
    paths, optimal_path = path_finder.find_path(car.x + car.L * np.cos(car.yaw), car.y + car.L * np.sin(car.yaw), obstacles)
    if optimal_path:
        path_drawer.draw(optimal_path.x, optimal_path.y)

    # update car
    ai = longitudinal_controller.feedback(car.v - target_speed, 0.1)
    if optimal_path:
        di = lateral_controller.feedback(car.x, car.y, car.yaw, car.v, optimal_path.x, optimal_path.y, optimal_path.yaw)
    else:
        di = lateral_controller.feedback(car.x, car.y, car.yaw, car.v, ref_path["x"], ref_path["y"], ref_path["yaw"])
    car.update(ai, di)

    # publish car
    car.publish_marker()
    car.publish_odom()
    rate.sleep()