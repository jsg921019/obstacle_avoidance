#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import rospkg
import pickle
from visualization_msgs.msg import MarkerArray
import numpy as np

from model.vehicle import KinematicBicycle
from control.pid import PID_Controller
from control.stanley import Stanley
from pathfinding.frenet import Frenet
from rviz_display.markers import PathMarker, PathsMarkerArray, TextMarker

rospy.init_node("car")

######## load reference path ########

rospack = rospkg.RosPack()
path = rospack.get_path("map_server")

with open(path + "/src/ref_path.pkl", "rb") as f:
    ref_path = pickle.load(f)



######## load obstacles info ########

path = rospack.get_path("obstacles")

with open(path + "/src/obstacles.pkl", "rb") as f:
    obstacles = pickle.load(f)
xy, yaw = np.hsplit(obstacles, [2])
yaw = np.column_stack([np.cos(yaw), np.sin(yaw)])
obstacles = np.vstack([xy -1.4*yaw, xy +1.4*yaw])



######## Target ########

target_speed = 20.0 / 3.6
start_x, start_y, start_yaw = ref_path["x"][30], ref_path["y"][30], ref_path["yaw"][30]
target_x, target_y = ref_path["x"][430], ref_path["y"][430]



######## Instances ########

# Marker Converter
path2marker = PathMarker()
paths2markerarray = PathsMarkerArray()
text2marker = TextMarker()

# driving car instance
car = KinematicBicycle(start_x, start_y, start_yaw)

# publishers
car.init_marker_pub(topic="driving", frame_id="map", ns="driving", id=1)
car.init_odom_pub(name="odom", child_frame_id="car1", frame_id="map")
paths_pub = rospy.Publisher("paths", MarkerArray, queue_size=1)

# controller instance
longitudinal_controller = PID_Controller(Kp=0.5, Kd=0, Ki=0.0003)
lateral_controller = Stanley(k=0.8, ks=0.5, kd=0, L=2.8)

# path finding instance
path_finder = Frenet(ref_path, start_x, start_y, start_yaw)



######## Main ########
rate = rospy.Rate(10)

while not rospy.is_shutdown():

    # find optimal path from Frenet
    paths, optimal_path = path_finder.find_path(car.x + car.L * np.cos(car.yaw), car.y + car.L * np.sin(car.yaw), obstacles)
    ma = []
    if optimal_path:
        ma = paths2markerarray.convert(paths)
        ma.append(path2marker.convert(optimal_path))
    
    # update car
    ai = longitudinal_controller.feedback(car.v - target_speed, 0.1)
    if optimal_path:
        di = lateral_controller.feedback(car.x, car.y, car.yaw, car.v, optimal_path.x, optimal_path.y, optimal_path.yaw)
    else:
        di = lateral_controller.feedback(car.x, car.y, car.yaw, car.v, ref_path["x"], ref_path["y"], ref_path["yaw"])
    car.update(ai, di)
    ma.append(text2marker.convert("speed : %.2f" %car.v, 5))

    # check if near target
    if np.hypot(car.x - target_x, car.y - target_y) < 2.5:  
            car.x, car.y, car.yaw, car.v, car.a = start_x, start_y, start_yaw, 0, 0
            path_finder.reset(start_x, start_y, start_yaw)

    # publish car / paths / ui for visualization
    car.publish_marker()
    car.publish_odom()
    paths_pub.publish(ma)

    rate.sleep()