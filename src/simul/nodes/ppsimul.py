#!/usr/bin/python
#-*- coding: utf-8 -*-

import rospy
import rospkg
import pickle
from visualization_msgs.msg import MarkerArray
import numpy as np

from model.vehicle import KinematicBicycle
from control.pid import PID_Controller
from control.purepursuit import PurePursuit
from pathfinding.frenet import Frenet
from rviz_display.markers import PathMarker, PathsMarkerArray, TextMarker, ArrowDrawer

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



######## Publishers ########

path2marker = PathMarker()
paths2markerarray = PathsMarkerArray()
text2marker = TextMarker()
paths_pub = rospy.Publisher("paths", MarkerArray, queue_size=1)
arrowdrawer = ArrowDrawer()

######## Instances ########

car = KinematicBicycle(start_x, start_y, start_yaw)
car.init_marker_pub(topic="driving", frame_id="map", ns="driving", id=1)
car.init_odom_pub(name="odom", child_frame_id="car1", frame_id="map")

longitudinal_controller = PID_Controller(0.5, 0, 0.0003)
lateral_controller = PurePursuit(k=1.0)

path_finder = Frenet(ref_path, start_x, start_y, start_yaw)


######## Main ########
rate = rospy.Rate(10)

while not rospy.is_shutdown():

    # find optimal path from Frenet
    paths, optimal_path = path_finder.find_path(car.x, car.y, obstacles)
    ma = paths2markerarray.convert(paths)
    if optimal_path:
        ma.append(path2marker.convert(optimal_path))
    
    # update car
    ai = longitudinal_controller.feedback(car.v - target_speed, 0.1)
    if optimal_path:
        _x, _y, di = lateral_controller.feedback(car.x, car.y, car.yaw, car.v, optimal_path.x, optimal_path.y)
    else:
        di = lateral_controller.feedback(car.x, car.y, car.yaw, car.v, ref_path["x"], ref_path["y"], ref_path["yaw"])
    arrowdrawer.draw(_x,_y,3, _x,_y, 0.5)
    car.update(ai, di, dt=0.1)
    ma.append(text2marker.convert("speed : %.2f" %car.v, 5))

    # check if near target
    if np.hypot(car.x - target_x, car.y - target_y) < 3 :  
            car.x, car.y, car.yaw, car.v, car.a = start_x, start_y, start_yaw, 0, 0
            path_finder.reset(start_x, start_y, start_yaw)

    # publish car / paths / ui for visualization
    car.publish_marker()
    car.publish_odom()
    paths_pub.publish(ma)

    rate.sleep()