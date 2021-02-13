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

######## Constants ########

start_ind = 30
target_speed = 20.0 / 3.6
obstacles = [[-30.458553086311177, 14.414126077364653, -0.53125179508589049], [-3.0498906544701483, 2.544585769133195, -0.28522847787702166], [16.517578547291212, 21.377199222248414, 1.0650195975940742], [37.964664882119735, 36.035169960021058, -0.37898259996841843], [61.461619252942874, 18.741150237288814, -1.2338908994573341], [52.224132065398798, -8.4852862022935458, -2.0868517894874943], [26.550675034402591, -10.530933162455529, 2.6357276099486535], [-0.80380752675549849, -4.477030648168701, -2.7048020772870105], [-18.959061589166485, -27.32287936690695, -2.0781794125158246]]
obstacles = [[x + 1.4*np.cos(yaw), y + 1.4*np.sin(yaw)] for x, y, yaw in obstacles] + [[x - 1.4*np.cos(yaw), y - 1.4*np.sin(yaw)] for x, y, yaw in obstacles]


######## load reference path ########

rospack = rospkg.RosPack()
path = rospack.get_path("map_server")

with open(path + "/src/ref_path.pkl", "rb") as f:
    ref_path = pickle.load(f)



######## Publishers ########

path2marker = PathMarker()
paths2markerarray = PathsMarkerArray()
text2marker = TextMarker()
paths_pub = rospy.Publisher("paths", MarkerArray, queue_size=1)



######## Instances ########

car = KinematicBicycle(x=ref_path["x"][start_ind], y=ref_path["y"][start_ind], yaw=ref_path["yaw"][start_ind], v=0.1)
car.init_marker_pub(topic="driving", frame_id="map", ns="driving", id=1)
car.init_odom_pub(name="odom", child_frame_id="car1", frame_id="map")

longitudinal_controller = PID_Controller(0.5, 0, 0.00001)
lateral_controller = Stanley(0.8, 0.5, 0, 2.875)

path_finder = Frenet(ref_path, ref_path["x"][start_ind], ref_path["y"][start_ind], ref_path["yaw"][start_ind])
# obstacles2 = []
# i = -1
# for s in range(50, 300, 30):
#     xx, yy, yaww = path_finder.get_cartesian(s, i*1.2)
#     obstacles2.append([xx, yy, yaww])
#     i *= -1
# print(obstacles2)

######## Main ########
rate = rospy.Rate(10)
while not rospy.is_shutdown():

    # find optimal path from Frenet
    paths, optimal_path = path_finder.find_path(car.x + car.L * np.cos(car.yaw), car.y + car.L * np.sin(car.yaw), obstacles)
    ma = paths2markerarray.convert(paths)
    if optimal_path:
        ma.append(path2marker.convert(optimal_path))
    
    # update car
    ai = longitudinal_controller.feedback(car.v - target_speed, 0.1)
    if optimal_path:
        di = lateral_controller.feedback(car.x, car.y, car.yaw, car.v, optimal_path.x, optimal_path.y, optimal_path.yaw)
    else:
        di = lateral_controller.feedback(car.x, car.y, car.yaw, car.v, ref_path["x"], ref_path["y"], ref_path["yaw"])
    car.update(ai, di)
    ma.append(text2marker.convert("speed : %.2f" %car.v, 5))

    # publish car / paths / ui for visualization
    car.publish_marker()
    car.publish_odom()
    paths_pub.publish(ma)
    rate.sleep()