#! /usr/bin/env python

import rospy, math
import cv2, time, rospy
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray
from pathfinding.dubins import dubins_path_planning
from control.stanley import Stanley

xycar_msg = Int32MultiArray()

arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0
started = False
refreshed = False
steer = Stanley(k=2.0, L=0.0)

def callback(msg):
    global refreshed
    for i in msg.markers:
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z

        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w
    refreshed = True

rospy.init_node('ar_drive_info')
rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size = 1 )
xycar_msg = Int32MultiArray()
rate = rospy.Rate(10)
k = 0.8

while not rospy.is_shutdown():
    speed = 8
    roll, pitch, yaw = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    l = np.hypot(arData["DX"], arData["DY"])
    cte = -l * np.sin(yaw)
    yaw_term = yaw + np.arctan2(arData["DX"], arData["DY"])
    cte_term = np.arctan2(k*cte, 20.0 * speed)
    angle = yaw_term + cte_term
    print('--')
    print("yaw term :", yaw_term)
    print("cte term :", cte_term)
    angle = 5. /2. * np.degrees(angle) #arData["DX"]*0.5
    print('--')
    # print(refreshed)
    # if not started:
    #     if arData["DY"] > 0 :
    #         started = True
    #         l = np.hypot(arData["DX"], arData["DY"])
    #         x = -l * np.sin(yaw)
    #         y = l * np.cos(yaw)
    #         path_x, path_y, path_yaw, mode, path_length = dubins_path_planning(x, y, yaw, 0.0, 250.0, -np.pi / 2.0, 0.005, 0.05)
    #     rate.sleep()
    #     continue

    # speed = 10.0

    # if refreshed:
    #     roll, pitch, yaw = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    #     l = np.hypot(arData["DX"], arData["DY"])
    #     x = -l * np.sin(yaw)
    #     y = l * np.cos(yaw)
    #     yaw = np.arctan2(-y, -x) + np.arctan2(arData["DX"], arData["DY"])
    #     refreshed = False
    # else:
    #     x += 20 * speed  * np.cos(yaw)
    #     y += 20* speed * np.sin(yaw)
    #     yaw += 20* speed * np.tan(steer_angle) / 75.0

    # print("=======================")
    # print(" x :", round(x,0))
    # print(" y :", round(y,0))
    # print(" yaw :", yaw)
    # print("angle :" , angle)
    # steer_angle = steer.feedback(x, y, yaw, 20*speed, path_x, path_y, path_yaw, dt=0.1)

    # angle = -np.degrees(steer_angle) * 5.0 / 2.0
    
    
    # if y < 70 :
    #     print("stop")
    #     speed = 0
    #     started = False
    xycar_msg.data = [angle, speed]
    motor_pub.publish(xycar_msg)

    rate.sleep()

