#! /usr/bin python
#-*- coding: utf-8 -*-

import math
import numpy as np
import rospy
import tf
from object_msgs.msg import Object

def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class Vehicle(object):
    def __init__(self):
        # GV70 PARAMETERS
        self.specs = {"LENGTH":4.7, "WIDTH":1.9, "HEIGHT" : 1.4, "L" : 2.8, "BACKTOWHEEL" : 1.0,
                       "WHEEL_LEN" : 0.3, "WHEEL_WIDTH" : 0.2, "TREAD" : 1.6}
        self.L = self.specs["L"]

    def set_specs(self, **kargs):
        print(kargs)
        for param in kargs:
            if param in self.params:
                self.params[param] = kargs[param]
                print("Changed")
            else:
                raise KeyError("wrong parameter %s" %param)
            self.L = self.specs["L"]

class KinematicBicycle(Vehicle):

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, a=0.0):
        super(KinematicBicycle, self).__init__()
        self.x, self.y, self.yaw = x, y, yaw
        self.v, self.a = v, a
        self.LIM_DELTA = np.radians(30)

    def update(self, a, delta, dt=0.1):
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.a = a
        delta = np.clip(delta, -self.LIM_DELTA, self.LIM_DELTA)
        self.yaw += self.v / self.specs["L"] * math.tan(delta) * dt
        self.yaw = pi_2_pi(self.yaw) # self.yaw % (2.0 * np.pi)
        self.v += a * dt

class CarParked(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, id=1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = 0.0
        self.accel = 0.0
        self.delta = 0.0

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.id = id
        self.object_pub = rospy.Publisher("/objects/car_" + str(id), Object, queue_size=1)

    def to_ros(self):
        quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 1.5),
            quat,
            rospy.Time.now(),
            "/car_" + str(self.id), "/map"
        )

        o = Object()
        o.header.frame_id = "/map"
        o.header.stamp = rospy.Time.now()
        o.id = self.id
        o.classification = o.CLASSIFICATION_CAR

        o.x = self.x
        o.y = self.y
        o.yaw = self.yaw
        o.v = self.v

        # input u
        o.a = self.accel
        o.delta = self.delta
        o.L = 4.475
        o.W = 1.850

        self.object_pub.publish(o)

