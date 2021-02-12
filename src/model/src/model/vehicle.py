#! /usr/bin python
#-*- coding: utf-8 -*-

import math
import numpy as np

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