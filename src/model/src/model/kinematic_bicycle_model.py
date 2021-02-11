#! /usr/bin python

import numpy as np
from model.vehicle import Vehicle

class KinematicBicycle(Vehicle):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, a=0.0, L=2):
        super(KinematicBicycle, self).__init__(L)
        
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.a = a
        self.max_steering = np.radians(30)

    def update(self, steer, a=0, dt=0.1):
        steer = np.clip(steer, -self.max_steering, self.max_steering)
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / self.L * np.tan(steer) * dt
        self.yaw = self.yaw % (2.0 * np.pi)
        self.v += a * dt
        self.a = a

def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, a=0.0, dt=0.1, L=2.875):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.a = a
        # self.rear_x = self.x - ((WB / 2) * np.cos(self.yaw))
        # self.rear_y = self.y - ((WB / 2) * np.sin(self.yaw))
        self.dt = dt
        self.L = L
        self.w = 0

    def update(self, a, delta):
        dt = self.dt

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.a = a
        self.w = self.v / self.L * np.tan(delta)
        self.yaw += self.w * dt
        self.yaw = pi_2_pi(self.yaw)
        self.v += a * dt

        # self.rear_x = self.x - ((WB / 2) * np.cos(self.yaw))
        # self.rear_y = self.y - ((WB / 2) * np.sin(self.yaw))
        # self.drawer.draw(self.x, self.y, 2, self.rear_x, self.rear_y, 2)

    # def calc_distance(self, point_x, point_y):
    #     dx = self.rear_x - point_x
    #     dy = self.rear_y - point_y
    #     return np.hypot(dx, dy)
