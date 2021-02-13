#! /usr/bin python
#-*- coding: utf-8 -*-

import numpy as np

class PurePursuit(object):
    def __init__(self, k, ks=0.1, L=2.8):
        self.k = k
        self.ks = ks
        self.L = L

    def feedback(self, x, y, yaw, v, map_xs, map_ys):

        look_dist = self.k * v
        for _x, _y in zip(map_xs, map_ys):
            d = (x-_x)**2 + (y-_y)**2
            if d > look_dist**2:
                yaw_vec = [self.L * np.cos(yaw), self.L * np.sin(yaw)]
                dist_vec = [_x-x, _y-y]
                return _x, _y, np.arctan2(2*np.cross(yaw_vec, dist_vec), d)