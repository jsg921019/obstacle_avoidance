#! /usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import math

class FrenetPath(object):

    def __init__(self, si, di, ddi, dddi, sf, df, ddf, dddf):

        A = np.array([[si**5,    si**4,    si**3,   si**2, si,  1.0],
                      [5*si**4,  4*si**3,  3*si**2, 2*si,  1.0, 0.0],
                      [20*si**3, 12*si**2, 6*si,    2.0,   0.0, 0.0],
                      [sf**5,    sf**4,    sf**3,   sf**2, sf,  1.0],
                      [5*sf**4,  4*sf**3,  3*sf**2, 2*sf,  1.0, 0.0],
                      [20*sf**3, 12*sf**2, 6*sf,    2.0,   0.0, 0.0]])

        b = np.array([di, ddi, dddi, df, ddf, dddf])

        x = np.linalg.solve(A, b)

        self.p = np.poly1d(x)
        self.dp = self.p.deriv()
        self.ddp = self.dp.deriv()
        self.target_d = df
        self.si = si
        self.sf = sf
        self.x = []
        self.y = []
        self.ds = []
        self.yaw = []
        

class Frenet(object):
    def __init__(self, reference_path, x, y, yaw):

        self.refx = reference_path["x"]
        self.refy = reference_path["y"]
        self.refs = reference_path["s"]
        self.refyaw = reference_path["yaw"]

        self.min_idx = 0
        self.max_idx = len(self.refx)-1

        s, d, yaw_road = self.get_frenet(x, y)
        self.prev_opt_path = FrenetPath(s, d, np.tan(yaw-yaw_road), 0, s+1, 0, 0, 0)

        self.MIN_SF = 8.0
        self.MAX_SF = 20.0
        self.DS = 4.0
        self.steps = 20

        self.LANE_WIDTH = 3.0

        # cost weights
        self.K_DIFF = 1.0
        self.K_KAPPA = 500
        self.K_SHORT = 10.0
        self.K_COLLISION = 0.01

        self.COL_CHECK = 2.0 # collision check distance [m]
        self.DF_SET = np.array([-self.LANE_WIDTH * 2.0/3.0, -self.LANE_WIDTH * 1.0/2.0, 0,  self.LANE_WIDTH * 1.0/2.0, self.LANE_WIDTH * 2.0/3.0])

    def get_frenet(self, x, y):
        
        idx = self.min_idx + np.argmin(np.hypot(x - self.refx[self.min_idx:self.max_idx], y - self.refy[self.min_idx:self.max_idx]))
        self.min_idx, self.max_idx = max(0,idx-10), min(len(self.refx)-1, idx+10)
        map_vec = [self.refx[idx + 1] - self.refx[idx], self.refy[idx + 1] - self.refy[idx]]
        ego_vec = [x - self.refx[idx], y - self.refy[idx]]
        next_wp = idx + 1 if np.dot(map_vec, ego_vec) >=0 else idx
        prev_wp = next_wp - 1

        n_x = self.refx[next_wp] - self.refx[prev_wp]
        n_y = self.refy[next_wp] - self.refy[prev_wp]
        ego_vec = [x - self.refx[prev_wp], y - self.refy[prev_wp]]
        map_vec = [n_x, n_y]

        d= np.cross(map_vec, ego_vec)/(self.refs[next_wp] - self.refs[prev_wp])
        s = self.refs[prev_wp] + np.dot(map_vec, ego_vec)/(self.refs[next_wp] - self.refs[prev_wp])
        heading = np.arctan2(n_y, n_x)

        return s, d, heading

    def get_cartesian(self, s, d):

        prev_wp = np.searchsorted(self.refs, s, 'left') - 1

        heading = self.refyaw[prev_wp]
        perp_heading = heading + np.deg2rad(90)

        seg_s = s - self.refs[prev_wp]
        x = self.refx[prev_wp] + seg_s*np.cos(heading) + d*np.cos(perp_heading)
        y = self.refy[prev_wp] + seg_s*np.sin(heading) + d*np.sin(perp_heading)

        return x, y, heading

    def find_path(self, x, y, obstacles):
        frenet_paths = self.calc_frenet_paths(x, y)
        frenet_paths = self.calc_global_paths(frenet_paths)
        frenet_paths = self.check_path(frenet_paths, obstacles)

        for fp in frenet_paths:

            # cost for consistency
            d_diff = (self.prev_opt_path.target_d - fp.target_d)**2

            # cost for path length
            path_sum = np.sum(fp.ds)

            # cost for curvature
            yaw_diff = fp.yaw[1:] - fp.yaw[:-1]
            yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
            yaw_diff = np.abs(yaw_diff)
            mean_kappa = np.sum(yaw_diff / fp.ds[:-1])
            
            # cost
            fp.c = self.K_DIFF * d_diff + self.K_SHORT * path_sum + self.K_KAPPA * mean_kappa + self.K_COLLISION / (fp.gap + 0.01)

        min_cost = float("inf")
        opt_path = None

        for fp in frenet_paths:
            if min_cost > fp.c:
                min_cost = fp.c
                opt_path = fp

        if opt_path is not None:
            self.prev_opt_path = opt_path

        return frenet_paths, opt_path

    def calc_frenet_paths(self, x, y):

        # path initial conidion
        si = self.get_frenet(x, y)[0]
        di = self.prev_opt_path.p(si)
        ddi = self.prev_opt_path.dp(si)
        dddi = self.prev_opt_path.ddp(si)

        #path final condition
        ddf = 0.0
        dddf = 0.0

        frenet_paths = []
        for df in self.DF_SET:
            for sf in np.arange(si+self.MIN_SF, si+self.MAX_SF + self.DS, self.DS):
                fp = FrenetPath(si, di, ddi, dddi, sf, df, ddf, dddf)
                frenet_paths.append(fp)

        return frenet_paths

    def calc_global_paths(self, fplist):
        for fp in fplist:
            fp.s_arr = np.linspace(fp.si, fp.si + self.MAX_SF, self.steps)
            fp.d_arr = np.where(fp.s_arr < fp.sf, fp.p(fp.s_arr), fp.target_d)

            for _s, _d in zip(fp.s_arr, fp.d_arr):
                _x, _y, _ = self.get_cartesian(_s, _d)
                fp.x.append(_x)
                fp.y.append(_y)
            fp.x = np.array(fp.x)
            fp.y = np.array(fp.y)

            dx = fp.x[1:] - fp.x[:-1]
            dy = fp.y[1:] - fp.y[:-1]
            fp.yaw = np.arctan2(dy, dx)
            fp.ds = np.hypot(dx, dy)
            fp.yaw = np.append(fp.yaw, fp.yaw[-1])
            fp.ds = np.append(fp.ds, fp.ds[-1])

        return fplist

    def check_path(self, fplist, obstacles):
        ok_ind = []
        for i, _path in enumerate(fplist):
            if self.collision_check(_path, obstacles):
                continue
            ok_ind.append(i)

        if not ok_ind:
            print("No Path Found")
        return [fplist[i] for i in ok_ind]

    def collision_check(self, fp, obstacles):
        for ox, oy in obstacles:

            d = [((_x - ox) ** 2 + (_y - oy) ** 2) for (_x, _y) in zip(fp.x, fp.y)]
            fp.gap = min(d) - self.COL_CHECK ** 2
            collision = fp.gap <= 0

            if collision:
                return True

        return False