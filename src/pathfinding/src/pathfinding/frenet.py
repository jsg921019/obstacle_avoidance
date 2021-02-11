#! /usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np

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

        s, d, yaw_road = self.get_frenet(x, y)
        self.prev_opt_path = FrenetPath(s, d, np.tan(yaw-yaw_road), 0, s+1, 0, 0, 0)

        self.MIN_SF = 10.0
        self.MAX_SF = 20.0
        self.DS = 2.0
        self.steps = 20

        self.K_MAX = 5.0

        self.LANE_WIDTH = 3.2

        # cost weights
        self.K_J = 0.1 # weight for jerk
        self.K_T = 0.1 # weight for terminal time
        self.K_D = 5.0 # weight for consistency
        self.K_V = 1.0 # weight for getting to target speed
        self.K_LAT = 3.0 # weight for lateral direction
        self.K_LON = 1.0 # weight for longitudinal direction

        self.COL_CHECK = 2.0 # collision check distance [m]
        self.DF_SET = np.array([-self.LANE_WIDTH * 1.0/2.0, 0,  self.LANE_WIDTH * 1.0/2.0])

    def get_frenet(self, x, y):

        idx = np.argmin(np.hypot(x - self.refx, y - self.refy))
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

            # cost for consistency
            d_diff = (self.prev_opt_path.target_d - fp.target_d)**2

            # cost
            fp.c = self.K_D * d_diff

            for _s, _d in zip(fp.s_arr, fp.d_arr):
                _x, _y, _ = self.get_cartesian(_s, _d)
                fp.x.append(_x)
                fp.y.append(_y)

            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(np.arctan2(dy, dx))
                fp.ds.append(np.hypot(dx, dy))

            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])

            # # calc curvature
            # for i in range(len(fp.yaw) - 1):
            #     yaw_diff = fp.yaw[i + 1] - fp.yaw[i]
            #     yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
            #     fp.kappa.append(yaw_diff / fp.ds[i])

        return fplist

    def check_path(self, fplist, obstacles):
        ok_ind = []
        reason = []
        for i, _path in enumerate(fplist):
            if self.collision_check(_path, obstacles):
                reason.append("collision")
                continue
            ok_ind.append(i)

        if not ok_ind:
            print(reason)
        return [fplist[i] for i in ok_ind]

    def collision_check(self, fp, obstacles):
        for ox, oy in obstacles:

            d = [((_x - ox) ** 2 + (_y - oy) ** 2) for (_x, _y) in zip(fp.x, fp.y)]
            collision = any([di <= self.COL_CHECK ** 2 for di in d])

            if collision:
                return True

        return False


######## backup #########

# def get_frenet(x, y, mapx, mapy, maps):

#     idx = np.argmin(np.hypot(x - mapx, y - mapy))
#     map_vec = [mapx[idx + 1] - mapx[idx], mapy[idx + 1] - mapy[idx]]
#     ego_vec = [x - mapx[idx], y - mapy[idx]]
#     next_wp = idx+1 if np.dot(map_vec, ego_vec) >=0 else idx
#     prev_wp = next_wp -1

#     n_x = mapx[next_wp] - mapx[prev_wp]
#     n_y = mapy[next_wp] - mapy[prev_wp]
#     ego_vec = [x-mapx[prev_wp], y-mapy[prev_wp]]
#     map_vec = [n_x, n_y]

#     d= np.cross(map_vec, ego_vec)/(maps[next_wp] - maps[prev_wp])
#     s = maps[prev_wp] + np.dot(map_vec, ego_vec)/(maps[next_wp] - maps[prev_wp])
#     heading = np.arctan2(n_y, n_x)

#     return s, d, heading

# def get_frenet(x, y, refs, dist_lookup):
#     curr = np.array([x,y])
#     distance, index = dist_lookup.query([x,y])

#     map_vec = refs[index + 1] - refs[index]
#     ego_vec = curr-refs[index]

#     direction  = np.sign(np.dot(map_vec, ego_vec))

#     if direction >= 0:
#         next_wp = index + 1
#     else:
#         next_wp = index

#     prev_wp = next_wp -1

#     n_x, n_y = refs[next_wp] - refs[prev_wp]
#     x_x, x_y = curr - refs[prev_wp]

#     proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y)
#     proj_x = proj_norm*n_x
#     proj_y = proj_norm*n_y

#     heading = np.arctan2(n_y, n_x) # [rad]

#     #-------- get frenet d
#     frenet_d = np.hypot(x_x-x_y,proj_x-proj_y)

#     ego_vec = [x-refs[prev_wp][0], y-refs[prev_wp][1], 0]
#     map_vec = [n_x, n_y, 0]
#     d_cross = np.cross(ego_vec,map_vec)
#     if d_cross[-1] > 0:
#         frenet_d = -frenet_d

#     #-------- get frenet s
#     frenet_s = 0
#     for i in range(prev_wp):
#         frenet_s += np.linalg.norm(refs[i] - refs[i+1])
#     frenet_s += np.linalg.norm(np.array([proj_x, proj_y]))

#     return frenet_s, frenet_d, heading

# def get_cartesian(s, d, mapx, mapy, maps, mapyaw):

#     prev_wp = np.searchsorted(maps, s, 'left') -1

#     heading = mapyaw[prev_wp]
#     perp_heading = heading + np.deg2rad(90)

#     seg_s = s - maps[prev_wp]
#     x = mapx[prev_wp] + seg_s*np.cos(heading) + d*np.cos(perp_heading)
#     y = mapy[prev_wp] + seg_s*np.sin(heading) + d*np.sin(perp_heading)

#     return x, y, heading

# class Frenet(object):
#     def __init__(self, reference_path):
#         #self.drawer = PointDrawer()
#         self.refd = reference_path
#         self.refs = np.column_stack([reference_path["x"], reference_path["y"]])
#         self.lookup = KDTree(self.refs)
#         self.MIN_T = 1 # minimum terminal time [s]
#         self.MAX_T = 5 # maximum terminal time [s]
#         self.DT_T = 0.5 # dt for terminal time [s] : MIN_T 에서 MAX_T 로 어떤 dt 로 늘려갈지를 나타냄
#         self.DT = 0.1 # timestep for update
        
#         self.V_MAX = 10      # maximum velocity [m/s]
#         self.ACC_MAX = 100 # maximum acceleration [m/ss]
#         self.K_MAX = 5   # maximum curvature [1/m]

#         self.TARGET_SPEED = 5 # target speed [m/s]
#         self.LANE_WIDTH = 4  # lane width [m]
#         # cost weights
#         self.K_J = 0.1 # weight for jerk
#         self.K_T = 0.1 # weight for terminal time
#         self.K_D = 5.0 # weight for consistency
#         self.K_V = 1.0 # weight for getting to target speed
#         self.K_LAT = 3.0 # weight for lateral direction
#         self.K_LON = 1.0 # weight for longitudinal direction

#         self.COL_CHECK = 2 # collision check distance [m]
#         self.DF_SET = np.array([-self.LANE_WIDTH * 1.0/2.0, 0,  self.LANE_WIDTH * 1.0/2.0])

#     def calc_frenet_paths(self, si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d):
#         frenet_paths = []

#         # generate path to each offset goal
#         for df in self.DF_SET:

#             # Lateral motion planning
#             for T in np.arange(self.MIN_T, self.MAX_T+self.DT_T, self.DT_T):
#                 fp = FrenetPath()
#                 lat_traj = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, T)

#                 fp.t = [t for t in np.arange(0.0, T, self.DT)]
#                 fp.d = [lat_traj.calc_pos(t) for t in fp.t]
#                 fp.d_d = [lat_traj.calc_vel(t) for t in fp.t]
#                 fp.d_dd = [lat_traj.calc_acc(t) for t in fp.t]
#                 fp.d_ddd = [lat_traj.calc_jerk(t) for t in fp.t]

#                 # Longitudinal motion planning (velocity keeping)
#                 tfp = deepcopy(fp)
#                 lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T)

#                 tfp.s = [lon_traj.calc_pos(t) for t in fp.t]
#                 tfp.s_d = [lon_traj.calc_vel(t) for t in fp.t]
#                 tfp.s_dd = [lon_traj.calc_acc(t) for t in fp.t]
#                 tfp.s_ddd = [lon_traj.calc_jerk(t) for t in fp.t]

#                 # 경로 늘려주기 (In case T < MAX_T)
#                 for _t in np.arange(T, self.MAX_T, self.DT):
#                     tfp.t.append(_t)
#                     tfp.d.append(tfp.d[-1])
#                     _s = tfp.s[-1] + tfp.s_d[-1] * self.DT
#                     tfp.s.append(_s)

#                     tfp.s_d.append(tfp.s_d[-1])
#                     tfp.s_dd.append(tfp.s_dd[-1])
#                     tfp.s_ddd.append(tfp.s_ddd[-1])

#                     tfp.d_d.append(tfp.d_d[-1])
#                     tfp.d_dd.append(tfp.d_dd[-1])
#                     tfp.d_ddd.append(tfp.d_ddd[-1])

#                 J_lat = sum(np.power(tfp.d_ddd, 2))  # lateral jerk
#                 J_lon = sum(np.power(tfp.s_ddd, 2))  # longitudinal jerk

#                 # cost for consistency
#                 d_diff = (tfp.d[-1] - opt_d) ** 2
#                 # cost for target speed
#                 v_diff = (self.TARGET_SPEED - tfp.s_d[-1]) ** 2

#                 # lateral cost
#                 tfp.c_lat = self.K_J * J_lat + self.K_T * T + self.K_D * d_diff
#                 # logitudinal cost
#                 tfp.c_lon = self.K_J * J_lon + self.K_T * T + self.K_V * v_diff

#                 # total cost combined
#                 tfp.c_tot = self.K_LAT * tfp.c_lat + self.K_LON * tfp.c_lon

#                 frenet_paths.append(tfp)

#         return frenet_paths

#     def find_path(self, x, y, v, a, yaw, obstacles):
#         s, d, yaw_road = get_frenet(x, y, self.refd["x"], self.refd["y"], self.refd["s"])
#         x, y, heading = get_cartesian(s, d, self.refd["x"], self.refd["y"], self.refd["s"], self.refd["yaw"])
#         #self.drawer.draw(x, y, 2)
#         yawi = 0 # yaw - yaw_road
#         # s 방향 초기조건
#         si = s
#         si_d = self.TARGET_SPEED # v*np.cos(yawi)
#         si_dd = a*np.cos(yawi)
#         sf_d = self.TARGET_SPEED
#         sf_dd = 0

#         # d 방향 초기조건
#         di = d
#         di_d = v*np.sin(yawi)
#         di_dd = a*np.sin(yawi)
#         df_d = 0
#         df_dd = 0

#         opt_d = di

#         frenet_paths = self.calc_frenet_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d)
#         frenet_paths = self.calc_global_paths(frenet_paths)
#         frenet_paths = self.check_path(frenet_paths, obstacles)
#         min_cost = float("inf")
#         opt_path = None
#         for i, fp in enumerate(frenet_paths):
#             if min_cost >= fp.c_tot:
#                 min_cost = fp.c_tot
#                 opt_path = fp
        
#         # if opt_path is None:
#         #     pass

#         return frenet_paths, opt_path

#     def calc_global_paths(self, fplist):
#         # transform trajectory from Frenet to Global
#         for fp in fplist:
#             for i in range(len(fp.s)):
#                 _s = fp.s[i]
#                 _d = fp.d[i]
#                 _x, _y, _ = get_cartesian(_s, _d, self.refd["x"], self.refd["y"], self.refd["s"], self.refd["yaw"])
#                 fp.x.append(_x)
#                 fp.y.append(_y)

#             for i in range(len(fp.x) - 1):
#                 dx = fp.x[i + 1] - fp.x[i]
#                 dy = fp.y[i + 1] - fp.y[i]
#                 fp.yaw.append(np.arctan2(dy, dx))
#                 fp.ds.append(np.hypot(dx, dy))

#             fp.yaw.append(fp.yaw[-1])
#             fp.ds.append(fp.ds[-1])

#             # calc curvature
#             for i in range(len(fp.yaw) - 1):
#                 yaw_diff = fp.yaw[i + 1] - fp.yaw[i]
#                 yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
#                 fp.kappa.append(yaw_diff / fp.ds[i])

#         return fplist

#     def check_path(self, fplist, obstacles):
#         ok_ind = []
#         reason = []
#         for i, _path in enumerate(fplist):
#             acc_squared = [(abs(a_s**2 + a_d**2)) for (a_s, a_d) in zip(_path.s_dd, _path.d_dd)]
#             if any([v > self.V_MAX for v in _path.s_d]):  # Max speed check
#                 reason.append("speed exceeded")
#                 continue
#             elif any([acc > self.ACC_MAX**2 for acc in acc_squared]):
#                 reason.append("acc exceeded")
#                 continue
#             elif any([abs(kappa) > self.K_MAX for kappa in fplist[i].kappa]):  # Max curvature check
#                 reason.append("curvature exceeded")
#                 continue
#             elif self.collision_check2(_path, obstacles):
#                 reason.append("collision")
#                 continue

#             ok_ind.append(i)
#         if not ok_ind:
#             print(reason)
#         return [fplist[i] for i in ok_ind]

#     def collision_check2(self, fp, obstacles):
#         for ox, oy in obstacles:

#             d = [((_x - ox) ** 2 + (_y - oy) ** 2) for (_x, _y) in zip(fp.x, fp.y)]
#             collision = any([di <= self.COL_CHECK ** 2 for di in d])

#             if collision:
#                 return True

#         return False