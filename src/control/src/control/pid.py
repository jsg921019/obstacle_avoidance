#!/usr/bin/env python

import numpy as np

class PID_Controller(object):
    def __init__(self, Kp, Kd, Ki):
        self.Kp, self.Kd, self.Ki = Kp, Kd, Ki
        self.prev_error = None
        self.int_error = 0

    def feedback(self, error, dt):
        if self.prev_error is None:
            self.prev_error = error
        diff_error = error - self.prev_error
        self.prev_error = error
        self.int_error += error
        return -(self.Kp * error + self.Kd * diff_error/dt + self.Ki * self.int_error)

if __name__ == "__main__":

    import matplotlib.pyplot as plt
    from model.kinematic_bicycle_model import KinematicBicycle

    plant = KinematicBicycle(x=0.0, y=0.0, yaw=0.0, v=2.0, L=3)
    pid = PID_Controller(4.7894736842105257, 3.5789473684210522, 0.0028421052631578949)

    ys = []
    ts = []
    e =[]
    dt = 0.1
    target =1
    for step in range(500):
        t = step * dt
        error = plant.y - target
        steer = pid.feedback(error, dt)
        plant.update(steer,0, dt)
        e.append(abs(error))
        ys.append(plant.y)
        ts.append(plant.x)
    print(np.mean(e))
    plt.figure()
    plt.plot([0, ts[-1]], [target, target], 'r-', label="reference")
    plt.plot(ts, ys, 'b-', label="PID controller")
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend(loc="best")
    plt.axis("equal")

    plt.show()