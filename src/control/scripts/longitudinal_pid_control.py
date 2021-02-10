#! /usr/bin python

import numpy as np
import matplotlib.pyplot as plt
from control.pid import PID_Controller

dt = 0.01
v = 0.1
target_speed = 20.0 / 3.6
# min_error = 1
# for Kp in np.logspace(-2, 2, 20):
#     for Kd in np.logspace(-2, 2, 20):
#         for Ki in np.logspace(-5, -1, 20):
#             v = 0.1
#             pid = PID_Controller(Kp, Kd, Ki)
#             tl = []
#             vl = []
#             el = []
#             for i in range(1000):
#                 t = i * dt
#                 error = v - target_speed
#                 a = pid.feedback(error, dt)
#                 if a > 20 : break
#                 v = v + a * dt
#                 tl.append(t)
#                 vl.append(v)
#                 el.append(abs(error))
#             else:
#                 err = np.mean(el)
#                 if err < min_error:
#                     print(Kp, Kd, Ki, err)
#                     min_error = err


Kp, Kd, Ki = 3.3598182862837809, 0.01, 1.0000000000000001e-05

pid = PID_Controller(Kp, Kd, Ki)
tl = []
vl = []
el = []
for i in range(1000):
    t = i * dt
    error = v - target_speed
    a = pid.feedback(error, dt)
    v = v + a * dt
    tl.append(t)
    vl.append(v)
    el.append(abs(error))
print(np.mean(el))
plt.figure()
plt.plot([0, tl[-1]], [target_speed, target_speed], 'r-', label="reference")
plt.plot(tl, vl, 'b-', label="PID controller")
plt.xlabel('v (m/s)')
plt.ylabel('t (s)')
plt.legend(loc="best")
plt.axis("equal")

plt.show()