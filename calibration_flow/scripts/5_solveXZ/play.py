import numpy as np
import quaternion
from math import pi, cos, sin

def __quaternion_by_axis__(theta, v):
    t = theta/2
    return np.quaternion(cos(t), v[0]*sin(t), v[1]*sin(t), v[2]*sin(t))

def __cal_axis_angle_from_q__(q):
    theta = 2*np.arccos(q.w)
    v = np.array([q.x, q.y, q.z])/sin(theta/2)

    return theta, v

if __name__ == "__main__":
    v = np.array([0.0, 0.0, 1.0])
    theta1 = np.radians(30)
    theta2 = np.radians(25)

    q1 = __quaternion_by_axis__(theta1, v)
    q2 = __quaternion_by_axis__(theta2, v)

    t1,v1 = __cal_axis_angle_from_q__(q1)
    t2,v2 = __cal_axis_angle_from_q__(q2)
    
    q3 = q1.inverse()*q2
    t3,v3 = __cal_axis_angle_from_q__(q3)
    print(q1)