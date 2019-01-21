import numpy as np
import quaternion
from math import cos, sin

q0 = np.quaternion(0.0, 0.0, -1.0, 0.0)
R = np.identity(3)
t = np.radians(10)
R[0:2, 0:2] =  np.matrix([[cos(t), -sin(t)], [sin(t), cos(t)]])
q = quaternion.from_rotation_matrix(R)
print(q0*q)
