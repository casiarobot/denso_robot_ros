import numpy as np
import quaternion

q = np.quaternion(0, 0, -1, 0)
print(quaternion.as_rotation_matrix(q))
print(q)
np.quaternion( )