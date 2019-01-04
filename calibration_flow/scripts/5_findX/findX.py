from math import sin, cos, pi
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
import numpy as np
import numpy.matlib
import quaternion
import yaml



def __get_ABXZ__(A_PATH, B_PATH, X_PATH, Z_PATH, Z2_PATH):
    with open(A_PATH) as f:
      As = np.array(yaml.load(f))
      print('Get As from yaml file.')
    with open(B_PATH) as f:
      Bs = np.array(yaml.load(f))
      print('Get Bs from yaml file.')
    with open(X_PATH) as f:
      X = np.array(yaml.load(f))
      print('Get X from yaml file.')
    with open(Z_PATH) as f:
      Z = np.array(yaml.load(f))
      print('Get Z from yaml file.')
    with open(Z2_PATH) as f:
      Z2 = np.array(yaml.load(f))
      print('Get Z2 from yaml file.')
    return As, Bs, X, Z, Z2

def __create_unit_vector__(thetaS, phiS, r=1):
    '''
    Create unit vector base on Spherical coordinate.
    '''
    v = np.zeros(3)
    v[0] = r*sin(thetaS)*cos(phiS)
    v[1] = r*sin(thetaS)*sin(phiS)
    v[2] = r*cos(thetaS)
    return v

def __create_unit_quaternion__(theta, thetaS, phiS):
    '''
    `theta` is the rotation angle around a vector
    `thetaS` and `phiS` is the angle base on Spherical coordinate.
    '''
    t = theta
    v = __create_unit_vector__(thetaS, phiS)
    return np.quaternion(cos(t), v[0]*sin(t), v[1]*sin(t), v[2]*sin(t))

def __solveXZ__(As, Bs):
    def objRot(X, HAs, HBs):
        HX = np.matlib.identity(4)
        HZ = np.matlib.identity(4)
        qX = __create_unit_quaternion__(*X[0:3])
        qZ = __create_unit_quaternion__(*X[3:6])
        HX[0:3, 0:3] = quaternion.as_rotation_matrix(qX)
        HZ[0:3, 0:3] = quaternion.as_rotation_matrix(qZ)
        HX[0:3, 3] = np.reshape(X[6:9], (3,1))
        HZ[0:3, 3] = np.reshape(X[9:12], (3,1))
        fval = np.zeros((len(HBs), 12))
        for i, (HAi, HBi) in enumerate(zip(HAs, HBs)):
            error = HBi - HZ*HAi*HX
            fval[i, :] = np.array(error[0:3, :]).flatten()
        
        return fval.flatten()


    # Z = Bi*X*Ai
    x0 = np.random.rand(12)
    lb = (-2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -10, -10, -5, -10, -10, -5)
    ub = (2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 2*pi, 10, 10, 10, 10, 10, 10)
    res = least_squares(objRot, x0, args=(As, Bs), method='lm',
                         verbose=2, ftol=1e-15, xtol=1e-15)
    
    # Check
    HX = np.matlib.identity(4)
    HZ = np.matlib.identity(4)
    qX = __create_unit_quaternion__(*res['x'][0:3])
    qZ = __create_unit_quaternion__(*res['x'][3:6])
    HX[0:3, 0:3] = quaternion.as_rotation_matrix(qX)
    HZ[0:3, 0:3] = quaternion.as_rotation_matrix(qZ)
    HX[0:3, 3] = np.reshape(res['x'][6:9], (3,1))
    HZ[0:3, 3] = np.reshape(res['x'][9:12], (3,1))

    return HX, HZ

def test_solution(As, Bs, X, Z):
    residual = np.zeros(len(As))
    for i, (A, B) in enumerate(zip(As, Bs)):
        residual[i] = np.linalg.norm(B - Z*A*X)

    return residual

def main(A_PATH, B_PATH, X_PATH, Z_PATH, Z2_PATH):
    As, Bs, X, Z, Z2 = __get_ABXZ__(A_PATH, B_PATH, X_PATH, Z_PATH, Z2_PATH)
    HX, HZ = __solveXZ__(As, Bs)
    # print(HX)
    # print(HZ)
    r = test_solution(As, Bs, X, HZ)
    r2 = test_solution(As, Bs, X, Z2)
    print(r)
    print(r2)
if __name__ == "__main__":
    A_PATH = 'goal/As.yaml'
    B_PATH = 'goal/Bs.yaml'
    X_PATH = 'goal/X.yaml'
    Z_PATH = 'goal/Z.yaml'
    Z2_PATH = 'goal/Z2.yaml'
    
    main(A_PATH, B_PATH, X_PATH, Z_PATH, Z2_PATH)