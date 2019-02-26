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

def __cal_axis_angle_from_q__(q):
    theta = 2*np.arccos(q.w)
    v = np.array([q.x, q.y, q.z])/sin(theta/2)

    return theta, v

def calPositionAndOrientationError(H, H_exact):
    H1 = np.array(H)
    H2 = np.array(H_exact)
    pH = np.sort(np.abs(H1[0:3, 3] - H2[0:3, 3]))

    pError = np.linalg.norm(pH[0:2])
    q1 = quaternion.from_rotation_matrix(H1[0:3, 0:3])
    q2 = quaternion.from_rotation_matrix(H2[0:3, 0:3])
    q = q1.inverse()*q2
    theta, v = __cal_axis_angle_from_q__(q)
    oError = np.degrees(theta) 
    if oError > 180.0:
        oError = 360.0 - oError 
    return pError, oError

    
def main(DEBUG):
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)
    
    BASE = path['solveXZ'] if DEBUG else path['ROOT']
    AP_BASE = path['APose'] if DEBUG else path['ROOT']
    CC_BASE = path['CCalibration'] if DEBUG else path['ROOT']

    A_PATH = CC_BASE + 'goal/As.yaml'
    B_PATH = AP_BASE + 'goal/Bs.yaml'
    X_PATH = AP_BASE + 'goal/X.yaml'
    Z_PATH = AP_BASE + 'goal/Z.yaml'
    Z2_PATH = AP_BASE + 'goal/Z2.yaml'
    EX_PATH = BASE + 'goal/EX.yaml'
    HX_PATH = BASE + 'goal/HX.yaml'
    HZ_PATH = BASE + 'goal/HZ.yaml'

    As, Bs, X, Z, Z2 = __get_ABXZ__(A_PATH, B_PATH, X_PATH, Z_PATH, Z2_PATH)
    HX, HZ = __solveXZ__(As, Bs)


    # print(HX)
    # print(HZ)
    r = test_solution(As, Bs, X, HZ)
    r2 = test_solution(As, Bs, X, Z2)

    pError, oError = calPositionAndOrientationError(X, HX)
    print('Postion error: {} mm, Orientation error: {} degree'.format(pError*1000, oError))

    with open(HX_PATH, 'w') as f:
        HX = np.linalg.inv(HX) # from flange to camera
        yaml.dump(HX.tolist(), f, default_flow_style=False)
        print('Save the solved X matrix to yaml data file.')
    with open(HZ_PATH, 'w') as f:
        yaml.dump(HZ.tolist(), f, default_flow_style=False)
        print('Save the solved Z matrix to yaml data file.')

    with open(EX_PATH, 'w') as f:
        X = np.linalg.inv(X) # from flange to camera
        yaml.dump(X.tolist(), f, default_flow_style=False)
        print('Save the exact X matrix to yaml data file.')

if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main(DEBUG=DEBUG)