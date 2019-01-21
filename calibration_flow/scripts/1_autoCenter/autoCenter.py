import time
from math import cos, sin, pi
import cv2
import numpy as np
import quaternion
import glob
import yaml

def __get_axis_unit_vector__(p0, p1):
    vec = p1 - p0
    return vec/np.linalg.norm(vec)

def __get_pixel_coordinate_of_ARid__(ARcors, ARids, ptId):
    ptIndex = np.where(ARids==ptId)[0][0]
    return ARcors[ptIndex][0, 0]

def __get_angle_between_vectors__(v1, v2):
    uv1 = v1/np.linalg.norm(v1)
    uv2 = v2/np.linalg.norm(v2)
    return np.arctan2(uv2[1],uv2[0]) - np.arctan2(uv1[1],uv1[0])

def __get_angle_between_vectors_2__(v1, v2):
    uv1 = v1/np.linalg.norm(v1)
    uv2 = v2/np.linalg.norm(v2)
    a = np.dot(uv1, uv2)
    b = np.linalg.norm(uv1)*np.linalg.norm(uv2)
    return np.arccos(a/b)

def cal_center_axis_vector(ARcors, ARids):
    ptCen = __get_pixel_coordinate_of_ARid__(ARcors, ARids, 24)
    ptRight = __get_pixel_coordinate_of_ARid__(ARcors, ARids, 25)
    ptUp = __get_pixel_coordinate_of_ARid__(ARcors, ARids, 13)
    x_uvec = __get_axis_unit_vector__(ptCen, ptRight)
    y_uvec = __get_axis_unit_vector__(ptCen, ptUp)
    angle = __get_angle_between_vectors__(x_uvec, np.array([1, 0]))
    return x_uvec, y_uvec, ptCen, -angle

def get_cam_parameter(camBrand, FOV):
    FOV = np.array(FOV) # Field of view (mm)
    if camBrand == 'basler':
        cmos = np.array([5.7, 4.3]) # sensor size (mm)
        reso = np.array([2592, 1944]) # resolution (px)
        f = 12 # focal length (mm)
        (fx, fy) = f*reso/cmos # (px)
        (u0, v0) = reso/2 # (px)
        # intrinsic matrix
        cmtx = np.matrix([[fx, 0, u0],
                          [0, fy, v0],
                          [0, 0, 1]]) 
        M = np.mean(FOV/cmos) # Magnification
        WD = M*f  # Work distance (mm)
        # Magnification unit pixel to mm
        Mpx2mm = M*cmos/reso*np.array([1, -1])
    elif camBrand == 'toshiba':
        raise('Not supported camera') 
    elif camBrand == 'gazebo':
        reso = np.array([2592, 1944]) # resolution (px)
        (u0, v0) = reso/2 # (px)
        # intrinsic matrix
        cmtx = np.matrix([[0, 0, u0],
                          [0, 0, v0],
                          [0, 0, 1]]) 
        # Magnification unit pixel to mm
        Mpx2mm = np.array([20.0/210.0, 20.0/210.0])*np.array([-1, 1])
    else:
        raise('Not supported camera') 

    return (cmtx, Mpx2mm)

def convert_center_ROSgoal(dq, Init_GOAL):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    with open(Init_GOAL) as f:
        init_goal = np.array(yaml.load(f), dtype='float')

    q0 = np.quaternion(init_goal[6], init_goal[3], init_goal[4], init_goal[5])
    H = np.identity(4)
    H[3,0] = init_goal[0] + dq[0]
    H[3,1] = init_goal[1] + dq[1]
    H[3,2] = init_goal[2]
    H[0:2, 0:2] =  np.matrix([[cos(dq[2]), -sin(dq[2])], [sin(dq[2]), cos(dq[2])]])
    q = q0*quaternion.from_rotation_matrix(H[0:3, 0:3])
    goal = np.array([H[3,0], H[3,1], H[3,2], q.x, q.y, q.z, q.w])
    return goal

def compute_compensation(BASE, IMAGE_PATH):
    # Generate ChArUco pattern
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    board = cv2.aruco.CharucoBoard_create(11, 9, 0.020, 0.010, dictionary)
    img = board.draw((200*11,200*9))
    cv2.imwrite(BASE + 'img/charuco.png',img)

    # Camera parameter
    FOV = (0, 0) # (mm)
    (cmtx, Mpx2mm) = get_cam_parameter('gazebo', FOV)

    # Compute center coordinate in pixel
    frame = cv2.imread(IMAGE_PATH)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ARcors, ARids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    cv2.aruco.drawDetectedMarkers(frame, ARcors, ARids)
    x_uvec, y_uvec, ptCen, angle = cal_center_axis_vector(ARcors, ARids)

    # Compensate of pixel and mm
    dU = np.array(gray.shape)[::-1]/2 - ptCen
    dQ = Mpx2mm*dU[::-1]
    
    # convert unit to meter
    dq = np.array([dQ[0]/1000, dQ[1]/1000, angle]) # dx, dy, angle
    
    return dq

def main(DEBUG):
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)
        
    BASE = path['ACenter'] if DEBUG else path['ROOT']
    AF_BASE = path['AFocus'] if DEBUG else path['ROOT']
    IMAGE_PATH = BASE + 'img/init.bmp'
    Init_GOAL = BASE + 'goal/init_goal.yaml'
    AC_GOAL = BASE + 'goal/ac_goal.yaml'
    AF_GOAL = AF_BASE + 'goal/af_goal.yaml'

    # Compute compensation in meter
    dq = compute_compensation(BASE, IMAGE_PATH)

    # Convert compensation to ROS goal
    ac_goal = convert_center_ROSgoal(dq, Init_GOAL)
    
    # Set auto focus goal moved along z-axis of center goal
    n = 11 # number of goal
    z = 0.04 # interval of movement
    af_goals = np.ones((n, 7)) * ac_goal
    for ind, dz in enumerate(np.linspace(-z/2, z/2, 11)):
        af_goals[ind, 2] = ac_goal[2] + dz
    
    # print(dq)
    # print(ac_goal)
    # print(af_goals)

    with open(AC_GOAL, 'w') as f:
        yaml.dump(ac_goal.tolist(), f, default_flow_style=False)
    
    with open(AF_GOAL, 'w') as f:
        yaml.dump(af_goals.tolist(), f, default_flow_style=False)

if __name__ == '__main__':
    '''
    Input: Initial image
    Output: Compensate value of x, y, Rz direction
    DEBUG mode: Result save in the local folder
    '''
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main(DEBUG=DEBUG)