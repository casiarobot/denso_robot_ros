import time
import cv2
import numpy as np
import glob

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
    angle = np.degrees(angle)
    return x_uvec, y_uvec, ptCen, angle

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

def main():
    USER_BASA = '/home/poyuchen/'
    SCRIPT_BASE = USER_BASA + 'catkin_ws/src/denso_robot_ros/calibration_flow/scripts/'
    ROS_HOME_BASE = USER_BASA + 'catkin_ws/src/ros_home_denso_robot/'
    SOURCE_IMAGE_PATH = ROS_HOME_BASE + 'img/center.bmp'
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    board = cv2.aruco.CharucoBoard_create(11, 9, 0.020, 0.010, dictionary)
    img = board.draw((200*11,200*9))
    #Dump the calibration board to a file
    cv2.imwrite('charuco.png',img)

    FOV = (0, 0) # (mm)
    (cmtx, Mpx2mm) = get_cam_parameter('gazebo', FOV)

    images = sorted(glob.glob(SOURCE_IMAGE_PATH))
    for ind, fname in enumerate(images):
        frame = cv2.imread(fname)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ARcors, ARids, _ = cv2.aruco.detectMarkers(gray, dictionary)
        cv2.aruco.drawDetectedMarkers(frame, ARcors, ARids)
        x_uvec, y_uvec, ptCen, angle = cal_center_axis_vector(ARcors, ARids)

        # Compensate of pixel and mm
        dU = np.array(gray.shape)[::-1]/2 - ptCen
        dX = Mpx2mm*dU[::-1]
        print(dU)
        print(angle, dX)

        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame', 1200, 800)
        cv2.imshow('frame', frame)
        cv2.waitKey(500)

if __name__ == '__main__':
    main()