import time
import cv2
import matplotlib.pyplot as plt
import numpy as np
import quaternion
import glob
import yaml
import frame3D


def as_quaternion(rvecs, tvecs):
    pose_amount = len(rvecs)
    # Extrinsic parameter
    extr_q = np.zeros((pose_amount, 7))
    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        # extr_mat[i, 0:3, 0:3] = cv2.Rodrigues(rvec)[0]
        q = quaternion.from_rotation_vector(rvec.flatten())
        extr_q[i] = (tvec[0], tvec[1], tvec[2],
                    q.x, q.y, q.z, q.w)
    return extr_q

def as_homogeneous_mat(rvecs, tvecs):
    pose_amount = len(rvecs)
    # Extrinsic parameter from Camera to Pattern
    As = np.zeros((pose_amount, 4, 4))
    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        As[i, 0:3, 0:3] = cv2.Rodrigues(rvec)[0]
        As[i, 0:3 ,3] = tvec.flatten()
        As[i, 3, 3] = 1.0
    
    # iAs = np.zeros((pose_amount, 4, 4))
    # for i, A in enumerate(As):
    #     iAs[i] = np.linalg.inv(A) # (Pattern to Camera)
    return As

def as_ROSgoal(Homo_mats):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    cmd_amount = len(Homo_mats)
    goals = np.zeros((cmd_amount, 7))
    for i, H in enumerate(Homo_mats):
        rot = H[0:3, 0:3]
        trans = H[0:3, 3]
        q = quaternion.from_rotation_matrix(rot)
        goals[i] = (trans[0], trans[1], trans[2],
                    q.x, q.y, q.z, q.w)
    return goals

def main(DEBUG, output_pattern_img=True):
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)
    
    BASE = path['PoseEstimation'] if DEBUG else path['ROOT']
    AC_BASE = path['ACenter'] if DEBUG else path['ROOT']
    AP_BASE = path['APose'] if DEBUG else path['ROOT']
    IMAGE_PATH = AP_BASE + 'img/ap*.bmp'
    INTMAT = AC_BASE + 'goal/camera_mat.yaml'
    INTMAT_CP = BASE + 'goal/camera_mat.yaml'
    EXTMAT = BASE + 'goal/As.yaml'

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    board = cv2.aruco.CharucoBoard_create(11, 9, 0.010, 0.005, dictionary)

    #Dump the calibration board to a file
    if output_pattern_img:
        img = board.draw((100*11,100*9))
        cv2.imwrite(BASE +'charuco.png',img)

    # Camera parameter
    with open(INTMAT) as f:
        cameraMatrix = np.array(yaml.load(f))
    distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    rvecs = []
    tvecs = []

    images = sorted(glob.glob(IMAGE_PATH))
    for ind, fname in enumerate(images):
        frame = cv2.imread(fname)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ARcors, ARids, _ = cv2.aruco.detectMarkers(gray, dictionary)

        if len(ARcors)>0:
            ret, CHcors, CHids = cv2.aruco.interpolateCornersCharuco(ARcors, ARids, gray, board)
            retval, rvec, tvec	=	cv2.aruco.estimatePoseCharucoBoard(CHcors, CHids, board, cameraMatrix, distCoeffs)

            if retval and CHcors is not None and CHids is not None and len(CHcors)>3:
                rvecs.append(rvec)
                tvecs.append(tvec)

            cv2.aruco.drawDetectedMarkers(frame, ARcors, ARids)

        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame', 1200, 800)
        cv2.imshow('frame', frame)
        cv2.waitKey(10)

    cv2.destroyAllWindows()
    imsize = gray.shape
    # ext_mat: Extrinsic matrix from Pattern to Camera
    As = as_homogeneous_mat(rvecs, tvecs)
    
    with open(EXTMAT, 'w') as f:
        yaml.dump(As.tolist(), f, default_flow_style=False)
        print('Save the Extrinsic matrix to yaml data file.')

    with open(INTMAT_CP, 'w') as f:
        yaml.dump(cameraMatrix.tolist(), f, default_flow_style=False)
        print('Save the Intrinsic matrix to yaml data file.')

if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main(DEBUG=DEBUG)