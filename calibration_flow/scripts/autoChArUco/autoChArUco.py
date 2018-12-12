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
    # Extrinsic parameter
    extr_mat = np.zeros((pose_amount, 4, 4))
    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        extr_mat[i, 0:3, 0:3] = cv2.Rodrigues(rvec)[0]
        extr_mat[i, 0:3 ,3] = tvec.flatten()
        extr_mat[i, 3, 3] = 1.0
    return extr_mat

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

def main(SOURCE_IMAGE_PATH, ROS_GOAL_PATH, output_pattern_img=False):

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    board = cv2.aruco.CharucoBoard_create(11, 9, 0.020, 0.010, dictionary)

    #Dump the calibration board to a file
    if output_pattern_img:
        img = board.draw((200*11,200*9))
        cv2.imwrite('charuco.png',img)

    allCHCors = []
    allCHIds = []

    images = sorted(glob.glob(SOURCE_IMAGE_PATH))
    for ind, fname in enumerate(images):
        frame = cv2.imread(fname)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ARcors, ARids, _ = cv2.aruco.detectMarkers(gray, dictionary)

        if len(ARcors)>0:
            ret, CHcors, CHids = cv2.aruco.interpolateCornersCharuco(ARcors, ARids, gray, board)
            if CHcors is not None and CHids is not None and len(CHcors)>3:
                allCHCors.append(CHcors)
                allCHIds.append(CHids)

            cv2.aruco.drawDetectedMarkers(frame, ARcors, ARids)

        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame', 1200, 800)
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

    cv2.destroyAllWindows()
    imsize = gray.shape
    retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(allCHCors, allCHIds, board, imsize, None, None)
    print(retval)
    ext_mat = as_homogeneous_mat(rvecs, tvecs)
    ext_q = as_quaternion(rvecs, tvecs)

    #################
    # Testing 
    #################
    WD = 0.400 # work distence(m)

    World = frame3D.Frame(np.matlib.identity(4))
    Base = frame3D.Frame(World.pose)
    Pattern = frame3D.Frame(Base.pose)
    Image = frame3D.Frame(Base.pose)
    Camera = frame3D.Frame(Base.pose)
    Flange = frame3D.Frame(Base.pose)
    orientation = frame3D.Orientation()
    ax = frame3D.make_3D_fig(axSize=0.45 )
    
    # {Base as World}
    Base.plot_frame(ax, 'Base')

    # {Pattern}
    cmd_Z1 = orientation.asRad((0.3, 0, 0, 0, 0, 90)).cmd()
    Pattern.transform(cmd_Z1, refFrame=Base.pose)
    Pattern.plot_frame(ax, 'Pattern')
    cmd_Z2 = orientation.asRad((0.3+0.11, -0.09, 0, 0, 0, 90)).cmd()
    Z = Image.transform(cmd_Z2, refFrame=Base.pose)
    Image.plot_frame(ax, 'Image')

    # {Camera}
    cmd_A0 = orientation.asRad((0, 0, WD, 0, 180, 0)).cmd()
    A0 = Camera.transform(cmd_A0, refFrame=World.pose)
    Camera.transform(cmd_A0, refFrame=Pattern.pose)
    Camera.plot_frame(ax, 'init_camera')

    # {Flange}
    cmd_X = orientation.asRad((0, 0, -0.050, 0, 0, 90)).cmd()
    X = Flange.transform(cmd_X, refFrame=World.pose)
    Flange.transform(cmd_X, refFrame=Camera.pose)
    Flange.plot_frame(ax, 'init_flange')

    # Verify data 
    As = ext_mat
    Bs = np.zeros_like(As)
    for i, A in enumerate(As):
        A = np.linalg.inv(A)
        Bs[i] = Z*A*X
        # {Flange}
        Flange.transform_by_rotation_mat(Bs[i], refFrame=Base.pose)
        Flange.plot_frame(ax, '')
        # {Camera}
        Camera.transform_by_rotation_mat(A, refFrame=Image.pose)
        Camera.plot_frame(ax, '') 

    goals = as_ROSgoal(Bs)
    with open(ROS_GOAL_PATH, 'w') as f:
        yaml.dump(goals.tolist(), f, default_flow_style=False)
        print('Save the ROS goal to yaml data file.')

    plt.show()
if __name__ == "__main__":
    SOURCE_IMAGE_PATH = 'img/ap*.bmp'
    ROS_GOAL_PATH = 'goal/cc_goal.yaml'
    main(SOURCE_IMAGE_PATH, ROS_GOAL_PATH)