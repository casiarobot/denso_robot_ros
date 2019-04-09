# import sys
# sys.path.append('/usr/local/python/3.5')


import numpy as np
import numpy.matlib
import quaternion
import cv2
import glob
import yaml
import frame3D
import matplotlib.pyplot as plt

def as_homogeneous_mat(rvec, tvec):
    # Extrinsic parameter from Camera to Object
    mat = np.matlib.identity(4)
    mat[0:3, 0:3] = cv2.Rodrigues(rvec)[0]
    mat[0:3 ,3] = tvec.reshape(3,1)
    return mat

def __cal_axis_angle_from_q__(q):
    theta = 2*np.arccos(q.w)
    v = np.array([q.x, q.y, q.z])/np.sin(theta/2)

    return theta, v

def calPositionAndOrientationError(H, H_exact):
    H1 = np.array(H)
    H2 = np.array(H_exact)
    pError = np.linalg.norm(H1[0:2, 3] - H2[0:2, 3])
    q1 = quaternion.from_rotation_matrix(H1[0:3, 0:3])
    q2 = quaternion.from_rotation_matrix(H2[0:3, 0:3])
    q = q1.inverse()*q2
    theta, v = __cal_axis_angle_from_q__(q)
    oError = np.degrees(theta) 
    if oError > 180.0:
        oError = 360.0 - oError 
    return pError, oError

def as_ROSgoal(Homo_mat):
    '''
    ROS goal: goal = (x, y, z, qx, qy, qz, qw)
    '''
    q = quaternion.from_rotation_matrix(Homo_mat[0:3, 0:3])
    return np.array([Homo_mat[0, 3], Homo_mat[1, 3], Homo_mat[2, 3], q.x, q.y, q.z, q.w])

def main(DEBUG):
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    BASE = path['holeSearching'] if DEBUG else path['ROOT']
    PE_BASE = path['PoseEstimation'] if DEBUG else path['ROOT']
    XZ_BASE = path['solveXZ'] if DEBUG else path['ROOT']

    Init_Hole_GOAL = BASE + 'goal/init_hole.yaml'
    IMAGE_PATH = BASE + 'img/hs.bmp'
    CAMERA_MAT = PE_BASE + 'goal/camera_mat.yaml'
    HX_PATH = XZ_BASE + 'goal/HX.yaml'
    D_MAT = BASE + 'goal/D.yaml'
    HW_MAT = BASE + 'goal/HW.yaml'
    EW_MAT = BASE + 'goal/EW.yaml'
    HS_GOAL_PATH = BASE + 'goal/hs_goal.yaml'
    WHole_PATH = BASE + 'goal/Whole.yaml'
    Bc_PATH = BASE + 'goal/Bc.yaml'

    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250 )
    squareLength = 0.01   # Here, our measurement unit is m.
    markerLength = 0.25/40.0   # Here, our measurement unit is m.
    arucoParams = cv2.aruco.DetectorParameters_create()
    with open(CAMERA_MAT) as f:
        cameraMatrix = np.array(yaml.load(f))
        distCoeffs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]) # Assume to zero

    images = sorted(glob.glob(IMAGE_PATH))
    for ind, fname in enumerate(images):
        frame = cv2.imread(fname)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams)

        if len(corners)>0:
            diamondCorners, diamondIds = cv2.aruco.detectCharucoDiamond(gray, corners, ids, squareLength/markerLength)

            if len(diamondCorners) >= 1: 
                img_with_diamond = cv2.aruco.drawDetectedDiamonds(frame, diamondCorners, diamondIds, (0,255,0))
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(diamondCorners, squareLength, cameraMatrix, distCoeffs)
                img_with_diamond = cv2.aruco.drawAxis(img_with_diamond, cameraMatrix, distCoeffs, rvec, tvec, 1)    # axis length 100 can be changed according to your requirement
        else:
            img_with_diamond = gray

        cv2.namedWindow('diamond', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('diamond', 1200, 800)
        cv2.imshow("diamond", img_with_diamond)   # display
        cv2.waitKey()
    cv2.destroyAllWindows()

    D = as_homogeneous_mat(rvec, tvec) # from Camera to Object
    with open(D_MAT, 'w') as f:
        yaml.dump(D.tolist(), f, default_flow_style=False)
        print('Save the D matrix to yaml data file.')

    #################
    # Testing 
    #################
    with open(HX_PATH) as f:
        X = np.matrix(yaml.load(f))

    with open(Init_Hole_GOAL) as f:
        ih = np.array(yaml.load(f)) # ROS goal: (x, y, z, qx, qy,qz, qw)
        q = np.quaternion(ih[6], ih[3], ih[4], ih[5])
        
        B0 = np.matlib.identity(4)
        B0[0:3, 0:3] = quaternion.as_rotation_matrix(q)
        B0[0:3, 3] = ih[0:3].reshape(3, 1)

    W = B0*X*D
    W[0:3, 0:3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    with open(HW_MAT, 'w') as f:
        yaml.dump(W.tolist(), f, default_flow_style=False)
        print('Save the solved W matrix to yaml data file.')
    W = B0*X*D
    World = frame3D.Frame(np.matlib.identity(4))
    Base = frame3D.Frame(World.pose)
    Flange = frame3D.Frame(Base.pose)
    Camera = frame3D.Frame(Base.pose)
    Workpiece = frame3D.Frame(Base.pose)
    orientation = frame3D.Orientation()
    ax = frame3D.make_3D_fig(axSize=0.45 )
    
    # {Base as World}
    Base.plot_frame(ax, 'Base')
    Flange.transform_by_rotation_mat(B0, refFrame=Base.pose)
    Camera.transform_by_rotation_mat(X, refFrame=Flange.pose)
    Workpiece.transform_by_rotation_mat(D, refFrame=Camera.pose)
    Base.plot_frame(ax, 'Base')
    Camera.plot_frame(ax, 'Camera') 
    Flange.plot_frame(ax, 'Flange') 
    Workpiece.plot_frame(ax, 'Workpiece')
    
    cmd_W = orientation.asRad((-0.06, -0.3, 0.011, 0, 0, 90)).cmd()
    W2 = Workpiece.transform(cmd_W, refFrame=World.pose)
    # with open(EW_MAT, 'w') as f:
    #     yaml.dump(W2.tolist(), f, default_flow_style=False)
    #     print('Save the exact W matrix to yaml data file.')

    pError, oError = calPositionAndOrientationError(W, W2)
    # print('Postion error: {} mm, Orientation error: {} degree'.format(pError*1000, oError))

    plt.show()
    
    # Validation
    # print(W)
    Whole = W
    Whole[1, 3] = Whole[1, 3] + 0.015 - 0.0750
    Whole[0, 3] = Whole[0, 3] - 0.015 + 0.1000
    WD = 0.32692938211698336
    cmd_Dc = orientation.asRad((0.0, 0.0, WD, 0.0, 180, 0.0)).cmd()
    Dc = Workpiece.transform(cmd_Dc, refFrame=World.pose)
    Bc = Whole*np.linalg.inv(Dc)*np.linalg.inv(X)
    
    goal = as_ROSgoal(Bc)
    with open(HS_GOAL_PATH, 'w') as f:
        yaml.dump(goal.tolist(), f, default_flow_style=False)
        print('Save the ROS goal to yaml data file.')

    with open(Bc_PATH, 'w') as f:
        yaml.dump(Bc.tolist(), f, default_flow_style=False)
        print('Save the ROS goal to yaml data file.')

    with open(WHole_PATH, 'w') as f:
        yaml.dump(Whole.tolist(), f, default_flow_style=False)
        print('Save the ROS goal to yaml data file.')

    Flange.transform_by_rotation_mat(Bc, refFrame=Base.pose)
    Camera.transform_by_rotation_mat(X, refFrame=Flange.pose)
    Workpiece.transform_by_rotation_mat(W, refFrame=Base.pose)

    Base.plot_frame(ax, 'Base')
    Camera.plot_frame(ax, 'Camera') 
    Flange.plot_frame(ax, 'Flange') 
    Workpiece.plot_frame(ax, 'Workpiece')
    plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main(DEBUG=DEBUG)