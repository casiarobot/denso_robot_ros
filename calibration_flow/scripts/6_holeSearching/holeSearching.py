# import sys
# sys.path.append('/usr/local/python/3.5')


import numpy as np
import quaternion
import cv2
import glob
import yaml
import frame3D
import matplotlib.pyplot as plt
# calibrationFile = "calibrationFileName.xml"
# calibrationParams = cv2.FileStorage(calibrationFile, cv2.FILE_STORAGE_READ)
# camera_matrix = calibrationParams.getNode("cameraMatrix").mat()
# dist_coeffs = calibrationParams.getNode("distCoeffs").mat()

# r = calibrationParams.getNode("R").mat()
# new_camera_matrix = calibrationParams.getNode("newCameraMatrix").mat()

# image_size = (1920, 1080)
# map1, map2 = cv2.fisheye.initUndistortRectifyMap(camera_matrix, dist_coeffs, r, new_camera_matrix, image_size, cv2.CV_16SC2)

# PATH SETTING

def as_homogeneous_mat(rvec, tvec):
    # Extrinsic parameter from Object to Camera
    D_inv = np.identity(4)
    D_inv[0:3, 0:3] = cv2.Rodrigues(rvec)[0]
    D_inv[0:3 ,3] = tvec.flatten()

    # Extrinsic parameter from Camera to Object
    D = np.linalg.inv(D_inv)
    return np.matrix(D)

def main(DEBUG):
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    BASE = path['holeSearching'] if DEBUG else path['ROOT']
    CC_BASE = path['CCalibration'] if DEBUG else path['ROOT']
    XZ_BASE = path['solveXZ'] if DEBUG else path['ROOT']

    Init_Hole_GOAL = BASE + 'goal/init_hole.yaml'
    IMAGE_PATH = BASE + 'img/hs.png'
    CAMERA_MAT = CC_BASE + 'goal/camera_mat.yaml'
    HX_PATH = XZ_BASE + 'goal/HX.yaml'
    D_MAT = BASE + 'goal/D.yaml'

    aruco_dict = cv2.aruco.getPredefinedDictionary( cv2.aruco.DICT_6X6_250 )
    squareLength = 0.01   # Here, our measurement unit is m.
    markerLength = 0.25/40.0   # Here, our measurement unit is m.
    arucoParams = cv2.aruco.DetectorParameters_create()
    with open(CAMERA_MAT) as f:
        cameraMatrix = np.array (yaml.load(f))
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
                img_with_diamond = cv2.aruco.drawAxis(img_with_diamond, cameraMatrix, distCoeffs, rvec, tvec, 100)    # axis length 100 can be changed according to your requirement
        else:
            img_with_diamond = gray

        cv2.namedWindow('diamond', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('diamond', 1200, 800)
        cv2.imshow("diamond", img_with_diamond)   # display
        cv2.waitKey(1)
    cv2.destroyAllWindows()

    D = as_homogeneous_mat(rvec, tvec)
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

    World = frame3D.Frame(np.matlib.identity(4))
    Base = frame3D.Frame(World.pose)
    Flange = frame3D.Frame(Base.pose)
    Camera = frame3D.Frame(Base.pose)
    Workpiece = frame3D.Frame(Base.pose)
    Hole = frame3D.Frame(Base.pose)
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
    plt.show()


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main(DEBUG=DEBUG)