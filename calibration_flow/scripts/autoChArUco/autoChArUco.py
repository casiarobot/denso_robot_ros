import time
import cv2
import numpy as np
import glob

def as_quaternion(rvecs, tvecs):
    pass

def as_homogeneous_mat(rvecs, tvecs):
    pose_amount = len(rvecs)
    # Extrinsic parameter
    extr_mat = np.zeros((pose_amount, 4, 4))
    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        extr_mat[i, 0:3, 0:3] = cv2.Rodrigues(rvec)[0]
        extr_mat[i, 0:3 ,3] = tvec.flatten()
        extr_mat[i, 3, 3] = 1.0
    return extr_mat

def main(SOURCE_IMAGE_PATH, output_pattern_img=False):

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

if __name__ == "__main__":
    SOURCE_IMAGE_PATH = 'img/ap*.bmp'
    main(SOURCE_IMAGE_PATH)