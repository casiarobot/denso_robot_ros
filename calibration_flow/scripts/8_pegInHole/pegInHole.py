
import numpy as np
import numpy.matlib
import quaternion
import cv2
import glob
import yaml
import frame3D
import matplotlib.pyplot as plt

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

    BASE = path['pegInHole'] if DEBUG else path['ROOT']
    XZ_BASE = path['solveXZ'] if DEBUG else path['ROOT']
    HS_BASE = path['holeSearching'] if DEBUG else path['ROOT']
    TCP_BASE = path['TCPCalibration'] if DEBUG else path['ROOT']
    WHole_PATH = HS_BASE + 'goal/Whole.yaml'
    Y_PATH = TCP_BASE + 'goal/y.yaml'
    PIH_goal_PATH = BASE + 'goal/pih_goal.yaml'
    Wh_PATH = HS_BASE + 'goal/Whole.yaml'
    Y_PATH = TCP_BASE + 'goal/Y.yaml'
    X_PATH = XZ_BASE + 'goal/HX.yaml'
    with open(Wh_PATH) as f:
        Wh = np.matrix(yaml.load(f))
    with open(Y_PATH) as f:
        Y = np.matrix(yaml.load(f))
    with open(X_PATH) as f:
        X = np.matrix(yaml.load(f))
    Wh[1, 3] = Wh[1, 3] + 0.001
    Wh[0, 3] = Wh[0, 3] - 0.05

    WDs = [0.04, 0.02, 0.0, -0.01, -0.02]
    wayposes = np.zeros((len(WDs),7))
    for i, WD in enumerate(WDs):
        C = np.matrix([[0.0, 1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0, WD], [0.0, 0.0, 0.0, 1.0]])
        B = Wh*np.linalg.inv(C)*np.linalg.inv(Y)
        pih_goal = as_ROSgoal(B)
        wayposes[i] = pih_goal

    with open(PIH_goal_PATH, 'w') as f:
        yaml.dump(wayposes.tolist(), f, default_flow_style=False)

    # Data visualization
    World = frame3D.Frame(np.matlib.identity(4))
    Base = frame3D.Frame(World.pose)
    Workpiece = frame3D.Frame(Base.pose)
    Flange = frame3D.Frame(Base.pose)
    Tool = frame3D.Frame(Base.pose)
    Camera = frame3D.Frame(Base.pose)
    orientation = frame3D.Orientation()
    ax = frame3D.make_3D_fig(axSize=0.45)
  
    Flange.transform_by_rotation_mat(B, refFrame=Base.pose)
    Tool.transform_by_rotation_mat(Y, refFrame=Flange.pose)
    Camera.transform_by_rotation_mat(X, refFrame=Flange.pose)
    Workpiece.transform_by_rotation_mat(Wh, refFrame=Base.pose)

    Base.plot_frame(ax, 'Base')
    Flange.plot_frame(ax, 'Flange')
    Tool.plot_frame(ax, 'Tool')
    Camera.plot_frame(ax, 'Camera')
    Workpiece.plot_frame(ax, 'Workpiece ')

    # plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main(DEBUG=DEBUG)