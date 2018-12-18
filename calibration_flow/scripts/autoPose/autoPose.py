import numpy as np
from math import cos, sin
import matplotlib.pyplot as plt
import frame3D
import scipy.optimize
import yaml
import quaternion

########################
# Calculate camera pose
########################
def get_position_from_spherical(refFrame, rho, theta, phi):
    '''
    Get photo position(x,y,z) in spherical cooridnate and convert it to world frame coordinate
    - a_0: x-value with respact to frame 0
    - a_w: x-value with respact to world frame
    '''
    origin = refFrame*np.matrix([[0], [0], [0], [1]])
    a_0 = rho*sin(phi)*cos(theta)
    b_0 = rho*sin(phi)*sin(theta)
    c_0 = rho*cos(phi)
    p_photo = refFrame*np.matrix([[a_0], [b_0], [c_0], [1]])
    (a_w, b_w, c_w) = convert_to_vector(p_photo)
    normal_to_origin_vector = convert_to_vector(origin - p_photo)
    return (a_w, b_w, c_w, normal_to_origin_vector)

def objFun_normal_vector(X, position_1, refFrame):
    (Rx, Ry, Rz) = X
    (a, b, c, n1) = position_1
    orientation_1 = (a, b, c, Rx, Ry, Rz)
    tarFrame = frame3D.H(orientation_1)
    # Constraint 1
    n1_hat = get_normal_vector(tarFrame)
    angle = get_angle_between_vectors(n1, n1_hat)
    # Constraint 2
    n0_z = get_normal_vector(refFrame, axis='z')
    n0_y = get_normal_vector(refFrame, axis='y')
    n1_y = get_normal_vector(tarFrame, axis='y')
    n1_y_proj = get_project_vector_on_plane(n1_y, n0_z)
    angle2 = get_angle_between_vectors(n0_y, n1_y_proj)
    
    return np.array([angle, angle2, 0])

def get_angle_between_vectors(n0, n1):
    numerator = np.dot(n0, n1)
    denominator = np.linalg.norm(n0) * np.linalg.norm(n1)
    
    if np.isclose(numerator, denominator):
        # print('numerator:{},denominator:{}'.format(numerator, denominator))
        angle = 0
    else:
        angle = np.arccos(numerator/denominator)
    return angle

def convert_to_vector(matrix_vector):
    '''
    Convert a matrix_vector, which is a 4*1 homogeneous matrix, to a 3*1 array vector
    '''
    return np.array(matrix_vector).flatten()[0:3]

def get_normal_vector(refFrame, axis='z'):
    origin = refFrame*np.matrix([[0], [0], [0], [1]])
    if axis == 'z':
        normal = refFrame*np.matrix([[0], [0], [1], [1]])
    elif axis == 'x':
        normal = refFrame*np.matrix([[1], [0], [0], [1]])
    elif axis == 'y':
        normal = refFrame*np.matrix([[0], [1], [0], [1]])
    return convert_to_vector(normal - origin)

def get_project_vector_on_plane(vector, normal_vector):
    proj_on_normal_vector = np.dot(vector, normal_vector) / np.linalg.norm(normal_vector)**2 * normal_vector
    proj_on_plane = vector - proj_on_normal_vector
    return proj_on_plane

def get_photo_orientation_by_given_shoot_angle(guess_orientation, refFrame, focal_length, shoot_angle, theta):
    position_1 = get_position_from_spherical(refFrame, rho = focal_length, theta = theta, phi = shoot_angle)

    res = scipy.optimize.leastsq(objFun_normal_vector, guess_orientation, 
                        args=(position_1, refFrame), full_output=1)
    print(res[2]['fvec'])
    # print(res[0])
    (Rx, Ry, Rz) = res[0] 
    (a, b, c, _) = position_1
    orientation_1 = [a, b, c, Rx, Ry, Rz]
    return orientation_1

def main_calculateCameraPose(WD, pose_amount, polar_ang, itvl_azimuthal, CAMERA_POSE_PATH):
    # ax = frame3D.make_3D_fig(axSize=200)
    World = frame3D.Frame(np.matlib.identity(4))
    Camera = frame3D.Frame(World.pose)
    orientation = frame3D.Orientation()

    thetas = np.linspace(0, 360, pose_amount) # azimuthal angle
    X0 = (np.radians(45), np.radians(45), np.radians(45)) # initial guess
    camOrientations = np.zeros((pose_amount,6)) 

    # World.plot_frame(ax, '')
    for ind, theta in enumerate(thetas) :
        print(theta)
        orientation_2 = get_photo_orientation_by_given_shoot_angle(X0,\
                     World.pose, WD, np.radians(polar_ang), np.radians(theta))
        X0 = orientation_2[3:6]
        cmd = orientation.asItself(orientation_2).cmd()
        Camera.transform(cmd, refFrame=World.pose)
        # Camera.plot_frame(ax, '')
        camOrientations[ind, :] = Camera.get_orientation_from_pose(Camera.pose, refFrame=World.pose)
    return camOrientations

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

if __name__ =='__main__':
    import os.path
    WD = 0.400 # work distence(m)
    pose_amount = 12 # Amount of camera pose
    phi = 25 # polarangle 
    theta_interval = 30 # interval of azimuthal angle
    CAMERA_POSE_PATH = 'autoPose/data/camera_pose.yaml'
    ROS_GOAL_PATH = 'goal/ap_goal.yaml'

    camOrientations = main_calculateCameraPose(WD, pose_amount, phi, 
                                    theta_interval, CAMERA_POSE_PATH)                    
    with open(CAMERA_POSE_PATH, 'w') as f:
        yaml.dump(camOrientations.tolist(), f, default_flow_style=False)
        print('Save the camera pose data to yaml data file.')
    
    # Verify data 
    World = frame3D.Frame(np.matlib.identity(4))
    Pattern = frame3D.Frame(World.pose)
    Camera = frame3D.Frame(Pattern.pose)
    Flange = frame3D.Frame(Pattern.pose)
    Base = frame3D.Frame(Pattern.pose)
    Flange_test = frame3D.Frame(Pattern.pose)
    orientation = frame3D.Orientation()
    ax = frame3D.make_3D_fig(axSize=0.45 )
    
    #################
    # Initial 
    #################
    # {Pattern as World}
    Pattern.plot_frame(ax, 'Pattern')

    # {Camera}
    cmd_A0 = orientation.asRad((0, 0, WD, 0, 180, 0)).cmd()
    A0 = Camera.transform(cmd_A0, refFrame=World.pose)
    Camera.transform(cmd_A0, refFrame=Pattern.pose)
    Camera.plot_frame(ax, 'init_camera')
    
    # {Flange}
    cmd_X = orientation.asRad((0, 0, -0.050, 0, 0, 90)).cmd()
    X = Flange.transform(cmd_X, refFrame=World.pose)
    Flange.transform(cmd_X, refFrame=Camera.pose)
    # Flange.plot_frame(ax, 'init_flange')

    # {Base}
    cmd_Z = orientation.asRad((0.3, 0, 0, 0, 0, 90)).cmd()
    Z = Base.transform(cmd_Z, refFrame=World.pose)
    cmd_Z = orientation.asRad((0, 0.3, 0, 0, 0, -90)).cmd()
    Base.transform(cmd_Z, refFrame=Pattern.pose)
    Base.plot_frame(ax, 'Base')

    # {Test Flange calculate from Z*A*X}
    B = Z*A0*X
    Flange_test.transform_by_rotation_mat(B, refFrame=Base.pose)
    Flange_test.plot_frame(ax, 'Test')
    #################
    # Calculate camera pose 
    #################
    print('done')
    As = np.zeros((pose_amount,4,4))
    Bs = np.zeros((pose_amount,4,4))
    for i, camOrientation in enumerate(camOrientations):
        cmd_A = orientation.asItself(camOrientation).cmd()
        As[i] = Camera.transform(cmd_A, refFrame=World.pose)
        Camera.transform(cmd_A, refFrame=Pattern.pose)
        Camera.plot_frame(ax, '')    
        Flange.transform(cmd_X, refFrame=Camera.pose)
        # Flange.plot_frame(ax, '')

        # {Test Flange calculate from Z*A*X}
        Bs[i] = Z*As[i]*X
        # Flange_test.transform_by_rotation_mat(Bs[i], refFrame=Base.pose)
        # Flange_test.plot_frame(ax, 'Test')
        print('X:{}'.format(X))
        print('Z:{}'.format(Z))
        print('A0:{}'.format(A0))
        print('A:{}'.format(As[0]))
    goals = as_ROSgoal(Bs)
    with open(ROS_GOAL_PATH, 'w') as f:
        yaml.dump(goals.tolist(), f, default_flow_style=False)
        print('Save the ROS goal to yaml data file.')
    plt.show()

