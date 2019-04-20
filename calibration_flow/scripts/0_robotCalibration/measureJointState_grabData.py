import numpy as np
import yaml
import matplotlib.pyplot as plt
import rospy
import hardward_controller
import time
from sensor_msgs.msg import JointState

t = []
J1p  = []
J2p  = []
J3p  = []
J4p  = []
J5p  = []
J6p  = []

def callback(data):
    global t, J1p, J2p, J3p, J4p, J5p, J6p
    stamp = data.header.stamp
    t.append(stamp.secs + stamp.nsecs*1e-9)
    joint_angles = data.position
    J1p.append(np.degrees(joint_angles[0]))
    J2p.append(np.degrees(joint_angles[1]))
    J3p.append(np.degrees(joint_angles[2]))
    J4p.append(np.degrees(joint_angles[3]))
    J5p.append(np.degrees(joint_angles[4]))
    J6p.append(np.degrees(joint_angles[5]))

def saveJointData():
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    J1_PATH = path['robotCalibration'] + 'goal/j1.yaml'
    ret = np.array(J1p)

    with open(J1_PATH, 'w') as f:
        yaml.dump(ret.tolist(), f, default_flow_style=False)
        print('save data')

def plotJoint():
    plt.figure()
    plt.plot(t, J1p)
    plt.show()

if __name__ == "__main__":

    Robot = hardward_controller.MoveGroupInteface()
    js_sub = rospy.Subscriber('/vs6242/joint_states', JointState, callback)
    J1 = [-30, 0, 30, 0, -30]
    for j1 in J1:
        goal = (np.radians(j1), 0, np.radians(90), 0, np.radians(90), 0)
        Robot.go_to_joint_state(goal)
        time.sleep(1)
    saveJointData()
    
    rospy.spin()
    print("============ Calibration process complete!")
    
