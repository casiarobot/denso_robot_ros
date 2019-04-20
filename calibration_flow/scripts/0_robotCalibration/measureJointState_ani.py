import numpy as np
import yaml
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rospy
import hardward_controller
import time
from sensor_msgs.msg import JointState


class RobotMeasurer():
    def __init__(self):
        self.js_topic = '/vs6242/joint_states'

    def saveJointData(self):
        # PATH SETTING
        CONFIG = 'config.yaml'
        with open(CONFIG) as f:
            path = yaml.load(f)

        J1_PATH = path['robotCalibration'] + 'goal/j1.yaml'
        ret = np.array(self.J1p)

        with open(J1_PATH, 'w') as f:
            yaml.dump(ret.tolist(), f, default_flow_style=False)
            print('save data')

    def measureJointAngle(self):
        self.time = []
        self.J1p  = []
        self.J2p  = []
        self.J3p  = []
        self.J4p  = []
        self.J5p  = []
        self.J6p  = []
        js_sub = rospy.Subscriber(self.js_topic, JointState, self.callback)


        fig = plt.figure()
        self.ax = fig.add_subplot(1,1,1)
        update_rate = 10 # in millisec
        ani = animation.FuncAnimation(fig, self.plot_update, interval= update_rate)
        plt.show()
        
        rospy.spin()

        print('hi')
        self.saveJointData()


    def callback(self, data):
        stamp = data.header.stamp
        self.time.append(stamp.secs + stamp.nsecs*1e-9)
        joint_angles = data.position
        self.J1p.append(np.degrees(joint_angles[0]))
        self.J2p.append(np.degrees(joint_angles[1]))
        self.J3p.append(np.degrees(joint_angles[2]))
        self.J4p.append(np.degrees(joint_angles[3]))
        self.J5p.append(np.degrees(joint_angles[4]))
        self.J6p.append(np.degrees(joint_angles[5]))

    def plot_update(self, i):
        self.ax.clear()
        line, = self.ax.plot(self.time, self.J1p)
        self.ax.set_ylim(-180,180)

def main():


    rospy.init_node('talker', anonymous=True)
    Measurer = RobotMeasurer()
    try:
        Measurer.measureJointAngle()

        # goal = (np.radians(30), 0, np.radians(90), 0, np.radians(90), 0)
        # Robot.go_to_joint_state(goal)
        # time.sleep(1)
        # goals = []
        # for j1 in np.linspace(-30, 30, 2):
        #     goal = (np.radians(j1), 0, np.radians(90), 0, np.radians(90), 0)
        #     Robot.go_to_joint_state(goal)
        #     time.sleep(1)
        

        print("============ Calibration process complete!")
    
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        DEBUG = sys.argv[1]
    else:
        DEBUG = True
    main()