#!/usr/bin/env python

import numpy as np
from math import pi
import rospy
import yaml
import time
import hardward_controller
import subprocess

current_goal = np.zeros((1,7))

def AutoCenter(Robot, Camera, path, DEBUG):
    ###############################
    ### Auto Center
    ###############################
    BASE = path['ACenter'] if DEBUG else path['ROOT']
    Init_GOAL = BASE + 'goal/init_goal.yaml'
    ACenter_GOAL = BASE + 'goal/ac_goal.yaml'

    # Initial shot
    goal = (0.3, -0.03, 0.5, -0.0871557427476582, -0.996194698091746, 0.0, 0.0)
    with open(Init_GOAL, 'w') as f:
        yaml.dump(list(goal), f, default_flow_style=False)

    Robot.go_to_pose_goal(goal)
    time.sleep(1)
    image = Camera.trigger(BASE + 'img/init.bmp')

    # Compute companation
    cmd = 'python ' + path['ACenter'] + 'autoCenter.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)
    with open(ACenter_GOAL) as f:
        goal = yaml.load(f)
        print('Get pose goal from yaml file.')

    # Move to center position
    Robot.go_to_pose_goal(goal)
    time.sleep(1)
    image = Camera.trigger(BASE + 'img/center.bmp')
    print("============ End Auto center process ============")

def AutoFocus(Robot, Camera, path, DEBUG):
    ###############################
    ### Step 2: Auto Focus
    ###############################
    BASE = path['AFocus'] if DEBUG else path['ROOT']
    AFocus_GOAL = BASE + 'goal/af_goal.yaml'
    BFocus_GOAL = BASE + 'goal/bf_goal.yaml'

    with open(AFocus_GOAL) as f:
        af_goals = yaml.load(f)

    for ind, goal in enumerate(af_goals):
        Robot.go_to_pose_goal(goal)
        time.sleep(1)
        img_name = BASE + 'img/af' + str(ind+1).zfill(2) + '.bmp'
        image = Camera.trigger(img_name)

    cmd = 'python ' + path['AFocus'] + 'autoFocus.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)
    with open(BFocus_GOAL) as f:
        goal = yaml.load(f)

    # Move to best focus position
    Robot.go_to_pose_goal(goal)
    time.sleep(1)
    image = Camera.trigger(BASE + 'img/bestFocused.bmp')
    print("============ End Auto focus process ============")

def AutoPose(Robot, Camera, path, DEBUG):
    ###############################
    ### Step 3: Auto Pose
    # ###############################
    BASE = path['APose'] if DEBUG else path['ROOT']
    AF_BASE = path['AFocus'] if DEBUG else path['ROOT']
    POSE_GOAL = BASE + 'goal/ap_goal.yaml'
    BFocus_GOAL = AF_BASE + 'goal/bf_goal.yaml'
    
    cmd = 'python ' + path['APose'] + 'autoPose.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)
    with open(POSE_GOAL) as f:
        pose_goals = yaml.load(f)
        print('Get pose goal from yaml file.')

    for ind, goal in enumerate(pose_goals):
        Robot.go_to_pose_goal(goal)
        print("============ save as ap{}.bmp".format(ind))
        time.sleep(1)
        img_name = BASE + 'img/ap' + str(ind+1).zfill(2) + '.bmp'
        image = Camera.trigger(img_name)

def CameraCalibration(Robot, Camera, path, DEBUG):
    ###############################
    ### Step 4: Camera Calibration
    ###############################
    BASE = path['CCalibration'] if DEBUG else path['ROOT']
    CC_GOAL = BASE + 'goal/cc_goal.yaml'
    
    cmd = 'python ' + path['CCalibration'] + 'cameraCalibration.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)

    with open(CC_GOAL) as f:
        pose_goals = yaml.load(f)
        print('Get pose goal from yaml file.')
    for ind, goal in enumerate(pose_goals):
        Robot.go_to_pose_goal(goal)
        print("============ Press `Enter` to execute camera trigger save as ap{}.bmp".format(ind))
        time.sleep(1)
        img_name = BASE + 'img/cc' + str(ind+1).zfill(2) + '.bmp'
        image = Camera.trigger(img_name)
    

def main(DEBUG=True):
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    APose_GOAL = path['ROOT'] + 'goal/ap_goal.yaml'
    CCalibration_GOAL = path['ROOT'] + 'goal/cc_goal.yaml'

    Robot = hardward_controller.MoveGroupInteface()
    Camera = hardward_controller.camera_shooter()
    try:
        # AutoCenter(Robot, Camera, path, DEBUG)
        # AutoFocus(Robot, Camera, path, DEBUG) 
        AutoPose(Robot, Camera, path, DEBUG)
        # CameraCalibration(Robot, Camera, path, DEBUG)


        print("============ Calibration process complete!")
    
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
  

