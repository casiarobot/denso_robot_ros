#!/usr/bin/env python

import numpy as np
from math import pi
import rospy
import yaml
import time
import hardward_controller
import subprocess

def AutoCenter(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Auto Center
    ###############################
    BASE = path['ACenter'] if DEBUG else path['ROOT']
    Init_GOAL = BASE + 'goal/init_goal.yaml'
    ACenter_GOAL = BASE + 'goal/ac_goal.yaml'

    # Initial shot
    if SIM:
        # goal = (0.3, -0.03, 0.4, -0.0871557427476582, -0.996194698091746, 0.0, 0.0) # GAZEBO
        # goal = (0.26, 0.0, 0.399978956713, 0.00106130271845, 0.999999396927, 0.000280212483897, 3.55297433221e-05)
        goal = (0.26, 0.00, 0.25, -9.94506391949e-06, -0.999999997498, 5.22583013222e-06, 6.98376583036e-05)
    else:
        goal = (0.209998087153, 1.11412077042e-06, 0.368260009103, -9.94506391949e-06, -0.999999997498, 5.22583013222e-06, 6.98376583036e-05)

    with open(Init_GOAL, 'w') as f:
        yaml.dump(list(goal), f, default_flow_style=False)

    Robot.go_to_pose_goal(goal)
    time.sleep(1)
    image = Camera.trigger(BASE + 'img/init.bmp')

    # Compute companation
    cmd = 'python ' + path['ACenter'] + 'autoCenter.py ' + str(SIM) + ' ' + str(DEBUG) 
    subprocess.call(cmd, shell=True)
    with open(ACenter_GOAL) as f:
        goal = yaml.load(f)
        print('Get pose goal from yaml file.')

    # Move to center position
    Robot.go_to_pose_goal(goal)
    time.sleep(1)
    image = Camera.trigger(BASE + 'img/center.bmp')
    print("============ End Auto center process ============")

def AutoFocus(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Step 2: Auto Focus
    ###############################
    BASE = path['AFocus'] if DEBUG else path['ROOT']
    AFocus_GOAL = BASE + 'goal/af_goal.yaml'
    BFocus_GOAL = BASE + 'goal/bf_goal.yaml'

    if not SIM:
        with open(AFocus_GOAL) as f:
            af_goals = yaml.load(f)

        for ind, goal in enumerate(af_goals):
            Robot.go_to_pose_goal(goal)
            time.sleep(1)
            img_name = BASE + 'img/af' + str(ind+1).zfill(2) + '.bmp'
            image = Camera.trigger(img_name)

    cmd = 'python ' + path['AFocus'] + 'autoFocus.py ' + str(SIM) + ' ' + str(DEBUG) 
    subprocess.call(cmd, shell=True)
    with open(BFocus_GOAL) as f:
        goal = yaml.load(f)

    # Move to best focus position
    Robot.go_to_pose_goal(goal)
    time.sleep(1)
    image = Camera.trigger(BASE + 'img/bestFocused.bmp')
    print("============ End Auto focus process ============")

def AutoPose(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Step 3: Auto Pose
    # ###############################
    BASE = path['APose'] if DEBUG else path['ROOT']
    AF_BASE = path['AFocus'] if DEBUG else path['ROOT']
    POSE_GOAL = BASE + 'goal/ap_goal.yaml'
    BFocus_GOAL = AF_BASE + 'goal/bf_goal.yaml'
    
    cmd = 'python ' + path['APose'] + 'autoPose.py ' + str(SIM) + ' ' + str(DEBUG) 
    subprocess.call(cmd, shell=True)
    with open(POSE_GOAL) as f:
        pose_goals = yaml.load(f)
        print('Get pose goal from yaml file.')

    for ind, goal in enumerate(pose_goals):
        Robot.go_to_pose_goal(goal)
        print("============ save as ap{}.bmp".format(ind+1))
        time.sleep(1)
        img_name = BASE + 'img/ap' + str(ind+1).zfill(2) + '.bmp'
        image = Camera.trigger(img_name)
    print("============ End Auto Pose process ============")
    
def CameraPoseEstimation(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Step 4: Camera Pose Estimation
    ###############################
    # if SIM:
    #     cmd = 'python ' + path['PoseEstimation'] + 'cameraCalibration.py ' + str(DEBUG)
    # else:
    #     cmd = 'python ' + path['PoseEstimation'] + 'solvePnP.py ' + str(DEBUG)
    cmd = 'python ' + path['PoseEstimation'] + 'cameraCalibration.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)
    print("============ End Camera Pose Estimation process ============")

def SolveXZ(Robot, Camera, path, DEBUG):
    ###############################``
    ### Step 5: Solve XZ
    ###############################    
    cmd = 'python ' + path['solveXZ'] + 'solveXZ.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)
    print("============ End SolveXZ process ============")

def HoleSearching(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Step 6: Hole searching
    ###############################
    BASE = path['holeSearching'] if DEBUG else path['ROOT']
    AF_BASE = path['AFocus'] if DEBUG else path['ROOT']
    Init_Hole_GOAL = BASE + 'goal/init_hole.yaml'
    HS_GOAL = BASE + 'goal/hs_goal.yaml'
    BFocus_GOAL = AF_BASE + 'goal/bf_goal.yaml'

    with open(BFocus_GOAL) as f:
        bf_goal = yaml.load(f)
    if SIM:
        goal = (0.0989688762978, -0.290417812266, 0.409766763058, -0.977195181237, -0.0266658008638, 0.210581453508, 0.00582788734334)
    else:
        z = bf_goal[2] - 0.003 +0.03
        goal = (0.11, 0.29, z, 0.0, -1.0, 0.0, 0.0)
        # goal = (0.13, 0.314, z, 0.0, -1.0, 0.0, 0.0)
        # goal = (0.214522443195, 0.257440181378, 0.440925534866, 0.701196313411, -0.712488543824, 0.023465310035, 0.0115405460771)
    with open(Init_Hole_GOAL, 'w') as f:
        yaml.dump(list(goal), f, default_flow_style=False)

    Robot.go_to_pose_goal(goal)
    print("============ save as hs.bmp")
    time.sleep(1)
    img_name = BASE + 'img/hs.bmp'
    image = Camera.trigger(img_name)

    cmd = 'python ' + path['holeSearching'] + 'holeSearching.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)
    with open(HS_GOAL) as f:
        goal = yaml.load(f)
        print('Get pose goal from yaml file.')

    # Move to center position
    # Robot.go_to_pose_goal(goal)
    # time.sleep(1)
    # image = Camera.trigger(BASE + 'img/center.bmp')


    print("============ End HoleSearching process ============") 

def PegInHole(Robot, Camera, path, SIM, DEBUG):
    ###############################
    ### Step 6: Hole searching
    ###############################
    cmd = 'python ' + path['pegInHole'] + 'pegInHole.py ' + str(DEBUG)
    subprocess.call(cmd, shell=True)

    BASE = path['pegInHole'] if DEBUG else path['ROOT']
    PIH_GOAL = BASE + 'goal/pih_goal.yaml'
    with open(PIH_GOAL) as f:
        goals = yaml.load(f)

    Robot.plan_cartesian_path(goals, 0.01)
   
    print("============ End Peg-in-Hole process ============")

def main(SIM, DEBUG=True):
    # PATH SETTING
    CONFIG = 'config.yaml'
    with open(CONFIG) as f:
        path = yaml.load(f)

    APose_GOAL = path['ROOT'] + 'goal/ap_goal.yaml'
    CCalibration_GOAL = path['ROOT'] + 'goal/cc_goal.yaml'

    Robot = hardward_controller.MoveGroupInteface()
    Camera = hardward_controller.camera_shooter()
    try:
        # AutoCenter(Robot, Camera, path, SIM, DEBUG)
        # AutoFocus(Robot, Camera, path, SIM, DEBUG)
        # AutoPose(Robot, Camera, path, SIM, DEBUG)
        # CameraPoseEstimation(Robot, Camera, path, SIM, DEBUG)
        # SolveXZ(Robot, Camera, path, DEBUG)
        # HoleSearching(Robot, Camera, path, SIM, DEBUG)
        PegInHole(Robot, Camera, path, SIM, DEBUG)
        print("============ Calibration process complete!")
    
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    SIM = False
    # SIM = True
    main(SIM)
  

