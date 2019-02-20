#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
import quaternion
from math import pi
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import yaml
import time

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupInteface(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupInteface, self).__init__()

    #################### 
    ## First initialize node:
    #################### 
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_python_interface',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    ####################
    ## Get current info
    ######################
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print group.get_current_pose().pose
    print ""

    # Misc variables
    self.robot = robot
    self.scene = scene
    self.group = group
    self.group_names = group_names

  def __set_pose_goal__(self, goal):
    ######################
    ## Cast goal to ros msg pose format
    ######################
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = goal[0]
    pose_goal.position.y = goal[1]
    pose_goal.position.z = goal[2]
    pose_goal.orientation.x = goal[3]
    pose_goal.orientation.y = goal[4]
    pose_goal.orientation.z = goal[5]
    pose_goal.orientation.w = goal[6]
    
    return pose_goal

  def go_to_joint_state(self, goal):

    ######################
    ## Define joint goal 
    ######################
    goal = np.array(goal)
    joint_goal = self.group.get_current_joint_values()
    joint_goal[0] = goal[0]
    joint_goal[1] = goal[1]
    joint_goal[2] = goal[2]
    joint_goal[3] = goal[3]
    joint_goal[4] = goal[4]
    joint_goal[5] = goal[5]

    ######################
    ## Execute joint goal
    ## Calling ``stop()`` ensures that there is no residual movement 
    ######################
    self.group.go(joint_goal, wait=True)
    self.group.stop()

    ######################
    ## For test:
    ######################
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self, goal):
    '''
    @param: goal       A list of floats, (x, y, z, qx, qy, qz, qw)
    @returns: bool
    '''
    ######################
    ## Planning to a Pose Goal
    ######################
    pose_goal = self.__set_pose_goal__(goal)
    a = self.group.set_pose_target(pose_goal)
    rospy.loginfo('Pose goal: {}'.format(a))

    ######################
    ## Execute Pose Goal
    ## Calling `stop()` ensures that there is no residual movement
    ######################
    plan = self.group.go(wait=True)
    self.group.stop()
    self.group.clear_pose_targets()

    ######################
    ## For test:
    ######################
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

class camera_shooter:
    def __init__(self):
        # rospy.init_node('camera_python_interface', anonymous=True)
        self.bridge = CvBridge()
        self.image_topic = "/camera/image_raw"

    def trigger(self, imgName=None):
      '''
      The image will save to $ROS_HOME directory, or 
      you can modify it by node "cwd" attribute
      '''
      data = rospy.wait_for_message(self.image_topic, Image)
      try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
          print(e)
      else:
          if imgName is not None:
            print("Save an image!")
            print("filename: {}".format(imgName))
            cv2.imwrite(imgName, cv_image)
          return cv_image

    def image_stream(self):
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.callback)

    def callback(self, data):
        print("Received an image!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            print("Save an image!")
            cv2.imshow('img', cv_image)
            cv2.waitKey(300)
  
if __name__ == '__main__':
    robot = MoveGroupInteface()
    robot.go_to_joint_state([1,2,3,4,5,6])
    # camera = camera_shooter() 
    # camera.trigger('img/hi.png')